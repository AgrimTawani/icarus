"""Mission conversation: measured context -> native tool -> result -> decision."""

import copy
import json
import threading
import time

from python.dcm.active_detection_feedback import ActiveDetectionFeedback
from python.dcm.agent_context import capture_context, context_message, system_prompt
from python.dcm.agent_tools import ToolExecutor, decision_tools, message_dict, validate_call
from python.dcm.operator_control import (
    OperatorInterrupt,
)


def record(client, event, value):
    episode = getattr(client, "episode", None)
    if episode is not None:
        episode.record(event, value)


def interrupted_call(function, interrupt, cancel=None):
    """Let operator input preempt inference without waiting for model latency."""
    result = {}
    ready = threading.Event()

    def work():
        try:
            result["value"] = function()
        except BaseException as error:  # noqa: BLE001 - relay worker failure to the controlling thread
            result["error"] = error
        finally:
            ready.set()

    worker = threading.Thread(target=work, daemon=True)
    worker.start()
    while not ready.wait(0.05):
        if interrupt.sequence():
            if cancel:
                # Model teardown must never delay the physical operator command.
                threading.Thread(target=cancel, daemon=True).start()
            raise InterruptedError("Operator stopped the mission")
    if interrupt.sequence():
        raise InterruptedError("Operator stopped the mission")
    if "error" in result:
        raise result["error"]
    return result["value"]


class AgentSession:
    """Retain original instructions, tool outcomes and clarification answers."""

    def __init__(self, mission, previous_mission=None):
        self.mission = mission
        self.messages = [
            {"role": "system", "content": system_prompt()},
            {"role": "user", "content": mission},
        ]
        self.actions = []
        self.rejected_validation_attempts = []
        self.rejected_validation_attempts_total = 0
        self.awaiting_user = False
        self.finished = False
        self.decisions = 0
        self.start_state = None
        self.previous_mission = previous_mission
        self.angular_scans = {}
        self.area_scans = {}
        self.observation_feedback_counts = {}
        self.detector_feedback = ActiveDetectionFeedback()
        self.tool_failure_counts = {}
        self.unresolved_tool_failures = {}
        self.mission_progress = ""

    def remember_tool_result(self, name, arguments, result):
        """Receipt consistency only; never infer obligations from mission text."""
        if name == 'execute_sequence':
            for item in result.get('actions', []):
                self.remember_tool_result(item['name'], item.get('arguments', {}), item['result'])
            return
        if result.get('status') == 'SUCCEEDED':
            self.unresolved_tool_failures.pop(name, None)
        elif result.get('status') not in (None, 'CANCELLED'):
            self.unresolved_tool_failures[name] = {
                'action': name, 'arguments': copy.deepcopy(arguments),
                'status': result.get('status'), 'message': result.get('message', '')}

    def record_validation_failure(self, name, arguments, error):
        """Keep bounded factual failure memory, separate from executed actions."""
        attempt = {'decision': self.decisions, 'action': name[:128] if isinstance(name, str) else None,
                   'status': 'INVALID_ARGUMENT', 'message': str(error)[:512],
                   'no_action_executed': True}
        try:
            encoded = json.dumps(arguments, ensure_ascii=False, allow_nan=False)
            if len(encoded) <= 1024:
                attempt['arguments'] = copy.deepcopy(arguments)
            else:
                attempt['arguments_excerpt'] = encoded[:1024]
                attempt['arguments_truncated'] = True
        except (ValueError, TypeError):
            # Malformed/nonfinite model output must not corrupt future JSON feedback.
            attempt['arguments_excerpt'] = repr(arguments)[:1024]
            attempt['arguments_not_json'] = True
        self.rejected_validation_attempts_total += 1
        self.rejected_validation_attempts.append(attempt)
        del self.rejected_validation_attempts[:-8]

    def answer(self, text):
        if not self.awaiting_user:
            raise ValueError("There is no pending clarification")
        self.messages.append({"role": "user", "content": text})
        self.awaiting_user = False

    def result(self, status, message):
        return {
            "architecture": "direct-agent-v1",
            "mission": self.mission,
            "status": status,
            "message": message,
            "decisions": self.decisions,
            "executed": list(self.actions),
            "unresolved_tool_failures": copy.deepcopy(list(self.unresolved_tool_failures.values())),
        }


def fly_mission(
    client,
    runtime,
    mission,
    mode="autonomous",
    max_decisions=64,
    show_prompts=False,
    stream=None,
    interrupt=None,
    session=None,
    echo=print,
    context_provider=capture_context,
):
    if mode not in ("approval", "autonomous"):
        raise ValueError("Unknown execution mode")
    session = session or AgentSession(mission)
    client._angular_scans = session.angular_scans
    client._scan_workflows = session.area_scans
    interrupt = interrupt or OperatorInterrupt(echo=echo)
    interrupt.enable_safety(client)
    interrupt.arm(client)
    executor = ToolExecutor(client, interrupt, session=session)
    # Clarification resumes the same mission. It must not reset unchanged
    # backend failures and permit endless retries after each user answer.
    failures = 0
    for previous in reversed(session.actions):
        if previous.get('source') == 'perception_feedback':
            continue
        # An explained capability incompatibility pauses for a new operator
        # choice; it is not an automatic failed retry. Keep prior real backend
        # failures, but do not exhaust the budget through legitimate answers.
        if previous['result'].get('requires_user_choice'):
            continue
        if previous['action'] in ('get_context', 'read_action_result') and previous['result']['status'] == 'SUCCEEDED':
            continue
        if previous['result']['status'] == 'SUCCEEDED':
            break
        failures += 1
    try:
        if failures >= 3:
            return session.result('incomplete', 'Repeated actions failed. ' +
                session.actions[-1]['result'].get('message', 'No further action executed.'))
        while session.decisions < max_decisions:
            if interrupt.sequence():
                raise InterruptedError("Operator stopped the mission")
            # Refresh only an already requested detector after physical action.
            # This is sampled perception, not another model decision or a mission
            # interpretation. Capture state afterward so sensor latency cannot
            # make the supplied vehicle state older than the new camera result.
            if session.detector_feedback.needs_refresh:
                refresh = interrupted_call(
                    lambda: session.detector_feedback.refresh(executor), interrupt)
                if refresh is not None:
                    snapshot = session.detector_feedback.snapshot()
                    observed = snapshot['observation']
                    feedback_result = {
                        'status': 'SUCCEEDED' if observed.get('fresh') and observed.get('validity') == 'valid' else 'FAILED',
                        'evidence': observed,
                        'message': observed.get('reason', 'Requested camera refreshed after motion.')}
                    feedback_entry = {
                        'action': 'detect', 'source': 'perception_feedback',
                        'arguments': copy.deepcopy(session.detector_feedback.interest),
                        'result': feedback_result}
                    # A real sensor execution belongs in the same chronological
                    # evidence ledger as explicit tool results. A parallel count
                    # field left progress claiming no post-motion observation.
                    session.actions.append(feedback_entry)
                    record(client, 'agent_perception_refresh', {
                        **feedback_entry, 'raw_result': refresh})
                    from python.dcm.observation_feedback import observation_feedback
                    for line in observation_feedback('detect', feedback_result,
                            session.detector_feedback.interest):
                        echo('  Camera feedback: ' + line)
            context = context_provider(client)
            camera_feedback = session.detector_feedback.snapshot()
            if camera_feedback is not None:
                metadata = {k: v for k, v in camera_feedback['observation'].items()
                            if k in ('validity', 'fresh', 'age_ms', 'reason', 'motion_revision')}
                context['current_camera_detection'] = {
                    **{k: v for k, v in camera_feedback.items() if k != 'observation'},
                    'observation': metadata,
                    'evidence_reference': ('Latest indexed detection observation, or camera_detection inside '
                                           'the latest scene-inspection receipt, in chronological execution history.')}
            session.angular_scans = getattr(client, '_angular_scans', {})
            session.area_scans = getattr(client, '_scan_workflows', {})
            from python.dcm.action_memory import indexed_results
            context['retained_action_result_count'] = sum(1 for _ in indexed_results(session.actions))
            tools = decision_tools(context, session.angular_scans, session.area_scans)
            messages = decision_messages(session, context)
            if show_prompts:
                echo(json.dumps({"messages": messages, "tools": tools}, indent=2))
            started = time.monotonic()
            from python.dcm.llama_runtime import IncompleteModelResponse
            from python.dcm.runtime_config import ContextBudgetExceeded

            def model_decision(concise=False):
                nonlocal messages
                for compact in (False, True):
                    try:
                        return interrupted_call(
                            (lambda: runtime.decide(messages, tools, concise=True)) if concise else
                            (lambda: runtime.decide(messages, tools)),
                            interrupt, getattr(runtime, 'stop', None))
                    except ContextBudgetExceeded as error:
                        if compact:
                            raise
                        # Token counting rejected only the presentation, before
                        # inference or flight. Keep the request and page receipt
                        # details; the model can retrieve anything it needs.
                        messages = decision_messages(session, context, lean=True)
                        record(client, 'agent_context_packing', {
                            'reason': str(error), 'no_action_executed': True,
                            'mode': 'retained_receipt_lookup'})
                        if show_prompts:
                            echo(json.dumps({'messages': messages, 'tools': tools}, indent=2))

            try:
                response = model_decision()
            except IncompleteModelResponse as error:
                record(
                    client,
                    "agent_response_retry",
                    {"reason": str(error), "no_action_executed": True},
                )
                session.decisions += 1
                if session.decisions >= max_decisions:
                    raise
                response = model_decision(concise=True)
            session.decisions += 1
            record(
                client,
                "agent_decision",
                {
                    "messages": messages,
                    "tools": tools,
                    "response": response,
                    "latency_ms": round((time.monotonic() - started) * 1000),
                    "stats": getattr(runtime, "last_stats", None),
                },
            )
            calls = response.get("tool_calls") or []
            if not calls:
                # Text is never interpreted as a flight instruction or an assertion of success.
                text = (
                    response.get("content")
                    or "I could not choose the next action. Please clarify the remaining task."
                )
                session.messages.append({"role": "assistant", "content": text})
                session.awaiting_user = True
                return session.result("awaiting_user", text)
            if len(calls) != 1:
                session.messages.append(
                    {
                        "role": "user",
                        "content": "No action executed. Return exactly one tool call; use execute_sequence for a short unconditional group.",
                    }
                )
                failures += 1
                if failures >= 3:
                    return session.result(
                        "incomplete",
                        "The model repeatedly returned conflicting tool calls. No further action was taken.",
                    )
                continue
            call = calls[0]
            name = call.get("function", {}).get("name")
            args = None
            try:
                args = call["function"].get("arguments", {})
                if isinstance(args, str):
                    args = json.loads(args)
                validate_call(name, args, tools=tools)
                # Agent-authored memory, never a controller argument or proof.
                args = dict(args)
                progress = args.pop('mission_progress', None)
                if progress is not None:
                    session.mission_progress = progress
            except (ValueError, KeyError, TypeError) as error:
                session.record_validation_failure(name, args, error)
                session.messages.append({"role": "assistant", "tool_calls": calls})
                session.messages.append(
                    {
                        "role": "tool",
                        "tool_call_id": call.get("id", ""),
                        "content": json.dumps(
                            {"status": "INVALID_ARGUMENT", "message": str(error)}
                        ),
                    }
                )
                failures += 1
                if failures >= 3:
                    return session.result(
                        "incomplete",
                        "The model could not form a valid tool call: " + str(error),
                    )
                continue
            session.messages.append({"role": "assistant", "tool_calls": calls})
            call_id = call.get("id", "")
            if name == "ask_user":
                session.messages.append(
                    {
                        "role": "tool",
                        "tool_call_id": call_id,
                        "content": "Question delivered; the next user message is the answer.",
                    }
                )
                session.awaiting_user = True
                return session.result("awaiting_user", args["question"])
            if name == "finish":
                if args['outcome'] == 'completed' and session.unresolved_tool_failures:
                    unresolved = list(session.unresolved_tool_failures.values())
                    feedback = {'status': 'UNRESOLVED_TOOL_FAILURES',
                        'message': 'These operations have failed and have no later successful receipt. '
                                   'Continue the unfinished operation, or report incomplete with its limitation.',
                        'failed_operations': unresolved}
                    session.messages.append({'role': 'tool', 'tool_call_id': call_id,
                                             'content': json.dumps(feedback)})
                    record(client, 'agent_completion_consistency', feedback)
                    continue
                session.messages.append(
                    {
                        "role": "tool",
                        "tool_call_id": call_id,
                        "content": "Result delivered.",
                    }
                )
                session.finished = True
                result = session.result(args["outcome"], args["message"])
                result["assessment_source"] = "mission_model"
                return result
            if mode == "approval" and name in (
                *executor_flight_names(),
                "execute_sequence",
            ):
                echo(f"Execute {name} {json.dumps(args)}? [y/N]")
                interrupt.disarm()
                if stream is not None and hasattr(stream, "discard_pending"):
                    stream.discard_pending()
                answer = stream.readline() if stream is not None else input()
                interrupt.arm(client)
                if interrupt.sequence():
                    raise InterruptedError("Operator stopped the mission")
                if answer.strip().lower() not in ("y", "yes"):
                    session.messages.append(
                        {
                            "role": "tool",
                            "tool_call_id": call_id,
                            "content": "Operator declined; not executed.",
                        }
                    )
                    return session.result(
                        "cancelled", "You declined the action. Mission stopped."
                    )
            echo(f"→ {name} {json.dumps(args, separators=(',', ':'))}")
            # Camera work can be slow; operator recovery must still start immediately.
            if name in (
                "detect",
                "inspect_scene",
                "assess_landing_zone",
                "localize_target",
            ):
                result = interrupted_call(
                    lambda name=name, args=args: executor.execute(name, args), interrupt
                )
            else:
                result = executor.execute(name, args)
            session.actions.append(
                {"action": name, "arguments": args, "result": result}
            )
            session.remember_tool_result(name, args, result)
            session.detector_feedback.observe(name, args, result)
            if (result.get('status') == 'SUCCEEDED' and
                    (name == 'localize_target' or
                     name in ('goto', 'orbit', 'overfly_target') and args.get('destination', {}).get('type')
                     in ('visible_object', 'registered_target'))):
                vocabulary = getattr(client, 'camera_detector_classes', lambda: [])()
                if vocabulary:
                    session.detector_feedback.request_interest(
                        {'camera': 'forward_d435i', 'classes': vocabulary})
            from python.dcm.observation_feedback import observation_feedback
            evidence = result.get('evidence', {})
            evidence = evidence if isinstance(evidence, dict) else {}
            scan_id = evidence.get('scan_id')
            previous_count = session.observation_feedback_counts.get(scan_id, 0) if isinstance(scan_id, str) else 0
            for line in observation_feedback(name, result, args,
                                             previous_observation_count=previous_count):
                echo('  ' + line)
            if isinstance(scan_id, str) and isinstance(evidence.get('observations'), list):
                session.observation_feedback_counts[scan_id] = len(evidence['observations'])
            if result.get('mission_complete') and name != 'cancel':
                # A pre-execution completion hint establishes neither fulfilled
                # instructions nor camera proof. The same agent sees actual
                # results before continuing remaining work or reporting finish.
                result.pop('mission_complete')
                result['action_batch_complete'] = True
                result['next_step'] = ('Listed actions completed. Compare remaining user obligations with execution history. '
                                       'Continue missing work or use finish to report the measured outcome and limitations; '
                                       'do not repeat completed actions.')
            record(client, "agent_tool_result", session.actions[-1])
            # Keep successful motion history compact; measured state is refreshed every turn.
            history_result = compact_result(result)
            session.messages.append(
                {
                    "role": "tool",
                    "tool_call_id": call_id,
                    "content": json.dumps(history_result, separators=(",", ":")),
                }
            )
            if interrupt.sequence():
                raise InterruptedError("Operator stopped the mission")
            if result.get('requires_user_choice'):
                # A confirmed hardware incompatibility needs an operator
                # choice; do not let a repair turn silently change the request.
                session.awaiting_user = True
                return session.result('awaiting_user', result['question'])
            if result["status"] == "SUCCEEDED":
                # Reading telemetry is not recovery from a rejected operation.
                # Preserve the retry budget across repeated refresh/retry loops.
                if name not in ('get_context', 'read_action_result'):
                    failures = 0
            else:
                failures += 1
            signature = json.dumps([name, {key: value for key, value in args.items()
                                          if key != 'mission_complete'}], sort_keys=True)
            if result['status'] == 'SUCCEEDED':
                session.tool_failure_counts.pop(signature, None)
            else:
                session.tool_failure_counts[signature] = session.tool_failure_counts.get(signature, 0) + 1
                if session.tool_failure_counts[signature] >= 3:
                    return session.result('incomplete',
                        f'Stopped after three failed attempts at the same {name} operation. '
                        + result.get('message', 'No successful execution was measured.'))
            if result.get("mission_complete"):
                session.finished = True
                if name == "cancel":
                    return session.result(
                        "cancelled", "Mission cancelled. " + result.get("message", "")
                    )
                return session.result(
                    "completed",
                    "Mission completed. All actions in the final call succeeded.",
                )
            if failures >= 3:
                return session.result(
                    "incomplete",
                    "Stopped after three unsuccessful calls. "
                    + result.get("message", "Check the latest tool result."),
                )
        return session.result(
            "incomplete",
            f"Reached {max_decisions} decisions; remaining work was not executed.",
        )
    except InterruptedError:
        while interrupt.latest():
            latest = interrupt.latest()
            executor.expected_interrupt = interrupt.sequence()
            result = executor.execute(latest["action"], {})
            session.actions.append(
                {
                    "action": latest["action"],
                    "arguments": {},
                    "result": result,
                    "operator": True,
                }
            )
            if executor.expected_interrupt == interrupt.sequence():
                return session.result(
                    "cancelled",
                    f"Mission stopped by you. {latest['action']}: {result['status']}. {result.get('message', '')}",
                )
        return session.result("cancelled", "Mission interrupted.")
    except Exception as error:  # noqa: BLE001 - report boundary failures without losing control cleanup
        record(client, "agent_error", {"message": str(error)})
        recovery = hold_on_error(client, echo)
        return session.result("incomplete", f"Mission stopped: {error}. {recovery}")
    finally:
        interrupt.disarm()
        interrupt.disable_safety()


def compact_result(value):
    """Retain outcomes, arguments and evidence; omit repeated telemetry and model metadata."""
    if isinstance(value, dict):
        if 'scan_id' in value and isinstance(value.get('observations'), list):
            result = {k: compact_result(v) for k, v in value.items() if k != 'observations'}
            result['observations'] = []
            for observation in value['observations']:
                entry = {k: compact_result(v) for k, v in observation.items() if k != 'evidence'}
                evidence = compact_result(observation.get('evidence', {}))
                # Station position/heading already identifies this view.
                # Raw pose and capture timing remain in the episode.
                evidence.pop('capture_pose', None)
                entry['evidence'] = evidence
                result['observations'].append(entry)
            from python.dcm.context_compaction import compact_scan_evidence
            return compact_scan_evidence(result)
        if value.get("schema") == "icarus.perception.target_estimate.v1":
            # Navigation stays server-owned. The model needs identity,
            # observability and measured goals, not duplicate sensor arrays.
            selected = {k: value[k] for k in (
                "target_id", "description", "found", "reason", "origin_id",
                "surface_range_m", "surface_range_kind", "bearing_deg",
                "measurement_view_candidates") if k in value}
            selected["geometry"] = {k: v for k, v in value.get("geometry", {}).items()
                if k in ("center", "orbit_center", "orbit_radius_m", "observed_extent",
                         "orbit_usable", "requires_more_views", "whole_shape_observed",
                         "height_m", "height_status", "height_evidence_age_ms", "suggestion",
                         "measurement_view_candidates")}
            selected["quality"] = {k: v for k, v in value.get("quality", {}).items()
                if k in ("age_ms", "viewpoints", "viewpoints_agree", "position_spread_m")}
            selected["tracking"] = {k: v for k, v in value.get("tracking", {}).items()
                if k in ("status", "reason", "geometry_age_ms")}
            if value.get("depth_at_capture"):
                selected["depth_at_capture"] = {k: v for k, v in value["depth_at_capture"].items()
                    if k in ("fresh", "age_ms", "available", "valid_fraction",
                             "median_optical_z_m", "minimum_radial_range_m", "object_associated")}
            return {k: compact_result(v) for k, v in selected.items()}
        if "counts" in value and "captured_at_unix_ms" in value:
            selected = {k: value[k] for k in ("counts", "camera", "captured_at_unix_ms",
                "capture_pose", "validity", "error", "fresh", "confidence_threshold",
                "age_ms", "source_tool", "count_scope", "motion_revision") if k in value}
            return {k: compact_result(v) for k, v in selected.items()}
        return {
            k: compact_result(v)
            for k, v in value.items()
            if k not in ("state", "runtime", "image", "provenance", "hull", "observed_hull",
                         "assumed_outline", "capture_pose_quality", "candidates", "capture_started_at_unix_ms",
                         "latency_ms")
        }
    if isinstance(value, list):
        return [compact_result(v) for v in value]
    if isinstance(value, float):
        return round(value, 3)
    return value


def conversation_tail(messages, exchanges=4):
    """Keep instructions/clarifications and latest native exchange once.

    Older actions and evidence are retained in the explicit execution ledger
    and observation history. Raw episodes retain full precision and payloads.
    """
    if len(messages) <= 4:
        return copy.deepcopy(messages)
    # Keep several actual exchanges: a single latest successful action makes
    # small models repeat it despite a contradictory aggregate ledger.
    keep = {0, 1, *range(max(2, len(messages)-2*exchanges), len(messages))}
    while min(keep - {0, 1}, default=2) > 2:
        start = min(keep - {0, 1})
        if messages[start].get('role') != 'tool':
            break
        keep.add(start-1)
    if messages[-1].get("role") == "tool":
        keep.add(len(messages)-2)
    for index, message in enumerate(messages[2:], 2):
        if message.get("role") == "user":
            keep.add(index)
        if any(call.get("function", {}).get("name") == "ask_user"
               for call in message.get("tool_calls", [])):
            keep.update((index, index+1))
    return [copy.deepcopy(message) for index, message in enumerate(messages) if index in keep]


def action_verification_scope(name, arguments=None):
    """Describe receipt semantics without interpreting the operator's mission."""
    if name in ('goto', 'orbit') and (arguments or {}).get('destination', {}).get('type') in ('visible_object', 'registered_target'):
        return 'Measured object workflow; only relationships explicitly measured in its evidence.'
    if name == 'goto' and (arguments or {}).get('destination', {}).get('type') == 'named_waypoint':
        return 'Arrival at a saved vehicle navigation/observation pose only; not above an object or at its center.'
    if name in {'goto', 'move_relative', 'takeoff', 'yaw', 'face_heading'}:
        return 'Requested vehicle motion only; no proof of object-relative goal, named waypoint or visual condition.'
    if name in {'inspect_scene', 'detect'}:
        return 'Observation at recorded camera pose/time only; inspect evidence and uncertainty, not motion success.'
    if name in {'localize_target', 'approach_target', 'orbit_target', 'overfly_target'}:
        return 'Measured target workflow; only relationships explicitly measured in its evidence.'
    return 'Only the recorded tool operation and its measured result.'


def decision_messages(session, context, lean=False):
    """Attach feedback to the tool result, never impersonate a new user request."""
    context = copy.deepcopy(context)
    context['agent_working_memory'] = {
        'note': session.mission_progress,
        'authority': 'model_written_note_not_operator_instruction_or_verified_evidence'}
    context['position_evidence'] = {
        'altitude_reference': 'above_home_not_above_object',
        'registered_target_available': any(isinstance(t, dict) and t.get('target_id')
                                         for t in context.get('targets', [])),
        'scope': 'Vehicle coordinates alone do not establish position relative to an object or a named waypoint.'}
    if session.rejected_validation_attempts:
        context['rejected_validation_attempts'] = copy.deepcopy(session.rejected_validation_attempts)
        context['rejected_validation_attempts_total'] = session.rejected_validation_attempts_total
        context['rejected_validation_attempts_omitted'] = (
            session.rejected_validation_attempts_total - len(session.rejected_validation_attempts))
    if session.previous_mission:
        context['previous_mission_reference'] = session.previous_mission
    # Current target geometry is supplied once, with millimetre precision.
    # Historical measurements below retain their factual range/time but refer
    # to this registry view instead of replaying every old geometry snapshot.
    if isinstance(context.get("targets"), list):
        context["targets"] = compact_result(context["targets"])
    target_ids = {target.get("target_id") for target in context.get("targets", [])
                  if isinstance(target, dict)} if isinstance(context.get("targets"), list) else set()
    if session.start_state is None and context.get("state", {}).get("local_position_ned"):
        session.start_state = {key: copy.deepcopy(context["state"][key]) for key in (
            "local_position_ned", "position", "attitude", "heading_deg", "relative_altitude_m", "home", "landed", "observed_at_unix_ms")
            if key in context["state"]}
    if session.start_state is not None:
        context["mission_start_state"] = copy.deepcopy(session.start_state)
    executed = []
    for entry in session.actions:
        if entry["action"] == "execute_sequence":
            items = entry["result"].get("actions", [])
        else:
            items = [
                {
                    "name": entry["action"],
                    "arguments": entry["arguments"],
                    "result": entry["result"],
                }
            ]
        for item in items:
            executed.append(
                {
                    "index": len(executed) + 1,
                    "action": item["name"],
                    "source": entry.get("source", "model_tool_call"),
                    "verification_scope": action_verification_scope(item['name'], item.get('arguments', {})),
                    "arguments": {
                        k: v
                        for k, v in item["arguments"].items()
                        if k != "mission_complete"
                    },
                    "status": item["result"]["status"],
                    "message": item["result"].get("message", ""),
                    **({'reason_code': item['result']['reason_code']}
                       if 'reason_code' in item['result'] else {}),
                    **({'motion': compact_result(item['result']['motion'])}
                       if 'motion' in item['result'] else {}),
                }
            )
    from python.dcm.context_compaction import compact_scan_receipts
    context["executed_actions"] = compact_scan_receipts(executed)
    # Supply factual counters, not an interpreted mission plan. Small models
    # should not have to count nested tool messages to know what actually ran.
    successful_counts = {}
    observations = []
    scan_observation_records = {}
    action_index = 0
    for entry in session.actions:
        if entry["action"] == "execute_sequence":
            items = entry["result"].get("actions", [])
        else:
            items = [{"name": entry["action"], "result": entry["result"]}]
        for item in items:
            action_index += 1
            if item["result"]["status"] == "SUCCEEDED":
                name = item["name"]
                successful_counts[name] = successful_counts.get(name, 0) + 1
            evidence = item["result"].get("evidence")
            if evidence is not None:
                from python.dcm.context_compaction import reference_registered_target_geometry
                evidence = reference_registered_target_geometry(evidence, target_ids)
                # A resumed scan returns its complete cumulative station
                # record. Keep the latest record once, rather than replaying
                # every prefix and growing model context quadratically.
                scan_id = (evidence.get("scan_id") if isinstance(evidence, dict)
                           and isinstance(evidence.get("observations"), list) else None)
                # A resumed workflow's cumulative record replaces its old
                # prefix at the NEW chronological position, not the old slot.
                if scan_id in scan_observation_records:
                    observations.remove(scan_observation_records[scan_id])
                observation = {
                    "after_action_index": action_index,
                    "tool": item["name"],
                    "status": item["result"]["status"],
                    "evidence": compact_result(evidence),
                }
                observations.append(observation)
                if scan_id:
                    scan_observation_records[scan_id] = observation
    for index, observation in enumerate(observations, 1):
        observation['observation_index'] = index
    context["successful_action_counts"] = successful_counts
    context['unresolved_tool_failures'] = copy.deepcopy(list(session.unresolved_tool_failures.values()))
    context["observations"] = observations
    last_observation = max((e['index'] for e in executed
        if e['action'] in ('detect', 'inspect_scene', 'localize_target', 'scan_heading',
                           'resume_heading_scan', 'scan_area', 'resume_scan')
        and e['status'] == 'SUCCEEDED'), default=0)
    context = {'execution_progress': {
        'last_completed_action': executed[-1] if executed else None,
        'actions_since_last_observation': [e['action'] for e in executed if e['index'] > last_observation],
        'observation_after_latest_action': bool(executed and last_observation == executed[-1]['index']),
    }, **context}
    # Explicitly mark historical view changes; a timestamp alone makes a
    # previous negative result too easy to reuse after a turn or translation.
    current_pose = context.get('state', {}).get('local_position_ned', {})
    current_yaw = context.get('state', {}).get('attitude', {}).get('yaw_rad')
    import math
    motion_names = set(executor_flight_names()) - {'arm', 'disarm'}
    last_motion_index = max((e['index'] for e in executed if e['action'] in motion_names), default=0)
    for observation in observations:
        evidence = observation.get('evidence', {})
        capture = evidence.get('capture_pose', {})
        pose = capture.get('local_position_ned', {})
        yaw = capture.get('attitude', {}).get('yaw_rad')
        if pose and current_pose and yaw is not None and current_yaw is not None:
            origin_changed = bool(pose.get('origin_id') and current_pose.get('origin_id')
                                  and pose['origin_id'] != current_pose['origin_id'])
            observation['view_change'] = {
                'translation_m': None if origin_changed else round(math.dist(
                    [current_pose.get(k, 0) for k in ('north_m', 'east_m', 'down_m')],
                    [pose.get(k, 0) for k in ('north_m', 'east_m', 'down_m')]), 3),
                'yaw_deg': round(abs(math.degrees(math.remainder(current_yaw-yaw, 2*math.pi))), 2),
                'origin_changed': origin_changed,
                'followed_by_motion_command': observation['after_action_index'] < last_motion_index,
            }
        if capture:
            # Keep the observed viewpoint once in decision-relevant units.
            # Full attitude and capture brackets remain in the recorded event.
            evidence['camera_pose_at_capture'] = {
                'coordinate_role': 'camera_origin_not_object_position_or_named_waypoint',
                **{k: pose[k] for k in ('north_m', 'east_m', 'origin_id') if k in pose}}
            if 'relative_altitude_m' in capture:
                evidence['camera_pose_at_capture']['altitude_above_home_m'] = capture['relative_altitude_m']
            if yaw is not None:
                evidence['camera_pose_at_capture']['heading_deg'] = round(math.degrees(yaw) % 360, 2)
            evidence.pop('capture_pose', None)
    # Workflows retain pending observations independently of model history.
    context['active_observation_workflows'] = [
        {'scan_id': ident, 'pending_phase': saved['phase'], 'swept_deg': saved['swept_deg'],
         'completed_observations': len(saved['observations']), 'resume_tool': 'resume_heading_scan',
         'exit_policy': saved.get('args', {}).get('exit_policy', 'legacy'),
         'continuation': ('pending_observation' if saved['phase'] == 'observe'
                          else 'optional_next_station_after_agent_branch_decision')}
        for ident, saved in getattr(session, 'angular_scans', {}).items() if not saved['completed']
    ]
    messages = conversation_tail(session.messages, exchanges=1 if lean else 4)
    # Evidence appears exactly once in indexed observations. Repeating each
    # cumulative scan in recent tool messages exhausted small-model context.
    for message in messages:
        if message.get('role') != 'tool':
            continue
        try:
            item = json.loads(message.get('content', ''))
        except (TypeError, ValueError):
            continue
        if isinstance(item, dict) and 'evidence' in item:
            item.pop('evidence')
            item['evidence_reference'] = 'indexed observations in live feedback'
            message['content'] = json.dumps(item, separators=(',', ':'))
        if isinstance(item, dict) and isinstance(item.get('context'), dict):
            details = item['context'].get('perception', {})
            if 'nearby_obstacles' in details:
                context['requested_obstacle_geometry'] = {
                    'observed_at_unix_ms': details.get('observed_at_unix_ms'),
                    'nearby_obstacles': compact_result(details['nearby_obstacles'])}
            item.pop('context')
            item['context_reference'] = 'fresh live context below'
            message['content'] = json.dumps(item, separators=(',', ':'))
    from python.dcm.action_memory import observation_memory_view
    context.update(observation_memory_view(context['observations'],
        **({'history_bytes': 0, 'keep_recent': 0} if lean else {})))
    if lean:
        from python.dcm.action_memory import lean_execution_receipt
        context['executed_actions'] = [lean_execution_receipt(entry)
                                       for entry in context['executed_actions']]
        if context['execution_progress']['last_completed_action']:
            context['execution_progress']['last_completed_action'] = lean_execution_receipt(
                context['execution_progress']['last_completed_action'])
        context['memory_packing'] = 'Receipt details are retained; use read_action_result by index/path when needed.'
    feedback = context_message(context, mission=session.mission)["content"]
    # Native tool conversations resume on a tool result. Adding a synthetic
    # user message here can make small models restart the original mission.
    messages[-1]["content"] = (messages[-1].get("content") or "") + "\n\n" + feedback
    return messages


def executor_flight_names():
    from python.dcm.agent_tools import FLIGHT, TARGET_FLIGHT, WORKFLOW_FLIGHT

    return {**FLIGHT, **TARGET_FLIGHT, **WORKFLOW_FLIGHT}


def hold_on_error(client, echo=print):
    try:
        state = message_dict(client.state())
        if not state.get("landed") and not state.get("manual_override_active"):
            result = ToolExecutor(client).execute("hold", {})
            return "Hold: " + result["status"] + ". " + result.get("message", "")
    except Exception as error:  # noqa: BLE001 - recovery failure must be reported
        return "Could not confirm hold: " + str(error)
    return ""


def safe_exit_recovery(client, echo=print):
    state = message_dict(client.state())
    if state.get("manual_override_active"):
        return {"status": "manual_override"}
    if not state.get("landed"):
        echo("Returning home before releasing control.")
        executor = ToolExecutor(client)
        recovery = executor.execute("return_home", {})
        if recovery.get('status') == 'SUCCEEDED':
            return recovery
        state = message_dict(client.state())
        if state.get('manual_override_active'):
            return {'status': 'manual_override', 'return_home_attempt': compact_result(recovery)}
        if state.get('landed') and not state.get('armed'):
            return {'status': 'SUCCEEDED', 'return_home_attempt': compact_result(recovery)}
        echo("Return-home did not complete; requesting landing here before releasing control.")
        landing = executor.execute('land', {})
        landing['return_home_attempt'] = compact_result(recovery)
        return landing
    if state.get("armed"):
        return ToolExecutor(client).execute("disarm", {})
    return {"status": "SUCCEEDED", "message": "Aircraft landed and disarmed."}


def summarize(result):
    label = ("Agent assessment" if result.get("assessment_source") == "mission_model"
             else "Status")
    summary = (
        result["message"]
        + f"\n{label}: {result['status']}; model decisions: {result['decisions']}."
    )
    # Surface receipts even when the model's prose calls the mission complete.
    # A failed attempt may have been recovered later; this is not a mission
    # verdict and must not reject otherwise successful recovery or retries.
    unsuccessful = list(dict.fromkeys(
        f"{item['action']} [{item['result']['status']}]"
        for item in result.get('executed', [])
        if item.get('action') and item.get('result', {}).get('status')
        and item['result']['status'] != 'SUCCEEDED'))
    if unsuccessful:
        summary += "\nRecorded unsuccessful tool attempts (including retries): " + ", ".join(unsuccessful) + "."
    return summary


def session_episode_summary(results, error=None):
    success = (
        error is None
        and bool(results)
        and all(r["status"] == "completed" for r in results)
    )
    return (
        "completed" if success else "incomplete",
        {
            "success": success,
            "success_source": "mission_model_completion_reports",
            "independent_mission_verification": False,
            "missions": results,
            "error": str(error) if error else None,
        },
    )


def dumps(result):
    return json.dumps(result, indent=2)

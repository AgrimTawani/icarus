"""Mission loop: the model picks one tool, the executor runs it, the result goes back.

The conversation is append-only (system prompt, mission, then each call and
its brief result with the current vehicle state), so the model server reuses
its prompt cache and each step only processes the newest exchange.
"""

import json
import threading
import time

from python.dcm.agent_context import capture_context, system_prompt
from python.dcm.agent_tools import (
    FLIGHT, SKILLS, TARGET_FLIGHT, WORKFLOW_FLIGHT, ToolExecutor, decision_tools, drop_padding, message_dict, validate_call)
from python.dcm.operator_control import OperatorInterrupt
from python.dcm.viewpoint_gate import ViewpointGate

MAX_CONSECUTIVE_FAILURES = 3
EVIDENCE_KEYS = ('found', 'answer', 'validity', 'counts', 'count', 'objects', 'target_id', 'reason', 'surface_range_m',
                 'final_surface_range_m', 'bearing_deg', 'requested_standoff_m', 'error_code')


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
    """The mission conversation and the actions actually executed."""

    def __init__(self, mission, previous_mission=None):
        self.mission = mission
        self.previous_mission = previous_mission
        request = mission
        if previous_mission:
            request += (f"\n(Previous mission, for reference only: {previous_mission['request']}"
                        f" -> {previous_mission['outcome']})")
        self.messages = [{"role": "system", "content": system_prompt()},
                         {"role": "user", "content": request}]
        self.actions = []
        self.decisions = 0
        self.awaiting_user = False
        self.pending_question = None
        self.operator_updates = []
        self.finished = False
        self.viewpoint_gate = ViewpointGate()

    def answer(self, text):
        if not self.awaiting_user:
            raise ValueError("There is no pending clarification")
        self.messages.append({"role": "user", "content": text})
        self.operator_updates.append({'question': self.pending_question, 'answer': text})
        self.pending_question = None
        self.awaiting_user = False

    def result(self, status, message):
        if status == 'awaiting_user':
            self.pending_question = message
        return {
            "architecture": "direct-agent-v1",
            "mission": self.mission,
            "status": status,
            "message": message,
            "decisions": self.decisions,
            "executed": list(self.actions),
            "operator_updates": list(self.operator_updates),
        }


def brief(result):
    """The facts the model needs from a tool result; the episode keeps everything."""
    summary = {"status": result.get("status")}
    for key in ("message", "question", "reason_code"):
        if result.get(key):
            summary[key] = result[key]
    evidence = result.get("evidence")
    if isinstance(evidence, dict):
        facts = {key: evidence[key] for key in EVIDENCE_KEYS if key in evidence}
        if facts:
            summary["evidence"] = facts
    return summary


DEFAULT_LABELS = {'takeoff_altitude_agl_m': 'takeoff altitude {} m', 'object_standoff_m': 'object standoff {} m',
                  'overflight_clearance_m': 'overflight clearance {} m', 'yaw_rate_deg_s': 'turn rate {} deg/s',
                  'maximum_ground_speed_mps': 'ground speed {} m/s'}


def vehicle_state(context, with_defaults=False):
    """One line of current measured state for the model; operating defaults on the first step."""
    state = context.get("state", {})
    parts = [str(state.get("flight_phase", "unknown")).replace("FLIGHT_PHASE_", "").lower(),
             "armed" if state.get("armed") else "disarmed"]
    if isinstance(state.get("relative_altitude_m"), (int, float)):
        parts.append(f"altitude {state['relative_altitude_m']:.1f} m above home")
    if isinstance(state.get("heading_deg"), (int, float)):
        parts.append(f"heading {state['heading_deg'] % 360:.0f} deg")
    position = state.get("local_position_ned", {})
    if all(isinstance(position.get(k), (int, float)) for k in ("north_m", "east_m")):
        parts.append(f"position north {position['north_m']:.1f} m, east {position['east_m']:.1f} m")
    targets = [target for target in context.get("targets", []) if isinstance(target, dict) and target.get("target_id")]
    if targets:
        parts.append("measured targets: " + ", ".join(
            f"{target['target_id']} ({target.get('description', 'object')})" for target in targets))
    waypoints = [point['name'] for point in context.get("registered_waypoints", {}).get("entries", [])]
    if waypoints:
        parts.append("saved waypoints: " + ", ".join(waypoints))
    line = "Vehicle state: " + "; ".join(parts) + "."
    defaults = context.get("defaults", {})
    if with_defaults and defaults:
        line += "\nDefaults: " + ", ".join(label.format(defaults[key]) for key, label in DEFAULT_LABELS.items()
                                         if isinstance(defaults.get(key), (int, float))) + "."
    return line


def drop_oldest_exchange(messages):
    """Free context by removing the oldest call/result pair; never the request."""
    for index in range(2, len(messages) - 2):
        if messages[index].get("role") == "assistant" and messages[index + 1].get("role") == "tool":
            del messages[index:index + 2]
            return True
    return False


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
    interrupt = interrupt or OperatorInterrupt(echo=echo)
    interrupt.enable_safety(client)
    interrupt.arm(client)
    executor = ToolExecutor(client, interrupt, session=session)
    visual_inference = getattr(runtime, "visual_inference", None)
    if callable(visual_inference):
        client.visual_inference = visual_inference
    from python.dcm.llama_runtime import IncompleteModelResponse
    from python.dcm.runtime_config import ContextBudgetExceeded
    failures = 0
    try:
        while session.decisions < max_decisions:
            if interrupt.sequence():
                raise InterruptedError("Operator stopped the mission")
            context = context_provider(client)
            from python.dcm.destinations import catalogue_context
            context['registered_waypoints'] = catalogue_context(True)  # names only; tools resolve poses
            tools = decision_tools(context)
            first = not any(m.get("role") == "assistant" for m in session.messages)
            session.messages[-1]["content"] = ((session.messages[-1].get("content") or "") + "\n"
                                               + vehicle_state(context, with_defaults=first))
            if show_prompts:
                echo(json.dumps({"messages": session.messages, "tools": tools}, indent=2))
            started = time.monotonic()
            concise = False
            messages = list(session.messages)  # snapshot: the conversation keeps growing
            while True:
                try:
                    response = interrupted_call(
                        (lambda: runtime.decide(messages, tools, concise=True)) if concise
                        else (lambda: runtime.decide(messages, tools)),
                        interrupt, getattr(runtime, 'stop', None))
                    break
                except ContextBudgetExceeded:
                    if not drop_oldest_exchange(session.messages):
                        raise
                    messages = list(session.messages)
                except IncompleteModelResponse as error:
                    if concise:
                        raise
                    record(client, "agent_response_retry", {"reason": str(error), "no_action_executed": True})
                    concise = True
            session.decisions += 1
            record(client, "agent_decision", {
                "messages": messages, "tools": tools, "response": response,
                "latency_ms": round((time.monotonic() - started) * 1000),
                "stats": getattr(runtime, "last_stats", None)})
            calls = response.get("tool_calls") or []
            if not calls:
                # Text is never interpreted as a flight instruction or an assertion of success.
                text = response.get("content") or "I could not choose the next action. Please clarify the remaining task."
                session.messages.append({"role": "assistant", "content": text})
                session.awaiting_user = True
                return session.result("awaiting_user", text)
            call = calls[0]  # one step at a time; extra proposals are discarded
            session.messages.append({"role": "assistant", "tool_calls": [call]})
            call_id = call.get("id", "")
            name = call.get("function", {}).get("name")
            try:
                args = call["function"].get("arguments", {})
                args = json.loads(args) if isinstance(args, str) else args
                validate_call(name, args, tools=tools, detector_classes=context.get('detector_classes'))
                args = drop_padding(name, dict(args))
            except (ValueError, KeyError, TypeError) as error:
                result = {"status": "INVALID_ARGUMENT", "message": str(error)}
                session.messages.append({"role": "tool", "tool_call_id": call_id, "content": json.dumps(result)})
                echo(f"  Invalid call {name}: {error}")
                failures += 1
                if failures >= MAX_CONSECUTIVE_FAILURES:
                    return session.result("incomplete", "The model could not form a valid tool call: " + str(error))
                continue
            if name == "ask_user":
                session.messages.append({"role": "tool", "tool_call_id": call_id,
                                         "content": "Question delivered; the next user message is the answer."})
                session.awaiting_user = True
                return session.result("awaiting_user", args["question"])
            if name == "finish":
                session.messages.append({"role": "tool", "tool_call_id": call_id, "content": "Result delivered."})
                session.finished = True
                result = session.result(args["outcome"], args["message"])
                result["assessment_source"] = "mission_model"
                return result
            if mode == "approval" and name in executor_flight_names():
                echo(f"Execute {name} {json.dumps(args)}? [y/N]")
                interrupt.disarm()
                if stream is not None and hasattr(stream, "discard_pending"):
                    stream.discard_pending()
                answer = stream.readline() if stream is not None else input()
                interrupt.arm(client)
                if interrupt.sequence():
                    raise InterruptedError("Operator stopped the mission")
                if answer.strip().lower() not in ("y", "yes"):
                    session.messages.append({"role": "tool", "tool_call_id": call_id,
                                             "content": "Operator declined; not executed."})
                    return session.result("cancelled", "You declined the action. Mission stopped.")
            echo(f"→ {name} {json.dumps(args, separators=(',', ':'))}")
            if name in ("detect", "inspect_scene", "assess_landing_zone", "localize_target"):
                # Camera work can be slow; operator recovery must still start immediately.
                result = interrupted_call(lambda: executor.execute(name, args), interrupt)
            else:
                result = executor.execute(name, args)
            session.actions.append({"action": name, "arguments": args, "result": result})
            record(client, "agent_tool_result", session.actions[-1])
            from python.dcm.observation_feedback import observation_feedback
            for line in observation_feedback(name, result, args):
                echo("  " + line)
            for view in (result.get("evidence") or {}).get("views", []) if name == "search_for_object" else []:
                echo(f"  view at {view['heading_deg']} deg: found={view['found']} — {view.get('answer') or view['status']}")
            for step in (result.get("evidence") or {}).get("acquisition") or []:
                echo(f"  acquire: found={step.get('found')} reason={step.get('reason')} range={step.get('surface_range_m')}"
                     + (f" then {step['then']['action']} {step['then']['status']}" if step.get('then') else ""))
            if result.get("status") != "SUCCEEDED":
                echo(f"  Tool result: {result.get('status')} — {result.get('message', '')}")
            session.messages.append({"role": "tool", "tool_call_id": call_id,
                                     "content": json.dumps(brief(result), separators=(",", ":"))})
            if interrupt.sequence():
                raise InterruptedError("Operator stopped the mission")
            if result.get("requires_user_choice"):
                session.awaiting_user = True
                return session.result("awaiting_user", result["question"])
            if name == "cancel" and result.get("status") == "SUCCEEDED":
                return session.result("cancelled", "Mission cancelled. " + result.get("message", ""))
            failures = 0 if result.get("status") == "SUCCEEDED" else failures + 1
            if failures >= MAX_CONSECUTIVE_FAILURES:
                return session.result("incomplete", f"Stopped after {failures} unsuccessful calls in a row. "
                                      + result.get("message", "Check the latest tool result."))
        return session.result("incomplete", f"Reached {max_decisions} decisions; remaining work was not executed.")
    except InterruptedError:
        while interrupt.latest():
            latest = interrupt.latest()
            executor.expected_interrupt = interrupt.sequence()
            result = executor.execute(latest["action"], {})
            session.actions.append({"action": latest["action"], "arguments": {}, "result": result, "operator": True})
            if executor.expected_interrupt == interrupt.sequence():
                return session.result(
                    "cancelled", f"Mission stopped by you. {latest['action']}: {result['status']}. {result.get('message', '')}")
        return session.result("cancelled", "Mission interrupted.")
    except Exception as error:  # noqa: BLE001 - report boundary failures without losing control cleanup
        record(client, "agent_error", {"message": str(error)})
        recovery = hold_on_error(client, echo)
        return session.result("incomplete", f"Mission stopped: {error}. {recovery}")
    finally:
        interrupt.disarm()
        interrupt.disable_safety()


def executor_flight_names():
    return {**FLIGHT, **TARGET_FLIGHT, **WORKFLOW_FLIGHT, **SKILLS}


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
            return {'status': 'manual_override', 'return_home_attempt': brief(recovery)}
        if state.get('landed') and not state.get('armed'):
            landing = executor.execute('land', {})
            landing['return_home_attempt'] = brief(recovery)
            return landing
        echo("Return-home did not complete; requesting landing here before releasing control.")
        landing = executor.execute('land', {})
        landing['return_home_attempt'] = brief(recovery)
        return landing
    if state.get("armed"):
        disarm = ToolExecutor(client).execute("disarm", {})
        if disarm.get('status') != 'SUCCEEDED':
            return disarm
    return ToolExecutor(client).execute('land', {})


def summarize(result):
    label = "Agent assessment" if result.get("assessment_source") == "mission_model" else "Status"
    summary = result["message"] + f"\n{label}: {result['status']}; model decisions: {result['decisions']}."
    # Surface receipts even when the model's prose calls the mission complete.
    unsuccessful = list(dict.fromkeys(
        f"{item['action']} [{item['result']['status']}]"
        for item in result.get('executed', [])
        if item.get('action') and item.get('result', {}).get('status')
        and item['result']['status'] != 'SUCCEEDED'))
    if unsuccessful:
        summary += "\nRecorded unsuccessful tool attempts (including retries): " + ", ".join(unsuccessful) + "."
    return summary


def session_episode_summary(results, error=None):
    success = error is None and bool(results) and all(r["status"] == "completed" for r in results)
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

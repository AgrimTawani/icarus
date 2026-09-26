"""Live DCM flight loop: chat a mission, watch proposals, approve, execute.

This is the first code path where a model proposal can reach an aircraft, so
the ordering matters and is deliberate:

    live state -> curate -> freshness -> model -> validate -> freshness again
    -> operator approval -> Drone API -> guardrails -> executor

The model never touches MAVLink, never chooses what it can see, and never
executes anything itself. It returns one JSON object; everything after that is
this module's dispatch table, the Drone API's own validation and the
independent safety supervisor.

Freshness is checked **twice**, which offline replay never needed. A decision
costs 0.5 s when warm and up to 13 s cold, so the world can move between the
observation the model reasoned about and the moment its answer arrives. The
second check discards a proposal computed against a state that no longer
holds, rather than executing stale reasoning.
"""

import json
import re
import sys
import time
import uuid

from python.dcm.actions import ACTION_METHODS, build_request
from python.dcm.contract import (
    ALLOWED_ACTIONS,
    DeadlineExceeded,
    assess_freshness,
    curate,
    curate_v3,
    material_state_change,
    render_prompt,
    validate_decision,
)
from python.dcm.health import DcmHealthMachine

MODES = ("approval", "autonomous")
MAX_DECISIONS = 40
FAILED_ACTION_RECOVERY_BACKOFF_S = 0.5


class MissionMemory:
    """What the model is told about a mission and what has happened in it.

    Deliberately small: the mission text as the operator typed it, and the
    actions that have actually completed. Adding history is what took the
    model from never proposing `land` to proposing it, so this is load-bearing
    rather than convenience.
    """

    def __init__(self, mission, macro_id=None):
        self.mission = mission
        self.mission_metadata = ({"macro_id": macro_id} if macro_id else None)
        self.completed = []
        self.rejected = 0
        self.last_feedback = None

    def remember(self, action, outcome, result=None, arguments=None):
        item = {"action": action, "outcome": outcome}
        if result is not None:
            item["result"] = result
        if arguments is not None:
            item["arguments"] = dict(arguments)
        self.completed.append(item)
        self.last_feedback = {"action": action, "outcome": outcome}

    def reject(self, proposal, error):
        self.rejected += 1
        self.last_feedback = {
            "proposal_rejected": (proposal or {}).get("action"),
            "reason": str(error),
        }

    def as_history(self):
        return list(self.completed)

    def repeated_failure_action(self, threshold=3):
        """Return an action that failed consecutively enough to stop retries."""
        if len(self.completed) < threshold:
            return None
        recent = self.completed[-threshold:]
        action = recent[0].get("action")
        if (action and all(item.get("action") == action and
                           item.get("outcome") != "SUCCEEDED"
                           for item in recent)):
            return action
        return None


def observe(client, memory, decision_index, context_version="v2"):
    """Build the bounded observation from live state and perception.

    Absent perception yields an empty dict rather than an exception, because
    the freshness rule already refuses an observation without a map age. A
    missing sensor should stop the model being asked, not stop the process.
    """
    from icarus.v1 import drone_api_pb2, drone_api_pb2_grpc

    from python.dataset_tools.episode import _safe_message

    state = _safe_message(client.state())
    perception = {}
    try:
        stub = drone_api_pb2_grpc.PerceptionServiceStub(client.channel)
        perception = _safe_message(stub.GetPerception(
            drone_api_pb2.GetPerceptionRequest(
                session_id=client.session_id, vehicle_id="icarus-01"),
            timeout=2))
    except Exception:  # noqa: BLE001 - absent perception is refused, not fatal
        perception = {}
    now = int(time.time() * 1000)
    event = {"seq": decision_index, "unix_ms": now}
    if context_version not in ("v2", "v3"):
        raise ValueError("context_version must be v2 or v3")
    builder = curate_v3 if context_version == "v3" else curate
    runtime_body = getattr(client, "capability_profile", lambda: None)()
    kwargs = ({"mission_metadata": memory.mission_metadata,
               "runtime_body": runtime_body}
              if context_version == "v3" else {})
    observation = builder(state, perception, memory.last_feedback, memory.mission, event,
                         history=memory.as_history(), **kwargs)
    reference = getattr(client, "mission_reference", dict)()
    if reference:
        observation["mission_reference"] = reference
    obligations = getattr(client, "mission_obligations",
                          getattr(client, "mission_guidance", lambda *_: {}))(
        memory.mission, memory.as_history(), observation)
    if obligations:
        observation["mission_obligations"] = obligations
    if context_version == "v3":
        from python.dcm.v2_context import compile_mission_contract, finalize_context
        observation["mission_contract"] = compile_mission_contract(
            memory.mission, observation.get("mission_macro"), reference,
            obligations)
        observation = finalize_context(observation)
    return observation


def _observe_for_context(client, memory, decision_index, context_version):
    """Preserve the legacy three-argument observation seam for V1/V2 callers."""
    if context_version == "v2":
        return observe(client, memory, decision_index)
    return observe(client, memory, decision_index, context_version)


def approve(proposal, mode, stream=sys.stdin, out=None):
    """Return True when an action may execute.

    In approval mode the default is **no**: a bare Enter, an EOF or anything
    that is not an explicit yes declines. An operator who is not paying
    attention should not be able to launch an aircraft by pressing return.
    """
    if mode == "autonomous":
        return True
    prompt = f"    approve {_describe(proposal)}? [y/N] "
    if out is None:
        print(prompt, end="", flush=True)
    else:
        out(prompt)
    answer = stream.readline()
    return answer.strip().lower() in ("y", "yes")


def _describe(proposal):
    arguments = proposal.get("arguments") or {}
    if not arguments:
        return proposal["action"]
    rendered = ", ".join(f"{name}={value}" for name, value in arguments.items())
    return f"{proposal['action']} ({rendered})"


def _recovery_evidence(observation):
    """Return deterministic evidence that permits abandoning pending objectives."""
    state = observation.get("state") or {}
    summary = observation.get("state_summary") or {}
    failures = observation.get("failure_signals") or {}
    if state.get("manual_override_active") or state.get("geofence_breached"):
        return True
    if int(failures.get("consecutive_failures") or 0) >= 2:
        return True
    battery = summary.get("battery") or {}
    if type(battery.get("remaining_percent")) in (int, float) and \
            battery["remaining_percent"] <= 20:
        return True
    unhealthy = {"HEALTH_LEVEL_DEGRADED", "HEALTH_LEVEL_CRITICAL",
                 "HEALTH_LEVEL_UNAVAILABLE"}
    if battery.get("level") in unhealthy:
        return True
    estimator = summary.get("estimator") or {}
    if estimator.get("level") in unhealthy or any(
            estimator.get(key) is False for key in
            ("attitude_valid", "horizontal_position_valid", "vertical_position_valid")):
        return True
    mavlink = state.get("mavlink") or {}
    return mavlink.get("connected") is False or mavlink.get("level") in unhealthy


def _mission_progress_error(observation, proposal):
    """Reject safe early termination while measurable mission work is pending.

    The model still selects every progress action. This gate only prevents a
    terminal proposal from falsely declaring completion; deterministic safety
    evidence always retains authority to abandon the mission.
    """
    contract = observation.get("mission_contract") or {}
    pending = list(contract.get("pending") or [])
    action = proposal.get("action")
    if not pending or action not in ("return_home", "land", "none"):
        return None
    blocking = ([item for item in pending if item != "return_home"]
                if action == "return_home" else pending)
    if not blocking or _recovery_evidence(observation):
        return None
    return (f"terminal action {action} cannot bypass pending objectives: "
            + ", ".join(blocking))


_OBJECTIVE_ACTIONS = {
    "flight": {"arm", "takeoff", "hold"},
    "building_visual_inspection": {"goto", "hold", "inspect_scene"},
    "building_hover": {"goto", "hold"},
    "inspection_orbit": {"goto", "hold", "orbit"},
    "person_count": {"goto", "hold", "detect"},
    "hold_position": {"hold"},
    "return_home": {"hold", "return_home", "land"},
}


def _objective_progress_error(observation, proposal):
    """Enforce the active evidence objective without selecting its next action."""
    contract = observation.get("mission_contract") or {}
    active = contract.get("active_objective_id")
    action = proposal.get("action")
    if not active or _recovery_evidence(observation):
        return None
    allowed = set(_OBJECTIVE_ACTIONS.get(active, ()))
    if not allowed:
        return None
    objective = next((item for item in contract.get("objectives", [])
                      if item.get("id") == active), {})
    if objective.get("allowed_progress_actions"):
        allowed = set(objective["allowed_progress_actions"])
    if action not in allowed:
        return (f"action {action} does not advance active objective {active}; "
                f"allowed progress actions are {', '.join(sorted(allowed))}")

    def differs(arguments, reviewed, keys):
        return any(type(arguments.get(key)) not in (int, float) or
                   abs(float(arguments[key]) - float(reviewed[key])) > 1e-6
                   for key in keys)

    arguments = proposal.get("arguments") or {}
    if action == "goto" and objective.get("reviewed_waypoint") and differs(
            arguments, objective["reviewed_waypoint"],
            (("north_m", "east_m", "altitude_agl_m", "heading_deg")
             if "heading_deg" in objective["reviewed_waypoint"] else
             ("north_m", "east_m", "altitude_agl_m"))):
        return f"goto must use the reviewed waypoint for active objective {active}"
    if action == "orbit" and objective.get("reviewed_orbit") and differs(
            arguments, objective["reviewed_orbit"],
            ("center_north_m", "center_east_m", "radius_m",
             "altitude_agl_m", "revolutions")):
        return "orbit must use the reviewed geometry for the active objective"
    if action == "takeoff" and objective.get("target_altitude_agl_m") is not None and \
            abs(float(arguments.get("target_altitude_agl_m", -1)) -
                float(objective["target_altitude_agl_m"])) > 1e-6:
        return "takeoff must use the requested altitude for the active objective"
    if action == "hold" and active == "hold_position" and \
            int(arguments.get("duration_ms", 0)) < int(objective.get("duration_ms", 0)):
        return "hold duration is shorter than the active objective"
    if action == "detect" and objective.get("detector_classes") and not set(
            objective["detector_classes"]).issubset(arguments.get("classes", [])):
        return "detect omits a class required by the active objective"
    if action == "inspect_scene" and active == "building_visual_inspection":
        question = str(arguments.get("question", "")).lower()
        if not any(noun in question for noun in ("building", "structure", "tower")):
            return "building inspection question must ask about the reviewed structure"
        if re.search(r"\b(?:count|people|persons?|how many)\b", question):
            return "building inspection question must not replace detector object counting"
    return None


def _physical_transition_error(observation, proposal):
    """Refuse lifecycle/navigation no-ops the autopilot might accept idempotently."""
    phase = (observation.get("state_summary") or {}).get("flight_phase") or \
        (observation.get("state") or {}).get("flight_phase")
    action = proposal.get("action")
    if action == "arm" and phase not in ("FLIGHT_PHASE_DISARMED", None):
        return f"arm is invalid in {phase}"
    if action == "takeoff" and phase != "FLIGHT_PHASE_ARMED_GROUND":
        return f"takeoff is invalid in {phase}"
    if action in ("goto", "orbit") and phase != "FLIGHT_PHASE_AIRBORNE":
        return f"{action} requires an airborne aircraft, found {phase}"
    return None


def fly_mission(client, runtime, mission, mode="approval", descriptor=None,
                max_decisions=MAX_DECISIONS, stream=sys.stdin, echo=print,
                context_version="v2", macro_id=None):
    """Run one mission to termination. Returns a summary dict."""
    if mode not in MODES:
        raise ValueError("mode must be one of " + ", ".join(MODES))
    if context_version not in ("v2", "v3"):
        raise ValueError("context_version must be v2 or v3")
    from icarus.v1 import action_pb2

    episode = getattr(client, "episode", None)
    if descriptor is not None and episode is not None:
        set_model = getattr(episode, "set_model", None)
        if set_model is not None:
            set_model(descriptor.as_record())

    memory = MissionMemory(mission, macro_id=macro_id)
    health = DcmHealthMachine()
    executed = []
    counts = {"proposed": 0, "invalid": 0, "stale": 0, "timeout": 0,
              "declined": 0, "executed": 0, "failed": 0}

    for index in range(1, max_decisions + 1):
        observation = _observe_for_context(client, memory, index, context_version)
        repeated_failure = memory.repeated_failure_action()
        if repeated_failure:
            error = (f"stopped after 3 consecutive failed {repeated_failure} "
                     "attempts; further identical commands are suppressed")
            echo(f"[{index}] {error}")
            _record(client, index, observation, None, None,
                    "repeated_failure_breaker", error,
                    health=health.record())
            break
        stale = assess_freshness(observation)
        if stale:
            counts["stale"] += 1
            transition = health.transition("stale", airborne=_airborne(observation))
            echo(f"[{index}] refused before asking: {stale}")
            if transition.fallback_action:
                fallback = {"action": transition.fallback_action, "arguments": {}}
                outcome, detail = _execute(client, action_pb2, fallback, index, echo)
                counts["executed"] += 1
                counts["failed"] += outcome != "SUCCEEDED"
                memory.remember(fallback["action"], outcome,
                                arguments=fallback["arguments"])
                executed.append({**fallback, "outcome": outcome, "senior_fallback": True})
                _record(client, index, observation, None, None,
                        "fallback_executed", stale, proposal=fallback,
                        outcome=outcome, result=detail,
                        health=health.record())
            else:
                _record(client, index, observation, None, None, "stale", stale,
                        health=health.record())
            time.sleep(0.5)
            continue

        if not health.allow_inference():
            action = health.circuit_fallback(airborne=_airborne(observation))
            if action and action != "none":
                fallback = {"action": action, "arguments": {}}
                outcome, detail = _execute(client, action_pb2, fallback, index, echo)
                counts["executed"] += 1
                counts["failed"] += outcome != "SUCCEEDED"
                memory.remember(action, outcome,
                                arguments=fallback["arguments"])
                executed.append({**fallback, "outcome": outcome,
                                 "senior_fallback": True})
                _record(client, index, observation, None, None,
                        "circuit_suppressed", "DCM circuit is open",
                        proposal=fallback, outcome=outcome,
                        result=detail, health=health.record())
                if action in ("return_home", "land") and outcome == "SUCCEEDED":
                    break
            else:
                _record(client, index, observation, None, None,
                        "circuit_suppressed", "DCM circuit is open",
                        health=health.record())
            continue

        prompt = render_prompt(observation)
        started = time.monotonic()
        raw = None
        reason_code = None
        try:
            raw = runtime.propose(observation)
            proposal, reason_code = validate_decision(
                raw, allow_reason_code=context_version == "v3")
            status = "valid"
            error = None
        except DeadlineExceeded as expired:
            counts["timeout"] += 1
            status, proposal, error = "timeout", None, str(expired)
        except ValueError as invalid:
            counts["invalid"] += 1
            status, proposal, error = "invalid", None, str(invalid)
        latency_ms = (time.monotonic() - started) * 1000
        counts["proposed"] += 1

        macro = observation.get("mission_macro") or {}
        if proposal is not None and macro:
            always_safe = {"none", "hold", "return_home", "land"}
            permitted = set(macro.get("allowed_actions", ())) | always_safe
            if proposal["action"] not in permitted:
                counts["invalid"] += 1
                error = (f"action {proposal['action']} is outside reviewed macro "
                         f"{macro.get('id')} permissions")
                transition = health.transition("invalid", airborne=_airborne(observation))
                echo(f"[{index}] refused {proposal['action']}: {error}")
                memory.reject(proposal, error)
                _record(client, index, observation, prompt, raw, "invalid", error,
                        latency_ms, proposal=proposal, reason_code=reason_code,
                        health=health.record())
                continue

        if proposal is not None:
            progress_error = ((_physical_transition_error(observation, proposal)
                               if context_version == "v3" else None) or
                              _mission_progress_error(observation, proposal) or
                              _objective_progress_error(observation, proposal))
            if progress_error:
                counts["invalid"] += 1
                transition = health.transition("invalid", airborne=_airborne(observation))
                echo(f"[{index}] refused {proposal['action']}: {progress_error}")
                memory.reject(proposal, progress_error)
                _record(client, index, observation, prompt, raw, "invalid",
                        progress_error, latency_ms, proposal=proposal,
                        reason_code=reason_code, health=health.record())
                continue

        if proposal is None:
            memory.reject(None, error)
            transition = health.transition(status, airborne=_airborne(observation))
            echo(f"[{index}] {status}: {error}")
            if transition.fallback_action:
                fallback = {"action": transition.fallback_action, "arguments": {}}
                outcome, detail = _execute(client, action_pb2, fallback, index, echo)
                counts["executed"] += 1
                counts["failed"] += outcome != "SUCCEEDED"
                memory.remember(fallback["action"], outcome,
                                arguments=fallback["arguments"])
                executed.append({**fallback, "outcome": outcome, "senior_fallback": True})
                _record(client, index, observation, prompt, raw,
                        "fallback_executed", error, latency_ms, fallback, outcome,
                        reason_code=reason_code, health=health.record(),
                        result=detail)
                if fallback["action"] in ("return_home", "land") and \
                        outcome == "SUCCEEDED":
                    break
            else:
                _record(client, index, observation, prompt, raw, status, error,
                        latency_ms, reason_code=reason_code, health=health.record())
            continue

        # The world may have moved while the model was thinking. Offline
        # replay could not surface this; a 13 s cold start can.
        confirm = _observe_for_context(client, memory, index, context_version)
        moved = assess_freshness(confirm) or material_state_change(observation, confirm)
        if moved:
            counts["stale"] += 1
            transition = health.transition(
                "stale_after", airborne=_airborne(observation))
            echo(f"[{index}] discarded {proposal['action']}: {moved}")
            if transition.fallback_action:
                fallback = {"action": transition.fallback_action, "arguments": {}}
                outcome, detail = _execute(
                    client, action_pb2, fallback, index, echo)
                counts["executed"] += 1
                counts["failed"] += outcome != "SUCCEEDED"
                memory.remember(fallback["action"], outcome,
                                arguments=fallback["arguments"])
                executed.append({**fallback, "outcome": outcome,
                                 "senior_fallback": True})
                _record(client, index, observation, prompt, raw,
                        "fallback_executed", moved, latency_ms,
                        proposal=fallback, outcome=outcome,
                        result=detail, health=health.record())
                if fallback["action"] in ("return_home", "land") and \
                        outcome == "SUCCEEDED":
                    break
            else:
                _record(client, index, observation, prompt, raw, "stale_after",
                        moved, latency_ms, health=health.record())
            continue

        echo(f"[{index}] proposes {_describe(proposal)}"
             f"  ({latency_ms:.0f} ms)")
        if proposal["action"] == "none":
            health.transition("none")
            echo("      model proposes no action; mission ends")
            _record(client, index, observation, prompt, raw, "none", None,
                    latency_ms, reason_code=reason_code, health=health.record())
            break

        if not approve(proposal, mode, stream,
                       out=None if echo is print else echo):
            counts["declined"] += 1
            memory.reject(proposal, "operator declined proposal")
            echo("      declined")
            _record(client, index, observation, prompt, raw, "declined", None,
                    latency_ms, proposal, operator_approval=False,
                    reason_code=reason_code, health=health.record())
            continue

        outcome, detail = _execute(client, action_pb2, proposal, index, echo)
        health.transition("executed" if outcome == "SUCCEEDED" else "error",
                          airborne=_airborne(observation))
        memory.remember(
            proposal["action"], outcome,
            detail if proposal["action"] in
            ("detect", "inspect_scene", "assess_landing_zone") and
            outcome == "SUCCEEDED" else None,
            arguments=proposal["arguments"])
        executed.append({"action": proposal["action"],
                         "arguments": proposal["arguments"],
                         "outcome": outcome})
        if proposal["action"] in ("detect", "inspect_scene", "assess_landing_zone") and outcome == "SUCCEEDED":
            executed[-1]["result"] = detail
        counts["executed"] += 1
        if outcome not in ("SUCCEEDED",):
            counts["failed"] += 1
        _record(client, index, observation, prompt, raw, "executed",
                None if outcome == "SUCCEEDED" else str(detail),
                latency_ms, proposal, outcome, result=detail,
                operator_approval=(True if mode == "approval" else None),
                reason_code=reason_code, health=health.record())

        # A faulted command path can recover a fraction of a second later.
        # Without a backoff an autonomous client repeatedly submits the same
        # command in one scheduler timeslice, which adds no useful recovery
        # opportunity and can flood a recovering control link.
        if outcome != "SUCCEEDED":
            time.sleep(FAILED_ACTION_RECOVERY_BACKOFF_S)

        if proposal["action"] in ("land", "return_home") and outcome == "SUCCEEDED":
            echo("      aircraft is down; mission complete")
            break

    # Never release a live V3 control lease merely because the model proposed
    # `none`, exhausted its budget, or entered its circuit breaker. A senior,
    # deterministic recovery returns an airborne aircraft before the client
    # closes. This does not turn an incomplete semantic mission into success.
    if context_version == "v3" and \
            memory.repeated_failure_action() != "return_home" and not any(
            item["action"] in ("return_home", "land") and
            item["outcome"] == "SUCCEEDED" for item in executed):
        recovery_observation = _observe_for_context(
            client, memory, max_decisions + 1, context_version)
        if _airborne(recovery_observation):
            fallback = {"action": "return_home", "arguments": {}}
            echo("decision loop ended airborne; senior recovery requests return_home")
            outcome, detail = _execute(
                client, action_pb2, fallback, max_decisions + 1, echo)
            counts["executed"] += 1
            counts["failed"] += outcome != "SUCCEEDED"
            memory.remember("return_home", outcome, arguments={})
            executed.append({**fallback, "outcome": outcome,
                             "senior_fallback": True})
            _record(client, max_decisions + 1, recovery_observation, None, None,
                    "mission_exit_recovery",
                    None if outcome == "SUCCEEDED" else str(detail),
                    proposal=fallback, outcome=outcome,
                    result=detail, health=health.record())

    completion = getattr(client, "assess_mission_completion", lambda *_: None)(mission, executed)
    return {"mission": mission, "mode": mode, "counts": counts,
            "executed": executed, "semantic_completion": completion,
            "model": descriptor.as_record() if descriptor else None}


def session_episode_summary(results, error=None):
    """Return the final outcome and compact score stored when a DCM client closes.

    ``MissionClient.close`` seals its active episode unconditionally.  The live
    chat wrapper may run several missions in that one client session, so it
    must give ``close`` a truthful terminal outcome before it does so.  A
    declined proposal is an operator decision, not a failed flight; a terminal
    action failure or an uncaught session error is retained as a failed
    episode.
    """
    failed = error is not None or any(
        result.get("counts", {}).get("failed", 0) > 0
        or (result.get("semantic_completion") or {}).get("success") is False
        for result in results)
    score = {"missions": list(results), "status": "failed" if failed else "completed"}
    if error is not None:
        score["error"] = {"type": type(error).__name__, "message": str(error)}
    return score["status"], score


def _execute(client, action_pb2, proposal, index, echo):
    """Send one approved action and wait for its terminal state."""
    action = proposal["action"]
    if action == "detect":
        try:
            result = client.detect(proposal["arguments"]["classes"])
        except Exception as failure:  # noqa: BLE001 - semantic failures are data
            echo(f"      detect failed: {failure}")
            return "FAILED", str(failure)
        echo("      -> SUCCEEDED (" + json.dumps(result, sort_keys=True) + ")")
        return "SUCCEEDED", result
    if action == "assess_landing_zone":
        try:
            result = client.assess_landing_zone()
        except Exception as failure:  # noqa: BLE001 - semantic failures are data
            echo(f"      landing assessment failed: {failure}")
            return "FAILED", str(failure)
        echo("      -> SUCCEEDED (" + json.dumps(result, sort_keys=True) + ")")
        return "SUCCEEDED", result
    if action == "inspect_scene":
        try:
            result = client.inspect_scene(proposal["arguments"]["question"])
        except Exception as failure:  # noqa: BLE001 - inspection failures are data
            echo(f"      visual inspection failed: {failure}")
            return "FAILED", str(failure)
        echo("      -> SUCCEEDED (" + json.dumps(result, sort_keys=True) + ")")
        return "SUCCEEDED", result
    # A unique name per decision: context() derives the idempotency key from
    # it, so reusing a name would make the second identical action a no-op.
    context = client.context(f"{action}-{index}-{uuid.uuid4().hex[:8]}")
    request, timeout = build_request(action_pb2, action,
                                     proposal["arguments"], context)
    method = ACTION_METHODS[action]
    try:
        receipt = getattr(client.action_api, method)(request, timeout=5)
        terminal = client.wait_action(receipt, timeout)
    except Exception as failure:  # noqa: BLE001 - a rejected action is data
        echo(f"      {action} failed: {failure}")
        return "FAILED", str(failure)
    outcome = terminal.state.replace("ACTION_STATE_", "") if isinstance(
        terminal.state, str) else _state_name(terminal.state)
    echo(f"      -> {outcome}"
         + (f" ({terminal.message})" if getattr(terminal, "message", "") else ""))
    return outcome, getattr(terminal, "message", "")


def _state_name(value):
    from icarus.v1 import action_pb2
    return action_pb2.ActionState.Name(value).replace("ACTION_STATE_", "")


def _airborne(observation):
    state = observation.get("state") or {}
    summary = observation.get("state_summary") or {}
    phase = summary.get("flight_phase") or state.get("flight_phase")
    if phase == "FLIGHT_PHASE_AIRBORNE":
        return True
    if phase in ("FLIGHT_PHASE_DISARMED", "FLIGHT_PHASE_ARMED_GROUND"):
        return False
    armed = summary.get("armed", state.get("armed"))
    landed = summary.get("landed", state.get("landed"))
    return armed is True and landed is not True


def _record(client, index, observation, prompt, raw, status, error,
            latency_ms=0.0, proposal=None, outcome=None,
            operator_approval=None, reason_code=None, health=None, result=None):
    """Write the decision into the episode.

    Phase 10 requires model prompts and responses to be replayable alongside
    state, actions and guardrail decisions, so the prompt actually sent and
    the response actually received are recorded here rather than reconstructed
    later from a version number.
    """
    episode = getattr(client, "episode", None)
    if episode is None:
        return
    episode.record("dcm_decision", {
        "decision": index,
        "observation": observation,
        "prompt": prompt,
        "raw_response": raw if isinstance(raw, str) else None,
        "reason_code": reason_code,
        "validator_verdict": ("accepted" if proposal is not None and
                               status != "invalid" else "not_accepted"),
        "proposal": proposal,
        "status": status,
        "error": error,
        "result": result,
        "latency_ms": round(latency_ms, 3),
        "outcome": outcome,
        "executed": (proposal is not None and outcome is not None and
                     status in ("executed", "fallback_executed",
                                "circuit_suppressed", "mission_exit_recovery")),
        # True/False means an approval-mode operator made an explicit decision;
        # null means no operator approval was requested (autonomous mode or a
        # pre-execution refusal). This is annotation data, not authority.
        "operator_approval": operator_approval,
        "allowed_actions": list(ALLOWED_ACTIONS),
        "mission_macro": observation.get("mission_macro"),
        "context_sha256": observation.get("context_sha256"),
        "health": health,
    })


def summarize(result):
    counts = result["counts"]
    lines = [
        f"mission   {result['mission']}",
        f"mode      {result['mode']}",
        "counts    " + "  ".join(f"{k}={v}" for k, v in counts.items()),
    ]
    if result["executed"]:
        lines.append("executed  " + ", ".join(
            f"{a['action']}:{a['outcome']}" for a in result["executed"]))
    else:
        lines.append("executed  nothing")
    return "\n".join(lines)


def dumps(result):
    return json.dumps(result, indent=2, sort_keys=True)

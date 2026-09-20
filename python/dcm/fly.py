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
import sys
import time
import uuid

from python.dcm.actions import ACTION_METHODS, build_request
from python.dcm.contract import (
    ALLOWED_ACTIONS,
    DeadlineExceeded,
    assess_freshness,
    curate,
    render_prompt,
    validate_proposal,
)

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

    def __init__(self, mission):
        self.mission = mission
        self.completed = []
        self.rejected = 0

    def remember(self, action, outcome):
        self.completed.append({"action": action, "outcome": outcome})

    def as_history(self):
        return list(self.completed)


def observe(client, memory, decision_index):
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
    return curate(state, perception, None, memory.mission, event,
                  history=memory.as_history())


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


def fly_mission(client, runtime, mission, mode="approval", descriptor=None,
                max_decisions=MAX_DECISIONS, stream=sys.stdin, echo=print):
    """Run one mission to termination. Returns a summary dict."""
    if mode not in MODES:
        raise ValueError("mode must be one of " + ", ".join(MODES))
    from icarus.v1 import action_pb2

    memory = MissionMemory(mission)
    executed = []
    counts = {"proposed": 0, "invalid": 0, "stale": 0, "timeout": 0,
              "declined": 0, "executed": 0, "failed": 0}

    for index in range(1, max_decisions + 1):
        observation = observe(client, memory, index)
        stale = assess_freshness(observation)
        if stale:
            counts["stale"] += 1
            echo(f"[{index}] refused before asking: {stale}")
            _record(client, index, observation, None, None, "stale", stale)
            time.sleep(0.5)
            continue

        prompt = render_prompt(observation)
        started = time.monotonic()
        raw = None
        try:
            raw = runtime.propose(observation)
            proposal = validate_proposal(raw)
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

        if proposal is None:
            echo(f"[{index}] {status}: {error}")
            _record(client, index, observation, prompt, raw, status, error,
                    latency_ms)
            continue

        # The world may have moved while the model was thinking. Offline
        # replay could not surface this; a 13 s cold start can.
        confirm = observe(client, memory, index)
        moved = assess_freshness(confirm)
        if moved:
            counts["stale"] += 1
            echo(f"[{index}] discarded {proposal['action']}: {moved}")
            _record(client, index, observation, prompt, raw, "stale_after",
                    moved, latency_ms)
            continue

        echo(f"[{index}] proposes {_describe(proposal)}"
             f"  ({latency_ms:.0f} ms)")
        if proposal["action"] == "none":
            echo("      model proposes no action; mission ends")
            _record(client, index, observation, prompt, raw, "none", None,
                    latency_ms)
            break

        if not approve(proposal, mode, stream,
                       out=None if echo is print else echo):
            counts["declined"] += 1
            memory.rejected += 1
            echo("      declined")
            _record(client, index, observation, prompt, raw, "declined", None,
                    latency_ms, proposal, operator_approval=False)
            continue

        outcome, detail = _execute(client, action_pb2, proposal, index, echo)
        memory.remember(proposal["action"], outcome)
        executed.append({"action": proposal["action"],
                         "arguments": proposal["arguments"],
                         "outcome": outcome})
        counts["executed"] += 1
        if outcome not in ("SUCCEEDED",):
            counts["failed"] += 1
        _record(client, index, observation, prompt, raw, "executed", detail,
                latency_ms, proposal, outcome,
                operator_approval=(True if mode == "approval" else None))

        # A faulted command path can recover a fraction of a second later.
        # Without a backoff an autonomous client repeatedly submits the same
        # command in one scheduler timeslice, which adds no useful recovery
        # opportunity and can flood a recovering control link.
        if outcome != "SUCCEEDED":
            time.sleep(FAILED_ACTION_RECOVERY_BACKOFF_S)

        if proposal["action"] in ("land", "return_home") and outcome == "SUCCEEDED":
            echo("      aircraft is down; mission complete")
            break

    return {"mission": mission, "mode": mode, "counts": counts,
            "executed": executed,
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
        result.get("counts", {}).get("failed", 0) > 0 for result in results)
    score = {"missions": list(results), "status": "failed" if failed else "completed"}
    if error is not None:
        score["error"] = {"type": type(error).__name__, "message": str(error)}
    return score["status"], score


def _execute(client, action_pb2, proposal, index, echo):
    """Send one approved action and wait for its terminal state."""
    action = proposal["action"]
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


def _record(client, index, observation, prompt, raw, status, error,
            latency_ms=0.0, proposal=None, outcome=None,
            operator_approval=None):
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
        "proposal": proposal,
        "status": status,
        "error": error,
        "latency_ms": round(latency_ms, 3),
        "outcome": outcome,
        "executed": status == "executed",
        # True/False means an approval-mode operator made an explicit decision;
        # null means no operator approval was requested (autonomous mode or a
        # pre-execution refusal). This is annotation data, not authority.
        "operator_approval": operator_approval,
        "allowed_actions": list(ALLOWED_ACTIONS),
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

"""Offline, observe-only DCM decision replay.

This module deliberately has no Drone API, MAVLink, or simulator imports. A
runtime receives only a curated snapshot and returns one JSON proposal. The
proposal is syntax-checked and recorded; it is never sent to an aircraft.
"""

import hashlib
import json
import math
import time
import uuid
from pathlib import Path
from typing import Protocol

from python.dataset_tools.replay import replay

SCHEMA = "icarus.dcm.observe.v1"
ALLOWED_ACTIONS = ("none", "arm", "takeoff", "hold", "return_home", "land")
TERMINAL = {
    "ACTION_STATE_REJECTED", "ACTION_STATE_SUCCEEDED",
    "ACTION_STATE_CANCELLED", "ACTION_STATE_TIMED_OUT",
    "ACTION_STATE_FAILED", "ACTION_STATE_PREEMPTED",
    "ACTION_STATE_ABORTED_BY_SAFETY",
}


class ModelRuntime(Protocol):
    """A provider adapter must return one JSON response for one observation."""

    name: str

    def propose(self, observation: dict) -> str:
        ...


class MockRuntime:
    """Wiring test only. It is not a flight policy or model baseline."""

    name = "mock-no-action"

    def propose(self, observation: dict) -> str:
        return '{"action":"none","arguments":{}}'


def _number(value, minimum, maximum):
    return (type(value) in (int, float) and math.isfinite(value)
            and minimum <= value <= maximum)


def validate_proposal(raw, allowed_actions=ALLOWED_ACTIONS):
    """Validate the initial, deliberately narrow Phase 11 action vocabulary."""
    if not isinstance(raw, str):
        # ValueError, not TypeError: the caller classifies every ValueError as
        # an invalid proposal and anything else as a runtime failure.
        raise ValueError("proposal must be JSON text")  # noqa: TRY004
    try:
        proposal = json.loads(raw)
    except (TypeError, ValueError) as error:
        raise ValueError("proposal is not one JSON object") from error
    if not isinstance(proposal, dict) or set(proposal) != {"action", "arguments"}:
        raise ValueError("proposal must contain only action and arguments")
    action = proposal["action"]
    arguments = proposal["arguments"]
    if not isinstance(action, str) or action not in allowed_actions:
        raise ValueError("action is not allowed")
    if not isinstance(arguments, dict):
        raise ValueError("arguments must be an object")  # noqa: TRY004
    if action == "takeoff":
        if set(arguments) != {"target_altitude_agl_m"} or not _number(
                arguments["target_altitude_agl_m"], 0.5, 10.0):
            raise ValueError("takeoff requires target_altitude_agl_m in [0.5, 10]")
    elif action == "hold":
        if set(arguments) not in (set(), {"duration_ms"}):
            raise ValueError("hold accepts only duration_ms")
        if "duration_ms" in arguments and (type(arguments["duration_ms"]) is not int
                                       or not 1 <= arguments["duration_ms"] <= 60_000):
            raise ValueError("hold duration_ms must be 1..60000")
    elif arguments:
        raise ValueError(action + " accepts no arguments")
    return proposal


def _curate(state, perception, previous_result, mission, event):
    """Exclude authority, identifiers, raw data, and the recorded next action."""
    return {
        "mission": mission,
        "source_event_seq": event["seq"],
        "observed_at_unix_ms": event["unix_ms"],
        "state": {
            key: state.get(key) for key in (
                "sequence", "observed_at_unix_ms", "flight_phase", "armed",
                "landed", "relative_altitude_m", "ground_speed_mps",
                "local_position_ned", "battery", "estimator", "mavlink",
                "manual_override_active", "geofence_breached")
            if key in state
        },
        "perception": {
            key: perception.get(key) for key in (
                "sequence", "observed_at_unix_ms", "overall",
                "local_map_available", "local_map_age_ms", "path_ahead_clear",
                "nearest_obstacle_distance_m", "nearest_obstacle_bearing_deg")
            if key in perception
        },
        "previous_result": previous_result,
        "allowed_actions": list(ALLOWED_ACTIONS),
    }


def _sha256(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def observe_episode(episode, runtime, output_root, timeout_ms=5000,
                    check_guardrails=True):
    """Replay a sealed episode at its recorded pre-action decision points."""
    episode = Path(episode)
    if not isinstance(timeout_ms, int) or timeout_ms <= 0:
        raise ValueError("timeout_ms must be positive")
    verified = replay(episode, check_guardrails=check_guardrails)
    manifest = json.loads((episode / "manifest.json").read_text())
    stream_path = episode / "events.jsonl"
    output_root = Path(output_root)
    output_root.mkdir(parents=True, exist_ok=True)
    output = output_root / (episode.name + "_" + uuid.uuid4().hex[:12])
    output.mkdir()
    decisions = output / "decisions.jsonl"
    counts = {"valid": 0, "invalid": 0, "timeout": 0, "error": 0}
    perception = {}
    previous_result = None
    proposals = 0
    with stream_path.open(encoding="utf-8") as source, decisions.open("x", encoding="utf-8") as target:
        for line in source:
            event = json.loads(line)
            kind = event["kind"]
            payload = event["payload"]
            if kind == "perception":
                perception = payload
            elif kind == "action_status" and payload.get("state") in TERMINAL:
                previous_result = {
                    "type": payload.get("type"), "state": payload["state"],
                    "reason_code": payload.get("reason_code"),
                }
            elif kind == "guardrail_validation":
                observation = _curate(
                    payload["state"], perception, previous_result,
                    manifest["mission"], event)
                started = time.monotonic_ns()
                raw = None
                proposal = None
                error = None
                try:
                    raw = runtime.propose(observation)
                    elapsed_ms = (time.monotonic_ns() - started) / 1_000_000
                    if elapsed_ms > timeout_ms:
                        status = "timeout"
                    else:
                        proposal = validate_proposal(raw)
                        status = "valid"
                except ValueError as failure:
                    elapsed_ms = (time.monotonic_ns() - started) / 1_000_000
                    status, error = "invalid", str(failure)
                # A model runtime may fail in any way; record it as an error
                # rather than letting one bad decision abort the replay.
                except Exception as failure:  # noqa: BLE001
                    elapsed_ms = (time.monotonic_ns() - started) / 1_000_000
                    status, error = "error", type(failure).__name__
                counts[status] += 1
                proposals += 1
                recorded_action = next(iter(payload["command"]))
                target.write(json.dumps({
                    "decision": proposals, "observation": observation,
                    "recorded_action": recorded_action,
                    "proposal": proposal,
                    "raw_response": raw if isinstance(raw, str) else None,
                    "status": status, "error": error,
                    "latency_ms": round(elapsed_ms, 3),
                    "executed": False,
                }, sort_keys=True) + "\n")
    summary = {
        "schema": SCHEMA, "mode": "observe", "executed_actions": 0,
        "source_episode": verified["episode_id"],
        "source_stream_sha256": _sha256(stream_path),
        "runtime": runtime.name, "timeout_ms": timeout_ms,
        "decision_points": proposals, "counts": counts,
        "decisions_sha256": _sha256(decisions),
    }
    (output / "summary.json").write_text(json.dumps(summary, indent=2) + "\n")
    return output, summary

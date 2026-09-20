"""Offline, observe-only DCM decision replay.

This module has no Drone API client, gRPC channel or MAVLink imports, and
never opens a path to an aircraft. A runtime receives only a curated snapshot
and returns one JSON proposal, which is syntax-checked and recorded, never
executed. It does shell out to `icarus-check-guardrail`, a standalone binary
that answers one offline question, whether a proposal would be accepted by
the real safety policy; that is a subprocess call to check a hypothesis, not
a control path.

What a model may see, say and be asked is defined in `contract`, not here.
"""

import hashlib
import json
import time
import uuid
from pathlib import Path

from python.dataset_tools.replay import replay
from python.dcm.contract import (
    ALLOWED_ACTIONS,
    CONTRACT_VERSION,
    CONTRACT_VERSION_HISTORY,
    FRESHNESS_LIMITS,
    is_flight_action,
    PROMPT_VERSION,
    VOCABULARY_VERSION,
    DeadlineExceeded,
    ModelRuntime,
    RuntimeDescriptor,
    assess_freshness,
    curate,
    render_prompt,
    validate_proposal,
)
from python.dcm.guardrail_check import GuardrailUnavailable, check_proposal

__all__ = [
    "ALLOWED_ACTIONS", "MockRuntime", "ModelRuntime", "RuntimeDescriptor",
    "observe_episode", "render_prompt", "validate_proposal",
]

SCHEMA = "icarus.dcm.observe.v1"
TERMINAL = {
    "ACTION_STATE_REJECTED", "ACTION_STATE_SUCCEEDED",
    "ACTION_STATE_CANCELLED", "ACTION_STATE_TIMED_OUT",
    "ACTION_STATE_FAILED", "ACTION_STATE_PREEMPTED",
    "ACTION_STATE_ABORTED_BY_SAFETY",
}


class MockRuntime:
    """Wiring test only. It is not a flight policy or model baseline."""

    name = "mock-no-action"

    def propose(self, observation: dict) -> str:
        return '{"action":"none","arguments":{}}'


def _action_name(action_type):
    """ACTION_TYPE_ARM -> arm, so history reads in the action vocabulary."""
    if not isinstance(action_type, str):
        return None
    return action_type.replace("ACTION_TYPE_", "").lower()


def _sha256(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def observe_episode(episode, runtime, output_root, timeout_ms=5000,
                    check_guardrails=True, descriptor=None,
                    include_history=False, include_elapsed=False,
                    check_proposal_guardrails=True):
    """Replay a sealed episode at its recorded pre-action decision points.

    `check_guardrails` (existing) replays the episode's *recorded* actions
    through the guardrails to confirm the stream is internally consistent.
    `check_proposal_guardrails` (new) is unrelated: it asks whether the
    *model's* proposal would itself be accepted by the real safety policy,
    which the contract's bounds check cannot know. Both can run independently.
    """
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
    counts = {"valid": 0, "invalid": 0, "timeout": 0, "error": 0, "stale": 0}
    guardrail_counts = {"would_execute": 0, "would_reject": 0, "unchecked": 0}
    perception = {}
    previous_result = None
    proposals = 0
    # Completed actions only. A terminal status is recorded after its own
    # guardrail validation, so the pending action can never appear here.
    completed = [] if include_history else None
    episode_started_ms = None
    with stream_path.open(encoding="utf-8") as source, decisions.open("x", encoding="utf-8") as target:
        for line in source:
            event = json.loads(line)
            kind = event["kind"]
            payload = event["payload"]
            if kind == "episode_start":
                episode_started_ms = event["unix_ms"]
            elif kind == "perception":
                perception = payload
            elif kind == "action_status" and payload.get("state") in TERMINAL:
                previous_result = {
                    "type": payload.get("type"), "state": payload["state"],
                    "reason_code": payload.get("reason_code"),
                }
                if completed is not None:
                    completed.append({
                        "action": _action_name(payload.get("type")),
                        "outcome": payload["state"].replace("ACTION_STATE_", ""),
                    })
            elif kind == "guardrail_validation":
                elapsed = None
                if include_elapsed and episode_started_ms is not None:
                    elapsed = event["unix_ms"] - episode_started_ms
                observation = curate(
                    payload["state"], perception, previous_result,
                    manifest["mission"], event, history=completed,
                    elapsed_ms=elapsed)
                raw = None
                proposal = None
                error = None
                stale = assess_freshness(observation)
                if stale:
                    # Refuse before asking. A model given stale state would
                    # answer confidently about a situation that no longer
                    # holds, and that answer would be recorded as valid.
                    status, error, elapsed_ms = "stale", stale, 0.0
                else:
                    started = time.monotonic_ns()
                    try:
                        raw = runtime.propose(observation)
                        elapsed_ms = (time.monotonic_ns() - started) / 1_000_000
                        if elapsed_ms > timeout_ms:
                            status = "timeout"
                        else:
                            proposal = validate_proposal(raw)
                            status = "valid"
                    except DeadlineExceeded as failure:
                        # The adapter enforced its own deadline and abandoned
                        # the request; trust it over the elapsed-time check.
                        elapsed_ms = (time.monotonic_ns() - started) / 1_000_000
                        status, error = "timeout", str(failure)
                    except ValueError as failure:
                        elapsed_ms = (time.monotonic_ns() - started) / 1_000_000
                        status, error = "invalid", str(failure)
                    # A model runtime may fail in any way; record it as an
                    # error rather than letting one bad decision abort the
                    # replay.
                    except Exception as failure:  # noqa: BLE001
                        elapsed_ms = (time.monotonic_ns() - started) / 1_000_000
                        status, error = "error", type(failure).__name__
                counts[status] += 1
                proposals += 1

                guardrail = None
                # Semantic tools and "none" never become flight commands, so
                # they have no guardrail verdict; this is not an unchecked
                # flight proposal.
                if (status == "valid" and check_proposal_guardrails
                        and is_flight_action(proposal["action"])):
                    # A proposal can satisfy the contract's shape and bounds
                    # and still be something the flight safety policy would
                    # refuse, such as a takeoff while disarmed. Checked
                    # against the full recorded state, not the curated
                    # observation: curation deliberately hides fields such as
                    # position and authority that the guardrails need.
                    try:
                        guardrail = check_proposal(
                            proposal["action"], proposal["arguments"],
                            payload["state"])
                        guardrail_counts[
                            "would_execute" if guardrail["would_execute"]
                            else "would_reject"] += 1
                    except GuardrailUnavailable as unavailable:
                        guardrail = {"checked": False,
                                    "reason": str(unavailable)}
                        guardrail_counts["unchecked"] += 1

                # Some runtimes (llama.cpp) report their own generation speed
                # and token counts per decision; record rather than recompute.
                runtime_stats = getattr(runtime, "last_stats", None)
                recorded_action = next(iter(payload["command"]))
                target.write(json.dumps({
                    "decision": proposals, "observation": observation,
                    "recorded_action": recorded_action,
                    "proposal": proposal,
                    "raw_response": raw if isinstance(raw, str) else None,
                    "status": status, "error": error,
                    "latency_ms": round(elapsed_ms, 3),
                    "guardrail": guardrail,
                    "runtime_stats": runtime_stats,
                    "executed": False,
                }, sort_keys=True) + "\n")
    summary = {
        "schema": SCHEMA, "mode": "observe", "executed_actions": 0,
        "source_episode": verified["episode_id"],
        "source_stream_sha256": _sha256(stream_path),
        "runtime": runtime.name, "timeout_ms": timeout_ms,
        "decision_points": proposals, "counts": counts,
        "guardrail_counts": guardrail_counts,
        "contract": {
            "contract_version": (CONTRACT_VERSION_HISTORY if include_history
                                 else CONTRACT_VERSION),
            "vocabulary_version": VOCABULARY_VERSION,
            # The runtime renders the prompt, so its descriptor is the
            # authority on which variant was actually used.
            "prompt_version": (descriptor.prompt_version if descriptor
                               else PROMPT_VERSION),
            "allowed_actions": list(ALLOWED_ACTIONS),
            "freshness_limits": dict(FRESHNESS_LIMITS),
        },
        "model": descriptor.as_record() if descriptor else None,
        "decisions_sha256": _sha256(decisions),
    }
    (output / "summary.json").write_text(json.dumps(summary, indent=2) + "\n")
    return output, summary

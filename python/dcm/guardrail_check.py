"""Check a model proposal against the real flight safety policy, offline.

The contract's `validate_proposal` only checks shape and argument bounds. It
cannot know that a `takeoff` proposed while disarmed is unsafe, or that a
`goto` is unreachable given the current geofence state, because that is the
guardrails' job, not the contract's. This module asks the actual guardrails
binary the same question the Drone API asks on every live command, so
"valid" and "would actually be allowed to execute" stop being conflated in
evaluation results.

It shells out to `icarus-check-guardrail` rather than reimplementing the
policy in Python, for the same reason the C++ safety supervisor exists at
all: the flight safety decision must have exactly one implementation.

The check runs against the full recorded state, not the curated observation
the model was shown. Curation deliberately hides fields such as position and
authority from the model; the guardrails need them to render a real verdict.
"""

import json
import subprocess
import uuid
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
DEFAULT_BINARY = ROOT / "build/phase8/icarus-check-guardrail"
DEFAULT_POLICY = ROOT / "config/safety/v1.yaml"


class GuardrailUnavailable(Exception):
    """The checker binary or policy is not built/present.

    Deliberately distinct from a guardrail rejection: this means the question
    could not be asked at all, not that the answer was no.
    """


def _synthetic_context(action_pb2, state, now_unix_ms):
    """A context built to test the command, not the paperwork around it.

    minimum_state_sequence is left at 0 to skip the "server has not observed
    this state" check, and now_unix_ms is taken as the state's own timestamp
    so the state-age check passes with zero elapsed time. Freshness relative
    to the moment of decision is the contract's job (`assess_freshness`); this
    checks only whether the action itself would be accepted against the state
    it was proposed against.
    """
    return action_pb2.CommandContext(
        request_id=uuid.uuid4().hex,
        idempotency_key=uuid.uuid4().hex,
        vehicle_id=state.get("vehicle_id", "icarus-01"),
        client_id="dcm-guardrail-check",
        issued_at_unix_ms=now_unix_ms,
        expires_at_unix_ms=now_unix_ms + 5_000,
        minimum_state_sequence=0,
    )


def check_proposal(action, arguments, full_state, binary=None, policy=None):
    """Return {"checked": True, "would_execute": bool, "reason_code", ...}.

    `full_state` is the recorded or live DroneState as a dict (protobuf JSON,
    preserving_proto_field_name or camelCase both parse), not the curated
    observation. Raises GuardrailUnavailable if the binary or policy file is
    missing, so a caller can record "unchecked" rather than a false verdict.
    """
    from icarus.v1 import action_pb2

    from python.dcm.actions import build_command

    binary = Path(binary or DEFAULT_BINARY)
    policy = Path(policy or DEFAULT_POLICY)
    if not binary.is_file():
        raise GuardrailUnavailable(
            f"{binary} missing; build it with "
            "cmake --build build/phase8 --target icarus-check-guardrail")
    if not policy.is_file():
        raise GuardrailUnavailable(f"safety policy missing at {policy}")

    now = int(full_state.get("observed_at_unix_ms") or 0)
    if isinstance(full_state.get("observed_at_unix_ms"), str):
        now = int(full_state["observed_at_unix_ms"])
    context = _synthetic_context(action_pb2, full_state, now)
    command = build_command(action_pb2, action, arguments, context)

    from google.protobuf.json_format import MessageToDict
    request = {
        "command": MessageToDict(command, preserving_proto_field_name=True),
        "state": full_state,
        "authorized": True,
        "now_unix_ms": now,
    }
    try:
        completed = subprocess.run(
            [str(binary), str(policy)], input=json.dumps(request),
            capture_output=True, text=True, timeout=10, check=False)
    except (OSError, subprocess.TimeoutExpired) as failure:
        raise GuardrailUnavailable(str(failure)) from failure
    if completed.returncode != 0:
        raise GuardrailUnavailable(
            f"icarus-check-guardrail exited {completed.returncode}: "
            f"{completed.stderr.strip()}")
    result = json.loads(completed.stdout)
    return {
        "checked": True,
        "would_execute": bool(result.get("valid", False)),
        "reason_code": result.get("reasonCode", result.get("reason_code")),
        "message": result.get("message"),
    }

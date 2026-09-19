"""Provider-neutral model contract for the DCM.

Everything a model is shown, and everything it is allowed to say back, is
declared here behind explicit version strings. A provider adapter implements
`ModelRuntime` and nothing else: it never decides what is observable, what is
allowed, or what counts as fresh enough to act on.

Three rules this module exists to hold:

* The recorded next action is comparison data. It is never model input.
* The action vocabulary is data, not control flow. Widening it is a versioned
  edit to one table rather than new branches in a validator, and the prompt is
  generated from that same table so the two cannot drift apart.
* A stale observation is refused before the model is asked, not after.

This module has no Drone API, MAVLink or simulator imports, and performs no
I/O except reading a runtime manifest when explicitly asked.
"""

import dataclasses
import hashlib
import json
import math
from pathlib import Path
from typing import Protocol

CONTRACT_VERSION = "dcm-contract-v1"
PROMPT_VERSION = "dcm-prompt-v1"
VOCABULARY_VERSION = "dcm-actions-v1"

# The deliberately narrow first vocabulary. It is much smaller than the Drone
# API on purpose: a model may only ask for what has been explicitly modelled
# here, and every argument carries its own closed bounds.
ACTIONS = {
    "none": {},
    "arm": {},
    "takeoff": {
        "target_altitude_agl_m": {
            "kind": "number", "required": True, "minimum": 0.5, "maximum": 10.0,
        },
    },
    "hold": {
        "duration_ms": {
            "kind": "integer", "required": False, "minimum": 1, "maximum": 60_000,
        },
    },
    "return_home": {},
    "land": {},
}

ALLOWED_ACTIONS = tuple(ACTIONS)

# Observation age limits. The perception limit matches the ObstacleMap expiry
# in perception/obstacle_map/obstacle_map.hpp; if that default changes, this
# must change with it.
FRESHNESS_LIMITS = {
    "state_age_ms": 1_000,
    "perception_age_ms": 750,
}

# State and perception fields the model may see. Anything absent from these
# tuples never reaches a model: authority leases, request identifiers, vehicle
# identifiers and raw sensor data are all excluded by omission.
STATE_FIELDS = (
    "sequence", "observed_at_unix_ms", "flight_phase", "armed", "landed",
    "relative_altitude_m", "ground_speed_mps", "local_position_ned",
    "battery", "estimator", "mavlink", "manual_override_active",
    "geofence_breached",
)
PERCEPTION_FIELDS = (
    "sequence", "observed_at_unix_ms", "overall", "local_map_available",
    "local_map_age_ms", "path_ahead_clear", "nearest_obstacle_distance_m",
    "nearest_obstacle_bearing_deg",
)


class ModelRuntime(Protocol):
    """A provider adapter returns one JSON response for one observation.

    Implementations must enforce their own hard deadline at the process
    boundary. Returning late is not sufficient: a runtime that hangs must be
    terminated, not merely reported.
    """

    name: str

    def propose(self, observation: dict) -> str:
        ...


class StaleObservationError(ValueError):
    """The observation was too old to put in front of a model."""


class DeadlineExceeded(Exception):
    """A runtime did not answer within its deadline and was abandoned.

    Deliberately not a ValueError: a missed deadline is a runtime failure, not
    a malformed proposal, and the two are counted separately.
    """


def _describe_bounds(spec):
    kind = "integer" if spec["kind"] == "integer" else "number"
    return (f"{kind} in [{spec['minimum']}, {spec['maximum']}]"
            + ("" if spec["required"] else ", optional"))


def _check_value(value, spec):
    """Return a problem description, or None when the value is acceptable."""
    if spec["kind"] == "integer":
        # `type(...) is not int` rather than isinstance: bool is a subclass of
        # int, and True must not be accepted as a duration.
        if type(value) is not int:
            return "must be an integer"
    elif type(value) not in (int, float) or not math.isfinite(value):
        return "must be a finite number"
    if not spec["minimum"] <= value <= spec["maximum"]:
        return f"must be in [{spec['minimum']}, {spec['maximum']}]"
    return None


def validate_proposal(raw, actions=ACTIONS):
    """Parse and bound-check one model response.

    Every rejection raises ValueError. Callers classify ValueError as an
    invalid proposal and anything else as a runtime failure, so this function
    must not raise TypeError for malformed input.
    """
    if not isinstance(raw, str):
        raise ValueError("proposal must be JSON text")  # noqa: TRY004
    try:
        proposal = json.loads(raw)
    except (TypeError, ValueError) as error:
        raise ValueError("proposal is not one JSON object") from error
    if not isinstance(proposal, dict) or set(proposal) != {"action", "arguments"}:
        raise ValueError("proposal must contain only action and arguments")
    action = proposal["action"]
    arguments = proposal["arguments"]
    if not isinstance(action, str) or action not in actions:
        raise ValueError("action is not allowed")
    if not isinstance(arguments, dict):
        raise ValueError("arguments must be an object")  # noqa: TRY004
    specification = actions[action]
    unexpected = sorted(set(arguments) - set(specification))
    if unexpected:
        raise ValueError(action + " does not accept " + ", ".join(unexpected))
    for name, spec in specification.items():
        if name not in arguments:
            if spec["required"]:
                raise ValueError(action + " requires " + name)
            continue
        problem = _check_value(arguments[name], spec)
        if problem:
            raise ValueError(name + " " + problem)
    return proposal


def _as_epoch_ms(value):
    """Coerce a millisecond timestamp, or return None if it is not one.

    Protobuf's canonical JSON mapping renders int64 as a *string* while int32
    stays a number, so recorded episodes carry observed_at_unix_ms as
    "1789827075223" and local_map_age_ms as 28. Both must be accepted.
    """
    if type(value) is int:
        return value
    if isinstance(value, str) and value.lstrip("-").isdigit():
        return int(value)
    return None


def assess_freshness(observation, limits=FRESHNESS_LIMITS):
    """Return a reason string when an observation is too stale to act on.

    A missing timestamp is treated as stale rather than fresh. Absent evidence
    of freshness is not evidence of freshness, and the safe default is to
    decline to ask the model.
    """
    now = _as_epoch_ms(observation.get("observed_at_unix_ms"))
    if now is None:
        return "observation has no timestamp"
    state = observation.get("state") or {}
    observed = _as_epoch_ms(state.get("observed_at_unix_ms"))
    if observed is None:
        return "state has no timestamp"
    age = now - observed
    if age > limits["state_age_ms"]:
        return f"state is {age} ms old, limit {limits['state_age_ms']} ms"
    perception = observation.get("perception") or {}
    map_age = _as_epoch_ms(perception.get("local_map_age_ms"))
    if perception and map_age is None:
        return "perception has no map age"
    if map_age is not None and map_age > limits["perception_age_ms"]:
        return (f"perception map is {map_age} ms old, "
                f"limit {limits['perception_age_ms']} ms")
    return None


def curate(state, perception, previous_result, mission, event, actions=ACTIONS):
    """Build the bounded observation a model is allowed to see.

    The recorded next action is deliberately not a parameter. It is attached to
    the report by the caller only after the runtime has answered.
    """
    return {
        "contract_version": CONTRACT_VERSION,
        "vocabulary_version": VOCABULARY_VERSION,
        "mission": mission,
        "source_event_seq": event["seq"],
        "observed_at_unix_ms": event["unix_ms"],
        "state": {k: state.get(k) for k in STATE_FIELDS if k in state},
        "perception": {k: perception.get(k) for k in PERCEPTION_FIELDS
                       if k in perception},
        "previous_result": previous_result,
        "allowed_actions": list(actions),
    }


SYSTEM_PROMPT = """\
You are a flight decision component for an uncrewed aircraft. You do not fly \
the aircraft; you propose one action, which independent safety software then \
validates and may reject.

Reply with exactly one JSON object and nothing else. No prose, no explanation, \
no code fences. The object has exactly two keys: "action" and "arguments".

Allowed actions:
{vocabulary}

Rules:
- Choose exactly one action from the list above.
- "arguments" is always an object. Use {{}} when the action takes no arguments.
- Never invent an action, an argument, or a unit.
- If no action is appropriate or the situation is unclear, reply with \
{{"action":"none","arguments":{{}}}}.\
"""


def render_vocabulary(actions=ACTIONS):
    """Describe the action table in prompt form, generated from the table."""
    lines = []
    for action, specification in actions.items():
        if not specification:
            lines.append(f'- "{action}": no arguments')
            continue
        described = ", ".join(
            f"{name} ({_describe_bounds(spec)})"
            for name, spec in specification.items())
        lines.append(f'- "{action}": {described}')
    return "\n".join(lines)


def render_prompt(observation, actions=ACTIONS):
    """Return the versioned system and user text for one decision.

    The vocabulary shown to the model is generated from the same table the
    validator enforces, so a model cannot be invited to emit something that
    would then be rejected.
    """
    return {
        "prompt_version": PROMPT_VERSION,
        "system": SYSTEM_PROMPT.format(vocabulary=render_vocabulary(actions)),
        "user": ("Current situation:\n"
                 + json.dumps(observation, indent=2, sort_keys=True)
                 + "\n\nRespond with one JSON object."),
    }


@dataclasses.dataclass(frozen=True)
class RuntimeDescriptor:
    """Everything that must be recorded to make a decision reproducible."""

    model_id: str
    family: str
    quantization: str
    artifact_path: str
    artifact_sha256: str
    runtime: str
    runtime_revision: str
    context_length: int
    temperature: float
    deadline_ms: int
    prompt_version: str = PROMPT_VERSION
    contract_version: str = CONTRACT_VERSION
    vocabulary_version: str = VOCABULARY_VERSION

    @classmethod
    def from_manifest(cls, manifest_path, role="primary", **overrides):
        """Build a descriptor from a setup-model-runtime manifest."""
        manifest = json.loads(Path(manifest_path).read_text())
        for artifact in manifest["artifacts"]:
            if artifact["role"] == role:
                break
        else:
            raise ValueError("no artifact with role " + role)
        fields = {
            "model_id": Path(artifact["path"]).stem,
            "family": "qwen",
            "quantization": artifact["quantization"],
            "artifact_path": artifact["path"],
            "artifact_sha256": artifact["sha256"],
            "runtime": "llama_cpp",
            "runtime_revision": manifest["llama_cpp_revision"],
            "context_length": 8192,
            "temperature": 0.0,
            "deadline_ms": 5_000,
        }
        fields.update(overrides)
        return cls(**fields)

    def verify_artifact(self):
        """Re-hash the weights so a swapped or truncated file fails loudly."""
        path = Path(self.artifact_path)
        if not path.is_file():
            raise FileNotFoundError("model artifact missing: " + str(path))
        digest = hashlib.sha256()
        with path.open("rb") as handle:
            for block in iter(lambda: handle.read(1024 * 1024), b""):
                digest.update(block)
        actual = digest.hexdigest()
        if actual != self.artifact_sha256:
            raise ValueError(
                f"model artifact checksum mismatch for {path}: "
                f"expected {self.artifact_sha256}, found {actual}")
        return actual

    def as_record(self):
        """The provenance block written into every observe report."""
        return dataclasses.asdict(self)

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

import json
import math
from typing import Protocol

from python.dcm.runtime_config import (
    DeadlineExceeded as DeadlineExceeded,  # noqa: PLC0414 - offline compatibility export
)
from python.dcm.runtime_config import (
    RuntimeDescriptor as RuntimeDescriptor,  # noqa: PLC0414 - offline compatibility export
)

# Two observation variants, selected by configuration so they can be compared
# under identical conditions. v1 shows only the most recent action result; v2
# adds the sequence of actions already completed. The version travels with
# every report so a result can never be read against the wrong contract.
CONTRACT_VERSION = "dcm-contract-v1"
CONTRACT_VERSION_HISTORY = "dcm-contract-v2-history"
# V3 is the compact default context. Existing V1/V2 reports remain comparable
# and their prompt/response shapes are retained for historical evaluation.
CONTRACT_VERSION_CONTEXT = "dcm-contract-v3-compact"

# How many completed actions the model may see. Bounded so a long mission
# cannot crowd out the current state.
HISTORY_LIMIT = 12
# v2 grounds every decision in the current physical aircraft state rather than
# presenting the payload as an abstract JSON exercise.  It is a model-visible
# change and must therefore travel with every report.
PROMPT_VERSION = "dcm-prompt-v6-mission-evidence"
PROMPT_VERSION_CONTEXT = "dcm-prompt-v11-plan-once-visual-schedule"
# The optional ending variant adds extra terminal emphasis above the base v2
# embodiment guidance. It remains separately versioned for fair comparison.
PROMPT_VERSION_ENDING = "dcm-prompt-v3-ending"
# v2 adds goto and orbit. The version changes because a report scored
# against a different action set is not comparable. The optional
# target-relative arguments (goto face_point, orbit face_center/viewpoints,
# minimum_clearance_m) leave the action set unchanged and so share v5.
VOCABULARY_VERSION = "dcm-actions-v5-relative-yaw"
REASON_CODES = frozenset({
    "MISSION_OBJECTIVE_PENDING",
    "STATE_TRANSITION_REQUIRED",
    "EVIDENCE_REQUIRED",
    "SAFETY_HOLD",
    "RECOVERY_REQUIRED",
    "MISSION_COMPLETE",
    "NO_SAFE_ACTION",
})

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
        "maximum_climb_rate_mps": {
            "kind": "number", "required": False, "minimum": 0.3, "maximum": 2.0,
            "description": "requested climb-rate limit; use the live BODY constraint for slowly/quickly",
        },
    },
    "hold": {
        "duration_ms": {
            "kind": "integer", "required": False, "minimum": 1, "maximum": 60_000,
        },
    },
    "return_home": {},
    "land": {},
    # Navigation coordinate bounds describe displacements from measured HOME,
    # not absolute EKF coordinates. Compiled/local targets use
    # validate_local_proposal to apply these identical limits after rebasing.
    # Navigation actions. Bounds are deliberately tighter than the safety
    # policy in config/safety/v1.yaml (30 m ceiling, 100 m from home, 2.5 m
    # minimum clearance) so the contract refuses before the guardrails have
    # to. The furthest reachable point is sqrt(50^2 + 50^2) = 70.7 m from
    # home, and an orbit at the corner of its range reaches 71.6 m. Targets
    # computed in flight from perception are further held to the policy's
    # 30 m runtime fence by the guardrails (actions.build_request
    # runtime_target=True). minimum_clearance_m can only tighten the policy's
    # 2.0 m live LiDAR floor, so its minimum is that floor.
    "goto": {
        "north_m": {
            "kind": "number", "required": True, "minimum": -50.0, "maximum": 50.0,
        },
        "east_m": {
            "kind": "number", "required": True, "minimum": -50.0, "maximum": 50.0,
        },
        "altitude_agl_m": {
            "kind": "number", "required": True, "minimum": 0.5, "maximum": 25.0,
        },
        "acceptance_radius_m": {
            "kind": "number", "required": False, "minimum": 0.000001, "maximum": 10.0,
            "description": "position completion tolerance in metres; semantic navigation derives it from measured displacement",
        },
        "heading_deg": {
            "kind": "number", "required": False, "minimum": 0.0, "maximum": 360.0,
        },
        "face_point": {
            "kind": "local_point", "required": False, "minimum": -100.0,
            "maximum": 100.0,
            "description": "on arrival yaw to look at this point; not with heading_deg",
        },
        "maximum_ground_speed_mps": {
            "kind": "number", "required": False, "minimum": 0.5, "maximum": 3.0,
            "description": "horizontal navigation speed target (historical field name); actual speed varies during acceleration/settling",
        },
        "minimum_clearance_m": {
            "kind": "number", "required": False, "minimum": 2.0, "maximum": 10.0,
            "description": "stricter live obstacle clearance",
        },
    },
    "orbit": {
        "center_north_m": {
            "kind": "number", "required": True, "minimum": -40.0, "maximum": 40.0,
        },
        "center_east_m": {
            "kind": "number", "required": True, "minimum": -40.0, "maximum": 40.0,
        },
        "radius_m": {
            "kind": "number", "required": True, "minimum": 3.0, "maximum": 15.0,
        },
        "altitude_agl_m": {
            "kind": "number", "required": True, "minimum": 0.5, "maximum": 25.0,
        },
        "revolutions": {
            "kind": "number", "required": False, "minimum": 0.25, "maximum": 3.0,
        },
        "clockwise": {
            "kind": "boolean", "required": False,
            "description": "orbit direction seen from above; true=clockwise (default), false=counter-clockwise",
        },
        "face_center": {
            "kind": "boolean", "required": False,
            "description": "true keeps the camera on the centre at every waypoint",
        },
        "viewpoints": {
            "kind": "integer", "required": False, "minimum": 2, "maximum": 12,
            "description": "stop and hold at this many evenly spaced points; revolutions <= 1",
        },
        "viewpoint_dwell_ms": {
            "kind": "integer", "required": False, "minimum": 0,
            "maximum": 20_000,
            "description": "hold at each viewpoint; needs viewpoints",
        },
        "maximum_ground_speed_mps": {
            "kind": "number", "required": False, "minimum": 0.5, "maximum": 3.0,
            "description": "horizontal navigation speed setpoint; actual speed may be lower while turning or settling",
        },
        "minimum_clearance_m": {
            "kind": "number", "required": False, "minimum": 2.0, "maximum": 10.0,
            "description": "stricter live obstacle clearance",
        },
    },
    # Relative rather than absolute yaw is essential for scans: a 360-degree
    # turn begins and ends at the same heading, so an absolute-heading command
    # cannot demonstrate that the rotation physically happened.
    "yaw": {
        "angle_deg": {
            "kind": "number", "required": True, "minimum": -360.0,
            "maximum": 360.0,
            "description": "relative yaw; positive=clockwise/right, negative=counter-clockwise/left",
        },
        "rate_deg_s": {
            "kind": "number", "required": False, "minimum": 5.0,
            "maximum": 60.0,
            "description": "bounded yaw rate in degrees per second",
        },
    },
    # Semantic analysis is not a flight command. It is an explicit request to
    # the pinned vision boundary and cannot reach MAVLink or the action RPC.
    "detect": {
        "classes": {
            "kind": "class_list", "required": True, "minimum_items": 1,
            "maximum_items": 16, "maximum_item_length": 64,
        },
        "window_s": {
            "kind": "number", "required": False, "minimum": 0.2,
            "maximum": 30.0,
            "description": "camera observation window; repeated scan steps use a short window",
        },
    },
    # Qualitative visual interpretation is a separate inspection boundary. It
    # accepts a short question, never a command, coordinate, or flight target.
    # Its executor has no Drone API or MAVLink access.
    "inspect_scene": {
        "question": {
            "kind": "text", "required": True, "minimum_length": 1,
            "maximum_length": 240,
        },
    },
    # This is evidence gathering only. It is deliberately separate from a
    # land command: a DCM can inspect a calibrated terrain source but cannot
    # turn that assessment into a flight action or bypass landing guardrails.
    "assess_landing_zone": {},
}

ALLOWED_ACTIONS = tuple(ACTIONS)
FLIGHT_ACTIONS = frozenset(ACTIONS) - {
    "none", "detect", "inspect_scene", "assess_landing_zone",
}

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




def _describe_bounds(spec):
    if spec["kind"] == "local_point":
        return (f"object {{north_m, east_m}}, each a number in "
                f"[{spec['minimum']}, {spec['maximum']}]"
                + ("" if spec["required"] else ", optional"))
    if spec["kind"] == "class_list":
        return (f"array of {spec['minimum_items']}..{spec['maximum_items']} "
                f"class-name strings (each <= {spec['maximum_item_length']} chars)")
    if spec["kind"] == "text":
        return (f"text of {spec['minimum_length']}..{spec['maximum_length']} "
                "characters")
    if spec["kind"] == "boolean":
        return "true or false" + ("" if spec["required"] else ", optional")
    kind = "integer" if spec["kind"] == "integer" else "number"
    return (f"{kind} in [{spec['minimum']}, {spec['maximum']}]"
            + ("" if spec["required"] else ", optional"))


def _check_value(value, spec):
    """Return a problem description, or None when the value is acceptable."""
    if spec["kind"] == "class_list":
        if not isinstance(value, list):
            return "must be an array of class-name strings"
        if not spec["minimum_items"] <= len(value) <= spec["maximum_items"]:
            return (f"must contain {spec['minimum_items']}.."
                    f"{spec['maximum_items']} items")
        normalized = []
        for item in value:
            if not isinstance(item, str):
                return "must contain only strings"
            compact = " ".join(item.strip().lower().split())
            if not compact or len(compact) > spec["maximum_item_length"]:
                return "contains an empty or oversized class name"
            if item != compact:
                return "class names must be normalized lowercase strings"
            if compact in normalized:
                return "must not contain duplicate class names"
            normalized.append(compact)
        return None
    if spec["kind"] == "text":
        if not isinstance(value, str):
            return "must be text"
        compact = " ".join(value.strip().split())
        if value != compact:
            return "must not have leading, trailing, or repeated whitespace"
        if not spec["minimum_length"] <= len(value) <= spec["maximum_length"]:
            return (f"must contain {spec['minimum_length']}.."
                    f"{spec['maximum_length']} characters")
        return None
    if spec["kind"] == "boolean":
        # Exact type: 1/0 and "true" are not directions.
        return None if type(value) is bool else "must be true or false"
    if spec["kind"] == "local_point":
        if not isinstance(value, dict) or set(value) != {"north_m", "east_m"}:
            return "must be an object with exactly north_m and east_m"
        for axis in ("north_m", "east_m"):
            coordinate = value[axis]
            if type(coordinate) not in (int, float) or \
                    not math.isfinite(coordinate):
                return axis + " must be a finite number"
            if not spec["minimum"] <= coordinate <= spec["maximum"]:
                return (f"{axis} must be in "
                        f"[{spec['minimum']}, {spec['maximum']}]")
        return None
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


def validate_local_proposal(raw, home, actions=ACTIONS):
    """Validate EKF-local targets against the same *home-relative* envelope.

    Coordinate translation never changes a motion limit. The original local
    target is returned for transport; only a validation copy is rebased.
    """
    try:
        proposal = json.loads(raw)
    except (TypeError, ValueError):
        return validate_proposal(raw, actions)
    if not isinstance(proposal, dict) or proposal.get("action") not in {"goto", "orbit"}:
        return validate_proposal(raw, actions)
    arguments = proposal.get("arguments")
    if not isinstance(arguments, dict):
        return validate_proposal(raw, actions)
    if not isinstance(home, dict) or any(type(home.get(axis)) not in (int, float)
            or not math.isfinite(home[axis]) for axis in ("north_m", "east_m")):
        raise ValueError("local navigation bounds need a measured autopilot home")
    checked = json.loads(raw)
    destination = checked["arguments"]
    prefix = "center_" if proposal["action"] == "orbit" else ""
    def rebase(point, prefix=""):
        for axis in ("north_m", "east_m"):
            field = prefix + axis
            if field in point:
                value = point[field]
                if type(value) not in (int, float) or not math.isfinite(value):
                    raise ValueError(field + " must be a finite local coordinate")
                point[field] = value - home[axis]
    rebase(destination, prefix)
    if isinstance(destination.get("face_point"), dict):
        rebase(destination["face_point"])
    validate_proposal(json.dumps(checked), actions)
    return proposal


def is_flight_action(action):
    """Whether an action needs the C++ flight guardrail/executor path."""
    return action in FLIGHT_ACTIONS


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
    if action == "yaw" and abs(float(arguments.get("angle_deg", 0.0))) < 1.0:
        raise ValueError("yaw angle_deg magnitude must be at least 1 degree")
    _check_combinations(action, arguments)
    return proposal


# Matches guardrails::kMinimumFacePointDistanceM: closer than this, "look at
# that point from here" has no stable heading.
MINIMUM_FACE_POINT_DISTANCE_M = 0.5


def _check_combinations(action, arguments):
    """Refuse argument pairs that are each in bounds but conflict."""
    if action == "goto" and "face_point" in arguments:
        if "heading_deg" in arguments:
            raise ValueError("goto accepts heading_deg or face_point, not both")
        point = arguments["face_point"]
        if math.hypot(point["north_m"] - arguments["north_m"],
                      point["east_m"] - arguments["east_m"]) < \
                MINIMUM_FACE_POINT_DISTANCE_M:
            raise ValueError(
                "face_point must be at least "
                f"{MINIMUM_FACE_POINT_DISTANCE_M} m from the destination")
    if action == "orbit":
        if "viewpoints" in arguments and arguments.get("revolutions", 1.0) > 1.0:
            raise ValueError("orbit viewpoints need revolutions <= 1")
        if "viewpoint_dwell_ms" in arguments and "viewpoints" not in arguments:
            raise ValueError("viewpoint_dwell_ms needs viewpoints")


def validate_decision(raw, actions=ACTIONS, allow_reason_code=False):
    """Validate an action and an optional bounded audit reason code.

    V1/V2 remain byte-for-byte strict through :func:`validate_proposal`.
    Compact V3 adds one closed reason code, which is removed before the action
    enters any validator, guardrail, request builder, or executor.
    """
    if not allow_reason_code:
        return validate_proposal(raw, actions), None
    if not isinstance(raw, str):
        raise ValueError("decision must be JSON text")  # noqa: TRY004
    try:
        decision = json.loads(raw)
    except (TypeError, ValueError) as error:
        raise ValueError("decision is not one JSON object") from error
    if not isinstance(decision, dict) or set(decision) != {
            "action", "arguments", "reason_code"}:
        raise ValueError(
            "V3 decision must contain only action, arguments, and reason_code")
    reason_code = decision["reason_code"]
    if reason_code not in REASON_CODES:
        raise ValueError("reason_code is not allowed")
    action_text = json.dumps({"action": decision["action"],
                              "arguments": decision["arguments"]})
    return validate_proposal(action_text, actions), reason_code


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


def material_state_change(before, after, position_tolerance_m=1.0,
                          altitude_tolerance_m=0.75):
    """Return a reason when the state materially changed during inference."""
    first = {**(before.get("state_summary") or {}),
             **(before.get("state") or {})}
    second = {**(after.get("state_summary") or {}),
              **(after.get("state") or {})}
    for field in ("armed", "landed", "flight_phase", "manual_override_active",
                  "geofence_breached"):
        if field in first and field in second and first[field] != second[field]:
            return f"state changed during inference: {field}"
    a_alt, b_alt = first.get("relative_altitude_m"), second.get("relative_altitude_m")
    if type(a_alt) in (int, float) and type(b_alt) in (int, float) and \
            abs(a_alt - b_alt) > altitude_tolerance_m:
        return "altitude changed materially during inference"
    a_pos, b_pos = first.get("local_position_ned") or {}, second.get("local_position_ned") or {}
    axes = ("north_m", "east_m", "down_m")
    if all(type(a_pos.get(key)) in (int, float) and
           type(b_pos.get(key)) in (int, float) for key in axes):
        distance = math.sqrt(sum((a_pos[key] - b_pos[key]) ** 2 for key in axes))
        if distance > position_tolerance_m:
            return f"position changed {distance:.2f} m during inference"
    first_perception = before.get("perception_summary") or before.get("perception") or {}
    second_perception = after.get("perception_summary") or after.get("perception") or {}
    for field in ("overall", "path_ahead_clear"):
        if field in first_perception and field in second_perception and \
                first_perception[field] != second_perception[field]:
            return f"perception changed during inference: {field}"
    return None


def curate(state, perception, previous_result, mission, event, actions=ACTIONS,
           history=None, elapsed_ms=None):
    """Build the bounded observation a model is allowed to see.

    The recorded next action is deliberately not a parameter. It is attached to
    the report by the caller only after the runtime has answered.

    `history` is the sequence of actions already *completed*, never the pending
    one. Passing None selects the v1 observation, which showed only the most
    recent result; a model given that cannot tell whether a mission has just
    begun or is nearly over.
    """
    observation = {
        "contract_version": (CONTRACT_VERSION_HISTORY if history is not None
                             else CONTRACT_VERSION),
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
    if history is not None:
        observation["actions_completed"] = list(history)[-HISTORY_LIMIT:]
    if elapsed_ms is not None:
        # A weak progress signal: the episode records the mission name but no
        # target altitude or hover duration, so there is nothing to measure
        # completion against. Time since the mission began is what the stream
        # actually supports.
        observation["mission_elapsed_ms"] = elapsed_ms
    return observation


SYSTEM_PROMPT_LEGACY = """\
You are the decision-making component of a real multirotor aircraft in a \
physical or simulated 3-D environment. The state and perception data that \
follow describe its actual current situation, not a hypothetical test case. \
Every proposal has physical consequences for position, altitude, battery and \
collision risk. A mission is complete only when the aircraft is safely on the \
ground. You do not fly the aircraft; you propose one action, which independent \
safety software then validates and may reject.

Reply with exactly one JSON object and nothing else. No prose and no code \
fences. {response_contract}

Allowed actions:
{vocabulary}

Rules:
- Choose exactly one action from the list above.
- Follow the operator request and successful action history without inventing
  a target or geometry. You may choose a safer action when state or perception
  makes progress unsafe.
- When a reviewed mission macro is selected, use only its ``allowed_actions``;
  hold, return_home, land and none remain available as safety exits.
- "arguments" is always an object. Use {{}} when the action takes no arguments.
- Never invent an action, an argument, or a unit.
- Respect the present physical flight state, even if the mission text asks for
  something else: a disarmed aircraft on the ground must arm before takeoff;
  takeoff is only appropriate when it is armed and on the ground; an airborne
  aircraft must never arm or take off again. Navigation and landing are only
  appropriate after takeoff. Do not use a prohibited action merely because it
  appears in the allowed vocabulary.
- If the aircraft is airborne and safety/perception is uncertain, choose hold
  or return_home rather than another takeoff. If it is landed and a completed
  mission has no further safe task, choose none.
- Use ``detect`` for structured supported object classes and ``inspect_scene``
  for qualitative current-camera questions. Neither perception action grants
  permission to move the aircraft or supplies navigation geometry.
- The environment is unknown except for live state and live perception. Never
  invent or infer a waypoint, target, landmark, route, or object location.
- Before proposing ``return_home``, ``land`` or ``none``, inspect successful
  action history and complete every requested mandatory visual requirement.
- Honor the operator's explicit task order. If a mission says to orbit and
  then count people, complete the orbit before requesting ``detect``.
- If no action is appropriate or the situation is unclear, reply with \
{no_action}.{ending}\
"""


ENDING_GUIDANCE = """
- Terminal reminder: holding keeps the aircraft airborne and is not a way to \
end a mission."""


def render_vocabulary(actions=ACTIONS):
    """Describe the action table in prompt form, generated from the table."""
    lines = []
    for action, specification in actions.items():
        if not specification:
            lines.append(f'- "{action}": no arguments')
            continue
        described = ", ".join(
            f"{name} ({_describe_bounds(spec)}"
            + (f"; {spec['description']}" if spec.get("description") else "")
            + ")" for name, spec in specification.items())
        lines.append(f'- "{action}": {described}')
    return "\n".join(lines)


def render_prompt(observation, actions=ACTIONS, ending_guidance=False):
    """Historical offline action replay only; live missions use agent_context."""
    if observation.get("context_version") == "dcm-context-v3-compact":
        raise ValueError("The precompiled mission context has been retired")
    return {
        "prompt_version": PROMPT_VERSION_ENDING if ending_guidance else PROMPT_VERSION,
        "system": SYSTEM_PROMPT_LEGACY.format(
            vocabulary=render_vocabulary(actions),
            response_contract='The object has exactly two keys: "action" and "arguments".',
            no_action='{"action":"none","arguments":{}}',
            ending=ENDING_GUIDANCE if ending_guidance else ""),
        "user": "Current situation:\n" + json.dumps(observation, sort_keys=True, indent=2)
                + "\n\nRespond with one JSON object.",
    }

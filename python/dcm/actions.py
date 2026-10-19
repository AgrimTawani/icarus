"""Build a typed Drone API request from one validated proposal.

Shared by `fly.py`, which sends the result to the real Drone API, and
`guardrail_check.py`, which sends the same message to the real guardrails
offline. Both must build the identical request for the identical proposal;
splitting the builder out means there is only one place that can drift from
the contract, rather than two copies that quietly diverge.

Every branch here is reachable only for an action `contract.validate_proposal`
already accepted, and every argument has already been bound-checked. This
module translates; it does not decide.
"""

import math

ACTION_METHODS = {
    "arm": "Arm", "takeoff": "Takeoff", "hold": "Hold", "land": "Land",
    "return_home": "ReturnHome", "goto": "Goto", "orbit": "Orbit",
    "yaw": "Yaw",
}


# Default hold at each orbit viewpoint: long enough to settle a camera frame
# capture, short enough that "each side" stays a quick mission.
DEFAULT_VIEWPOINT_DWELL_MS = 3_000
# The executor's worst case to settle on one viewpoint before it times out.
VIEWPOINT_SETTLE_ALLOWANCE_S = 20


def navigation_acceptance_radius(distance_m, policy):
    """Bound completion tolerance by the measured commanded displacement.

    Scale ordinary moves without demanding sub-centimetre convergence for a
    near-stationary heading change. The configurable floor represents control
    resolution; it does not alter the requested destination.
    """
    maximum = float(policy.get("goto_acceptance_max_m", 1.5))
    minimum = float(policy.get("goto_acceptance_min_m", 0.1))
    fraction = float(policy.get("goto_acceptance_distance_fraction", 0.25))
    if not (math.isfinite(distance_m) and distance_m >= 0 and
            math.isfinite(maximum) and maximum > 0 and
            math.isfinite(minimum) and 0 < minimum <= maximum and
            math.isfinite(fraction) and 0 < fraction < 0.5):
        raise ValueError("invalid navigation acceptance policy or displacement")
    return max(minimum, min(maximum, distance_m * fraction))


def build_request(action_pb2, action, arguments, context, runtime_target=False):
    """Return (typed request message, a generous wait-timeout in seconds).

    ``runtime_target`` marks goto/orbit coordinates that were computed in
    flight from perception rather than declared in the plan; the guardrails
    hold those to the policy's tighter runtime geofence. It is set by the
    caller that computed them, never by a model argument.
    """
    keep_heading = action_pb2.HeadingPolicy(
        mode=action_pb2.HEADING_MODE_KEEP_CURRENT)
    # Only touched when requested, so a caller that never asks for a runtime
    # target keeps working against protos generated before it existed.
    source = ({"target_source": action_pb2.TARGET_SOURCE_RUNTIME_ESTIMATE}
              if runtime_target else {})
    if action == "arm":
        return action_pb2.ArmRequest(context=context), 55
    if action == "takeoff":
        return action_pb2.TakeoffRequest(
            context=context,
            target_altitude_agl_m=arguments["target_altitude_agl_m"],
            heading=keep_heading,
            limits=action_pb2.ActionLimits(
                maximum_climb_rate_mps=arguments.get(
                    "maximum_climb_rate_mps", 2.0),
                execution_timeout_ms=60_000),
        ), 70
    if action == "hold":
        duration = int(arguments.get("duration_ms", 5_000))
        return action_pb2.HoldRequest(
            context=context, duration_ms=duration, heading=keep_heading,
        ), duration // 1000 + 20
    if action == "land":
        return action_pb2.LandRequest(context=context), 100
    if action == "return_home":
        return action_pb2.ReturnHomeRequest(context=context), 120
    if action == "goto":
        heading = keep_heading
        if "heading_deg" in arguments:
            heading = action_pb2.HeadingPolicy(
                mode=action_pb2.HEADING_MODE_ABSOLUTE,
                heading_deg=arguments["heading_deg"])
        elif "face_point" in arguments:
            # Only the horizontal direction matters; the point takes the
            # destination altitude so it is a well-formed local position.
            heading = action_pb2.HeadingPolicy(
                mode=action_pb2.HEADING_MODE_FACE_POINT,
                face_point=local_position(action_pb2, {
                    **arguments["face_point"],
                    "altitude_agl_m": arguments["altitude_agl_m"]}))
        limits = {"maximum_ground_speed_mps": arguments.get(
                      "maximum_ground_speed_mps", 3.0),
                  "execution_timeout_ms": 120_000}
        # Sent only when asked for: the executor enforces max(policy floor,
        # request) as its live LiDAR floor, so a default here would silently
        # raise the 2.0 m floor. Planning keeps its 2.5 m envelope regardless.
        if "minimum_clearance_m" in arguments:
            limits["minimum_clearance_m"] = arguments["minimum_clearance_m"]
        return action_pb2.GotoRequest(
            context=context,
            destination=local_position(action_pb2, arguments),
            acceptance_radius_m=arguments.get("acceptance_radius_m", 1.5),
            heading=heading,
            limits=action_pb2.ActionLimits(**limits),
            **source,
        ), 130
    if action == "orbit":
        # Without face_center retain the entry heading; with it the executor
        # faces the centre from every waypoint (FACE_POINT
        # with no explicit point means the orbit centre).
        heading = (action_pb2.HeadingPolicy(
            mode=action_pb2.HEADING_MODE_FACE_POINT)
            if arguments.get("face_center") else keep_heading)
        extra = dict(source)
        # Small orbits are flown one short segment at a time, so allow for
        # extra revolutions rather than a flat three minutes.
        requested_speed = arguments.get("maximum_ground_speed_mps", 2.0)
        cruise_seconds = (2 * math.pi * arguments["radius_m"] *
                          arguments.get("revolutions", 1.0) / requested_speed)
        timeout = max(180, 60 + int(90 * arguments.get("revolutions", 1.0)),
                      60 + math.ceil(cruise_seconds * 1.75))
        if "viewpoints" in arguments:
            dwell = int(arguments.get("viewpoint_dwell_ms",
                                      DEFAULT_VIEWPOINT_DWELL_MS))
            extra.update(viewpoints=arguments["viewpoints"],
                         viewpoint_dwell_ms=dwell)
            timeout += arguments["viewpoints"] * (
                dwell // 1000 + 1 + VIEWPOINT_SETTLE_ALLOWANCE_S)
        if "minimum_clearance_m" in arguments:
            extra["limits"] = action_pb2.ActionLimits(
                minimum_clearance_m=arguments["minimum_clearance_m"])
        return action_pb2.OrbitRequest(
            context=context,
            center=local_position(action_pb2, arguments, prefix="center_"),
            radius_m=arguments["radius_m"],
            altitude_m=arguments["altitude_agl_m"],
            ground_speed_mps=arguments.get("maximum_ground_speed_mps", 2.0),
            clockwise=arguments.get("clockwise", True),
            revolutions=arguments.get("revolutions", 1.0),
            heading=heading,
            **extra,
        ), timeout
    if action == "yaw":
        angle = float(arguments["angle_deg"])
        rate = float(arguments.get("rate_deg_s", 30.0))
        timeout = int(abs(angle) / rate * 2.0 + 15.0)
        return action_pb2.YawRequest(
            context=context,
            relative_angle_deg=angle,
            rate_deg_s=rate,
        ), timeout
    raise ValueError("no dispatch for action " + action)


def local_position(action_pb2, arguments, prefix=""):
    from python.dcm._paths import ensure_generated_proto_on_path
    ensure_generated_proto_on_path()

    from icarus.v1 import state_pb2
    return state_pb2.Position(
        local_ned=state_pb2.LocalPositionNed(
            north_m=arguments[prefix + "north_m"],
            east_m=arguments[prefix + "east_m"],
            # Horizontal coordinates are EKF-local; altitude is explicitly
            # above home. Backend resolves the measured vertical-frame offset.
            altitude_above_home_m=arguments["altitude_agl_m"],
            # The guardrails reject a local position with an empty origin id
            # outright (REASON_CODE_INVALID_ARGUMENT). Every goto and orbit
            # proposal was failing this check silently until the offline
            # guardrail check caught it: "silently" because it never reached
            # the contract's own bounds check, which has no notion of
            # origin frames. "ekf-origin" is the identifier the MAVLink
            # gateway itself sets (cpp/mavlink_gateway/mavlink_gateway.cpp)
            # and what the Phase 8/9 acceptance clients already use.
            origin_id=arguments.get("origin_id", "ekf-origin"),
        ))


def build_command(action_pb2, action, arguments, context, runtime_target=False):
    """Wrap a request into the ActionCommand oneof the guardrails validate.

    The oneof field names match the contract's action names exactly (arm,
    takeoff, hold, land, return_home, goto, orbit, yaw), so no separate mapping
    table is needed here.
    """
    request, _ = build_request(action_pb2, action, arguments, context,
                               runtime_target=runtime_target)
    return action_pb2.ActionCommand(**{action: request})

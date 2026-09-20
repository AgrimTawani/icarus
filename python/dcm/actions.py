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

ACTION_METHODS = {
    "arm": "Arm", "takeoff": "Takeoff", "hold": "Hold", "land": "Land",
    "return_home": "ReturnHome", "goto": "Goto", "orbit": "Orbit",
}


def build_request(action_pb2, action, arguments, context):
    """Return (typed request message, a generous wait-timeout in seconds)."""
    keep_heading = action_pb2.HeadingPolicy(
        mode=action_pb2.HEADING_MODE_KEEP_CURRENT)
    if action == "arm":
        return action_pb2.ArmRequest(context=context), 55
    if action == "takeoff":
        return action_pb2.TakeoffRequest(
            context=context,
            target_altitude_agl_m=arguments["target_altitude_agl_m"],
            heading=keep_heading,
            limits=action_pb2.ActionLimits(
                maximum_climb_rate_mps=2, execution_timeout_ms=60_000),
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
        return action_pb2.GotoRequest(
            context=context,
            destination=local_position(action_pb2, arguments),
            acceptance_radius_m=1.5,
            heading=keep_heading,
            limits=action_pb2.ActionLimits(
                maximum_ground_speed_mps=3.0,
                minimum_clearance_m=2.5,
                execution_timeout_ms=120_000),
        ), 130
    if action == "orbit":
        return action_pb2.OrbitRequest(
            context=context,
            center=local_position(action_pb2, arguments, prefix="center_"),
            radius_m=arguments["radius_m"],
            altitude_m=arguments["altitude_agl_m"],
            ground_speed_mps=2.0,
            clockwise=True,
            revolutions=arguments.get("revolutions", 1.0),
            heading=keep_heading,
        ), 180
    raise ValueError("no dispatch for action " + action)


def local_position(action_pb2, arguments, prefix=""):
    from python.dcm._paths import ensure_generated_proto_on_path
    ensure_generated_proto_on_path()

    from icarus.v1 import state_pb2
    return state_pb2.Position(
        local_ned=state_pb2.LocalPositionNed(
            north_m=arguments[prefix + "north_m"],
            east_m=arguments[prefix + "east_m"],
            down_m=-arguments["altitude_agl_m"],
            # The guardrails reject a local position with an empty origin id
            # outright (REASON_CODE_INVALID_ARGUMENT). Every goto and orbit
            # proposal was failing this check silently until the offline
            # guardrail check caught it: "silently" because it never reached
            # the contract's own bounds check, which has no notion of
            # origin frames. "ekf-origin" is the identifier the MAVLink
            # gateway itself sets (cpp/mavlink_gateway/mavlink_gateway.cpp)
            # and what the Phase 8/9 acceptance clients already use.
            origin_id="ekf-origin",
        ))


def build_command(action_pb2, action, arguments, context):
    """Wrap a request into the ActionCommand oneof the guardrails validate.

    The oneof field names match the contract's action names exactly (arm,
    takeoff, hold, land, return_home, goto, orbit), so no separate mapping
    table is needed here.
    """
    request, _ = build_request(action_pb2, action, arguments, context)
    return action_pb2.ActionCommand(**{action: request})

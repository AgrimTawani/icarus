"""Conservative pose metadata for asynchronous camera capture.

Reception time is not exposure synchronization. Report the pose bracket and
its motion uncertainty rather than asserting that a late VLM saw current pose.
"""

import math


def bracket_pose(before, after):
    keys = ("local_position_ned", "attitude", "relative_altitude_m", "observed_at_unix_ms")
    pose = {key: before[key] for key in keys if key in before}
    a, b = before.get("local_position_ned"), after.get("local_position_ned")
    if not isinstance(a, dict) or not isinstance(b, dict):
        return pose, {"method": "unavailable", "position_uncertainty_m": None,
                      "heading_uncertainty_deg": None}
    distance = math.sqrt(sum((float(b.get(k, 0)) - float(a.get(k, 0))) ** 2
                             for k in ("north_m", "east_m", "down_m")))
    heading = abs((float((after.get("attitude") or {}).get("yaw_rad", 0)) -
                   float((before.get("attitude") or {}).get("yaw_rad", 0)) +
                   math.pi) % (2 * math.pi) - math.pi)
    return pose, {"method": "pre_capture_pose_with_bracket_motion_bound",
                  "position_uncertainty_m": round(distance, 3),
                  "heading_uncertainty_deg": round(math.degrees(heading), 2),
                  "exposure_synchronized": False}

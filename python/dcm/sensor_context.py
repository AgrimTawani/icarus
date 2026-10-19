"""Small measured sensor context. Reception is not evidence of EKF fusion."""

import math
import time


def sensor_summary(state, *, now_ms=None, max_age_ms=2000):
    now_ms = int(time.time() * 1000) if now_ms is None else now_ms
    raw = state.get("navigation_sensors") or {}
    result = {"height_reference": "above_home_not_ground",
              "optical_flow_fusion": "unknown"}
    for name, stamp, fields in (
        ("downward_range", "range", ("downward_range_m",)),
        ("optical_flow", "flow", ("flow_velocity_x_mps", "flow_velocity_y_mps",
                                   "flow_quality")),
        ("barometer", "pressure", ("pressure_hpa",)),
    ):
        received = int(raw.get(stamp + "_received_at_unix_ms", 0))
        age = now_ms - received
        valid = received > 0 and 0 <= age <= max_age_ms
        if name == "downward_range":
            valid = valid and raw.get("range_valid", False)
        values = {key: raw.get(key, 0) for key in fields}
        valid = valid and all(type(v) in (int, float) and math.isfinite(v)
                              for v in values.values())
        if name == "barometer":
            valid = valid and values["pressure_hpa"] > 0
        result[name] = {"available": bool(valid),
                        "age_ms": age if received else None}
        if valid:
            result[name].update(values)
    return result

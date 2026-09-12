#!/usr/bin/python3
"""Load and strictly validate Phase 5 scenario definitions."""

import json
import math
from pathlib import Path

from build_akshu_candidate import ROOT

SCENARIO_DIR = ROOT / "simulation/scenarios"
REQUIRED = {
    "version",
    "name",
    "world_profile",
    "environment_preset",
    "vehicle_profile",
    "initial_pose",
    "home",
    "seed",
    "wind",
    "obstacles",
    "sensor_profile",
    "sensor_fault_schedule",
    "gps_degradation",
    "communication",
    "battery",
    "mission",
    "maximum_duration_s",
    "success",
    "ground_truth",
}


def resolve_scenario(value):
    path = Path(value)
    if not path.suffix:
        path = SCENARIO_DIR / (value + ".json")
    elif not path.is_absolute():
        path = ROOT / path
    return path.resolve()


def _number(value, label, minimum=None):
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        raise TypeError(f"{label} must be numeric")
    if not math.isfinite(value) or (minimum is not None and value < minimum):
        raise ValueError(f"invalid {label}: {value}")


def load_scenario(value):
    path = resolve_scenario(value)
    if path.parent != SCENARIO_DIR.resolve() or path.suffix != ".json":
        raise ValueError("scenario must be a JSON file in simulation/scenarios")
    data = json.loads(path.read_text())
    missing, extra = REQUIRED - data.keys(), data.keys() - REQUIRED
    if missing or extra:
        raise ValueError(f"scenario keys missing={sorted(missing)} extra={sorted(extra)}")
    if data["version"] != 1 or data["name"] != path.stem:
        raise ValueError("scenario version must be 1 and name must match filename")
    if data["world_profile"] not in ("empty", "wind", "obstacles", "adverse"):
        raise ValueError("invalid world_profile")
    if data["environment_preset"] not in ("empty", "custom", "mixed_village"):
        raise ValueError("invalid environment_preset")
    if data["vehicle_profile"] != "akshu_compact_sitl":
        raise ValueError("unsupported vehicle_profile")
    if data["sensor_profile"] != "physical":
        raise ValueError("invalid sensor_profile")
    pose = data["initial_pose"]
    if set(pose) != {"x_m", "y_m", "z_m", "yaw_deg"}:
        raise ValueError("initial_pose must define x_m, y_m, z_m, yaw_deg")
    for key, item_value in pose.items():
        _number(item_value, "initial_pose." + key)
    home = data["home"]
    if set(home) != {"latitude_deg", "longitude_deg", "elevation_m", "heading_deg"}:
        raise ValueError("invalid home keys")
    for key, item_value in home.items():
        _number(item_value, "home." + key)
    if not isinstance(data["seed"], int) or isinstance(data["seed"], bool):
        raise TypeError("seed must be an integer")
    _number(data["maximum_duration_s"], "maximum_duration_s", 1)
    wind = data["wind"]
    expected_wind = {
        "enabled", "speed_m_s", "direction_deg", "rise_time_s",
        "gust_amplitude_fraction", "gust_period_s",
        "direction_swing_deg", "direction_period_s", "operational_limit_m_s",
    }
    if set(wind) != expected_wind:
        raise ValueError("invalid wind keys")
    if not isinstance(wind["enabled"], bool):
        raise TypeError("wind.enabled must be boolean")
    for key in expected_wind - {"enabled"}:
        _number(wind[key], "wind." + key, 0)
    if wind["enabled"] != (wind["speed_m_s"] > 0):
        raise ValueError("wind.enabled must agree with speed_m_s")
    if not isinstance(data["obstacles"], list):
        raise TypeError("obstacles must be a list")
    names = set()
    for item in data["obstacles"]:
        if item.get("name") in names or item.get("type") not in ("box", "wall", "building", "tree"):
            raise ValueError("obstacle names must be unique and types supported")
        names.add(item.get("name"))
        if set(item) - {"name", "type", "center_m", "size_m", "radius_m", "height_m", "canopy_radius_m", "canopy_collision"}:
            raise ValueError(f"unexpected obstacle keys in {item.get('name')}")
        if item["type"] == "tree":
            if len(item.get("center_m", [])) != 3 or item.get("canopy_collision") not in ("enabled", "disabled"):
                raise ValueError("tree requires center and explicit canopy collision policy")
            for key in ("radius_m", "height_m", "canopy_radius_m"):
                _number(item.get(key), item["name"] + "." + key, 0.001)
        else:
            if len(item.get("center_m", [])) != 3 or len(item.get("size_m", [])) != 3:
                raise ValueError("box-like obstacles require three-dimensional center and size")
            for dimension in item["size_m"]:
                _number(dimension, item["name"] + ".size", 0.001)
            if item["type"] == "building" and item["size_m"][2] > 6:
                raise ValueError("buildings are limited to two storeys / 6 m")
    for event in data["sensor_fault_schedule"]:
        if set(event) != {"channel", "mode", "start_s", "duration_s"}:
            raise ValueError("invalid sensor fault event")
        if event["channel"] not in ("rgb", "depth", "lidar", "range", "imu", "gps", "compass", "barometer", "battery"):
            raise ValueError("invalid fault channel")
        if event["mode"] not in ("drop", "freeze", "delay"):
            raise ValueError("invalid fault mode")
        _number(event["start_s"], "fault start", 0)
        _number(event["duration_s"], "fault duration", 0.001)
    for section in ("gps_degradation", "communication", "battery", "mission", "success", "ground_truth"):
        if not isinstance(data[section], dict):
            raise TypeError(section + " must be an object")
    communication = data["communication"]
    for key in ("delay_s", "dropout_s"):
        _number(communication.get(key), "communication." + key, 0)
    gps = data["gps_degradation"]
    for key in ("position_stddev_m", "velocity_stddev_m_s"):
        _number(gps.get(key), "gps_degradation." + key, 0)
    battery = data["battery"]
    _number(battery.get("initial_soc"), "battery.initial_soc", 0)
    _number(battery.get("load_w"), "battery.load_w", 0)
    if battery["initial_soc"] > 1:
        raise ValueError("battery.initial_soc must be 0..1")
    if communication.get("delay_s", 0) > 0 and not any(
        event["mode"] == "delay" for event in data["sensor_fault_schedule"]
    ):
        raise ValueError("communication delay requires a scheduled delay event")
    if communication.get("dropout_s", 0) > 0 and not any(
        event["mode"] == "drop" and event["duration_s"] == communication["dropout_s"]
        for event in data["sensor_fault_schedule"]
    ):
        raise ValueError("communication dropout requires a matching drop event")
    if data["mission"].get("type") != "takeoff_hover_land":
        raise ValueError("only takeoff_hover_land is currently executable")
    if data["mission"].get("altitude_m") != 3 or data["mission"].get("hover_s") != 10:
        raise ValueError("current flight controller requires altitude_m=3 and hover_s=10")
    if data["success"].get("expected_outcome") not in ("pass", "reject"):
        raise ValueError("success.expected_outcome must be pass or reject")
    expected_reject = wind["speed_m_s"] > wind["operational_limit_m_s"]
    if (data["success"]["expected_outcome"] == "reject") != expected_reject:
        raise ValueError("reject outcome must match the configured wind limit")
    return path, data


def canonical_bytes(data):
    return (json.dumps(data, sort_keys=True, separators=(",", ":")) + "\n").encode()

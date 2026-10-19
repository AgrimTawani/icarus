import importlib.util
import math
from pathlib import Path
from types import SimpleNamespace

import pytest

ROOT = Path(__file__).resolve().parents[2]
SPEC = importlib.util.spec_from_file_location(
    "autopilot_sensor_bridge", ROOT / "scripts/simulation/autopilot_sensor_bridge.py"
)
bridge = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(bridge)


def scan(values, angle=0, step=0, minimum=0.1, maximum=40):
    return SimpleNamespace(
        count=len(values),
        vertical_count=1,
        ranges=values,
        angle_min=angle,
        angle_step=step,
        vertical_angle_min=0,
        vertical_angle_step=0,
        range_min=minimum,
        range_max=maximum,
    )


def test_lidar_frame_and_mounting_offset():
    # Gazebo +Y is left, hence -90deg (sector54) in clockwise FRD.
    sectors = bridge.proximity_sectors(scan([5], angle=math.pi / 2))
    assert sectors[54] == 500
    assert sectors[18] == 65535
    assert bridge.proximity_sectors(scan([5]))[0] == 507
    assert bridge.proximity_sectors(scan([5], angle=math.pi))[36] == 493


def test_lidar_unknown_is_not_no_obstacle():
    assert bridge.proximity_sectors(scan([math.nan]))[0] == 65535
    assert bridge.proximity_sectors(scan([0.5]))[0] == 65535
    assert bridge.proximity_sectors(scan([math.inf]))[0] == 4001
    assert bridge.proximity_sectors(scan([5, 3, math.inf]))[0] == 307


def test_lidar_vertical_slice_excludes_high_returns():
    value = scan([4])
    value.vertical_angle_min = math.radians(30)
    assert bridge.proximity_sectors(value) == [65535] * 72


def test_range_values_invalid_and_no_hit():
    assert bridge.downward_distance(scan([2.0, 2.02, math.nan], maximum=30)) == 201
    assert bridge.downward_distance(scan([math.inf], maximum=30)) == 3001
    assert bridge.downward_distance(scan([math.nan], maximum=30)) is None
    assert bridge.downward_distance(scan([0.01], minimum=0.08, maximum=30)) == 7
    assert bridge.downward_distance(scan([-math.inf], minimum=0.08, maximum=30)) == 7


def test_malformed_scan_fails_closed():
    value = scan([3])
    value.vertical_count = 2
    with pytest.raises(ValueError):
        bridge.proximity_sectors(value)
    with pytest.raises(ValueError):
        bridge.downward_distance(value)


def test_flow_units_and_quality_gates():
    value = {
        "frame": "body_frd",
        "integration_interval_s": 0.05,
        "flow_integral_rad": {"x": 0.01, "y": -0.02},
        "quality": 200,
    }
    assert bridge.flow_rates(value) == pytest.approx((0.2, -0.4, 200))
    value["quality"] = 20
    assert bridge.flow_rates(value)[2] == 0
    value["flow_integral_rad"]["x"] = math.nan
    with pytest.raises(ValueError):
        bridge.flow_rates(value)


def test_no_stale_duplicate_or_dropped_samples_replayed():
    queue = bridge.LatestSamples()
    queue.put("flow", 1, "first", 10)
    assert queue.take(10.1)["flow"][1] == "first"
    assert queue.take(10.2) == {}
    queue.put("flow", 1, "duplicate", 10.2)
    assert queue.take(10.2) == {}
    queue.put("flow", 2, "expired", 11)
    assert queue.take(11.5) == {}
    queue.dropped.add("flow")
    queue.put("flow", 3, "fault", 12)
    assert queue.take(12) == {}


def test_gazebo_parameters_keep_gps_baro_and_add_aiding():
    params = bridge.expected_parameters()
    assert params["RNGFND1_TYPE"] == 10
    assert params["FLOW_TYPE"] == 5
    assert params["PRX1_TYPE"] == 2
    assert params["EK3_SRC1_POSXY"] == 3
    assert params["EK3_SRC1_POSZ"] == 1
    assert params["EK3_SRC2_VELXY"] == 5
    assert params["EK3_SRC_OPTIONS"] == 1
    assert params["AVOID_BACKUP_SPD"] == 0

"""MissionClient.localize_target is evidence only: VLM selects, LiDAR measures."""

import importlib.util
import math
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import MagicMock, patch

ROOT = Path(__file__).resolve().parents[2]


def _module():
    path = ROOT / "scripts/autonomy/run_mission.py"
    spec = importlib.util.spec_from_file_location("icarus_run_mission_localize", path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


STATE = {"local_position_ned": {"north_m": 0.0, "east_m": 0.0, "down_m": -6.76,
                                "origin_id": "ekf-origin"},
         "attitude": {"yaw_rad": math.radians(1.0)}, "relative_altitude_m": 5.0}


def _perception(observed_at):
    face = [(13.8, -4.1), (13.9, 4.1)]
    return {"local_map_available": True, "observed_at_unix_ms": str(observed_at),
            "obstacle_cluster_total": 1,
            "nearby_obstacles": [{
                "obstacle_id": "lidar-9-0", "sensor_id": "mid360", "point_count": 362,
                "top_down_m": -7.69, "bottom_down_m": -4.98,
                "observed_at_unix_ms": str(observed_at),
                "centroid": {"north_m": 13.85, "east_m": 0.0},
                "hull": [{"north_m": n, "east_m": e, "down_m": -7.69} for n, e in face]}]}


def _client(module, grounding):
    client = module.MissionClient.__new__(module.MissionClient)
    client.episode = SimpleNamespace(session={"simulator": "gazebo"}, record=MagicMock())
    client.simulation_session = {"simulator": "gazebo"}
    client.vision_profile = "edge-vision"
    client.target_localizer = None
    client.edge_vlm_limiter = SimpleNamespace(admit=lambda: None)
    # Any flight RPC would raise: localization must not command the vehicle.
    client.action_api = MagicMock(side_effect=AssertionError("no actions"))
    client.context = MagicMock(side_effect=AssertionError("no command context"))
    client._capture_rgb = MagicMock(return_value="gazebo_forward_d435i_ephemeral_capture")
    client.state = MagicMock()
    client._perception_with_obstacles = MagicMock(
        side_effect=lambda: _perception(int(module.time.time() * 1000)))
    patcher = patch("python.perception.vlm.ground_description", return_value=grounding)
    return client, patcher


def _grounding(present, text):
    return {"present": present, "presence_answer": "Yes." if present else "No",
            "grounding_text": text, "latency_ms": 17_000.0, "model_load_ms": 3_000.0,
            "image": {"sha256": "abc"}, "runtime": {"family": "qwen2-vl"}}


def test_localize_target_measures_with_lidar_and_records_evidence():
    module = _module()
    client, patcher = _client(module, _grounding(True, "(319,402),(648,859)"))
    with patcher, patch.object(module, "MessageToDict", return_value=STATE):
        estimate = client.localize_target("building behind the group of people")
    assert estimate["found"] is True
    assert estimate["flight_authority"] is False
    assert 13.5 < estimate["surface_range_m"] < 14.2
    # Only the front face was measured; its center is not an inferred
    # building center several metres behind that face.
    assert 13.5 < estimate["estimated_position"]["north_m"] < 14.2
    assert estimate["position_basis"] == "center_of_observed_lidar_extent"
    assert "assumed_outline" not in estimate["footprint"]
    assert estimate["footprint"]["height_m"] == 5.93
    assert estimate["quality"]["lidar_returns"] == 362
    assert estimate["quality"]["lidar_refreshed_after_vlm"] is True
    assert estimate["vlm"]["presence_answer"] == "Yes."
    assert {item.get("sensor") for item in estimate["provenance"]} >= {
        "intel_realsense_d435i_rgb", "livox_mid360"}
    client.action_api.assert_not_called()
    client.episode.record.assert_called_once()
    assert client.episode.record.call_args.args[0] == "semantic_target_localization"


def test_localize_target_reports_not_found_when_vlm_sees_no_match():
    module = _module()
    client, patcher = _client(module, _grounding(False, None))
    with patcher, patch.object(module, "MessageToDict", return_value=STATE):
        estimate = client.localize_target("building behind the group of people")
    assert estimate["found"] is False
    assert estimate["reason"] == "vlm_reported_no_match"
    assert "estimated_position" not in estimate


def test_localize_target_separates_forward_view_from_visual_subject():
    module = _module()
    client, patcher = _client(module, _grounding(True, "(319,402),(648,859)"))
    with patcher as ground, patch.object(module, "MessageToDict", return_value=STATE):
        estimate = client.localize_target("building in front")
    assert ground.call_args.args[1] == "building"
    assert estimate["description"] == "building in front"
    assert estimate["vlm"]["visual_query"] == "building"
    assert estimate["vlm"]["view_selector"] == "forward_d435i"


def test_localize_target_refuses_uncalibrated_simulators():
    module = _module()
    client, patcher = _client(module, _grounding(True, "(1,1),(2,2)"))
    client.simulation_session = {"simulator": "airsim"}
    with patcher:
        try:
            client.localize_target("building")
        except RuntimeError as error:
            assert "calibration" in str(error)
        else:
            raise AssertionError("AirSim camera calibration is not defined")


def test_localization_retains_sensor_to_model_coordinate_provenance():
    module = _module()
    coordinate_evidence = {'grounding_format':'json_pixel_xyxy',
        'pixel_box_xyxy':[8,4,72,36], 'model_pixel_box_xyxy':[16,8,144,72],
        'image_coordinate_transform':{'source_width':80,'source_height':40,
                                    'model_width':160,'model_height':80}}
    client, patcher = _client(module, _grounding(True, '(319,402),(648,859)') | coordinate_evidence)
    with patcher, patch.object(module, 'MessageToDict', return_value=STATE):
        estimate = client.localize_target('visible structure')
    for key, value in coordinate_evidence.items():
        assert estimate['vlm'][key] == value

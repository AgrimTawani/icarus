import importlib.util
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import patch

ROOT = Path(__file__).resolve().parents[2]


def _module():
    path = ROOT / "scripts/autonomy/run_mission.py"
    spec = importlib.util.spec_from_file_location("icarus_run_mission_test", path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def test_native_full_vocabulary_request_reaches_fixed_detector_and_returns_complete_counts(monkeypatch):
    import time
    from icarus.v1 import state_pb2
    from python.dcm.agent_tools import validate_call
    from python.perception.classes import YOLO_CLASSES
    from python.perception import edge_vision
    module=_module()
    client=module.MissionClient.__new__(module.MissionClient)
    client.vision_profile='edge-vision'
    client.simulation_session={'simulator':'gazebo'}
    client.episode=SimpleNamespace(session={'simulator':'gazebo'},record=lambda *args:None)
    client.edge_detection_limiter=SimpleNamespace(admit=lambda:None)
    client.edge_trackers={}
    client.state=lambda:state_pb2.DroneState(landed=True)
    client._edge_manifest=lambda:None
    client._edge_detector=lambda:None
    captures=[]
    def capture(output,**kwargs):
        output.write_bytes(b'P6\n1 1\n255\n\0\0\0');captures.append(output);return 'test_capture'
    client._capture_rgb=capture
    calls=[]
    def infer(*args,**kwargs):
        calls.append(args)
        return {'schema':'icarus.edge.yolo.v1','classes':sorted(YOLO_CLASSES),
            'observed_at_unix_ms':int(time.time()*1000),'confidence_threshold':0.25,
            'detections':[{'class':'person','confidence':0.9}]}
    monkeypatch.setattr(edge_vision,'detect_edge_image',infer)
    classes=client.camera_detector_classes()
    validate_call('detect',{'classes':classes,'camera':'forward_d435i'})
    result=client.detect(classes,camera='forward_d435i')
    assert set(result['counts'])==YOLO_CLASSES
    assert result['counts']['person']==1
    assert len(captures)==len(calls)==1
    assert all(value==0 for name,value in result['counts'].items() if name!='person')


def test_realistic_airsim_count_is_inconclusive_without_reviewed_truth():
    module = _module()
    client = module.MissionClient.__new__(module.MissionClient)
    client.simulation_session = {"simulator": "airsim", "profile": "airsim-realistic"}
    client.vision_profile = "edge-vision"
    executed = [
        {"action": "inspect_scene", "outcome": "SUCCEEDED",
         "arguments": {"question": "Describe the visible structure."},
         "result": {"answer": "A tower is visible."}},
        {"action": "detect", "outcome": "SUCCEEDED",
         "arguments": {"classes": ["person"]},
         "result": {"unique_person_count": 0}},
    ]
    result = client.assess_mission_completion(
        "Check the visible building and count people", executed)
    assert result["success"] is False
    assert result["observed_person_count"] == 0
    assert result["verification"] == "inconclusive_no_airsim_ground_truth"
    assert result["satisfied"] == ["building_visual_inspection"]


def test_realistic_airsim_prompt_fails_without_inspection_or_detection():
    module = _module()
    client = module.MissionClient.__new__(module.MissionClient)
    client.simulation_session = {"simulator": "airsim", "profile": "airsim-realistic"}
    client.vision_profile = "edge-vision"
    result = client.assess_mission_completion(
        "Inspect the building and count the number of people", [])
    assert result["success"] is False
    assert any("detect evidence" in reason for reason in result["reasons"])
    assert any("inspect_scene evidence" in reason for reason in result["reasons"])


def test_environment_reference_is_always_blank_even_when_session_contains_truth():
    module = _module()
    client = module.MissionClient.__new__(module.MissionClient)
    client.simulation_session = {
        "simulator": "airsim",
        "mission_reference": {
            "person_count_ground_truth": {"expected_visible_count": 4},
            "known_landmarks": [{"id": "hidden-place", "north_m": 10}],
        },
    }
    assert not hasattr(client, "mission_reference")


def test_gazebo_environment_reference_is_also_blank():
    module = _module()
    client = module.MissionClient.__new__(module.MissionClient)
    client.simulation_session = {
        "simulator": "gazebo", "scenario": "edge_people_building"}
    assert not hasattr(client, "mission_reference")


def test_live_body_describes_hardware_but_not_world_content():
    module = _module()
    client = module.MissionClient.__new__(module.MissionClient)
    client.capabilities = SimpleNamespace(
        vehicle_type="MULTIROTOR", autopilot="ARDUPILOT_COPTER",
        autopilot_version="4.6.2",
        capabilities=[SimpleNamespace(name="arm", supported=True),
                      SimpleNamespace(name="unused", supported=False)])
    client.simulation_session = {
        "simulator": "airsim", "profile": "airsim-realistic",
        "controller": "ArduPilot-SITL",
        "settings": "config/simulation/airsim/settings.realistic-city.json",
        "environment": {"id": "private-world-name"},
    }
    body = client.capability_profile()
    assert body["autopilot_version"] == "4.6.2"
    assert body["airframe"]["gazebo_airframe_equivalent"] is False
    assert body["sensors"] == ["Gps", "IcarusLidar", "Imu", "Lidar1"]
    assert "0" in body["cameras"]
    assert body["autopilot_parameter_overrides"]["SCHED_LOOP_RATE"] == "150"
    assert "environment" not in body
    assert "private-world-name" not in str(body)


def test_gazebo_visual_inspection_uses_forward_camera_not_belly_camera(tmp_path):
    module = _module()
    client = module.MissionClient.__new__(module.MissionClient)
    client.simulation_session = {"simulator": "gazebo"}
    with patch.object(module.subprocess, "run") as run:
        source = client._capture_rgb(tmp_path / "frame.ppm")
    command = run.call_args.args[0]
    assert command[command.index("--topic") + 1] == \
        "/icarus/sensors/d435i/image"
    assert source == "gazebo_forward_d435i_ephemeral_capture"


def test_legacy_regex_obligation_api_is_removed():
    module = _module()
    client = module.MissionClient.__new__(module.MissionClient)
    client.simulation_session = {"simulator": "gazebo"}
    assert not hasattr(client, "mission_obligations")
    assert not hasattr(client, "mission_guidance")


def test_visual_target_hover_never_uses_a_hidden_session_waypoint():
    module = _module()
    client = module.MissionClient.__new__(module.MissionClient)
    client.vision_profile = "edge-vision"
    client.simulation_session = {
        "simulator": "airsim",
        "mission_reference": {
            "primary_target_id": "hidden-place",
            "known_landmarks": [{
                "id": "hidden-place",
                "hover_waypoint": {
                    "north_m": 12, "east_m": -4, "altitude_agl_m": 15,
                },
            }],
        },
    }
    result = client.assess_mission_completion(
        "Hover over the building",
        [{"action": "goto", "outcome": "SUCCEEDED", "arguments": {
            "north_m": 12, "east_m": -4, "altitude_agl_m": 15}},
         {"action": "hold", "outcome": "SUCCEEDED", "arguments": {
             "duration_ms": 5000}}])
    assert result["success"] is False
    assert "building_hover" not in result["satisfied"]
    assert any("no environment waypoint is injected" in reason
               for reason in result["reasons"])


def test_non_vision_wind_mission_still_gets_semantic_completion():
    module = _module()
    client = module.MissionClient.__new__(module.MissionClient)
    client.vision_profile = "default"
    client.simulation_session = {
        "simulator": "gazebo", "scenario": "wind_gusting"}
    result = client.assess_mission_completion(
        "take off, hover for 8 seconds in gusting wind, then return home",
        [{"action": "takeoff", "outcome": "SUCCEEDED", "arguments": {
            "target_altitude_agl_m": 5}},
         {"action": "hold", "outcome": "SUCCEEDED", "arguments": {
             "duration_ms": 8000}},
         {"action": "return_home", "outcome": "SUCCEEDED", "arguments": {}}])
    assert result == {
        "required": ["hold_position", "return_home"],
        "satisfied": ["hold_position", "return_home"],
        "reasons": [], "success": True}


def test_session_metadata_cannot_turn_airsim_detection_into_verified_truth():
    module = _module()
    client = module.MissionClient.__new__(module.MissionClient)
    client.vision_profile = "edge-vision"
    client.simulation_session = {
        "simulator": "airsim",
        "mission_reference": {
            "person_count_ground_truth": {
                "expected_visible_count": 2,
                "review_status": "reviewed",
                "camera_id": "0",
            },
        },
    }
    result = client.assess_mission_completion(
        "Count people", [{
            "action": "detect", "outcome": "SUCCEEDED",
            "arguments": {"classes": ["person"]},
            "result": {"unique_person_count": 2},
        }])
    assert result["success"] is False
    assert result["observed_person_count"] == 2
    assert result["verification"] == "inconclusive_no_airsim_ground_truth"
    assert result["satisfied"] == []
    assert not hasattr(client, "mission_reference")

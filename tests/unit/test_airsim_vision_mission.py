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


def test_realistic_airsim_count_is_inconclusive_without_reviewed_truth():
    module = _module()
    client = module.MissionClient.__new__(module.MissionClient)
    client.simulation_session = {"simulator": "airsim", "profile": "airsim-realistic"}
    client.vision_profile = "edge-vision"
    executed = [
        {"action": "inspect_scene", "outcome": "SUCCEEDED",
         "arguments": {"question": "Describe the building."},
         "result": {"answer": "A tower is visible."}},
        {"action": "detect", "outcome": "SUCCEEDED",
         "arguments": {"classes": ["person"]},
         "result": {"unique_person_count": 0}},
    ]
    result = client.assess_mission_completion(
        "Check that building and count people", executed)
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


def test_realistic_airsim_reference_never_claims_hidden_truth():
    module = _module()
    client = module.MissionClient.__new__(module.MissionClient)
    client.simulation_session = {
        "simulator": "airsim",
        "mission_reference": {
            "person_count_ground_truth": {
                "expected_visible_count": 0,
                "review_status": "reviewed",
                "camera_id": "0",
            },
            "known_landmarks": [{"id": "tower", "north_m": 10, "east_m": 5}],
            "count_window_s": 1,
        },
    }
    client.vision_profile = "edge-vision"
    reference = client.mission_reference()
    assert reference["known_landmarks"][0]["id"] == "tower"
    assert "unique_person_count" not in reference
    assert reference["visual_requirements"]["person_count_requires_detect"] is True


def test_live_body_describes_connected_airsim_firmware_and_sensors():
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
        "environment": {"id": "airvln-env-6"},
    }
    body = client.capability_profile()
    assert body["autopilot_version"] == "4.6.2"
    assert body["airframe"]["gazebo_airframe_equivalent"] is False
    assert body["sensors"] == ["Gps", "Imu", "Lidar1"]
    assert "0" in body["cameras"]
    assert body["autopilot_parameter_overrides"]["SCHED_LOOP_RATE"] == "150"


def test_gazebo_reference_exposes_reviewed_primary_building_geometry():
    module = _module()
    client = module.MissionClient.__new__(module.MissionClient)
    client.vision_profile = "edge-vision"
    client.simulation_session = {
        "simulator": "gazebo", "scenario": "edge_people_building"}
    reference = client.mission_reference()
    assert reference["primary_target_id"] == "north_building"
    landmark = reference["known_landmarks"][0]
    assert landmark["hover_waypoint"]["north_m"] == 18
    assert landmark["orbit"]["center_north_m"] == 18
    assert landmark["inspection_waypoint"]["north_m"] == 4
    assert landmark["person_observation_waypoint"]["altitude_agl_m"] == 4


def test_gazebo_visual_inspection_uses_forward_camera_not_belly_camera(tmp_path):
    module = _module()
    client = module.MissionClient.__new__(module.MissionClient)
    client.simulation_session = {"simulator": "gazebo"}
    with patch.object(module.subprocess, "run") as run:
        source = client._capture_rgb(tmp_path / "frame.ppm")
    command = run.call_args.args[0]
    assert command[command.index("--topic") + 1] == "/icarus/sensors/rgbd/image"
    assert source == "gazebo_forward_rgbd_ephemeral_capture"


def test_full_natural_mission_exposes_ordered_camera_and_hold_obligations():
    module = _module()
    client = module.MissionClient.__new__(module.MissionClient)
    client.vision_profile = "edge-vision"
    client.simulation_session = {
        "simulator": "gazebo", "scenario": "edge_people_building"}
    mission = ("Arm, take off to 5 metres, inspect the building, orbit it, "
               "count the four people, hold position for 5 seconds, then return home")
    obligations = client.mission_obligations(mission, [], {})
    assert obligations["pending"] == [
        "flight", "building_visual_inspection", "inspection_orbit",
        "person_count", "hold_position", "return_home"]
    by_id = {item["id"]: item for item in obligations["requirements"]}
    assert by_id["building_visual_inspection"]["reviewed_waypoint"]["north_m"] == 4
    assert by_id["person_count"]["reviewed_waypoint"]["north_m"] == 4
    assert by_id["hold_position"]["duration_ms"] == 5000


def test_pronoun_inspection_and_qualified_people_count_are_not_dropped():
    module = _module()
    client = module.MissionClient.__new__(module.MissionClient)
    client.vision_profile = "edge-vision"
    client.simulation_session = {
        "simulator": "gazebo", "scenario": "edge_people_building"}
    mission = ("Arm, take off to 5 metres, fly to the north building, visually "
               "inspect it, orbit it once, then count all visible people, hover "
               "for 5 seconds, and return home.")
    obligations = client.mission_obligations(mission, [], {})
    assert obligations["pending"] == [
        "flight", "building_visual_inspection", "inspection_orbit",
        "person_count", "hold_position", "return_home"]
    by_id = {item["id"]: item for item in obligations["requirements"]}
    assert by_id["building_visual_inspection"]["reviewed_waypoint"][
        "heading_deg"] == 0
    assert by_id["person_count"]["detector_classes"] == ["person"]
    assert by_id["hold_position"]["duration_ms"] == 5000


def test_natural_person_count_variants_create_detection_obligation():
    module = _module()
    client = module.MissionClient.__new__(module.MissionClient)
    client.vision_profile = "edge-vision"
    client.simulation_session = {
        "simulator": "gazebo", "scenario": "edge_people_building"}
    for mission in ("count the four people", "count how many people are visible",
                    "detect people"):
        obligations = client.mission_obligations(mission, [], {})
        assert "person_count" in obligations["pending"]


def test_detect_building_uses_visual_inspection_obligation():
    module = _module()
    client = module.MissionClient.__new__(module.MissionClient)
    client.vision_profile = "edge-vision"
    client.simulation_session = {
        "simulator": "gazebo", "scenario": "edge_people_building"}
    obligations = client.mission_obligations("detect the north building", [], {})
    assert obligations["pending"] == ["building_visual_inspection"]


def test_gazebo_building_hover_is_scored_from_scenario_reference():
    module = _module()
    client = module.MissionClient.__new__(module.MissionClient)
    client.vision_profile = "edge-vision"
    client.simulation_session = {
        "simulator": "gazebo", "scenario": "edge_people_building"}
    result = client.assess_mission_completion(
        "hover over the north building for 4 seconds, then return home",
        [{"action": "goto", "outcome": "SUCCEEDED", "arguments": {
            "north_m": 18, "east_m": 0, "altitude_agl_m": 9}},
         {"action": "hold", "outcome": "SUCCEEDED", "arguments": {
             "duration_ms": 4000}},
         {"action": "return_home", "outcome": "SUCCEEDED", "arguments": {}}])
    assert result["success"] is True
    assert "building_hover" in result["satisfied"]


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


def test_count_no_of_people_and_reviewed_building_hover_are_enforced():
    module = _module()
    client = module.MissionClient.__new__(module.MissionClient)
    client.vision_profile = "edge-vision"
    client.simulation_session = {
        "simulator": "airsim",
        "mission_reference": {
            "person_count_ground_truth": {
                "expected_visible_count": 0,
                "review_status": "reviewed",
                "camera_id": "0",
            },
            "primary_target_id": "office",
            "known_landmarks": [{
                "id": "office",
                "hover_waypoint": {
                    "north_m": 12, "east_m": -4, "altitude_agl_m": 15,
                },
            }],
        },
    }
    executed = [
        {"action": "inspect_scene", "outcome": "SUCCEEDED", "arguments": {
            "question": "Describe the office."}, "result": {"answer": "office"}},
        {"action": "goto", "outcome": "SUCCEEDED", "arguments": {
            "north_m": 12, "east_m": -4, "altitude_agl_m": 15}},
        {"action": "hold", "outcome": "SUCCEEDED", "arguments": {
            "duration_ms": 3000}},
        {"action": "detect", "outcome": "SUCCEEDED", "arguments": {
            "classes": ["person"]}, "result": {"unique_person_count": 0}},
    ]
    result = client.assess_mission_completion(
        "Check that building, hover over it, and count no. of people", executed)
    assert result["success"] is True
    assert result["observed_person_count"] == 0
    assert result["expected_person_count"] == 0
    assert "verified_person_count" in result["satisfied"]
    assert "building_hover" in result["satisfied"]


def test_building_hover_requires_ordered_detection():
    module = _module()
    client = module.MissionClient.__new__(module.MissionClient)
    client.vision_profile = "edge-vision"
    client.simulation_session = {
        "simulator": "airsim", "mission_reference": {
            "primary_target_id": "office",
            "known_landmarks": [{"id": "office", "hover_waypoint": {
                "north_m": 12, "east_m": -4, "altitude_agl_m": 15}}],
        }}
    executed = [
        {"action": "detect", "outcome": "SUCCEEDED",
         "arguments": {"classes": ["person"]},
         "result": {"unique_person_count": 1}},
        {"action": "inspect_scene", "outcome": "SUCCEEDED",
         "arguments": {"question": "building"}, "result": {}},
        {"action": "goto", "outcome": "SUCCEEDED", "arguments": {
            "north_m": 12, "east_m": -4, "altitude_agl_m": 15}},
        {"action": "hold", "outcome": "SUCCEEDED",
         "arguments": {"duration_ms": 3000}},
    ]
    result = client.assess_mission_completion(
        "Check that building, hover over it, and count no. of people", executed)
    assert result["success"] is False
    assert any("after the building hover" in reason for reason in result["reasons"])


def test_live_obligations_report_evidence_without_forcing_an_action():
    module = _module()
    client = module.MissionClient.__new__(module.MissionClient)
    client.vision_profile = "edge-vision"
    client.simulation_session = {
        "simulator": "airsim", "mission_reference": {
            "primary_target_id": "office",
            "known_landmarks": [{"id": "office", "hover_waypoint": {
                "north_m": 12, "east_m": -4, "altitude_agl_m": 15}}],
        }}
    # Protobuf JSON omits false scalar fields, so airborne observations may
    # have no explicit ``landed`` key.
    state = {"state": {"armed": True, "flight_phase": "FLIGHT_PHASE_AIRBORNE"}}
    mission = "Check that building, hover over it for 4 seconds, count no. of people"
    guidance = client.mission_obligations(mission, [], state)
    assert "required_next_action" not in guidance
    assert guidance["authority"] == "completion_evidence_only"
    assert set(guidance["pending"]) == {
        "flight", "building_visual_inspection", "building_hover", "person_count"}
    history = [{"action": "inspect_scene", "outcome": "SUCCEEDED"}]
    guidance = client.mission_obligations(mission, history, state)
    assert "building_visual_inspection" not in guidance["pending"]
    hover = next(item for item in guidance["requirements"]
                 if item["id"] == "building_hover")
    assert hover["reviewed_waypoint"] == {
        "north_m": 12, "east_m": -4, "altitude_agl_m": 15}
    assert hover["duration_ms"] == 4000
    history.append({"action": "goto", "outcome": "SUCCEEDED", "arguments": {
        "north_m": 12, "east_m": -4, "altitude_agl_m": 15}})
    guidance = client.mission_obligations(mission, history, state)
    assert "building_hover" in guidance["pending"]
    history.append({"action": "hold", "outcome": "SUCCEEDED"})
    guidance = client.mission_obligations(mission, history, state)
    assert "building_hover" not in guidance["pending"]
    assert "person_count" in guidance["pending"]

    history.append({"action": "detect", "outcome": "SUCCEEDED"})
    mission += ", then return home"
    guidance = client.mission_obligations(mission, history, state)
    assert guidance["pending"] == ["flight", "return_home"]


def test_live_obligations_do_not_choose_arm_or_takeoff():
    module = _module()
    client = module.MissionClient.__new__(module.MissionClient)
    client.vision_profile = "edge-vision"
    client.simulation_session = {"simulator": "airsim", "mission_reference": {}}
    mission = "Arm, take off to 3 metres, and hover"
    disarmed = {"state": {"landed": True,
                          "flight_phase": "FLIGHT_PHASE_DISARMED"}}
    obligations = client.mission_obligations(mission, [], disarmed)
    assert obligations["pending"] == ["flight"]
    assert "required_next_action" not in obligations
    armed = {"state": {"armed": True, "landed": True,
                       "flight_phase": "FLIGHT_PHASE_ARMED_GROUND"}}
    obligations = client.mission_obligations(mission, [], armed)
    requirement = obligations["requirements"][0]
    assert requirement["target_altitude_agl_m"] == 3.0
    assert requirement["status"] == "pending"

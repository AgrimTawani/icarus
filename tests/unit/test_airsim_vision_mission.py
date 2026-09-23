import importlib.util
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]


def _module():
    path = ROOT / "scripts/autonomy/run_mission.py"
    spec = importlib.util.spec_from_file_location("icarus_run_mission_test", path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def test_realistic_airsim_prompt_requires_structured_visual_evidence():
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
    assert result["success"] is True
    assert result["observed_person_count"] == 0
    assert result["satisfied"] == [
        "visible_person_count", "building_visual_inspection"]


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
            "known_landmarks": [{"id": "tower", "north_m": 10, "east_m": 5}],
            "count_window_s": 1,
        },
    }
    client.vision_profile = "edge-vision"
    reference = client.mission_reference()
    assert reference["known_landmarks"][0]["id"] == "tower"
    assert "unique_person_count" not in reference
    assert reference["visual_requirements"]["person_count_requires_detect"] is True


def test_count_no_of_people_and_reviewed_building_hover_are_enforced():
    module = _module()
    client = module.MissionClient.__new__(module.MissionClient)
    client.vision_profile = "edge-vision"
    client.simulation_session = {
        "simulator": "airsim",
        "mission_reference": {
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


def test_live_guidance_sequences_inspect_hover_hold_and_detection():
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
    guidance = client.mission_guidance(mission, [], state)
    assert guidance["required_next_action"] == "inspect_scene"
    history = [{"action": "inspect_scene", "outcome": "SUCCEEDED"}]
    guidance = client.mission_guidance(mission, history, state)
    assert guidance == {"required_next_action": "goto", "arguments": {
        "north_m": 12, "east_m": -4, "altitude_agl_m": 15},
        "reason": "requested hover has not reached its reviewed waypoint"}
    history.append({"action": "goto", "outcome": "SUCCEEDED"})
    guidance = client.mission_guidance(mission, history, state)
    assert guidance["required_next_action"] == "hold"
    assert guidance["arguments"]["duration_ms"] == 4000
    history.append({"action": "hold", "outcome": "SUCCEEDED"})
    assert client.mission_guidance(mission, history, state)[
        "required_next_action"] == "detect"

    history.append({"action": "detect", "outcome": "SUCCEEDED"})
    mission += ", then return home"
    assert client.mission_guidance(mission, history, state)[
        "required_next_action"] == "return_home"


def test_live_guidance_stabilizes_arm_and_takeoff_sequence():
    module = _module()
    client = module.MissionClient.__new__(module.MissionClient)
    client.vision_profile = "edge-vision"
    client.simulation_session = {"simulator": "airsim", "mission_reference": {}}
    mission = "Arm, take off to 3 metres, and hover"
    disarmed = {"state": {"landed": True,
                          "flight_phase": "FLIGHT_PHASE_DISARMED"}}
    assert client.mission_guidance(mission, [], disarmed)[
        "required_next_action"] == "arm"
    armed = {"state": {"armed": True, "landed": True,
                       "flight_phase": "FLIGHT_PHASE_ARMED_GROUND"}}
    guidance = client.mission_guidance(mission, [], armed)
    assert guidance == {"required_next_action": "takeoff",
                        "arguments": {"target_altitude_agl_m": 3.0},
                        "reason": "requested flight is armed but still on the ground"}

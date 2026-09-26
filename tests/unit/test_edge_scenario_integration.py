"""Headless integration check for the deterministic edge vision scenario."""
import json
import subprocess
import sys
import xml.etree.ElementTree as ET
from pathlib import Path


ROOT = Path(__file__).resolve().parents[2]


def test_edge_people_building_generates_headlessly_with_ground_truth():
    completed = subprocess.run(
        [sys.executable, "scripts/simulation/build_phase5_world.py", "edge_people_building"],
        cwd=ROOT, text=True, capture_output=True, check=True, timeout=30)
    world, truth = [Path(line) for line in completed.stdout.splitlines()]
    data = json.loads(truth.read_text())
    rendered = world.read_text()
    assert "north_building" in rendered
    assert '<actor name="person_01">' in rendered
    assert "gazebo_fuel_cache" in rendered
    assert {target["id"] for target in data["targets"]} == {
        "person_01", "person_02", "person_03", "person_04"}
    assert data["unique_person_count"] == 4
    scenario = json.loads((ROOT / "simulation/scenarios/edge_people_building.json").read_text())
    assert scenario["obstacles"][0]["center_m"] == [0, 18, 3]  # Gazebo ENU: north is +Y.
    assert scenario["ground_truth"]["known_landmarks"][0]["local_ned_m"] == [18, 0, -3]
    assert scenario["ground_truth"]["primary_target_id"] == "north_building"
    assert scenario["ground_truth"]["known_landmarks"][0]["hover_waypoint"] == {
        "north_m": 18, "east_m": 0, "altitude_agl_m": 9}
    assert scenario["ground_truth"]["known_landmarks"][0]["orbit"]["radius_m"] == 10
    assert scenario["ground_truth"]["known_landmarks"][0]["inspection_waypoint"] == {
        "north_m": 4, "east_m": 0, "altitude_agl_m": 4, "heading_deg": 0}
    assert scenario["ground_truth"]["known_landmarks"][0]["person_observation_waypoint"] == {
        "north_m": 4, "east_m": 0, "altitude_agl_m": 4, "heading_deg": 0}
    building = scenario["obstacles"][0]
    bx, by, _ = building["center_m"]
    sx, sy, _ = building["size_m"]
    assert all(abs(target["center_m"][0] - bx) > sx / 2
               or abs(target["center_m"][1] - by) > sy / 2
               for target in scenario["targets"])
    root = ET.parse(world).getroot()
    for target in scenario["targets"]:
        actor = root.find(f".//actor[@name='{target['id']}']")
        assert actor is not None
        assert actor.findtext("pose") == "0 0 0 0 0 0"
        waypoint = actor.find("script/trajectory/waypoint/pose")
        expected = target["center_m"]
        assert [float(value) for value in waypoint.text.split()[:2]] == expected[:2]

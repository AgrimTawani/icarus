"""Structural contract checks for the narrow-route acceptance client."""

from pathlib import Path
import sys
import unittest


ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / "scripts/simulation"))

from scenario_config import load_scenario
from score_phase5_trajectory import point_obstacle_clearance


class NarrowRouteAcceptanceTest(unittest.TestCase):
    def test_uses_typed_api_truth_scoring_and_episode_capture(self):
        source = (ROOT / "scripts/autonomy/run_narrow_route_acceptance.py").read_text()
        self.assertIn('session.get("scenario") != "obstacle_course"', source)
        self.assertIn('score(scenario, trajectory, "narrow"', source)
        self.assertIn('north_m=target[1], east_m=target[0], down_m=-target[2]', source)
        self.assertIn("client.action_api.Goto", source)
        self.assertIn("safe detour", source)
        self.assertIn("client.episode_outcome", source)
        self.assertNotIn("pymavlink", source)

    def test_narrow_route_goal_satisfies_its_declared_vehicle_clearance(self):
        _, scenario = load_scenario("obstacle_course")
        route = next(item for item in scenario["ground_truth"]["routes"]
                     if item["name"] == "narrow")
        goal = route["waypoints_enu_m"][-1]
        clearance = min(
            point_obstacle_clearance(goal, obstacle) - 0.35
            for obstacle in scenario["obstacles"]
        )
        self.assertGreaterEqual(clearance, route["clearance_m"])


if __name__ == "__main__":
    unittest.main()

"""Structural contract checks for the narrow-route acceptance client."""

from pathlib import Path
import unittest


ROOT = Path(__file__).resolve().parents[2]


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


if __name__ == "__main__":
    unittest.main()

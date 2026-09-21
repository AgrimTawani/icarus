"""Schema checks for simulator-only MAVLink fault scenarios."""

import sys
import unittest
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / "scripts/simulation"))

from scenario_config import load_scenario, sitl_battery_parameters  # noqa: E402


class ScenarioConfigTests(unittest.TestCase):
    def test_sitl_battery_overlay_uses_the_same_scenario_soc(self):
        _, scenario = load_scenario("adverse_combined")
        parameters = sitl_battery_parameters(scenario)
        self.assertEqual(parameters["SIM_BATT_CAP_AH"], 10.0)
        self.assertEqual(parameters["BATT_CAPACITY"], 10_000)
        self.assertAlmostEqual(parameters["SIM_BATT_VOLTAGE"], 22.47)

    def test_mavlink_loss_schedule_is_accepted(self):
        _, scenario = load_scenario("mavlink_loss")
        self.assertEqual(
            scenario["mavlink_fault_schedule"],
            [{"mode": "loss", "start_s": 15, "duration_s": 3}],
        )

    def test_mavlink_delay_schedule_is_accepted(self):
        _, scenario = load_scenario("mavlink_delay")
        event = scenario["mavlink_fault_schedule"][0]
        self.assertEqual(event["mode"], "delay")
        self.assertEqual(event["latency_ms"], 250)


if __name__ == "__main__":
    unittest.main()

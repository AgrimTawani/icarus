"""Baseline tests.

The scripted controller is the floor every model is measured against, so what
matters is that it decides from state alone, never sees the answer, and puts
safety ahead of mission progress.
"""

import json
import unittest

from python.dcm.baseline import ScriptedRuntime
from python.dcm.contract import validate_proposal


def observation(mission="takeoff_hover_land", completed=(), **state):
    base = {"flight_phase": "FLIGHT_PHASE_DISARMED", "armed": False,
            "landed": True, "relative_altitude_m": 0.0}
    perception = state.pop("perception", {"path_ahead_clear": True,
                                          "overall": "HEALTH_LEVEL_HEALTHY"})
    base.update(state)
    return {"mission": mission, "state": base, "perception": perception,
            "actions_completed": [{"action": a, "outcome": "SUCCEEDED"}
                                  for a in completed]}


def airborne(**extra):
    return observation(flight_phase="FLIGHT_PHASE_AIRBORNE", armed=True,
                       landed=False, relative_altitude_m=3.0, **extra)


class ContractComplianceTests(unittest.TestCase):
    def test_every_decision_is_a_valid_proposal(self):
        runtime = ScriptedRuntime()
        cases = [
            observation(),
            observation(completed=["arm"]),
            observation(armed=True),
            airborne(),
            airborne(completed=["hold"]),
            airborne(perception={"path_ahead_clear": False}),
            airborne(geofence_breached=True),
            airborne(battery={"remaining_percent": 12.0}),
        ]
        for case in cases:
            with self.subTest(case=case["state"]["flight_phase"]):
                validate_proposal(runtime.propose(case))

    def test_it_never_reads_the_recorded_action(self):
        # A baseline that peeked would score perfectly and mean nothing.
        runtime = ScriptedRuntime()
        case = airborne(completed=["hold"])
        case["recorded_action"] = "orbit"
        self.assertEqual(runtime.decide(case)["action"], "land")


class SafetyPrecedenceTests(unittest.TestCase):
    def setUp(self):
        self.runtime = ScriptedRuntime()

    def test_a_blocked_path_outranks_mission_progress(self):
        case = airborne(completed=["hold"],
                        perception={"path_ahead_clear": False})
        self.assertEqual(self.runtime.decide(case)["action"], "hold")

    def test_unavailable_perception_holds_while_airborne(self):
        case = airborne(perception={"overall": "HEALTH_LEVEL_UNAVAILABLE"})
        self.assertEqual(self.runtime.decide(case)["action"], "hold")

    def test_a_geofence_breach_returns_home(self):
        self.assertEqual(
            self.runtime.decide(airborne(geofence_breached=True))["action"],
            "return_home")

    def test_a_low_battery_returns_home_only_when_airborne(self):
        low = {"remaining_percent": 20.0}
        self.assertEqual(
            self.runtime.decide(airborne(battery=low))["action"],
            "return_home")
        # On the ground a low battery is not a reason to fly anywhere.
        self.assertNotEqual(
            self.runtime.decide(observation(battery=low))["action"],
            "return_home")


class MissionSequenceTests(unittest.TestCase):
    def setUp(self):
        self.runtime = ScriptedRuntime()

    def test_a_disarmed_aircraft_arms_first(self):
        self.assertEqual(self.runtime.decide(observation())["action"], "arm")

    def test_an_armed_aircraft_on_the_ground_takes_off(self):
        self.assertEqual(
            self.runtime.decide(observation(armed=True, landed=True))["action"],
            "takeoff")

    def test_an_airborne_aircraft_holds_then_lands(self):
        self.assertEqual(self.runtime.decide(airborne())["action"], "hold")
        self.assertEqual(
            self.runtime.decide(airborne(completed=["hold"]))["action"],
            "land")

    def test_navigation_also_counts_as_work_done(self):
        for action in ("goto", "orbit"):
            with self.subTest(action=action):
                self.assertEqual(
                    self.runtime.decide(airborne(completed=[action]))["action"],
                    "land")


class AltitudeTests(unittest.TestCase):
    def setUp(self):
        self.runtime = ScriptedRuntime()

    def altitude(self, mission):
        case = observation(mission=mission, armed=True, landed=True)
        return self.runtime.decide(case)["arguments"]["target_altitude_agl_m"]

    def test_an_altitude_in_the_mission_text_is_used(self):
        self.assertEqual(self.altitude("take off to 7 metres"), 7.0)
        self.assertEqual(self.altitude("climb to 4m and hold"), 4.0)

    def test_a_missing_altitude_falls_back_to_a_default(self):
        self.assertEqual(self.altitude("just fly"), 3.0)

    def test_an_out_of_bounds_altitude_is_clamped_not_proposed(self):
        # The baseline should fail by choosing badly, never by being rejected
        # on a technicality the contract would catch.
        self.assertEqual(self.altitude("take off to 500 metres"), 10.0)
        self.assertEqual(self.altitude("take off to 0.1 metres"), 0.5)


class DeterminismTests(unittest.TestCase):
    def test_the_same_observation_always_gives_the_same_answer(self):
        runtime = ScriptedRuntime()
        case = airborne(completed=["hold"])
        answers = {json.dumps(runtime.decide(case), sort_keys=True)
                   for _ in range(25)}
        self.assertEqual(len(answers), 1)

    def test_it_reports_a_stable_name_for_reports(self):
        self.assertEqual(ScriptedRuntime().name, "scripted-baseline-v1")


if __name__ == "__main__":
    unittest.main()

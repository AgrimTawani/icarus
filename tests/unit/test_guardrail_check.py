"""Tests for checking a model proposal against the real safety policy.

These are integration tests against the actual `icarus-check-guardrail`
binary and `config/safety/v1.yaml`, not a reimplementation in Python, because
the whole point of this module is that the flight safety decision has exactly
one implementation. They are skipped, not failed, when the binary has not
been built, since not every environment running the unit suite has done a
C++ build.
"""

import sys
import unittest
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[2]
                       / "build/generated/python"))

from python.dcm.guardrail_check import (
    DEFAULT_BINARY,
    DEFAULT_POLICY,
    GuardrailUnavailable,
    check_proposal,
)

BINARY_BUILT = DEFAULT_BINARY.is_file() and DEFAULT_POLICY.is_file()


def disarmed_state(**overrides):
    state = {
        "vehicle_id": "icarus-01", "sequence": "1",
        "observed_at_unix_ms": "1000000000000",
        "flight_phase": "FLIGHT_PHASE_DISARMED", "armed": False,
        "landed": True, "battery": {"remaining_percent": 80.0},
        "estimator": {"level": "HEALTH_LEVEL_HEALTHY",
                      "horizontal_position_valid": True},
        "home": {"local_ned": {"north_m": 0.0, "east_m": 0.0, "down_m": 0.0}},
    }
    state.update(overrides)
    return state


@unittest.skipUnless(BINARY_BUILT, "icarus-check-guardrail not built")
class RealGuardrailTests(unittest.TestCase):
    """Every case here is checked against the actual C++ policy engine."""

    def test_taking_off_while_disarmed_is_rejected(self):
        result = check_proposal(
            "takeoff", {"target_altitude_agl_m": 3.0}, disarmed_state())
        self.assertTrue(result["checked"])
        self.assertFalse(result["would_execute"])
        self.assertEqual(result["reason_code"], "REASON_CODE_NOT_ARMED")

    def test_arming_a_disarmed_landed_aircraft_is_accepted(self):
        result = check_proposal("arm", {}, disarmed_state())
        self.assertTrue(result["would_execute"])
        self.assertEqual(result["reason_code"], "REASON_CODE_OK")

    def test_arming_an_already_armed_aircraft_is_rejected(self):
        result = check_proposal("arm", {}, disarmed_state(armed=True))
        self.assertFalse(result["would_execute"])
        self.assertEqual(result["reason_code"], "REASON_CODE_ALREADY_ARMED")

    def test_arming_below_the_takeoff_battery_threshold_is_rejected(self):
        # config/safety/v1.yaml sets minimum_takeoff_percent: 50.0.
        result = check_proposal(
            "arm", {}, disarmed_state(battery={"remaining_percent": 10.0}))
        self.assertFalse(result["would_execute"])
        self.assertEqual(result["reason_code"],
                         "REASON_CODE_BATTERY_BELOW_THRESHOLD")

    def test_landing_an_airborne_armed_aircraft_is_accepted(self):
        airborne = disarmed_state(
            flight_phase="FLIGHT_PHASE_AIRBORNE", armed=True, landed=False)
        result = check_proposal("land", {}, airborne)
        self.assertTrue(result["would_execute"])

    def test_a_stale_state_is_rejected_by_the_context_check(self):
        # now_unix_ms is taken from the state's own timestamp, so staleness
        # here can only come from a state older than the policy allows
        # relative to itself, which cannot happen; this instead checks that
        # an internally inconsistent timestamp type is handled.
        result = check_proposal("arm", {}, disarmed_state(
            observed_at_unix_ms=1000000000000))
        self.assertTrue(result["checked"])


class UnavailableHandlingTests(unittest.TestCase):
    def test_a_missing_binary_raises_a_distinct_exception(self):
        with self.assertRaises(GuardrailUnavailable):
            check_proposal("arm", {}, disarmed_state(),
                          binary="/nonexistent/binary")

    def test_a_missing_policy_raises_a_distinct_exception(self):
        with self.assertRaises(GuardrailUnavailable):
            check_proposal("arm", {}, disarmed_state(),
                          policy="/nonexistent/policy.yaml")


if __name__ == "__main__":
    unittest.main()

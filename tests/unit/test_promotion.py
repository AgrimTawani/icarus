import unittest

from python.dcm.promotion import check_autonomous, check_observe


class PromotionGateTests(unittest.TestCase):
    def test_observe_does_not_promote_insufficient_or_unsafe_report(self):
        result = check_observe({"totals": {"comparable_points": 99,
            "counts": {"invalid": 1, "error": 0, "timeout": 0},
            "guardrail_unchecked": 0, "guardrail_rejected": 0},
            "per_action": {"coverage": {"land": {"situations": 1,
                "always_agreed": 1}}}})
        self.assertFalse(result["passed"])
        self.assertFalse(next(c for c in result["checks"]
                              if c["name"] == "held_out_scoreable_points")["passed"])

    def test_observe_passes_only_exact_policy_shape(self):
        result = check_observe({"totals": {"comparable_points": 100,
            "counts": {"invalid": 0, "error": 0, "timeout": 0},
            "guardrail_unchecked": 0, "guardrail_rejected": 0},
            "per_action": {"coverage": {"land": {"situations": 3,
                "always_agreed": 3}}}})
        self.assertTrue(result["passed"])

    def test_autonomous_requires_replayable_complete_mission_set(self):
        result = check_autonomous({"episodes": 45, "closed_loop_episodes": 45,
            "mission_success_rate": 0.95, "safety_interventions": 0})
        self.assertTrue(result["passed"])
        failed = check_autonomous({"episodes": 44, "closed_loop_episodes": 44,
            "mission_success_rate": 1.0, "safety_interventions": 0})
        self.assertFalse(failed["passed"])


if __name__ == "__main__":
    unittest.main()

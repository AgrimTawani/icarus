import json
import tempfile
import unittest
from pathlib import Path

from python.dcm.v2_promotion import decide


def evaluation(path, context, agreement):
    report = {
        "contract_version": context, "episodes": 2, "repeats": 3,
        "per_episode": [{"episode": "a"}, {"episode": "b"}],
        "totals": {"decision_points": 120, "invalid_rate": 0.0,
                   "timeout_rate": 0.0, "error_rate": 0.0,
                   "guardrail_rejection_rate": 0.0,
                   "tool_selection_agreement_rate": agreement,
                   "recovery_rate": 0.5},
        "latency": {"steady_state_medians": {"median_ms": 500}},
        "resource_peaks": {"peak_vram_mib": 4000},
    }
    Path(path).write_text(json.dumps(report))


def live(path):
    report = {"per_episode": [
        {"scenario": "s", "seed": 1}, {"scenario": "s", "seed": 2},
        {"scenario": "s", "seed": 3}],
        "mission_success_rate": 1.0, "safety_interventions": 0,
        "completion_time_ms": [100, 110, 120], "action_count": 12,
        "recovery_success_rate": None}
    Path(path).write_text(json.dumps(report))


class PromotionTests(unittest.TestCase):
    def test_passes_only_with_offline_improvement_and_paired_live_evidence(self):
        with tempfile.TemporaryDirectory() as temp:
            root = Path(temp)
            evaluation(root / "base.json", "v2", 0.7)
            evaluation(root / "candidate.json", "v3", 0.8)
            live(root / "base-live.json")
            live(root / "candidate-live.json")
            _, report = decide(root / "base.json", root / "candidate.json",
                               root / "out", baseline_live_path=root / "base-live.json",
                               candidate_live_path=root / "candidate-live.json")
            self.assertEqual(report["decision"], "promote-v2")

    def test_missing_live_evidence_fails_closed(self):
        with tempfile.TemporaryDirectory() as temp:
            root = Path(temp)
            evaluation(root / "base.json", "v2", 0.7)
            evaluation(root / "candidate.json", "v3", 0.8)
            _, report = decide(root / "base.json", root / "candidate.json",
                               root / "out")
            self.assertEqual(report["decision"], "retain-v1")
            self.assertIn("paired_live_gazebo_episodes",
                          [g["name"] for g in report["gates"] if not g["passed"]])


if __name__ == "__main__":
    unittest.main()

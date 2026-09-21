import hashlib
import json
import tempfile
import unittest
from pathlib import Path

from python.dcm.live_evaluation import score_campaign, score_episode


class LiveEvaluationTests(unittest.TestCase):
    def make_episode(self, root):
        folder = Path(root) / "episode"
        folder.mkdir()
        events = [
            ("episode_start", {}), ("state", {}), ("perception", {}),
            ("dcm_decision", {"status": "timeout", "executed": False}),
            ("dcm_decision", {"status": "executed", "executed": True,
                              "outcome": "SUCCEEDED", "proposal": {"action": "hold"},
                              "expected_action": "hold"}),
            ("action_status", {"action_id": "safety-hold",
                               "state": "ACTION_STATE_ABORTED_BY_SAFETY"}),
            ("episode_end", {}),
        ]
        counts = {}
        stream = folder / "events.jsonl"
        with stream.open("w") as output:
            for seq, (kind, payload) in enumerate(events, 1):
                counts[kind] = counts.get(kind, 0) + 1
                output.write(json.dumps({"seq": seq, "unix_ms": seq * 100,
                    "monotonic_ns": seq * 100, "kind": kind, "payload": payload}) + "\n")
        (folder / "manifest.json").write_text(json.dumps({
            "schema": "icarus.episode.v1", "episode_id": "episode",
            "mission": "unit", "source": "physical", "started_unix_ms": 100,
            "finished_unix_ms": 700, "outcome": "completed",
            "score": {"status": "completed"}, "code_revision": "a" * 40,
            "source_tree_sha256": "b" * 64, "vehicle_id": "icarus-01",
            "privacy": {}, "config_sha256": {}, "stream_errors": [],
            "streams": {"events.jsonl": {"sha256": hashlib.sha256(
                stream.read_bytes()).hexdigest(), "records": len(events), "counts": counts}},
        }))
        return folder

    def test_scores_real_closed_loop_dimensions(self):
        with tempfile.TemporaryDirectory() as root:
            score = score_episode(self.make_episode(root), check_guardrails=False)
            self.assertTrue(score["closed_loop"])
            self.assertTrue(score["mission_success"])
            self.assertEqual(score["action_count"], 1)
            self.assertEqual(score["completion_time_ms"], 600)
            self.assertEqual(score["recovery_success_rate"], 1.0)
            self.assertEqual(score["correct_tool_selection_rate"], 1.0)
            self.assertEqual(score["safety_interventions"], 1)

    def test_campaign_preserves_unmeasured_metrics_as_none(self):
        with tempfile.TemporaryDirectory() as root:
            episode = self.make_episode(root)
            output, report = score_campaign([episode], Path(root) / "reports",
                                            check_guardrails=False)
            self.assertTrue((output / "live_evaluation.json").is_file())
            self.assertEqual(report["mission_success_rate"], 1.0)
            self.assertEqual(report["correct_tool_selection_rate"], 1.0)


if __name__ == "__main__":
    unittest.main()

"""The physical and simulator capture paths must seal the same schema."""

import json
import tempfile
import unittest
from pathlib import Path

from jsonschema.exceptions import ValidationError

from python.dataset_tools.episode import Episode
from python.dataset_tools.replay import verify_complete


class EpisodeSchemaTests(unittest.TestCase):
    def test_physical_source_uses_current_complete_episode_schema(self):
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            episode = Episode(root, "physical-schema-contract", "serial://test", None)
            episode.record("state", {"sequence": "1"})
            episode.record("perception", {"sequence": "1"})
            directory = episode.seal("completed", {"status": "completed"})
            result = verify_complete(directory, check_guardrails=False)
            self.assertEqual(result["source"], "physical")
            self.assertTrue(result["complete"])
            self.assertTrue(result["replay_deterministic"])
            manifest = json.loads((directory / "manifest.json").read_text())
            self.assertEqual(set(manifest["dcm_feedback"]),
                             {"feedback.json", "review_queue.json"})
            self.assertTrue((directory / "feedback.json").is_file())
            self.assertTrue((directory / "review_queue.json").is_file())

    def test_invalid_live_decision_is_rejected_before_recording(self):
        with tempfile.TemporaryDirectory() as temporary:
            episode = Episode(Path(temporary), "m", "test", None)
            before = episode.seq
            with self.assertRaises(ValidationError):
                episode.record("dcm_decision", {"status": "invalid"})
            self.assertEqual(episode.seq, before)
            episode.seal()

    def test_seal_automatically_generates_a_human_review_candidate(self):
        with tempfile.TemporaryDirectory() as temporary:
            episode = Episode(Path(temporary), "m", "test", None)
            for decision in range(1, 4):
                episode.record("dcm_decision", {
                    "decision": decision,
                    "observation": {},
                    "proposal": {"action": "hold", "arguments": {}},
                    "status": "executed",
                    "executed": True,
                    "outcome": "FAILED",
                    "latency_ms": 1.0,
                })
            directory = episode.seal("failed")
            queue = json.loads((directory / "review_queue.json").read_text())
            self.assertEqual(queue["proposals"][0]["id"], "hold")
            self.assertEqual(queue["proposals"][0]["requested_action"],
                             "human_review")


if __name__ == "__main__":
    unittest.main()

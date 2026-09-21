"""The physical and simulator capture paths must seal the same schema."""

import tempfile
import unittest
from pathlib import Path

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


if __name__ == "__main__":
    unittest.main()

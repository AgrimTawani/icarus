import hashlib
import json
import tempfile
import unittest
from pathlib import Path

from python.dataset_tools.regression import build_campaign


class RegressionCampaignTests(unittest.TestCase):
    def episode(self, root, name):
        directory = Path(root) / name
        directory.mkdir()
        events = [("episode_start", {}), ("state", {}), ("perception", {}),
                  ("episode_end", {})]
        counts = {}
        stream = directory / "events.jsonl"
        with stream.open("w") as output:
            for seq, (kind, payload) in enumerate(events, 1):
                counts[kind] = counts.get(kind, 0) + 1
                output.write(json.dumps({"seq": seq, "unix_ms": seq,
                    "monotonic_ns": seq, "kind": kind, "payload": payload}) + "\n")
        (directory / "manifest.json").write_text(json.dumps({
            "schema": "icarus.episode.v1", "episode_id": name,
            "mission": "regression", "source": "physical", "started_unix_ms": 1,
            "finished_unix_ms": 4, "outcome": "passed", "code_revision": "a" * 40,
            "source_tree_sha256": "b" * 64, "vehicle_id": "icarus-01",
            "privacy": {}, "config_sha256": {}, "stream_errors": [], "model": None,
            "streams": {"events.jsonl": {"sha256": hashlib.sha256(
                stream.read_bytes()).hexdigest(), "records": len(events), "counts": counts}},
        }))
        return directory

    def test_campaign_fingerprint_is_independent_of_argument_order(self):
        with tempfile.TemporaryDirectory() as root:
            alpha = self.episode(root, "alpha")
            beta = self.episode(root, "beta")
            one, first = build_campaign([beta, alpha], Path(root) / "out-one",
                                        check_guardrails=False)
            two, second = build_campaign([alpha, beta], Path(root) / "out-two",
                                         check_guardrails=False)
            self.assertTrue((one / "campaign.json").is_file())
            self.assertTrue((two / "campaign.json").is_file())
            self.assertEqual(first["campaign_fingerprint_sha256"],
                             second["campaign_fingerprint_sha256"])
            self.assertEqual([row["episode_id"] for row in first["episodes"]],
                             ["alpha", "beta"])


if __name__ == "__main__":
    unittest.main()

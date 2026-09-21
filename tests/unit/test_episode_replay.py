import hashlib
import json
import tempfile
import unittest
from pathlib import Path

from python.dataset_tools.replay import ReplayError, replay, verify_complete


class EpisodeReplayTests(unittest.TestCase):
    def make_episode(self, directory):
        folder = Path(directory) / "example"
        folder.mkdir()
        events = [
            ("episode_start", {"mission": "unit"}),
            ("state", {"sequence": "1"}),
            ("perception", {"observed_at_unix_ms": "1000"}),
            ("action_request", {"method": "Hold", "request": {}}),
            ("action_receipt", {"method": "Hold", "receipt": {}}),
            ("action_status", {"action_id": "a", "state": "ACTION_STATE_EXECUTING"}),
            ("action_status", {"action_id": "a", "state": "ACTION_STATE_ABORTED_BY_SAFETY",
                                "message": "perception stale: safe hold commanded"}),
            ("episode_end", {"outcome": "passed"}),
        ]
        counts = {}
        path = folder / "events.jsonl"
        with path.open("w") as output:
            for seq, (kind, payload) in enumerate(events, 1):
                counts[kind] = counts.get(kind, 0) + 1
                output.write(json.dumps({
                    "seq": seq, "unix_ms": 1000 + (seq - 1) * 150,
                    "monotonic_ns": seq * 150_000_000,
                    "kind": kind, "payload": payload,
                }) + "\n")
        digest = hashlib.sha256(path.read_bytes()).hexdigest()
        (folder / "manifest.json").write_text(json.dumps({
            "schema": "icarus.episode.v1", "episode_id": "example",
            "mission": "unit", "source": "simulation", "outcome": "passed",
            "config_sha256": {}, "stream_errors": [],
            "streams": {"events.jsonl": {
                "sha256": digest, "records": len(events), "counts": counts,
            }},
        }))
        return folder

    def test_replay_checks_safety_evidence(self):
        with tempfile.TemporaryDirectory() as temporary:
            folder = self.make_episode(temporary)
            result = replay(folder, check_guardrails=False)
            self.assertEqual(result["status"], "passed")
            self.assertTrue(result["safety_replay"][0]["evidence"])

    def test_tampered_stream_fails(self):
        with tempfile.TemporaryDirectory() as temporary:
            folder = self.make_episode(temporary)
            with (folder / "events.jsonl").open("a") as output:
                output.write("{}\n")
            with self.assertRaisesRegex(ReplayError, "hash mismatch"):
                replay(folder, check_guardrails=False)

    def test_missing_state_fails_even_if_resealed(self):
        with tempfile.TemporaryDirectory() as temporary:
            folder = self.make_episode(temporary)
            path = folder / "events.jsonl"
            records = [json.loads(line) for line in path.read_text().splitlines()]
            records = [item for item in records if item["kind"] != "state"]
            for seq, item in enumerate(records, 1):
                item["seq"] = seq
            path.write_text("".join(json.dumps(item) + "\n" for item in records))
            manifest_path = folder / "manifest.json"
            manifest = json.loads(manifest_path.read_text())
            stream = manifest["streams"]["events.jsonl"]
            stream["sha256"] = hashlib.sha256(path.read_bytes()).hexdigest()
            stream["records"] -= 1
            del stream["counts"]["state"]
            manifest_path.write_text(json.dumps(manifest))
            with self.assertRaisesRegex(ReplayError, "stream missing"):
                replay(folder, check_guardrails=False)

    def test_complete_gate_requires_current_provenance_and_is_deterministic(self):
        with tempfile.TemporaryDirectory() as temporary:
            folder = self.make_episode(temporary)
            manifest_path = folder / "manifest.json"
            manifest = json.loads(manifest_path.read_text())
            manifest.update({
                "started_unix_ms": 1_000,
                "finished_unix_ms": 2_100,
                "code_revision": "a" * 40,
                "source_tree_sha256": "b" * 64,
                "vehicle_id": "icarus-01",
                "privacy": {"operator_identifiers": "excluded"},
            })
            # The fixture has no compact simulator files, so it represents a
            # physical episode. Both sources share event/manifest format.
            manifest["source"] = "physical"
            manifest_path.write_text(json.dumps(manifest))
            result = verify_complete(folder, check_guardrails=False)
            self.assertTrue(result["complete"])
            self.assertTrue(result["replay_deterministic"])
            self.assertEqual(len(result["replay_result_sha256"]), 64)

    def test_complete_gate_hash_checks_sensor_snapshot(self):
        with tempfile.TemporaryDirectory() as temporary:
            folder = self.make_episode(temporary)
            sensor = folder / "raw_sensors" / "lidar.pbstream"
            sensor.parent.mkdir()
            sensor.write_bytes(b"compact sensor sample")
            manifest_path = folder / "manifest.json"
            manifest = json.loads(manifest_path.read_text())
            manifest["raw_sensor_snapshot"] = {
                "files": {"raw_sensors/lidar.pbstream": hashlib.sha256(
                    sensor.read_bytes()).hexdigest()}}
            manifest_path.write_text(json.dumps(manifest))
            self.assertEqual(replay(folder, check_guardrails=False)["status"], "passed")
            sensor.write_bytes(b"tampered")
            with self.assertRaisesRegex(ReplayError, "sensor snapshot hash mismatch"):
                replay(folder, check_guardrails=False)


if __name__ == "__main__":
    unittest.main()

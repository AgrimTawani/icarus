"""Phase 10 simulator sensor-snapshot coverage."""

import json
import tempfile
import unittest
from pathlib import Path

from python.dataset_tools.episode import Episode


class EpisodeSensorSnapshotTests(unittest.TestCase):
    def test_seal_copies_bounded_simulator_sensor_streams(self):
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            sensors = root / "sim_run" / "sensors"
            sensors.mkdir(parents=True)
            (sensors / "schema.json").write_text(json.dumps({
                "format": "protobuf stream", "image_pixels_saved": False,
                "channels": {"imu": {}, "rgb": {}},
            }))
            (sensors / "index.jsonl").write_text('{"channel":"imu"}\n')
            (sensors / "imu.pbstream").write_bytes(b"sample")
            episode = Episode(root, "test", "127.0.0.1:50051", {
                "run_directory": str(root / "sim_run"),
                "scenario": "missing_scenario",
            })
            episode.seal("completed", {"test": True})
            manifest = json.loads((episode.directory / "manifest.json").read_text())
            self.assertTrue(manifest["raw_sensor_payloads"])
            snapshot = manifest["raw_sensor_snapshot"]
            self.assertFalse(snapshot["image_pixels_saved"])
            self.assertEqual(snapshot["channels"], ["imu", "rgb"])
            self.assertIn("raw_sensors/imu.pbstream", snapshot["files"])
            self.assertEqual((episode.directory / "raw_sensors/imu.pbstream").read_bytes(), b"sample")

    def test_physical_or_missing_sensor_stream_keeps_snapshot_absent(self):
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            episode = Episode(root, "test", "127.0.0.1:50051", None)
            episode.seal()
            manifest = json.loads((episode.directory / "manifest.json").read_text())
            self.assertFalse(manifest["raw_sensor_payloads"])
            self.assertIsNone(manifest["raw_sensor_snapshot"])


if __name__ == "__main__":
    unittest.main()

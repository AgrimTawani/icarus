import json
import math
import sys
import unittest
from types import ModuleType, SimpleNamespace
from unittest.mock import patch

import numpy as np

from python.perception.depth_feedback import DepthFrameCache, GazeboDepthSource, decode_gazebo_depth


def cache(**kwargs):
    return DepthFrameCache(source="test-camera", simulation_only=False,
                           horizontal_fov_deg=90, minimum_depth_m=0.3,
                           maximum_depth_m=10, calibration="test fixture", **kwargs)


class DepthFeedbackTests(unittest.TestCase):
    def test_gazebo_source_subscribes_once_throttles_and_releases(self):
        nodes = []

        class FakeNode:
            def __init__(self, options):
                self.partition = options.partition
                self.subscriptions = []
                self.unsubscriptions = []
                nodes.append(self)

            def subscribe(self, message_type, topic, callback):
                self.subscriptions.append(topic)
                self.callback = callback
                return True

            def unsubscribe(self, topic):
                self.unsubscriptions.append(topic)

        gz = ModuleType("gz")
        transport = ModuleType("gz.transport13")
        transport.Node, transport.NodeOptions = FakeNode, SimpleNamespace
        gz.transport13 = transport
        msgs = ModuleType("gz.msgs10")
        image = ModuleType("gz.msgs10.image_pb2")
        image.Image = object
        packages = {"gz": gz, "gz.transport13": transport,
                    "gz.msgs10": msgs, "gz.msgs10.image_pb2": image}
        with patch.dict(sys.modules, packages):
            source = GazeboDepthSource({"partition": "isolated-test"})
            self.assertFalse(source.summary()["available"])
            message = SimpleNamespace(
                width=10, height=10, step=40, pixel_format_type=13,
                data=np.full((10, 10), 3, dtype=np.float32).tobytes(),
                header=SimpleNamespace(stamp=SimpleNamespace(sec=2, nsec=3)),
            )
            with patch("python.perception.depth_feedback.time.monotonic", return_value=10):
                nodes[0].callback(message)
                first = source.snapshot()
                nodes[0].callback(message)
                self.assertIs(first, source.snapshot())
            self.assertEqual(source.summary()["center"]["median_optical_z_m"], 3)
            self.assertEqual(source.summary()["sensor_clock"], "gazebo_simulation")
            self.assertEqual(source.summary()["sensor_timestamp_ns"], 2_000_000_003)
            self.assertEqual(nodes[0].partition, "isolated-test")
            self.assertEqual(len(nodes[0].subscriptions), 1)
            source.close()
            self.assertEqual(len(nodes[0].unsubscriptions), 1)
            self.assertFalse(source.summary()["available"])

    def test_continuous_context_contains_cached_depth_and_target_references(self):
        from python.dcm._paths import ensure_generated_proto_on_path
        ensure_generated_proto_on_path()
        from icarus.v1 import drone_api_pb2_grpc as rpc
        from python.dcm.agent_context import capture_context

        frames = cache()
        frames.update(np.full((10, 10), 2.5))
        client = SimpleNamespace(
            state=lambda: {}, session_id="test", channel=None,
            state_api=SimpleNamespace(GetHealth=lambda *a, **k: {}),
            depth_feedback=frames.summary,
            target_feedback=lambda: [{"target_id": "measured-1"}],
        )
        perception = SimpleNamespace(GetPerception=lambda *a, **k: {})
        with patch.object(rpc, "PerceptionServiceStub", return_value=perception):
            result = capture_context(client)
            self.assertEqual(result["depth"]["center"]["median_optical_z_m"], 2.5)
            self.assertEqual(result["targets"], [{"target_id": "measured-1"}])
            client.depth_feedback = lambda: (_ for _ in ()).throw(RuntimeError("offline"))
            result = capture_context(client)
            self.assertFalse(result["depth"]["available"])
            self.assertEqual(result["depth"]["reason"], "offline")

    def test_unavailable_and_invalid_returns_do_not_claim_clear_space(self):
        frames = cache()
        self.assertFalse(frames.summary()["available"])
        depth = np.zeros((10, 10), dtype=np.float32)
        depth[0, :4] = [np.nan, np.inf, -1, 11]
        frames.update(depth, received_at_unix_ms=1000)
        result = frames.summary(1100)
        self.assertFalse(result["available"])
        self.assertIsNone(result["minimum_optical_z_m"])
        self.assertIn("does not establish free space", result["reason"])
        json.dumps(result, allow_nan=False)

    def test_sector_evidence_and_center_do_not_imply_target_identity(self):
        frames = cache()
        depth = np.full((30, 30), 6, dtype=np.float32)
        depth[:, :10] = 2
        depth[:, 20:] = 4
        frames.update(depth, received_at_unix_ms=1000, sensor_timestamp_ns=99)
        result = frames.summary(1300)
        self.assertEqual(result["minimum_optical_z_m"], 2)
        self.assertEqual(result["sectors"]["left"]["median_optical_z_m"], 2)
        self.assertEqual(result["sectors"]["right"]["median_optical_z_m"], 4)
        self.assertEqual(result["center"]["median_optical_z_m"], 6)
        self.assertFalse(result["object_associated"])
        self.assertEqual(result["sensor_timestamp_ns"], 99)
        self.assertEqual(result["received_at_unix_ms"], 1000)
        self.assertFalse(result["simulation_only"])

    def test_old_future_and_failed_frames_are_not_fresh(self):
        frames = cache()
        frames.update(np.ones((10, 10)), received_at_unix_ms=1000)
        self.assertTrue(frames.summary(1750)["fresh"])
        self.assertFalse(frames.summary(1751)["fresh"])
        self.assertFalse(frames.summary(999)["fresh"])
        frames.invalidate("encoding changed")
        self.assertFalse(frames.summary(1100)["available"])
        self.assertFalse(frames.summary(1100)["fresh"])

    def test_snapshot_preserves_original_pixels_and_timestamp_across_updates(self):
        frames = cache()
        depth = np.full((10, 10), 3, dtype=np.float32)
        frames.update(depth, received_at_unix_ms=1000)
        original = frames.snapshot()
        depth[:] = 8
        frames.update(depth, received_at_unix_ms=2000)
        result = original.region([0, 0, 500, 500], 2100)
        self.assertEqual(result["median_optical_z_m"], 3)
        self.assertEqual(result["received_at_unix_ms"], 1000)
        self.assertFalse(result["fresh"])
        self.assertEqual(frames.summary(2100)["median_optical_z_m"], 8)
        with self.assertRaises(ValueError):
            original.depth_m[0, 0] = 5

    def test_radial_range_differs_from_optical_depth_off_axis(self):
        frames = cache()
        frames.update(np.full((10, 10), 3), received_at_unix_ms=1000)
        result = frames.snapshot().region([0, 0, 100, 100], 1000)
        self.assertEqual(result["minimum_optical_z_m"], 3)
        self.assertAlmostEqual(result["minimum_radial_range_m"],
                               3 * math.sqrt(1 + 0.9 ** 2 + 0.9 ** 2), places=5)

    def test_region_and_calibration_validation(self):
        frames = cache()
        frames.update(np.ones((10, 10)), received_at_unix_ms=1000)
        for box in ([0, 0, 0, 1], [-1, 0, 1, 1], [0, 0, 1001, 1],
                    [0, 2, 1, 1], [0, 0, float("nan"), 1]):
            with self.subTest(box=box), self.assertRaises(ValueError):
                frames.snapshot().region(box)
        with self.assertRaises(ValueError):
            cache(maximum_age_ms=0)
        with self.assertRaises(ValueError):
            frames.update(np.ones((2, 2)))

    def test_decode_gazebo_padded_rows_and_reject_color_bytes(self):
        depth = np.arange(100, dtype=np.float32).reshape(10, 10)
        payload = b"".join(row.tobytes() + b"pad!" for row in depth)
        message = SimpleNamespace(width=10, height=10, step=44,
                                  pixel_format_type=13, data=payload)
        np.testing.assert_array_equal(decode_gazebo_depth(message), depth)
        message.pixel_format_type = 7
        with self.assertRaises(ValueError):
            decode_gazebo_depth(message)
        message.pixel_format_type = 13
        message.data = payload[:-1]
        with self.assertRaises(ValueError):
            decode_gazebo_depth(message)


if __name__ == "__main__":
    unittest.main()

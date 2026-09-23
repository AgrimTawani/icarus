import math
import unittest

from python.simulation.airsim_adapter import normalize_snapshot
from scripts.simulation.airsim.sensor_bridge import HEADER, POINT, encode_packet


class AirSimAdapterTests(unittest.TestCase):
    def test_normalization_preserves_contract_and_excludes_truth(self):
        now = 2_000
        snapshot = normalize_snapshot(
            {"timestamp": 1_000_000_000, "landed_state": 1,
             "collision": {"has_collided": True}},
            [{"time_stamp": 1_000_000_000, "image_type": 0,
              "width": 256, "height": 144, "image_data_uint8": b"raw"}],
            {"time_stamp": 1_000_000_000,
             "point_cloud": [3.0, 4.0, 0.0]}, now_unix_ms=now)
        self.assertEqual(snapshot["frame"], "NED")
        self.assertEqual(snapshot["perception"]["nearest_obstacle_distance_m"], 5.0)
        encoded = repr(snapshot)
        self.assertNotIn("image_data_uint8", encoded)
        self.assertNotIn("has_collided", encoded)
        self.assertIn("collision", snapshot["truth_fields_excluded"])

    def test_bridge_packet_filters_nonfinite_points_and_is_bounded(self):
        packet = encode_packet([1.0, 2.0, 3.0, math.nan, 0.0, 0.0], 100, 1)
        self.assertEqual(len(packet), HEADER.size + POINT.size)
        self.assertEqual(HEADER.unpack_from(packet)[2], 1)


if __name__ == "__main__":
    unittest.main()

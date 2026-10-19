"""Zero-valued proto3 scalars that carry meaning must survive dict conversion.

Episode 20260930T024405_8d41e216673d aborted two valid missions because a fresh
LiDAR map reported local_map_age_ms == 0, MessageToDict omitted the proto3
default, and assess_freshness then saw "perception has no map age".
"""

import sys
import unittest
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[2]
                       / "build/generated/python"))

from icarus.v1 import perception_pb2, state_pb2

from python.dataset_tools.episode import _replay_message, _safe_message
from python.dcm.contract import FRESHNESS_LIMITS, assess_freshness

NOW = 1_789_827_075_269


def _observation(perception):
    return {
        "observed_at_unix_ms": NOW,
        "state": {"observed_at_unix_ms": NOW - 20},
        "perception": perception,
    }


class ZeroMeaningfulFieldTests(unittest.TestCase):
    def test_zero_map_age_is_kept(self):
        summary = perception_pb2.PerceptionSummary(
            observed_at_unix_ms=NOW, local_map_available=True,
            local_map_age_ms=0, path_ahead_clear=False, sequence=7)
        for convert in (_safe_message, _replay_message):
            with self.subTest(convert=convert.__name__):
                value = convert(summary)
                self.assertEqual(value["local_map_age_ms"], 0)
                self.assertIs(value["path_ahead_clear"], False)
                self.assertIs(value["local_map_available"], True)

    def test_unlisted_defaults_stay_omitted(self):
        value = _safe_message(perception_pb2.PerceptionSummary(
            observed_at_unix_ms=NOW, local_map_available=True))
        self.assertNotIn("detections", value)
        self.assertNotIn("nearest_obstacle_distance_m", value)
        self.assertNotIn("landing_zone_quality", value)

    def test_nested_zero_ages_are_kept(self):
        state = state_pb2.DroneState(
            mavlink=state_pb2.LinkState(connected=True, heartbeat_age_ms=0),
            sources=[state_pb2.SourceHealth(source_id="lidar", age_ms=0),
                     state_pb2.SourceHealth(source_id="gps", age_ms=40)])
        value = _safe_message(state)
        self.assertEqual(value["mavlink"]["heartbeat_age_ms"], 0)
        self.assertEqual([s["age_ms"] for s in value["sources"]], [0, 40])
        # An unset nested message is not materialised.
        self.assertNotIn("local_position_ned", value)


class MapAgeFreshnessTests(unittest.TestCase):
    def test_zero_map_age_from_protobuf_is_fresh(self):
        summary = perception_pb2.PerceptionSummary(
            observed_at_unix_ms=NOW, local_map_available=True,
            local_map_age_ms=0, path_ahead_clear=True, sequence=3)
        self.assertIsNone(assess_freshness(_observation(_safe_message(summary))))

    def test_zero_map_age_dict_is_fresh(self):
        self.assertIsNone(assess_freshness(
            _observation({"local_map_age_ms": 0, "local_map_available": True})))

    def test_missing_map_age_is_stale(self):
        reason = assess_freshness(_observation({"local_map_available": True}))
        self.assertEqual(reason, "perception has no map age")

    def test_stale_map_age_is_stale(self):
        limit = FRESHNESS_LIMITS["perception_age_ms"]
        reason = assess_freshness(_observation(_safe_message(
            perception_pb2.PerceptionSummary(
                observed_at_unix_ms=NOW, local_map_age_ms=limit + 1))))
        self.assertIn("perception map is", reason)
        # The C++ producer reports UINT32_MAX when no map was ever observed.
        reason = assess_freshness(_observation(_safe_message(
            perception_pb2.PerceptionSummary(local_map_age_ms=2**32 - 1))))
        self.assertIn("perception map is", reason)


if __name__ == "__main__":
    unittest.main()

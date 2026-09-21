import json
import math
import tempfile
import unittest
from pathlib import Path

import numpy as np

from python.perception.landing_zone import assess_landing_zone, load_landing_source


def flat_depth(height=80, width=100, distance=4.0):
    return np.full((height, width), distance, dtype=np.float32)


class LandingZoneTests(unittest.TestCase):
    def test_versioned_source_configuration_is_loaded(self):
        root = Path(__file__).resolve().parents[2]
        source = load_landing_source(root / "config/perception/landing_zone.json")
        self.assertEqual(source["sensor_id"], "forward_rgbd")
        self.assertEqual(source["optical_axis_body_frd"], [1.0, 0.0, 0.0])

    def test_invalid_source_configuration_is_rejected(self):
        with tempfile.TemporaryDirectory() as temporary:
            path = Path(temporary) / "source.json"
            path.write_text(json.dumps({"version": 1, "active_source": {}}))
            with self.assertRaises(ValueError):
                load_landing_source(path)

    def test_flat_downward_frame_is_suitable(self):
        result = assess_landing_zone(flat_depth(), optical_axis_body_frd=(0, 0, 1))
        self.assertTrue(result["assessable"])
        self.assertTrue(result["suitable"])
        self.assertLess(result["slope_deg"], 0.001)
        self.assertLess(result["roughness_m"], 0.001)

    def test_forward_camera_is_refused_not_scored(self):
        result = assess_landing_zone(flat_depth(), optical_axis_body_frd=(1, 0, 0))
        self.assertFalse(result["assessable"])
        self.assertFalse(result["suitable"])
        self.assertIn("not calibrated downward", result["reason"])

    def test_rough_surface_is_not_suitable(self):
        depth = flat_depth()
        depth[25:55:2, 25:75:2] += 0.25
        result = assess_landing_zone(depth, optical_axis_body_frd=(0, 0, 1))
        self.assertTrue(result["assessable"])
        self.assertFalse(result["suitable"])
        self.assertGreater(result["roughness_m"], 0.04)

    def test_footprint_radius_changes_the_samples_considered(self):
        depth = flat_depth()
        # A rough feature is away from the centre. A small airframe footprint
        # does not include it; a larger required footprint does.
        depth[35:45, 61:69] += 0.35
        small = assess_landing_zone(
            depth, optical_axis_body_frd=(0, 0, 1), vehicle_radius_m=0.5,
            required_clearance_m=0.0)
        large = assess_landing_zone(
            depth, optical_axis_body_frd=(0, 0, 1), vehicle_radius_m=1.5,
            required_clearance_m=0.0)
        self.assertTrue(small["suitable"])
        self.assertFalse(large["suitable"])

    def test_invalid_depth_does_not_become_suitable(self):
        depth = flat_depth()
        depth[:] = math.nan
        result = assess_landing_zone(depth, optical_axis_body_frd=(0, 0, 1))
        self.assertTrue(result["assessable"])
        self.assertFalse(result["suitable"])
        self.assertIn("insufficient", result["reason"])


if __name__ == "__main__":
    unittest.main()

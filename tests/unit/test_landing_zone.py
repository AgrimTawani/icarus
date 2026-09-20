import math
import unittest

import numpy as np

from python.perception.landing_zone import assess_landing_zone


def flat_depth(height=80, width=100, distance=4.0):
    return np.full((height, width), distance, dtype=np.float32)


class LandingZoneTests(unittest.TestCase):
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

    def test_invalid_depth_does_not_become_suitable(self):
        depth = flat_depth()
        depth[:] = math.nan
        result = assess_landing_zone(depth, optical_axis_body_frd=(0, 0, 1))
        self.assertTrue(result["assessable"])
        self.assertFalse(result["suitable"])
        self.assertIn("insufficient", result["reason"])


if __name__ == "__main__":
    unittest.main()

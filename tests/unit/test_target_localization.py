"""Pure geometry tests for measured target localization (no simulator)."""

import math
import unittest
from pathlib import Path

from python.perception.target_localization import (
    CameraModel,
    TargetLocalizer,
    associate,
    box_to_pixels,
    candidates_from_perception,
    convex_hull,
    estimate_footprint,
    load_localization_config,
    normalize_description,
    parse_grounding_box,
    project_candidate,
    visual_grounding_description,
)

ROOT = Path(__file__).resolve().parents[2]
CONFIG = load_localization_config(ROOT / "config/perception/target_localization.json")


def _obstacle(obstacle_id, hull, top_down=-6.0, bottom_down=-3.0, points=120,
              clipped=False, observed_at=1_000):
    north = sum(p[0] for p in hull) / len(hull)
    east = sum(p[1] for p in hull) / len(hull)
    return {"obstacle_id": obstacle_id, "sensor_id": "mid360",
            "point_count": points, "top_down_m": top_down,
            "bottom_down_m": bottom_down, "top_clipped": clipped,
            "observed_at_unix_ms": observed_at,
            "centroid": {"north_m": north, "east_m": east},
            "hull": [{"north_m": n, "east_m": e, "down_m": top_down} for n, e in hull]}


def _state(north=0.0, east=0.0, down=-5.0, yaw_deg=0.0):
    return {"relative_altitude_m": -down,
            "local_position_ned": {"north_m": north, "east_m": east,
                                   "down_m": down, "origin_id": "ekf-origin"},
            "attitude": {"yaw_rad": math.radians(yaw_deg)}}


# A building face 14 m north spanning east -4..4, a wall far west and a tree.
FACE = [(14.0, -4.0), (14.0, 4.0)]
WALL = [(10.0, -19.4), (10.0, -14.9)]
TREE = [(28.0, 13.0), (30.0, 16.4), (29.0, 14.0)]


def _perception(*obstacles, observed_at=1_000):
    return {"local_map_available": True, "observed_at_unix_ms": observed_at,
            "nearby_obstacles": list(obstacles)}


class CameraModelTests(unittest.TestCase):
    def setUp(self):
        self.camera = CameraModel.from_config(CONFIG["camera"])

    def test_centre_and_edges_map_to_heading_and_half_fov(self):
        self.assertAlmostEqual(self.camera.relative_bearing_deg(424), 0.0)
        self.assertAlmostEqual(self.camera.relative_bearing_deg(0), -43.065, places=2)
        self.assertAlmostEqual(self.camera.relative_bearing_deg(848), 43.065, places=2)

    def test_column_round_trip(self):
        for bearing in (-40.0, -10.0, 0.0, 12.5, 42.0):
            column = self.camera.column_px(bearing)
            self.assertAlmostEqual(self.camera.relative_bearing_deg(column), bearing)
        self.assertIsNone(self.camera.column_px(120.0))


class ParsingTests(unittest.TestCase):
    def test_qwen_grounding_box_is_parsed(self):
        self.assertEqual(parse_grounding_box("(319,402),(648,859)<|box_end|>"),
                         (319, 402, 648, 859))
        self.assertEqual(parse_grounding_box("<|box_start|>( 1, 2 ), (30,40)"),
                         (1, 2, 30, 40))

    def test_degenerate_or_out_of_range_boxes_are_rejected(self):
        for text in ("NONE", "", None, "(10,10),(5,20)", "(0,0),(1001,20)",
                     "(200,500),(800,500)"):
            with self.subTest(text=text):
                self.assertIsNone(parse_grounding_box(text))

    def test_description_is_bounded(self):
        self.assertEqual(normalize_description("  the  building "), "the building")
        for value in ("", None, "x" * 121):
            with self.subTest(value=value), self.assertRaises(ValueError):
                normalize_description(value)

    def test_forward_view_selector_is_not_sent_as_object_attribute(self):
        for description, expected in (
                ("building in front", "building"),
                ("the red truck directly ahead", "the red truck"),
                ("warehouse in front of us", "warehouse"),
                ("a tower visible ahead", "a tower")):
            with self.subTest(description=description):
                parsed = visual_grounding_description(description)
                self.assertEqual(parsed["description"], description)
                self.assertEqual(parsed["visual_query"], expected)
                self.assertEqual(parsed["view"], "forward_d435i")

    def test_object_to_object_relations_are_preserved(self):
        for description in ("building in front of the church",
                            "car ahead of the bus",
                            "tree behind the shed"):
            with self.subTest(description=description):
                parsed = visual_grounding_description(description)
                self.assertEqual(parsed["visual_query"], description)
                self.assertIsNone(parsed["view"])


class AssociationTests(unittest.TestCase):
    def setUp(self):
        self.camera = CameraModel.from_config(CONFIG["camera"])
        self.candidates = candidates_from_perception(_perception(
            _obstacle("wall", WALL, top_down=-3.0, bottom_down=-2.4, points=12),
            _obstacle("face", FACE), _obstacle("tree", TREE, top_down=-7.1,
                                               bottom_down=-1.0, points=40)))
        self.pose = {"north_m": 0.0, "east_m": 0.0, "down_m": -5.0,
                     "yaw_rad": 0.0, "pitch_rad": 0.0}

    def test_projection_matches_face_columns(self):
        face = next(c for c in self.candidates if c["id"] == "face")
        projection = project_candidate(face, self.pose, self.camera)
        x1, y1, x2, y2 = projection["box_px"]
        # tan(15.95 deg) * 454.6 px focal length about the 424 px centre.
        self.assertAlmostEqual(x1, 424 - 454.62 * 4 / 14, delta=1.0)
        self.assertAlmostEqual(x2, 424 + 454.62 * 4 / 14, delta=1.0)
        self.assertLess(y1, 240)  # top above the horizon (building taller than hover)
        self.assertGreater(y2, 380)  # base assumed on terrain below
        self.assertAlmostEqual(projection["surface_range_m"], 14.0)

    def test_vlm_box_selects_overlapping_cluster(self):
        box = box_to_pixels((319, 402, 648, 859), self.camera)
        best, ranked = associate(box, self.candidates, self.pose, self.camera,
                                 CONFIG["association"])
        self.assertEqual(best["candidate"]["id"], "face")
        self.assertGreater(best["image_iou"], 0.6)
        self.assertEqual(ranked[0]["candidate"]["id"], "face")

    def test_clusters_out_of_view_are_not_candidates(self):
        pose = dict(self.pose, yaw_rad=math.radians(180.0))
        box = box_to_pixels((319, 402, 648, 859), self.camera)
        best, ranked = associate(box, self.candidates, pose, self.camera,
                                 CONFIG["association"])
        self.assertIsNone(best)
        self.assertEqual(ranked, [])

    def test_box_without_lidar_support_is_not_associated(self):
        # A region straight up in the sky: no cluster projects there.
        box = box_to_pixels((450, 0, 550, 100), self.camera)
        best, _ = associate(box, self.candidates, self.pose, self.camera,
                            CONFIG["association"])
        self.assertIsNone(best)


class FootprintTests(unittest.TestCase):
    def test_single_face_reports_only_observed_extent_without_inventing_depth(self):
        footprint = estimate_footprint(FACE, [(0.0, 0.0)], CONFIG["footprint"])
        self.assertFalse(footprint["depth_observed"])
        self.assertNotIn("assumption", footprint)
        self.assertNotIn("assumed_outline", footprint)
        self.assertAlmostEqual(footprint["centre"][0], 14.0)
        self.assertAlmostEqual(footprint["centre"][1], 0.0)
        self.assertAlmostEqual(footprint["radius_m"], 4.0)
        self.assertEqual(footprint["extent"], {"min_north_m": 14, "max_north_m": 14,
                                             "min_east_m": -4, "max_east_m": 4})
        self.assertEqual(footprint["hull"], sorted(FACE))

    def test_observed_extent_does_not_depend_on_which_side_the_viewer_occupies(self):
        front = estimate_footprint(FACE, [(0.0, 0.0)], CONFIG["footprint"])
        back = estimate_footprint(FACE, [(30.0, 0.0)], CONFIG["footprint"])
        self.assertEqual(front, back)

    def test_rotated_thin_face_bounds_contain_only_measured_returns(self):
        points = [(2.0, 3.0), (8.0, 9.0)]
        footprint = estimate_footprint(points, [(0, 10)], CONFIG["footprint"])
        self.assertFalse(footprint["depth_observed"])
        self.assertEqual(footprint["extent"], {"min_north_m": 2, "max_north_m": 8,
                                             "min_east_m": 3, "max_east_m": 9})
        self.assertAlmostEqual(footprint["radius_m"], math.sqrt(18))
        self.assertAlmostEqual(footprint["centre"][0], 5.0)
        self.assertAlmostEqual(footprint["centre"][1], 6.0)

    def test_two_faces_observe_depth_without_assumption(self):
        corner = [(14.0, -4.0), (14.0, 4.0), (22.0, 4.0)]
        footprint = estimate_footprint(corner, [(0.0, 10.0)], CONFIG["footprint"])
        self.assertTrue(footprint["depth_observed"])
        self.assertNotIn("assumption", footprint)
        self.assertAlmostEqual(footprint["centre"][0], 18.0)
        self.assertAlmostEqual(footprint["centre"][1], 0.0)

    def test_convex_hull_drops_interior_points(self):
        square = [(0, 0), (0, 2), (2, 2), (2, 0), (1, 1)]
        self.assertEqual(len(convex_hull(square)), 4)


class LocalizerTests(unittest.TestCase):
    def setUp(self):
        self.now = [100.0]
        self.localizer = TargetLocalizer(CONFIG, clock=lambda: self.now[0])
        self.perception = _perception(_obstacle("face", FACE, observed_at=99_900),
                                      _obstacle("wall", WALL, points=12,
                                                observed_at=99_900),
                                      observed_at=99_900)

    def _observe(self, text, yaw_deg=0.0, perception=None):
        return self.localizer.observe(
            "building behind the group of people", vlm_text=text,
            state=_state(yaw_deg=yaw_deg), perception=perception or self.perception,
            image_captured_at_unix_ms=99_900)

    def test_found_estimate_is_measured_and_has_no_confidence(self):
        estimate = self._observe("(319,402),(648,859)")
        self.assertTrue(estimate["found"])
        self.assertEqual(estimate["surface_range_m"], 14.0)
        self.assertAlmostEqual(estimate["estimated_position"]["north_m"], 14.0)
        self.assertEqual(estimate["position_basis"], "center_of_observed_lidar_extent")
        self.assertNotIn("assumed_outline", estimate["footprint"])
        self.assertFalse(estimate["footprint"]["depth_observed"])
        self.assertEqual(estimate["footprint"]["radius_m"], 4.0)
        self.assertAlmostEqual(estimate["bearing_deg"]["absolute"], 0.0)
        self.assertEqual(estimate["quality"]["lidar_returns"], 120)
        self.assertEqual(estimate["quality"]["viewpoints"], 1)
        self.assertFalse(estimate["quality"]["viewpoints_agree"])
        self.assertEqual(estimate["quality"]["age_ms"], 100)
        self.assertFalse(estimate["flight_authority"])
        self.assertNotIn("confidence", str(estimate))
        self.assertEqual(estimate["footprint"]["height_m"], 6.0)

    def test_repeated_views_agree_and_report_spread(self):
        self._observe("(319,402),(648,859)")
        self.now[0] += 20
        estimate = self._observe("(152,397),(512,897)", yaw_deg=15.0)
        self.assertTrue(estimate["found"])
        self.assertEqual(estimate["quality"]["viewpoints"], 2)
        self.assertTrue(estimate["quality"]["viewpoints_agree"])
        self.assertEqual(estimate["quality"]["position_spread_m"], 0.0)

    def test_vlm_none_is_not_found(self):
        estimate = self._observe(None)
        self.assertFalse(estimate["found"])
        self.assertEqual(estimate["reason"], "vlm_reported_no_match")
        self.assertNotIn("estimated_position", estimate)

    def test_selected_region_without_lidar_is_reported_as_bearing_only(self):
        estimate = self._observe("(450,0),(550,100)")
        self.assertFalse(estimate["found"])
        self.assertEqual(estimate["reason"], "no_lidar_cluster_in_selected_region")
        self.assertIn("image_relative", estimate["bearing_deg"])
        self.assertNotIn("estimated_position", estimate)

    def test_clipped_top_reports_only_a_lower_bound(self):
        perception = _perception(_obstacle("face", FACE, clipped=True,
                                           observed_at=99_900), observed_at=99_900)
        estimate = self._observe("(319,402),(648,859)", perception=perception)
        self.assertIsNone(estimate["footprint"]["height_m"])
        self.assertTrue(estimate["footprint"]["height_clipped"])
        self.assertEqual(estimate["footprint"]["height_lower_bound_m"], 6.0)

    def test_unavailable_lidar_is_not_found(self):
        estimate = self._observe("(319,402),(648,859)",
                                 perception={"local_map_available": False})
        self.assertFalse(estimate["found"])
        self.assertEqual(estimate["reason"], "lidar_perception_unavailable")


if __name__ == "__main__":
    unittest.main()

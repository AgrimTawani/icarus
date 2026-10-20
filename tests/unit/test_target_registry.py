"""Static target identity and range contracts; no simulator or model needed."""
import unittest
import math

from python.perception.target_registry import TargetRegistry


def point(n, e, origin="origin-A"):
    return {"north_m": n, "east_m": e, "origin_id": origin, "frame": "local_ned"}


def estimate(hull=None, *, depth=True, assumed=None, origin="origin-A"):
    hull = hull or [(10, -2), (12, -2), (12, 2), (10, 2)]
    return {"found": True, "description": "an object", "observed_at_unix_ms": 10000,
            "estimated_position": point(11, 0, origin),
            "surface_range_m": 10, "surface_point": point(10, 0, origin),
            "footprint": {"hull": [point(*p, origin) for p in hull],
                          "depth_observed": depth, "assumed_outline": assumed,
                          "height_m": 6, "height_clipped": False,
                          "height_reference": "home_arming_altitude"},
            "lidar_cluster_id": "transient-1", "quality": {"age_ms": 0,
            "viewpoints": 2, "position_spread_m": 0.1}}


def state(n=0, e=0, origin="origin-A"):
    return {"local_position_ned": point(n, e, origin) | {"down_m": -5},
            "relative_altitude_m": 5, "attitude": {"yaw_rad": 0}}


def obstacle(hull=None, *, stamp=10000, identifier="transient-2"):
    return {"hull": [point(*p) for p in (hull or [(10, -2), (12, -2), (12, 2), (10, 2)])],
            "observed_at_unix_ms": stamp, "point_count": 50,
            "obstacle_id": identifier, "sensor_id": "lidar", "top_down_m": -6.0,
            "bottom_down_m": -2.0}


def perception(*objects, stamp=10000):
    return {"local_map_available": True, "observed_at_unix_ms": stamp,
            "nearby_obstacles": list(objects)}


def static_reference_fixture():
    now = [10.0]
    registry = TargetRegistry(clock=lambda: now[0])
    source = estimate()
    source['vehicle_at_capture'] = {'north_m': 0, 'east_m': 0}
    target_id = registry.register(source)['target_id']
    registry.refresh(target_id, perception(obstacle()), state(n=12, e=8))
    assert registry.get(target_id)['geometry']['orbit_usable']
    return registry, target_id, now


def test_static_anchor_survives_language_latency_and_loss_of_view_without_fresh_range():
    registry, target_id, now = static_reference_fixture()
    now[0] = 30
    live = registry.refresh(target_id, perception(stamp=30000), state())
    assert live['found'] is False and live['reason'] == 'target_not_observed'
    assert live['geometry']['height_m'] is None
    anchor = registry.anchor(target_id, state(n=11))
    assert anchor['reference_available'] is True
    assert anchor['current_observation_available'] is False
    assert anchor['geometry']['orbit_usable'] is True
    assert anchor['footprint']['height_m'] == 6
    assert anchor['reference_age_ms'] == 20000
    assert anchor['geometry']['height_evidence_age_ms'] == 20000
    assert 'surface_range_m' not in anchor
    summary = registry.summaries(state())[0]
    assert summary['reference_available'] and summary['geometry']['orbit_usable']
    assert 'found' not in summary and 'surface_range_m' not in summary
    assert registry.maximum_age_ms == 1500


def test_static_height_uses_local_top_and_rebases_when_home_changes():
    registry, target_id, now = static_reference_fixture()
    now[0] = 45
    current = state()
    current['local_position_ned']['down_m'] = -9
    current['relative_altitude_m'] = 11  # Home is two metres below the local origin.
    anchor = registry.anchor(target_id, current)
    assert anchor['footprint']['height_m'] == 8
    assert anchor['geometry']['height_reference'] == 'current_home_from_local_ned'
    assert current['relative_altitude_m'] - anchor['footprint']['height_m'] == 3


def test_clipped_refresh_does_not_erase_a_previously_measured_static_top():
    registry, target_id, now = static_reference_fixture()
    now[0] = 11
    clipped = obstacle(stamp=11000)
    clipped.update(top_clipped=True, top_down_m=-3)
    registry.refresh(target_id, perception(clipped, stamp=11000), state())
    anchor = registry.anchor(target_id, state())
    assert anchor['footprint']['height_m'] == 6
    assert anchor['geometry']['height_evidence_age_ms'] == 1000


def test_static_reference_does_not_survive_an_origin_change_or_transfer_sessions():
    registry, target_id, now = static_reference_fixture()
    assert registry.anchor(target_id, state(origin='other'))['reference_available'] is False
    registry.refresh(target_id, perception(obstacle()), state(origin='other'))
    assert registry.anchor(target_id, state())['reference_available'] is False
    assert TargetRegistry().anchor(target_id, state())['reason'] == 'unknown_target_id'
    replacement = registry.register(estimate())['target_id']
    assert replacement != target_id


def test_unknown_or_future_height_is_not_invented_for_a_static_reference():
    now = [10.0]
    registry = TargetRegistry(clock=lambda: now[0])
    source = estimate()
    reference = registry.register(source)['target_id']
    assert registry.anchor(reference, state())['footprint']['height_m'] is None
    source['observed_at_unix_ms'] = 20000
    reference = registry.register(source, new_target=True)['target_id']
    assert registry.anchor(reference, state())['reference_available'] is False


class TargetRegistryTests(unittest.TestCase):
    def setUp(self):
        self.now = 10
        self.registry = TargetRegistry(clock=lambda: self.now)

    def test_id_owned_by_session_and_copies_cannot_mutate_registry(self):
        first = self.registry.register(estimate())
        reference = first["target_id"]
        first["geometry"]["center"]["north_m"] = -900
        self.assertEqual(self.registry.get(reference)["geometry"]["center"]["north_m"], 11)
        foreign = TargetRegistry(clock=lambda: self.now).get(reference)
        self.assertFalse(foreign["found"])
        self.assertEqual(foreign["reason"], "unknown_target_id")

    def test_geometric_identity_ignores_description_and_cluster_id(self):
        first = self.registry.register(estimate())
        changed = estimate()
        changed.update(description="totally different words", lidar_cluster_id="other")
        second = self.registry.register(changed)
        self.assertEqual(first["target_id"], second["target_id"])
        self.assertNotEqual(first["target_id"], self.registry.register(changed, new_target=True)["target_id"])

    def test_range_is_current_surface_distance_not_center_radius(self):
        reference = self.registry.register(estimate())["target_id"]
        result = self.registry.refresh(reference, perception(obstacle()), state(n=7))
        self.assertTrue(result["found"])
        self.assertEqual(result["surface_range_m"], 3)
        self.assertEqual(result["surface_point"]["north_m"], 10)
        self.assertEqual(result["lidar_cluster_id"], "transient-2")
        self.assertEqual(result["geometry"]["height_m"], 6)
        self.assertFalse(result["geometry"]["whole_shape_observed"])

    def test_single_face_does_not_become_assumed_orbit(self):
        result = self.registry.register(estimate([(10, -4), (10, 4)], depth=False,
                                               assumed=[point(18, -4), point(18, 4)]))
        self.assertFalse(result["geometry"]["orbit_usable"])
        self.assertTrue(result["geometry"]["requires_more_views"])
        self.assertEqual(result["geometry"]["center"]["north_m"], 10)
        self.assertEqual(len(result["geometry"]["observed_hull"]), 2)

    def test_interior_centroid_of_triangle_is_not_assumed_rectangle_center(self):
        result = self.registry.register(estimate([(0, 0), (9, 0), (0, 3)]))
        self.assertEqual(result["geometry"]["center"]["north_m"], 3)
        self.assertEqual(result["geometry"]["center"]["east_m"], 1)
        self.assertFalse(result["geometry"]["orbit_usable"])
        self.assertTrue(result["geometry"]["requires_more_views"])

    def test_refresh_retains_old_measurements_without_merging_nearby_object(self):
        reference = self.registry.register(estimate())["target_id"]
        near = obstacle([(12, 2), (15, 2), (15, 4), (12, 4)], identifier="neighbor")
        result = self.registry.refresh(reference, perception(near, obstacle()), state())
        self.assertTrue(result["found"])
        self.assertLessEqual(max(p["north_m"] for p in result["geometry"]["observed_hull"]), 12)

    def test_partial_face_remains_associated_with_large_accumulated_hull(self):
        reference = self.registry.register(estimate([(10, -4), (18, -4), (18, 4), (10, 4)]))["target_id"]
        result = self.registry.refresh(reference,
            perception(obstacle([(10, -4), (10, 4)])), state(n=7))
        self.assertTrue(result["found"])
        self.assertEqual(result["surface_range_m"], 3)
        self.assertEqual(len(result["geometry"]["observed_hull"]), 4)

    def test_corner_contact_is_not_identity(self):
        reference = self.registry.register(estimate([(10, -4), (10, 4)], depth=False))["target_id"]
        result = self.registry.refresh(reference, perception(obstacle([(10, 4), (18, 4)])), state())
        self.assertFalse(result["found"])
        self.assertEqual(result["reason"], "target_not_observed")

    def test_ambiguous_observations_do_not_transfer_reference(self):
        reference = self.registry.register(estimate())["target_id"]
        result = self.registry.refresh(reference,
            perception(obstacle(identifier="a"), obstacle(identifier="b")), state())
        self.assertFalse(result["found"])
        self.assertEqual(result["reason"], "target_association_ambiguous")
        self.assertEqual(result["lidar_cluster_id"], "transient-1")

    def test_missing_stale_or_wrong_origin_are_explicit(self):
        reference = self.registry.register(estimate())["target_id"]
        cases = [(perception(), state(), "target_not_observed"),
                 (perception(obstacle(stamp=1)), state(), "target_not_observed"),
                 (perception(obstacle(), stamp=1), state(), "lidar_perception_stale"),
                 (perception(obstacle()), state(origin="B"), "target_origin_changed"),
                 (perception(obstacle()) | {"origin_id": "B"}, state(), "perception_origin_mismatch")]
        for sensed, pose, reason in cases:
            with self.subTest(reason=reason):
                result = self.registry.refresh(reference, sensed, pose)
                self.assertFalse(result["found"])
                self.assertEqual(result["reason"], reason)
        self.assertTrue(self.registry.refresh(reference, perception(obstacle()), state())["found"])

    def test_staleness_visible_without_refresh(self):
        reference = self.registry.register(estimate())["target_id"]
        self.now = 12
        result = self.registry.get(reference)
        self.assertFalse(result["found"])
        self.assertFalse(result["geometry"]["orbit_usable"])
        self.assertEqual(result["reason"], "target_geometry_stale")
        self.assertEqual(self.registry.summaries()[0]["tracking"]["geometry_age_ms"], 2000)

    def test_fused_depth_evidence_survives_later_partial_view(self):
        source = estimate()
        source["vehicle_at_capture"] = {"north_m": 0, "east_m": 0}
        first = self.registry.register(source)
        side = estimate([(10, -2), (10, 2)], depth=False,
                        assumed=[point(14, 2), point(14, -2)])
        side["vehicle_at_capture"] = {"north_m": 11, "east_m": -6}
        partial = self.registry.register(side)
        self.assertEqual(first["target_id"], partial["target_id"])
        self.assertTrue(partial["geometry"]["orbit_usable"])
        self.assertEqual(len(partial["geometry"]["observed_hull"]), 4)

    def test_specific_origin_failure_is_not_hidden_by_geometry_age(self):
        reference = self.registry.register(estimate())["target_id"]
        self.now = 20
        result = self.registry.refresh(reference, perception(obstacle()), state(origin="B"))
        self.assertEqual(result["reason"], "target_origin_changed")

    def test_nonfinite_coordinates_do_not_enter_registry(self):
        bad = estimate()
        bad["footprint"]["hull"][0]["north_m"] = float("nan")
        self.assertFalse(self.registry.register(bad)["found"])
        self.assertEqual(self.registry.summaries(), [])

    def test_per_vertex_origin_mismatch_is_not_ignored(self):
        reference = self.registry.register(estimate())["target_id"]
        wrong = obstacle()
        wrong["hull"][0]["origin_id"] = "origin-B"
        result = self.registry.refresh(reference, perception(wrong), state())
        self.assertFalse(result["found"])

    def test_protobuf_int64_decimal_string_timestamps(self):
        source = estimate()
        source["observed_at_unix_ms"] = "10000"
        reference = self.registry.register(source)["target_id"]
        result = self.registry.refresh(reference,
            perception(obstacle(stamp="10000"), stamp="10000"), state(n=7))
        self.assertTrue(result["found"])
        self.assertEqual(result["surface_range_m"], 3)
        self.assertEqual(result["tracking"]["geometry_age_ms"], 0)

    def test_malformed_timestamp_strings_are_not_parsed(self):
        reference = self.registry.register(estimate())["target_id"]
        for stamp in ("1e4", " 10000", "10000.0", "NaN", "-1", "", None, True):
            with self.subTest(stamp=stamp):
                result = self.registry.refresh(reference,
                    perception(obstacle(), stamp=stamp), state())
                self.assertFalse(result["found"])
                self.assertEqual(result["reason"], "lidar_perception_stale")

    def test_current_home_reference_recomputes_roof_height(self):
        reference = self.registry.register(estimate())["target_id"]
        shifted = state()
        shifted["relative_altitude_m"] = 1  # Same down=-5; home is now down=-4.
        result = self.registry.refresh(reference, perception(obstacle()), shifted)
        self.assertEqual(result["footprint"]["height_m"], 2)
        self.assertEqual(result["geometry"]["height_m"], 2)
        self.assertEqual(result["footprint"]["top_down_m"], -6)
        self.assertEqual(result["footprint"]["home_down_m"], -4)
        self.assertEqual(result["geometry"]["height_evidence_age_ms"], 0)

    def test_horizontal_refresh_does_not_refresh_missing_vertical_measurement(self):
        reference = self.registry.register(estimate())["target_id"]
        self.now = 11
        scan = obstacle(stamp=11000)
        del scan["top_down_m"]
        result = self.registry.refresh(reference, perception(scan, stamp=11000), state())
        self.assertTrue(result["found"])
        self.assertEqual(result["tracking"]["geometry_age_ms"], 0)
        self.assertEqual(result["geometry"]["height_evidence_age_ms"], 1000)
        self.assertEqual(result["geometry"]["height_status"], "unavailable")
        self.assertIsNone(result["footprint"]["height_m"])
        self.assertIsNone(result["geometry"]["height_m"])
        self.assertTrue(result["footprint"]["height_clipped"])

    def test_clipped_top_is_a_lower_bound_not_roof_height(self):
        reference = self.registry.register(estimate())["target_id"]
        scan = obstacle() | {"top_down_m": -8, "top_clipped": True}
        result = self.registry.refresh(reference, perception(scan), state())
        self.assertIsNone(result["footprint"]["height_m"])
        self.assertEqual(result["footprint"]["height_lower_bound_m"], 8)
        self.assertEqual(result["geometry"]["height_status"], "clipped")
        self.assertEqual(result["geometry"]["height_evidence_age_ms"], 0)

    def test_nonfinite_top_cannot_retain_old_roof_height(self):
        reference = self.registry.register(estimate())["target_id"]
        result = self.registry.refresh(reference,
            perception(obstacle() | {"top_down_m": float("nan")}), state())
        self.assertTrue(result["found"])
        self.assertIsNone(result["footprint"]["height_m"])
        self.assertEqual(result["geometry"]["height_status"], "unavailable")

    def test_stale_vertical_evidence_cannot_be_used_as_current_height(self):
        reference = self.registry.register(estimate())["target_id"]
        self.now = 12
        result = self.registry.get(reference)
        self.assertIsNone(result["footprint"]["height_m"])
        self.assertEqual(result["geometry"]["height_status"], "stale")
        self.assertEqual(result["geometry"]["height_evidence_age_ms"], 2000)

    def test_refreshed_lidar_height_uses_scan_time_not_old_vlm_image(self):
        observed = estimate()
        observed["image_captured_at_unix_ms"] = 1000
        observed["quality"].update(lidar_refreshed_after_vlm=True, age_ms=30)
        result = self.registry.register(observed)
        self.assertEqual(result["geometry"]["height_status"], "measured")
        self.assertEqual(result["geometry"]["height_evidence_age_ms"], 30)
        self.assertEqual(result["footprint"]["height_m"], 6)

    def test_known_surface_fragments_fuse_without_inventing_back_extent(self):
        reference = self.registry.register(estimate([(10, -4), (10, 4)], depth=False))["target_id"]
        left = obstacle([(10, -4), (10, 0)], identifier="left")
        right = obstacle([(10, 0.1), (10, 4)], identifier="right")
        result = self.registry.refresh(reference, perception(left, right), state(n=7, e=-2))
        self.assertTrue(result["found"])
        self.assertEqual(result["surface_range_m"], 3)
        self.assertEqual(result["tracking"]["association_method"], "contiguous_measured_surface_fragments")
        self.assertFalse(result["geometry"]["orbit_usable"])
        self.assertEqual(len(result["tracking"]["candidates"]), 2)

    def test_fragments_with_different_vertical_interval_remain_ambiguous(self):
        reference = self.registry.register(estimate([(10, -4), (10, 4)], depth=False))["target_id"]
        left = obstacle([(10, -4), (10, 0)])
        right = obstacle([(10, 0), (10, 4)]) | {"top_down_m": -12}
        result = self.registry.refresh(reference, perception(left, right), state())
        self.assertEqual(result["reason"], "target_association_ambiguous")

    def test_adjacent_unknown_surface_is_not_fused(self):
        reference = self.registry.register(estimate([(10, -4), (10, 4)], depth=False))["target_id"]
        known = obstacle([(10, -4), (10, 4)])
        adjacent = obstacle([(10, 4), (10, 4.2)])
        result = self.registry.refresh(reference, perception(known, adjacent), state())
        self.assertFalse(result["found"])
        self.assertEqual(result["reason"], "target_association_ambiguous")
        self.assertEqual(len(result["tracking"]["candidates"]), 2)

    def test_fragment_gap_and_sensor_mismatch_remain_ambiguous(self):
        reference = self.registry.register(estimate([(10, -4), (10, 4)], depth=False))["target_id"]
        for right in (obstacle([(10, 2), (10, 4)]),
                      obstacle([(10, 0), (10, 4)]) | {"sensor_id": "other"}):
            result = self.registry.refresh(reference,
                perception(obstacle([(10, -4), (10, 0)]), right), state())
            self.assertEqual(result["reason"], "target_association_ambiguous")

    def test_orbit_circle_encloses_observed_extents_not_only_triangle_vertices(self):
        result = self.registry.register(estimate([(0, 0), (8, 0), (0, 8)]))
        self.assertAlmostEqual(result["geometry"]["center"]["north_m"], 8 / 3)
        self.assertEqual(result["geometry"]["orbit_center"]["north_m"], 4)
        self.assertAlmostEqual(result["geometry"]["orbit_radius_m"], 32 ** .5)
        self.assertFalse(result["geometry"]["whole_shape_observed"])

    def test_diagnostics_include_best_out_of_tolerance_candidate_on_loss(self):
        reference = self.registry.register(estimate())["target_id"]
        result = self.registry.refresh(reference,
            perception(obstacle([(30, 0), (30, 8)])), state())
        self.assertFalse(result["found"])
        candidate = result["tracking"]["candidates"][0]
        self.assertFalse(candidate["within_tolerance"])
        self.assertEqual(candidate["bounds"]["min_north_m"], 30)
        self.assertEqual(candidate["top_down_m"], -6)

    def test_future_sensor_timestamp_is_not_fresh(self):
        reference = self.registry.register(estimate())["target_id"]
        result = self.registry.refresh(reference, perception(obstacle(), stamp=999999), state())
        self.assertFalse(result["found"])

    def test_optional_viewpoints_use_measured_face_not_assumed_back_outline(self):
        source = estimate([(10, -4), (10, 4)], depth=False,
                          assumed=[point(999, -900), point(999, 900)])
        source["vehicle_at_capture"] = {"north_m": 0, "east_m": 0}
        result = self.registry.register(source)
        views = result["geometry"]["measurement_view_candidates"]
        self.assertEqual([(v["north_m"], v["east_m"]) for v in views], [(6, -8), (6, 8)])
        self.assertEqual([v["side"] for v in views], ["left", "right"])
        for view in views:
            self.assertFalse(view["flight_authority"])
            self.assertFalse(view["route_clearance_verified"])
            self.assertEqual(view["origin_id"], "origin-A")
        self.assertEqual(result["geometry"]["observed_extent"]["max_north_m"], 10)

    def test_measurement_viewpoints_rotate_with_measured_face_and_viewer(self):
        angle = 0.731
        def rotate(n, e):
            return (n * math.cos(angle) - e * math.sin(angle),
                    n * math.sin(angle) + e * math.cos(angle))
        source = estimate([rotate(10, -4), rotate(10, 4)], depth=False)
        source["vehicle_at_capture"] = {"north_m": 0, "east_m": 0}
        views = self.registry.register(source)["geometry"]["measurement_view_candidates"]
        for view, expected in zip(views, (rotate(6, -8), rotate(6, 8))):
            self.assertAlmostEqual(view["north_m"], expected[0])
            self.assertAlmostEqual(view["east_m"], expected[1])

    def test_viewpoints_need_pose_and_support_nonplanar_measured_patches(self):
        source = estimate([(10, -4), (10, 4)], depth=False)
        self.assertNotIn("measurement_view_candidates", self.registry.register(source)["geometry"])
        source = estimate([(10, -4), (14, -4), (14, 4), (10, 4)], depth=False)
        source["vehicle_at_capture"] = {"north_m": 0, "east_m": 0}
        geometry = self.registry.register(source, new_target=True)["geometry"]
        self.assertEqual(len(geometry["measurement_view_candidates"]), 2)
        for candidate in geometry["measurement_view_candidates"]:
            self.assertLessEqual(candidate["north_m"], 6)
            self.assertGreaterEqual(abs(candidate["east_m"]), 8)
            self.assertFalse(candidate["route_clearance_verified"])

    def test_measurement_viewpoint_offset_is_configurable(self):
        registry = TargetRegistry(clock=lambda: self.now, observation_standoff_m=2)
        source = estimate([(10, -4), (10, 4)], depth=False)
        source["vehicle_at_capture"] = {"north_m": 0, "east_m": 0}
        views = registry.register(source)["geometry"]["measurement_view_candidates"]
        self.assertEqual([(v["north_m"], v["east_m"]) for v in views], [(8, -6), (8, 6)])

    def test_refresh_viewpoints_use_current_side_and_stay_optional(self):
        source = estimate([(10, -4), (10, 4)], depth=False)
        source["vehicle_at_capture"] = {"north_m": 0, "east_m": 0}
        reference = self.registry.register(source)["target_id"]
        result = self.registry.refresh(reference,
            perception(obstacle([(10, -4), (10, 4)])), state(n=20))
        views = result["geometry"]["measurement_view_candidates"]
        self.assertEqual([v["north_m"] for v in views], [14, 14])
        self.assertFalse(result["geometry"]["orbit_usable"])

    def test_unfound_estimate_is_not_registered(self):
        failure = {"found": False, "reason": "no_match"}
        self.assertEqual(self.registry.register(failure), failure)
        self.assertEqual(self.registry.summaries(), [])


if __name__ == "__main__":
    unittest.main()


def test_curved_front_patch_depth_and_claimed_frame_count_do_not_make_orbit_usable():
    registry = TargetRegistry(clock=lambda: 10)
    front = estimate([(8, 0), (8.586, -1.414), (10, -2), (10, 2), (8.586, 1.414)])
    front['vehicle_at_capture'] = {'north_m': 0, 'east_m': 0}
    front['quality']['viewpoints'] = 99
    first = registry.register(front)
    repeated = registry.register(front)
    assert first['target_id'] == repeated['target_id']
    assert repeated['geometry']['depth_observed']
    assert repeated['geometry']['measured_view_count'] == 1
    assert not repeated['geometry']['orbit_usable']
    assert len(repeated['geometry']['measurement_view_candidates']) == 2


def test_curved_patch_enrichment_accumulates_measured_side_not_guessed_back():
    registry = TargetRegistry(clock=lambda: 10)
    front = estimate([(8, 0), (8.586, -1.414), (10, -2), (10, 2), (8.586, 1.414)])
    front['vehicle_at_capture'] = {'north_m': 0, 'east_m': 0}
    first = registry.register(front)
    side = estimate([(8, 0), (8.586, -1.414), (10, -2), (11.414, -1.414), (12, 0)])
    side['vehicle_at_capture'] = {'north_m': 10, 'east_m': -6}
    updated = registry.register(side)
    geometry = updated['geometry']
    assert updated['target_id'] == first['target_id']
    assert geometry['measured_view_separation_deg'] >= 45
    assert geometry['orbit_usable']
    assert geometry['observed_extent']['max_north_m'] == 12
    assert not geometry['whole_shape_observed']
    assert 'does not bound unseen surfaces' in geometry['view_baseline_scope']


def test_radial_approach_does_not_supply_angular_baseline():
    registry = TargetRegistry(clock=lambda: 10)
    source = estimate(); source['vehicle_at_capture'] = {'north_m': 0, 'east_m': 0}
    target = registry.register(source)
    updated = registry.refresh(target['target_id'], perception(obstacle()), state(n=5))
    assert updated['geometry']['measured_view_count'] == 2
    assert updated['geometry']['measured_view_separation_deg'] == 0
    assert not updated['geometry']['orbit_usable']


def test_curved_candidates_preserve_requested_standoff_from_measured_hull():
    from python.perception.target_registry import _measurement_views
    from python.perception.target_localization import _nearest_on_hull
    hull = [(8, 0), (8.586, -1.414), (10, -2), (10, 2), (8.586, 1.414)]
    views = _measurement_views(hull, {'north_m': 0, 'east_m': 0}, 'origin-A', 10)
    assert len(views) == 2
    for v in views:
        assert _nearest_on_hull(hull, v['north_m'], v['east_m'])[1] >= 10
        assert v['observation_standoff_m'] == 10

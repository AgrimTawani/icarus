"""Session-owned static target references backed by measured LiDAR geometry.

A reference identifies an observation, never an object from a scene manifest.
Descriptions are kept for display only. Transient sensor cluster IDs are not
identity; refresh uses overlap of measured world-coordinate surfaces instead.
"""
from __future__ import annotations

from copy import deepcopy
import math
import statistics
import time
import uuid

from .target_localization import convex_hull, vehicle_pose, _nearest_on_hull


def _number(value):
    return type(value) in (int, float) and math.isfinite(value)


def _timestamp(value):
    """MessageToDict represents protobuf int64 milliseconds as decimal text."""
    if isinstance(value, str):
        if not value or len(value) > 16 or any(c not in "0123456789" for c in value):
            return None
        value = int(value)
    if not _number(value) or value < 0 or int(value) != value:
        return None
    return int(value)


def _points(vertices, origin):
    if not isinstance(vertices, list) or not vertices:
        raise ValueError("target has no measured hull")
    points = []
    for vertex in vertices:
        if not isinstance(vertex, dict):
            raise ValueError("invalid measured hull vertex")
        if vertex.get("frame", "local_ned") != "local_ned":
            raise ValueError("target geometry must use local_ned")
        if vertex.get("origin_id", origin) != origin:
            raise ValueError("target geometry origin mismatch")
        n, e = vertex.get("north_m", 0.0), vertex.get("east_m", 0.0)
        if not _number(n) or not _number(e):
            raise ValueError("target geometry contains nonfinite coordinates")
        points.append((float(n), float(e)))
    return convex_hull(points)


def _point(point, origin):
    return {"north_m": point[0], "east_m": point[1],
            "frame": "local_ned", "origin_id": origin}


def _shape(hull):
    """Area and interior centroid of the observed convex polygon."""
    twice_area = sum(a[0] * b[1] - b[0] * a[1]
                     for a, b in zip(hull, hull[1:] + hull[:1]))
    if abs(twice_area) < 1e-8:
        return 0.0, (sum(p[0] for p in hull) / len(hull),
                     sum(p[1] for p in hull) / len(hull))
    center = tuple(sum((a[axis] + b[axis]) * (a[0] * b[1] - b[0] * a[1])
                       for a, b in zip(hull, hull[1:] + hull[:1])) /
                   (3 * twice_area) for axis in (0, 1))
    return abs(twice_area) / 2, center


def _association(a, b):
    """Require substantial mutual surface overlap, not a shared corner.

    Median distances tolerate one newly visible face without permitting a
    neighboring object's single close point to transfer the reference.
    """
    distances_a = [_nearest_on_hull(b, *point)[1] for point in a]
    distances_b = [_nearest_on_hull(a, *point)[1] for point in b]
    scores = [max(statistics.median(distances_a), statistics.median(distances_b))]
    # A current scan can see just one previously observed face. Requiring the
    # entire accumulated hull to stay visible would lose the target as soon
    # as a face is occluded. A whole measured edge on the previous boundary
    # is useful association evidence; a lone touching vertex is not.
    if len(a) >= 2:
        scores.append(max(distances_a))
    if len(b) >= 2:
        scores.append(max(distances_b))
    return min(scores)


def _bounds(hull):
    return {"min_north_m": min(p[0] for p in hull),
            "max_north_m": max(p[0] for p in hull),
            "min_east_m": min(p[1] for p in hull),
            "max_east_m": max(p[1] for p in hull)}


def _measurement_views(hull, viewer, origin, standoff):
    """Optional side-view geometry for a measured surface patch, never a route.

    The side toward the observing camera is measured from that camera pose.
    No outline assumption, object label or scene coordinate is consulted.
    """
    if len(hull) < 2 or not isinstance(viewer, dict):
        return []
    if not all(_number(viewer.get(key)) for key in ("north_m", "east_m")):
        return []
    a, b = max(((a, b) for a in hull for b in hull), key=lambda pair: math.dist(*pair))
    length = math.dist(a, b)
    if length < 0.5:
        return []
    axis = ((b[0] - a[0]) / length, (b[1] - a[1]) / length)
    normal = (-axis[1], axis[0])
    midpoint = ((a[0] + b[0]) / 2, (a[1] + b[1]) / 2)
    cross = [(p[0] - midpoint[0]) * normal[0] +
             (p[1] - midpoint[1]) * normal[1] for p in hull]
    if max(cross) - min(cross) > min(0.6, length * 0.15):
        # A curved visible patch also needs more views. Use only its measured
        # support bounds in the camera-facing frame; do not extrapolate a back.
        _, midpoint = _shape(hull)
        toward = (viewer["north_m"] - midpoint[0], viewer["east_m"] - midpoint[1])
        distance = math.hypot(*toward)
        if distance < 0.25:
            return []
        normal = (toward[0] / distance, toward[1] / distance)
        right = (normal[1], -normal[0])
        front = max((p[0]-midpoint[0])*normal[0] + (p[1]-midpoint[1])*normal[1] for p in hull)
        lateral = [(p[0]-midpoint[0])*right[0] + (p[1]-midpoint[1])*right[1] for p in hull]
        views = []
        for side, edge, sign in (("left", min(lateral), -1), ("right", max(lateral), 1)):
            position = tuple(midpoint[i] + (front + standoff)*normal[i] + (edge + sign*standoff)*right[i] for i in (0, 1))
            views.append({"side": side, **_point(position, origin),
                "face_observed_midpoint_heading_deg": math.degrees(math.atan2(midpoint[1]-position[1], midpoint[0]-position[0])) % 360,
                "observation_standoff_m": standoff,
                "purpose": "optional side view outside measured patch support bounds; unseen shape unknown",
                "route_clearance_verified": False, "flight_authority": False})
        return views
    toward_viewer = ((viewer["north_m"] - midpoint[0]) * normal[0] +
                     (viewer["east_m"] - midpoint[1]) * normal[1])
    if abs(toward_viewer) < 0.25:
        return []  # Camera side is ambiguous from an edge-on observation.
    if toward_viewer < 0:
        normal = (-normal[0], -normal[1])
    right = (normal[1], -normal[0])
    endpoints = sorted((a, b), key=lambda p: p[0] * right[0] + p[1] * right[1])
    views = []
    for side, endpoint, sign in (("left", endpoints[0], -1), ("right", endpoints[1], 1)):
        position = (endpoint[0] + standoff * (normal[0] + sign * right[0]),
                    endpoint[1] + standoff * (normal[1] + sign * right[1]))
        views.append({"side": side, **_point(position, origin),
                      "face_observed_midpoint_heading_deg": math.degrees(math.atan2(
                          midpoint[1] - position[1], midpoint[0] - position[0])) % 360,
                      "observation_standoff_m": standoff,
                      "purpose": "optional viewpoint beyond a measured face edge; retain current altitude",
                      "route_clearance_verified": False, "flight_authority": False})
    return views


def _fuse_surface_fragments(candidates, known_hull, sensor_id):
    """Fuse contiguous disjoint pieces of one already measured planar face.

    Proximity alone is insufficient: require the pieces to remain inside the
    observed envelope, share sensor/vertical interval, lie on one plane, and
    cover a contiguous surface. Duplicated or overlapping hypotheses remain
    ambiguous. This never extends an object into neighboring unknown space.
    """
    tolerance = 0.4  # Metres: surface noise/segmentation gaps, not object size.
    if not 2 <= len(candidates) <= 6:
        return None
    selected = [c[1] for c in candidates]
    sensor = selected[0].get("sensor_id")
    if not sensor or (sensor_id and sensor != sensor_id):
        return None
    vertical = []
    for candidate in selected:
        top, bottom = candidate.get("top_down_m"), candidate.get("bottom_down_m")
        if candidate.get("sensor_id") != sensor or not _number(top) or not _number(bottom) or top > bottom:
            return None
        vertical.append((top, bottom))
    if any(abs(top - vertical[0][0]) > tolerance or abs(bottom - vertical[0][1]) > tolerance
           for top, bottom in vertical):
        return None
    bounds = _bounds(known_hull)
    combined = [point for _, _, hull in candidates for point in hull]
    if any(not bounds["min_north_m"] - tolerance <= n <= bounds["max_north_m"] + tolerance or
           not bounds["min_east_m"] - tolerance <= e <= bounds["max_east_m"] + tolerance or
           _nearest_on_hull(known_hull, n, e)[1] > tolerance for n, e in combined):
        return None
    start, end = max(((a, b) for a in combined for b in combined),
                     key=lambda pair: math.dist(*pair))
    length = math.dist(start, end)
    if length <= tolerance:
        return None
    axis = ((end[0] - start[0]) / length, (end[1] - start[1]) / length)
    project = lambda point: (point[0] - start[0]) * axis[0] + (point[1] - start[1]) * axis[1]
    if any(abs((p[0] - start[0]) * axis[1] - (p[1] - start[1]) * axis[0]) > tolerance
           for p in combined):
        return None
    intervals = sorted((min(map(project, hull)), max(map(project, hull))) for _, _, hull in candidates)
    for left, right in zip(intervals, intervals[1:]):
        overlap = left[1] - right[0]
        shorter = min(left[1] - left[0], right[1] - right[0])
        if shorter <= tolerance or overlap < -tolerance or overlap > 0.2 * shorter:
            return None
    fused = deepcopy(selected[0])
    fused.update(obstacle_id="+".join(str(c.get("obstacle_id", "")) for c in selected),
                 point_count=sum(c.get("point_count", 0) for c in selected),
                 observed_at_unix_ms=min(_timestamp(c["observed_at_unix_ms"]) for c in selected),
                 top_down_m=min(top for top, _ in vertical),
                 bottom_down_m=max(bottom for _, bottom in vertical),
                 top_clipped=any(c.get("top_clipped", False) for c in selected))
    return fused, convex_hull(combined)


class TargetRegistry:
    """One registry per flight client/session; returned dictionaries are copies.

    ``orbit_usable`` means a two-dimensional estimate has separated views.
    It never means unseen surfaces are known or a complete route is clear.
    ``surface_range_m`` is horizontal LiDAR surface distance, not optical-z
    camera depth, center radius, or guaranteed clearance to every obstacle.
    """

    def __init__(self, *, clock=time.time, maximum_age_ms=1500,
                 association_tolerance_m=1.5, ambiguity_margin_m=0.3,
                 observation_standoff_m=4.0, minimum_orbit_view_separation_deg=45.0):
        if not _number(observation_standoff_m) or observation_standoff_m <= 0:
            raise ValueError("observation_standoff_m must be finite and positive")
        if not _number(minimum_orbit_view_separation_deg) or not 0 < minimum_orbit_view_separation_deg <= 180:
            raise ValueError("minimum orbit view separation must be in (0, 180]")
        self.minimum_orbit_view_separation_deg = minimum_orbit_view_separation_deg
        self.clock = clock
        self.maximum_age_ms = maximum_age_ms
        self.association_tolerance_m = association_tolerance_m
        self.ambiguity_margin_m = ambiguity_margin_m
        self.observation_standoff_m = observation_standoff_m
        self._targets = {}

    def _age(self, timestamp):
        return int(self.clock() * 1000) - int(timestamp)

    @staticmethod
    def _record_view(item, viewer):
        if not isinstance(viewer, dict) or not all(_number(viewer.get(k)) for k in ("north_m", "east_m")):
            return
        point = (float(viewer["north_m"]), float(viewer["east_m"]))
        views = item.setdefault("_viewpoints", [])
        if point not in views:
            views.append(point)
            # Retain the original baseline and a bounded recent pose history.
            if len(views) > 32:
                del views[1]

    def _geometry(self, item):
        hull = item["_hull"]
        area, center = _shape(hull)
        footprint = item.get("footprint", {})
        depth_observed = item["_depth_observed"]
        # Curved front surfaces have depth without observing the far side.
        # Require an actual angular baseline, not the localizer's frame count.
        # This is evidence enrichment only, never a full-shape guarantee.
        views = item.get("_viewpoints", [])
        bearings = [math.atan2(v[1]-center[1], v[0]-center[0]) for v in views
                    if math.dist(v, center) > 0.25]
        separation = max((abs(math.degrees(math.remainder(a-b, 2*math.pi)))
                          for a in bearings for b in bearings), default=0.0)
        usable = (area > 0.01 and depth_observed
                  and separation >= self.minimum_orbit_view_separation_deg)
        origin = item["origin_id"]
        bounds = _bounds(hull)
        orbit_center = ((bounds["min_north_m"] + bounds["max_north_m"]) / 2,
                        (bounds["min_east_m"] + bounds["max_east_m"]) / 2)
        item["geometry"] = {
            "observed_hull": [_point(p, origin) for p in hull],
            "center": _point(center, origin),
            "radius_m": max(math.dist(center, p) for p in hull),
            "orbit_center": _point(orbit_center, origin),
            "orbit_radius_m": math.hypot(bounds["max_north_m"] - bounds["min_north_m"],
                                         bounds["max_east_m"] - bounds["min_east_m"]) / 2,
            "orbit_basis": "circle enclosing observed axis-aligned extents; unseen extent remains unknown",
            "observed_area_m2": area,
            "observed_extent": bounds,
            "depth_observed": depth_observed,
            "whole_shape_observed": False,
            "orbit_usable": usable,
            "requires_more_views": not usable,
            "measured_view_count": len(views),
            "measured_view_separation_deg": round(separation, 2),
            "minimum_view_separation_deg": self.minimum_orbit_view_separation_deg,
            "view_baseline_scope": "separated measured views; does not bound unseen surfaces",
            "viewpoints": item.get("quality", {}).get("viewpoints", 1),
            "position_spread_m": item.get("quality", {}).get("position_spread_m"),
            "height_m": footprint.get("height_m"),
            "height_clipped": footprint.get("height_clipped", True),
            "height_reference": footprint.get("height_reference"),
            "height_status": item.get("_height_status", "unavailable"),
            "height_evidence_age_ms": (self._age(item["_height_timestamp"])
                                       if item.get("_height_timestamp") is not None else None),
            "uncertainty": "Observed surfaces only; unseen extent is unknown.",
        }
        if not usable:
            item["geometry"]["suggestion"] = "Observe the same target from a separated viewpoint to enrich measured extent; unseen surfaces remain unknown."
            views = _measurement_views(hull, item.get("_viewer", item.get("vehicle_at_capture")),
                                       origin, self.observation_standoff_m)
            if views:
                item["geometry"]["measurement_view_candidates"] = views

    def _public(self, item):
        result = deepcopy({k: v for k, v in item.items() if not k.startswith("_")})
        age = self._age(item["_geometry_timestamp"])
        result["tracking"]["geometry_age_ms"] = age
        vertical_timestamp = item.get("_height_timestamp")
        vertical_age = self._age(vertical_timestamp) if vertical_timestamp is not None else None
        result["geometry"]["height_evidence_age_ms"] = vertical_age
        result["footprint"]["height_evidence_age_ms"] = vertical_age
        if vertical_age is not None and not -100 <= vertical_age <= self.maximum_age_ms:
            result["geometry"]["height_m"] = result["footprint"]["height_m"] = None
            result["geometry"]["height_status"] = result["footprint"]["height_status"] = "stale"
        if (age > self.maximum_age_ms or age < -100) and result.get("found"):
            result["found"] = False
            result["tracking"].update(status="stale", reason="target_geometry_stale")
            result["reason"] = "target_geometry_stale"
        result["geometry"]["orbit_usable"] &= result.get("found", False)
        return result

    def _fail(self, item, reason, status="lost"):
        item["found"] = False
        item["reason"] = reason
        item["tracking"].update(status=status, reason=reason)
        return self._public(item)

    def register(self, estimate, new_target=False):
        """Register a successful visual/LiDAR observation or return its failure.

        Repeated observations reuse an ID only for one unambiguous geometric
        match in the same origin. ``new_target`` explicitly starts a reference.
        """
        if not estimate.get("found"):
            return deepcopy(estimate)
        origin = (estimate.get("estimated_position") or {}).get("origin_id", "")
        if not origin:
            return deepcopy(estimate) | {"found": False, "reason": "target_origin_missing"}
        try:
            hull = _points(estimate.get("footprint", {}).get("hull"), origin)
        except ValueError as exc:
            return deepcopy(estimate) | {"found": False, "reason": str(exc)}
        candidates = []
        if not new_target:
            for key, item in self._targets.items():
                if item["origin_id"] == origin:
                    score = _association(hull, item["_hull"])
                    if score <= self.association_tolerance_m:
                        candidates.append((score, key))
        candidates.sort()
        if len(candidates) > 1 and candidates[1][0] - candidates[0][0] <= self.ambiguity_margin_m:
            return deepcopy(estimate) | {"found": False, "reason": "target_identity_ambiguous"}
        key = candidates[0][1] if candidates else "target_" + uuid.uuid4().hex[:16]
        previous = self._targets.get(key)
        timestamp = _timestamp(estimate.get("observed_at_unix_ms", 0))
        age = estimate.get("quality", {}).get("age_ms", 0)
        if timestamp is None or not _number(age):
            return deepcopy(estimate) | {"found": False, "reason": "invalid_target_timestamp"}
        item = deepcopy(estimate)
        item.update(target_id=key, origin_id=origin, frame="local_ned",
                    tracking={"status": "tracked", "reason": None},
                    _hull=convex_hull(hull + (previous["_hull"] if previous else [])),
                    _geometry_timestamp=int(timestamp - age),
                    _depth_observed=(bool(estimate.get("footprint", {}).get("depth_observed"))
                                     and not estimate.get("footprint", {}).get("assumed_outline"))
                                    or bool(previous and previous["_depth_observed"]))
        item["_viewpoints"] = deepcopy(previous.get("_viewpoints", [])) if previous else []
        self._record_view(item, estimate.get("vehicle_at_capture"))
        footprint = item.get("footprint", {})
        valid_height = _number(footprint.get("height_m")) and not footprint.get("height_clipped", True)
        height_timestamp = int(timestamp - age)
        if not estimate.get("quality", {}).get("lidar_refreshed_after_vlm"):
            height_timestamp = min(height_timestamp,
                _timestamp(estimate.get("image_captured_at_unix_ms")) or height_timestamp)
        item["_height_timestamp"] = height_timestamp if valid_height else None
        item["_height_status"] = "measured" if valid_height else "unavailable"
        self._geometry(item)
        self._targets[key] = item
        return self._public(item)

    def get(self, target_id):
        """Unknown/session-foreign IDs are explicit failures, not exceptions."""
        item = self._targets.get(target_id)
        if item is None:
            return {"target_id": target_id, "found": False, "reason": "unknown_target_id",
                    "tracking": {"status": "unknown", "reason": "unknown_target_id"}}
        return self._public(item)

    def summaries(self):
        """Current copy of target evidence for the agent feedback context."""
        summaries = []
        for item in list(self._targets.values())[-8:]:
            public = self._public(item)
            summaries.append({key: public[key] for key in (
                "target_id", "description", "found", "reason",
                "surface_range_m", "surface_range_kind", "origin_id") if key in public} | {
                "tracking": {key: value for key, value in public["tracking"].items()
                             if key != "candidates"},
                "geometry": {key: public["geometry"][key] for key in (
                    "center", "radius_m", "orbit_center", "orbit_radius_m", "orbit_usable", "requires_more_views",
                    "height_m", "height_clipped", "height_status", "height_evidence_age_ms",
                    "whole_shape_observed", "observed_extent", "measurement_view_candidates",
                    "measured_view_count", "measured_view_separation_deg", "minimum_view_separation_deg",
                    "view_baseline_scope")
                    if key in public["geometry"]}})
        return summaries

    def refresh(self, target_id, perception, state):
        """Update one target from current LiDAR; no image/model invocation."""
        item = self._targets.get(target_id)
        if item is None:
            return self.get(target_id)
        item["tracking"].pop("candidates", None)
        item["tracking"].pop("association_method", None)
        try:
            pose = vehicle_pose(state)
        except (ValueError, TypeError):
            return self._fail(item, "vehicle_pose_unavailable")
        if pose["origin_id"] != item["origin_id"]:
            return self._fail(item, "target_origin_changed", "invalid")
        if not perception.get("local_map_available"):
            return self._fail(item, "lidar_perception_unavailable")
        stamp = _timestamp(perception.get("observed_at_unix_ms", 0))
        if stamp is None or not -100 <= self._age(stamp) <= self.maximum_age_ms:
            return self._fail(item, "lidar_perception_stale", "stale")
        if perception.get("origin_id", pose["origin_id"]) != pose["origin_id"]:
            return self._fail(item, "perception_origin_mismatch", "invalid")
        candidates = []
        for candidate in perception.get("nearby_obstacles", []) or []:
            timestamp = _timestamp(candidate.get("observed_at_unix_ms", 0))
            if timestamp is None or not -100 <= self._age(timestamp) <= self.maximum_age_ms:
                continue
            if not _number(candidate.get("point_count", 0)) or candidate.get("point_count", 0) <= 0:
                continue
            try:
                hull = _points(candidate.get("hull"), pose["origin_id"])
            except ValueError:
                continue
            score = _association(item["_hull"], hull)
            candidates.append((score, candidate, hull))
        candidates.sort(key=lambda value: value[0])
        item["tracking"]["candidates"] = [
            {"score_m": round(score, 4), "id": c.get("obstacle_id"),
             "sensor_id": c.get("sensor_id"), "point_count": c.get("point_count"),
             "top_down_m": c.get("top_down_m"), "bottom_down_m": c.get("bottom_down_m"),
             "top_clipped": c.get("top_clipped", False),
             "observed_at_unix_ms": c.get("observed_at_unix_ms"), "bounds": _bounds(hull),
             "hull": [list(p) for p in hull[:12]], "hull_vertices": len(hull),
             "within_tolerance": score <= self.association_tolerance_m}
            for score, c, hull in candidates[:6]]
        candidates = [candidate for candidate in candidates if candidate[0] <= self.association_tolerance_m]
        if not candidates:
            return self._fail(item, "target_not_observed")
        if len(candidates) > 1 and candidates[1][0] - candidates[0][0] <= self.ambiguity_margin_m:
            fused = _fuse_surface_fragments(candidates, item["_hull"], item.get("lidar_sensor_id"))
            if fused is None:
                return self._fail(item, "target_association_ambiguous", "ambiguous")
            candidate, hull = fused
            measurement_hulls = [part[2] for part in candidates]
            item["tracking"]["association_method"] = "contiguous_measured_surface_fragments"
        else:
            _, candidate, hull = candidates[0]
            measurement_hulls = [hull]
            item["tracking"]["association_method"] = "single_geometric_match"
        surface, distance = min((_nearest_on_hull(part, pose["north_m"], pose["east_m"])
                                 for part in measurement_hulls), key=lambda value: value[1])
        item.update(found=True, surface_range_m=distance,
                    surface_point=_point(surface, pose["origin_id"]),
                    surface_range_kind="horizontal_lidar_surface_distance",
                    lidar_cluster_id=candidate.get("obstacle_id", ""),
                    lidar_sensor_id=candidate.get("sensor_id", ""),
                    observed_at_unix_ms=candidate["observed_at_unix_ms"],
                    _geometry_timestamp=_timestamp(candidate["observed_at_unix_ms"]),
                    _viewer={"north_m": pose["north_m"], "east_m": pose["east_m"]},
                    _hull=convex_hull(item["_hull"] + hull))
        self._record_view(item, item["_viewer"])
        item.pop("reason", None)
        item["tracking"].update(status="tracked", reason=None)
        # Horizontal refresh cannot renew an old roof measurement. Derive
        # vertical evidence from this scan using the current home reference;
        # a home reset can occur without changing the local EKF origin.
        footprint = item["footprint"]
        top = candidate.get("top_down_m")
        clipped = candidate.get("top_clipped", False)
        vertical_valid = _number(top) and isinstance(clipped, bool)
        footprint.update(height_m=None, height_clipped=True,
                         height_reference="home_arming_altitude",
                         height_lower_bound_m=None)
        if vertical_valid:
            measured_height = pose["terrain_down_m"] - top
            item["_height_timestamp"] = _timestamp(candidate["observed_at_unix_ms"])
            item["_height_status"] = "clipped" if clipped else "measured"
            footprint.update(height_m=None if clipped else measured_height,
                             height_clipped=clipped,
                             height_lower_bound_m=measured_height,
                             top_down_m=top, home_down_m=pose["terrain_down_m"],
                             height_observed_at_unix_ms=item["_height_timestamp"])
        else:
            item["_height_status"] = "unavailable"
            footprint.pop("top_down_m", None)
            footprint.pop("home_down_m", None)
            footprint.pop("height_observed_at_unix_ms", None)
        footprint["height_status"] = item["_height_status"]
        # The accumulated measured hull is in geometry. Never add assumed
        # back-face vertices while updating the fresh vertical measurements.
        self._geometry(item)
        return self._public(item)

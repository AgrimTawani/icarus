"""Measured target localization: the VLM chooses, LiDAR and geometry measure.

Given a short operator description ("building behind the group of people"),
the VLM answers only *which image region* matches: a grounding box in the
model's 0..1000 image coordinates, or NONE.  Every metric quantity comes from
the Drone API's LiDAR cluster evidence and the vehicle state:

* bearing   -- camera intrinsics applied to the selected image columns;
* range     -- the LiDAR cluster associated with that image region;
* footprint -- the cluster's horizontal hull, fused across repeated views;
* quality   -- return counts, view agreement, spread and evidence age.

Nothing here reads mission text to accept or reject a plan, carries static
environment knowledge, or reports a probability: no calibrated one exists.
The estimate is evidence for a bounded runtime objective, never a command.
"""

from __future__ import annotations

import json
import math
import re
import time
from dataclasses import dataclass
from pathlib import Path

SCHEMA = "icarus.perception.target_estimate.v1"
METHOD = "vlm_grounding_box+lidar_cluster"
MAX_DESCRIPTION_CHARS = 120
_BOX = re.compile(r"\(\s*(\d{1,4})\s*,\s*(\d{1,4})\s*\)\s*,\s*"
                  r"\(\s*(\d{1,4})\s*,\s*(\d{1,4})\s*\)")

# These phrases select the already-authorized forward view; they are not a
# visual attribute of the object in that view.  Keeping them in a VLM query
# turns e.g. ``building in front`` into an incomplete object-to-object
# relation ("in front of what?") and can make an otherwise visible object
# fail the presence gate.  Object-relative relations such as ``building in
# front of the hangar`` intentionally do not match this suffix grammar.
_FORWARD_VIEW_SUFFIX = re.compile(
    r"\s+(?:(?:visible\s+)?(?:directly\s+)?ahead|"
    r"(?:directly\s+)?in\s+front(?:\s+of\s+(?:us|me|the\s+drone))?)\s*$",
    re.IGNORECASE,
)


def normalize_description(description: str) -> str:
    if not isinstance(description, str):
        raise ValueError("target description must be text")
    compact = " ".join(description.strip().split())
    if not compact or len(compact) > MAX_DESCRIPTION_CHARS:
        raise ValueError(
            f"target description must be 1..{MAX_DESCRIPTION_CHARS} characters")
    return compact


def visual_grounding_description(description: str) -> dict:
    """Separate a camera-view selector from the thing to ground.

    Runtime target strings predate typed view fields, so a planner may encode
    the forward-camera selector in the observed description.  This boundary
    keeps the operator description for reporting and target identity while
    giving the VLM only the visual subject.  It never removes a relation with
    another named object.
    """
    original = normalize_description(description)
    match = _FORWARD_VIEW_SUFFIX.search(original)
    if match is None:
        return {"description": original, "visual_query": original,
                "view": None}
    visual_query = original[:match.start()].rstrip(" ,")
    if not visual_query:
        return {"description": original, "visual_query": original,
                "view": None}
    return {"description": original, "visual_query": visual_query,
            "view": "forward_d435i"}


def load_localization_config(path) -> dict:
    """Load the versioned camera calibration and association policy."""
    config = json.loads(Path(path).read_text())
    if config.get("version") != 1:
        raise ValueError("unsupported target localization configuration")
    camera = config.get("camera", {})
    for name in ("width_px", "height_px"):
        if not isinstance(camera.get(name), int) or camera[name] <= 0:
            raise ValueError("invalid camera " + name)
    fov = camera.get("horizontal_fov_deg")
    if not isinstance(fov, (int, float)) or not 1 <= fov <= 179:
        raise ValueError("invalid camera horizontal_fov_deg")
    for section, keys in (
            ("association", ("minimum_image_iou", "minimum_bearing_overlap",
                             "minimum_lidar_returns")),
            ("multiview", ("history_s", "agreement_tolerance_m",
                           "agreement_range_fraction")),
            ("footprint", ("single_face_depth_m", "single_face_depth_fraction"))):
        values = config.get(section, {})
        for key in keys:
            if not isinstance(values.get(key), (int, float)) or values[key] < 0:
                raise ValueError(f"invalid {section}.{key}")
    if not isinstance(config.get("maximum_perception_age_ms"), int):
        raise ValueError("invalid maximum_perception_age_ms")
    return config


def airsim_localization_config(config: dict, mount: dict) -> dict:
    """Swap in the AirSim forward camera; association policy stays the same.

    ``mount`` is camera_geometry.airsim_camera(): FLU pose, so pitch>0 looks
    down (this model's pitch_offset sign) and yaw>0 looks left.
    """
    camera = {**config["camera"], "sensor_id": "airsim_camera_0",
              "width_px": mount["width"], "height_px": mount["height"],
              "horizontal_fov_deg": mount["horizontal_fov_deg"],
              "yaw_offset_deg": -math.degrees(mount["pose_flu"][5]),
              "pitch_offset_deg": math.degrees(mount["pose_flu"][4]),
              "calibration": "AirSim settings.json camera 0 capture settings and mount pose"}
    return {**config, "camera": camera, "lidar": {**config["lidar"], "sensor_id": "airsim_lidar1"}}


def _wrap_deg(value: float) -> float:
    return (value + 180.0) % 360.0 - 180.0


@dataclass(frozen=True)
class CameraModel:
    """Pinhole forward camera; square pixels, principal point at centre."""

    width_px: int
    height_px: int
    horizontal_fov_deg: float
    yaw_offset_deg: float = 0.0
    pitch_offset_deg: float = 0.0

    @classmethod
    def from_config(cls, camera: dict) -> CameraModel:
        return cls(int(camera["width_px"]), int(camera["height_px"]),
                   float(camera["horizontal_fov_deg"]),
                   float(camera.get("yaw_offset_deg", 0.0)),
                   float(camera.get("pitch_offset_deg", 0.0)))

    @property
    def focal_px(self) -> float:
        return (self.width_px / 2.0) / math.tan(
            math.radians(self.horizontal_fov_deg) / 2.0)

    @property
    def half_fov_deg(self) -> float:
        return self.horizontal_fov_deg / 2.0

    def relative_bearing_deg(self, column_px: float) -> float:
        """Bearing of an image column relative to the vehicle heading."""
        return self.yaw_offset_deg + math.degrees(
            math.atan((column_px - self.width_px / 2.0) / self.focal_px))

    def column_px(self, relative_bearing_deg: float) -> float | None:
        angle = _wrap_deg(relative_bearing_deg - self.yaw_offset_deg)
        if abs(angle) >= 89.0:
            return None
        return self.width_px / 2.0 + self.focal_px * math.tan(math.radians(angle))

    def row_px(self, depression_rad: float, vehicle_pitch_rad: float) -> float:
        """Image row of a ray ``depression_rad`` below the local horizontal.

        A nose-up vehicle pitch lowers every scene point in the image.  Roll is
        ignored; it is small in hover and only rotates the box slightly.
        """
        angle = depression_rad + vehicle_pitch_rad - math.radians(
            self.pitch_offset_deg)
        angle = max(-1.5, min(1.5, angle))
        return self.height_px / 2.0 + self.focal_px * math.tan(angle)


def parse_grounding_box(text: str):
    """Return the first (x1, y1, x2, y2) box in 0..1000 coordinates or None."""
    if not isinstance(text, str):
        return None
    match = _BOX.search(text)
    if not match:
        return None
    x1, y1, x2, y2 = (int(value) for value in match.groups())
    if not all(0 <= value <= 1000 for value in (x1, y1, x2, y2)):
        return None
    if x2 <= x1 or y2 <= y1:
        return None
    return (x1, y1, x2, y2)


def box_to_pixels(box_1000, camera: CameraModel):
    x1, y1, x2, y2 = box_1000
    return (x1 / 1000.0 * camera.width_px, y1 / 1000.0 * camera.height_px,
            x2 / 1000.0 * camera.width_px, y2 / 1000.0 * camera.height_px)


def _float(mapping, key, default=0.0):
    value = mapping.get(key, default)
    return float(value) if isinstance(value, (int, float)) else default


def vehicle_pose(state: dict) -> dict:
    """Extract the local-NED pose from a DroneState dictionary."""
    position = state.get("local_position_ned")
    attitude = state.get("attitude") or {}
    if not isinstance(position, dict):
        raise ValueError("vehicle state has no local NED position")  # noqa: TRY004 - missing telemetry is a measurement error
    values = [position.get(key, 0.0) for key in ("north_m", "east_m", "down_m")]
    values += [attitude.get(key, 0.0) for key in ("yaw_rad", "pitch_rad", "roll_rad")]
    values.append(state.get("relative_altitude_m", 0.0))
    if any(type(value) not in (int, float) or not math.isfinite(value) for value in values):
        raise ValueError("vehicle pose contains invalid measured coordinates")
    down = _float(position, "down_m")
    # Terrain reference: home (arming) altitude.  The EKF origin's down drifts
    # with barometer bias between flights; relative altitude is re-referenced
    # to home at arming, so down + relative altitude is where home terrain is.
    terrain = down + float(state.get("relative_altitude_m", 0.0))
    return {"north_m": _float(position, "north_m"),
            "east_m": _float(position, "east_m"),
            "down_m": down, "terrain_down_m": terrain,
            "origin_id": position.get("origin_id", ""),
            "yaw_rad": _float(attitude, "yaw_rad"),
            "pitch_rad": _float(attitude, "pitch_rad"),
            "roll_rad": _float(attitude, "roll_rad")}


def candidates_from_perception(perception: dict) -> list[dict]:
    """Normalize PerceptionSummary.nearby_obstacles (MessageToDict form)."""
    candidates = []
    for item in perception.get("nearby_obstacles", []) or []:
        hull = [(_float(vertex, "north_m"), _float(vertex, "east_m"))
                for vertex in item.get("hull", []) or []]
        if not hull:
            continue
        centroid = item.get("centroid") or {}
        candidates.append({
            "id": item.get("obstacle_id", ""),
            "sensor_id": item.get("sensor_id", ""),
            "hull": hull,
            "centroid": (_float(centroid, "north_m"), _float(centroid, "east_m")),
            "point_count": int(item.get("point_count", 0) or 0),
            "top_down_m": _float(item, "top_down_m"),
            "bottom_down_m": _float(item, "bottom_down_m"),
            "top_clipped": bool(item.get("top_clipped", False)),
            "observed_at_unix_ms": int(item.get("observed_at_unix_ms", 0) or 0),
        })
    return candidates


def _nearest_on_hull(hull, north, east):
    best, best_distance = hull[0], math.inf
    count = len(hull)
    for index in range(count):
        a, b = hull[index], hull[(index + 1) % count]
        dn, de = b[0] - a[0], b[1] - a[1]
        length = dn * dn + de * de
        t = 0.0 if length <= 0 else max(0.0, min(1.0, (
            (north - a[0]) * dn + (east - a[1]) * de) / length))
        point = (a[0] + t * dn, a[1] + t * de)
        distance = math.hypot(point[0] - north, point[1] - east)
        if distance < best_distance:
            best, best_distance = point, distance
    return best, best_distance


def project_candidate(candidate: dict, pose: dict, camera: CameraModel):
    """Project a LiDAR cluster into an image box (pixels) or None if unseen.

    Columns come from the hull's bearing span.  The top row uses the highest
    return at the nearest surface range.  The bottom row assumes the object
    stands on terrain at the home reference height; LiDAR cannot see the base
    below its lowest beam, so this is a stated assumption used only to
    compare with the VLM box, never as a measurement.
    """
    north, east = pose["north_m"], pose["east_m"]
    heading = math.degrees(pose["yaw_rad"])
    centre = candidate["centroid"]
    centre_bearing = math.degrees(math.atan2(centre[1] - east, centre[0] - north))
    offsets = [_wrap_deg(math.degrees(math.atan2(v[1] - east, v[0] - north))
                         - centre_bearing) for v in candidate["hull"]]
    relative_min = _wrap_deg(centre_bearing - heading) + min(offsets)
    relative_max = _wrap_deg(centre_bearing - heading) + max(offsets)
    half = camera.half_fov_deg
    visible_min, visible_max = max(relative_min, -half), min(relative_max, half)
    if visible_max <= visible_min:
        return None
    surface, surface_range = _nearest_on_hull(candidate["hull"], north, east)
    surface_range = max(surface_range, 0.5)
    top = math.atan2(candidate["top_down_m"] - pose["down_m"], surface_range)
    base_down = max(candidate["bottom_down_m"], pose.get("terrain_down_m", 0.0))
    bottom = math.atan2(base_down - pose["down_m"], surface_range)
    rows = sorted((camera.row_px(top, pose["pitch_rad"]),
                   camera.row_px(bottom, pose["pitch_rad"])))
    box = (camera.column_px(visible_min), max(0.0, rows[0]),
           camera.column_px(visible_max), min(float(camera.height_px), rows[1]))
    if box[3] <= box[1]:
        return None
    return {"box_px": box, "relative_bearing_span_deg": (relative_min, relative_max),
            "visible_fraction": (visible_max - visible_min) /
            max(relative_max - relative_min, 1e-6),
            "surface_point": surface, "surface_range_m": surface_range}


def _iou(a, b):
    width = min(a[2], b[2]) - max(a[0], b[0])
    height = min(a[3], b[3]) - max(a[1], b[1])
    if width <= 0 or height <= 0:
        return 0.0
    inter = width * height
    union = (a[2] - a[0]) * (a[3] - a[1]) + (b[2] - b[0]) * (b[3] - b[1]) - inter
    return inter / union if union > 0 else 0.0


def associate(box_px, candidates, pose, camera: CameraModel, association: dict):
    """Pick the LiDAR cluster whose projected image box best matches the VLM box.

    Returns (best, ranked) where ``best`` is None when no cluster meets the
    configured geometric overlap.  Ranked entries are kept as evidence.
    """
    ranked = []
    box_width = max(box_px[2] - box_px[0], 1e-6)
    for candidate in candidates:
        projection = project_candidate(candidate, pose, camera)
        if projection is None:
            continue
        cluster_box = projection["box_px"]
        overlap = max(0.0, min(box_px[2], cluster_box[2]) -
                      max(box_px[0], cluster_box[0]))
        ranked.append({"candidate": candidate, "projection": projection,
                       "image_iou": _iou(box_px, cluster_box),
                       "bearing_overlap": overlap / box_width})
    ranked.sort(key=lambda item: (-item["image_iou"], -item["bearing_overlap"]))
    best = next((item for item in ranked
                 if item["image_iou"] >= association["minimum_image_iou"]
                 and item["bearing_overlap"] >= association["minimum_bearing_overlap"]
                 and item["candidate"]["point_count"] >=
                 association["minimum_lidar_returns"]), None)
    return best, ranked


def _cross(o, a, b):
    return (a[0] - o[0]) * (b[1] - o[1]) - (a[1] - o[1]) * (b[0] - o[0])


def convex_hull(points):
    unique = sorted(set((float(n), float(e)) for n, e in points))
    if len(unique) < 3:
        return unique
    lower, upper = [], []
    for point in unique:
        while len(lower) >= 2 and _cross(lower[-2], lower[-1], point) <= 0:
            lower.pop()
        lower.append(point)
    for point in reversed(unique):
        while len(upper) >= 2 and _cross(upper[-2], upper[-1], point) <= 0:
            upper.pop()
        upper.append(point)
    return lower[:-1] + upper[:-1]


def _fitted_rectangle(hull, tolerance_m=0.3):
    """Return (centre, long_axis_unit, long_m, short_m) of the hull's rectangle.

    Candidate orientations are the hull edges.  LiDAR sees surfaces, so the
    preferred rectangle is the one whose sides coincide with the most observed
    hull boundary (an L of two faces fits its legs, not its hypotenuse); ties
    fall back to the smaller area.
    """
    if len(hull) == 1:
        return hull[0], (1.0, 0.0), 0.0, 0.0
    best = None
    count = len(hull)
    for index in range(count):
        a, b = hull[index], hull[(index + 1) % count]
        length = math.hypot(b[0] - a[0], b[1] - a[1])
        if length <= 1e-9:
            continue
        axis = ((b[0] - a[0]) / length, (b[1] - a[1]) / length)
        normal = (-axis[1], axis[0])
        along = [p[0] * axis[0] + p[1] * axis[1] for p in hull]
        across = [p[0] * normal[0] + p[1] * normal[1] for p in hull]
        bounds = (min(along), max(along), min(across), max(across))
        coincident = 0.0
        for edge in range(count):
            p, q = edge, (edge + 1) % count
            for values, low, high in ((along, bounds[0], bounds[1]),
                                      (across, bounds[2], bounds[3])):
                if any(abs(values[p] - side) <= tolerance_m and
                       abs(values[q] - side) <= tolerance_m for side in (low, high)):
                    coincident += math.hypot(hull[q][0] - hull[p][0],
                                             hull[q][1] - hull[p][1])
                    break
        area = (bounds[1] - bounds[0]) * (bounds[3] - bounds[2])
        key = (-round(coincident, 6), area)
        if best is None or key < best[0]:
            best = (key, axis, normal) + bounds
    if best is None:
        return hull[0], (1.0, 0.0), 0.0, 0.0
    _, axis, normal, a0, a1, c0, c1 = best
    ma, mc = (a0 + a1) / 2.0, (c0 + c1) / 2.0
    centre = (ma * axis[0] + mc * normal[0], ma * axis[1] + mc * normal[1])
    length_a, length_c = a1 - a0, c1 - c0
    if length_c > length_a:
        return centre, normal, length_c, length_a
    return centre, axis, length_a, length_c


def estimate_footprint(points, viewers, footprint_policy: dict) -> dict:
    """Bounds of measured LiDAR hull points, never inferred unseen surfaces.

    A thin visible face has unknown depth. Its center, radius and extent refer
    only to the observed points. Viewpoints are accepted for caller compatibility
    but cannot turn a single visible face into an assumed complete solid.
    """
    hull = convex_hull(points)
    centre, _, long_m, short_m = _fitted_rectangle(hull)
    threshold = max(footprint_policy["single_face_depth_m"],
                    footprint_policy["single_face_depth_fraction"] * long_m)
    depth_observed = short_m >= threshold
    radius = max(math.hypot(p[0] - centre[0], p[1] - centre[1]) for p in hull)
    return {"centre": centre, "radius_m": radius, "hull": hull,
            "depth_observed": depth_observed,
            "visible_width_m": long_m, "visible_depth_m": short_m,
            "extent": {"min_north_m": min(p[0] for p in hull),
                       "max_north_m": max(p[0] for p in hull),
                       "min_east_m": min(p[1] for p in hull),
                       "max_east_m": max(p[1] for p in hull)}}


def _round(value, digits=2):
    return None if value is None else round(float(value), digits)


def _point(point, origin_id=""):
    return {"north_m": _round(point[0], 3), "east_m": _round(point[1], 3),
            "frame": "local_ned", "origin_id": origin_id}


class TargetLocalizer:
    """Fuses per-view selections of one described target into an estimate.

    Each call to :meth:`observe` is one view: one image, the VLM's selection
    and the LiDAR/state snapshot taken with that image.  Views of the same
    description within ``history_s`` are compared; agreement and spread are
    purely geometric.  The localizer never moves the vehicle; extra
    viewpoints come from separately authorized yaw actions.
    """

    def __init__(self, config: dict, clock=time.time):
        self.config = config
        self.camera = CameraModel.from_config(config["camera"])
        self.clock = clock
        self.history: dict[str, list[dict]] = {}

    def reset(self, description: str | None = None):
        if description is None:
            self.history.clear()
        else:
            self.history.pop(normalize_description(description), None)

    def observe(self, description, *, vlm_text, state, perception,
                image_captured_at_unix_ms, provenance=(), refreshed_perception=None):
        """Fuse one view.  ``vlm_text`` is the grounding answer, or None when
        the VLM's presence gate reported no match in this image."""
        description = normalize_description(description)
        now_ms = int(self.clock() * 1000)
        pose = vehicle_pose(state)
        views = [view for view in self.history.get(description, [])
                 if now_ms - view["observed_at_unix_ms"] <=
                 self.config["multiview"]["history_s"] * 1000]
        self.history[description] = views
        base = {"schema": SCHEMA, "target": {"kind": "observed", "description": description},
                "description": description, "observed_at_unix_ms": now_ms,
                "image_captured_at_unix_ms": int(image_captured_at_unix_ms),
                "provenance": list(provenance), "flight_authority": False}
        box = parse_grounding_box(vlm_text) if vlm_text is not None else None
        if box is None:
            return base | {"found": False,
                           "reason": ("vlm_reported_no_match" if vlm_text is None
                                      else "vlm_box_unparseable"),
                           "quality": {"method": METHOD, "viewpoints": 0,
                                       "prior_views": len(views)}}
        box_px = box_to_pixels(box, self.camera)
        image_bearing = self.camera.relative_bearing_deg((box_px[0] + box_px[2]) / 2.0)
        heading = math.degrees(pose["yaw_rad"])
        bearing_evidence = {
            "image_relative": _round(image_bearing),
            "image_absolute": _round((heading + image_bearing) % 360.0),
            "image_span_relative": [
                _round(self.camera.relative_bearing_deg(box_px[0])),
                _round(self.camera.relative_bearing_deg(box_px[2]))]}
        perception_age = now_ms - int(perception.get("observed_at_unix_ms", 0) or 0)
        if not perception.get("local_map_available", False):
            return base | {"found": False, "reason": "lidar_perception_unavailable",
                           "bearing_deg": bearing_evidence, "box_1000": list(box),
                           "quality": {"method": METHOD, "viewpoints": 0}}
        candidates = candidates_from_perception(perception)
        best, ranked = associate(box_px, candidates, pose, self.camera,
                                 self.config["association"])
        evidence = [{"id": item["candidate"]["id"],
                     "image_iou": _round(item["image_iou"], 3),
                     "bearing_overlap": _round(item["bearing_overlap"], 3),
                     "lidar_returns": item["candidate"]["point_count"],
                     "surface_range_m": _round(item["projection"]["surface_range_m"])}
                    for item in ranked[:4]]
        if best is None:
            return base | {"found": False, "reason": "no_lidar_cluster_in_selected_region",
                           "bearing_deg": bearing_evidence, "box_1000": list(box),
                           "candidates": evidence,
                           "quality": {"method": METHOD, "viewpoints": 0,
                                       "lidar_candidates": len(candidates),
                                       "age_ms": perception_age}}
        selected = best["candidate"]
        refreshed = False
        if refreshed_perception is not None:
            fresh = self._track(selected, candidates_from_perception(refreshed_perception))
            if fresh is not None:
                selected, refreshed = fresh, True
        single = estimate_footprint(selected["hull"], [(pose["north_m"], pose["east_m"])],
                                    self.config["footprint"])
        view = {"observed_at_unix_ms": now_ms,
                "vehicle": (pose["north_m"], pose["east_m"]),
                "heading_deg": heading,
                "hull": selected["hull"], "centre": single["centre"],
                "top_down_m": selected["top_down_m"],
                "top_height_m": pose["terrain_down_m"] - selected["top_down_m"],
                "top_clipped": selected["top_clipped"],
                "point_count": selected["point_count"],
                "scan_observed_at_unix_ms": selected["observed_at_unix_ms"]}
        surface, surface_range = _nearest_on_hull(selected["hull"], pose["north_m"],
                                                  pose["east_m"])
        tolerance = max(self.config["multiview"]["agreement_tolerance_m"],
                        self.config["multiview"]["agreement_range_fraction"] * surface_range)
        agreeing = [prior for prior in views
                    if math.hypot(prior["centre"][0] - view["centre"][0],
                                  prior["centre"][1] - view["centre"][1]) <= tolerance]
        disagreeing = len(views) - len(agreeing)
        views.append(view)
        consensus = agreeing + [view]
        fused = estimate_footprint([p for item in consensus for p in item["hull"]],
                                   [item["vehicle"] for item in consensus],
                                   self.config["footprint"])
        spread = max((math.hypot(a["centre"][0] - b["centre"][0],
                                 a["centre"][1] - b["centre"][1])
                      for a in consensus for b in consensus), default=0.0)
        measured_top_down = min((item['top_down_m'] for item in consensus
                                 if not item['top_clipped']), default=None)
        height = pose['terrain_down_m'] - measured_top_down if measured_top_down is not None else None
        centre = fused["centre"]
        absolute = math.degrees(math.atan2(centre[1] - pose["east_m"],
                                           centre[0] - pose["north_m"])) % 360.0
        origin = pose["origin_id"]
        age_ms = int(self.clock() * 1000) - selected["observed_at_unix_ms"]
        return base | {
            "found": True,
            # Relative bearings refer to the heading when the frame was taken.
            "vehicle_at_capture": {"north_m": _round(pose["north_m"], 3),
                                   "east_m": _round(pose["east_m"], 3),
                                   "down_m": _round(pose["down_m"], 3),
                                   "heading_deg": _round(heading % 360.0)},
            "bearing_deg": {"relative": _round(_wrap_deg(absolute - heading)),
                            "absolute": _round(absolute)} | bearing_evidence,
            "surface_range_m": _round(surface_range),
            "surface_point": _point(surface, origin),
            "estimated_position": _point(centre, origin),
            "position_basis": "center_of_observed_lidar_extent",
            "footprint": {
                "radius_m": _round(fused["radius_m"]),
                "extent": {key: _round(value, 3) for key, value in fused["extent"].items()},
                "hull": [_point(p, origin) for p in fused["hull"]],
                "depth_observed": fused["depth_observed"],
                "geometry_basis": "observed_lidar_hull_only",
                "whole_shape_observed": False,
                "visible_width_m": _round(fused["visible_width_m"]),
                "visible_depth_m": _round(fused["visible_depth_m"]),
                "height_m": _round(height),
                "top_down_m": _round(measured_top_down),
                "home_down_m": _round(pose['terrain_down_m']),
                "height_reference": "home_arming_altitude",
                "height_clipped": height is None,
                "height_lower_bound_m": _round(max(item["top_height_m"] for item in consensus)),
            },
            "box_1000": list(box),
            "candidates": evidence,
            "quality": {
                "method": METHOD,
                "lidar_returns": selected["point_count"],
                "viewpoints": len(consensus),
                "viewpoints_agree": len(consensus) >= 2 and disagreeing == 0,
                "disagreeing_viewpoints": disagreeing,
                "position_spread_m": _round(spread),
                "age_ms": age_ms,
                "image_iou": _round(best["image_iou"], 3),
                "bearing_overlap": _round(best["bearing_overlap"], 3),
                "lidar_refreshed_after_vlm": refreshed,
            },
            "lidar_cluster_id": selected["id"],
            "lidar_sensor_id": selected["sensor_id"],
        }

    @staticmethod
    def _track(selected, fresh_candidates, tolerance_m=1.5):
        """Find the same physical cluster in a later scan by hull proximity."""
        best, best_distance = None, math.inf
        for candidate in fresh_candidates:
            distance = math.hypot(candidate["centroid"][0] - selected["centroid"][0],
                                  candidate["centroid"][1] - selected["centroid"][1])
            if distance < best_distance:
                best, best_distance = candidate, distance
        return best if best_distance <= tolerance_m else None

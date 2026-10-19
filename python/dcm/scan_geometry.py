"""Metric scan geometry in one immutable heading frame.

This module produces paths, not promises of sensor coverage. ``lane_spacing_m``
should come from the usable sensor swath and desired overlap. Rectangle width is
rightward and length is forward from the initial corner. Circle centre is one
radius forward of the initial perimeter point. Heading is clockwise from north.
No vehicle motion or mission-language interpretation occurs here.
"""
from __future__ import annotations

import math
from numbers import Real

_MAX_DIMENSION_M = 10_000.0


def _number(name: str, value: object, *, positive: bool = False) -> float:
    if isinstance(value, bool) or not isinstance(value, Real):
        raise ValueError(f"{name} must be a finite number")
    result = float(value)
    if not math.isfinite(result) or (positive and result <= 0):
        raise ValueError(f"{name} must be a finite {'positive ' if positive else ''}number")
    return result


def _dimension(name: str, value: object) -> float:
    result = _number(name, value, positive=True)
    if result > _MAX_DIMENSION_M:
        raise ValueError(f"{name} exceeds {_MAX_DIMENSION_M:g} m geometry limit")
    return result


def generate_scan(
    *,
    shape: str,
    coverage: str,
    start_north_m: float,
    start_east_m: float,
    heading_deg: float,
    altitude_agl_m: float,
    width_m: float | None = None,
    length_m: float | None = None,
    radius_m: float | None = None,
    lane_spacing_m: float | None = None,
    clockwise: bool = True,
    max_points: int = 256,
    max_segment_m: float = 5.0,
) -> list[dict[str, float]]:
    """Generate a rectangle/square or circle perimeter or parallel-lane path.

    The returned first waypoint is the input position. Perimeters close exactly;
    area paths finish at the last lane rather than adding an unrequested return.
    Rectangle clockwise traversal starts forward; counterclockwise starts right.
    Circle traversal uses chords of at most ``max_segment_m`` and 22.5 degrees.
    Area lanes include both extremes and are separated by no more than the
    requested spacing; connecting segments stay inside the convex scan boundary.
    ``clockwise`` controls perimeter direction only. ``max_points`` is a resource
    bound and raises rather than silently degrading path resolution.
    """
    if shape not in {"rectangle", "square", "circle"}:
        raise ValueError("shape must be rectangle, square, or circle")
    if coverage not in {"perimeter", "area"}:
        raise ValueError("coverage must be perimeter or area")
    if not isinstance(clockwise, bool):
        raise ValueError("clockwise must be a boolean")
    if isinstance(max_points, bool) or not isinstance(max_points, int) or not 4 <= max_points <= 100_000:
        raise ValueError("max_points must be an integer between 4 and 100000")
    north = _number("start_north_m", start_north_m)
    east = _number("start_east_m", start_east_m)
    heading = math.radians(_number("heading_deg", heading_deg) % 360.0)
    altitude = _number("altitude_agl_m", altitude_agl_m)
    if altitude < 0:
        raise ValueError("altitude_agl_m must be nonnegative")
    segment = _dimension("max_segment_m", max_segment_m)
    spacing = _dimension("lane_spacing_m", lane_spacing_m) if lane_spacing_m is not None else None
    if coverage == "area" and spacing is None:
        raise ValueError("area paths require lane_spacing_m derived from sensor swath")

    points: list[tuple[float, float]]
    if shape in {"rectangle", "square"}:
        if radius_m is not None:
            raise ValueError("radius_m is only valid for circles")
        width = _dimension("width_m", width_m)
        length = width if shape == "square" and length_m is None else _dimension("length_m", length_m)
        if shape == "square" and not math.isclose(width, length, rel_tol=1e-9, abs_tol=1e-9):
            raise ValueError("a square requires equal width_m and length_m")
        if coverage == "perimeter":
            points = [(0.0, 0.0), (length, 0.0), (length, width), (0.0, width), (0.0, 0.0)]
            if not clockwise:
                points.reverse()
        else:
            assert spacing is not None
            # Divide length uniformly instead of leaving a narrow final lane.
            lanes = _bounded_intervals(length, spacing, max_points)
            _check_count(2 * (lanes + 1), max_points)
            points = []
            for i in range(lanes + 1):
                forward = length * i / lanes
                edges = (0.0, width) if i % 2 == 0 else (width, 0.0)
                points.extend((forward, right) for right in edges)
    else:
        if width_m is not None or length_m is not None:
            raise ValueError("width_m and length_m are only valid for rectangles/squares")
        radius = _dimension("radius_m", radius_m)
        if coverage == "perimeter":
            # Arc length bound is conservative for chord length and avoids tiny
            # asin arguments underflowing at extreme requested resolutions.
            intervals = max(16, _bounded_intervals(2.0 * math.pi * radius, segment, max_points))
            _check_count(intervals + 1, max_points)
            direction = 1.0 if clockwise else -1.0
            points = [(0.0, 0.0)]
            for i in range(1, intervals):
                angle = direction * 2.0 * math.pi * i / intervals
                points.append((radius * (1.0 - math.cos(angle)), -radius * math.sin(angle)))
            points.append((0.0, 0.0))
        else:
            assert spacing is not None
            # At least two intervals retain the diameter lane even when a
            # supplied swath is wider than the whole circle.
            lanes = max(2, _bounded_intervals(2.0 * radius, spacing, max_points))
            _check_count(2 * lanes, max_points)
            points = [(0.0, 0.0)]
            for i in range(1, lanes):
                forward = 2.0 * radius * i / lanes
                half_chord = math.sqrt(max(0.0, radius * radius - (forward - radius) ** 2))
                edges = (-half_chord, half_chord) if i % 2 else (half_chord, -half_chord)
                points.extend((forward, right) for right in edges)
            points.append((2.0 * radius, 0.0))

    _check_count(len(points), max_points)
    cosine, sine = math.cos(heading), math.sin(heading)
    result = [
        {
            "north_m": north + forward * cosine - right * sine,
            "east_m": east + forward * sine + right * cosine,
            "altitude_agl_m": altitude,
        }
        for forward, right in points
    ]
    if not all(math.isfinite(v) for point in result for v in point.values()):
        raise ValueError("generated coordinates are not finite")
    return result


def _bounded_intervals(distance: float, spacing: float, max_points: int) -> int:
    ratio = distance / spacing
    if not math.isfinite(ratio) or ratio > max_points:
        raise ValueError(f"scan resolution exceeds max_points={max_points}")
    return max(1, math.ceil(ratio))


def _check_count(count: int, max_points: int) -> None:
    if count > max_points:
        raise ValueError(f"scan requires {count} waypoints, exceeding max_points={max_points}")

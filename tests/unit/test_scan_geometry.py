"""Frame, shape and bounded-resource contracts for scan geometry."""
import math

import pytest

from python.dcm.scan_geometry import generate_scan


def make(**overrides):
    arguments = dict(shape="rectangle", coverage="perimeter", start_north_m=0,
                     start_east_m=0, heading_deg=0, altitude_agl_m=4,
                     width_m=1, length_m=1)
    arguments.update(overrides)
    return generate_scan(**arguments)


def pairs(points):
    return [(p["north_m"], p["east_m"]) for p in points]


def test_one_metre_square_right_then_forward():
    assert pairs(make(clockwise=False)) == [(0, 0), (0, 1), (1, 1), (1, 0), (0, 0)]
    assert all(p["altitude_agl_m"] == 4 for p in make())


def test_square_alias_and_clockwise_reverse():
    square = make(shape="square", length_m=None)
    assert square == make()
    assert make(clockwise=False) == list(reversed(square))


@pytest.mark.parametrize("shape,coverage", [("rectangle", "perimeter"), ("rectangle", "area"),
                                              ("circle", "perimeter"), ("circle", "area")])
def test_rotation_translation_invariance(shape, coverage):
    dimensions = dict(width_m=3, length_m=5) if shape == "rectangle" else dict(width_m=None, length_m=None, radius_m=3)
    baseline = make(shape=shape, coverage=coverage, lane_spacing_m=1, **dimensions)
    moved = make(shape=shape, coverage=coverage, lane_spacing_m=1, start_north_m=17,
                 start_east_m=-23, heading_deg=37, **dimensions)
    c, s = math.cos(math.radians(37)), math.sin(math.radians(37))
    for a, b in zip(baseline, moved):
        assert b["north_m"] == pytest.approx(17 + a["north_m"] * c - a["east_m"] * s)
        assert b["east_m"] == pytest.approx(-23 + a["north_m"] * s + a["east_m"] * c)


def test_heading_90_right_is_south_forward_is_east():
    points = make(heading_deg=90, clockwise=False)
    assert points[1]["north_m"] == pytest.approx(-1)
    assert points[1]["east_m"] == pytest.approx(0, abs=1e-12)
    assert points[2]["north_m"] == pytest.approx(-1)
    assert points[2]["east_m"] == pytest.approx(1)


def test_rectangle_area_alternates_includes_edges_and_bounds_spacing():
    points = pairs(make(coverage="area", width_m=6, length_m=7, lane_spacing_m=2))
    lanes = sorted(set(n for n, _ in points))
    assert lanes[0] == 0 and lanes[-1] == 7
    assert max(b-a for a,b in zip(lanes, lanes[1:])) <= 2
    assert points[:4] == [(0, 0), (0, 6), (1.75, 6), (1.75, 0)]
    assert all(0 <= n <= 7 and 0 <= e <= 6 for n,e in points)
    assert points[-1] != points[0]  # No unrequested return leg.


def circle(**kwargs):
    return make(shape="circle", width_m=None, length_m=None, radius_m=3, **kwargs)


def test_circle_closure_radius_and_direction():
    points = pairs(circle())
    assert points[0] == points[-1] == (0, 0)
    assert points[1][1] < 0  # Starting south of centre: clockwise initially west.
    assert all(math.hypot(n-3, e) == pytest.approx(3) for n,e in points)
    reverse = pairs(circle(clockwise=False))
    for a,b in zip(points, reverse):
        assert a[0] == pytest.approx(b[0])
        assert a[1] == pytest.approx(-b[1])


def test_circle_resolution_bounds_chords():
    points = pairs(circle(max_segment_m=0.25))
    assert all(math.dist(a,b) <= .25 for a,b in zip(points, points[1:]))


def test_circle_area_chords_and_connectors_stay_inside_and_spacing_bounded():
    points = pairs(circle(coverage="area", lane_spacing_m=.7))
    assert points[0] == (0,0) and points[-1] == (6,0)
    assert all(math.hypot(n-3,e) <= 3+1e-9 for n,e in points)
    for a,b in zip(points, points[1:]):
        midpoint = ((a[0]+b[0])/2, (a[1]+b[1])/2)
        assert math.hypot(midpoint[0]-3,midpoint[1]) <= 3+1e-9
    lanes = sorted(set(n for n,e in points))
    assert max(b-a for a,b in zip(lanes, lanes[1:])) <= .7


def test_wide_circle_swath_still_includes_diameter():
    assert len(circle(coverage="area", lane_spacing_m=100)) == 4


@pytest.mark.parametrize("kwargs", [
    {"shape":"triangle"}, {"coverage":"full"}, {"width_m":0}, {"length_m":-1},
    {"width_m":float("nan")}, {"start_east_m":float("inf")}, {"heading_deg":True},
    {"width_m":"1"}, {"altitude_agl_m":-1}, {"clockwise":1}, {"max_points":True},
    {"max_points":3}, {"max_points":1.5}, {"width_m":10001}, {"radius_m":2},
    {"coverage":"area"}, {"coverage":"area","lane_spacing_m":0},
    {"shape":"square","length_m":2}, {"max_segment_m":0},
    {"coverage":"area","lane_spacing_m":1e-320},
])
def test_invalid_inputs(kwargs):
    with pytest.raises(ValueError):
        make(**kwargs)


def test_point_budget_does_not_silently_coarsen():
    with pytest.raises(ValueError, match="max_points"):
        circle(max_segment_m=.01)
    with pytest.raises(ValueError, match="max_points"):
        make(coverage="area", width_m=100, length_m=100, lane_spacing_m=.01)
    with pytest.raises(ValueError, match="max_points"):
        make(max_points=4)


def test_circle_rejects_rectangle_parameters():
    with pytest.raises(ValueError, match="only valid"):
        make(shape="circle", radius_m=2)

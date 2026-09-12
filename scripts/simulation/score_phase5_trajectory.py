#!/usr/bin/python3
"""Score a sampled ENU trajectory against Phase 5 ground truth."""

import argparse
import json
import math
from itertools import pairwise
from pathlib import Path

from scenario_config import load_scenario


def distance(a, b):
    return math.sqrt(sum((x - y) ** 2 for x, y in zip(a, b)))


def segment_point_distance(a, b, point):
    delta = [b[i] - a[i] for i in range(3)]
    length2 = sum(value * value for value in delta)
    if not length2:
        return distance(a, point)
    t = max(0.0, min(1.0, sum((point[i] - a[i]) * delta[i] for i in range(3)) / length2))
    closest = [a[i] + t * delta[i] for i in range(3)]
    return distance(closest, point)


def segment_box_hit(a, b, center, size, margin):
    low = [center[i] - size[i] / 2 - margin for i in range(3)]
    high = [center[i] + size[i] / 2 + margin for i in range(3)]
    enter, leave = 0.0, 1.0
    for i in range(3):
        delta = b[i] - a[i]
        if abs(delta) < 1e-12:
            if a[i] < low[i] or a[i] > high[i]:
                return False
            continue
        first, second = (low[i] - a[i]) / delta, (high[i] - a[i]) / delta
        if first > second:
            first, second = second, first
        enter, leave = max(enter, first), min(leave, second)
        if enter > leave:
            return False
    return True


def collisions(scenario, a, b, radius):
    hits = []
    for obstacle in scenario["obstacles"]:
        if obstacle["type"] != "tree":
            if segment_box_hit(a, b, obstacle["center_m"], obstacle["size_m"], radius):
                hits.append(obstacle["name"])
            continue
        x, y = obstacle["center_m"][:2]
        trunk_center = [x, y, obstacle["height_m"] / 2]
        trunk_size = [2 * obstacle["radius_m"], 2 * obstacle["radius_m"], obstacle["height_m"]]
        if segment_box_hit(a, b, trunk_center, trunk_size, radius):
            hits.append(obstacle["name"] + ":trunk")
        if obstacle["canopy_collision"] == "enabled":
            canopy = [x, y, obstacle["height_m"] + obstacle["canopy_radius_m"] * 0.65]
            if segment_point_distance(a, b, canopy) <= obstacle["canopy_radius_m"] + radius:
                hits.append(obstacle["name"] + ":canopy")
    return hits


def score(scenario, points, route_name=None, vehicle_radius_m=0.35):
    if len(points) < 2:
        raise ValueError("trajectory needs at least two points")
    times = [point["t_s"] for point in points]
    if any(b <= a for a, b in pairwise(times)):
        raise ValueError("trajectory times must strictly increase")
    xyz = [[point[key] for key in ("x_m", "y_m", "z_m")] for point in points]
    hits = []
    for a, b in pairwise(xyz):
        hits.extend(collisions(scenario, a, b, vehicle_radius_m))
    route, goal_error = None, None
    if route_name:
        route = next((item for item in scenario["ground_truth"].get("routes", []) if item["name"] == route_name), None)
        if route is None:
            raise ValueError("unknown route: " + route_name)
        goal_error = distance(xyz[-1], route["waypoints_enu_m"][-1])
    duration = times[-1] - times[0]
    checks = {
        "collision_free": not hits,
        "within_duration": duration <= scenario["maximum_duration_s"],
        "goal_reached": goal_error is None or goal_error <= 1.0,
    }
    return {
        "status": "passed" if all(checks.values()) else "failed",
        "checks": checks,
        "collisions": sorted(set(hits)),
        "duration_s": duration,
        "route": route_name,
        "goal_error_m": goal_error,
        "vehicle_radius_m": vehicle_radius_m,
    }


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("scenario")
    parser.add_argument("trajectory", type=Path, help="JSON object with a points array")
    parser.add_argument("--route")
    parser.add_argument("--vehicle-radius-m", type=float, default=0.35)
    parser.add_argument("--output", type=Path)
    args = parser.parse_args()
    _, scenario = load_scenario(args.scenario)
    points = json.loads(args.trajectory.read_text())["points"]
    result = score(scenario, points, args.route, args.vehicle_radius_m)
    text = json.dumps(result, indent=2) + "\n"
    if args.output:
        args.output.write_text(text)
    print(text, end="")
    raise SystemExit(0 if result["status"] == "passed" else 1)


if __name__ == "__main__":
    main()

#!/usr/bin/python3
"""Generate deterministic SDF and ground truth from a Phase 5 scenario."""

import argparse
import copy
import hashlib
import json
import math
import subprocess
import xml.etree.ElementTree as ET

from build_akshu_candidate import ROOT
from scenario_config import canonical_bytes, load_scenario


def element(parent, tag, text=None, **attributes):
    child = ET.SubElement(parent, tag, attributes)
    if text is not None:
        child.text = str(text)
    return child


def material(visual, rgba):
    node = element(visual, "material")
    element(node, "ambient", rgba)
    element(node, "diffuse", rgba)


def box_visual(link, name, pose, size, rgba):
    node = element(link, "visual", name=name)
    element(node, "pose", " ".join(map(str, pose)) + " 0 0 0")
    shape = element(element(node, "geometry"), "box")
    element(shape, "size", " ".join(map(str, size)))
    material(node, rgba)
    return node


def box_model(world, item):
    model = element(world, "model", name=item["name"])
    element(model, "static", "true")
    cx, cy, cz = item["center_m"]
    element(model, "pose", f"{cx} {cy} {cz} 0 0 0")
    link = element(model, "link", name="structure")
    for kind in ("collision", "visual"):
        node = element(link, kind, name=kind)
        geometry = element(node, "geometry")
        shape = element(geometry, "box")
        element(shape, "size", " ".join(map(str, item["size_m"])))
        if kind == "visual":
            colour = "0.48 0.50 0.53 1" if item["type"] == "building" else "0.62 0.34 0.16 1"
            material(node, colour)
    if item["type"] == "building":
        sx, sy, sz = item["size_m"]
        box_visual(link, "roof", [0, 0, sz / 2 + 0.16], [sx + 0.35, sy + 0.35, 0.32], "0.22 0.10 0.07 1")
        box_visual(link, "door", [sx / 2 + 0.012, -sy * 0.22, -sz / 2 + 1.05], [0.025, 1.05, 2.1], "0.20 0.13 0.08 1")
        floors = (0,) if sz < 5.5 else (-sz * 0.22, sz * 0.22)
        for floor_index, z in enumerate(floors):
            for side_index, y in enumerate((-sy * 0.28, sy * 0.28)):
                box_visual(link, f"window_front_{floor_index}_{side_index}", [sx / 2 + 0.014, y, z], [0.03, min(1.25, sy * 0.22), 0.85], "0.20 0.52 0.68 1")
                box_visual(link, f"window_back_{floor_index}_{side_index}", [-sx / 2 - 0.014, y, z], [0.03, min(1.25, sy * 0.22), 0.85], "0.20 0.52 0.68 1")


def tree_model(world, item):
    model = element(world, "model", name=item["name"])
    element(model, "static", "true")
    x, y = item["center_m"][:2]
    element(model, "pose", f"{x} {y} 0 0 0 0")
    link = element(model, "link", name="tree")
    trunk_height, radius = item["height_m"], item["radius_m"]
    for kind in ("collision", "visual"):
        node = element(link, kind, name="trunk_" + kind)
        element(node, "pose", f"0 0 {trunk_height / 2} 0 0 0")
        shape = element(element(node, "geometry"), "cylinder")
        element(shape, "radius", radius)
        element(shape, "length", trunk_height)
        if kind == "visual":
            material(node, "0.30 0.16 0.07 1")
    canopy_z = trunk_height + item["canopy_radius_m"] * 0.65
    kinds = ["visual"]
    if item["canopy_collision"] == "enabled":
        kinds.insert(0, "collision")
    for kind in kinds:
        node = element(link, kind, name="canopy_" + kind)
        element(node, "pose", f"0 0 {canopy_z} 0 0 0")
        sphere = element(element(node, "geometry"), "sphere")
        element(sphere, "radius", item["canopy_radius_m"])
        if kind == "visual":
            material(node, "0.12 0.38 0.10 1")
    for index, (dx, dy, dz, scale) in enumerate((
        (-0.65, 0, 0.15, 0.72), (0.62, 0.12, 0.10, 0.70),
        (0, -0.58, 0.28, 0.68), (0.05, 0.58, 0.20, 0.66),
    )):
        node = element(link, "visual", name=f"canopy_cluster_{index}")
        element(node, "pose", f"{dx} {dy} {canopy_z + dz} 0 0 0")
        sphere = element(element(node, "geometry"), "sphere")
        element(sphere, "radius", item["canopy_radius_m"] * scale)
        material(node, "0.10 0.32 0.08 1")


def decorate_site(world):
    ground = world.find("model[@name='test_pad']/link/visual[@name='ground']/material")
    if ground is not None:
        ground.find("ambient").text = "0.16 0.30 0.14 1"
        ground.find("diffuse").text = "0.20 0.38 0.17 1"
    model = element(world, "model", name="village_ground_details")
    element(model, "static", "true")
    link = element(model, "link", name="details")
    box_visual(link, "road_east_west", [0, -4.5, 0.012], [60, 5.5, 0.024], "0.12 0.13 0.14 1")
    box_visual(link, "road_branch", [7.5, 10, 0.013], [5.5, 29, 0.026], "0.12 0.13 0.14 1")
    for index, x in enumerate(range(-26, 29, 4)):
        box_visual(link, f"road_mark_{index}", [x, -4.5, 0.029], [2, 0.12, 0.01], "0.92 0.78 0.18 1")
    box_visual(link, "footpath", [0, -1.45, 0.025], [60, 0.55, 0.05], "0.58 0.55 0.49 1")


def build(scenario_value):
    _path, scenario = load_scenario(scenario_value)
    model_dir = ROOT / "simulation/models" / ("phase5_" + scenario["name"])
    model_dir.mkdir(parents=True, exist_ok=True)
    subprocess.run(
        [
            "/usr/bin/python3",
            str(ROOT / "scripts/simulation/build_compact_flight.py"),
            "--sensor-profile",
            scenario["sensor_profile"],
            "--target-model-dir",
            str(model_dir),
            "--model-only",
        ],
        check=True,
        capture_output=True,
        text=True,
        timeout=30,
    )
    model_tree = ET.parse(model_dir / "model.sdf")
    model = model_tree.getroot().find("model")
    navsat = model.find("link/sensor[@name='navsat']/navsat")
    degraded = scenario["gps_degradation"]
    for direction in ("horizontal", "vertical"):
        navsat.find(f"position_sensing/{direction}/noise/stddev").text = str(
            degraded["position_stddev_m"]
        )
        navsat.find(f"velocity_sensing/{direction}/noise/stddev").text = str(
            degraded["velocity_stddev_m_s"]
        )
    battery = model.find("plugin[@name='gz::sim::systems::LinearBatteryPlugin']")
    battery.find("initial_charge").text = str(
        10.0 * scenario["battery"]["initial_soc"]
    )
    battery.find("power_load").text = str(scenario["battery"]["load_w"])
    ET.indent(model_tree)
    model_xml = ET.tostring(model_tree.getroot(), encoding="unicode") + "\n"
    (model_dir / "model.sdf").write_text(model_xml)
    (model_dir / "model.config").write_text(
        '<model><name>Phase 5 ' + scenario["name"]
        + '</name><version>1.0</version><sdf version="1.9">model.sdf</sdf>'
        + '<description>Generated, scenario-specific Icarus compact model.</description></model>\n'
    )
    source = ET.parse(ROOT / "simulation/worlds/compact_flight.sdf")
    root = source.getroot()
    world = root.find("world")
    world_name = "icarus_" + scenario["name"]
    world.set("name", world_name)
    pose = scenario["initial_pose"]
    vehicle = world.find("include[name='icarus_compact']")
    vehicle.find("uri").text = "model://phase5_" + scenario["name"]
    vehicle.find("pose").text = (
        f"{pose['x_m']} {pose['y_m']} {pose['z_m']} 0 0 "
        f"{math.radians(pose['yaw_deg'])}"
    )
    spherical = world.find("spherical_coordinates")
    home = scenario["home"]
    for tag, key in (("latitude_deg", "latitude_deg"), ("longitude_deg", "longitude_deg"),
                     ("elevation", "elevation_m"), ("heading_deg", "heading_deg")):
        spherical.find(tag).text = str(home[key])

    preset_path = ROOT / "config/simulation/environment_presets.json"
    presets = json.loads(preset_path.read_text())
    preset = scenario["environment_preset"]
    obstacles = copy.deepcopy(presets.get(preset, {}).get("obstacles", []))
    obstacles.extend(copy.deepcopy(scenario["obstacles"]))
    if obstacles:
        decorate_site(world)

    wind = scenario["wind"]
    if wind["enabled"]:
        angle = math.radians(wind["direction_deg"])
        wind_node = element(world, "wind")
        element(wind_node, "linear_velocity", f"{wind['speed_m_s'] * math.cos(angle):.9f} {wind['speed_m_s'] * math.sin(angle):.9f} 0")
        plugin = element(world, "plugin", filename="gz-sim-wind-effects-system", name="gz::sim::systems::WindEffects")
        element(plugin, "force_approximation_scaling_factor", "1")
        horizontal = element(plugin, "horizontal")
        magnitude = element(horizontal, "magnitude")
        element(magnitude, "time_for_rise", wind["rise_time_s"])
        if wind["gust_amplitude_fraction"]:
            sine = element(magnitude, "sin")
            element(sine, "amplitude_percent", wind["gust_amplitude_fraction"])
            element(sine, "period", wind["gust_period_s"])
        direction = element(horizontal, "direction")
        element(direction, "time_for_rise", wind["rise_time_s"])
        if wind["direction_swing_deg"]:
            sine = element(direction, "sin")
            element(sine, "amplitude", wind["direction_swing_deg"])
            element(sine, "period", wind["direction_period_s"])

    for item in obstacles:
        (tree_model if item["type"] == "tree" else box_model)(world, item)

    output_dir = ROOT / "simulation/worlds/generated"
    output_dir.mkdir(parents=True, exist_ok=True)
    output = output_dir / (scenario["name"] + ".sdf")
    ET.indent(source)
    xml = ET.tostring(root, encoding="unicode") + "\n"
    output.write_text(xml)
    truth = {
        "scenario": scenario["name"],
        "world": world_name,
        "seed": scenario["seed"],
        "home": scenario["home"],
        "initial_pose": scenario["initial_pose"],
        "wind": scenario["wind"],
        "environment_preset": preset,
        "environment_preset_sha256": hashlib.sha256(preset_path.read_bytes()).hexdigest(),
        "obstacles": obstacles,
        "routes": scenario["ground_truth"].get("routes", []),
        "world_sha256": hashlib.sha256(xml.encode()).hexdigest(),
        "scenario_sha256": hashlib.sha256(canonical_bytes(scenario)).hexdigest(),
        "vehicle_model_sha256": hashlib.sha256(model_xml.encode()).hexdigest(),
    }
    truth_path = output.with_suffix(".ground_truth.json")
    truth_path.write_text(json.dumps(truth, indent=2, sort_keys=True) + "\n")
    return output, truth_path, scenario, truth


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("scenario")
    args = parser.parse_args()
    output, truth, _, _ = build(args.scenario)
    print(output)
    print(truth)


if __name__ == "__main__":
    main()

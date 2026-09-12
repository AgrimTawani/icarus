#!/usr/bin/python3
"""Compose the compact visuals/sensors with provisional rigid-body flight physics."""

import argparse
import copy
import json
import math
import xml.etree.ElementTree as ET
from pathlib import Path

from build_akshu_candidate import ROOT
from vehicle_mass_properties import calculate, component_inertia, load_manifest


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--sensor-profile", choices=("physical",), default="physical"
    )
    parser.add_argument("--target-model-dir", type=Path, default=None)
    parser.add_argument(
        "--model-only",
        action="store_true",
        help="Build only the selected model output; leave shared world/GUI sources unchanged",
    )
    args = parser.parse_args()
    target = args.target_model_dir or ROOT / "simulation/models/akshu_compact_sitl"
    if not target.is_absolute():
        target = ROOT / target
    target.mkdir(parents=True, exist_ok=True)
    tree = ET.parse(ROOT / "simulation/models/akshu_compact/model.sdf")
    model = tree.getroot().find("model")
    model.set("name", "icarus_compact")
    model.set("canonical_link", "base_link")
    model.find("static").text = "false"
    ET.SubElement(model, "self_collide").text = "false"
    base = model.find("link")
    # Native Gazebo WindEffects only applies aerodynamic force to opted-in links.
    # Keeping this enabled in still air has no effect and lets Phase 5 worlds
    # select wind without maintaining a second vehicle model.
    ET.SubElement(base, "enable_wind").text = "true"
    original = (
        ET.parse(ROOT / "simulation/models/mark4_v2/model.sdf").getroot().find("model")
    )
    imu = copy.deepcopy(original.find("link/sensor"))
    imu.find("pose").text = "0 0 -0.026 3.141592653589793 0 0"
    base.append(imu)  # Dedicated FRD IMU for SITL; public recording IMU stays FLU.
    manifest_path = ROOT / "config/simulation/vehicle_components.json"
    manifest, manifest_sha256 = load_manifest(manifest_path)
    components = manifest["components"]
    base_components = [item for item in components if item["link"] == "base_link"]
    propellers = {
        int(item["id"].split("_")[-1]): item
        for item in components
        if item["id"].startswith("propeller_")
    }
    if set(propellers) != {1, 2, 3, 4}:
        raise ValueError("vehicle manifest must define propellers 1..4")
    layout = [
        {"motor": number, "position_m": propellers[number]["center_m"]}
        for number in range(1, 5)
    ]
    mass, cg, inertia = calculate(base_components)
    total_mass, total_cg, total_inertia = calculate(components)
    inertial = ET.SubElement(base, "inertial")
    ET.SubElement(inertial, "mass").text = str(mass)
    ET.SubElement(inertial, "pose").text = " ".join(map(str, cg)) + " 0 0 0"
    tensor = ET.SubElement(inertial, "inertia")
    for name, i, j in [
        ("ixx", 0, 0),
        ("iyy", 1, 1),
        ("izz", 2, 2),
        ("ixy", 0, 1),
        ("ixz", 0, 2),
        ("iyz", 1, 2),
    ]:
        ET.SubElement(tensor, name).text = str(inertia[i, j])

    def collision(name, center, size, yaw=0):
        node = ET.SubElement(base, "collision", name=name)
        ET.SubElement(node, "pose").text = " ".join(map(str, center)) + f" 0 0 {yaw}"
        shape = ET.SubElement(ET.SubElement(node, "geometry"), "box")
        ET.SubElement(shape, "size").text = " ".join(map(str, size))
        return node

    collision("chassis_collision", [0, 0, -0.026], [0.22, 0.105, 0.042])
    collision("battery_collision", [0, 0, -0.11], [0.18, 0.075, 0.055])
    collision("lidar_collision", [0, 0, 0.016], [0.045, 0.045, 0.022])
    collision("camera_collision", [0.121, 0, -0.028], [0.022, 0.070, 0.024])
    for side in (-1, 1):
        node = collision(
            f"landing_{side}", [0, side * 0.085, -0.1841], [0.18, 0.018, 0.012]
        )
        node.append(
            ET.fromstring(
                "<surface><friction><ode><mu>0.9</mu><mu2>0.9</mu2></ode></friction></surface>"
            )
        )
        collision(
            f"arm_{side}",
            [0, 0, -0.0445],
            [0.4438, 0.024, 0.007],
            side * math.atan2(0.173387, 0.1385),
        )
    for m, prop in zip(layout, [28, 26, 25, 27]):
        n = m["motor"]
        rotor = copy.deepcopy(original.find(f"link[@name='motor_{n:02d}']"))
        ET.SubElement(rotor, "enable_wind").text = "true"
        rotor.find("pose").text = " ".join(map(str, m["position_m"])) + " 0 0 0"
        propeller = propellers[n]
        rotor_inertial = rotor.find("inertial")
        rotor_inertial.find("mass").text = str(propeller["mass_kg"])
        rotor_pose = rotor_inertial.find("pose")
        if rotor_pose is None:
            rotor_pose = ET.SubElement(rotor_inertial, "pose")
        rotor_pose.text = "0 0 0 0 0 0"
        propeller_tensor = component_inertia(propeller)
        rotor_tensor = rotor_inertial.find("inertia")
        for name, i, j in [
            ("ixx", 0, 0),
            ("iyy", 1, 1),
            ("izz", 2, 2),
            ("ixy", 0, 1),
            ("ixz", 0, 2),
            ("iyz", 1, 2),
        ]:
            rotor_tensor.find(name).text = str(propeller_tensor[i, j])
        for visual in list(rotor.findall("visual")):
            rotor.remove(visual)
        visual = base.find(f"visual[@name='source_prop_{prop}']")
        base.remove(visual)
        # Existing STL vertices are body-frame metres; cancel link displacement.
        ET.SubElement(visual, "pose").text = (
            " ".join(str(-v) for v in m["position_m"]) + " 0 0 0"
        )
        rotor.append(visual)
        model.append(rotor)
        model.append(
            copy.deepcopy(original.find(f"joint[@name='rotor_{n:02d}_joint']"))
        )
    for plugin in original.findall("plugin"):
        model.append(copy.deepcopy(plugin))
    bridge = (
        ET.parse(ROOT / "simulation/models/mark4_v2_sitl/model.sdf")
        .getroot()
        .find("model")
    )
    for plugin in bridge.findall("plugin"):
        model.append(copy.deepcopy(plugin))
    atmosphere_config = json.loads(
        (ROOT / "config/simulation/atmosphere.json").read_text()
    )
    atmosphere = ET.SubElement(
        model,
        "plugin",
        filename="IcarusTurbulentAtmosphere",
        name="icarus::TurbulentAtmosphere",
    )
    atmosphere_values = {
        "seed": 42,
        "mean_speed": 0,
        "direction_deg": 0,
        "turbulence_intensity": atmosphere_config["minimum_turbulence_intensity"],
        "rise_time": 3,
        "reference_height": atmosphere_config["reference_height_m"],
        "shear_exponent": atmosphere_config["shear_exponent"],
        "air_density": atmosphere_config["air_density_kg_m3"],
        "projected_area": " ".join(map(str, atmosphere_config["projected_area_m2"])),
        "drag_coefficient": " ".join(map(str, atmosphere_config["drag_coefficient"])),
        "rotational_drag": " ".join(map(str, atmosphere_config["rotational_drag"])),
        "center_of_pressure": " ".join(map(str, atmosphere_config["center_of_pressure_m"])),
        "buffeting_moment": atmosphere_config["buffeting_moment_coefficient"],
    }
    for name, value in atmosphere_values.items():
        ET.SubElement(atmosphere, name).text = str(value)
    ET.indent(tree)
    tree.write(target / "model.sdf", encoding="unicode")
    (target / "model.config").write_text(
        '<model><name>Icarus compact SITL</name><version>0.4</version><sdf version="1.9">model.sdf</sdf><description>Component-derived rigid-body model with rotors, physical-noise sensors and ArduPilot.</description></model>\n'
    )
    (target / "mass_properties.json").write_text(
        json.dumps(
            {
                "status": manifest["status"],
                "component_manifest": str(manifest_path.relative_to(ROOT)),
                "component_manifest_sha256": manifest_sha256,
                "total_mass_kg": total_mass,
                "total_cg_m": total_cg.tolist(),
                "total_inertia_kg_m2": total_inertia.tolist(),
                "base_mass_kg": mass,
                "base_cg_m": cg.tolist(),
                "base_inertia_kg_m2": inertia.tolist(),
                "components": components,
                "motor_layout": layout,
            },
            indent=2,
        )
        + "\n"
    )
    world = ET.parse(ROOT / "simulation/worlds/akshu_sensor_validation.sdf")
    w = world.getroot().find("world")
    w.set("name", "compact_flight")
    inc = w.find("include")
    inc.find("uri").text = "model://akshu_compact_sitl"
    inc.find("pose").text = "0 0 .195 0 0 0"
    w.remove(w.find("model[@name='sensor_target']"))
    from sensor_profiles import configure

    configure(model, w, args.sensor_profile)
    ET.indent(tree)
    tree.write(target / "model.sdf", encoding="unicode")
    if args.model_only:
        print("Built compact model only; total mass", total_mass, "kg; base CG", cg)
        return
    # No review camera: user explicitly disabled picture/video capture.
    ET.indent(world)
    world.write(ROOT / "simulation/worlds/compact_flight.sdf", encoding="unicode")
    gui_source = (
        (ROOT / "simulation/launch/mark4_gui.config").read_text().split("?>", 1)[1]
    )
    gui = ET.fromstring("<gui>" + gui_source + "</gui>")
    gui.find(
        "plugin[@filename='MinimalScene']/camera_pose"
    ).text = "3 -4 2.5 0 0.12 2.2143"
    gui.find(
        "plugin[@filename='MinimalScene']/gz-gui/title"
    ).text = "Icarus compact — live SITL flight"
    (ROOT / "simulation/launch/compact_gui.config").write_text(
        "\n".join(ET.tostring(child, encoding="unicode") for child in gui).rstrip()
        + "\n"
    )
    print("Built compact flight model; total mass", total_mass, "kg; base CG", cg)


if __name__ == "__main__":
    main()

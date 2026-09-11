#!/usr/bin/python3
"""Two-level simulation chassis and native Gazebo sensors, not hardware-fit CAD."""

import json
import xml.etree.ElementTree as ET

import numpy as np
from build_akshu_candidate import DTYPE, ROOT, box, cylinder, write_stl

TARGET = ROOT / "simulation/models/akshu_compact"


def main():
    meshes = TARGET / "meshes"
    meshes.mkdir(parents=True, exist_ok=True)
    parts = []

    def add(name, geo, color):
        parts.append((name, geo, color))

    dark, accent = "0.10 0.13 0.17 1", "0.95 0.38 0.08 1"
    previous = ROOT / "simulation/models/akshu_icarus_v1"
    for p in json.loads((previous / "packaging.json").read_text())["parts"]:
        if p["name"].startswith(
            (
                "source_arm",
                "source_prop",
                "extended_leg",
                "leg_extension",
                "motor_",
                "shaft_",
            )
        ):
            geo = (
                np.frombuffer(
                    (previous / "meshes" / (p["name"] + ".stl")).read_bytes(),
                    dtype=DTYPE,
                    offset=84,
                )["v"].astype(float)
                * 1000
            )
            add(p["name"], geo, p["color"])

    def plate(z):
        outline = np.array(
            [
                [-110, -39],
                [-92, -55],
                [88, -55],
                [118, -30],
                [118, 30],
                [88, 55],
                [-92, 55],
                [-110, 39],
            ]
        )
        faces = []
        for i in range(len(outline)):
            a, b = outline[i], outline[(i + 1) % len(outline)]
            bottom = [[*a, z - 2], [*b, z - 2]]
            top = [[*a, z + 2], [*b, z + 2]]
            faces.extend(
                [
                    [[0, 0, z - 2], bottom[1], bottom[0]],
                    [[0, 0, z + 2], top[0], top[1]],
                    [bottom[0], bottom[1], top[1]],
                    [bottom[0], top[1], top[0]],
                ]
            )
        return np.array(faces)

    add("chamfered_lower_plate", plate(-45), dark)
    add("chamfered_top_cover", plate(-7), dark)
    # One shallow electronics bay, one underslung battery level. No compute tower.
    for y in (-51, 51):
        for x in (-80, -40, 0, 40, 80):
            add(f"vent_rib_{x}_{y}", box([x, y, -26], [5, 5, 34]), accent)
        add(f"side_rail_{y}", box([0, y, -39], [182, 5, 8]), dark)
    add(
        "internal_electronics_visual",
        box([0, 0, -30], [145, 80, 12]),
        "0.12 0.30 0.24 1",
    )
    add("battery", box([0, 0, -110], [180, 75, 55]), "0.12 0.25 0.45 1")
    add("battery_tray", box([0, 0, -141], [190, 85, 7]), dark)
    for x in (-75, 75):
        for y in (-43, 43):
            add(f"tray_hanger_{x}_{y}", cylinder([x, y, -91], 3, 93), accent)
        add(f"battery_strap_{x}", box([x, 0, -80], [12, 80, 4]), dark)
        for y in (-39, 39):
            add(f"strap_side_{x}_{y}", box([x, y, -110], [12, 3, 58]), dark)
    add("lidar_mount", cylinder([0, 0, 0], 25, 10), dark)
    add("lidar_housing", cylinder([0, 0, 16], 22, 22), "0.25 0.30 0.34 1")
    add("lidar_optical_band", cylinder([0, 0, 20], 22.5, 7), "0.08 0.32 0.40 1")
    add("camera_housing", box([121, 0, -28], [22, 70, 24]), dark)
    for y in (-23, 0, 23):
        add(
            f"camera_window_{y}",
            cylinder([133, y, -28], 6, 2, axis=0),
            "0.03 0.10 0.14 1",
        )
    add("gnss_patch", box([-77, 0, -1], [28, 28, 8]), "0.75 0.79 0.82 1")
    add("range_housing", box([110, 0, -96], [22, 24, 20]), dark)
    add("range_bracket", box([105, 0, -65], [6, 24, 42]), accent)

    sdf = ET.Element("sdf", version="1.9")
    model = ET.SubElement(sdf, "model", name="akshu_compact")
    ET.SubElement(model, "static").text = "true"
    link = ET.SubElement(model, "link", name="base_link")
    for name, geo, color in parts:
        write_stl(meshes / (name + ".stl"), geo / 1000)
        vis = ET.SubElement(link, "visual", name=name)
        mesh = ET.SubElement(ET.SubElement(vis, "geometry"), "mesh")
        ET.SubElement(mesh, "uri").text = f"model://akshu_compact/meshes/{name}.stl"
        mat = ET.SubElement(vis, "material")
        ET.SubElement(mat, "diffuse").text = color
        ET.SubElement(mat, "ambient").text = color

    def sensor(name, kind, pose, hz):
        element = ET.SubElement(link, "sensor", name=name, type=kind)
        ET.SubElement(element, "pose").text = pose
        ET.SubElement(element, "topic").text = "/icarus/sensors/" + name
        ET.SubElement(element, "always_on").text = "true"
        ET.SubElement(element, "update_rate").text = str(hz)
        return element

    camera = sensor("rgbd", "rgbd_camera", "0.135 0 -0.028 0 0 0", 15)
    camera.append(
        ET.fromstring(
            "<camera><horizontal_fov>1.5184</horizontal_fov><image><width>640</width><height>480</height><format>R8G8B8</format></image><clip><near>0.05</near><far>30</far></clip></camera>"
        )
    )
    lidar = sensor("lidar", "gpu_lidar", "0 0 0.028 0 0 0", 10)
    lidar.append(
        ET.fromstring(
            "<lidar><scan><horizontal><samples>360</samples><resolution>1</resolution><min_angle>-3.14159265</min_angle><max_angle>3.14159265</max_angle></horizontal><vertical><samples>16</samples><resolution>1</resolution><min_angle>-0.2618</min_angle><max_angle>0.5236</max_angle></vertical></scan><range><min>0.1</min><max>40</max><resolution>0.01</resolution></range></lidar>"
        )
    )
    down = sensor("range_down", "gpu_lidar", "0.110 0 -0.108 0 1.57079632679 0", 20)
    down.append(
        ET.fromstring(
            "<lidar><scan><horizontal><samples>1</samples><resolution>1</resolution><min_angle>0</min_angle><max_angle>0</max_angle></horizontal></scan><range><min>0.02</min><max>40</max><resolution>0.001</resolution></range></lidar>"
        )
    )
    sensor("imu", "imu", "0 0 -0.026 0 0 0", 200).append(ET.Element("imu"))
    sensor("navsat", "navsat", "-0.077 0 0.003 0 0 0", 5).append(ET.Element("navsat"))
    ET.indent(sdf)
    ET.ElementTree(sdf).write(TARGET / "model.sdf", encoding="unicode")
    (TARGET / "model.config").write_text(
        '<model><name>Icarus compact two-level chassis</name><version>0.2</version><sdf version="1.9">model.sdf</sdf><description>Simulation-first chassis with native Gazebo sensors; hardware packaging not validated.</description></model>\n'
    )
    write_stl(TARGET / "icarus_compact_mm.stl", np.concatenate([p[1] for p in parts]))
    world = ET.parse(ROOT / "simulation/worlds/mark4_validation.sdf")
    w = world.getroot().find("world")
    w.set("name", "icarus_sensor_validation")
    include = w.find("include")
    include.find("uri").text = "model://akshu_compact"
    include.find("name").text = "icarus_compact"
    include.find("pose").text = "0 0 1 0 0 0"
    for filename, name in [("sensors", "Sensors"), ("navsat", "NavSat")]:
        plugin = ET.SubElement(
            w,
            "plugin",
            filename=f"gz-sim-{filename}-system",
            name=f"gz::sim::systems::{name}",
        )
        if filename == "sensors":
            ET.SubElement(plugin, "render_engine").text = "ogre2"
    obstacle = ET.SubElement(w, "model", name="sensor_target")
    ET.SubElement(obstacle, "static").text = "true"
    ET.SubElement(obstacle, "pose").text = "3 0 1 0 0 0"
    obstacle.append(
        ET.fromstring(
            '<link name="target"><visual name="target"><geometry><box><size>0.2 2 2</size></box></geometry><material><diffuse>0.8 0.2 0.1 1</diffuse></material></visual><collision name="target"><geometry><box><size>0.2 2 2</size></box></geometry></collision></link>'
        )
    )
    for collision in w.findall(".//collision"):
        collision.set("name", collision.get("name") + "_collision")
    ET.indent(world)
    world.write(
        ROOT / "simulation/worlds/akshu_sensor_validation.sdf", encoding="unicode"
    )
    print(TARGET)


if __name__ == "__main__":
    main()

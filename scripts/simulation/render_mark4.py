#!/usr/bin/python3
"""Render actual Gazebo geometry to files, independently of a locked desktop.

This is a static model inspection scene, not a flight-test screenshot.
"""

import argparse
import os
import subprocess
import time
import uuid
import xml.etree.ElementTree as ET
from pathlib import Path

from test_mark4_motors import stop

ROOT = Path(__file__).resolve().parents[2]


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--mesh", type=Path)
    parser.add_argument("--model", default="model://mark4_v2")
    parser.add_argument("--vehicle-height", type=float, default=0.156)
    parser.add_argument("--mesh-scale", type=float, default=0.001)
    parser.add_argument("--mesh-center", type=float, nargs=3, default=[0, 0, 0])
    parser.add_argument("--output", type=Path)
    args = parser.parse_args()
    directory = args.output or ROOT / "logs/simulation" / (
        "mark4_render_" + uuid.uuid4().hex[:8]
    )
    directory.mkdir(parents=True)
    sdf = ET.parse(ROOT / "simulation/worlds/mark4_validation.sdf")
    world = sdf.getroot().find("world")
    world.set("name", "mark4_inspection")
    vehicle = world.find("include")
    vehicle.find("uri").text = args.model
    vehicle.find("pose").text = f"0 0 {args.vehicle_height} 0 0 0"
    ET.SubElement(vehicle, "static").text = "true"
    if args.mesh:
        world.remove(vehicle)
        model = ET.SubElement(world, "model", name="imported_stl")
        ET.SubElement(model, "static").text = "true"
        ET.SubElement(model, "pose").text = (
            " ".join(str(-v * args.mesh_scale) for v in args.mesh_center) + " 0 0 0"
        )
        link = ET.SubElement(model, "link", name="mesh")
        visual = ET.SubElement(link, "visual", name="original_stl")
        mesh = ET.SubElement(ET.SubElement(visual, "geometry"), "mesh")
        ET.SubElement(mesh, "uri").text = str(args.mesh.resolve())
        ET.SubElement(mesh, "scale").text = " ".join([str(args.mesh_scale)] * 3)
        material = ET.SubElement(visual, "material")
        ET.SubElement(material, "diffuse").text = "0.45 0.52 0.56 1"
    plugin = ET.SubElement(
        world,
        "plugin",
        filename="gz-sim-sensors-system",
        name="gz::sim::systems::Sensors",
    )
    ET.SubElement(plugin, "render_engine").text = "ogre2"
    for name, pose in [
        ("detail", "0.72 -0.90 0.62 0 0.36 2.246"),
        ("overhead", "0.01 0 1.25 0 1.56 0"),
        ("front", f"1.15 0 {args.vehicle_height + 0.02} 0 0 3.14159265"),
        ("side", f"0 -1.15 {args.vehicle_height + 0.02} 0 0 1.57079633"),
    ]:
        model = ET.SubElement(world, "model", name="inspection_" + name)
        ET.SubElement(model, "static").text = "true"
        ET.SubElement(model, "pose").text = pose
        link = ET.SubElement(model, "link", name="camera_mount")
        sensor = ET.SubElement(link, "sensor", name=name, type="camera")
        ET.SubElement(sensor, "topic").text = "/inspection/" + name
        ET.SubElement(sensor, "always_on").text = "true"
        ET.SubElement(sensor, "update_rate").text = "1"
        camera = ET.SubElement(sensor, "camera")
        ET.SubElement(camera, "horizontal_fov").text = "0.80"
        img = ET.SubElement(camera, "image")
        ET.SubElement(img, "width").text = "1600"
        ET.SubElement(img, "height").text = "1000"
        clip = ET.SubElement(camera, "clip")
        ET.SubElement(clip, "near").text = "0.02"
        ET.SubElement(clip, "far").text = "100"
        save = ET.SubElement(camera, "save", enabled="true")
        ET.SubElement(save, "path").text = str(directory / name)
    world_path = directory / "inspection.sdf"
    sdf.write(world_path, encoding="unicode")
    env = os.environ.copy()
    env["GZ_PARTITION"] = "icarus_render_" + uuid.uuid4().hex
    os.environ["GZ_PARTITION"] = env["GZ_PARTITION"]
    from gz.msgs10.image_pb2 import Image
    from gz.transport13 import Node

    node = Node()
    # Gazebo's system scheduler activates rendering sensors with subscribers.
    for name in ("detail", "overhead", "front", "side"):
        node.subscribe(Image, "/inspection/" + name, lambda _msg: None)
    env["GZ_SIM_RESOURCE_PATH"] = str(ROOT / "simulation/models")
    # Select the installed NVIDIA EGL vendor for offscreen rendering on this
    # hybrid workstation. This is local process configuration, not a driver change.
    nvidia = Path("/usr/share/glvnd/egl_vendor.d/10_nvidia.json")
    if nvidia.exists():
        env["__EGL_VENDOR_LIBRARY_FILENAMES"] = str(nvidia)
    with (directory / "gazebo.log").open("w") as log:
        process = subprocess.Popen(
            [
                "gz",
                "sim",
                "-s",
                "-r",
                "--headless-rendering",
                "-v",
                "3",
                str(world_path),
            ],
            env=env,
            stdout=log,
            stderr=subprocess.STDOUT,
            start_new_session=True,
        )
        try:
            deadline = time.monotonic() + 50
            while not all(
                list((directory / name).glob("*.png"))
                for name in ("detail", "overhead", "front", "side")
            ):
                if process.poll() is not None or time.monotonic() > deadline:
                    raise RuntimeError(
                        "Render failed; inspect " + str(directory / "gazebo.log")
                    )
                time.sleep(0.25)
            print("Rendered model inspection:", directory, flush=True)
        finally:
            stop(process)


if __name__ == "__main__":
    main()

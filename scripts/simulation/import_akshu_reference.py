#!/usr/bin/python3
"""Extract a reversible Gazebo reference from the specific user-provided STL.

No mesh repair, shape deformation, mass assignment or flight-model replacement.
Component IDs are checked against the original SHA-256 before extraction.
"""

import argparse
import hashlib
import json
import shutil
import struct
import xml.etree.ElementTree as ET
from pathlib import Path

import numpy as np
from inspect_stl import inspect

ROOT = Path(__file__).resolve().parents[2]
EXPECTED = "64f89254d2b68226b695534b8ea12a29d7ab912551ee680c2a2bf11897f152aa"


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("source", type=Path)
    source = parser.parse_args().source
    target = ROOT / "simulation/models/akshu_reference"
    raw = source.read_bytes()
    if hashlib.sha256(raw).hexdigest() != EXPECTED:
        raise ValueError(
            "Source STL changed; inspect and update the extraction recipe first"
        )
    report = inspect(source, with_faces=True)
    comps = report["components"]
    dtype = np.dtype([("normal", "<f4", (3,)), ("v", "<f4", (3, 3)), ("attr", "<u2")])
    triangles = np.frombuffer(raw, dtype=dtype, offset=84)["v"].astype(float)
    # Millimetres inferred from ~253 mm propellers and ~37 mm motor housings.
    # Source +Y is the camera-facing end; map it to Gazebo body +X.
    cx = np.mean([comps[i]["bounds_center"][0] for i in range(4)])
    cy = np.mean([comps[i]["bounds_center"][1] for i in range(4)])
    origin = np.array([cx, cy, comps[25]["min"][2]])
    rotation = np.array([[0, 1, 0], [-1, 0, 0], [0, 0, 1]])
    transformed = (triangles - origin) @ rotation.T * 0.001
    target.mkdir(parents=True, exist_ok=True)
    meshes = target / "meshes"
    meshes.mkdir(exist_ok=True)
    original = meshes / "akshu_original.stl"
    if (
        original.exists()
        and hashlib.sha256(original.read_bytes()).hexdigest() != EXPECTED
    ):
        raise ValueError("Existing source copy differs; refusing to overwrite it")
    if not original.exists():
        shutil.copyfile(source, original)

    def export(name, indices, offset=None):
        values = transformed[indices].copy()
        if offset is not None:
            values -= offset
        output = np.zeros(len(indices), dtype=dtype)
        output["v"] = values
        normals = np.cross(values[:, 1] - values[:, 0], values[:, 2] - values[:, 0])
        lengths = np.linalg.norm(normals, axis=1)
        normals /= lengths[:, None]
        output["normal"] = normals
        (meshes / (name + ".stl")).write_bytes(
            b"Icarus reference extraction".ljust(80, b" ")
            + struct.pack("<I", len(output))
            + output.tobytes()
        )

    # Each motor housing and propeller was identified from bounds and renders.
    # The paired index order is ArduPilot motor order, with no mirror operation.
    rotors = [(2, 28), (1, 26), (0, 25), (3, 27)]
    prop_ids = {25, 26, 27, 28}
    body_indices = np.concatenate(
        [c["face_indices"] for i, c in enumerate(comps) if i not in prop_ids]
    )
    export("body", body_indices)
    sdf = ET.Element("sdf", version="1.9")
    model = ET.SubElement(sdf, "model", name="icarus_akshu_reference")
    ET.SubElement(model, "static").text = "true"

    def visual(link, name, color):
        v = ET.SubElement(link, "visual", name=name)
        mesh = ET.SubElement(ET.SubElement(v, "geometry"), "mesh")
        ET.SubElement(mesh, "uri").text = (
            "model://akshu_reference/meshes/" + name + ".stl"
        )
        ET.SubElement(ET.SubElement(v, "material"), "diffuse").text = color

    base = ET.SubElement(model, "link", name="base_link")
    visual(base, "body", "0.45 0.52 0.56 1")
    motor_layout = []
    for index, (motor_id, prop_id) in enumerate(rotors, 1):
        motor_center = np.array(comps[motor_id]["bounds_center"])
        motor_center[2] = origin[2]
        pose = (motor_center - origin) @ rotation.T * 0.001
        name = f"rotor_{index:02d}"
        export(name, comps[prop_id]["face_indices"], pose)
        link = ET.SubElement(model, "link", name=f"motor_{index:02d}")
        ET.SubElement(link, "pose").text = " ".join(map(str, pose)) + " 0 0 0"
        visual(link, name, "0.2 0.55 0.7 1" if index <= 2 else "0.8 0.35 0.13 1")
        joint = ET.SubElement(
            model, "joint", name=f"rotor_{index:02d}_joint", type="revolute"
        )
        ET.SubElement(joint, "parent").text = "base_link"
        ET.SubElement(joint, "child").text = f"motor_{index:02d}"
        ET.SubElement(ET.SubElement(joint, "axis"), "xyz").text = "0 0 1"
        motor_layout.append(
            {
                "motor": index,
                "position_m": pose.tolist(),
                "source_motor_component": motor_id,
                "source_prop_component": prop_id,
            }
        )
    ET.indent(sdf)
    ET.ElementTree(sdf).write(target / "model.sdf", encoding="unicode")
    config = ET.fromstring(
        '<model><name>Icarus Akshu reference</name><version>0.1</version><sdf version="1.9">model.sdf</sdf><author><name>Icarus project</name></author><description>User STL reference; static, no validated flight dynamics.</description></model>'
    )
    ET.indent(config)
    ET.ElementTree(config).write(target / "model.config", encoding="unicode")
    extraction = {
        "source_sha256": EXPECTED,
        "units_assumption": "millimetres",
        "source_origin_units": origin.tolist(),
        "source_to_flu_rotation": rotation.tolist(),
        "motor_layout": motor_layout,
        "triangles_input": len(triangles),
        "triangles_exported": len(body_indices)
        + sum(comps[p]["triangles"] for _, p in rotors),
        "status": "Static reference only; masses, rotor handedness and physical interfaces not validated.",
    }
    (target / "extraction.json").write_text(json.dumps(extraction, indent=2) + "\n")
    assert extraction["triangles_input"] == extraction["triangles_exported"]
    print(json.dumps(extraction, indent=2))


if __name__ == "__main__":
    main()

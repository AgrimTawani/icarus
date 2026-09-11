#!/usr/bin/python3
"""Reproducible packaging CAD, not fabrication CAD or a validated flight model.

Retains only source arms, leg feet and prop visuals. All electronics, motor
proxies, plates, mounts and battery geometry are rebuilt. Coordinates are FLU mm.
"""

import hashlib
import json
import struct
import xml.etree.ElementTree as ET

import numpy as np
from import_akshu_reference import EXPECTED, ROOT
from inspect_stl import inspect

TARGET = ROOT / "simulation/models/akshu_icarus_v1"
DTYPE = np.dtype([("normal", "<f4", (3,)), ("v", "<f4", (3, 3)), ("attr", "<u2")])
PARTS = []


def box(center, size):
    vertices = np.array([[x, y, z] for z in (-1, 1) for y in (-1, 1) for x in (-1, 1)])
    vertices = vertices * np.array(size) / 2 + center
    faces = [
        [0, 2, 3],
        [0, 3, 1],
        [4, 5, 7],
        [4, 7, 6],
        [0, 1, 5],
        [0, 5, 4],
        [2, 6, 7],
        [2, 7, 3],
        [0, 4, 6],
        [0, 6, 2],
        [1, 3, 7],
        [1, 7, 5],
    ]
    return vertices[faces]


def cylinder(center, radius, height, axis=2, inner=0):
    faces = []
    for i in range(48):
        a, b = np.array([i, i + 1]) * 2 * np.pi / 48
        points = np.array(
            [
                [r * np.cos(t), r * np.sin(t), z]
                for z in (-height / 2, height / 2)
                for r in (inner, radius)
                for t in (a, b)
            ]
        )
        for face in (
            [2, 3, 7],
            [2, 7, 6],
            [0, 2, 6],
            [0, 6, 4],
            [1, 5, 7],
            [1, 7, 3],
            [4, 6, 7],
            [4, 7, 5],
            [0, 1, 3],
            [0, 3, 2],
            [0, 4, 5],
            [0, 5, 1],
        ):
            tri = points[list(face)].copy()
            if axis != 2:
                tri[:, [axis, 2]] = tri[:, [2, axis]]
                tri = tri[[0, 2, 1]]
            if np.linalg.norm(np.cross(tri[1] - tri[0], tri[2] - tri[0])) > 1e-8:
                faces.append(tri + center)
    return np.array(faces)


def add(name, geometry, color, description):
    PARTS.append(
        {
            "name": name,
            "geometry": np.asarray(geometry),
            "color": color,
            "description": description,
        }
    )


def write_stl(path, triangles):
    values = np.zeros(len(triangles), dtype=DTYPE)
    values["v"] = triangles
    normals = np.cross(
        triangles[:, 1] - triangles[:, 0], triangles[:, 2] - triangles[:, 0]
    )
    lengths = np.linalg.norm(normals, axis=1)
    assert np.all(lengths > 1e-12), "Degenerate triangles"
    values["normal"] = normals / lengths[:, None]
    path.write_bytes(
        b"ICARUS packaging candidate; NOT fabrication-ready".ljust(80, b" ")
        + struct.pack("<I", len(values))
        + values.tobytes()
    )


def main():
    source = ROOT / "simulation/models/akshu_reference/meshes/akshu_original.stl"
    raw = source.read_bytes()
    assert hashlib.sha256(raw).hexdigest() == EXPECTED
    components = inspect(source, with_faces=True)["components"]
    extraction = json.loads((source.parents[1] / "extraction.json").read_text())
    triangles = np.frombuffer(raw, dtype=DTYPE, offset=84)["v"].astype(float)
    triangles = (triangles - extraction["source_origin_units"]) @ np.array(
        extraction["source_to_flu_rotation"]
    ).T
    dark, blue, orange = "0.08 0.10 0.13 1", "0.12 0.48 0.65 1", "0.95 0.40 0.09 1"
    retained = [44, 45, 46, 47, 79, 80, 81, 82, 25, 26, 27, 28]
    for index in retained:
        mesh = triangles[components[index]["face_indices"]].copy()
        name = (
            f"source_arm_{index}"
            if index < 49 and index > 28
            else f"source_prop_{index}"
        )
        if index >= 79:
            name = f"extended_leg_{index}"
            upper = mesh.reshape(-1, 3)
            anchor = upper[upper[:, 2] > upper[:, 2].max() - 1].mean(axis=0)
            mesh[:, :, 2] -= 35
            add(
                f"leg_extension_{index}",
                cylinder(anchor - [0, 0, 17.5], 7, 35),
                dark,
                "35 mm extension concept; source leg joint requires mechanical detailing",
            )
        add(
            name,
            mesh,
            blue if index in (25, 26) else orange if index in (27, 28) else dark,
            "Source structural geometry"
            if index > 28
            else "Source 10-inch prop visual; not verified HQ geometry or handedness",
        )
    # Replace every original motor and shaft with dimensioned envelope proxies.
    for motor in extraction["motor_layout"]:
        x, y, _ = np.array(motor["position_m"]) * 1000
        n = motor["motor"]
        add(
            f"motor_{n}",
            cylinder([x, y, -23], 21, 36),
            dark,
            "42 x 36 mm reserved motor envelope; XRotor3115 mounting drawing still required",
        )
        add(
            f"motor_collar_{n}",
            cylinder([x, y, -8], 21.5, 4),
            orange,
            "Motor visual collar",
        )
        add(f"shaft_{n}", cylinder([x, y, 4], 2.5, 22), dark, "Prop shaft proxy")
    add(
        "lower_plate",
        box([0, 0, -45], [200, 110, 5]),
        dark,
        "New widened 200 x 110 x 5 mm centre plate",
    )
    add(
        "compute_shelf",
        box([0, 0, 37], [150, 110, 5]),
        dark,
        "New elevated compute shelf; 20.5 mm above source blade envelope",
    )
    for x in (-64, 64):
        for y in (-44, 44):
            add(
                f"deck_post_{x}_{y}",
                cylinder([x, y, -2.75], 4, 74.5, inner=1.7),
                orange,
                "Hollow M3-clearance standoff concept; plate drilling pending",
            )
    envelopes = [
        (
            "thor_assembly",
            [0, 0, 67.5],
            [140, 100, 51],
            "0.24 0.28 0.31 1",
            "Reserved module + custom carrier + cooling volume including fins; NOT developer kit CAD",
        ),
        (
            "pixhawk_6x",
            [0, 0, 0],
            [90, 60, 32],
            "0.15 0.18 0.22 1",
            "Conservative FC + baseboard allocation; verify chosen baseboard",
        ),
        (
            "esc",
            [0, 0, -59],
            [55, 55, 12],
            "0.17 0.42 0.25 1",
            "65 A 4-in-1 ESC reserved enclosure and connector allowance",
        ),
        (
            "battery_6s_10ah",
            [0, 0, -110],
            [180, 75, 55],
            "0.12 0.25 0.48 1",
            "Provisional 6S 10 Ah pack envelope; select actual pack before fabrication",
        ),
        (
            "mid360",
            [0, 0, 133],
            [65, 65, 60],
            "0.80 0.83 0.85 1",
            "Livox MID-360 nominal envelope, visual proxy, not active sensor",
        ),
        (
            "oak_camera",
            [144, 0, -40],
            [30, 100, 32],
            "0.25 0.28 0.30 1",
            "Conservative OAK-D Lite-class enclosure allocation; forward +X",
        ),
        (
            "lw20",
            [120, 0, -108],
            [30, 20, 43],
            "0.12 0.18 0.21 1",
            "LW20/C nominal envelope; optical axis downward",
        ),
        (
            "gnss",
            [-107, 0, 69],
            [45, 45, 18],
            "0.90 0.91 0.88 1",
            "Provisional GNSS antenna envelope below lidar scan level",
        ),
    ]
    for name, center, size, color, description in envelopes:
        geometry = (
            box([0, 0, 64.5], [140, 100, 45])
            if name == "thor_assembly"
            else box(center, size)
        )
        add(name, geometry, color, description)
    # Cooling fins are inside the allocated compute envelope, not extra volume.
    for y in range(-44, 45, 8):
        add(
            f"cooling_fin_{y}",
            box([0, y, 90], [128, 2, 6]),
            dark,
            "Illustrative cooling; thermal validation required",
        )
    add(
        "lidar_pedestal",
        box([0, 0, 98], [70, 70, 10]),
        dark,
        "Raised lidar mount; cooling interaction needs validation",
    )
    for x in (-60, 60):
        for y in (-40, 40):
            add(
                f"compute_pad_{x}_{y}",
                cylinder([x, y, 40.75], 5, 2.5),
                orange,
                "Compute mounting spacer",
            )
    add("esc_mount", box([0, 0, -50.25], [40, 40, 5.5]), dark, "ESC mounting spacer")
    add(
        "fc_isolation_pad",
        box([0, 0, -20], [70, 50, 8]),
        orange,
        "Isolated FC carrier concept",
    )
    add("fc_support", box([0, 0, -33.25], [75, 55, 18.5]), dark, "FC support")
    add(
        "battery_tray", box([0, 0, -141], [190, 85, 7]), dark, "Underslung battery tray"
    )
    for x in (-75, 75):
        for y in (-43, 43):
            add(
                f"tray_post_{x}_{y}",
                cylinder([x, y, -90], 3, 90),
                orange,
                "Battery tray hanger outside pack",
            )
        for y in (-39, 39):
            add(
                f"battery_strap_{x}_{y}",
                box([x, y, -110], [14, 3, 55]),
                dark,
                "Battery restraint sides",
            )
        add(
            f"strap_top_{x}",
            box([x, 0, -81], [14, 81, 3]),
            dark,
            "Battery restraint top",
        )
    add(
        "camera_bracket",
        box([115, 0, -52], [38, 38, 5]),
        orange,
        "Forward camera shelf",
    )
    for y in (-35, 0, 35):
        add(
            f"camera_lens_{y}",
            cylinder([160, y, -40], 7, 3, axis=0),
            "0.03 0.06 0.10 1",
            "Forward optical window proxy",
        )
    add(
        "range_bracket",
        box([110, 0, -72], [40, 30, 5]),
        orange,
        "Rangefinder mounting shelf",
    )
    add("range_post", box([98, 0, -60], [5, 30, 24]), dark, "Range bracket support")
    add(
        "range_spacer",
        box([120, 0, -79.25], [25, 18, 9.5]),
        dark,
        "Rangefinder mount spacer",
    )
    add("range_optic", cylinder([120, 0, -130], 6, 2), blue, "Downward optical window")
    add(
        "gnss_mast",
        cylinder([-107, 0, 3], 4, 114),
        dark,
        "Rear GNSS mast; cable routing pending",
    )
    add(
        "gnss_foot",
        box([-100, 0, -43], [24, 28, 5]),
        orange,
        "GNSS mast attachment concept",
    )
    meshes = TARGET / "meshes"
    meshes.mkdir(parents=True, exist_ok=True)
    sdf = ET.Element("sdf", version="1.9")
    model = ET.SubElement(sdf, "model", name="akshu_icarus_v1")
    ET.SubElement(model, "static").text = "true"
    link = ET.SubElement(model, "link", name="packaging_candidate")
    manifest = []
    for part in PARTS:
        write_stl(meshes / (part["name"] + ".stl"), part["geometry"] / 1000)
        visual = ET.SubElement(link, "visual", name=part["name"])
        mesh = ET.SubElement(ET.SubElement(visual, "geometry"), "mesh")
        ET.SubElement(
            mesh, "uri"
        ).text = f"model://akshu_icarus_v1/meshes/{part['name']}.stl"
        mat = ET.SubElement(visual, "material")
        ET.SubElement(mat, "diffuse").text = part["color"]
        ET.SubElement(mat, "ambient").text = part["color"]
        manifest.append(
            {k: v for k, v in part.items() if k != "geometry"}
            | {
                "min_mm": part["geometry"].min(axis=(0, 1)).tolist(),
                "max_mm": part["geometry"].max(axis=(0, 1)).tolist(),
            }
        )
    ET.indent(sdf)
    ET.ElementTree(sdf).write(TARGET / "model.sdf", encoding="unicode")
    (TARGET / "model.config").write_text(
        '<model><name>Icarus custom Akshu packaging v1</name><version>0.1</version><sdf version="1.9">model.sdf</sdf><description>Static packaging study; no validated flight dynamics or fabrication interfaces.</description></model>\n'
    )
    assembly = np.concatenate([p["geometry"] for p in PARTS])
    write_stl(TARGET / "icarus_custom_assembly_mm.stl", assembly)
    # Check reserved payload envelopes independently; mounting contacts intentional.
    conflicts = []
    for i, (name, c, s, _, _) in enumerate(envelopes):
        for other, oc, osize, _, _ in envelopes[i + 1 :]:
            if np.all(np.abs(np.array(c) - oc) < (np.array(s) + osize) / 2):
                conflicts.append([name, other])
    rotor_conflicts = []
    for name, c, s, _, _ in envelopes:
        lo, hi = np.array(c) - np.array(s) / 2, np.array(c) + np.array(s) / 2
        for m in extraction["motor_layout"]:
            xy = np.array(m["position_m"][:2]) * 1000
            distance = np.linalg.norm(xy - np.clip(xy, lo[:2], hi[:2]))
            if distance < 127 and lo[2] < 14 and hi[2] > 0:
                rotor_conflicts.append([name, m["motor"]])
    assert not conflicts and not rotor_conflicts, (conflicts, rotor_conflicts)
    report = {
        "status": "STATIC PACKAGING CANDIDATE ONLY",
        "source_sha256": EXPECTED,
        "retained_source_components": retained,
        "removed_source_component_count": len(components) - len(retained),
        "units": "mm in assembly STL; metres in Gazebo meshes",
        "parts": manifest,
        "checks": {
            "payload_AABB_overlaps": conflicts,
            "payload_rotor_swept_volume_intersections": rotor_conflicts,
            "tray_ground_clearance_mm": float(-144.5 - assembly[:, :, 2].min()),
            "blade_to_compute_shelf_vertical_gap_mm": 20.5,
        },
        "limitations": [
            "Not a boolean-unioned printable assembly",
            "Fastener holes, wiring, cooling, load paths and vibration isolation need detailed CAD",
            "Motor mount compatibility and extended landing-gear attachments unverified",
            "Clearance checks cover reserved payload boxes, not every structural triangle or flexible blade",
            "No new mass, inertia, thrust or flight validation; original SITL model unchanged",
        ],
    }
    (TARGET / "packaging.json").write_text(json.dumps(report, indent=2) + "\n")
    print(json.dumps(report["checks"], indent=2))
    print(TARGET / "icarus_custom_assembly_mm.stl")


if __name__ == "__main__":
    main()

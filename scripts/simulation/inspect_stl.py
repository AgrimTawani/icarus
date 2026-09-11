#!/usr/bin/python3
"""Inspect binary STL geometry without assuming units or solid material density."""

import argparse
import hashlib
import json
import struct
from pathlib import Path

import numpy as np


def inspect(path, with_faces=False):
    raw = path.read_bytes()
    count = struct.unpack_from("<I", raw, 80)[0]
    if len(raw) != 84 + count * 50:
        raise ValueError("Expected a complete binary STL")
    data = np.frombuffer(
        raw,
        dtype=np.dtype(
            [("normal", "<f4", (3,)), ("v", "<f4", (3, 3)), ("attr", "<u2")]
        ),
        offset=84,
    )
    triangles = data["v"].astype(np.float64)
    if not np.isfinite(triangles).all():
        raise ValueError("Nonfinite vertex coordinates")
    vertices, indices = np.unique(triangles.reshape(-1, 3), axis=0, return_inverse=True)
    faces = indices.reshape(-1, 3)
    parent = np.arange(len(vertices))

    def root(i):
        while parent[i] != i:
            parent[i] = parent[parent[i]]
            i = parent[i]
        return i

    for a, b, c in faces:
        a, b, c = root(a), root(b), root(c)
        parent[b] = a
        parent[c] = a
    roots = np.array([root(i) for i in range(len(vertices))])
    face_root = roots[faces[:, 0]]
    edges = np.sort(
        np.concatenate([faces[:, [0, 1]], faces[:, [1, 2]], faces[:, [2, 0]]]), axis=1
    )
    _, edge_counts = np.unique(edges, axis=0, return_counts=True)
    areas = (
        np.linalg.norm(
            np.cross(
                triangles[:, 1] - triangles[:, 0], triangles[:, 2] - triangles[:, 0]
            ),
            axis=1,
        )
        / 2
    )
    components = []
    for key in np.unique(face_root):
        mask = face_root == key
        tri = triangles[mask]
        points = tri.reshape(-1, 3)
        lo, hi = points.min(axis=0), points.max(axis=0)
        shifted = tri - points.mean(axis=0)
        volume = (
            np.einsum(
                "ij,ij->i", shifted[:, 0], np.cross(shifted[:, 1], shifted[:, 2])
            ).sum()
            / 6
        )
        components.append(
            {
                "triangles": int(mask.sum()),
                "min": lo.tolist(),
                "max": hi.tolist(),
                "size": (hi - lo).tolist(),
                "bounds_center": ((hi + lo) / 2).tolist(),
                "surface_area_units2": float(areas[mask].sum()),
                "signed_volume_units3": float(volume),
                **({"face_indices": np.flatnonzero(mask)} if with_faces else {}),
            }
        )
    components.sort(key=lambda c: c["triangles"], reverse=True)
    lo, hi = vertices.min(axis=0), vertices.max(axis=0)
    return {
        "source": str(path.resolve()),
        "sha256": hashlib.sha256(raw).hexdigest(),
        "format": "binary STL",
        "units": "unspecified",
        "triangles": count,
        "unique_vertices": len(vertices),
        "min": lo.tolist(),
        "max": hi.tolist(),
        "size": (hi - lo).tolist(),
        "boundary_edges": int(np.count_nonzero(edge_counts == 1)),
        "nonmanifold_edges": int(np.count_nonzero(edge_counts > 2)),
        "degenerate_triangles": int(np.count_nonzero(areas < 1e-10)),
        "connected_components": len(components),
        "components": components,
        "notes": "Exact-coordinate welding. Component bounds/volumes do not identify materials or CAD parts; enclosed volumes may overlap.",
    }


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("stl", type=Path)
    parser.add_argument("--output", required=True, type=Path)
    args = parser.parse_args()
    result = inspect(args.stl)
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(result, indent=2) + "\n")
    print(json.dumps({k: v for k, v in result.items() if k != "components"}, indent=2))
    for i, c in enumerate(result["components"][:35]):
        print(
            i,
            c["triangles"],
            "size",
            np.round(c["size"], 3),
            "center",
            np.round(c["bounds_center"], 3),
        )

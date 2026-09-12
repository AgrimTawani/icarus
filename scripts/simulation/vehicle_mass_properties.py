"""Calculate rigid-body mass properties from the component source of truth."""

import hashlib
import json
import math
from pathlib import Path

import numpy as np


def component_inertia(component):
    mass = float(component["mass_kg"])
    dimensions = [float(value) for value in component["dimensions_m"]]
    if component["shape"] == "box":
        x, y, z = dimensions
        return np.diag(
            [
                mass * (y * y + z * z) / 12,
                mass * (x * x + z * z) / 12,
                mass * (x * x + y * y) / 12,
            ]
        )
    if component["shape"] == "cylinder":
        radius, length = dimensions
        radial = mass * (3 * radius * radius + length * length) / 12
        return np.diag([radial, radial, mass * radius * radius / 2])
    raise ValueError("unsupported component shape: " + str(component["shape"]))


def calculate(components):
    if not components:
        raise ValueError("at least one component is required")
    ids = [component["id"] for component in components]
    if len(ids) != len(set(ids)):
        raise ValueError("component identifiers must be unique")
    for component in components:
        mass = float(component["mass_kg"])
        center = component["center_m"]
        if not math.isfinite(mass) or mass <= 0 or len(center) != 3:
            raise ValueError("invalid mass or center for " + component["id"])
        if not all(math.isfinite(float(value)) for value in center):
            raise ValueError("invalid center for " + component["id"])
        component_inertia(component)
    mass = sum(float(component["mass_kg"]) for component in components)
    cg = sum(
        float(component["mass_kg"]) * np.array(component["center_m"], dtype=float)
        for component in components
    ) / mass
    inertia = np.zeros((3, 3))
    for component in components:
        item_mass = float(component["mass_kg"])
        inertia += component_inertia(component)
        offset = np.array(component["center_m"], dtype=float) - cg
        inertia += item_mass * (
            np.dot(offset, offset) * np.eye(3) - np.outer(offset, offset)
        )
    if np.linalg.eigvalsh(inertia).min() <= 0:
        raise ValueError("calculated inertia tensor is not positive definite")
    return mass, cg, inertia


def load_manifest(path):
    path = Path(path)
    raw = path.read_bytes()
    manifest = json.loads(raw)
    if manifest.get("version") != 1 or not isinstance(manifest.get("components"), list):
        raise ValueError("unsupported vehicle component manifest")
    return manifest, hashlib.sha256(raw).hexdigest()

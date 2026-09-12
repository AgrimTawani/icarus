import json
import sys
import xml.etree.ElementTree as ET
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / "scripts/simulation"))

from vehicle_mass_properties import calculate, load_manifest


def test_component_manifest_generates_valid_full_aircraft_properties():
    manifest, digest = load_manifest(
        ROOT / "config/simulation/vehicle_components.json"
    )
    mass, cg, inertia = calculate(manifest["components"])
    assert mass == 4.343
    assert len(digest) == 64
    assert abs(cg[0]) < 0.005 and abs(cg[1]) < 0.005 and cg[2] < 0
    assert np.linalg.eigvalsh(inertia).min() > 0


def test_generated_sdf_and_report_match_component_source():
    report = json.loads(
        (ROOT / "simulation/models/akshu_compact_sitl/mass_properties.json").read_text()
    )
    tree = ET.parse(ROOT / "simulation/models/akshu_compact_sitl/model.sdf")
    model = tree.getroot().find("model")
    base_mass = float(model.find("link[@name='base_link']/inertial/mass").text)
    rotor_mass = sum(
        float(model.find(f"link[@name='motor_{number:02d}']/inertial/mass").text)
        for number in range(1, 5)
    )
    assert abs(base_mass + rotor_mass - report["total_mass_kg"]) < 1e-12
    assert report["component_manifest"] == "config/simulation/vehicle_components.json"
    assert len(report["components"]) == 20


def test_duplicate_component_identifier_is_rejected():
    item = {
        "id": "duplicate",
        "link": "base_link",
        "mass_kg": 1,
        "center_m": [0, 0, 0],
        "shape": "box",
        "dimensions_m": [1, 1, 1],
    }
    try:
        calculate([item, dict(item)])
    except ValueError as error:
        assert "unique" in str(error)
    else:
        raise AssertionError("duplicate component identifiers were accepted")

"""Hardware sensor-profile invariants that do not require a running Gazebo."""

import math
import sys
import xml.etree.ElementTree as ET
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / "scripts/simulation"))

from sensor_profiles import (
    METRES_PER_LATITUDE_DEGREE,
    configure,
    navsat_horizontal_noise_degrees,
)


def test_navsat_horizontal_noise_compensates_for_gazebo_degree_bug():
    model = ET.fromstring(
        "<model><link name='base_link'><sensor name='gps' type='navsat'>"
        "<navsat/></sensor></link></model>"
    )
    world = ET.fromstring("<world/>")
    config = configure(model, world, "physical")
    navsat = model.find("link/sensor/navsat")
    horizontal = float(
        navsat.find("position_sensing/horizontal/noise/stddev").text
    )
    vertical = float(navsat.find("position_sensing/vertical/noise/stddev").text)

    intended_m = config["stddev"]["gps_position"]
    assert math.isclose(
        horizontal,
        intended_m / METRES_PER_LATITUDE_DEGREE,
        rel_tol=1e-15,
    )
    assert vertical == intended_m
    assert math.isclose(
        navsat_horizontal_noise_degrees(0.5),
        0.5 / METRES_PER_LATITUDE_DEGREE,
        rel_tol=1e-15,
    )

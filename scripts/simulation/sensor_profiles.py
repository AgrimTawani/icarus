"""Native Gazebo sensor configuration, shared by isolated and flight tests."""

import json
import xml.etree.ElementTree as ET
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]


def configure(model, world, profile="nominal"):
    config = json.loads((ROOT / "config/simulation/sensor_profiles.json").read_text())
    scale = config["profiles"][profile]
    base = model.find("link[@name='base_link']")
    for name, kind, pose, hz, body in [
        ("compass", "magnetometer", "-0.077 0 0.003 0 0 0", 50, "<magnetometer/>"),
        (
            "barometer",
            "air_pressure",
            "0 0 -0.026 0 0 0",
            30,
            "<air_pressure><reference_altitude>584</reference_altitude></air_pressure>",
        ),
    ]:
        sensor = ET.SubElement(base, "sensor", name=name, type=kind)
        ET.SubElement(sensor, "pose").text = pose
        ET.SubElement(sensor, "topic").text = "/icarus/sensors/" + name
        ET.SubElement(sensor, "always_on").text = "true"
        ET.SubElement(sensor, "update_rate").text = str(hz)
        sensor.append(ET.fromstring(body))

    def noise(parent, path, key):
        for segment in path.split("/"):
            found = parent.find(segment)
            if found is None:
                found = ET.SubElement(parent, segment)
            parent = found
        node = ET.SubElement(parent, "noise")
        if path in ("camera", "lidar"):
            ET.SubElement(node, "type").text = "gaussian"
        else:
            node.set("type", "gaussian")
        ET.SubElement(node, "mean").text = str(scale * config["mean"][key])
        ET.SubElement(node, "stddev").text = str(scale * config["stddev"][key])

    for sensor in base.findall("sensor"):
        kind = sensor.get("type")
        if kind == "imu":
            for axis in "xyz":
                noise(sensor, "imu/linear_acceleration/" + axis, "accelerometer")
                noise(sensor, "imu/angular_velocity/" + axis, "gyro")
        elif kind == "navsat":
            for direction in ("horizontal", "vertical"):
                noise(sensor, "navsat/position_sensing/" + direction, "gps_position")
                noise(sensor, "navsat/velocity_sensing/" + direction, "gps_velocity")
        elif kind == "magnetometer":
            for axis in "xyz":
                noise(sensor, "magnetometer/" + axis, "magnetometer")
        elif kind == "air_pressure":
            noise(sensor, "air_pressure/pressure", "pressure")
        elif kind == "gpu_lidar":
            noise(
                sensor,
                "lidar",
                "range_down" if sensor.get("name") == "range_down" else "lidar",
            )
        elif kind == "rgbd_camera":
            noise(sensor, "camera", "camera")
    for filename, name in [
        ("magnetometer", "Magnetometer"),
        ("air-pressure", "AirPressure"),
    ]:
        plugin = ET.SubElement(
            world,
            "plugin",
            filename=f"gz-sim-{filename}-system",
            name=f"gz::sim::systems::{name}",
        )
        if filename == "magnetometer":
            ET.SubElement(plugin, "use_units_gauss").text = "false"
            ET.SubElement(plugin, "use_earth_frame_ned").text = "false"
    ET.SubElement(world, "magnetic_field").text = "2.15e-5 0 -4.27e-5"
    battery = ET.SubElement(
        model,
        "plugin",
        filename="gz-sim-linearbatteryplugin-system",
        name="gz::sim::systems::LinearBatteryPlugin",
    )
    b = config["battery"]
    values = {
        "battery_name": "flight_battery",
        "voltage": b["voltage"],
        "open_circuit_voltage_constant_coef": b["voltage"],
        "open_circuit_voltage_linear_coef": b["voltage_slope"],
        "initial_charge": b["initial_charge_ah"],
        "capacity": b["capacity_ah"],
        "resistance": b["resistance_ohm"],
        "smooth_current_tau": 1,
        "power_load": b["load_w"],
        "fix_issue_225": "true",
        "start_on_motion": "true",
        "power_draining_topic": "/icarus/battery/start",
        "enable_recharge": "false",
    }
    for key, value in values.items():
        ET.SubElement(battery, key).text = str(value)
    return config

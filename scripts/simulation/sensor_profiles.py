"""Native Gazebo sensor configuration, shared by isolated and flight tests."""

import json
import xml.etree.ElementTree as ET
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]

# gz-sensors currently applies horizontal NavSat position noise directly to
# latitude / longitude expressed in degrees, even though SDFormat specifies
# metres.  Keep the public configuration in SI units and compensate at the
# generated-SDF boundary.  One latitude degree is sufficient here because
# Gazebo exposes a single horizontal noise value for both axes.
METRES_PER_LATITUDE_DEGREE = 111_320.0


def navsat_horizontal_noise_degrees(value_m):
    """Translate an SDFormat metre intent for the current Gazebo implementation."""
    return float(value_m) / METRES_PER_LATITUDE_DEGREE


def configure(model, world, profile="physical"):
    config = json.loads((ROOT / "config/simulation/sensor_profiles.json").read_text())
    scale = config["profiles"][profile]
    base = model.find("link[@name='base_link']")
    for name, kind, pose, hz, body in [
        ("h_rtk_compass", "magnetometer", "-0.091 -0.068 0.044 0 0 0", 50, "<magnetometer/>"),
        # The external H-RTK RM3100 is the normal heading source, but the
        # Pixhawk 6X Pro also contains a BMM150.  Publish both independently
        # so fault and source-selection tests do not pretend one compass is
        # two pieces of hardware.
        ("pixhawk_magnetometer", "magnetometer", "0 0 0.03345 0 0 0", 50, "<magnetometer/>"),
        (
            "pixhawk_barometer",
            "air_pressure",
            "0 0 0.03345 0 0 0",
            30,
            "<air_pressure><reference_altitude>584</reference_altitude></air_pressure>",
        ),
        # Gazebo's AirPressure plugin does not model the electrical
        # differences between the ICP20100 and BMP388.  Separate topics still
        # preserve the Pixhawk's redundant measurement topology.
        (
            "pixhawk_barometer_secondary",
            "air_pressure",
            "0 0 0.03345 0 0 0",
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

    def noise(parent, path, key, value_scale=1.0):
        for segment in path.split("/"):
            found = parent.find(segment)
            if found is None:
                found = ET.SubElement(parent, segment)
            parent = found
        node = parent.find("noise")
        if node is None:
            node = ET.SubElement(parent, "noise")
        else:
            node.clear()
        if path in ("camera", "lidar"):
            ET.SubElement(node, "type").text = "gaussian"
        else:
            node.set("type", "gaussian")
        ET.SubElement(node, "mean").text = str(
            value_scale * scale * config["mean"][key]
        )
        ET.SubElement(node, "stddev").text = str(
            value_scale * scale * config["stddev"][key]
        )

    for sensor in base.findall("sensor"):
        kind = sensor.get("type")
        if kind == "imu":
            for axis in "xyz":
                noise(sensor, "imu/linear_acceleration/" + axis, "accelerometer")
                noise(sensor, "imu/angular_velocity/" + axis, "gyro")
        elif kind == "navsat":
            noise(
                sensor,
                "navsat/position_sensing/horizontal",
                "gps_position",
                value_scale=1.0 / METRES_PER_LATITUDE_DEGREE,
            )
            noise(sensor, "navsat/position_sensing/vertical", "gps_position")
            for direction in ("horizontal", "vertical"):
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
                "range_down" if sensor.get("name") == "ark_flow_range" else "lidar",
            )
        elif kind in ("rgbd_camera", "camera"):
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
    autopilot = model.find("plugin[@name='ArduPilotPlugin']")
    if autopilot is not None:
        topic = autopilot.find("battery_state_topic")
        if topic is None:
            topic = ET.SubElement(autopilot, "battery_state_topic")
        topic.text = f"/model/{model.get('name')}/battery/flight_battery/state"
    b = config["battery"]
    try:
        from battery_profile import configure_motor_load
    except ImportError:
        from scripts.simulation.battery_profile import configure_motor_load
    dynamic_load = configure_motor_load(model, b)
    values = {
        "battery_name": "flight_battery",
        "voltage": b["voltage"],
        "open_circuit_voltage_constant_coef": b["voltage"],
        "open_circuit_voltage_linear_coef": b["voltage_slope"],
        "initial_charge": b["initial_charge_ah"],
        "capacity": b["capacity_ah"],
        "resistance": b["resistance_ohm"],
        "smooth_current_tau": 1,
        "power_load": 0 if dynamic_load else b["load_w"],
        "start_draining": "true" if dynamic_load else "false",
        "fix_issue_225": "true",
        "start_on_motion": "true",
        "power_draining_topic": "/icarus/battery/start",
        "enable_recharge": "false",
    }
    for key, value in values.items():
        ET.SubElement(battery, key).text = str(value)
    return config

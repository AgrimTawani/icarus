#!/usr/bin/python3
"""Bounded live Gazebo sensor smoke test; owns and cleans its process group."""

import json
import math
import os
import subprocess
import time
import uuid
import xml.etree.ElementTree as ET

from build_akshu_candidate import ROOT
from sensor_profiles import configure
from test_mark4_motors import stop


def main():
    os.environ["GZ_PARTITION"] = "icarus_sensor_test_" + uuid.uuid4().hex
    os.environ["GZ_IP"] = "127.0.0.1"
    os.environ["GZ_SIM_RESOURCE_PATH"] = str(ROOT / "simulation/models")
    vendor = "/usr/share/glvnd/egl_vendor.d/10_nvidia.json"
    if os.path.exists(vendor):
        os.environ["__EGL_VENDOR_LIBRARY_FILENAMES"] = vendor
    from gz.msgs10.fluid_pressure_pb2 import FluidPressure
    from gz.msgs10.image_pb2 import Image
    from gz.msgs10.imu_pb2 import IMU
    from gz.msgs10.laserscan_pb2 import LaserScan
    from gz.msgs10.magnetometer_pb2 import Magnetometer
    from gz.msgs10.navsat_pb2 import NavSat
    from gz.transport13 import Node

    received = {}
    counts = {}
    node = Node()

    def callback(name):
        def receive(message):
            received[name] = message
            counts[name] = counts.get(name, 0) + 1

        return receive

    specifications = [
        ("d435i/image", Image),
        ("d435i/depth_image", Image),
        ("brio_down", Image),
        ("landing_depth_proxy/depth_image", Image),
        ("mid360", LaserScan),
        ("ark_flow_range", LaserScan),
        ("pixhawk_imu_primary", IMU),
        ("pixhawk_imu_secondary", IMU),
        ("pixhawk_imu_tertiary", IMU),
        ("d435i_imu", IMU),
        ("mid360_imu", IMU),
        ("ark_flow_imu", IMU),
        ("h_rtk_gnss", NavSat),
        ("h_rtk_compass", Magnetometer),
        ("pixhawk_magnetometer", Magnetometer),
        ("pixhawk_barometer", FluidPressure),
        ("pixhawk_barometer_secondary", FluidPressure),
    ]
    for name, kind in specifications:
        assert node.subscribe(kind, "/icarus/sensors/" + name, callback(name))
    output = ROOT / "logs/simulation" / ("compact_sensors_" + uuid.uuid4().hex[:8])
    output.mkdir(parents=True)
    # Exercise the same physical profile that is embedded into the dynamic
    # flight model.  The checked-in isolated world includes only the sensors
    # intrinsic to the visual model; redundant Pixhawk compass/barometer
    # channels are profile-generated and must not be skipped by this smoke.
    world = ET.parse(ROOT / "simulation/worlds/akshu_sensor_validation.sdf")
    world_node = world.getroot().find("world")
    include = world_node.find("include")
    include_index = list(world_node).index(include)
    world_node.remove(include)
    model = ET.parse(
        ROOT / "simulation/models/akshu_compact/model.sdf"
    ).getroot().find("model")
    model.set("name", "icarus_compact")
    ET.SubElement(model, "pose").text = "0 0 1 0 0 0"
    world_node.insert(include_index, model)
    configure(model, world_node, "physical")
    ET.indent(world)
    generated_world = output / "world.sdf"
    world.write(generated_world, encoding="unicode")
    with (output / "gazebo.log").open("w") as log:
        process = subprocess.Popen(
            [
                "gz",
                "sim",
                "-s",
                "-r",
                "--headless-rendering",
                "-v",
                "3",
                str(generated_world),
            ],
            stdout=log,
            stderr=subprocess.STDOUT,
            start_new_session=True,
        )
        try:
            deadline = time.monotonic() + 50
            while len(received) < len(specifications) or min(counts.values(), default=0) < 3:
                if time.monotonic() > deadline or process.poll() is not None:
                    raise RuntimeError(f"Sensor timeout: {counts}; see {output}")
                time.sleep(0.1)
            rgb, depth = received["d435i/image"], received["d435i/depth_image"]
            assert (rgb.width, rgb.height) == (848, 480) and len(rgb.data) > 0
            assert (depth.width, depth.height) == (848, 480) and len(depth.data) > 0
            brio = received["brio_down"]
            assert (brio.width, brio.height) == (1280, 720) and len(brio.data) > 0
            downward_depth = received["landing_depth_proxy/depth_image"]
            assert ((downward_depth.width, downward_depth.height) == (320, 180)
                    and len(downward_depth.data) > 0)
            ranges = received["ark_flow_range"].ranges
            finite_range = [value for value in ranges if math.isfinite(value)]
            assert len(ranges) == 32 and finite_range
            assert all(0.86 < value < 0.92 for value in finite_range), finite_range
            lidar = received["mid360"]
            finite = [r for r in lidar.ranges if math.isfinite(r)]
            assert len(lidar.ranges) == 500 * 40 and finite
            # Target front face is 2.9 m from lidar; one near-horizontal ray hits.
            assert any(abs(r - 2.9) < 0.08 for r in finite)
            gps = received["h_rtk_gnss"]
            assert abs(gps.latitude_deg + 35.363262) < 0.001, (
                gps.latitude_deg,
                gps.longitude_deg,
                gps.altitude,
            )
            acceleration = received["pixhawk_imu_primary"].linear_acceleration.z
            assert abs(acceleration - 9.80665) < 0.1, acceleration
            for name in ("pixhawk_imu_secondary", "pixhawk_imu_tertiary",
                         "d435i_imu", "mid360_imu", "ark_flow_imu"):
                acceleration_vector = received[name].linear_acceleration
                acceleration_norm = math.sqrt(
                    acceleration_vector.x**2 + acceleration_vector.y**2 +
                    acceleration_vector.z**2)
                # The downward ARK frame is rotated relative to body FLU; the
                # gravity magnitude, not a particular local axis, is invariant.
                assert abs(acceleration_norm - 9.80665) < 0.1, (
                    name, acceleration_vector)
            for name in ("h_rtk_compass", "pixhawk_magnetometer"):
                field = received[name].field_tesla
                assert all(math.isfinite(value) for value in (field.x, field.y, field.z))
            for name in ("pixhawk_barometer", "pixhawk_barometer_secondary"):
                assert 90000 < received[name].pressure < 102000
            report = {
                "status": "PASS",
                "counts": counts,
                "rgb_size": [rgb.width, rgb.height],
                "lidar_rays": len(lidar.ranges),
                "downward_range_m": sum(finite_range) / len(finite_range),
                "imu_z_m_s2": acceleration,
                "latitude_deg": gps.latitude_deg,
                "scope": "Static sensor data and known targets; not SITL flight validation",
            }
            (output / "results.json").write_text(json.dumps(report, indent=2) + "\n")
            print(json.dumps(report, indent=2), flush=True)
            print(output, flush=True)
        finally:
            stop(process)


if __name__ == "__main__":
    main()

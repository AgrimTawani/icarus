#!/usr/bin/python3
"""Bounded live Gazebo sensor smoke test; owns and cleans its process group."""

import json
import math
import os
import subprocess
import time
import uuid

from build_akshu_candidate import ROOT
from test_mark4_motors import stop


def main():
    os.environ["GZ_PARTITION"] = "icarus_sensor_test_" + uuid.uuid4().hex
    os.environ["GZ_SIM_RESOURCE_PATH"] = str(ROOT / "simulation/models")
    vendor = "/usr/share/glvnd/egl_vendor.d/10_nvidia.json"
    if os.path.exists(vendor):
        os.environ["__EGL_VENDOR_LIBRARY_FILENAMES"] = vendor
    from gz.msgs10.image_pb2 import Image
    from gz.msgs10.imu_pb2 import IMU
    from gz.msgs10.laserscan_pb2 import LaserScan
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

    for name, kind in [
        ("rgbd/image", Image),
        ("rgbd/depth_image", Image),
        ("lidar", LaserScan),
        ("range_down", LaserScan),
        ("imu", IMU),
        ("navsat", NavSat),
    ]:
        assert node.subscribe(kind, "/icarus/sensors/" + name, callback(name))
    output = ROOT / "logs/simulation" / ("compact_sensors_" + uuid.uuid4().hex[:8])
    output.mkdir(parents=True)
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
                str(ROOT / "simulation/worlds/akshu_sensor_validation.sdf"),
            ],
            stdout=log,
            stderr=subprocess.STDOUT,
            start_new_session=True,
        )
        try:
            deadline = time.monotonic() + 50
            while len(received) < 6 or min(counts.values(), default=0) < 3:
                if time.monotonic() > deadline or process.poll() is not None:
                    raise RuntimeError(f"Sensor timeout: {counts}; see {output}")
                time.sleep(0.1)
            rgb, depth = received["rgbd/image"], received["rgbd/depth_image"]
            assert (rgb.width, rgb.height) == (640, 480) and len(rgb.data) > 0
            assert (depth.width, depth.height) == (640, 480) and len(depth.data) > 0
            ranges = received["range_down"].ranges
            assert len(ranges) == 1 and 0.86 < ranges[0] < 0.91, list(ranges)
            lidar = received["lidar"]
            finite = [r for r in lidar.ranges if math.isfinite(r)]
            assert len(lidar.ranges) == 360 * 16 and finite
            # Target front face is 2.9 m from lidar; one near-horizontal ray hits.
            assert any(abs(r - 2.9) < 0.08 for r in finite)
            gps = received["navsat"]
            assert abs(gps.latitude_deg + 35.363262) < 0.001
            acceleration = received["imu"].linear_acceleration.z
            assert abs(acceleration - 9.80665) < 0.1, acceleration
            report = {
                "status": "PASS",
                "counts": counts,
                "rgb_size": [rgb.width, rgb.height],
                "lidar_rays": len(lidar.ranges),
                "downward_range_m": ranges[0],
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

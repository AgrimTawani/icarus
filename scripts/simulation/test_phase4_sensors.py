#!/usr/bin/python3
"""Live physical-noise tests with numeric-only output and channel failures."""

import argparse
import json
import math
import os
import subprocess
import threading
import time
import uuid
import xml.etree.ElementTree as ET
from pathlib import Path

import numpy as np
from sensor_health import SensorChannel
from sensor_profiles import ROOT, configure
from test_mark4_motors import stop


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--profile", choices=("physical",), default="physical")
    parser.add_argument("--only", help="Test one channel independently")
    parser.add_argument("--duration", type=float, default=10)
    parser.add_argument("--live-faults", action="store_true")
    args = parser.parse_args()
    if args.duration < 6:
        raise ValueError("At least six seconds required for rate/noise statistics")
    os.environ["GZ_PARTITION"] = "icarus_phase4_" + uuid.uuid4().hex
    os.environ["GZ_SIM_RESOURCE_PATH"] = str(ROOT / "simulation/models")
    vendor = Path("/usr/share/glvnd/egl_vendor.d/10_nvidia.json")
    if vendor.exists():
        os.environ["__EGL_VENDOR_LIBRARY_FILENAMES"] = str(vendor)
    from gz.msgs10.battery_state_pb2 import BatteryState
    from gz.msgs10.boolean_pb2 import Boolean
    from gz.msgs10.fluid_pressure_pb2 import FluidPressure
    from gz.msgs10.image_pb2 import Image
    from gz.msgs10.imu_pb2 import IMU
    from gz.msgs10.laserscan_pb2 import LaserScan
    from gz.msgs10.magnetometer_pb2 import Magnetometer
    from gz.msgs10.navsat_pb2 import NavSat
    from gz.msgs10.stringmsg_pb2 import StringMsg
    from gz.transport13 import Node

    specs = {
        "rgb": ("rgbd/image", Image, 15),
        "depth": ("rgbd/depth_image", Image, 15),
        "imu": ("imu", IMU, 200),
        "gps": ("navsat", NavSat, 5),
        "compass": ("compass", Magnetometer, 50),
        "barometer": ("barometer", FluidPressure, 30),
        "lidar": ("lidar", LaserScan, 10),
        "range": ("range_down", LaserScan, 20),
        "battery": (
            "/model/icarus_compact/battery/flight_battery/state",
            BatteryState,
            None,
        ),
    }
    if args.only:
        specs = {args.only: specs[args.only]}
    directory = (
        ROOT
        / "logs/simulation"
        / ("phase4_" + args.profile + "_" + uuid.uuid4().hex[:8])
    )
    directory.mkdir(parents=True)
    world = ET.parse(ROOT / "simulation/worlds/akshu_sensor_validation.sdf")
    w = world.getroot().find("world")
    w.set("name", "phase4_sensors")
    w.remove(w.find("include"))
    model = (
        ET.parse(ROOT / "simulation/models/akshu_compact/model.sdf")
        .getroot()
        .find("model")
    )
    model.set("name", "icarus_compact")
    ET.SubElement(model, "pose").text = "0 0 1 0 0 0"
    w.append(model)
    config = configure(model, w, args.profile)
    ET.indent(world)
    world.write(directory / "world.sdf", encoding="unicode")
    node = Node()
    samples = {key: [] for key in specs}
    lock = threading.Lock()

    def callback(name):
        def receive(msg):
            stamp = msg.header.stamp.sec + msg.header.stamp.nsec * 1e-9
            if name == "rgb":
                # Numeric intensity statistics only. Pixels are never retained.
                value = float(np.frombuffer(msg.data, dtype=np.uint8).mean())
            elif name == "depth":
                data = np.frombuffer(msg.data, dtype=np.float32)
                value = float(data[(msg.height // 2) * msg.width + msg.width // 2])
            elif name == "imu":
                value = msg.linear_acceleration.z
            elif name == "gps":
                value = msg.altitude
            elif name == "compass":
                value = math.sqrt(
                    msg.field_tesla.x**2 + msg.field_tesla.y**2 + msg.field_tesla.z**2
                )
            elif name == "barometer":
                value = msg.pressure
            elif name == "battery":
                # Gazebo fix_issue_225 reports percentage on a 0..100 scale.
                value = msg.charge / msg.capacity
            else:
                value = msg.ranges[5 * 360 + 179] if name == "lidar" else msg.ranges[0]
            with lock:
                samples[name].append((stamp, time.monotonic(), value))

        return receive

    callbacks = []
    for name, (topic, kind, _) in specs.items():
        cb = callback(name)
        callbacks.append(cb)
        assert node.subscribe(
            kind, topic if topic.startswith("/") else "/icarus/sensors/" + topic, cb
        )
    drain = node.advertise("/icarus/battery/start", Boolean)
    fault_publisher = node.advertise("/icarus/test/sensor_fault", StringMsg)
    live_results = []
    with (directory / "gazebo.log").open("w") as log:
        process = subprocess.Popen(
            [
                "gz",
                "sim",
                "-s",
                "-r",
                "--headless-rendering",
                "--seed",
                "42",
                "-v",
                "2",
                str(directory / "world.sdf"),
            ],
            stdout=log,
            stderr=subprocess.STDOUT,
            start_new_session=True,
        )
        recorder = None
        recorder_log = None
        try:
            if args.live_faults:
                recorder_log = (directory / "recorder.log").open("w")
                recorder = subprocess.Popen(["/usr/bin/python3",str(ROOT / "scripts/simulation/record_compact_sensors.py"),"--directory",str(directory)],stdout=recorder_log,stderr=subprocess.STDOUT,start_new_session=True)
            deadline = time.monotonic() + 40
            while not all(len(v) >= 3 for v in samples.values()):
                assert time.monotonic() < deadline and process.poll() is None, {
                    k: len(v) for k, v in samples.items()
                }
                time.sleep(0.1)
            drain.publish(Boolean(data=True))
            began = time.monotonic()
            while time.monotonic() - began < args.duration:
                assert process.poll() is None
                drain.publish(Boolean(data=True))
                time.sleep(0.1)
            if args.live_faults:
                def health():
                    path = directory / "sensors/health.json"
                    return json.loads(path.read_text()) if path.exists() else {}
                assert health().get("status") == "ready", health()
                for name in specs:
                    for mode in ("drop", "freeze"):
                        started = time.monotonic()
                        while name not in health().get("stale", []):
                            fault_publisher.publish(StringMsg(data=json.dumps({"channel":name,"mode":mode})))
                            assert time.monotonic()-started < 5, (name,mode,health())
                            time.sleep(.05)
                        h = health()
                        assert h["stale"] == [name], h
                        detection = time.monotonic()-started
                        started = time.monotonic()
                        while health().get("status") != "ready":
                            fault_publisher.publish(StringMsg(data=json.dumps({"channel":name,"mode":"normal"})))
                            assert time.monotonic()-started < 5, (name,"recovery",health())
                            time.sleep(.05)
                        live_results.append({"channel":name,"fault":mode,"stale_detected_s":detection,"recovered":True})
        finally:
            if recorder is not None:
                stop(recorder)
                recorder_log.close()
            stop(process)
    report = {"profile": args.profile, "media_saved": False, "channels": {}, "live_faults":live_results}
    (directory / "numeric_samples.json").write_text(json.dumps(samples) + "\n")
    for name, rows in samples.items():
        data = np.array(rows)
        data = data[data[:, 0] > data[0, 0] + 2]
        assert len(data) >= 15, (name, len(data))
        rate = (len(data) - 1) / (data[-1, 0] - data[0, 0])
        expected = specs[name][2]
        if expected:
            assert 0.85 * expected < rate < 1.15 * expected, (name, rate)
        assert np.all(np.isfinite(data[:, 2])), name
        nominal = {
            "range": 0.884,
            "lidar": 2.9,
            "compass": 5.8e-5,
            "imu": 9.80665,
            "depth": 2.765,
        }
        tolerance = {
            "range": 0.03,
            "lidar": 0.08,
            "compass": 2e-5,
            "imu": 0.15,
            "depth": 0.10,
        }
        if name in nominal:
            assert abs(data[:, 2].mean() - nominal[name]) < tolerance[name], (
                name,
                data[:, 2].mean(),
            )
        if name == "battery":
            assert data[-1, 2] < data[0, 2] and 0 < data[-1, 2] <= 1, (
                name,
                data[0, 2],
                data[-1, 2],
            )
        if name == "barometer":
            assert 90000 < data[:, 2].mean() < 102000
        if name in (
            "imu",
            "gps",
            "compass",
            "barometer",
            "lidar",
            "range",
            "rgb",
            "depth",
        ):
            assert data[:, 2].std() > 1e-10, (name, "noise not observed")
            keys = {"imu":"accelerometer", "gps":"gps_position", "compass":"magnetometer", "barometer":"pressure", "lidar":"lidar", "range":"range_down", "depth":"camera"}
            if name in keys:
                expected_std = config["stddev"][keys[name]]
                assert .5*expected_std < data[:,2].std() < 2*expected_std, (name,"noise magnitude",data[:,2].std())
        # Same actual received numeric observations through the consumer boundary.
        # Controlled clock avoids wall sleeps; drop/freeze/delay are independent.
        fault_results = {}
        for mode in ("drop", "freeze"):
            channel = SensorChannel(stale_s=config["health"]["stale_wall_s"])
            channel.receive(1, float(data[0, 2]), 0)
            channel.deliver(0)
            channel.mode = mode
            for i in range(1, 31):
                channel.receive(i + 1, float(data[i % len(data), 2]), i * 0.1)
                channel.deliver(i * 0.1)
            assert not channel.healthy(3), (name, mode)
            channel.mode = "normal"
            channel.receive(100, float(data[-1, 2]), 3.1)
            channel.deliver(3.1)
            assert channel.healthy(3.1)
            fault_results[mode] = "stale then recovered"
        channel = SensorChannel(latency_s=config["health"]["injected_latency_s"])
        channel.receive(1, float(data[0, 2]), 0)
        channel.deliver(0.19)
        assert not channel.healthy(0.19)
        channel.deliver(0.2)
        assert channel.healthy(0.2) and abs(channel.latencies[0] - 0.2) < 1e-8
        report["channels"][name] = {
            "rate_hz": rate,
            "mean": float(data[:, 2].mean()),
            "stddev": float(data[:, 2].std()),
            "samples": len(data),
            "first": float(data[0, 2]),
            "last": float(data[-1, 2]),
            "faults": fault_results,
            "injected_latency_s": channel.latencies[0],
            "wall_to_sim_rate": float(
                (data[-1, 0] - data[0, 0]) / (data[-1, 1] - data[0, 1])
            ),
        }
    report["status"] = "passed"
    (directory / "results.json").write_text(json.dumps(report, indent=2) + "\n")
    print(json.dumps(report, indent=2))
    print(directory)


if __name__ == "__main__":
    main()

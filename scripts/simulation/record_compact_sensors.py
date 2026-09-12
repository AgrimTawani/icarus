#!/usr/bin/python3
"""Bounded, timestamped Gazebo protobuf recording plus readiness/health status."""

import argparse
import json
import signal
import struct
import threading
import time
import zlib
from pathlib import Path

from sensor_health import SensorChannel


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--directory", required=True, type=Path)
    parser.add_argument("--include-wind", action="store_true")
    args = parser.parse_args()
    from gz.msgs10.battery_state_pb2 import BatteryState
    from gz.msgs10.fluid_pressure_pb2 import FluidPressure
    from gz.msgs10.image_pb2 import Image
    from gz.msgs10.imu_pb2 import IMU
    from gz.msgs10.laserscan_pb2 import LaserScan
    from gz.msgs10.magnetometer_pb2 import Magnetometer
    from gz.msgs10.navsat_pb2 import NavSat
    from gz.msgs10.stringmsg_pb2 import StringMsg
    from gz.msgs10.vector3d_pb2 import Vector3d
    from gz.transport13 import Node

    output = args.directory / "sensors"
    output.mkdir()
    specs = [
        ("rgb", "/icarus/sensors/rgbd/image", Image, 1),
        ("depth", "/icarus/sensors/rgbd/depth_image", Image, 1),
        ("lidar", "/icarus/sensors/lidar", LaserScan, 5),
        ("range", "/icarus/sensors/range_down", LaserScan, 10),
        ("imu", "/icarus/sensors/imu", IMU, 20),
        ("gps", "/icarus/sensors/navsat", NavSat, 5),
        ("compass", "/icarus/sensors/compass", Magnetometer, 20),
        ("barometer", "/icarus/sensors/barometer", FluidPressure, 10),
        (
            "battery",
            "/model/icarus_compact/battery/flight_battery/state",
            BatteryState,
            5,
        ),
    ]
    if args.include_wind:
        specs.append(("wind", "/icarus/environment/wind", Vector3d, 10))
    files = {
        name: (output / (name + ".pbstream")).open("wb") for name, _, _, _ in specs
    }
    index = (output / "index.jsonl").open("w")
    lock = threading.Lock()
    state = {
        "status": "starting",
        "received": {},
        "recorded": {},
        "last_wall": {},
        "bytes": 0,
    }
    last_stamp = {}
    stopping = threading.Event()

    def stop(_sig, _frame):
        stopping.set()

    signal.signal(signal.SIGINT, stop)
    signal.signal(signal.SIGTERM, stop)
    node = Node()
    health_publisher = node.advertise("/icarus/health/sensors", StringMsg)
    config = json.loads(
        (
            Path(__file__).resolve().parents[2]
            / "config/simulation/sensor_profiles.json"
        ).read_text()
    )
    channels = {
        name: SensorChannel(stale_s=config["health"]["stale_wall_s"])
        for name, _, _, _ in specs
    }
    def inject_fault(message):
        try:
            request = json.loads(message.data)
            name, mode = request["channel"], request["mode"]
            if name not in channels or mode not in ("normal", "drop", "freeze", "delay"):
                return
            latency = float(
                request.get("latency_s", config["health"]["injected_latency_s"])
            )
            if latency < 0 or latency > 5:
                return
            with lock:
                channels[name].mode = "normal" if mode == "delay" else mode
                channels[name].latency_s = latency if mode == "delay" else 0
        except (KeyError, ValueError, TypeError):
            return
    assert node.subscribe(StringMsg, "/icarus/test/sensor_fault", inject_fault)
    callbacks = []

    def callback(name, hz):
        def receive(message):
            with lock:
                if stopping.is_set():
                    return
                state["received"][name] = state["received"].get(name, 0) + 1
                state["last_wall"][name] = time.monotonic()
                stamp = message.header.stamp.sec + message.header.stamp.nsec * 1e-9
                channels[name].receive(stamp, None, time.monotonic())
                channels[name].deliver(time.monotonic())
                if stamp - last_stamp.get(name, -1e10) < 1 / hz - 1e-6:
                    return
                # Keep image metadata only. Never write camera pixels to disk.
                if name in ("rgb", "depth"):
                    metadata = Image()
                    metadata.CopyFrom(message)
                    metadata.ClearField("data")
                    data = zlib.compress(metadata.SerializeToString(), level=1)
                else:
                    data = zlib.compress(message.SerializeToString(), level=1)
                if state["bytes"] + len(data) + 4 > 512 * 1024 * 1024:
                    state["error"] = "Recording exceeded 512 MiB cap"
                    stopping.set()
                    return
                try:
                    stream = files[name]
                    offset = stream.tell()
                    stream.write(struct.pack("<I", len(data)) + data)
                    index.write(
                        json.dumps(
                            {
                                "channel": name,
                                "simulation_s": stamp,
                                "wall_monotonic_s": time.monotonic(),
                                "offset": offset,
                                "compressed_bytes": len(data),
                            }
                        )
                        + "\n"
                    )
                    state["recorded"][name] = state["recorded"].get(name, 0) + 1
                    state["bytes"] += len(data) + 4
                    last_stamp[name] = stamp
                except OSError as error:
                    state["error"] = str(error)
                    stopping.set()

        return receive

    (output / "schema.json").write_text(
        json.dumps(
            {
                "compression": "zlib",
                "image_pixels_saved": False,
                "format": "Repeated uint32 little-endian compressed length followed by zlib-compressed protobuf; index offsets point to length prefix",
                "channels": {
                    n: {
                        "topic": t,
                        "protobuf_type": k.DESCRIPTOR.full_name,
                        "record_hz": h,
                    }
                    for n, t, k, h in specs
                },
            },
            indent=2,
        )
        + "\n"
    )
    for name, topic, kind, hz in specs:
        cb = callback(name, hz)
        callbacks.append(cb)
        assert node.subscribe(kind, topic, cb)

    def publish(final=False):
        with lock:
            now = time.monotonic()
            state["stale"] = [n for n, _, _, _ in specs if not channels[n].healthy(now)]
            state["status"] = (
                "failed"
                if "error" in state
                else "stopped"
                if final
                else "ready"
                if len(state["received"]) == len(specs)
                and min(state["received"].values()) >= 3
                and not state["stale"]
                else "starting"
            )
            state["updated_monotonic_s"] = now
            temp = output / "health.tmp"
            temp.write_text(json.dumps(state, indent=2) + "\n")
            temp.replace(output / "health.json")
            health_publisher.publish(StringMsg(data=json.dumps(state)))
            index.flush()

    try:
        while not stopping.wait(0.25):
            publish()
    finally:
        stopping.set()
        publish(final=True)
        for stream in files.values():
            stream.close()
        index.close()
    return 1 if "error" in state else 0


if __name__ == "__main__":
    raise SystemExit(main())

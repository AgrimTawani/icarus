#!/usr/bin/python3
"""Verify telemetry only; never generate images or video."""

import argparse
import json
import struct
import zlib
from pathlib import Path


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("directory", type=Path)
    args = parser.parse_args()
    from gz.msgs10.battery_state_pb2 import BatteryState
    from gz.msgs10.fluid_pressure_pb2 import FluidPressure
    from gz.msgs10.image_pb2 import Image
    from gz.msgs10.imu_pb2 import IMU
    from gz.msgs10.laserscan_pb2 import LaserScan
    from gz.msgs10.magnetometer_pb2 import Magnetometer
    from gz.msgs10.navsat_pb2 import NavSat
    from gz.msgs10.vector3d_pb2 import Vector3d

    types = {
        t.DESCRIPTOR.full_name: t
        for t in (
            Image,
            IMU,
            LaserScan,
            NavSat,
            Magnetometer,
            FluidPressure,
            BatteryState,
            Vector3d,
        )
    }
    folder = args.directory / "sensors"
    schema = json.loads((folder / "schema.json").read_text())
    health = json.loads((folder / "health.json").read_text())
    results = {}
    for name, spec in schema["channels"].items():
        count, first, last, previous = 0, None, None, -1
        with (folder / (name + ".pbstream")).open("rb") as stream:
            while prefix := stream.read(4):
                assert len(prefix) == 4, "Truncated length prefix"
                size = struct.unpack("<I", prefix)[0]
                data = stream.read(size)
                assert len(data) == size, "Truncated message"
                if schema.get("compression") == "zlib":
                    data = zlib.decompress(data)
                msg = types[spec["protobuf_type"]]()
                msg.ParseFromString(data)
                stamp = msg.header.stamp.sec + msg.header.stamp.nsec * 1e-9
                assert stamp > previous, f"Non-increasing timestamps in {name}"
                previous = stamp
                if first is None:
                    first = stamp
                last = stamp
                if (
                    name in ("rgb", "depth")
                    and schema.get("image_pixels_saved") is False
                ):
                    assert not msg.data and msg.width == 640 and msg.height == 480
                count += 1
        assert count == health["recorded"][name] and count >= 3
        results[name] = {
            "messages": count,
            "first_simulation_s": first,
            "last_simulation_s": last,
            "average_record_hz": (count - 1) / (last - first),
        }
    (args.directory / "recording_verified.json").write_text(
        json.dumps({"status": "passed", "channels": results}, indent=2) + "\n"
    )
    print(json.dumps(results, indent=2))


if __name__ == "__main__":
    main()

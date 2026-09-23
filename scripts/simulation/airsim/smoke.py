#!/usr/bin/env python3
"""Read-only AirSim RPC and camera smoke test; does not arm or move a vehicle."""

import argparse
import math

from python.simulation.airsim_rpc import MsgpackRpcClient


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--host", default="127.0.0.1")
    parser.add_argument("--port", type=int, default=41451)
    parser.add_argument("--timeout", type=float, default=10)
    parser.add_argument("--vehicle", default="Icarus")
    parser.add_argument("--lidar", default="Lidar1")
    args = parser.parse_args()
    rpc = MsgpackRpcClient(args.host, args.port, args.timeout)
    try:
        if not rpc.ping():
            raise SystemExit("AirSim ping returned false")
        vehicles = rpc.list_vehicles()
        state = rpc.multirotor_state(args.vehicle)
        lidar = rpc.lidar(args.lidar, args.vehicle)
        # Image type 0 is Scene and 1 is DepthPlanar. Both are read-only.
        requests = [
            {"camera_name": "0", "image_type": 0,
             "pixels_as_float": False, "compress": True},
            {"camera_name": "0", "image_type": 1,
             "pixels_as_float": True, "compress": False},
        ]
        images = rpc.images(requests, args.vehicle)
    finally:
        rpc.close()
    if args.vehicle not in vehicles:
        raise SystemExit(f"configured ArduCopter vehicle {args.vehicle!r} is absent: {vehicles}")
    if type(state.get("timestamp")) is not int or state["timestamp"] <= 0:
        raise SystemExit("AirSim returned no timestamped multirotor state")
    if len(images) != 2 or any(image.get("time_stamp", 0) <= 0 for image in images):
        raise SystemExit("AirSim did not return timestamped RGB and depth responses")
    if images[0].get("image_type") != 0 or not images[0].get("image_data_uint8"):
        raise SystemExit("AirSim RGB response is empty")
    depth = images[1].get("image_data_float") or []
    if images[1].get("image_type") != 1 or not any(math.isfinite(x) for x in depth):
        raise SystemExit("AirSim depth response is empty or non-finite")
    points = lidar.get("point_cloud") or []
    if lidar.get("time_stamp", 0) <= 0 or len(points) < 3 or len(points) % 3:
        raise SystemExit("AirSim LiDAR response is empty or malformed")
    print(f"RPC ping: ok; ArduCopter: {args.vehicle}; "
          f"RGB: {images[0]['width']}x{images[0]['height']}; "
          f"depth: {images[1]['width']}x{images[1]['height']}; "
          f"LiDAR points: {len(points) // 3}; timestamps: ok")


if __name__ == "__main__":
    main()

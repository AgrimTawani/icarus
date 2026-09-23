#!/usr/bin/env python3
"""Bridge read-only AirSim LiDAR samples to the Icarus perception adapter.

The packet is intentionally sensor-only. This process has no AirSim control
methods and cannot arm or move a vehicle; flight control remains exclusively
Drone API -> MAVLink -> ArduPilot.
"""

import argparse
import math
import socket
import struct
import time

from python.simulation.airsim_rpc import MsgpackRpcClient

MAGIC = b"ICAR"
VERSION = 1
MAX_POINTS = 4096
HEADER = struct.Struct("<4sHHqQ")
POINT = struct.Struct("<fff")


def encode_packet(point_cloud, captured_at_unix_ms, sequence):
    triples = []
    for index in range(0, len(point_cloud) - 2, 3):
        point = point_cloud[index:index + 3]
        if all(type(value) in (int, float) and math.isfinite(value)
               for value in point):
            triples.append(point)
        if len(triples) == MAX_POINTS:
            break
    header = HEADER.pack(MAGIC, VERSION, len(triples),
                         captured_at_unix_ms, sequence)
    return header + b"".join(POINT.pack(*point) for point in triples)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--rpc-host", default="127.0.0.1")
    parser.add_argument("--rpc-port", type=int, default=41451)
    parser.add_argument("--vehicle", default="Icarus")
    parser.add_argument("--lidar", default="Lidar1")
    parser.add_argument("--target-host", default="127.0.0.1")
    parser.add_argument("--target-port", type=int, default=41452)
    parser.add_argument("--rate-hz", type=float, default=10.0)
    args = parser.parse_args()
    if not 1 <= args.rate_hz <= 100:
        parser.error("--rate-hz must be in [1, 100]")
    rpc = MsgpackRpcClient(args.rpc_host, args.rpc_port, timeout=5)
    output = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    interval = 1.0 / args.rate_hz
    sequence = 0
    try:
        while True:
            started = time.monotonic()
            lidar = rpc.lidar(args.lidar, args.vehicle)
            sequence += 1
            packet = encode_packet(
                lidar.get("point_cloud") or [], int(time.time() * 1000), sequence)
            output.sendto(packet, (args.target_host, args.target_port))
            time.sleep(max(0.0, interval - (time.monotonic() - started)))
    finally:
        output.close()
        rpc.close()


if __name__ == "__main__":
    main()

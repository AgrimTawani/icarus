#!/usr/bin/env python3
"""Capture one timestamped AirSim RGB frame for bounded vision inference."""

import argparse
import hashlib
from pathlib import Path

import cv2
import numpy as np

from python.simulation.airsim_rpc import MsgpackRpcClient


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", required=True, type=Path)
    parser.add_argument("--host", default="127.0.0.1")
    parser.add_argument("--port", type=int, default=41451)
    parser.add_argument("--vehicle", default="Icarus")
    parser.add_argument("--camera", default="0")
    parser.add_argument("--timeout", type=float, default=10)
    args = parser.parse_args()
    rpc = MsgpackRpcClient(args.host, args.port, args.timeout)
    try:
        responses = rpc.images([{
            "camera_name": args.camera,
            "image_type": 0,
            "pixels_as_float": False,
            "compress": True,
        }], args.vehicle)
    finally:
        rpc.close()
    if len(responses) != 1:
        raise SystemExit("AirSim returned no RGB response")
    response = responses[0]
    encoded = response.get("image_data_uint8") or b""
    image = cv2.imdecode(np.frombuffer(encoded, dtype=np.uint8), cv2.IMREAD_COLOR)
    if (int(response.get("time_stamp", 0)) <= 0 or image is None or
            image.size == 0):
        raise SystemExit("AirSim RGB response is empty, malformed, or untimestamped")
    args.output.parent.mkdir(parents=True, exist_ok=True)
    if not cv2.imwrite(str(args.output), image):
        raise SystemExit("could not encode AirSim RGB frame: " + str(args.output))
    payload = args.output.read_bytes()
    print(f"Captured AirSim RGB {image.shape[1]}x{image.shape[0]} "
          f"sha256={hashlib.sha256(payload).hexdigest()} output={args.output}")


if __name__ == "__main__":
    main()

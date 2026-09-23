#!/usr/bin/env python3
"""Read-only AirSim RPC and camera smoke test; does not arm or move a vehicle."""

import argparse
import socket
import threading

import msgpack


class Rpc:
    def __init__(self, host, port, timeout):
        self.sock = socket.create_connection((host, port), timeout=timeout)
        self.sock.settimeout(timeout)
        self.unpacker = msgpack.Unpacker(raw=False)
        self.sequence = 0
        self.lock = threading.Lock()

    def close(self):
        self.sock.close()

    def call(self, method, *params):
        with self.lock:
            self.sequence += 1
            request_id = self.sequence
            self.sock.sendall(msgpack.packb([0, request_id, method, list(params)], use_bin_type=True))
            while True:
                packet = self.sock.recv(4096)
                if not packet:
                    raise ConnectionError("AirSim RPC socket closed")
                self.unpacker.feed(packet)
                for message in self.unpacker:
                    if isinstance(message, list) and len(message) == 4 and message[0] == 1 and message[1] == request_id:
                        if message[2]:
                            raise RuntimeError(f"AirSim RPC error: {message[2]}")
                        return message[3]


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--host", default="127.0.0.1")
    parser.add_argument("--port", type=int, default=41451)
    parser.add_argument("--timeout", type=float, default=10)
    args = parser.parse_args()
    rpc = Rpc(args.host, args.port, args.timeout)
    try:
        if not rpc.call("ping"):
            raise SystemExit("AirSim ping returned false")
        vehicles = rpc.call("listVehicles") or []
        # Image type 0 is Scene. This is a read-only API request and proves
        # that offscreen rendering is producing an API-visible response.
        images = rpc.call("simGetImages", [{"camera_name": "0", "image_type": 0,
                                              "pixels_as_float": False, "compress": True}], "", False) or []
    finally:
        rpc.close()
    print(f"RPC ping: ok; vehicles: {vehicles}; scene image responses: {len(images)}")
    if not images:
        raise SystemExit("AirSim returned no camera response")


if __name__ == "__main__":
    main()

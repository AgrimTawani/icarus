"""Small synchronous read-only AirSim msgpack-RPC client.

Derived from AerialClaw ``adapters/airsim_rpc.py`` at e01adaa under the MIT
License, Copyright (c) 2026 XDEI Group. Control methods were deliberately
removed: Icarus may read simulated sensors here but may only control through
its Drone API -> MAVLink -> ArduPilot route.
"""

import socket
import threading

import msgpack


class MsgpackRpcClient:
    def __init__(self, host="127.0.0.1", port=41451, timeout=10.0):
        self._sock = socket.create_connection((host, port), timeout=timeout)
        self._sock.settimeout(timeout)
        self._msg_id = 0
        self._lock = threading.Lock()
        self._unpacker = msgpack.Unpacker(raw=False)

    def close(self):
        self._sock.close()

    def call(self, method, *args):
        with self._lock:
            self._msg_id = (self._msg_id + 1) & 0xFFFFFFFF
            message_id = self._msg_id
            payload = msgpack.packb(
                [0, message_id, method, list(args)], use_bin_type=True)
            self._sock.sendall(payload)
            self._unpacker = msgpack.Unpacker(raw=False)
            while True:
                chunk = self._sock.recv(4096)
                if not chunk:
                    raise ConnectionError("AirSim RPC connection closed")
                self._unpacker.feed(chunk)
                for message in self._unpacker:
                    if (isinstance(message, list) and len(message) == 4
                            and message[0] == 1 and message[1] == message_id):
                        if message[2]:
                            raise RuntimeError(f"AirSim RPC error: {message[2]}")
                        return message[3]

    def ping(self):
        return bool(self.call("ping"))

    def list_vehicles(self):
        return self.call("listVehicles") or []

    def multirotor_state(self, vehicle):
        return self.call("getMultirotorState", vehicle) or {}

    def images(self, requests, vehicle):
        return self.call("simGetImages", requests, vehicle, False) or []

    def lidar(self, sensor, vehicle):
        return self.call("getLidarData", sensor, vehicle) or {}

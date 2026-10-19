#!/usr/bin/env python3
"""Gazebo-only sensor input to SITL; never issues flight or parameter commands.
Initial battery coulomb-counter alignment is performed once, disarmed, from
measured Gazebo charge; this is not physical recharge or a periodic reset.

Run with ArduPilot's venv (pymavlink + system Gazebo bindings). SERIAL2 uses
UDP loopback; the operator/GCS link and its heartbeat failsafe remain separate.
Missing/invalid samples are not replayed as fresh sensor observations.
"""

import argparse
import json
import math
import os
import signal
import threading
import time
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[2]
PARAMETERS = ROOT / "simulation/parameters/gazebo_sensors.parm"
TOPICS = {
    "lidar": "/icarus/sensors/mid360",
    "range": "/icarus/sensors/ark_flow_range",
    "flow": "/icarus/sensors/ark_flow/optical_flow",
}
MAX_AGE_S = 0.35


def expected_parameters():
    return {
        line.split()[0]: float(line.split()[1])
        for line in PARAMETERS.read_text().splitlines()
        if line.strip() and not line.startswith("#")
    }


def downward_distance(scan):
    """Median axial distance of valid rays; distinguish no-return from corrupt data."""
    h, v = int(scan.count), int(scan.vertical_count)
    if h < 1 or v < 1 or len(scan.ranges) != h * v:
        raise ValueError("malformed downward scan")
    values = np.asarray(scan.ranges, dtype=float).reshape(v, h)
    valid = np.isfinite(values) & (values >= scan.range_min) & (values < scan.range_max)
    if not valid.any():
        # Positive infinity is the Gazebo no-hit convention. NaNs are unknown.
        if np.all(np.isposinf(values) | (values >= scan.range_max)):
            return round(scan.range_max * 100) + 1
        if np.all(
            np.isneginf(values) | (np.isfinite(values) & (values < scan.range_min))
        ):
            return max(0, math.ceil(scan.range_min * 100) - 1)
        return None
    az = scan.angle_min + np.arange(h) * scan.angle_step
    el = scan.vertical_angle_min + np.arange(v) * scan.vertical_angle_step
    axial = values * np.cos(el)[:, None] * np.cos(az)[None, :]
    return max(0, round(float(np.median(axial[valid])) * 100))


def proximity_sectors(scan):
    """Horizontal Mid360 slice -> 72 clockwise BODY_FRD sectors, centimetres.

    Retain the existing 1m self-mask, but masked/NaN returns remain UNKNOWN,
    not free space. The companion's full 3-D cloud is unchanged. Positive
    infinity means observed no-hit; unknown sectors use MAVLink UINT16_MAX.
    """
    h, v = int(scan.count), int(scan.vertical_count)
    if h < 1 or v < 1 or len(scan.ranges) != h * v:
        raise ValueError("malformed lidar scan")
    distances = [65535] * 72
    minimum = max(1.0, scan.range_min)
    for row in range(v):
        elevation = scan.vertical_angle_min + row * scan.vertical_angle_step
        if abs(elevation) > math.radians(5):
            continue
        for column in range(h):
            r = scan.ranges[row * h + column]
            azimuth = scan.angle_min + column * scan.angle_step
            sector = math.floor((-math.degrees(azimuth) % 360 + 2.5) / 5) % 72
            if math.isinf(r) and r > 0 or math.isfinite(r) and r >= scan.range_max:
                if distances[sector] == 65535:
                    distances[sector] = round(scan.range_max * 100) + 1
            elif math.isfinite(r) and minimum <= r < scan.range_max:
                # Include the known forward sensor mounting offset.
                x = r * math.cos(elevation) * math.cos(azimuth) + 0.070
                y = -r * math.cos(elevation) * math.sin(azimuth)
                sector = (
                    math.floor((math.degrees(math.atan2(y, x)) % 360 + 2.5) / 5) % 72
                )
                distances[sector] = min(
                    distances[sector], round(math.hypot(x, y) * 100)
                )
    return distances


def flow_rates(observation):
    if observation.get("frame") != "body_frd":
        raise ValueError("flow must be calibrated body_frd")
    dt = float(observation["integration_interval_s"])
    if not math.isfinite(dt) or not 0.005 <= dt <= 0.2:
        raise ValueError("invalid flow integration interval")
    x = float(observation["flow_integral_rad"]["x"]) / dt
    y = float(observation["flow_integral_rad"]["y"]) / dt
    if not all(math.isfinite(n) for n in (x, y)):
        raise ValueError("non-finite flow")
    quality = int(observation["quality"])
    if not 0 <= quality <= 255:
        raise ValueError("invalid flow quality")
    # Do not let weak correlation or aliased/extreme motion aid navigation.
    if quality < 100 or math.hypot(x, y) >= 4.0:
        quality = 0
    return x, y, quality


class LatestSamples:
    """Single-slot queues: never resend stale or duplicate simulation samples."""

    def __init__(self):
        self.lock = threading.Lock()
        self.samples = {}
        self.last_stamp = {}
        self.last_receive = {}
        self.dropped = set()
        self.invalid = 0

    def put(self, name, stamp, payload, now):
        with self.lock:
            if (
                name in self.dropped
                or not math.isfinite(stamp)
                or stamp <= self.last_stamp.get(name, -1)
            ):
                return
            self.last_stamp[name] = stamp
            self.last_receive[name] = now
            self.samples[name] = (stamp, payload, now)

    def take(self, now):
        with self.lock:
            pending, self.samples = self.samples, {}
            return {
                k: v
                for k, v in pending.items()
                if k not in self.dropped and 0 <= now - v[2] <= MAX_AGE_S
            }


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--directory", type=Path, required=True)
    args = parser.parse_args()
    # Fixed loopback endpoint intentionally has no real-aircraft CLI override.
    os.environ["MAVLINK20"] = "1"
    from battery_profile import BatteryStartupSync
    from gz.msgs10.battery_state_pb2 import BatteryState
    from gz.msgs10.laserscan_pb2 import LaserScan
    from gz.msgs10.stringmsg_pb2 import StringMsg
    from gz.transport13 import Node
    from pymavlink import mavutil

    link = mavutil.mavlink_connection(
        "udpin:127.0.0.1:14562",
        source_system=1,
        source_component=158,
        dialect="ardupilotmega",
    )
    node, samples = Node(), LatestSamples()
    stopping = threading.Event()
    signal.signal(signal.SIGINT, lambda *_: stopping.set())
    signal.signal(signal.SIGTERM, lambda *_: stopping.set())
    counters = {name: 0 for name in TOPICS}
    state = {
        "schema": "icarus.sim.autopilot-sensors.v1",
        "status": "starting",
        "simulation_only": True,
        "link": "udp:127.0.0.1:14562",
        "parameters": {},
        "sent": counters,
        "telemetry": {},
        "flow_fusion_configured": False,
        "flow_fusion_verified": False,
        "note": "Reception is not proof of EKF fusion; inspect acceptance/DataFlash evidence.",
    }
    expected = expected_parameters()
    battery_sync = BatteryStartupSync(args.directory / "battery_initialization.json")
    battery_params = args.directory / "gazebo_battery.parm"
    if battery_params.exists():
        expected.update({line.split()[0]: float(line.split()[1])
                         for line in battery_params.read_text().splitlines()
                         if line.strip() and not line.startswith('#')})
    def battery_sample(message):
        try:
            with samples.lock:
                battery_sync.observe(message.charge, message.capacity, message.voltage,
                                     message.current, message.header.stamp.sec + message.header.stamp.nsec*1e-9,
                                     time.monotonic())
        except ValueError:
            with samples.lock:
                samples.invalid += 1
    if not node.subscribe(BatteryState, "/model/icarus_compact/battery/flight_battery/state", battery_sample):
        raise RuntimeError("cannot subscribe to authoritative simulator battery")
    callbacks = []

    def callback(name):
        def receive(message):
            try:
                stamp = message.header.stamp.sec + message.header.stamp.nsec * 1e-9
                if name == "flow":
                    observation = json.loads(message.data)
                    payload = flow_rates(observation)
                elif name == "range":
                    payload = downward_distance(message)
                    if payload is None:
                        raise ValueError("invalid range")
                else:
                    payload = proximity_sectors(message)
                    if all(value == 65535 for value in payload):
                        raise ValueError("no valid proximity rays")
                samples.put(name, stamp, payload, time.monotonic())
            except (ValueError, KeyError, TypeError, OverflowError):
                with samples.lock:
                    samples.invalid += 1

        return receive

    for name, topic in TOPICS.items():
        cb = callback(name)
        callbacks.append(cb)
        if not node.subscribe(StringMsg if name == "flow" else LaserScan, topic, cb):
            raise RuntimeError("cannot subscribe to " + topic)

    def fault(message):
        # Existing simulation fault bus only; no natural-language mission parsing.
        try:
            data = json.loads(message.data)
        except ValueError:
            return
        name = {"lidar": "lidar", "range": "range", "optical_flow": "flow"}.get(
            data.get("channel")
        )
        if name:
            with samples.lock:
                if data.get("mode") == "drop":
                    samples.dropped.add(name)
                    samples.samples.pop(name, None)
                elif data.get("mode") == "normal":
                    samples.dropped.discard(name)

    node.subscribe(StringMsg, "/icarus/test/sensor_fault", fault)

    last_heartbeat, next_request, next_health = 0.0, 0.0, 0.0
    requested_stream = False
    armed = False
    battery_sync_started = time.monotonic()
    health_path = args.directory / "autopilot_sensors.json"
    try:
        while not stopping.wait(0.005):
            now = time.monotonic()
            for _ in range(100):
                message = link.recv_match(blocking=False)
                if message is None:
                    break
                if message.get_srcSystem() != 1 or message.get_srcComponent() != 1:
                    continue
                kind = message.get_type()
                if kind == "HEARTBEAT":
                    last_heartbeat = now
                    armed = bool(message.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED)
                elif kind == "COMMAND_ACK" and message.command == mavutil.mavlink.MAV_CMD_BATTERY_RESET:
                    battery_sync.acknowledge(message.result == mavutil.mavlink.MAV_RESULT_ACCEPTED, now)
                elif kind == "PARAM_VALUE":
                    name = message.param_id
                    if isinstance(name, bytes):
                        name = name.decode().rstrip("\0")
                    if name in expected:
                        state["parameters"][name] = float(message.param_value)
                elif kind in (
                    "OPTICAL_FLOW",
                    "DISTANCE_SENSOR",
                    "SYS_STATUS",
                    "RANGEFINDER",
                ):
                    record = message.to_dict()
                    record["received_monotonic_s"] = now
                    key = kind
                    if kind == "DISTANCE_SENSOR":
                        key += "_" + str(message.orientation)
                    state["telemetry"][key] = record
                    if kind == "SYS_STATUS":
                        with samples.lock:
                            battery_sync.verify(message.battery_remaining, now)
                elif kind == "STATUSTEXT" and "fusing optical flow" in str(
                    message.text
                ):
                    state["flow_fusion_verified"] = True
                    state["flow_fusion_evidence"] = str(message.text)
            with samples.lock:
                percent = battery_sync.request(now, armed, bool(last_heartbeat and now-last_heartbeat < 3))
            if percent is not None:
                # Only a fresh simulation session may align the AP counter.
                # The authoritative physical battery is never changed.
                link.mav.command_long_send(1, 1, mavutil.mavlink.MAV_CMD_BATTERY_RESET,
                                           0, 1, percent, 0, 0, 0, 0, 0)
            if not battery_sync.initialized and now-battery_sync_started > 60:
                raise RuntimeError("battery startup SOC acknowledgement/telemetry verification timed out")
            if last_heartbeat and not requested_stream:
                link.mav.request_data_stream_send(
                    1, 1, mavutil.mavlink.MAV_DATA_STREAM_ALL, 10, 1
                )
                requested_stream = True
            if last_heartbeat and now >= next_request:
                missing = [k for k in expected if k not in state["parameters"]]
                for key in missing[:8]:
                    link.mav.param_request_read_send(1, 1, key.encode(), -1)
                next_request = now + 0.25
            if last_heartbeat and now - last_heartbeat < 3:
                for name, (stamp, payload, received) in samples.take(now).items():
                    if name == "lidar":
                        link.mav.obstacle_distance_send(
                            int(stamp * 1e6), 0, payload, 5, 100, 4000, 5.0, 0.0, 12
                        )
                    elif name == "range":
                        link.mav.distance_sensor_send(
                            int(stamp * 1000) % 2**32, 8, 3000, payload, 0, 11, 25, 255
                        )
                        state["last_range_cm"] = payload
                    else:
                        x, y, quality = payload
                        # MAVLink2 extension flow_rate_x/y is what this pinned
                        # AP_OpticalFlow_MAV backend accepts. It uses AHRS gyro.
                        link.mav.optical_flow_send(
                            int(stamp * 1e6), 1, 0, 0, 0, 0, quality, -1.0, x, y
                        )
                        state["last_flow"] = {
                            "x_rad_s": x,
                            "y_rad_s": y,
                            "quality": quality,
                        }
                    counters[name] += 1
            if now >= next_health:
                with samples.lock:
                    state["source_age_s"] = {
                        k: now - samples.last_receive.get(k, now - 999) for k in TOPICS
                    }
                    state["invalid_samples"] = samples.invalid
                state["parameter_mismatches"] = {
                    k: {"expected": value, "actual": state["parameters"].get(k)}
                    for k, value in expected.items()
                    if not math.isclose(
                        state["parameters"].get(k, math.inf), value, abs_tol=1e-5
                    )
                }
                state["flow_fusion_configured"] = not any(
                    name in state["parameter_mismatches"]
                    for name in (
                        "FLOW_TYPE",
                        "EK3_SRC2_VELXY",
                        "EK3_SRC_OPTIONS",
                        "EK3_FLOW_USE",
                    )
                )
                state["heartbeat_age_s"] = (
                    now - last_heartbeat if last_heartbeat else None
                )
                telemetry_ready = all(
                    now - state["telemetry"].get(k, {}).get("received_monotonic_s", 0)
                    < 2
                    for k in (
                        "OPTICAL_FLOW",
                        "DISTANCE_SENSOR_25",
                        "DISTANCE_SENSOR_0",
                        "SYS_STATUS",
                    )
                )
                state["battery_initialization"] = battery_sync.record
                ready = (
                    battery_sync.initialized
                    and last_heartbeat
                    and now - last_heartbeat < 3
                    and not state["parameter_mismatches"]
                    and all(n > 2 for n in counters.values())
                    and all(age < 2 for age in state["source_age_s"].values())
                    and telemetry_ready
                )
                state["status"] = "ready" if ready else "degraded"
                state["updated_monotonic_s"] = now
                temp = health_path.with_suffix(".tmp")
                temp.write_text(json.dumps(state, indent=2) + "\n")
                temp.replace(health_path)
                next_health = now + 0.25
    finally:
        state["status"] = "stopped"
        health_path.write_text(json.dumps(state, indent=2) + "\n")
        link.close()


if __name__ == "__main__":
    main()

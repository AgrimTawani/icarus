#!/usr/bin/env python3
"""Deterministic MAVLink controller for the official Iris smoke test."""

from __future__ import annotations

import argparse
import json
import sys
import time
from pathlib import Path

from pymavlink import mavutil


def wait_until(predicate, timeout_s: float, description: str) -> None:
    deadline = time.monotonic() + timeout_s
    while time.monotonic() < deadline:
        if predicate():
            return
        time.sleep(0.1)
    raise TimeoutError(f"timed out waiting for {description}")


def wait_heartbeat(master: mavutil.mavfile, timeout_s: float = 30.0) -> None:
    heartbeat = master.wait_heartbeat(timeout=timeout_s)
    if heartbeat is None:
        raise TimeoutError("no MAVLink heartbeat")
    master.target_system = heartbeat.get_srcSystem()
    master.target_component = heartbeat.get_srcComponent()


def set_mode(master: mavutil.mavfile, mode_name: str) -> None:
    mapping = master.mode_mapping()
    if mode_name not in mapping:
        raise RuntimeError(f"mode {mode_name} unavailable: {sorted(mapping)}")
    master.mav.set_mode_send(
        master.target_system,
        mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
        mapping[mode_name],
    )

    def selected() -> bool:
        message = master.recv_match(type="HEARTBEAT", blocking=True, timeout=1)
        return message is not None and message.custom_mode == mapping[mode_name]

    wait_until(selected, 10.0, f"mode {mode_name}")


def wait_navigation_ready(master: mavutil.mavfile) -> None:
    """Wait for a 3D fix and a populated global-position stream."""
    has_fix = False
    has_position = False
    deadline = time.monotonic() + 60.0
    while time.monotonic() < deadline:
        message = master.recv_match(
            type=["GPS_RAW_INT", "GLOBAL_POSITION_INT"], blocking=True, timeout=1
        )
        if message is None:
            continue
        if message.get_type() == "GPS_RAW_INT" and message.fix_type >= 3:
            has_fix = True
        elif message.get_type() == "GLOBAL_POSITION_INT":
            has_position = True
        if has_fix and has_position:
            return
    raise TimeoutError("navigation state did not become ready")


def arm(master: mavutil.mavfile) -> None:
    deadline = time.monotonic() + 45.0
    next_request = 0.0
    status_messages: list[str] = []
    while time.monotonic() < deadline:
        now = time.monotonic()
        if now >= next_request:
            master.mav.command_long_send(
                master.target_system,
                master.target_component,
                mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
                0,
                1,
                0,
                0,
                0,
                0,
                0,
                0,
            )
            next_request = now + 2.0

        message = master.recv_match(blocking=True, timeout=0.5)
        if message is None:
            continue
        if message.get_type() == "STATUSTEXT":
            status_messages.append(str(message.text))
            status_messages = status_messages[-8:]
        if message.get_type() == "HEARTBEAT" and master.motors_armed():
            return

    detail = "; ".join(status_messages) or "no STATUSTEXT received"
    raise TimeoutError(f"timed out waiting for vehicle to arm: {detail}")


def takeoff(master: mavutil.mavfile, altitude_m: float) -> None:
    master.mav.command_long_send(
        master.target_system,
        master.target_component,
        mavutil.mavlink.MAV_CMD_NAV_TAKEOFF,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        altitude_m,
    )


def relative_altitude(master: mavutil.mavfile) -> float | None:
    message = master.recv_match(type="GLOBAL_POSITION_INT", blocking=True, timeout=1)
    if message is None:
        return None
    return message.relative_alt / 1000.0


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--connect", default="tcp:127.0.0.1:5760")
    parser.add_argument("--altitude", type=float, default=5.0)
    parser.add_argument("--hover-seconds", type=float, default=5.0)
    parser.add_argument("--result", type=Path, required=True)
    args = parser.parse_args()

    started = time.time()
    result: dict[str, object] = {
        "status": "failed",
        "target_altitude_m": args.altitude,
        "hover_seconds": args.hover_seconds,
        "connection": args.connect,
    }

    master = mavutil.mavlink_connection(args.connect, autoreconnect=True)
    try:
        wait_heartbeat(master)
        result["system_id"] = master.target_system
        result["component_id"] = master.target_component
        master.mav.request_data_stream_send(
            master.target_system,
            master.target_component,
            mavutil.mavlink.MAV_DATA_STREAM_ALL,
            10,
            1,
        )

        wait_navigation_ready(master)
        set_mode(master, "GUIDED")
        arm(master)
        set_mode(master, "GUIDED")
        takeoff(master, args.altitude)

        peak_altitude = 0.0

        def reached_altitude() -> bool:
            nonlocal peak_altitude
            altitude = relative_altitude(master)
            if altitude is None:
                return False
            peak_altitude = max(peak_altitude, altitude)
            return altitude >= args.altitude * 0.90

        wait_until(reached_altitude, 45.0, "takeoff altitude")

        hover_deadline = time.monotonic() + args.hover_seconds
        hover_samples: list[float] = []
        while time.monotonic() < hover_deadline:
            altitude = relative_altitude(master)
            if altitude is not None:
                peak_altitude = max(peak_altitude, altitude)
                hover_samples.append(altitude)

        set_mode(master, "LAND")
        def disarmed() -> bool:
            master.recv_match(type="HEARTBEAT", blocking=True, timeout=1)
            return not master.motors_armed()

        wait_until(disarmed, 60.0, "vehicle to disarm after landing")

        result.update(
            status="passed",
            peak_altitude_m=round(peak_altitude, 3),
            hover_sample_count=len(hover_samples),
            hover_min_altitude_m=round(min(hover_samples), 3),
            hover_max_altitude_m=round(max(hover_samples), 3),
        )
        return 0
    except Exception as exc:  # noqa: BLE001 - persist the exact smoke-test failure
        result["error"] = f"{type(exc).__name__}: {exc}"
        print(result["error"], file=sys.stderr)
        return 1
    finally:
        result["duration_s"] = round(time.time() - started, 3)
        args.result.write_text(json.dumps(result, indent=2) + "\n", encoding="utf-8")
        master.close()


if __name__ == "__main__":
    raise SystemExit(main())

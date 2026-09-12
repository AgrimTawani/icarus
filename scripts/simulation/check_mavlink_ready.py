#!/usr/bin/env python3
"""Verify that SITL publishes usable navigation telemetry without arming it."""

import argparse
import time

from pymavlink import mavutil


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--endpoint", default="tcp:127.0.0.1:5760")
    parser.add_argument("--timeout", type=float, default=45.0)
    args = parser.parse_args()

    connection = mavutil.mavlink_connection(args.endpoint, source_system=254)
    heartbeat = connection.wait_heartbeat(timeout=args.timeout)
    if heartbeat is None:
        raise TimeoutError("ArduPilot heartbeat timed out")
    connection.target_system = heartbeat.get_srcSystem()
    connection.target_component = heartbeat.get_srcComponent()
    connection.mav.request_data_stream_send(
        connection.target_system,
        connection.target_component,
        mavutil.mavlink.MAV_DATA_STREAM_ALL,
        10,
        1,
    )
    deadline = time.monotonic() + args.timeout
    gps_fix = 0
    global_position = False
    while time.monotonic() < deadline:
        message = connection.recv_match(blocking=True, timeout=0.5)
        if message is None:
            continue
        if message.get_type() == "GPS_RAW_INT":
            gps_fix = int(message.fix_type)
        elif message.get_type() == "GLOBAL_POSITION_INT":
            global_position = True
        if gps_fix >= 3 and global_position:
            connection.close()
            print(f"MAVLink ready: GPS fix {gps_fix}, global position available")
            return
    connection.close()
    raise TimeoutError(
        f"navigation readiness timed out: gps_fix={gps_fix}, "
        f"global_position={global_position}"
    )


if __name__ == "__main__":
    main()

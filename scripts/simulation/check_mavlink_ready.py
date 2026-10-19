#!/usr/bin/env python3
"""Wait for measured SITL navigation readiness without arming it."""

import argparse
import time

# MAVLink EKF_STATUS_FLAGS; predicted position is not actual navigation validity.
EKF_ATTITUDE = 1
EKF_POS_HORIZ_REL = 8
EKF_POS_HORIZ_ABS = 16
EKF_POS_VERT_ABS = 32
EKF_UNINITIALIZED = 1024


class NavigationReadiness:
    def __init__(self, maximum_age_s=3.0):
        self.maximum_age_s = maximum_age_s
        self.seen = {}
        self.gps_fix = 0
        self.ekf_flags = 0
        self.autopilot_boot_ms = None

    def observe(self, message, now):
        kind = message.get_type()
        if kind in ('HEARTBEAT', 'GPS_RAW_INT', 'GLOBAL_POSITION_INT', 'EKF_STATUS_REPORT'):
            self.seen[kind] = now
        if kind == 'GPS_RAW_INT':
            self.gps_fix = int(message.fix_type)
        elif kind == 'GLOBAL_POSITION_INT':
            self.autopilot_boot_ms = getattr(message, 'time_boot_ms', None)
        elif kind == 'EKF_STATUS_REPORT':
            self.ekf_flags = int(message.flags)

    def status(self, now):
        fresh = all(kind in self.seen and 0 <= now-self.seen[kind] <= self.maximum_age_s
                    for kind in ('HEARTBEAT', 'GPS_RAW_INT', 'GLOBAL_POSITION_INT', 'EKF_STATUS_REPORT'))
        flags = self.ekf_flags
        ekf_healthy = bool(flags & EKF_ATTITUDE and flags & (EKF_POS_HORIZ_ABS | EKF_POS_HORIZ_REL)
                           and flags & EKF_POS_VERT_ABS and not flags & EKF_UNINITIALIZED)
        return {'ready': fresh and self.gps_fix >= 3 and ekf_healthy,
                'fresh': fresh, 'gps_fix': self.gps_fix, 'ekf_flags': flags,
                'ekf_healthy': ekf_healthy, 'autopilot_boot_ms': self.autopilot_boot_ms}


def main():
    from pymavlink import mavutil

    parser = argparse.ArgumentParser()
    parser.add_argument('--endpoint', default='tcp:127.0.0.1:5760')
    parser.add_argument('--timeout', type=float, default=45.0)
    args = parser.parse_args()
    if not 0 < args.timeout <= 600:
        parser.error("--timeout must be positive and at most 600 seconds")
    started = time.monotonic()
    deadline = started + args.timeout
    connection = mavutil.mavlink_connection(args.endpoint, source_system=254)
    readiness = NavigationReadiness()
    try:
        heartbeat = connection.wait_heartbeat(timeout=max(0, deadline-time.monotonic()))
        if heartbeat is None:
            raise TimeoutError('ArduPilot heartbeat timed out')
        connection.target_system = heartbeat.get_srcSystem()
        connection.target_component = heartbeat.get_srcComponent()
        readiness.observe(heartbeat, time.monotonic())
        connection.mav.request_data_stream_send(
            connection.target_system, connection.target_component,
            mavutil.mavlink.MAV_DATA_STREAM_ALL, 10, 1)
        connection.mav.command_long_send(
            connection.target_system, connection.target_component,
            mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL, 0,
            mavutil.mavlink.MAVLINK_MSG_ID_EKF_STATUS_REPORT, 200000, 0, 0, 0, 0, 0)
        previous = None
        while time.monotonic() < deadline:
            message = connection.recv_match(blocking=True, timeout=min(0.5, max(0, deadline-time.monotonic())))
            if message is None:
                continue
            if (message.get_srcSystem(), message.get_srcComponent()) != (connection.target_system, connection.target_component):
                continue
            now = time.monotonic()
            readiness.observe(message, now)
            if message.get_type() == "STATUSTEXT":
                print("ArduPilot: " + str(message.text), flush=True)
            status = readiness.status(now)
            diagnostic = (status["gps_fix"], status["ekf_flags"])
            if diagnostic != previous:
                print(f"Navigation initialization: gps_fix={diagnostic[0]}, ekf_flags={diagnostic[1]}", flush=True)
                previous = diagnostic
            if status['ready']:
                print(f"MAVLink ready after {now-started:.1f}s wall: GPS fix {status['gps_fix']}, fresh global position and healthy fused EKF (flags={status['ekf_flags']}, boot_ms={status['autopilot_boot_ms']})")
                return
        raise TimeoutError(f'navigation readiness timed out: {readiness.status(time.monotonic())}')
    finally:
        connection.close()


if __name__ == '__main__':
    main()

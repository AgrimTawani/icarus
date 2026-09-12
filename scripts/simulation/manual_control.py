#!/usr/bin/env python3
"""Simulation-only keyboard and Xbox RC client for an active Icarus session."""

import argparse
import json
import math
import os
import time
from pathlib import Path

import pygame
from pymavlink import mavutil

ROOT = Path(__file__).resolve().parents[2]
ACTIVE_SESSION = ROOT / "logs/simulation/active_session.json"
RELEASE = 0
IGNORE = 65535
XBOX_MODE_HAT = {
    (-1, 0): "STABILIZE",
    (0, -1): "ALT_HOLD",
    (0, 1): "LOITER",
    (1, 0): "ACRO",
}


def clamp(value, minimum=-1.0, maximum=1.0):
    return max(minimum, min(maximum, value))


def shape_axis(value, deadzone=0.08):
    """Apply a dead zone and gentle exponential response to a normalized axis."""
    value = clamp(float(value))
    if abs(value) <= deadzone:
        return 0.0
    scaled = (abs(value) - deadzone) / (1.0 - deadzone)
    return math.copysign(scaled**1.5, value)


def throttle_from_axis(value, deadzone=0.08):
    """Map a spring-centered Xbox axis to transmitter throttle: center=0, up=1."""
    upward = clamp(-float(value), 0.0, 1.0)
    if upward <= deadzone:
        return 0.0
    return (upward - deadzone) / (1.0 - deadzone)


def throttle_pwm(value):
    return round(1000 + 1000 * clamp(value, 0.0, 1.0))


def pwm(value, reverse=False):
    value = -value if reverse else value
    return round(1500 + 400 * clamp(value))


def load_session():
    if not ACTIVE_SESSION.is_file():
        raise RuntimeError("No active simulator; run ./scripts/start-sim first")
    session = json.loads(ACTIVE_SESSION.read_text())
    if session.get("status") != "ready":
        raise RuntimeError("Simulator session is not ready")
    try:
        os.kill(int(session["launcher_pid"]), 0)
    except (KeyError, ProcessLookupError, ValueError) as error:
        raise RuntimeError("Simulator session is stale; restart ./scripts/start-sim") from error
    if session.get("mavlink_endpoint") != "tcp:127.0.0.1:5760":
        raise RuntimeError("Manual control is restricted to the local SITL endpoint")
    return session


def open_joystick(index):
    if index is None or pygame.joystick.get_count() == 0:
        return None
    if not 0 <= index < pygame.joystick.get_count():
        raise ValueError(f"controller {index} not found")
    joystick = pygame.joystick.Joystick(index)
    joystick.init()
    return joystick


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--controller", type=int, default=0)
    parser.add_argument("--keyboard-only", action="store_true")
    parser.add_argument("--takeoff-altitude", type=float, default=3.0)
    parser.add_argument("--list-controllers", action="store_true")
    parser.add_argument(
        "--connection-test",
        action="store_true",
        help="Verify the active local MAVLink session without opening pilot controls",
    )
    args = parser.parse_args()
    if not 1.0 <= args.takeoff_altitude <= 10.0:
        raise ValueError("takeoff altitude must be between 1 and 10 metres")

    pygame.init()
    pygame.joystick.init()
    if args.list_controllers:
        if pygame.joystick.get_count() == 0:
            print("No SDL controllers detected")
        for index in range(pygame.joystick.get_count()):
            item = pygame.joystick.Joystick(index)
            print(f"{index}: {item.get_name()}")
        return

    session = load_session()
    joystick = None if args.keyboard_only else open_joystick(args.controller)
    master = mavutil.mavlink_connection(session["mavlink_endpoint"], source_system=255)
    print("Waiting for ArduPilot heartbeat...", flush=True)
    heartbeat = master.wait_heartbeat(timeout=30)
    if heartbeat is None:
        raise RuntimeError("ArduPilot heartbeat timed out")
    master.target_system = heartbeat.get_srcSystem()
    master.target_component = heartbeat.get_srcComponent()
    mode_numbers = master.mode_mapping()
    if args.connection_test:
        master.mav.heartbeat_send(
            mavutil.mavlink.MAV_TYPE_GCS,
            mavutil.mavlink.MAV_AUTOPILOT_INVALID,
            0,
            0,
            0,
        )
        master.mav.rc_channels_override_send(
            master.target_system,
            master.target_component,
            RELEASE,
            RELEASE,
            RELEASE,
            RELEASE,
            IGNORE,
            IGNORE,
            IGNORE,
            IGNORE,
        )
        master.close()
        pygame.quit()
        print("Manual-control connection check passed")
        return

    screen = pygame.display.set_mode((820, 570))
    pygame.display.set_caption("Icarus Manual Pilot — simulation only")
    font = pygame.font.Font(None, 28)
    small = pygame.font.Font(None, 22)
    clock = pygame.time.Clock()
    state = {
        "armed": bool(heartbeat.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED),
        "mode": "UNKNOWN",
        "altitude_m": 0.0,
        "battery_v": None,
        "gps_fix": 0,
        "local_position": False,
        "status": "Connected; LOITER requested",
    }
    arm_pending = False
    arm_deadline = 0.0
    next_arm_request = 0.0
    pending_takeoff = False
    takeoff_sent = False
    running = True
    next_gcs_heartbeat = 0.0

    client_dir = Path(session["run_directory"]) / "clients"
    client_dir.mkdir(exist_ok=True)
    log_path = client_dir / ("manual_" + time.strftime("%Y%m%dT%H%M%S") + ".jsonl")
    log = log_path.open("w")

    def record(event, **fields):
        log.write(json.dumps({"time": time.time(), "event": event, **fields}) + "\n")
        log.flush()

    def set_mode(name):
        if name not in mode_numbers:
            state["status"] = f"Mode unavailable in this ArduPilot build: {name}"
            record("mode_unavailable", mode=name)
            return
        master.mav.set_mode_send(
            master.target_system,
            mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
            mode_numbers[name],
        )
        warning = {
            "ACRO": " — rate control; no self-level or altitude hold",
            "STABILIZE": " — self-level; manual throttle",
            "ALT_HOLD": " — altitude hold; no position hold",
            "LOITER": " — position and altitude hold",
        }.get(name, "")
        state["status"] = f"Requested {name}{warning}"
        record("mode_request", mode=name)

    def command(command_id, first=0.0, seventh=0.0):
        master.mav.command_long_send(
            master.target_system,
            master.target_component,
            command_id,
            0,
            first,
            0,
            0,
            0,
            0,
            0,
            seventh,
        )

    def arm():
        nonlocal arm_pending, arm_deadline, next_arm_request
        set_mode("LOITER")
        arm_pending = True
        arm_deadline = time.monotonic() + 45
        next_arm_request = 0.0
        state["status"] = "Arm pending; waiting for EKF/GPS readiness"
        record("arm_request")

    def disarm():
        nonlocal arm_pending
        arm_pending = False
        if state["altitude_m"] > 0.3:
            state["status"] = "Disarm blocked above 0.3 m; LAND instead"
            return
        command(mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 0)
        state["status"] = "Disarm requested"
        record("disarm_request")

    def takeoff():
        nonlocal pending_takeoff, takeoff_sent
        if not state["armed"]:
            state["status"] = "Arm first, then request takeoff"
            return
        set_mode("GUIDED")
        pending_takeoff = True
        takeoff_sent = False
        state["status"] = f"Preparing takeoff to {args.takeoff_altitude:.1f} m"

    def action(name):
        nonlocal arm_pending, pending_takeoff, takeoff_sent
        if name != "arm":
            arm_pending = False
        if name != "takeoff":
            pending_takeoff = False
            takeoff_sent = False
        if name == "arm":
            arm()
        elif name == "disarm":
            disarm()
        elif name == "takeoff":
            takeoff()
        elif name in ("LAND", "RTL", "LOITER", "ALT_HOLD", "STABILIZE", "ACRO"):
            set_mode(name)

    set_mode("LOITER")
    master.mav.request_data_stream_send(
        master.target_system,
        master.target_component,
        mavutil.mavlink.MAV_DATA_STREAM_ALL,
        20,
        1,
    )
    record("connected", controller=joystick.get_name() if joystick else None)
    previous_buttons = []
    previous_mode_chord = (False, (0, 0))
    try:
        while running:
            for event in pygame.event.get():
                if event.type == pygame.QUIT:
                    running = False
                elif event.type == pygame.KEYDOWN:
                    key_actions = {
                        pygame.K_RETURN: "arm",
                        pygame.K_BACKSPACE: "disarm",
                        pygame.K_t: "takeoff",
                        pygame.K_l: "LAND",
                        pygame.K_r: "RTL",
                        pygame.K_h: "LOITER",
                        pygame.K_1: "STABILIZE",
                        pygame.K_2: "ALT_HOLD",
                        pygame.K_3: "LOITER",
                        pygame.K_4: "ACRO",
                    }
                    if event.key == pygame.K_ESCAPE:
                        running = False
                    elif event.key in key_actions:
                        action(key_actions[event.key])

            while True:
                message = master.recv_match(blocking=False)
                if message is None:
                    break
                kind = message.get_type()
                if kind == "HEARTBEAT":
                    state["armed"] = bool(
                        message.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED
                    )
                    state["mode"] = mavutil.mode_string_v10(message)
                elif kind == "GLOBAL_POSITION_INT":
                    state["altitude_m"] = message.relative_alt / 1000.0
                elif kind == "GPS_RAW_INT":
                    state["gps_fix"] = message.fix_type
                elif kind == "LOCAL_POSITION_NED":
                    state["local_position"] = True
                elif kind == "SYS_STATUS" and message.voltage_battery != 65535:
                    state["battery_v"] = message.voltage_battery / 1000.0
                elif kind == "STATUSTEXT":
                    state["status"] = str(message.text)
                    record("status_text", text=str(message.text))

            now = time.monotonic()
            if arm_pending:
                if state["armed"]:
                    arm_pending = False
                    state["status"] = "Armed; press T or Xbox Y to take off"
                    record("armed")
                elif now >= arm_deadline:
                    arm_pending = False
                    state["status"] = "Arming timed out; check the latest PreArm message"
                    record("arm_timeout")
                elif now >= next_arm_request:
                    command(mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 1)
                    next_arm_request = now + 3
                    state["status"] = "Arming: waiting for position/EKF checks"
                    record(
                        "arm_retry",
                        gps_fix=state["gps_fix"],
                        local_position=state["local_position"],
                    )

            if pending_takeoff and state["mode"] == "GUIDED" and not takeoff_sent:
                command(
                    mavutil.mavlink.MAV_CMD_NAV_TAKEOFF,
                    seventh=args.takeoff_altitude,
                )
                takeoff_sent = True
                state["status"] = "Takeoff command sent"
                record("takeoff_request", altitude_m=args.takeoff_altitude)
            if (
                pending_takeoff
                and takeoff_sent
                and state["altitude_m"] >= 0.95 * args.takeoff_altitude
            ):
                pending_takeoff = False
                set_mode("LOITER")
                state["status"] = (
                    "Takeoff complete; LOITER active — raise throttle above center "
                    "to avoid a descent command"
                )

            keys = pygame.key.get_pressed()
            roll = float(keys[pygame.K_d]) - float(keys[pygame.K_a])
            pitch = float(keys[pygame.K_w]) - float(keys[pygame.K_s])
            yaw = float(keys[pygame.K_e]) - float(keys[pygame.K_q])
            throttle = float(keys[pygame.K_UP])
            if keys[pygame.K_DOWN]:
                throttle = 0.0

            if joystick:
                axes = [joystick.get_axis(i) for i in range(joystick.get_numaxes())]
                # SDL Xbox layout: left stick yaw/throttle, right stick roll/pitch.
                if len(axes) >= 5:
                    yaw += shape_axis(axes[0])
                    throttle = max(throttle, throttle_from_axis(axes[1]))
                    roll += shape_axis(axes[3])
                    pitch += shape_axis(-axes[4])
                buttons = [joystick.get_button(i) for i in range(joystick.get_numbuttons())]
                if len(previous_buttons) == len(buttons):
                    button_actions = {0: "arm", 1: "LAND", 2: "RTL", 3: "takeoff", 6: "disarm", 7: "LOITER"}
                    for index, name in button_actions.items():
                        if index < len(buttons) and buttons[index] and not previous_buttons[index]:
                            action(name)
                previous_buttons = buttons
                hat = joystick.get_hat(0) if joystick.get_numhats() else (0, 0)
                left_bumper = len(buttons) > 4 and bool(buttons[4])
                mode_chord = (left_bumper, hat)
                if (
                    left_bumper
                    and hat in XBOX_MODE_HAT
                    and mode_chord != previous_mode_chord
                ):
                    action(XBOX_MODE_HAT[hat])
                previous_mode_chord = mode_chord

            roll, pitch, yaw = map(clamp, (roll, pitch, yaw))
            throttle = clamp(throttle, 0.0, 1.0)
            output_throttle_pwm = 1000 if not state["armed"] else throttle_pwm(throttle)
            master.mav.rc_channels_override_send(
                master.target_system,
                master.target_component,
                pwm(roll),
                pwm(pitch, reverse=True),
                output_throttle_pwm,
                pwm(yaw),
                IGNORE,
                IGNORE,
                IGNORE,
                IGNORE,
            )
            if now >= next_gcs_heartbeat:
                master.mav.heartbeat_send(
                    mavutil.mavlink.MAV_TYPE_GCS,
                    mavutil.mavlink.MAV_AUTOPILOT_INVALID,
                    0,
                    0,
                    0,
                )
                next_gcs_heartbeat = now + 0.5

            screen.fill((18, 22, 28))
            title = font.render("ICARUS MANUAL PILOT — LOCAL SIMULATION ONLY", True, (112, 210, 255))
            screen.blit(title, (24, 20))
            controller_name = joystick.get_name() if joystick else "keyboard only"
            lines = [
                f"Mode: {state['mode']}    Armed: {state['armed']}    Altitude: {state['altitude_m']:.2f} m",
                f"Controller: {controller_name}",
                f"Navigation: GPS fix {state['gps_fix']}    Local position: {state['local_position']}",
                f"Axes  roll {roll:+.2f}  pitch {pitch:+.2f}  yaw {yaw:+.2f}  throttle {throttle * 100:5.1f}%",
                "Keyboard: W/S pitch | A/D roll | Q/E yaw | Up throttle | Down zero",
                "Enter arm | T takeoff | H hold | L land | R RTL | Backspace disarm",
                "Xbox: left stick yaw/throttle | right stick roll/pitch",
                "A arm | Y takeoff | Start hold | B land | X RTL | Back disarm",
                "Modes: hold LB + D-pad  left STABILIZE | down ALT_HOLD",
                "                         up LOITER | right ACRO",
                "WARNING: releasing the spring-centered throttle commands 0%",
                "Esc/window close: LAND if armed, then exit",
                f"Status: {state['status']}",
                f"Log: {log_path}",
            ]
            for index, line in enumerate(lines):
                color = (230, 235, 240) if index < 8 else (255, 205, 110)
                screen.blit(small.render(line, True, color), (24, 70 + index * 33))
            pygame.display.flip()
            clock.tick(20)
    finally:
        if state["armed"]:
            set_mode("LAND")
            state["status"] = "LAND requested on manual-client exit"
            record("automatic_land_on_exit")
            deadline = time.monotonic() + 70
            while state["armed"] and time.monotonic() < deadline:
                master.mav.rc_channels_override_send(
                    master.target_system,
                    master.target_component,
                    1500,
                    1500,
                    1000,
                    1500,
                    IGNORE,
                    IGNORE,
                    IGNORE,
                    IGNORE,
                )
                message = master.recv_match(type="HEARTBEAT", blocking=True, timeout=0.2)
                if message:
                    state["armed"] = bool(
                        message.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED
                    )
        master.mav.rc_channels_override_send(
            master.target_system,
            master.target_component,
            RELEASE,
            RELEASE,
            RELEASE,
            RELEASE,
            IGNORE,
            IGNORE,
            IGNORE,
            IGNORE,
        )
        record("disconnected", armed=state["armed"])
        log.close()
        master.close()
        pygame.quit()


if __name__ == "__main__":
    main()

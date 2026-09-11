#!/usr/bin/env python3
"""Deterministic SITL flight check with bounded waits and recorded telemetry."""

import argparse
import itertools
import json
import math
import re
import subprocess
import time
from pathlib import Path

from pymavlink import mavutil


def run(args):
    master = mavutil.mavlink_connection("tcp:127.0.0.1:5760", source_system=255)
    state = {}
    samples = []
    status = []
    parameters = {}
    next_heartbeat = 0
    began = time.monotonic()
    result = {"status": "failed", "altitude_target_m": args.altitude}
    log = (args.directory / "telemetry.jsonl").open("w")

    def pump():
        nonlocal next_heartbeat
        if time.monotonic() >= next_heartbeat:
            master.mav.heartbeat_send(
                mavutil.mavlink.MAV_TYPE_GCS,
                mavutil.mavlink.MAV_AUTOPILOT_INVALID,
                0,
                0,
                0,
            )
            next_heartbeat = time.monotonic() + 0.5
        msg = master.recv_match(blocking=True, timeout=0.1)
        if msg:
            state[msg.get_type()] = msg
            if msg.get_type() == "PARAM_VALUE":
                parameters[msg.param_id] = msg.param_value
            if msg.get_type() == "STATUSTEXT":
                status.append(str(msg.text))
                print(msg.text, flush=True)
            if msg.get_type() in (
                "ATTITUDE",
                "LOCAL_POSITION_NED",
                "GLOBAL_POSITION_INT",
                "SERVO_OUTPUT_RAW",
                "COMMAND_ACK",
            ):
                record = msg.to_dict()
                record["wall_elapsed_s"] = time.monotonic() - began
                log.write(json.dumps(record) + "\n")
        return msg

    def wait(predicate, seconds, label):
        deadline = time.monotonic() + seconds
        while time.monotonic() < deadline:
            pump()
            if predicate():
                print("OK:", label, flush=True)
                return
        raise TimeoutError(label + "; recent status: " + "; ".join(status[-5:]))

    def command(cmd, *params):
        master.mav.command_long_send(
            master.target_system,
            master.target_component,
            cmd,
            0,
            *(list(params) + [0] * (7 - len(params))),
        )

    def mode(name):
        number = master.mode_mapping()[name]
        master.mav.set_mode_send(
            master.target_system,
            mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
            number,
        )
        wait(
            lambda: state.get("HEARTBEAT") and state["HEARTBEAT"].custom_mode == number,
            10,
            name,
        )

    def altitude():
        return (
            state["GLOBAL_POSITION_INT"].relative_alt / 1000
            if "GLOBAL_POSITION_INT" in state
            else 0
        )

    try:
        wait(lambda: "HEARTBEAT" in state, 30, "heartbeat")
        hb = state["HEARTBEAT"]
        master.target_system = hb.get_srcSystem()
        master.target_component = hb.get_srcComponent()
        # Read back every explicit setting before arming. Parameter names and
        # units differ between ArduPilot releases; ignored defaults must fail.
        profile = (
            Path(__file__).resolve().parents[2]
            / "simulation/parameters/mark4_v2_base.parm"
        )
        requested = {}
        for line in profile.read_text().splitlines():
            fields = line.split("#", 1)[0].split()
            if fields:
                requested[fields[0]] = float(fields[1])
        for name in requested:
            for attempt in range(3):
                master.mav.param_request_read_send(
                    master.target_system, master.target_component, name.encode(), -1
                )
                # Allow bounded startup delays without skipping any readback.
                until = time.monotonic() + 5
                while name not in parameters and time.monotonic() < until:
                    pump()
                if name in parameters:
                    break
            if name not in parameters:
                raise RuntimeError("Parameter unavailable: " + name)
        print("OK: all parameters read back", flush=True)
        mismatches = {
            name: (expected, parameters[name])
            for name, expected in requested.items()
            if not math.isclose(expected, parameters[name], rel_tol=1e-5, abs_tol=1e-5)
        }
        if mismatches:
            raise RuntimeError("Parameter mismatch: " + str(mismatches))
        (args.directory / "parameters_verified.json").write_text(
            json.dumps(parameters, indent=2) + "\n"
        )
        master.mav.request_data_stream_send(
            master.target_system,
            master.target_component,
            mavutil.mavlink.MAV_DATA_STREAM_ALL,
            20,
            1,
        )
        wait(
            lambda: (
                "GPS_RAW_INT" in state
                and state["GPS_RAW_INT"].fix_type >= 3
                and "LOCAL_POSITION_NED" in state
            ),
            65,
            "navigation",
        )
        if args.preflight_only:
            result["status"] = "preflight_passed"
            return 0
        mode("GUIDED")
        arm_deadline = time.monotonic() + 45
        next_arm = 0
        while not master.motors_armed():
            if time.monotonic() > arm_deadline:
                raise RuntimeError("Arming refused: " + "; ".join(status[-8:]))
            if time.monotonic() > next_arm:
                command(mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 1)
                next_arm = time.monotonic() + 3
            pump()
        origin = state["LOCAL_POSITION_NED"]
        ox, oy = origin.x, origin.y
        command(mavutil.mavlink.MAV_CMD_NAV_TAKEOFF, 0, 0, 0, 0, 0, 0, args.altitude)
        wait(lambda: altitude() >= args.altitude * 0.95, 50, "takeoff")
        # Give the climb controller time to settle before measuring hover.
        settle = time.monotonic() + 3
        while time.monotonic() < settle:
            pump()
        deadline = time.monotonic() + 10
        last_boot = -1
        while time.monotonic() < deadline:
            pump()
            pos = state.get("LOCAL_POSITION_NED")
            att = state.get("ATTITUDE")
            if pos and att and pos.time_boot_ms != last_boot:
                last_boot = pos.time_boot_ms
                sample = {
                    "altitude_m": altitude(),
                    "drift_m": math.hypot(pos.x - ox, pos.y - oy),
                    "tilt_deg": math.degrees(max(abs(att.roll), abs(att.pitch))),
                }
                if "SERVO_OUTPUT_RAW" in state:
                    sample["motor_pwm"] = [
                        getattr(state["SERVO_OUTPUT_RAW"], f"servo{i}_raw")
                        for i in range(1, 5)
                    ]
                samples.append(sample)
                if (
                    sample["tilt_deg"] > 35
                    or sample["drift_m"] > 5
                    or sample["altitude_m"] > args.altitude + 2
                ):
                    raise RuntimeError("Flight envelope exceeded: " + str(sample))
        if len(samples) < 40:
            raise RuntimeError("Insufficient fresh hover telemetry")
        result["hover"] = {
            "samples": len(samples),
            "altitude_min_m": min(s["altitude_m"] for s in samples),
            "altitude_max_m": max(s["altitude_m"] for s in samples),
            "max_drift_m": max(s["drift_m"] for s in samples),
            "max_tilt_deg": max(s["tilt_deg"] for s in samples),
        }
        if args.world == "compact_flight":
            pwm_rows = [s["motor_pwm"] for s in samples if "motor_pwm" in s]
            if len(pwm_rows) < 40:
                raise RuntimeError(
                    "Insufficient motor telemetry for hover-thrust validation"
                )

            def thrust(pwm):
                throttle = (pwm - 1000) / 1000
                table = [(0, 0), (0.4, 0.761), (0.5, 1.336), (0.6, 1.871), (0.7, 2.451)]
                for (lo, lf), (hi, hf) in itertools.pairwise(table):
                    if throttle <= hi:
                        return lf + (hf - lf) * (throttle - lo) / (hi - lo)
                raise RuntimeError("Motor exceeded 70% operating limit")

            mean_thrust = sum(sum(thrust(p) for p in row) for row in pwm_rows) / len(
                pwm_rows
            )
            result["hover"]["mean_motor_pwm"] = sum(sum(row) for row in pwm_rows) / (
                4 * len(pwm_rows)
            )
            result["hover"]["curve_predicted_total_thrust_kgf"] = mean_thrust
            result["hover"]["thrust_weight_relative_error"] = (
                abs(mean_thrust - 4.343) / 4.343
            )
            if result["hover"]["thrust_weight_relative_error"] > 0.05:
                raise RuntimeError(
                    "Hover thrust inconsistent with reference curve and mass"
                )
            (args.directory / "hover_samples.json").write_text(
                json.dumps(samples) + "\n"
            )
        if any(
            abs(s["altitude_m"] - args.altitude) > args.max_hover_altitude_error
            or s["drift_m"] > args.max_hover_drift
            or s["tilt_deg"] > args.max_hover_tilt
            for s in samples
        ):
            raise RuntimeError("Hover acceptance gate failed: " + str(result["hover"]))
        if args.pause_at_hover:
            result["status"] = "hover_ready"
            return 0
        if args.direction_check:
            result["direction_checks"] = []

            def ground_truth():
                reply = subprocess.run(
                    [
                        "gz",
                        "topic",
                        "-e",
                        "-n",
                        "1",
                        "-t",
                        f"/world/{args.world}/dynamic_pose/info",
                    ],
                    capture_output=True,
                    text=True,
                    check=True,
                    timeout=6,
                )
                block = re.search(
                    rf'name: "{re.escape(args.model)}".*?position \{{(.*?)\}}.*?orientation \{{(.*?)\}}',
                    reply.stdout,
                    re.DOTALL,
                )
                if not block:
                    raise RuntimeError("Ground-truth pose unavailable")

                def fields(text):
                    return {
                        k: float(v)
                        for k, v in re.findall(r"([xyzw]):\s*([-+0-9.eE]+)", text)
                    }

                return fields(block[1]), fields(block[2])

            for north, east in ((1, 0), (1, 1), (0, 0)):
                master.mav.set_position_target_local_ned_send(
                    0,
                    master.target_system,
                    master.target_component,
                    mavutil.mavlink.MAV_FRAME_LOCAL_NED,
                    0x0DF8,
                    ox + north,
                    oy + east,
                    -args.altitude,
                    0,
                    0,
                    0,
                    0,
                    0,
                    0,
                    0,
                    0,
                )
                wait(
                    lambda north=north, east=east: (
                        math.hypot(
                            state["LOCAL_POSITION_NED"].x - ox - north,
                            state["LOCAL_POSITION_NED"].y - oy - east,
                        )
                        < 0.15
                    ),
                    25,
                    f"waypoint N={north} E={east}",
                )
                pos = state["LOCAL_POSITION_NED"]
                truth, _ = ground_truth()
                if (
                    abs(truth.get("y", 0) - north) > 0.3
                    or abs(truth.get("x", 0) - east) > 0.3
                    or abs(truth.get("z", 0) - (args.altitude + args.ground_height))
                    > 0.4
                ):
                    raise RuntimeError(
                        "Gazebo ENU / MAVLink NED mismatch: " + str(truth)
                    )
                result["direction_checks"].append(
                    {
                        "north_target": north,
                        "east_target": east,
                        "north": pos.x - ox,
                        "east": pos.y - oy,
                        "gazebo_enu": truth,
                    }
                )
            command(mavutil.mavlink.MAV_CMD_CONDITION_YAW, 135, 20, 1, 0)
            wait(
                lambda: abs(math.degrees(state["ATTITUDE"].yaw) - 135) < 3,
                20,
                "yaw 135 degrees",
            )
            _, q = ground_truth()
            yaw = math.degrees(
                math.atan2(
                    2 * (q.get("w", 1) * q.get("z", 0) + q.get("x", 0) * q.get("y", 0)),
                    1 - 2 * (q.get("y", 0) ** 2 + q.get("z", 0) ** 2),
                )
            )
            if abs(yaw + 45) > 5:
                raise RuntimeError("Gazebo yaw transform mismatch: " + str(yaw))
            result["gazebo_yaw_deg"] = yaw
        mode("LAND")
        wait(lambda: not master.motors_armed(), 70, "land and disarm")
        if abs(altitude()) > 0.3:
            raise RuntimeError("Disarmed above expected ground altitude")
        if args.direction_check:
            rest = []
            for _ in range(3):
                truth, _ = ground_truth()
                rest.append(truth.get("z", 0))
                until = time.monotonic() + 1
                while time.monotonic() < until:
                    pump()
            if max(rest) - min(rest) > 0.02 or any(
                abs(z - args.ground_height) > 0.03 for z in rest
            ):
                raise RuntimeError("Landing contact/rest gate failed: " + str(rest))
            result["landing_ground_truth_z_m"] = rest
        result.update(status="passed", landed_altitude_m=altitude())
        return 0
    except Exception as error:  # noqa: BLE001 - persist failure and request LAND before cleanup
        result["error"] = str(error)
        print("FAIL:", error, flush=True)
        if master.motors_armed():
            master.mav.set_mode_send(
                master.target_system,
                mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
                master.mode_mapping()["LAND"],
            )
        return 1
    finally:
        result["duration_s"] = time.monotonic() - began
        result["recent_status"] = status[-20:]
        (args.directory / "result.json").write_text(json.dumps(result, indent=2) + "\n")
        log.close()
        master.close()


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("--directory", required=True, type=Path)
    parser.add_argument("--altitude", default=3.0, type=float)
    parser.add_argument("--pause-at-hover", action="store_true")
    parser.add_argument("--direction-check", action="store_true")
    parser.add_argument("--world", default="mark4_validation")
    parser.add_argument("--model", default="icarus_mark4_v2_10")
    parser.add_argument("--ground-height", default=0.156, type=float)
    parser.add_argument("--preflight-only", action="store_true")
    parser.add_argument("--max-hover-drift", default=1.0, type=float)
    parser.add_argument("--max-hover-tilt", default=10.0, type=float)
    parser.add_argument("--max-hover-altitude-error", default=0.5, type=float)
    raise SystemExit(run(parser.parse_args()))

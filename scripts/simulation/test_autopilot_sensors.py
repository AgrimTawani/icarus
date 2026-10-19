#!/usr/bin/env python3
"""Opt-in live Gazebo acceptance: takeoff, XY/yaw, sensor loss/recovery, landing.

Run using third_party/ardupilot/.venv/bin/python after start-sim. Only the
verified local Gazebo session is accepted. Never uses a physical endpoint.
Ground truth is used for test assertions only, never forwarded to the EKF.
"""

import argparse
import json
import math
import os
import time
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--fly",
        action="store_true",
        required=True,
        help="Explicitly authorize this local simulation flight",
    )
    parser.add_argument(
        "--faults", action="store_true", help="Drop each sensor briefly while hovering"
    )
    parser.add_argument(
        "--proximity-stop",
        action="store_true",
        help="Test velocity-mode braking in edge-vision",
    )
    args = parser.parse_args()
    session = json.loads((ROOT / "logs/simulation/active_session.json").read_text())
    if (
        session.get("status") != "ready"
        or session.get("model") != "icarus_compact"
        or session.get("mavlink_endpoint") != "tcp:127.0.0.1:5760"
        or session.get("json_physics_endpoint") != "udp:127.0.0.1:9002"
    ):
        raise RuntimeError("requires a ready local Gazebo SITL session")
    os.kill(session["launcher_pid"], 0)
    if args.proximity_stop and session.get("scenario") != "edge_people_building":
        raise RuntimeError(
            "proximity-stop requires the reviewed edge-vision test scene"
        )
    os.environ["GZ_IP"] = "127.0.0.1"
    os.environ["GZ_PARTITION"] = session["partition"]
    os.environ["MAVLINK20"] = "1"
    from gz.msgs10.pose_v_pb2 import Pose_V
    from gz.msgs10.stringmsg_pb2 import StringMsg
    from gz.transport13 import Node
    from pymavlink import mavutil

    folder = Path(session["run_directory"]) / (
        "sensor_acceptance_" + time.strftime("%Y%m%dT%H%M%S")
    )
    folder.mkdir()
    health_path = Path(session["run_directory"]) / "autopilot_sensors.json"
    initial = json.loads(health_path.read_text())
    if (
        initial["status"] != "ready"
        or time.monotonic() - initial["updated_monotonic_s"] > 2
    ):
        raise RuntimeError("sensor bridge not ready")
    node, truth = Node(), {}

    def pose(message):
        for entry in message.pose:
            if entry.name == session["model"]:
                truth.update(
                    x=entry.position.x,
                    y=entry.position.y,
                    z=entry.position.z,
                    received=time.monotonic(),
                )

    assert node.subscribe(
        Pose_V, "/world/" + session["world"] + "/dynamic_pose/info", pose
    )
    publisher = node.advertise("/icarus/test/sensor_fault", StringMsg)
    link = mavutil.mavlink_connection(session["mavlink_endpoint"], source_system=255)
    state, report, rows = {}, {"status": "failed", "checks": []}, []
    next_hb, next_sample = 0.0, 0.0
    active_fault = None
    armed_by_test = False
    bit_names = {
        "lidar": "MAV_SYS_STATUS_SENSOR_PROXIMITY",
        "range": "MAV_SYS_STATUS_SENSOR_LASER_POSITION",
        "optical_flow": "MAV_SYS_STATUS_SENSOR_OPTICAL_FLOW",
    }

    def fault(channel, mode):
        publisher.publish(
            StringMsg(data=json.dumps({"channel": channel, "mode": mode}))
        )

    def pump():
        nonlocal next_hb, next_sample
        now = time.monotonic()
        if now >= next_hb:
            link.mav.heartbeat_send(6, 8, 0, 0, 0)
            next_hb = now + 0.5
        msg = link.recv_match(blocking=True, timeout=0.05)
        if msg and msg.get_srcComponent() == 1:
            key = msg.get_type()
            if key == "DISTANCE_SENSOR":
                key += "_" + str(msg.orientation)
            state[key] = msg
            if key == "STATUSTEXT":
                print(msg.text, flush=True)
        if now >= next_sample:
            row = {"monotonic_s": now, "truth": dict(truth)}
            for key in (
                "ATTITUDE",
                "LOCAL_POSITION_NED",
                "OPTICAL_FLOW",
                "DISTANCE_SENSOR_25",
                "SYS_STATUS",
            ):
                if key in state:
                    row[key] = state[key].to_dict()
            rows.append(row)
            next_sample = now + 0.1

    def wait(predicate, seconds, label):
        deadline = time.monotonic() + seconds
        while time.monotonic() < deadline:
            pump()
            if predicate():
                report["checks"].append(label)
                print("PASS:", label, flush=True)
                return
        raise TimeoutError(label)

    def dwell(seconds):
        end = time.monotonic() + seconds
        while time.monotonic() < end:
            pump()

    def mode(name):
        number = link.mode_mapping()[name]
        link.mav.set_mode_send(1, 1, number)
        wait(lambda: state["HEARTBEAT"].custom_mode == number, 8, "mode " + name)

    def command(cmd, *params):
        link.mav.command_long_send(
            1, 1, cmd, 0, *(list(params) + [0] * (7 - len(params)))
        )

    def healthy(channel):
        value = state.get("SYS_STATUS")
        bit = getattr(mavutil.mavlink, bit_names[channel])
        return value is not None and bool(value.onboard_control_sensors_health & bit)

    def altitude():
        return (
            state.get("GLOBAL_POSITION_INT").relative_alt / 1000
            if "GLOBAL_POSITION_INT" in state
            else 0
        )

    try:
        wait(lambda: "HEARTBEAT" in state, 10, "autopilot heartbeat")
        if link.motors_armed():
            raise RuntimeError("refusing to take over an already armed mission")
        link.mav.request_data_stream_send(
            1, 1, mavutil.mavlink.MAV_DATA_STREAM_ALL, 20, 1
        )
        wait(
            lambda: (
                "LOCAL_POSITION_NED" in state
                and time.monotonic() - truth.get("received", 0) < 1
            ),
            15,
            "navigation and independent Gazebo truth",
        )
        mode("GUIDED")
        dwell(2)  # allow GCS failsafe from a previous operator to clear
        armed_by_test = True
        arm_deadline, next_arm = time.monotonic() + 20, 0
        while not link.motors_armed() and time.monotonic() < arm_deadline:
            if time.monotonic() >= next_arm:
                command(mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 1)
                next_arm = time.monotonic() + 3
            pump()
        if not link.motors_armed():
            raise RuntimeError("arming refused; no checks bypassed")
        report["checks"].append("armed without bypass")
        ox, oy = state["LOCAL_POSITION_NED"].x, state["LOCAL_POSITION_NED"].y
        origin = dict(truth)
        command(mavutil.mavlink.MAV_CMD_NAV_TAKEOFF, 0, 0, 0, 0, 0, 0, 3)
        wait(lambda: altitude() > 2.85, 45, "takeoff 3m")
        dwell(4)
        wait(
            lambda: all(healthy(k) for k in bit_names),
            10,
            "all three sensors healthy in ArduPilot",
        )
        if not json.loads(health_path.read_text()).get("flow_fusion_verified"):
            raise AssertionError("ArduPilot has not confirmed optical-flow fusion")
        report["checks"].append("ArduPilot explicitly reported optical-flow fusion")
        # Local flat takeoff patch: truth is from Gazebo, not the EKF being tested.
        actual_range = state["DISTANCE_SENSOR_25"].current_distance / 100
        expected_range = truth["z"] - 0.1125
        if abs(actual_range - expected_range) > 0.2:
            raise AssertionError(
                f"range/truth mismatch: {actual_range} vs {expected_range}"
            )
        report["range_check"] = {
            "measured_m": actual_range,
            "expected_m": expected_range,
        }
        report["checks"].append("downward range matches Gazebo height within 0.2m")
        for north, east in ((1.5, 0), (1.5, 1.5), (0, 0)):
            link.mav.set_position_target_local_ned_send(
                0, 1, 1, 1, 0x0DF8, ox + north, oy + east, -3, 0, 0, 0, 0, 0, 0, 0, 0
            )
            wait(
                lambda north=north, east=east: (
                    math.hypot(
                        state["LOCAL_POSITION_NED"].x - ox - north,
                        state["LOCAL_POSITION_NED"].y - oy - east,
                    )
                    < 0.2
                ),
                25,
                f"move N={north} E={east}",
            )
            dwell(2)
            if (
                math.hypot(
                    truth["y"] - origin["y"] - north, truth["x"] - origin["x"] - east
                )
                > 0.4
            ):
                raise AssertionError("motion differs from independent Gazebo truth")
        yaw_target = state["ATTITUDE"].yaw + math.pi / 2
        command(mavutil.mavlink.MAV_CMD_CONDITION_YAW, 90, 15, 1, 1)
        wait(
            lambda: (
                abs(
                    math.atan2(
                        math.sin(state["ATTITUDE"].yaw - yaw_target),
                        math.cos(state["ATTITUDE"].yaw - yaw_target),
                    )
                )
                < math.radians(5)
            ),
            20,
            "relative yaw 90deg",
        )
        dwell(3)
        if args.faults:
            for channel in bit_names:
                active_fault = channel
                fault(channel, "drop")
                wait(
                    lambda channel=channel: not healthy(channel),
                    8,
                    channel + " loss visible to autopilot",
                )
                dwell(1)
                if (
                    abs(altitude() - 3) > 0.5
                    or math.hypot(
                        state["LOCAL_POSITION_NED"].x - ox,
                        state["LOCAL_POSITION_NED"].y - oy,
                    )
                    > 0.8
                ):
                    raise AssertionError("hover unstable during sensor loss")
                fault(channel, "normal")
                active_fault = None
                wait(
                    lambda channel=channel: healthy(channel), 8, channel + " recovered"
                )
        if args.proximity_stop:
            deadline, next_velocity, stationary_since = time.monotonic() + 45, 0, None
            minimum_clearance = math.inf
            while time.monotonic() < deadline:
                pump()
                now = time.monotonic()
                if now >= next_velocity:
                    # Position ignored, velocity NED only. This pinned ArduPilot
                    # Guided path invokes simple proximity avoidance explicitly.
                    link.mav.set_position_target_local_ned_send(
                        0, 1, 1, 1, 0x0DC7, 0, 0, 0, 0.6, 0, 0, 0, 0, 0, 0, 0
                    )
                    next_velocity = now + 0.1
                ranges = [
                    m.current_distance / 100
                    for k, m in state.items()
                    if k.startswith("DISTANCE_SENSOR_")
                    and m.orientation <= 7
                    and 0 < m.current_distance <= m.max_distance
                ]
                clearance = min(ranges, default=40)
                minimum_clearance = min(minimum_clearance, clearance)
                if clearance < 1.5 or truth["y"] > 13 or abs(truth["x"]) > 6:
                    raise AssertionError("proximity test safety envelope exceeded")
                velocity = state["LOCAL_POSITION_NED"]
                stopped = (
                    clearance < 2.6 and math.hypot(velocity.vx, velocity.vy) < 0.15
                )
                stationary_since = (stationary_since or now) if stopped else None
                if stationary_since and now - stationary_since > 2:
                    report["proximity_stop"] = {
                        "minimum_clearance_m": minimum_clearance,
                        "position_truth": dict(truth),
                    }
                    report["checks"].append(
                        "ArduPilot brakes velocity command using real LiDAR"
                    )
                    break
            else:
                raise TimeoutError("proximity braking not demonstrated")
            link.mav.set_position_target_local_ned_send(
                0, 1, 1, 1, 0x0DC7, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0
            )
            dwell(2)
        mode("LAND")
        wait(lambda: not link.motors_armed(), 70, "landed and disarmed")
        armed_by_test = False
        report["status"] = "passed"
    except BaseException as error:
        report["error"] = str(error)
        raise
    finally:
        if active_fault:
            fault(active_fault, "normal")
        if armed_by_test:
            try:
                mode("LAND")
                wait(lambda: not link.motors_armed(), 70, "recovery landed")
            except Exception as error:  # noqa: BLE001 - preserve recovery failure evidence
                report["recovery_error"] = str(error)
        report["samples"] = len(rows)
        (folder / "result.json").write_text(json.dumps(report, indent=2) + "\n")
        (folder / "telemetry.jsonl").write_text(
            "".join(json.dumps(row) + "\n" for row in rows)
        )
        print("Evidence:", folder, flush=True)
        link.close()


if __name__ == "__main__":
    main()

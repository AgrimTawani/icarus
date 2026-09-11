#!/usr/bin/python3
"""Phase 3.2: isolated Gazebo motor mapping, force, spin and response tests.

Uses Ubuntu's installed Gazebo Python bindings; run with /usr/bin/python3.
Each case starts a fresh paused server in its own transport partition.
The generated bench world has zero gravity and no ground, isolating motor forces.
"""

import copy
import json
import math
import os
import signal
import subprocess
import sys
import time
import uuid
import xml.etree.ElementTree as ET
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
MODEL = Path(
    os.environ.get(
        "ICARUS_MOTOR_TEST_MODEL", str(ROOT / "simulation/models/mark4_v2/model.sdf")
    )
)


def interrupted(signum, _frame):
    raise SystemExit(128 + signum)


def stop(process):
    # gz is a wrapper: stop its whole owned process group, with bounded waits.
    for sig in (signal.SIGINT, signal.SIGTERM, signal.SIGKILL):
        try:
            os.killpg(process.pid, sig)
        except ProcessLookupError:
            break
        try:
            process.wait(timeout=3)
        except subprocess.TimeoutExpired:
            continue
        try:
            os.killpg(process.pid, 0)
        except ProcessLookupError:
            break


def case(index, directory):
    os.environ["GZ_PARTITION"] = "icarus_motor_" + uuid.uuid4().hex
    os.environ["GZ_SIM_RESOURCE_PATH"] = str(ROOT / "simulation/models")
    from gz.msgs10.actuators_pb2 import Actuators
    from gz.msgs10.model_pb2 import Model
    from gz.msgs10.pose_v_pb2 import Pose_V
    from gz.transport13 import Node

    sdf = ET.Element("sdf", version="1.9")
    world = ET.SubElement(sdf, "world", name="motor_bench")
    ET.SubElement(world, "gravity").text = "0 0 0"
    physics = ET.SubElement(world, "physics", name="bench", type="ignored")
    ET.SubElement(physics, "max_step_size").text = "0.001"
    ET.SubElement(physics, "real_time_factor").text = "1"
    for name in ("Physics", "UserCommands"):
        filename = "physics" if name == "Physics" else "user-commands"
        ET.SubElement(
            world,
            "plugin",
            filename=f"gz-sim-{filename}-system",
            name=f"gz::sim::systems::{name}",
        )
    model = copy.deepcopy(ET.parse(MODEL).getroot().find("model"))
    model_name = model.get("name")
    # Direct-actuator bench excludes SITL adapters; these are tested by flight.
    for plugin in list(model.findall("plugin")):
        if plugin.get("filename") in ("ArduPilotPlugin", "IcarusMotorBridge"):
            model.remove(plugin)
    world.append(model)
    joint_pub = ET.SubElement(
        model,
        "plugin",
        filename="gz-sim-joint-state-publisher-system",
        name="gz::sim::systems::JointStatePublisher",
    )
    ET.SubElement(joint_pub, "topic").text = "/bench/joints"
    poses = ET.SubElement(
        model,
        "plugin",
        filename="gz-sim-pose-publisher-system",
        name="gz::sim::systems::PosePublisher",
    )
    for key, value in {
        "publish_model_pose": "true",
        "publish_link_pose": "true",
        "use_pose_vector_msg": "true",
        "update_frequency": "1000",
    }.items():
        ET.SubElement(poses, key).text = value
    world_path = directory / "world.sdf"
    ET.ElementTree(sdf).write(world_path, encoding="unicode")
    node = Node()
    samples = {"joints": None, "pose": None}

    def on_joints(msg):
        samples["joints"] = msg

    def on_pose(msg):
        samples["pose"] = msg

    node.subscribe(Model, "/bench/joints", on_joints)
    node.subscribe(Pose_V, f"/model/{model_name}/pose", on_pose)
    publisher = node.advertise("/icarus/vehicle_01/actuators/motor_speed", Actuators)

    def stamp(msg):
        if isinstance(msg, Pose_V) and msg.pose:
            msg = msg.pose[0]
        return msg.header.stamp.sec + msg.header.stamp.nsec * 1e-9 if msg else -1

    def wait_for(predicate, description):
        deadline = time.monotonic() + 15
        while not predicate():
            if time.monotonic() > deadline or process.poll() is not None:
                raise RuntimeError(
                    f"Timeout/server exit: {description}; stamps="
                    f"{[(k, stamp(v)) for k, v in samples.items()]}; topics={node.topic_list()}"
                )
            time.sleep(0.02)

    with (directory / "gazebo.log").open("w") as logfile:
        process = subprocess.Popen(
            ["gz", "sim", "-s", "-v", "2", str(world_path)],
            stdout=logfile,
            stderr=subprocess.STDOUT,
            start_new_session=True,
        )
        try:
            wait_for(
                lambda: "/world/motor_bench/control" in node.service_list(),
                "control discovery",
            )
            time.sleep(0.5)
            sim_time = 0.0

            def step(count):
                nonlocal sim_time
                # The installed Python binding's blocking request can starve
                # subscription callbacks; use the CLI for this separate RPC.
                reply = subprocess.run(
                    [
                        "gz",
                        "service",
                        "-s",
                        "/world/motor_bench/control",
                        "--reqtype",
                        "gz.msgs.WorldControl",
                        "--reptype",
                        "gz.msgs.Boolean",
                        "--timeout",
                        "5000",
                        "--req",
                        f"pause: true, multi_step: {count}",
                    ],
                    capture_output=True,
                    text=True,
                    check=True,
                    timeout=8,
                )
                if "data: true" not in reply.stdout:
                    raise RuntimeError(
                        f"World step rejected: {reply.stdout} {reply.stderr}"
                    )
                sim_time += count * 0.001
                wait_for(
                    lambda: (
                        min(stamp(samples["joints"]), stamp(samples["pose"]))
                        >= sim_time - 0.002
                    ),
                    f"telemetry for {sim_time:.3f} simulation seconds",
                )

            def command(values):
                msg = Actuators()
                msg.velocity.extend(values)
                for _ in range(5):
                    publisher.publish(msg)
                    time.sleep(0.04)

            def speeds():
                return {j.name: j.axis1.velocity for j in samples["joints"].joint}

            command([0] * 4)
            step(20)
            values = [300.0 if index in (i, 4) else 0.0 for i in range(4)]
            command(values)
            step(40)
            rise = speeds()
            step(160)
            measured = speeds()
            expected_signs = [1, 1, -1, -1]
            for i in range(4):
                name = f"rotor_{i + 1:02d}_joint"
                target = values[i] / 10 * expected_signs[i]
                assert abs(measured[name] - target) < 0.3, (name, measured, target)
                if values[i]:
                    assert 0.55 < abs(rise[name] / target) < 0.72, (
                        "rise response",
                        rise,
                    )
            pose = next(p for p in samples["pose"].pose if p.name == model_name)
            q = pose.orientation
            roll = math.atan2(
                2 * (q.w * q.x + q.y * q.z), 1 - 2 * (q.x * q.x + q.y * q.y)
            )
            pitch = math.asin(max(-1, min(1, 2 * (q.w * q.y - q.z * q.x))))
            yaw = math.atan2(
                2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z)
            )
            assert pose.position.z > 0.001, ("no upward thrust", pose)
            if index < 4:
                # tau = r cross F in FLU; yaw reaction opposes propeller spin.
                assert roll * [-1, 1, 1, -1][index] > 0.01, ("roll sign", roll)
                assert pitch * [-1, 1, -1, 1][index] > 0.01, ("pitch sign", pitch)
                assert yaw * -expected_signs[index] > 0.001, ("yaw sign", yaw)
            else:
                assert max(abs(roll), abs(pitch), abs(yaw)) < 0.005, (
                    "unbalanced collective",
                    roll,
                    pitch,
                    yaw,
                )
                # Independent integration of F=k*w^2, with 40 ms first-order rise.
                k = 4.648 * 9.80665 / (15606 * 2 * math.pi / 60) ** 2
                velocity = height = 0.0
                for n in range(200):
                    omega = 300 * (1 - math.exp(-n * 0.001 / 0.040))
                    velocity += 4 * k * omega**2 / 4.343 * 0.001
                    height += velocity * 0.001
                assert abs(pose.position.z - height) < 0.08 * height, (
                    "thrust magnitude",
                    pose.position.z,
                    height,
                )
            result = {
                "case": index,
                "joint_rad_s": measured,
                "rise_at_40ms_rad_s": rise,
                "height_m": pose.position.z,
                "roll_pitch_yaw_rad": [roll, pitch, yaw],
            }
            command([0] * 4)
            step(80)
            decay = speeds()
            for i in range(4):
                if values[i]:
                    ratio = abs(decay[f"rotor_{i + 1:02d}_joint"] / 30)
                    assert 0.33 < ratio < 0.41, ("decay response", decay)
            step(720)
            assert max(abs(v) for v in speeds().values()) < 0.01, "Motor failed to stop"
            if index == 4:
                physical_limit = 15606 * 2 * math.pi / 60
                command([physical_limit * 2] * 4)
                step(500)
                capped = speeds()
                for name, value in capped.items():
                    assert abs(abs(value) - physical_limit / 10) < 0.01, (
                        "speed ceiling",
                        name,
                        value,
                    )
                result["capped_joint_rad_s"] = capped
                command([0] * 4)
                step(1000)
                assert max(abs(v) for v in speeds().values()) < 0.01, (
                    "Capped motor failed to stop"
                )
            result["status"] = "PASS"
            (directory / "result.json").write_text(json.dumps(result, indent=2) + "\n")
            print(json.dumps(result), flush=True)
        finally:
            stop(process)


if __name__ == "__main__":
    if not __debug__:
        raise SystemExit("Run without -O: verification assertions must remain enabled")
    signal.signal(signal.SIGTERM, interrupted)
    signal.signal(signal.SIGINT, interrupted)
    if len(sys.argv) == 4 and sys.argv[1] == "--case":
        case(int(sys.argv[2]), Path(sys.argv[3]))
    else:
        directory = (
            ROOT
            / "logs/simulation"
            / (
                "mark4_motors_"
                + time.strftime("%Y%m%dT%H%M%S")
                + "_"
                + uuid.uuid4().hex[:6]
            )
        )
        directory.mkdir(parents=True)
        for index in range(5):
            subdir = directory / f"case_{index}"
            subdir.mkdir()
            # Worker operations have bounded timeouts and own their server
            # cleanup. Do not SIGKILL the worker while it owns a Gazebo group.
            subprocess.run(
                ["/usr/bin/python3", __file__, "--case", str(index), str(subdir)],
                check=True,
            )
        print("PASS: four individual motors and balanced collective; logs:", directory)

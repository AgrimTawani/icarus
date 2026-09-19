#!/usr/bin/env python3
"""Rigorous headless Phase 9 wind, clearance and fail-safe acceptance."""

import json
import math
import os
import sys
import threading
import time
from itertools import pairwise
from pathlib import Path

import grpc

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / "build/generated/python"))
sys.path.insert(0, str(Path(__file__).resolve().parent))
sys.path.append(str(ROOT / "scripts/simulation"))
sys.path.append("/usr/lib/python3/dist-packages")

from icarus.v1 import action_pb2, drone_api_pb2, drone_api_pb2_grpc, state_pb2
from run_mission import MissionClient
from scenario_config import load_scenario
from score_phase5_trajectory import score


def path_length(points):
    return sum(
        math.dist(
            [first["x_m"], first["y_m"], first["z_m"]],
            [second["x_m"], second["y_m"], second["z_m"]],
        )
        for first, second in pairwise(points)
    )


def main() -> int:
    session_path = ROOT / "logs/simulation/active_session.json"
    if not session_path.is_file():
        raise SystemExit("No active simulator; start the stress profile first")
    session = json.loads(session_path.read_text())
    if session.get("scenario") != "phase9_stress":
        raise SystemExit(
            "Phase 9 rigorous gate requires: "
            "./scripts/start-sim --profile simulation-perception-stress"
        )
    os.environ["GZ_PARTITION"] = session["partition"]

    from gz.msgs10.pose_v_pb2 import Pose_V
    from gz.msgs10.stringmsg_pb2 import StringMsg
    from gz.msgs10.vector3d_pb2 import Vector3d
    from gz.transport13 import Node

    _, scenario = load_scenario("phase9_stress")
    client = MissionClient("127.0.0.1:50051", "phase9_stress_acceptance")
    state_samples = []
    truth_samples = []
    wind_samples = []
    stop = threading.Event()
    sample_lock = threading.Lock()
    report = {"status": "failed", "scenario": "phase9_stress", "seed": 97}
    node = Node()
    fault_publisher = node.advertise("/icarus/test/sensor_fault", StringMsg)

    def truth_callback(message):
        for pose in message.pose:
            if pose.name == session["model"]:
                point = {
                    "t_s": time.monotonic(),
                    "x_m": pose.position.x,
                    "y_m": pose.position.y,
                    "z_m": pose.position.z,
                }
                with sample_lock:
                    if not truth_samples or point["t_s"] > truth_samples[-1]["t_s"]:
                        truth_samples.append(point)
                break

    def wind_callback(message):
        wind_samples.append(math.sqrt(message.x**2 + message.y**2 + message.z**2))

    if not node.subscribe(
        Pose_V, f"/world/{session['world']}/dynamic_pose/info", truth_callback
    ):
        raise SystemExit("Unable to subscribe to Gazebo ground-truth pose")
    if not node.subscribe(Vector3d, "/icarus/environment/wind", wind_callback):
        raise SystemExit("Unable to subscribe to simulated wind")

    def sample_state():
        while not stop.wait(0.1):
            try:
                state = client.state()
                state_samples.append(
                    {
                        "t_s": time.monotonic(),
                        "north_m": state.local_position_ned.north_m,
                        "east_m": state.local_position_ned.east_m,
                        "tilt_deg": math.degrees(
                            math.hypot(state.attitude.roll_rad, state.attitude.pitch_rad)
                        ),
                        "ground_speed_mps": state.ground_speed_mps,
                    }
                )
            except grpc.RpcError:
                return

    def inject_lidar(mode):
        fault_publisher.publish(
            StringMsg(data=json.dumps({"channel": "lidar", "mode": mode}))
        )

    sampler = None
    try:
        client.connect()
        perception_api = drone_api_pb2_grpc.PerceptionServiceStub(client.channel)
        perception = perception_api.GetPerception(
            drone_api_pb2.GetPerceptionRequest(
                session_id=client.session_id, vehicle_id="icarus-01"
            ),
            timeout=5,
        )
        if perception.overall != state_pb2.HEALTH_LEVEL_HEALTHY:
            raise RuntimeError("perception is not healthy before arming")
        client.acquire()
        sampler = threading.Thread(target=sample_state, daemon=True)
        sampler.start()
        arm = client.action_api.Arm(
            action_pb2.ArmRequest(context=client.context("phase9:arm")), timeout=5
        )
        client.wait_action(arm, 55)
        takeoff = client.action_api.Takeoff(
            action_pb2.TakeoffRequest(
                context=client.context("phase9:takeoff"),
                target_altitude_agl_m=3.0,
                limits=action_pb2.ActionLimits(execution_timeout_ms=60_000),
            ),
            timeout=5,
        )
        client.wait_action(takeoff, 70)
        start = client.state().local_position_ned
        with sample_lock:
            truth_start = len(truth_samples)
        destination = state_pb2.LocalPositionNed(
            origin_id="home",
            north_m=start.north_m + 12.0,
            east_m=start.east_m + 18.0,
            down_m=-3.0,
        )
        goto = client.action_api.Goto(
            action_pb2.GotoRequest(
                context=client.context("phase9:blocked-goto"),
                destination=state_pb2.Position(local_ned=destination),
                acceptance_radius_m=1.0,
                limits=action_pb2.ActionLimits(
                    maximum_ground_speed_mps=3.0,
                    execution_timeout_ms=120_000,
                    minimum_clearance_m=2.0,
                ),
            ),
            timeout=5,
        )
        terminal = client.wait_action(goto, 140)
        time.sleep(0.3)
        with sample_lock:
            flight_truth = list(truth_samples[truth_start:])
        if "safe detour" not in terminal.message:
            raise RuntimeError("blocked route completed without planner detour evidence")
        if len(flight_truth) < 50:
            raise RuntimeError("insufficient independent ground-truth trajectory samples")
        scored = score(scenario, flight_truth, "stress_blocked", 0.35)
        required_clearance = 1.5
        if scored["status"] != "passed":
            raise RuntimeError("ground-truth collision/goal scoring failed: " + str(scored))
        if scored["minimum_clearance_m"] < required_clearance:
            raise RuntimeError(
                f"minimum clearance {scored['minimum_clearance_m']:.3f} m "
                f"is below {required_clearance:.3f} m"
            )

        direct_length = math.dist(
            [flight_truth[0][key] for key in ("x_m", "y_m", "z_m")],
            [18.0, 12.0, 3.0],
        )
        flown_length = path_length(flight_truth)
        max_tilt = max(sample["tilt_deg"] for sample in state_samples)
        max_speed = max(sample["ground_speed_mps"] for sample in state_samples)
        if max_tilt > scenario["success"]["maximum_tilt_deg"]:
            raise RuntimeError(f"tilt limit exceeded: {max_tilt:.2f} deg")
        if max_speed > 5.0:
            raise RuntimeError(f"speed limit exceeded: {max_speed:.2f} m/s")
        if not wind_samples or max(wind_samples) < 4.0:
            raise RuntimeError("stress wind was not observed at the scoring process")

        hold = client.action_api.Hold(
            action_pb2.HoldRequest(
                context=client.context("phase9:dropout-hold"), duration_ms=15_000
            ),
            timeout=5,
        )

        def fault_cycle():
            time.sleep(0.4)
            inject_lidar("drop")
            time.sleep(1.4)
            inject_lidar("normal")

        fault_thread = threading.Thread(target=fault_cycle)
        fault_thread.start()
        dropout_terminal = None
        for status in client.action_api.WatchActionStatus(
            drone_api_pb2.WatchActionStatusRequest(
                session_id=client.session_id, action_id=hold.action_id
            ),
            timeout=20,
        ):
            print(
                f"{action_pb2.ActionType.Name(status.type)}: "
                f"{action_pb2.ActionState.Name(status.state)} — {status.message}",
                flush=True,
            )
            if status.state in {
                action_pb2.ACTION_STATE_ABORTED_BY_SAFETY,
                action_pb2.ACTION_STATE_FAILED,
                action_pb2.ACTION_STATE_SUCCEEDED,
            }:
                dropout_terminal = status
                break
        fault_thread.join(timeout=3)
        inject_lidar("normal")
        if (
            dropout_terminal is None
            or dropout_terminal.state != action_pb2.ACTION_STATE_ABORTED_BY_SAFETY
            or "perception stale" not in dropout_terminal.message
        ):
            raise RuntimeError("live LiDAR dropout did not produce safety abort/BRAKE")
        recovery_deadline = time.monotonic() + 5
        while time.monotonic() < recovery_deadline:
            perception = perception_api.GetPerception(
                drone_api_pb2.GetPerceptionRequest(
                    session_id=client.session_id, vehicle_id="icarus-01"
                ),
                timeout=2,
            )
            if perception.overall == state_pb2.HEALTH_LEVEL_HEALTHY:
                break
            time.sleep(0.1)
        else:
            raise RuntimeError("perception did not recover after injected dropout")

        land = client.action_api.Land(
            action_pb2.LandRequest(context=client.context("phase9:land")), timeout=5
        )
        client.wait_action(land, 100)
        report.update(
            status="passed",
            lidar_sequence=perception.sequence,
            planner_terminal_message=terminal.message,
            ground_truth=scored,
            required_clearance_m=required_clearance,
            ground_truth_samples=len(flight_truth),
            path_length_m=flown_length,
            direct_path_length_m=direct_length,
            path_efficiency=direct_length / flown_length,
            maximum_tilt_deg=max_tilt,
            maximum_ground_speed_mps=max_speed,
            wind_samples=len(wind_samples),
            maximum_observed_wind_mps=max(wind_samples),
            perception_dropout_terminal=action_pb2.ActionState.Name(
                dropout_terminal.state
            ),
            perception_recovered=True,
        )
        return 0
    except (grpc.RpcError, RuntimeError, TimeoutError) as error:
        report["error"] = str(error)
        return 1
    finally:
        inject_lidar("normal")
        stop.set()
        if sampler:
            sampler.join(timeout=2)
        if client.session_id:
            try:
                state = client.state()
                if state.armed and client.lease_id:
                    land = client.action_api.Land(
                        action_pb2.LandRequest(
                            context=client.context("phase9:recovery-land")
                        ),
                        timeout=5,
                    )
                    client.wait_action(land, 100)
            except (grpc.RpcError, RuntimeError, TimeoutError) as recovery_error:
                report["recovery_error"] = str(recovery_error)
        client.episode_outcome = report["status"]
        client.episode_score = report
        client.close()
        report["generated_at"] = time.strftime("%Y-%m-%dT%H:%M:%S%z")
        output = ROOT / "logs/phase9"
        output.mkdir(parents=True, exist_ok=True)
        path = output / f"rigorous_{time.strftime('%Y%m%dT%H%M%S')}.json"
        path.write_text(json.dumps(report, indent=2) + "\n")
        print(f"Phase 9 rigorous report: {path}")
        print(json.dumps(report, indent=2))


if __name__ == "__main__":
    raise SystemExit(main())

#!/usr/bin/env python3
"""Headless acceptance for the obstacle-course narrow route.

This is deliberately a normal typed Drone API client: it never imports
MAVLink.  The local planner must detour around the wall, while a separate
Gazebo-truth subscriber scores collision clearance only after flight.
"""

import json
import math
import os
import sys
import threading
import time
from pathlib import Path

import grpc

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / "build/generated/python"))
sys.path.insert(0, str(Path(__file__).resolve().parent))
sys.path.append(str(ROOT / "scripts/simulation"))
sys.path.append("/usr/lib/python3/dist-packages")

from icarus.v1 import action_pb2, state_pb2
from run_mission import MissionClient
from scenario_config import load_scenario
from score_phase5_trajectory import score


def main() -> int:
    session_path = ROOT / "logs/simulation/active_session.json"
    if not session_path.is_file():
        raise SystemExit("No active simulator; start obstacle_course first")
    session = json.loads(session_path.read_text())
    if session.get("scenario") != "obstacle_course":
        raise SystemExit("Narrow-route gate requires: ./scripts/start-sim --scenario obstacle_course")
    os.environ["GZ_PARTITION"] = session["partition"]

    from gz.msgs10.pose_v_pb2 import Pose_V
    from gz.transport13 import Node

    _, scenario = load_scenario("obstacle_course")
    route = next(item for item in scenario["ground_truth"]["routes"]
                 if item["name"] == "narrow")
    target = route["waypoints_enu_m"][-1]
    client = MissionClient("127.0.0.1:50051", "phase12_narrow_route")
    node = Node()
    truth = []
    lock = threading.Lock()
    report = {"status": "failed", "scenario": "obstacle_course",
              "route": "narrow", "seed": scenario["seed"]}

    def on_pose(message):
        for pose in message.pose:
            if pose.name == session["model"]:
                sample = {"t_s": time.monotonic(), "x_m": pose.position.x,
                          "y_m": pose.position.y, "z_m": pose.position.z}
                with lock:
                    if not truth or sample["t_s"] > truth[-1]["t_s"]:
                        truth.append(sample)
                return

    if not node.subscribe(Pose_V, f"/world/{session['world']}/dynamic_pose/info", on_pose):
        raise SystemExit("Unable to subscribe to Gazebo ground truth")

    try:
        client.connect()
        client.acquire()
        arm = client.action_api.Arm(
            action_pb2.ArmRequest(context=client.context("phase12:narrow-arm")), timeout=5)
        client.wait_action(arm, 55)
        takeoff = client.action_api.Takeoff(
            action_pb2.TakeoffRequest(
                context=client.context("phase12:narrow-takeoff"),
                target_altitude_agl_m=3.0,
                limits=action_pb2.ActionLimits(execution_timeout_ms=60_000)), timeout=5)
        client.wait_action(takeoff, 70)
        with lock:
            start_index = len(truth)
        goto = client.action_api.Goto(
            action_pb2.GotoRequest(
                context=client.context("phase12:narrow-goto"),
                destination=state_pb2.Position(local_ned=state_pb2.LocalPositionNed(
                    origin_id="home", north_m=target[0], east_m=target[1], down_m=-target[2])),
                acceptance_radius_m=1.0,
                limits=action_pb2.ActionLimits(maximum_ground_speed_mps=3.0,
                                                execution_timeout_ms=120_000,
                                                minimum_clearance_m=1.0)), timeout=5)
        terminal = client.wait_action(goto, 140)
        time.sleep(0.3)
        with lock:
            trajectory = list(truth[start_index:])
        if len(trajectory) < 50:
            raise RuntimeError("insufficient ground-truth trajectory samples")
        scored = score(scenario, trajectory, "narrow", vehicle_radius_m=0.35)
        required_clearance = float(route["clearance_m"])
        if scored["status"] != "passed":
            raise RuntimeError("narrow-route ground-truth scoring failed: " + str(scored))
        if scored["minimum_clearance_m"] < required_clearance:
            raise RuntimeError(
                f"narrow-route clearance {scored['minimum_clearance_m']:.3f} m "
                f"is below {required_clearance:.3f} m")
        if "safe detour" not in terminal.message:
            raise RuntimeError("narrow route completed without local-planner detour evidence")
        client.wait_action(client.action_api.Land(
            action_pb2.LandRequest(context=client.context("phase12:narrow-land")), timeout=5), 100)
        report.update(status="passed", planner_terminal_message=terminal.message,
                      ground_truth=scored, required_clearance_m=required_clearance,
                      ground_truth_samples=len(trajectory))
        return 0
    except (grpc.RpcError, RuntimeError, TimeoutError) as error:
        report["error"] = str(error)
        return 1
    finally:
        if client.session_id:
            try:
                state = client.state()
                if state.armed and client.lease_id:
                    client.wait_action(client.action_api.Land(
                        action_pb2.LandRequest(context=client.context("phase12:narrow-recovery")),
                        timeout=5), 100)
            except (grpc.RpcError, RuntimeError, TimeoutError) as error:
                report["recovery_error"] = str(error)
        client.episode_outcome = report["status"]
        client.episode_score = report
        client.close()
        report["generated_at"] = time.strftime("%Y-%m-%dT%H:%M:%S%z")
        output = ROOT / "logs/phase12"
        output.mkdir(parents=True, exist_ok=True)
        path = output / f"narrow_route_{time.strftime('%Y%m%dT%H%M%S')}.json"
        path.write_text(json.dumps(report, indent=2) + "\n")
        print(f"Phase 12 narrow-route report: {path}")
        print(json.dumps(report, indent=2))


if __name__ == "__main__":
    raise SystemExit(main())

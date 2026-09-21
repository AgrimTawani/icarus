#!/usr/bin/env python3
"""Prove the live Drone API refuses arming below the configured battery floor.

Run only against the reproducible ``adverse_combined`` simulator scenario. It
has a 35% initial SOC, below the V1 50% minimum-takeoff policy. The client never
sends MAVLink: the recorded rejected Arm request traverses the same typed API
and C++ guardrail validator as every normal mission.
"""

import json
import sys
import time
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
sys.path[:0] = [str(ROOT), str(ROOT / "build/generated/python"),
                str(Path(__file__).resolve().parent)]

from icarus.v1 import action_pb2
from run_mission import MissionClient


def main():
    session_path = ROOT / "logs/simulation/active_session.json"
    if not session_path.is_file():
        raise SystemExit("start ./scripts/start-sim --scenario adverse_combined first")
    session = json.loads(session_path.read_text())
    if session.get("scenario") != "adverse_combined":
        raise SystemExit("low-battery gate requires scenario adverse_combined")

    client = MissionClient("127.0.0.1:50051", "phase12_low_battery_rejection")
    report = {"status": "failed", "scenario": "adverse_combined"}
    try:
        client.connect()
        state = client.state()
        report["remaining_percent"] = state.battery.remaining_percent
        if state.battery.remaining_percent >= 50.0:
            raise RuntimeError("scenario battery is not below takeoff threshold")
        client.acquire()
        receipt = client.action_api.Arm(
            action_pb2.ArmRequest(context=client.context("phase12-low-battery-arm")),
            timeout=5)
        report.update({
            "disposition": action_pb2.ActionState.Name(receipt.disposition),
            "reason_code": action_pb2.ReasonCode.Name(receipt.reason_code),
            "message": receipt.message,
        })
        if receipt.disposition != action_pb2.ACTION_STATE_REJECTED:
            raise RuntimeError("arm was not rejected at low battery")
        if receipt.reason_code != action_pb2.REASON_CODE_BATTERY_BELOW_THRESHOLD:
            raise RuntimeError("arm rejection did not identify battery threshold")
        if client.state().armed:
            raise RuntimeError("aircraft armed despite low-battery rejection")
        report["status"] = "passed"
        return 0
    except Exception as error:  # retain a failed episode as evidence too
        report["error"] = str(error)
        print("FAIL:", error, flush=True)
        return 1
    finally:
        report["generated_at"] = time.strftime("%Y-%m-%dT%H:%M:%S%z")
        client.episode_outcome = report["status"]
        client.episode_score = report
        client.close()
        output = ROOT / "logs/phase12"
        output.mkdir(parents=True, exist_ok=True)
        path = output / ("low_battery_" + time.strftime("%Y%m%dT%H%M%S") + ".json")
        path.write_text(json.dumps(report, indent=2) + "\n")
        print("Phase 12 low-battery report:", path, flush=True)


if __name__ == "__main__":
    raise SystemExit(main())

#!/usr/bin/env python3
"""Exercise a DCM deadline against live simulator/API state without acting.

The synthetic runtime represents a model process that exceeded its hard
deadline. ``fly_mission`` must record the timeout, then terminate on a safe
``none`` response. No Drone API action is proposed, approved or dispatched.
"""

import json
import sys
import time
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
sys.path[:0] = [str(ROOT), str(ROOT / "build/generated/python"),
                str(Path(__file__).resolve().parent)]

from python.dcm.contract import DeadlineExceeded
from python.dcm.fly import fly_mission
from run_mission import MissionClient


class TimeoutThenNoneRuntime:
    name = "phase12-timeout-injector"

    def __init__(self):
        self.calls = 0

    def propose(self, _observation):
        self.calls += 1
        if self.calls == 1:
            raise DeadlineExceeded("injected Phase 12 DCM deadline")
        return '{"action":"none","arguments":{}}'


def main():
    session_path = ROOT / "logs/simulation/active_session.json"
    if not session_path.is_file():
        raise SystemExit("start ./scripts/start-sim first")
    client = MissionClient("127.0.0.1:50051", "phase12_dcm_timeout")
    report = {"status": "failed"}
    try:
        client.connect()
        client.acquire()
        result = fly_mission(
            client, TimeoutThenNoneRuntime(), "test DCM timeout recovery",
            mode="autonomous", max_decisions=2, echo=print)
        report["counts"] = result["counts"]
        report["executed"] = result["executed"]
        if result["counts"]["timeout"] != 1:
            raise RuntimeError("expected exactly one DCM timeout")
        if result["counts"]["executed"] != 0 or result["executed"]:
            raise RuntimeError("a timed-out DCM reached the action path")
        report["status"] = "passed"
        return 0
    except Exception as error:
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
        path = output / ("dcm_timeout_" + time.strftime("%Y%m%dT%H%M%S") + ".json")
        path.write_text(json.dumps(report, indent=2) + "\n")
        print("Phase 12 DCM-timeout report:", path, flush=True)


if __name__ == "__main__":
    raise SystemExit(main())

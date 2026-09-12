#!/usr/bin/env python3
"""Run repeated server-only start/ready/stop lifecycle checks."""

import argparse
import json
import os
import signal
import socket
import subprocess
import time
import uuid

from build_akshu_candidate import ROOT


def port_is_free(port, kind):
    with socket.socket(socket.AF_INET, kind) as probe:
        probe.bind(("127.0.0.1", port))


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--cycles", type=int, default=20)
    parser.add_argument("--profile", default="simulation-empty")
    parser.add_argument("--ready-timeout", type=float, default=90.0)
    args = parser.parse_args()
    if not 1 <= args.cycles <= 100:
        raise ValueError("cycles must be in 1..100")

    evidence = ROOT / "logs/simulation" / ("phase6_lifecycle_" + uuid.uuid4().hex[:8])
    evidence.mkdir(parents=True)
    active = ROOT / "logs/simulation/active_session.json"
    results = []
    for cycle in range(1, args.cycles + 1):
        if active.exists():
            raise RuntimeError(f"stale active session before cycle {cycle}")
        log_path = evidence / f"cycle_{cycle:02d}.log"
        with log_path.open("w") as log:
            process = subprocess.Popen(
                [
                    "/usr/bin/python3",
                    str(ROOT / "scripts/start-sim"),
                    "--profile",
                    args.profile,
                ],
                cwd=ROOT,
                stdout=log,
                stderr=subprocess.STDOUT,
                start_new_session=True,
            )
            session = None
            try:
                deadline = time.monotonic() + args.ready_timeout
                while time.monotonic() < deadline:
                    if process.poll() is not None:
                        raise RuntimeError(f"cycle {cycle} exited before readiness")
                    if active.exists():
                        candidate = json.loads(active.read_text())
                        if candidate.get("status") == "ready":
                            session = candidate
                            break
                    time.sleep(0.2)
                if session is None:
                    raise TimeoutError(f"cycle {cycle} readiness timed out")
                process.send_signal(signal.SIGINT)
                process.wait(timeout=30)
            finally:
                if process.poll() is None:
                    os.killpg(process.pid, signal.SIGTERM)
                    process.wait(timeout=20)
        if active.exists():
            raise RuntimeError(f"cycle {cycle} left an active session")
        port_is_free(5760, socket.SOCK_STREAM)
        port_is_free(9002, socket.SOCK_DGRAM)
        launch = json.loads((ROOT / session["run_directory"] / "launch.json").read_text())
        if launch.get("status") != "stopped":
            raise RuntimeError(f"cycle {cycle} did not stop cleanly")
        results.append(
            {
                "cycle": cycle,
                "status": "passed",
                "run_directory": session["run_directory"],
            }
        )
        print(f"cycle {cycle}/{args.cycles}: passed", flush=True)

    report = {
        "status": "passed",
        "profile": args.profile,
        "cycles": args.cycles,
        "results": results,
    }
    (evidence / "results.json").write_text(json.dumps(report, indent=2) + "\n")
    print(evidence)


if __name__ == "__main__":
    main()

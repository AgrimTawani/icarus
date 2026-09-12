#!/usr/bin/env python3
"""Run a named acceptance mission against an independently launched simulator."""

import argparse
import json
import os
import subprocess
import time
import uuid
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
ACTIVE_SESSION = ROOT / "logs/simulation/active_session.json"


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
        raise RuntimeError("Unsupported MAVLink endpoint")
    return session


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--mission",
        choices=("takeoff_hover_land", "preflight"),
        default="takeoff_hover_land",
    )
    parser.add_argument("--altitude", type=float, default=None)
    args = parser.parse_args()

    session = load_session()
    scenario_altitude = float(session["mission"]["altitude_m"])
    altitude = scenario_altitude if args.altitude is None else args.altitude
    if not 1.0 <= altitude <= 10.0:
        raise ValueError("altitude must be between 1 and 10 metres")
    if session.get("scenario") and altitude != scenario_altitude:
        raise ValueError(
            f"scenario {session['scenario']} fixes altitude at {scenario_altitude} m"
        )
    run_id = time.strftime("%Y%m%dT%H%M%S") + "_" + uuid.uuid4().hex[:6]
    directory = Path(session["run_directory"]) / "clients" / (args.mission + "_" + run_id)
    directory.mkdir(parents=True)
    command = [
        str(ROOT / "third_party/ardupilot/.venv/bin/python"),
        str(ROOT / "scripts/simulation/mark4_flight_check.py"),
        "--directory",
        str(directory),
        "--world",
        session["world"],
        "--model",
        session["model"],
        "--ground-height",
        str(session["ground_height_m"]),
        "--altitude",
        str(altitude),
    ]
    if args.mission == "preflight":
        command.append("--preflight-only")
    limits = session.get("success_limits", {})
    if limits:
        command.extend(
            [
                "--max-hover-drift",
                str(limits["maximum_drift_m"]),
                "--max-hover-tilt",
                str(limits["maximum_tilt_deg"]),
            ]
        )
    env = os.environ.copy()
    env["GZ_PARTITION"] = session["partition"]
    print("Mission artifacts:", directory, flush=True)
    completed = subprocess.run(command, env=env, check=False)
    raise SystemExit(completed.returncode)


if __name__ == "__main__":
    main()

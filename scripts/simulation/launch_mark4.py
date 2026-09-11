#!/usr/bin/env python3
"""Build, launch, verify and clean up the Mark4 Gazebo/SITL environment."""

import argparse
import hashlib
import json
import os
import signal
import socket
import subprocess
import time
import uuid
from pathlib import Path

from test_mark4_motors import stop

ROOT = Path(__file__).resolve().parents[2]


def interrupted(signum, _frame):
    raise SystemExit(128 + signum)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--gui", action="store_true")
    parser.add_argument(
        "--review",
        action="store_true",
        help="Pause at validated hover and leave GUI open",
    )
    parser.add_argument("--direction-check", action="store_true")
    args = parser.parse_args()
    signal.signal(signal.SIGINT, interrupted)
    signal.signal(signal.SIGTERM, interrupted)
    # Resolve conflicts before starting; never kill someone else's simulation.
    for port, kind in ((5760, socket.SOCK_STREAM), (9002, socket.SOCK_DGRAM)):
        with socket.socket(socket.AF_INET, kind) as check:
            check.bind(("127.0.0.1", port))
    run_id = time.strftime("%Y%m%dT%H%M%S") + "_" + uuid.uuid4().hex[:6]
    directory = ROOT / "logs/simulation" / ("mark4_flight_" + run_id)
    directory.mkdir(parents=True)
    sitl_state = directory / "sitl_state"
    sitl_state.mkdir()
    build = ROOT / "simulation/plugins/build"
    with (directory / "build.log").open("w") as log:
        subprocess.run(
            [
                "cmake",
                "-S",
                str(ROOT / "simulation/plugins"),
                "-B",
                str(build),
                "-G",
                "Ninja",
            ],
            stdout=log,
            stderr=subprocess.STDOUT,
            check=True,
        )
        subprocess.run(
            ["cmake", "--build", str(build), "-j", "2"],
            stdout=log,
            stderr=subprocess.STDOUT,
            check=True,
        )
    env = os.environ.copy()
    env["GZ_PARTITION"] = "icarus_mark4_" + run_id
    env["GZ_SIM_RESOURCE_PATH"] = str(ROOT / "simulation/models")
    env["GZ_SIM_SYSTEM_PLUGIN_PATH"] = (
        str(build) + ":" + str(ROOT / "third_party/ardupilot_gazebo/build")
    )
    commands = [
        [
            "gz",
            "sim",
            "-s",
            "-r",
            "-v",
            "3",
            str(ROOT / "simulation/worlds/mark4_validation.sdf"),
        ],
        [
            str(ROOT / "third_party/ardupilot/build/sitl/bin/arducopter"),
            "--model",
            "JSON",
            "--speedup",
            "1",
            "--home",
            "-35.363262,149.165237,584,90",
            "--wipe",
            "--defaults",
            str(
                ROOT / "third_party/ardupilot/Tools/autotest/default_params/copter.parm"
            )
            + ","
            + str(ROOT / "simulation/parameters/mark4_v2_base.parm"),
        ],
    ]
    inputs = [
        "simulation/models/mark4_v2/model.sdf",
        "simulation/models/mark4_v2_sitl/model.sdf",
        "simulation/worlds/mark4_validation.sdf",
        "simulation/parameters/mark4_v2_base.parm",
        "simulation/plugins/MotorBridge.cc",
        "simulation/plugins/build/libIcarusMotorBridge.so",
        "third_party/ardupilot/Tools/autotest/default_params/copter.parm",
        "third_party/ardupilot/build/sitl/bin/arducopter",
        "scripts/simulation/mark4_flight_check.py",
    ]
    hashes = {
        path: hashlib.file_digest((ROOT / path).open("rb"), "sha256").hexdigest()
        for path in inputs
    }
    revisions = {
        name: subprocess.check_output(
            ["git", "-C", str(ROOT / "third_party" / name), "rev-parse", "HEAD"],
            text=True,
        ).strip()
        for name in ("ardupilot", "ardupilot_gazebo")
    }
    (directory / "launch.json").write_text(
        json.dumps(
            {
                "commands": commands,
                "partition": env["GZ_PARTITION"],
                "world": "mark4_validation",
                "sha256": hashes,
                "revisions": revisions,
            },
            indent=2,
        )
        + "\n"
    )
    print("Artifacts:", directory, flush=True)
    processes = []
    logs = []
    try:
        for name, command in zip(("gazebo", "sitl"), commands):
            log = (directory / (name + ".log")).open("w")
            logs.append(log)
            processes.append(
                subprocess.Popen(
                    command,
                    cwd=sitl_state,
                    env=env,
                    stdout=log,
                    stderr=subprocess.STDOUT,
                    start_new_session=True,
                )
            )
        deadline = time.monotonic() + 30
        while True:
            if any(p.poll() is not None for p in processes):
                raise RuntimeError("Gazebo or SITL exited during startup; inspect logs")
            try:
                with socket.create_connection(("127.0.0.1", 5760), timeout=0.2):
                    break
            except OSError:
                if time.monotonic() > deadline:
                    raise TimeoutError("SITL MAVLink endpoint did not appear")
                time.sleep(0.25)

        def start_gui():
            log = (directory / "gui.log").open("w")
            logs.append(log)
            gui = subprocess.Popen(
                [
                    "gz",
                    "sim",
                    "-g",
                    "--gui-config",
                    str(ROOT / "simulation/launch/mark4_gui.config"),
                ],
                env=env,
                stdout=log,
                stderr=subprocess.STDOUT,
                start_new_session=True,
            )
            processes.append(gui)
            return gui

        if args.gui and not args.review:
            start_gui()
        controller = [
            str(ROOT / "third_party/ardupilot/.venv/bin/python"),
            str(ROOT / "scripts/simulation/mark4_flight_check.py"),
            "--directory",
            str(directory),
        ]
        if args.review:
            controller += ["--pause-at-hover"]
        if args.direction_check:
            controller += ["--direction-check"]
        with (directory / "controller.log").open("w") as log:
            child = subprocess.Popen(
                controller,
                env=env,
                stdout=log,
                stderr=subprocess.STDOUT,
                start_new_session=True,
            )
            processes.append(child)
            child.wait(timeout=240)
            if child.returncode:
                raise RuntimeError(
                    "Flight check failed: " + str(directory / "result.json")
                )
        print((directory / "result.json").read_text(), flush=True)
        if args.review:
            reply = subprocess.run(
                [
                    "gz",
                    "service",
                    "-s",
                    "/world/mark4_validation/control",
                    "--reqtype",
                    "gz.msgs.WorldControl",
                    "--reptype",
                    "gz.msgs.Boolean",
                    "--timeout",
                    "5000",
                    "--req",
                    "pause: true",
                ],
                env=env,
                check=True,
                timeout=8,
                capture_output=True,
                text=True,
            )
            if "data: true" not in reply.stdout:
                raise RuntimeError("Could not pause the review scene")
            gui = start_gui()
            print(
                "VISUAL REVIEW READY: Gazebo paused at hover. Close GUI or Ctrl+C to stop.",
                flush=True,
            )
            while gui.poll() is None:
                time.sleep(0.5)
        return 0
    finally:
        for process in reversed(processes):
            stop(process)
        for log in logs:
            log.close()


if __name__ == "__main__":
    raise SystemExit(main())

#!/usr/bin/python3
"""Launch the Icarus simulator, optionally with the legacy automated controller."""

import argparse
import fcntl
import hashlib
import ipaddress
import json
import os
import shutil
import signal
import socket
import subprocess
import time
import uuid

from build_akshu_candidate import ROOT
from scenario_config import load_scenario
from test_mark4_motors import stop


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--gui", action="store_true", help="Open the Gazebo graphical client"
    )
    parser.add_argument(
        "--server-only",
        action="store_true",
        help="Start Gazebo, SITL and sensors without taking control of the vehicle",
    )
    parser.add_argument(
        "--preflight-only",
        action="store_true",
        help="Check live sensors and navigation without running flight commands",
    )
    parser.add_argument(
        "--sensor-profile", choices=("nominal", "noisy"), default=None
    )
    parser.add_argument("--seed", type=int, default=None)
    parser.add_argument(
        "--scenario", help="Phase 5 scenario name from simulation/scenarios"
    )
    parser.add_argument(
        "--video-destination",
        default="127.0.0.1",
        help="Ground-station IPv4 address receiving the H.264/RTP stream",
    )
    parser.add_argument("--video-port", type=int, default=5600)
    args = parser.parse_args()
    if args.server_only and args.preflight_only:
        parser.error("--preflight-only belongs to the coupled acceptance runner, not --server-only")
    try:
        video_destination = str(ipaddress.IPv4Address(args.video_destination))
    except ipaddress.AddressValueError:
        parser.error("--video-destination must be an IPv4 address")
    if not 1 <= args.video_port <= 65535:
        parser.error("--video-port must be in 1..65535")
    scenario_path, scenario = (None, None)
    if args.scenario:
        scenario_path, scenario = load_scenario(args.scenario)
        if args.sensor_profile and args.sensor_profile != scenario["sensor_profile"]:
            raise ValueError("--sensor-profile conflicts with reproducible scenario")
        if args.seed is not None and args.seed != scenario["seed"]:
            raise ValueError("--seed conflicts with reproducible scenario")
    sensor_profile = scenario["sensor_profile"] if scenario else (args.sensor_profile or "nominal")
    seed = scenario["seed"] if scenario else (42 if args.seed is None else args.seed)
    for tool in ("gz", "cmake", "ninja"):
        if not shutil.which(tool):
            raise RuntimeError(f"Missing {tool}; no automatic dependency installation")
    for path in (
        "third_party/ardupilot/build/sitl/bin/arducopter",
        "third_party/ardupilot/.venv/bin/python",
        "third_party/ardupilot_gazebo/build/libArduPilotPlugin.so",
    ):
        if not (ROOT / path).is_file():
            raise RuntimeError("Missing dependency: " + path)
    if args.gui and not (
        os.environ.get("DISPLAY") or os.environ.get("WAYLAND_DISPLAY")
    ):
        raise RuntimeError("No desktop display; omit --gui")
    logs_root = ROOT / "logs/simulation"
    logs_root.mkdir(parents=True, exist_ok=True)
    active_session_path = logs_root / "active_session.json"
    if scenario and scenario["success"]["expected_outcome"] == "reject":
        run_id = time.strftime("%Y%m%dT%H%M%S") + "_" + uuid.uuid4().hex[:6]
        result = {
            "status": "passed",
            "decision": "rejected_before_launch",
            "scenario": scenario["name"],
            "reason": (
                f"wind {scenario['wind']['speed_m_s']} m/s exceeds operational "
                f"limit {scenario['wind']['operational_limit_m_s']} m/s"
            ),
            "armed": False,
            "processes_started": False,
        }
        output = logs_root / ("scenario_rejection_" + run_id + ".json")
        output.write_text(json.dumps(result, indent=2) + "\n")
        print(json.dumps(result, indent=2), flush=True)
        print("Result:", output, flush=True)
        return
    lock = (logs_root / "compact_launcher.lock").open("w")
    try:
        fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
    except BlockingIOError as error:
        raise RuntimeError("Another compact launcher is active") from error
    for port, kind in ((5760, socket.SOCK_STREAM), (9002, socket.SOCK_DGRAM)):
        with socket.socket(socket.AF_INET, kind) as test:
            test.bind(("127.0.0.1", port))

    def interrupted(signum, _frame):
        raise SystemExit(128 + signum)

    signal.signal(signal.SIGINT, interrupted)
    signal.signal(signal.SIGTERM, interrupted)
    run_id = time.strftime("%Y%m%dT%H%M%S") + "_" + uuid.uuid4().hex[:6]
    prefix = "scenario_" + scenario["name"] if scenario else "compact_flight"
    directory = logs_root / (prefix + "_" + run_id)
    directory.mkdir()
    state_dir = directory / "sitl_state"
    state_dir.mkdir()
    env = os.environ.copy()
    env["GZ_PARTITION"] = "icarus_compact_" + run_id
    env["GZ_SIM_RESOURCE_PATH"] = str(ROOT / "simulation/models")
    env["GZ_SIM_SYSTEM_PLUGIN_PATH"] = (
        str(ROOT / "simulation/plugins/build")
        + ":"
        + str(ROOT / "third_party/ardupilot_gazebo/build")
    )
    vendor = "/usr/share/glvnd/egl_vendor.d/10_nvidia.json"
    if os.path.exists(vendor):
        env["__EGL_VENDOR_LIBRARY_FILENAMES"] = vendor
    processes, handles = [], []
    session_published = False
    summary = {
        "status": "failed",
        "partition": env["GZ_PARTITION"],
        "run_directory": str(directory),
    }

    def start(name, command):
        handle = (directory / (name + ".log")).open("w")
        handles.append(handle)
        process = subprocess.Popen(
            command,
            cwd=state_dir,
            env=env,
            stdout=handle,
            stderr=subprocess.STDOUT,
            start_new_session=True,
        )
        processes.append((name, process))
        runtime = {
            "partition": env["GZ_PARTITION"],
            "children": {n: p.pid for n, p in processes},
        }
        temporary = directory / "runtime.tmp"
        temporary.write_text(json.dumps(runtime, indent=2) + "\n")
        temporary.replace(directory / "runtime.json")
        return process

    def health():
        path = directory / "sensors/health.json"
        return json.loads(path.read_text()) if path.exists() else {}

    def camera_health():
        path = directory / "camera_stream.json"
        return json.loads(path.read_text()) if path.exists() else {}

    def check_children():
        for name, p in processes:
            if p.poll() is not None:
                raise RuntimeError(
                    f"{name} exited ({p.returncode}); inspect {directory}"
                )

    print("Artifacts:", directory, flush=True)
    try:
        with (directory / "build.log").open("w") as log:
            build_command = (
                [
                    "/usr/bin/python3",
                    str(ROOT / "scripts/simulation/build_phase5_world.py"),
                    str(scenario_path),
                ]
                if scenario
                else [
                    "/usr/bin/python3",
                    str(ROOT / "scripts/simulation/build_compact_flight.py"),
                    "--sensor-profile",
                    sensor_profile,
                ]
            )
            for command in (
                build_command,
                [
                    "cmake",
                    "-S",
                    str(ROOT / "simulation/plugins"),
                    "-B",
                    str(ROOT / "simulation/plugins/build"),
                    "-G",
                    "Ninja",
                ],
                ["cmake", "--build", str(ROOT / "simulation/plugins/build"), "-j", "2"],
            ):
                subprocess.run(
                    command,
                    check=True,
                    stdout=log,
                    stderr=subprocess.STDOUT,
                    timeout=120,
                )
        world_path = (
            ROOT / "simulation/worlds/generated" / (scenario["name"] + ".sdf")
            if scenario
            else ROOT / "simulation/worlds/compact_flight.sdf"
        )
        model_path = (
            ROOT / "simulation/models" / ("phase5_" + scenario["name"]) / "model.sdf"
            if scenario
            else ROOT / "simulation/models/akshu_compact_sitl/model.sdf"
        )
        world_name = "icarus_" + scenario["name"] if scenario else "compact_flight"
        inputs = [
            ROOT / p
            for p in (
                "simulation/models/akshu_compact/model.sdf",
                "simulation/parameters/mark4_v2_base.parm",
                "simulation/plugins/build/libIcarusMotorBridge.so",
                "third_party/ardupilot/build/sitl/bin/arducopter",
                "scripts/simulation/mark4_flight_check.py",
                "scripts/simulation/launch_compact.py",
                "scripts/simulation/record_compact_sensors.py",
                "scripts/simulation/camera_stream.py",
            )
        ]
        inputs.extend((model_path, world_path))
        if scenario:
            truth_path = world_path.with_suffix(".ground_truth.json")
            inputs.extend((scenario_path, truth_path))
            shutil.copy2(scenario_path, directory / "scenario.json")
            shutil.copy2(truth_path, directory / "ground_truth.json")
        inputs += sorted(
            (ROOT / "simulation/models/akshu_compact/meshes").glob("*.stl")
        )
        hashes = {}
        for path in inputs:
            with path.open("rb") as f:
                hashes[str(path.relative_to(ROOT))] = hashlib.file_digest(
                    f, "sha256"
                ).hexdigest()
        summary["sha256"] = hashes
        summary["sensor_profile"] = sensor_profile
        summary["seed"] = seed
        summary["scenario"] = scenario["name"] if scenario else None
        summary["media_capture"] = False
        summary["revisions"] = {
            name: subprocess.check_output(
                ["git", "-C", str(ROOT / "third_party" / name), "rev-parse", "HEAD"],
                text=True,
            ).strip()
            for name in ("ardupilot", "ardupilot_gazebo")
        }
        start(
            "gazebo",
            [
                "gz",
                "sim",
                "-s",
                "-r",
                "--headless-rendering",
                "--seed",
                str(seed),
                "-v",
                "3",
                str(world_path),
            ],
        )
        start(
            "sitl",
            [
                str(ROOT / "third_party/ardupilot/build/sitl/bin/arducopter"),
                "--model",
                "JSON",
                "--speedup",
                "1",
                "--home",
                (
                    f"{scenario['home']['latitude_deg']},{scenario['home']['longitude_deg']},"
                    f"{scenario['home']['elevation_m']},{scenario['home']['heading_deg']}"
                    if scenario
                    else "-35.363262,149.165237,584,90"
                ),
                "--wipe",
                "--defaults",
                str(
                    ROOT
                    / "third_party/ardupilot/Tools/autotest/default_params/copter.parm"
                )
                + ","
                + str(ROOT / "simulation/parameters/mark4_v2_base.parm"),
            ],
        )
        recorder = start(
            "recorder",
            [
                "/usr/bin/python3",
                str(ROOT / "scripts/simulation/record_compact_sensors.py"),
                "--directory",
                str(directory),
            ],
        )
        deadline = time.monotonic() + 60
        while health().get("status") != "ready":
            check_children()
            if time.monotonic() > deadline:
                raise TimeoutError("Sensor readiness timed out: " + str(health()))
            time.sleep(0.25)
        print(
            "READY: all numeric sensor streams publishing; image pixels disabled",
            flush=True,
        )
        start(
            "camera_stream",
            [
                "/usr/bin/python3",
                str(ROOT / "scripts/simulation/camera_stream.py"),
                "--directory",
                str(directory),
                "--destination",
                video_destination,
                "--port",
                str(args.video_port),
            ],
        )
        camera_deadline = time.monotonic() + 20
        while True:
            check_children()
            video_health = camera_health()
            if video_health.get("status") == "ready":
                break
            if time.monotonic() > camera_deadline:
                raise TimeoutError("Camera stream readiness timed out: " + str(video_health))
            time.sleep(0.1)
        print(
            "READY: forward camera streaming H.264/RTP to "
            f"{video_destination}:{args.video_port}",
            flush=True,
        )
        if scenario and scenario["sensor_fault_schedule"]:
            start(
                "scenario_faults",
                [
                    "/usr/bin/python3",
                    str(ROOT / "scripts/simulation/run_scenario_faults.py"),
                    str(scenario_path),
                    "--directory",
                    str(directory),
                ],
            )
        if args.gui:
            start(
                "gui",
                [
                    "gz",
                    "sim",
                    "-g",
                    "--gui-config",
                    str(ROOT / "simulation/launch/compact_gui.config"),
                ],
            )
        if args.server_only:
            session = {
                "version": 1,
                "status": "ready",
                "launcher_pid": os.getpid(),
                "mavlink_endpoint": "tcp:127.0.0.1:5760",
                "json_physics_endpoint": "udp:127.0.0.1:9002",
                "video": {
                    "transport": "rtp-h264",
                    "codec": "h264",
                    "destination": video_destination,
                    "port": args.video_port,
                    "width": 640,
                    "height": 480,
                    "fps": 15,
                },
                "partition": env["GZ_PARTITION"],
                "run_directory": str(directory),
                "scenario": scenario["name"] if scenario else None,
                "world": world_name,
                "model": "icarus_compact",
                "ground_height_m": 0.1901,
                "mission": scenario["mission"] if scenario else {
                    "type": "takeoff_hover_land",
                    "altitude_m": 3.0,
                    "hover_s": 10.0,
                },
                "success_limits": scenario["success"] if scenario else {},
            }
            temporary = logs_root / "active_session.tmp"
            temporary.write_text(json.dumps(session, indent=2) + "\n")
            temporary.replace(active_session_path)
            session_published = True
            summary["status"] = "ready"
            summary["session"] = session
            print("SIMULATOR READY", flush=True)
            print("MAVLink: tcp:127.0.0.1:5760", flush=True)
            print("Camera: ./scripts/view-camera", flush=True)
            print("Manual: ./scripts/manual-control", flush=True)
            print("Mission: ./scripts/run-mission --mission takeoff_hover_land", flush=True)
            print("Stop: Ctrl+C in this terminal", flush=True)
            while True:
                check_children()
                h = health()
                if (
                    h.get("status") != "ready"
                    or time.monotonic() - h.get("updated_monotonic_s", 0) > 5
                ):
                    raise RuntimeError("Sensor recorder unhealthy: " + str(h))
                video_health = camera_health()
                if video_health.get("status") != "ready":
                    raise RuntimeError("Camera stream unhealthy: " + str(video_health))
                time.sleep(0.25)
        command = [
            str(ROOT / "third_party/ardupilot/.venv/bin/python"),
            str(ROOT / "scripts/simulation/mark4_flight_check.py"),
            "--directory",
            str(directory),
            "--world",
            world_name,
            "--model",
            "icarus_compact",
            "--ground-height",
            "0.1901",
        ]
        if not scenario:
            command.append("--direction-check")
        if args.preflight_only:
            command.append("--preflight-only")
        if scenario:
            command.extend(
                [
                    "--max-hover-drift",
                    str(scenario["success"]["maximum_drift_m"]),
                    "--max-hover-tilt",
                    str(scenario["success"]["maximum_tilt_deg"]),
                ]
            )
        controller = start("controller", command)
        deadline = time.monotonic() + (scenario["maximum_duration_s"] if scenario else 360)
        while controller.poll() is None:
            for name, p in processes:
                if name != "controller" and p.poll() is not None:
                    raise RuntimeError(f"{name} exited during flight")
            h = health()
            if (
                h.get("status") != "ready"
                or time.monotonic() - h.get("updated_monotonic_s", 0) > 5
            ):
                raise RuntimeError("Sensor recorder unhealthy: " + str(h))
            video_health = camera_health()
            if video_health.get("status") != "ready":
                raise RuntimeError("Camera stream unhealthy: " + str(video_health))
            if time.monotonic() > deadline:
                raise TimeoutError("Flight controller exceeded 360 seconds")
            time.sleep(0.25)
        if controller.returncode:
            raise RuntimeError(
                "Flight acceptance failed; inspect controller.log and result.json"
            )
        stop(recorder)
        h = health()
        if h.get("status") != "stopped" or h.get("error"):
            raise RuntimeError("Sensor recording did not finalize cleanly")
        verifier = start(
            "recording_check",
            [
                "/usr/bin/python3",
                str(ROOT / "scripts/simulation/inspect_flight_recording.py"),
                str(directory),
            ],
        )
        verifier.wait(timeout=90)
        if verifier.returncode:
            raise RuntimeError(
                "Recorded protobuf validation failed; inspect recording_check.log"
            )
        summary.update(
            flight=json.loads((directory / "result.json").read_text()),
            recording=h,
        )
        if scenario:
            rows = [json.loads(line) for line in (directory / "sensors/index.jsonl").read_text().splitlines()]
            imu_rows = [row for row in rows if row["channel"] == "imu"]
            rtf = (
                (imu_rows[-1]["simulation_s"] - imu_rows[0]["simulation_s"])
                / (imu_rows[-1]["wall_monotonic_s"] - imu_rows[0]["wall_monotonic_s"])
            )
            limits = scenario["success"]
            checks = {
                "mission": summary["flight"]["status"] == "passed",
                "drift": summary["flight"]["hover"]["max_drift_m"] <= limits["maximum_drift_m"],
                "tilt": summary["flight"]["hover"]["max_tilt_deg"] <= limits["maximum_tilt_deg"],
                "real_time_factor": rtf >= limits["minimum_real_time_factor"],
            }
            score = {"status": "passed" if all(checks.values()) else "failed", "checks": checks,
                     "real_time_factor": rtf, "limits": limits}
            (directory / "scenario_score.json").write_text(json.dumps(score, indent=2) + "\n")
            if score["status"] != "passed":
                raise RuntimeError("scenario score failed: " + str(checks))
            summary["scenario_score"] = score
        summary["status"] = "passed"
        print(json.dumps(summary["flight"], indent=2), flush=True)
    except SystemExit:
        summary["status"] = "stopped"
        raise
    except BaseException as error:
        summary["error"] = str(error)
        raise
    finally:
        signal.signal(signal.SIGINT, signal.SIG_IGN)
        signal.signal(signal.SIGTERM, signal.SIG_IGN)
        for _, process in reversed(processes):
            stop(process)
        for handle in handles:
            handle.close()
        if session_published and active_session_path.exists():
            try:
                active = json.loads(active_session_path.read_text())
            except (json.JSONDecodeError, OSError):
                active = {}
            if active.get("run_directory") == str(directory):
                active_session_path.unlink()
        summary["children_exit_codes"] = {name: p.poll() for name, p in processes}
        (directory / "launch.json").write_text(json.dumps(summary, indent=2) + "\n")
        lock.close()
        print("Stopped owned processes. Results:", directory, flush=True)


if __name__ == "__main__":
    main()

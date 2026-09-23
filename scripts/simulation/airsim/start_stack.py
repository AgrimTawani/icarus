#!/usr/bin/env python3
"""Start the isolated AirSim + ArduPilot SITL stack and prove sensor readiness."""

import argparse
import hashlib
import json
import os
import subprocess
import sys
import time
from pathlib import Path

ROOT = Path(__file__).resolve().parents[3]
SESSION = ROOT / "logs/simulation/airsim_active_session.json"
ARDUPILOT = ROOT / "third_party/ardupilot"
SITL = ARDUPILOT / "build/sitl/bin/arducopter"


def _sha256(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--binary", type=Path, required=True,
                        help="AirSim Unreal environment entrypoint")
    parser.add_argument("--startup-timeout", type=float, default=60)
    parser.add_argument("--rebuild-sitl", action="store_true")
    args = parser.parse_args()
    if args.rebuild_sitl or not SITL.is_file():
        environment = dict(os.environ)
        environment["PATH"] = str(ARDUPILOT / ".venv/bin") + os.pathsep + environment["PATH"]
        waf = ARDUPILOT / "modules/waf/waf-light"
        subprocess.run([str(waf), "configure", "--board", "sitl"],
                       cwd=ARDUPILOT, env=environment, check=True)
        subprocess.run([str(waf), "build", "--target", "bin/arducopter"],
                       cwd=ARDUPILOT, env=environment, check=True)
    subprocess.run([sys.executable, str(ROOT / "scripts/simulation/airsim/start.py"),
                    "--binary", str(args.binary),
                    "--startup-timeout", str(args.startup_timeout)], check=True)
    session = json.loads(SESSION.read_text())
    run_dir = ROOT / Path(session["runtime_home"]).parent
    sitl_log = run_dir / "arducopter.log"
    defaults = ",".join((
        str(ARDUPILOT / "Tools/autotest/default_params/copter.parm"),
        str(ARDUPILOT / "Tools/autotest/default_params/airsim-quadX.parm"),
        str(ROOT / "config/simulation/airsim/icarus-sitl.parm"),
    ))
    command = [str(SITL), "--model", "airsim-copter", "--speedup", "1",
               "--slave", "0", "--serial0=udpclient:127.0.0.1:14550",
               "--serial1=tcp:5760",
               "--sim-address=127.0.0.1", "--defaults", defaults, "--wipe", "-I0"]
    sitl_log.write_text("")
    sitl_unit = f"icarus-airsim-sitl-{int(time.time() * 1000)}"
    subprocess.run([
        "systemd-run", "--user", f"--unit={sitl_unit}", "--collect",
        "--property=Type=simple",
        f"--property=StandardOutput=append:{sitl_log}",
        f"--property=StandardError=append:{sitl_log}",
        f"--working-directory={run_dir}", "--", *command,
    ], check=True, stdout=subprocess.DEVNULL)
    sitl_pid = 0
    for _ in range(50):
        value = subprocess.check_output([
            "systemctl", "--user", "show", sitl_unit,
            "--property=MainPID", "--value"], text=True).strip()
        sitl_pid = int(value or 0)
        if sitl_pid:
            break
        time.sleep(0.1)
    if not sitl_pid:
        subprocess.run(["systemctl", "--user", "stop", sitl_unit], check=False)
        raise SystemExit("systemd did not publish the ArduPilot SITL pid")
    session.update({
        "schema": "icarus.simulation.airsim-stack.v1",
        "sitl_pid": sitl_pid,
        "sitl_systemd_unit": sitl_unit,
        "sitl_command": command,
        "sitl_log": str(sitl_log.relative_to(ROOT)),
        "sitl_binary_sha256": _sha256(SITL),
        "ardupilot_revision": subprocess.check_output(
            ["git", "rev-parse", "HEAD"], cwd=ARDUPILOT, text=True).strip(),
        "controller": "ArduPilot-SITL",
        "capability": "arducopter-sensor-conformance-candidate",
    })
    SESSION.write_text(json.dumps(session, indent=2) + "\n")
    deadline = time.monotonic() + args.startup_timeout
    smoke = [str(ROOT / "scripts/airsim-smoke"), "--timeout", "5"]
    try:
        while time.monotonic() < deadline:
            active = subprocess.run(
                ["systemctl", "--user", "is-active", "--quiet", sitl_unit],
                check=False).returncode == 0
            if not active:
                raise RuntimeError(
                    "ArduPilot SITL exited during startup")
            result = subprocess.run(smoke, text=True, capture_output=True, check=False)
            if result.returncode == 0:
                session["lidar_udp_port"] = 41452
                session["status"] = "ready"
                session["ready_unix_ms"] = int(time.time() * 1000)
                SESSION.write_text(json.dumps(session, indent=2) + "\n")
                print(result.stdout.strip())
                print("AirSim + ArduPilot stack is ready:", SESSION)
                return
            time.sleep(0.5)
        raise RuntimeError(
            "AirSim/ArduPilot stack did not pass sensor smoke before timeout")
    except BaseException:
        subprocess.run([str(ROOT / "scripts/stop-airsim")], check=False)
        raise


if __name__ == "__main__":
    main()

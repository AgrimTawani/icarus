#!/usr/bin/env python3
"""Start an isolated AirSim visual-simulation session.

This launcher starts only the Unreal/AirSim process. It never sends vehicle
commands and it refuses to overlap an active Gazebo session. ArduPilot SITL
must be started separately with ports matching the version-pinned AirSim
configuration before this command can be considered flight-ready.
"""

import argparse
import json
import os
import signal
import socket
import subprocess
import sys
import time
from pathlib import Path

ROOT = Path(__file__).resolve().parents[3]
DEFAULT_SETTINGS = ROOT / "config/simulation/airsim/settings.arducopter.json"
GAZEBO_SESSION = ROOT / "logs/simulation/active_session.json"
SESSION = ROOT / "logs/simulation/airsim_active_session.json"


def _pid_alive(pid):
    try:
        os.kill(pid, 0)
    except (ProcessLookupError, ValueError, TypeError):
        return False
    return True


def _port_open(port):
    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as probe:
        probe.settimeout(0.2)
        return probe.connect_ex(("127.0.0.1", port)) == 0


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    configured_binary = os.environ.get("ICARUS_AIRSIM_BINARY")
    parser.add_argument("--binary", type=Path,
                        default=Path(configured_binary) if configured_binary else None,
                        required=False)
    parser.add_argument("--settings", type=Path, default=DEFAULT_SETTINGS)
    parser.add_argument("--port", type=int, default=41451)
    parser.add_argument("--startup-timeout", type=float, default=45)
    parser.add_argument("--no-render-offscreen", action="store_true",
                        help="Do not request Unreal offscreen rendering (only for a verified custom binary).")
    parser.add_argument("--nullrhi", action="store_true",
                        help="Disable all rendering; refused unless --no-vision is also supplied.")
    parser.add_argument("--no-vision", action="store_true",
                        help="Acknowledge that this run cannot supply camera/depth evidence.")
    parser.add_argument("extra", nargs="*", help="Additional arguments for a verified environment binary.")
    args = parser.parse_args()
    if args.binary is None:
        parser.error("pass --binary or set ICARUS_AIRSIM_BINARY")
    if args.nullrhi and not args.no_vision:
        parser.error("-nullrhi disables image rendering; it requires --no-vision")
    if args.startup_timeout <= 0:
        parser.error("--startup-timeout must be positive")
    if GAZEBO_SESSION.exists():
        try:
            active = json.loads(GAZEBO_SESSION.read_text())
            if _pid_alive(active.get("launcher_pid")):
                parser.error("Gazebo session is active; stop it before starting the isolated AirSim session")
        except (OSError, json.JSONDecodeError):
            parser.error("cannot verify Gazebo session; remove only a confirmed stale session file")
    if SESSION.exists():
        try:
            active = json.loads(SESSION.read_text())
            if _pid_alive(active.get("pid")):
                parser.error("AirSim session is already active")
        except (OSError, json.JSONDecodeError):
            parser.error("cannot verify AirSim session; remove only a confirmed stale session file")
    if _port_open(args.port):
        parser.error(f"AirSim RPC port {args.port} is already in use")
    doctor = ROOT / "scripts/simulation/airsim/doctor.py"
    checked = subprocess.run([sys.executable, str(doctor), "--binary", str(args.binary),
                              "--settings", str(args.settings)], check=False)
    if checked.returncode:
        raise SystemExit(checked.returncode)
    command = [str(args.binary), f"-settings={args.settings}"]
    if args.nullrhi:
        command.append("-nullrhi")
    elif not args.no_render_offscreen:
        command.append("-RenderOffScreen")
    command.extend(args.extra)
    process = subprocess.Popen(command, cwd=args.binary.parent, start_new_session=True)
    deadline = time.monotonic() + args.startup_timeout
    while time.monotonic() < deadline:
        if process.poll() is not None:
            raise SystemExit(f"AirSim exited during startup ({process.returncode})")
        if _port_open(args.port):
            SESSION.parent.mkdir(parents=True, exist_ok=True)
            try:
                settings_path = str(args.settings.relative_to(ROOT))
            except ValueError:
                settings_path = str(args.settings)
            SESSION.write_text(json.dumps({
                "schema": "icarus.simulation.airsim-session.v1",
                "pid": process.pid, "rpc_port": args.port,
                "settings": settings_path,
                "rendering": "disabled" if args.nullrhi else "offscreen",
                "started_unix_ms": int(time.time() * 1000),
                "command": command,
            }, indent=2) + "\n")
            print("AirSim RPC is ready on port", args.port)
            print("Session:", SESSION)
            return
        time.sleep(0.25)
    os.killpg(process.pid, signal.SIGTERM)
    raise SystemExit(f"AirSim did not open RPC port {args.port} within {args.startup_timeout}s")


if __name__ == "__main__":
    main()

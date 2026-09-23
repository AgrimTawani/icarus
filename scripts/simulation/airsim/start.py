#!/usr/bin/env python3
"""Start an isolated AirSim visual-simulation session.

This launcher starts only the Unreal/AirSim process. It never sends vehicle
commands and it refuses to overlap an active Gazebo session. ArduPilot SITL
must be started separately with ports matching the version-pinned AirSim
configuration before this command can be considered flight-ready.
"""

import argparse
import hashlib
import json
import os
import shutil
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
LOG_ROOT = ROOT / "logs/simulation/airsim"


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


def _sha256(path):
    digest = hashlib.sha256()
    with Path(path).open("rb") as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def _terminate(process):
    """Terminate the child process group and reap it before another launch."""
    try:
        os.killpg(process.pid, signal.SIGTERM)
    except ProcessLookupError:
        return
    try:
        process.wait(timeout=10)
    except subprocess.TimeoutExpired:
        os.killpg(process.pid, signal.SIGKILL)
        process.wait(timeout=5)


def _wait_for_port_close(port, timeout=10):
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        if not _port_open(port):
            return
        time.sleep(0.1)


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
    parser.add_argument("--allow-simpleflight-smoke", action="store_true",
                        help="Permit a known SimpleFlight environment for visual/RPC smoke only; never flight tests.")
    parser.add_argument("extra", nargs="*", help="Additional arguments for a verified environment binary.")
    args = parser.parse_args()
    if args.binary is None:
        parser.error("pass --binary or set ICARUS_AIRSIM_BINARY")
    args.binary = args.binary.resolve()
    args.settings = args.settings.resolve()
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
                              "--settings", str(args.settings),
                              *(["--allow-windowed"] if args.no_render_offscreen else [])],
                             check=False)
    if checked.returncode:
        raise SystemExit(checked.returncode)
    # Classic AirSim does not consume a ``-settings=`` command-line option.
    # It reads $HOME/Documents/AirSim/settings.json. Give every launch a
    # private HOME so the reviewed file is guaranteed to be the file used and
    # the operator's real settings are never read or overwritten.
    started_ms = int(time.time() * 1000)
    runtime_home = LOG_ROOT / f"run-{started_ms}" / "home"
    settings_target = runtime_home / "Documents" / "AirSim" / "settings.json"
    settings_target.parent.mkdir(parents=True, exist_ok=True)
    shutil.copyfile(args.settings, settings_target)
    command = [str(args.binary)]
    if args.nullrhi:
        command.append("-nullrhi")
    elif not args.no_render_offscreen:
        command.append("-RenderOffScreen")
    command.extend(args.extra)
    LOG_ROOT.mkdir(parents=True, exist_ok=True)
    log_path = runtime_home.parent / "airsim.log"
    log_handle = log_path.open("w", encoding="utf-8")
    environment = dict(os.environ)
    environment["HOME"] = str(runtime_home)
    process = subprocess.Popen(command, cwd=args.binary.parent, start_new_session=True,
                               stdout=log_handle, stderr=subprocess.STDOUT,
                               env=environment)
    log_handle.close()
    deadline = time.monotonic() + args.startup_timeout
    while time.monotonic() < deadline:
        if process.poll() is not None:
            raise SystemExit(f"AirSim exited during startup ({process.returncode})")
        if _port_open(args.port):
            log_text = log_path.read_text(errors="replace")
            simpleflight = "LogTemp: SimpleFlight" in log_text
            if simpleflight and not args.allow_simpleflight_smoke:
                _terminate(process)
                _wait_for_port_close(args.port)
                raise SystemExit(
                    "AirSim environment selected SimpleFlight; refusing ArduCopter session. "
                    "Use a compatible environment, or pass --allow-simpleflight-smoke only for visual/RPC smoke.")
            SESSION.parent.mkdir(parents=True, exist_ok=True)
            try:
                settings_path = str(args.settings.relative_to(ROOT))
            except ValueError:
                settings_path = str(args.settings)
            SESSION.write_text(json.dumps({
                "schema": "icarus.simulation.airsim-session.v1",
                "pid": process.pid, "rpc_port": args.port,
                "settings": settings_path,
                "settings_sha256": _sha256(args.settings),
                "binary_sha256": _sha256(args.binary),
                "runtime_home": str(runtime_home.relative_to(ROOT)),
                "rendering": ("disabled" if args.nullrhi else
                              "windowed" if args.no_render_offscreen else "offscreen"),
                "capability": "visual-rpc-smoke-only" if simpleflight else "arducopter-candidate",
                "log": str(log_path.relative_to(ROOT)),
                "started_unix_ms": started_ms,
                "command": command,
            }, indent=2) + "\n")
            print("AirSim RPC is ready on port", args.port)
            print("Session:", SESSION)
            return
        time.sleep(0.25)
    _terminate(process)
    _wait_for_port_close(args.port)
    raise SystemExit(f"AirSim did not open RPC port {args.port} within {args.startup_timeout}s")


if __name__ == "__main__":
    main()

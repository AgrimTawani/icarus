#!/usr/bin/env python3
"""Stop only the AirSim process recorded by the isolated AirSim launcher."""

import json
import os
import signal
import socket
import subprocess
import time
from pathlib import Path

ROOT = Path(__file__).resolve().parents[3]
SESSION = ROOT / "logs/simulation/airsim_active_session.json"


def _port_open(port):
    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as probe:
        probe.settimeout(0.2)
        return probe.connect_ex(("127.0.0.1", port)) == 0


def _wait_port_closed(port, timeout=10):
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        if not _port_open(port):
            return True
        time.sleep(0.1)
    return False


def _stop_group(pid):
    if not isinstance(pid, int):
        return
    try:
        os.killpg(pid, signal.SIGTERM)
    except ProcessLookupError:
        return
    deadline = time.monotonic() + 5
    while time.monotonic() < deadline:
        try:
            os.killpg(pid, 0)
        except ProcessLookupError:
            return
        time.sleep(0.1)
    try:
        os.killpg(pid, signal.SIGKILL)
    except ProcessLookupError:
        pass


def main():
    if not SESSION.exists():
        raise SystemExit("No AirSim session file exists")
    session = json.loads(SESSION.read_text())
    pid = session.get("pid")
    if not isinstance(pid, int):
        raise SystemExit("AirSim session has no valid pid")
    _stop_group(session.get("sensor_bridge_pid"))
    unit = session.get("sitl_systemd_unit")
    if isinstance(unit, str) and unit.startswith("icarus-airsim-sitl-"):
        # A lock-step SITL may not service SIGTERM while blocked on AirSim.
        # Signal the exact transient unit, then bound shutdown with SIGKILL.
        subprocess.run(["systemctl", "--user", "kill", "--signal=SIGTERM", unit],
                       check=False, stdout=subprocess.DEVNULL,
                       stderr=subprocess.DEVNULL)
        time.sleep(0.5)
        subprocess.run(["systemctl", "--user", "kill", "--signal=SIGKILL", unit],
                       check=False, stdout=subprocess.DEVNULL,
                       stderr=subprocess.DEVNULL)
        subprocess.run(["systemctl", "--user", "reset-failed", unit], check=False,
                       stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    else:
        _stop_group(session.get("sitl_pid"))
    _stop_group(pid)
    if not _wait_port_closed(int(session.get("rpc_port", 41451))):
        raise SystemExit("Recorded AirSim group stopped but RPC port is still open")
    SESSION.unlink()
    print("Stopped recorded AirSim session")


if __name__ == "__main__":
    main()

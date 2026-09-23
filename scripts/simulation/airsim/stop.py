#!/usr/bin/env python3
"""Stop only the AirSim process recorded by the isolated AirSim launcher."""

import json
import os
import signal
from pathlib import Path

ROOT = Path(__file__).resolve().parents[3]
SESSION = ROOT / "logs/simulation/airsim_active_session.json"


def main():
    if not SESSION.exists():
        raise SystemExit("No AirSim session file exists")
    session = json.loads(SESSION.read_text())
    pid = session.get("pid")
    if not isinstance(pid, int):
        raise SystemExit("AirSim session has no valid pid")
    try:
        os.killpg(pid, signal.SIGTERM)
    except ProcessLookupError:
        pass
    SESSION.unlink()
    print("Stopped recorded AirSim session")


if __name__ == "__main__":
    main()

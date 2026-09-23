#!/usr/bin/env python3
"""Canonical Gazebo Harmonic / ArduPilot SITL launch entry point.

The legacy ``./scripts/start-sim`` command delegates here for compatibility.
"""

import os
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[3]
LAUNCHER = ROOT / "scripts" / "simulation" / "launch_compact.py"


if __name__ == "__main__":
    os.execv("/usr/bin/python3", ["/usr/bin/python3", str(LAUNCHER),
                                  "--server-only", *sys.argv[1:]])

#!/usr/bin/env python3
"""Validate an opt-in AirSim environment without starting a flight session."""

import argparse
import json
import os
from pathlib import Path

ROOT = Path(__file__).resolve().parents[3]
DEFAULT_SETTINGS = ROOT / "config/simulation/airsim/settings.arducopter.json"


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    configured_binary = os.environ.get("ICARUS_AIRSIM_BINARY")
    parser.add_argument("--binary", type=Path,
                        default=Path(configured_binary) if configured_binary else None)
    parser.add_argument("--settings", type=Path, default=DEFAULT_SETTINGS)
    parser.add_argument("--allow-windowed", action="store_true",
                        help="Accept a reviewed interactive AirSim ViewMode instead of NoDisplay.")
    args = parser.parse_args()
    findings = []
    if args.binary is None:
        findings.append("missing AirSim binary: pass --binary or set ICARUS_AIRSIM_BINARY")
    elif not args.binary.is_file() or not os.access(args.binary, os.X_OK):
        findings.append(f"AirSim binary is not executable: {args.binary}")
    try:
        settings = json.loads(args.settings.read_text())
    except (OSError, json.JSONDecodeError) as error:
        findings.append(f"invalid settings file {args.settings}: {error}")
        settings = {}
    vehicle = (settings.get("Vehicles") or {}).get("Icarus") or {}
    view_mode = settings.get("ViewMode")
    interactive_modes = {"FlyWithMe", "Fpv", "GroundObserver", "SpringArmChase", "Manual"}
    if args.allow_windowed:
        if view_mode not in interactive_modes:
            findings.append(
                "windowed settings must use a supported interactive ViewMode "
                f"({', '.join(sorted(interactive_modes))})")
    elif view_mode != "NoDisplay":
        findings.append("settings must set ViewMode=NoDisplay for headless visual tests")
    if vehicle.get("VehicleType") != "ArduCopter":
        findings.append("settings must use VehicleType=ArduCopter, never SimpleFlight")
    if settings.get("ApiServerPort") != 41451:
        findings.append("settings must use the documented isolated RPC port 41451")
    if findings:
        for finding in findings:
            print("FAIL:", finding)
        raise SystemExit(1)
    print("AirSim configuration is structurally valid.")
    print("Next: run start-airsim-stack; it starts AirSim before lock-step ArduPilot SITL.")


if __name__ == "__main__":
    main()

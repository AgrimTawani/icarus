#!/usr/bin/env python3
"""Start a verified custom-Icarus7 AirSim environment with ArduPilot SITL."""

import argparse
import hashlib
import json
import subprocess
from pathlib import Path

ROOT = Path(__file__).resolve().parents[3]
DEFAULT_INSTALL = ROOT / "third_party/airsim/environments/icarus7/packaged"
HEADLESS_SETTINGS = ROOT / "config/simulation/airsim/settings.icarus7.json"
GUI_SETTINGS = ROOT / "config/simulation/airsim/settings.icarus7.gui.json"


def sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--install-dir", type=Path, default=DEFAULT_INSTALL)
    parser.add_argument("--gui", action="store_true")
    parser.add_argument("--graphics-adapter", type=int, default=0)
    parser.add_argument("--startup-timeout", type=float, default=90)
    parser.add_argument("--mission-reference", type=Path)
    args = parser.parse_args()

    install = args.install_dir.resolve()
    receipt_path = install / "icarus7-build.json"
    if not receipt_path.is_file():
        raise SystemExit(
            f"verified Icarus7 build receipt not found: {receipt_path}; "
            "package with ./scripts/package-airsim-icarus7-environment")
    receipt = json.loads(receipt_path.read_text())
    if receipt.get("vehicle_id") != "icarus7":
        raise SystemExit("build receipt is not for the Icarus7 vehicle")
    binary = install / receipt["entrypoint"]
    if not binary.is_file() or sha256(binary) != receipt.get("entrypoint_sha256"):
        raise SystemExit("packaged Icarus7 entrypoint is missing or has changed")
    settings = GUI_SETTINGS if args.gui else HEADLESS_SETTINGS
    settings_key = "gui_sha256" if args.gui else "headless_sha256"
    if sha256(settings) != receipt.get("settings", {}).get(settings_key):
        raise SystemExit("Icarus7 settings changed after packaging; rebuild the environment")

    command = [
        str(ROOT / "scripts/start-airsim-stack"),
        "--binary", str(binary),
        "--settings", str(settings),
        "--environment-id", "icarus7-custom-v1",
        "--environment-license", "user-supplied-unreviewed",
        "--environment-source", "Icarus7_FlightControl_Review",
        "--startup-timeout", str(args.startup_timeout),
        "--binary-arg=-NoSound", "--binary-arg=-NoVSync",
        f"--binary-arg=-GraphicsAdapter={args.graphics_adapter}",
    ]
    if args.mission_reference:
        command.extend(["--mission-reference", str(args.mission_reference.resolve())])
    if args.gui:
        command.extend([
            "--no-render-offscreen", "--binary-arg=-windowed",
            "--binary-arg=-ResX=1280", "--binary-arg=-ResY=720",
        ])
    subprocess.run(command, cwd=ROOT, check=True)


if __name__ == "__main__":
    main()

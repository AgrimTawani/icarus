#!/usr/bin/env python3
"""Start a licensed external AirVLN city with Icarus ArduPilot integration."""

import argparse
import json
import subprocess
from pathlib import Path

ROOT = Path(__file__).resolve().parents[3]
CONFIG = ROOT / "config/simulation/airsim/airvln-simulators.json"
DEFAULT_INSTALL = ROOT / "third_party/airsim/environments/airvln/extracted"
SETTINGS = ROOT / "config/simulation/airsim/settings.realistic-city.json"
GUI_SETTINGS = ROOT / "config/simulation/airsim/settings.realistic-city.gui.json"


def find_entrypoint(install: Path, scene: int) -> Path:
    matches = sorted(install.glob(f"**/env_{scene}/LinuxNoEditor/AirVLN.sh"))
    if len(matches) != 1:
        detail = "not found" if not matches else f"ambiguous ({len(matches)} matches)"
        raise SystemExit(
            f"AirVLN scene env_{scene} is {detail} beneath {install}. "
            "Run ./scripts/fetch-airvln-environments first.")
    return matches[0]


def main():
    metadata = json.loads(CONFIG.read_text())
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--scene", type=int, default=metadata["default_scene"],
                        choices=metadata["available_scenes"])
    parser.add_argument("--install-dir", type=Path, default=DEFAULT_INSTALL)
    parser.add_argument("--graphics-adapter", type=int, default=0)
    parser.add_argument("--startup-timeout", type=float, default=90)
    parser.add_argument("--gui", action="store_true",
                        help="Open the Unreal city viewport (FlyWithMe) instead of running off-screen.")
    args = parser.parse_args()
    entrypoint = find_entrypoint(args.install_dir.resolve(), args.scene)
    reference = ROOT / "config/simulation/airsim" / f"airvln-env-{args.scene}-reference.json"
    settings = GUI_SETTINGS if args.gui else SETTINGS
    command = [
        str(ROOT / "scripts/start-airsim-stack"),
        "--binary", str(entrypoint),
        "--settings", str(settings),
        "--environment-id", f"airvln-env-{args.scene}-v{metadata['dataset_version']}",
        "--environment-license", metadata["license"],
        "--environment-source", metadata["source"],
        "--startup-timeout", str(args.startup_timeout),
        "--binary-arg=-NoSound",
        "--binary-arg=-NoVSync",
        f"--binary-arg=-GraphicsAdapter={args.graphics_adapter}",
    ]
    if args.gui:
        command.extend([
            "--no-render-offscreen",
            "--binary-arg=-windowed",
            "--binary-arg=-ResX=1280",
            "--binary-arg=-ResY=720",
        ])
    if reference.is_file():
        command.extend(["--mission-reference", str(reference)])
    subprocess.run(command, cwd=ROOT, check=True)


if __name__ == "__main__":
    main()

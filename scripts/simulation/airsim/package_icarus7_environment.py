#!/usr/bin/env python3
"""Compile and package an Icarus7-enabled AirSim project with Unreal Engine 4.27."""

import argparse
import hashlib
import json
import subprocess
from pathlib import Path

ROOT = Path(__file__).resolve().parents[3]
VALIDATOR = ROOT / "scripts/simulation/airsim/validate_icarus7_module.py"
MODULE_MANIFEST = ROOT / "simulation/airsim/vehicles/icarus7/manifest.json"
DEFAULT_OUTPUT = ROOT / "third_party/airsim/environments/icarus7/packaged"


def sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--ue-root", type=Path, required=True,
                        help="Source-built Unreal Engine 4.27 root")
    parser.add_argument("--environment-project", type=Path, required=True)
    parser.add_argument("--airsim-source", type=Path, required=True)
    parser.add_argument("--output", type=Path, default=DEFAULT_OUTPUT)
    parser.add_argument("--configuration", choices=("Development", "Shipping"),
                        default="Development")
    args = parser.parse_args()

    project = args.environment_project.resolve()
    ue_root = args.ue_root.resolve()
    output = args.output.resolve()
    run_uat = ue_root / "Engine/Build/BatchFiles/RunUAT.sh"
    if not run_uat.is_file():
        raise SystemExit(f"Unreal Engine 4.27 RunUAT.sh not found: {run_uat}")
    subprocess.run([
        "/usr/bin/python3", str(VALIDATOR),
        "--environment-project", str(project),
        "--airsim-source", str(args.airsim_source.resolve()),
    ], check=True)

    output.mkdir(parents=True, exist_ok=True)
    command = [
        str(run_uat), "BuildCookRun", f"-project={project}", "-noP4",
        "-platform=Linux", f"-clientconfig={args.configuration}",
        "-build", "-cook", "-allmaps", "-stage", "-pak", "-archive",
        f"-archivedirectory={output}", "-utf8output", "-nodebuginfo",
    ]
    subprocess.run(command, cwd=project.parent, check=True)
    entrypoints = sorted(output.glob("**/LinuxNoEditor/*.sh"))
    if len(entrypoints) != 1:
        raise SystemExit(
            f"expected one packaged Linux entrypoint under {output}, found {len(entrypoints)}")
    entrypoint = entrypoints[0]
    entrypoint.chmod(entrypoint.stat().st_mode | 0o100)
    module = json.loads(MODULE_MANIFEST.read_text())
    receipt = {
        "schema": "icarus.airsim.packaged-environment.v1",
        "vehicle_id": "icarus7",
        "entrypoint": str(entrypoint.relative_to(output)),
        "entrypoint_sha256": sha256(entrypoint),
        "environment_project": str(project),
        "airsim_revision": module["airsim_revision"],
        "pawn_class": module["pawn_class"],
        "settings": {
            "headless_sha256": sha256(ROOT / "config/simulation/airsim/settings.icarus7.json"),
            "gui_sha256": sha256(ROOT / "config/simulation/airsim/settings.icarus7.gui.json"),
        },
    }
    receipt_path = output / "icarus7-build.json"
    receipt_path.write_text(json.dumps(receipt, indent=2) + "\n")
    print(f"Packaged Icarus7 AirSim environment: {entrypoint}")
    print(f"Build receipt: {receipt_path}")


if __name__ == "__main__":
    main()

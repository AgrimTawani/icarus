#!/usr/bin/env python3
"""Install the reviewed Icarus7 assets and native pawn into an AirSim UE4 project."""

import argparse
import hashlib
import json
import shutil
import subprocess
from pathlib import Path

ROOT = Path(__file__).resolve().parents[3]
MODULE_ROOT = ROOT / "simulation/airsim/vehicles/icarus7"
PLUGIN_SOURCE = MODULE_ROOT / "Plugins/Icarus7AirSim"
PATCH = MODULE_ROOT / "patches/airsim-v1.8.0-icarus7-physics.patch"
MANIFEST = MODULE_ROOT / "manifest.json"
PINNED_AIRSIM = "caa13182328e866564c09fdeebca89ba024bc323"
ASSET_SUBDIR = Path("Content/Icarus7FlightControlReview")


def sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def resolve_asset_project(path: Path) -> Path:
    path = path.resolve()
    if list(path.glob("*.uproject")):
        return path
    children = [candidate.parent for candidate in path.glob("*/*.uproject")]
    if len(children) == 1:
        return children[0]
    raise SystemExit(f"asset project not found or ambiguous beneath {path}")


def verify_asset_project(project: Path, manifest: dict) -> None:
    projects = list(project.glob("*.uproject"))
    if len(projects) != 1:
        raise SystemExit(f"expected one .uproject in {project}, found {len(projects)}")
    descriptor = json.loads(projects[0].read_text())
    if descriptor.get("EngineAssociation") != "4.27":
        raise SystemExit("Icarus7 asset project must target Unreal Engine 4.27")

    expected = manifest["asset_project_sha256"]
    files = {
        "uproject": projects[0],
        "complete_uasset": project / ASSET_SUBDIR /
        "Meshes/SM_Icarus7_Companion_Complete.uasset",
        "complete_fbx": project / "SourceAssets/Icarus7_Companion_Complete.fbx",
    }
    for name, path in files.items():
        if not path.is_file():
            raise SystemExit(f"missing reviewed asset input: {path}")
        actual = sha256(path)
        if actual != expected[name]:
            raise SystemExit(f"asset digest mismatch for {path}: {actual}")

    required = [
        "Meshes/SM_Icarus7_Body.uasset",
        "Meshes/SM_Icarus7_Prop_FL.uasset",
        "Meshes/SM_Icarus7_Prop_FR.uasset",
        "Meshes/SM_Icarus7_Prop_RL.uasset",
        "Meshes/SM_Icarus7_Prop_RR.uasset",
        "Meshes/SM_Icarus7_RealSenseD435i.uasset",
        "Meshes/SM_Icarus7_LogitechBRIO.uasset",
        "Meshes/SM_Icarus7_MID360.uasset",
    ]
    missing = [name for name in required if not (project / ASSET_SUBDIR / name).is_file()]
    if missing:
        raise SystemExit("missing required Icarus7 assets: " + ", ".join(missing))


def enable_plugins(project_file: Path) -> None:
    descriptor = json.loads(project_file.read_text())
    plugins = descriptor.setdefault("Plugins", [])
    by_name = {item.get("Name"): item for item in plugins}
    for name in ("AirSim", "Icarus7AirSim"):
        if name in by_name:
            by_name[name]["Enabled"] = True
        else:
            plugins.append({"Name": name, "Enabled": True})
    project_file.write_text(json.dumps(descriptor, indent=2) + "\n")


def ensure_always_cook(project_root: Path) -> None:
    config = project_root / "Config/DefaultGame.ini"
    config.parent.mkdir(parents=True, exist_ok=True)
    line = '+DirectoriesToAlwaysCook=(Path="/Game/Icarus7FlightControlReview")'
    text = config.read_text() if config.exists() else ""
    if line in text:
        return
    if text and not text.endswith("\n"):
        text += "\n"
    text += "\n[/Script/UnrealEd.ProjectPackagingSettings]\n" + line + "\n"
    config.write_text(text)


def apply_physics_patch(airsim_source: Path) -> None:
    source = airsim_source.resolve()
    if not (source / ".git").exists():
        raise SystemExit(f"AirSim source is not a git checkout: {source}")
    revision = subprocess.check_output(
        ["git", "-C", str(source), "rev-parse", "HEAD"], text=True).strip()
    if revision != PINNED_AIRSIM:
        raise SystemExit(f"AirSim revision {revision} is not pinned {PINNED_AIRSIM}")
    header = source / "AirLib/include/vehicles/multirotor/firmwares/arducopter/ArduCopterParams.hpp"
    if "setupFrameIcarus7" not in header.read_text():
        check = subprocess.run(
            ["git", "-C", str(source), "apply", "--unidiff-zero", "--check", str(PATCH)],
            text=True, capture_output=True, check=False)
        if check.returncode:
            raise SystemExit("Icarus7 physics patch does not apply cleanly:\n" + check.stderr)
        subprocess.run(
            ["git", "-C", str(source), "apply", "--unidiff-zero", str(PATCH)],
            check=True)


def link_airsim_plugin(airsim_source: Path, project_root: Path) -> None:
    source = airsim_source.resolve() / "Unreal/Plugins/AirSim"
    destination = project_root / "Plugins/AirSim"
    destination.parent.mkdir(parents=True, exist_ok=True)
    if destination.is_symlink() and destination.resolve() == source:
        return
    if destination.exists() or destination.is_symlink():
        raise SystemExit(f"refusing to replace existing AirSim plugin: {destination}")
    destination.symlink_to(source, target_is_directory=True)


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--asset-project", type=Path, required=True,
                        help="Delivered Icarus7 UE4.27 content-project directory")
    parser.add_argument("--environment-project", type=Path, required=True,
                        help="AirSim-enabled environment .uproject to receive the module")
    parser.add_argument("--airsim-source", type=Path,
                        help="Pinned AirSim v1.8.0 checkout to patch for Icarus7 physics")
    parser.add_argument("--link-airsim-plugin", action="store_true",
                        help="Symlink the pinned AirSim plugin into the target source project")
    args = parser.parse_args()

    manifest = json.loads(MANIFEST.read_text())
    assets = resolve_asset_project(args.asset_project)
    verify_asset_project(assets, manifest)

    project_file = args.environment_project.resolve()
    if not project_file.is_file() or project_file.suffix != ".uproject":
        raise SystemExit(f"environment project is not a .uproject: {project_file}")
    target_descriptor = json.loads(project_file.read_text())
    if target_descriptor.get("EngineAssociation") != "4.27":
        raise SystemExit("target AirSim environment must use Unreal Engine 4.27")
    target_root = project_file.parent

    if args.airsim_source:
        apply_physics_patch(args.airsim_source)
    if args.link_airsim_plugin:
        if not args.airsim_source:
            parser.error("--link-airsim-plugin requires --airsim-source")
        link_airsim_plugin(args.airsim_source, target_root)

    shutil.copytree(assets / ASSET_SUBDIR, target_root / ASSET_SUBDIR,
                    dirs_exist_ok=True)
    shutil.copytree(PLUGIN_SOURCE, target_root / "Plugins/Icarus7AirSim",
                    dirs_exist_ok=True)
    enable_plugins(project_file)
    ensure_always_cook(target_root)

    receipt = {
        "schema": "icarus.airsim.vehicle-install.v1",
        "vehicle_id": "icarus7",
        "manifest": str(MANIFEST),
        "asset_project": str(assets),
        "environment_project": str(project_file),
        "airsim_revision": PINNED_AIRSIM if args.airsim_source else None,
        "pawn_class": manifest["pawn_class"],
        "license_status": "user-supplied; redistribution terms not present in asset package",
    }
    (target_root / ".icarus7-module.json").write_text(json.dumps(receipt, indent=2) + "\n")
    print(f"Installed Icarus7 AirSim module into {target_root}")
    print(f"Pawn class: {manifest['pawn_class']}")
    print("Next: compile/cook this project with Unreal Engine 4.27, then run the Icarus validator.")


if __name__ == "__main__":
    main()

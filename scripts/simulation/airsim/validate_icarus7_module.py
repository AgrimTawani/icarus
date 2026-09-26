#!/usr/bin/env python3
"""Validate Icarus7 AirSim assets, settings, native pawn ABI, and physics overlay."""

import argparse
import hashlib
import json
import math
import subprocess
from pathlib import Path

ROOT = Path(__file__).resolve().parents[3]
MODULE = ROOT / "simulation/airsim/vehicles/icarus7"
MANIFEST = MODULE / "manifest.json"
PAWN_CPP = MODULE / "Plugins/Icarus7AirSim/Source/Icarus7AirSim/Private/Icarus7AirSimPawn.cpp"
PATCH = MODULE / "patches/airsim-v1.8.0-icarus7-physics.patch"
SETTINGS = (
    ROOT / "config/simulation/airsim/settings.icarus7.json",
    ROOT / "config/simulation/airsim/settings.icarus7.gui.json",
)


def sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def fail(message: str) -> None:
    raise SystemExit("FAIL: " + message)


def check_settings(manifest: dict) -> None:
    expected_view = ("NoDisplay", "FlyWithMe")
    for path, view in zip(SETTINGS, expected_view):
        value = json.loads(path.read_text())
        vehicle = value.get("Vehicles", {}).get("Icarus", {})
        pawn = value.get("PawnPaths", {}).get("Icarus7", {}).get("PawnBP")
        if value.get("ViewMode") != view:
            fail(f"{path} must use ViewMode={view}")
        if vehicle.get("VehicleType") != "ArduCopter":
            fail(f"{path} is not ArduCopter")
        if vehicle.get("PawnPath") != "Icarus7" or pawn != manifest["pawn_class"]:
            fail(f"{path} does not select the native Icarus7 pawn")
        if vehicle.get("Model") != "Icarus7":
            fail(f"{path} does not select Icarus7 physics")
        if value.get("ApiServerPort") != 41451:
            fail(f"{path} does not use isolated RPC port 41451")
        for sensor in ("Imu", "Gps", "Lidar1", "DistanceDown"):
            if not vehicle.get("Sensors", {}).get(sensor, {}).get("Enabled"):
                fail(f"{path} missing enabled {sensor}")
        for camera in ("0", "3"):
            if camera not in vehicle.get("Cameras", {}):
                fail(f"{path} missing camera {camera}")


def check_pawn_abi() -> None:
    text = PAWN_CPP.read_text()
    for name in (
        "FrontCenterCamera", "FrontRightCamera", "FrontLeftCamera",
        "BackCenterCamera", "BottomCenterCamera",
        "Rotation0", "Rotation1", "Rotation2", "Rotation3",
        "CollisionBody", "CollisionLandingGear",
    ):
        if name not in text and not (name.startswith("Rotation") and "Rotation%d" in text):
            fail(f"native pawn is missing AirSim ABI component {name}")
    order = [text.index('TEXT("FR")'), text.index('TEXT("RL")'),
             text.index('TEXT("FL")'), text.index('TEXT("RR")')]
    if order != sorted(order):
        fail("native pawn rotor order is not FR, RL, FL, RR")


def check_physics(manifest: dict) -> None:
    physics = manifest["physics"]
    mass_properties = json.loads((ROOT / physics["source"]).read_text())
    if not math.isclose(physics["mass_kg"], mass_properties["total_mass_kg"], abs_tol=1e-9):
        fail("AirSim mass diverges from the canonical component-derived mass")
    source_inertia = mass_properties["total_inertia_kg_m2"]
    # FLU -> FRD tensor transform diag(1,-1,-1).
    signs = (1, -1, -1)
    expected = [[source_inertia[i][j] * signs[i] * signs[j]
                 for j in range(3)] for i in range(3)]
    for row, wanted in zip(physics["inertia_frd_kg_m2"], expected):
        for actual, reference in zip(row, wanted):
            if not math.isclose(actual, reference, abs_tol=1e-9):
                fail("AirSim FRD inertia diverges from canonical FLU inertia")
    rotor = physics
    thrust = (rotor["rotor_thrust_coefficient"] * 1.225
              * (rotor["max_rpm"] / 60.0) ** 2
              * rotor["propeller_diameter_m"] ** 4)
    if not math.isclose(thrust, rotor["max_thrust_per_rotor_n"], rel_tol=1e-7):
        fail("AirSim rotor coefficient does not reproduce the frozen thrust point")
    if "setupFrameIcarus7" not in PATCH.read_text():
        fail("AirSim physics patch is missing Icarus7 frame support")


def check_asset_project(path: Path, manifest: dict) -> None:
    project = path.resolve()
    if not list(project.glob("*.uproject")):
        children = [candidate.parent for candidate in project.glob("*/*.uproject")]
        if len(children) != 1:
            fail(f"cannot resolve asset project under {project}")
        project = children[0]
    projects = list(project.glob("*.uproject"))
    expected = manifest["asset_project_sha256"]
    complete = project / "Content/Icarus7FlightControlReview/Meshes/SM_Icarus7_Companion_Complete.uasset"
    fbx = project / "SourceAssets/Icarus7_Companion_Complete.fbx"
    values = {"uproject": projects[0], "complete_uasset": complete, "complete_fbx": fbx}
    for name, candidate in values.items():
        if not candidate.is_file() or sha256(candidate) != expected[name]:
            fail(f"reviewed asset input failed digest validation: {candidate}")


def resolve_environment_project(path: Path) -> Path:
    path = path.resolve()
    if path.is_file() and path.suffix == ".uproject":
        return path
    if path.is_dir():
        projects = list(path.glob("*.uproject"))
        if len(projects) == 1:
            return projects[0]
    fail(f"expected one AirSim environment .uproject at {path}")


def check_installed_environment(project_file: Path) -> None:
    root = project_file.resolve().parent
    required = (
        root / "Plugins/Icarus7AirSim/Icarus7AirSim.uplugin",
        root / "Content/Icarus7FlightControlReview/Meshes/SM_Icarus7_Body.uasset",
        root / "Content/Icarus7FlightControlReview/Meshes/SM_Icarus7_Prop_FR.uasset",
        root / ".icarus7-module.json",
    )
    for path in required:
        if not path.is_file():
            fail(f"installed environment is missing {path}")
    descriptor = json.loads(project_file.read_text())
    enabled = {item.get("Name") for item in descriptor.get("Plugins", []) if item.get("Enabled")}
    if not {"AirSim", "Icarus7AirSim"}.issubset(enabled):
        fail("target environment does not enable AirSim and Icarus7AirSim")


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--asset-project", type=Path,
                        help="Also verify the delivered source asset project and digests")
    parser.add_argument("--environment-project", type=Path,
                        help="Also verify a module-installed AirSim environment directory or .uproject")
    parser.add_argument("--airsim-source", type=Path,
                        help="Also verify the pinned AirSim checkout and applied physics patch")
    args = parser.parse_args()

    manifest = json.loads(MANIFEST.read_text())
    check_settings(manifest)
    check_pawn_abi()
    check_physics(manifest)
    if args.asset_project:
        check_asset_project(args.asset_project, manifest)
    if args.environment_project:
        check_installed_environment(resolve_environment_project(args.environment_project))
    if args.airsim_source:
        revision = subprocess.check_output(
            ["git", "-C", str(args.airsim_source.resolve()), "rev-parse", "HEAD"],
            text=True).strip()
        if revision != manifest["airsim_revision"]:
            fail(f"AirSim revision mismatch: {revision}")
        header = args.airsim_source.resolve() / (
            "AirLib/include/vehicles/multirotor/firmwares/arducopter/ArduCopterParams.hpp")
        if not header.is_file() or "setupFrameIcarus7" not in header.read_text():
            fail("Icarus7 physics patch is not applied to AirSim source")
    print("PASS: Icarus7 AirSim module, pawn ABI, settings, and physics contract are valid")


if __name__ == "__main__":
    main()

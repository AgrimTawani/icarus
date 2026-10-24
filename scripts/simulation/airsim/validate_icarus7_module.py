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
        check_external_sensor_load(value, str(path))


MAX_EXTERNAL_LIDAR_POINTS_PER_SECOND = 10000


def check_external_sensor_load(settings: dict, label: str = "settings") -> None:
    """Sensors with ExternalController are serialised into every SITL packet.

    The proven realistic-city profile streams at most an 8 ch / 10 k pts/s
    LiDAR to ArduPilot; the perception LiDAR stays local and is read over RPC.
    AirSim always writes a "lidar" section when any LiDAR exists and ArduPilot's
    JSON parser aborts at an empty point cloud, dropping the rc/rng keys that
    follow it; so the streamed LiDAR must exist and stay light (2026-10-24).
    """
    for vehicle in settings.get("Vehicles", {}).values():
        for name, sensor in vehicle.get("Sensors", {}).items():
            if sensor.get("SensorType") != 6 or not sensor.get("ExternalController"):
                continue
            if sensor.get("PointsPerSecond", 0) > MAX_EXTERNAL_LIDAR_POINTS_PER_SECOND:
                fail(f"{label}: LiDAR {name} streams {sensor.get('PointsPerSecond')} pts/s "
                     f"to ArduPilot; keep ExternalController false for the perception "
                     f"LiDAR or stay <= {MAX_EXTERNAL_LIDAR_POINTS_PER_SECOND} pts/s")


SITL_PARAMS = ROOT / "config/simulation/airsim/icarus7-sitl.parm"
# Icarus7 physics overlay (ArduCopterParams.hpp, setupFrameIcarus7): mass
# 4.4745 kg against 4 rotors of 45.6 N (C_T 0.1321, 15606 rpm, 0.254 m).
ICARUS7_MASS_KG = 4.4745
ICARUS7_ROTOR_MAX_THRUST_N = 0.1321402534 * 1.225 * (15606.0 / 60) ** 2 * 0.254 ** 4
ICARUS7_HOVER_PWM_FRACTION = ICARUS7_MASS_KG * 9.80665 / (4 * ICARUS7_ROTOR_MAX_THRUST_N)
ARDUPILOT_HOVER_THROTTLE_FLOOR = 0.125


def load_sitl_params(path: Path) -> dict:
    params = {}
    for line in path.read_text().splitlines():
        line = line.split("#", 1)[0].strip()
        if line:
            name, value = line.split()
            params[name] = float(value)
    return params


def check_sitl_hover_tuning(params: dict) -> None:
    """AirSim rotor thrust is linear in the PWM fraction (MOT_THST_EXPO 0).

    ArduPilot maps throttle t to PWM fraction SPIN_MIN + t * (1 - SPIN_MIN) and
    clamps MOT_THST_HOVER at 0.125. With T/W 4.15 the Icarus7 hovers at a PWM
    fraction of 0.24, below that floor under the default SPIN_MIN 0.15: a 5 m
    GUIDED takeoff climbed to 57 m with throttle at 0 (2026-10-24).
    """
    for name in ("MOT_SPIN_MIN", "MOT_THST_HOVER", "PSC_ACCZ_P", "PSC_ACCZ_I"):
        if name not in params:
            fail(f"icarus7-sitl.parm must set {name}")
    spin_min = params["MOT_SPIN_MIN"]
    hover = (ICARUS7_HOVER_PWM_FRACTION - spin_min) / (1.0 - spin_min)
    if hover < ARDUPILOT_HOVER_THROTTLE_FLOOR:
        fail(f"MOT_SPIN_MIN {spin_min} puts the Icarus7 hover throttle at {hover:.3f}, "
             f"below ArduPilot's MOT_THST_HOVER floor {ARDUPILOT_HOVER_THROTTLE_FLOOR}")
    if not math.isclose(params["MOT_THST_HOVER"], hover, abs_tol=0.01):
        fail(f"MOT_THST_HOVER {params['MOT_THST_HOVER']} does not match the Icarus7 "
             f"thrust model hover throttle {hover:.3f}")
    if not math.isclose(params["PSC_ACCZ_P"], params["MOT_THST_HOVER"], abs_tol=1e-6):
        fail("PSC_ACCZ_P must equal MOT_THST_HOVER (ArduPilot tuning guide)")
    if not math.isclose(params["PSC_ACCZ_I"], 2 * params["MOT_THST_HOVER"], abs_tol=1e-6):
        fail("PSC_ACCZ_I must equal 2 x MOT_THST_HOVER (ArduPilot tuning guide)")


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


def check_pawn_runtime_contract(text: str) -> None:
    """Rules learned from the first packaged Blocks run (2026-10-24)."""
    if 'CreateDefaultSubobject<UBoxComponent>(TEXT("VehicleRoot"))' not in text:
        fail("native pawn root must be a blocking UBoxComponent: AirSim sweeps "
             "the root on every move and a scene-component root falls through "
             "the world")
    if "/AirSim/Blueprints/BP_PIPCamera" not in text:
        fail("native pawn cameras must spawn BP_PIPCamera: bare APIPCamera has "
             "no capture components and aborts at spawn")


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
    check_pawn_runtime_contract(PAWN_CPP.read_text())
    check_physics(manifest)
    check_sitl_hover_tuning(load_sitl_params(SITL_PARAMS))
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

import json
import subprocess
import tempfile
import unittest
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
MODULE = ROOT / "simulation/airsim/vehicles/icarus7"
VALIDATOR = ROOT / "scripts/simulation/airsim/validate_icarus7_module.py"
START = ROOT / "scripts/simulation/airsim/start_icarus7.py"


class Icarus7AirSimModuleTests(unittest.TestCase):
    def test_static_vehicle_contract_passes(self):
        result = subprocess.run(
            ["/usr/bin/python3", str(VALIDATOR)], text=True,
            capture_output=True, check=False)
        self.assertEqual(result.returncode, 0, result.stdout + result.stderr)
        self.assertIn("pawn ABI", result.stdout)

    def test_settings_select_native_pawn_and_physics(self):
        manifest = json.loads((MODULE / "manifest.json").read_text())
        for name, view_mode in (
            ("settings.icarus7.json", "NoDisplay"),
            ("settings.icarus7.gui.json", "FlyWithMe"),
        ):
            settings = json.loads(
                (ROOT / "config/simulation/airsim" / name).read_text())
            vehicle = settings["Vehicles"]["Icarus"]
            self.assertEqual(settings["ViewMode"], view_mode)
            self.assertEqual(vehicle["VehicleType"], "ArduCopter")
            self.assertEqual(vehicle["PawnPath"], "Icarus7")
            self.assertEqual(vehicle["Model"], "Icarus7")
            self.assertEqual(
                settings["PawnPaths"]["Icarus7"]["PawnBP"],
                manifest["pawn_class"])

    def test_native_pawn_exposes_required_airsim_components(self):
        source = (MODULE / (
            "Plugins/Icarus7AirSim/Source/Icarus7AirSim/Private/"
            "Icarus7AirSimPawn.cpp")).read_text()
        for camera in (
            "FrontCenterCamera", "FrontRightCamera", "FrontLeftCamera",
            "BackCenterCamera", "BottomCenterCamera",
        ):
            self.assertIn(camera, source)
        self.assertIn('TEXT("Rotation%d")', source)
        self.assertLess(source.index('TEXT("FR")'), source.index('TEXT("RL")'))
        self.assertLess(source.index('TEXT("RL")'), source.index('TEXT("FL")'))
        self.assertLess(source.index('TEXT("FL")'), source.index('TEXT("RR")'))

    def test_launcher_refuses_an_unverified_package(self):
        with tempfile.TemporaryDirectory() as temp:
            result = subprocess.run(
                ["/usr/bin/python3", str(START), "--install-dir", temp],
                text=True, capture_output=True, check=False)
        self.assertNotEqual(result.returncode, 0)
        self.assertIn("build receipt not found", result.stderr)

    def test_public_wrappers_are_executable(self):
        for name in (
            "install-airsim-icarus7-module",
            "validate-airsim-icarus7-module",
            "package-airsim-icarus7-environment",
            "start-airsim-icarus7",
            "test-airsim-icarus7-airlib",
        ):
            path = ROOT / "scripts" / name
            self.assertTrue(path.is_file(), path)
            self.assertTrue(path.stat().st_mode & 0o100, path)


if __name__ == "__main__":
    unittest.main()

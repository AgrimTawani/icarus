import json
import subprocess
import tempfile
import unittest
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
DOCTOR = ROOT / "scripts/simulation/airsim/doctor.py"
START = ROOT / "scripts/simulation/airsim/start.py"


class AirSimScriptTests(unittest.TestCase):
    def _settings(self, directory):
        path = Path(directory) / "settings.json"
        path.write_text(json.dumps({
            "SettingsVersion": 1.2,
            "ViewMode": "NoDisplay",
            "ApiServerPort": 41451,
            "Vehicles": {"Icarus": {"VehicleType": "ArduCopter"}},
        }))
        return path

    def _binary(self, directory):
        path = Path(directory) / "AirSim.sh"
        path.write_text("#!/usr/bin/env bash\nexit 0\n")
        path.chmod(0o755)
        return path

    def test_doctor_accepts_arducopter_no_display_configuration(self):
        with tempfile.TemporaryDirectory() as temp:
            result = subprocess.run(
                ["/usr/bin/python3", str(DOCTOR), "--binary", str(self._binary(temp)),
                 "--settings", str(self._settings(temp))], text=True,
                capture_output=True, check=False)
        self.assertEqual(result.returncode, 0, result.stderr)
        self.assertIn("structurally valid", result.stdout)

    def test_doctor_rejects_simpleflight(self):
        with tempfile.TemporaryDirectory() as temp:
            settings = self._settings(temp)
            value = json.loads(settings.read_text())
            value["Vehicles"]["Icarus"]["VehicleType"] = "SimpleFlight"
            settings.write_text(json.dumps(value))
            result = subprocess.run(
                ["/usr/bin/python3", str(DOCTOR), "--binary", str(self._binary(temp)),
                 "--settings", str(settings)], text=True, capture_output=True,
                check=False)
        self.assertNotEqual(result.returncode, 0)
        self.assertIn("never SimpleFlight", result.stdout)

    def test_launcher_refuses_nullrhi_for_a_visual_run(self):
        result = subprocess.run(
            ["/usr/bin/python3", str(START), "--binary", "/does/not/exist",
             "--nullrhi"], text=True, capture_output=True, check=False)
        self.assertEqual(result.returncode, 2)
        self.assertIn("requires --no-vision", result.stderr)

    def test_public_wrappers_and_track_readmes_exist(self):
        for path in (
            ROOT / "scripts/start-airsim", ROOT / "scripts/stop-airsim",
            ROOT / "scripts/airsim-doctor", ROOT / "scripts/fetch-airsim-blocks",
            ROOT / "scripts/airsim-smoke", ROOT / "scripts/simulation/airsim/README.md",
            ROOT / "scripts/simulation/gazebo/README.md", ROOT / "scripts/README.md",
            ROOT / "config/simulation/airsim/blocks-v1.8.0.json",
        ):
            self.assertTrue(path.is_file(), path)

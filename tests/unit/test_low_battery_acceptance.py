import ast
import unittest
from pathlib import Path


class LowBatteryAcceptanceContractTests(unittest.TestCase):
    def test_acceptance_client_uses_only_the_typed_drone_api(self):
        root = Path(__file__).resolve().parents[2]
        source = root / "scripts/autonomy/run_low_battery_acceptance.py"
        tree = ast.parse(source.read_text())
        imports = {
            alias.name for node in ast.walk(tree)
            if isinstance(node, (ast.Import, ast.ImportFrom))
            for alias in getattr(node, "names", [])
        }
        self.assertFalse(any("mavlink" in item.lower() for item in imports))

    def test_low_battery_gate_requires_the_adverse_scenario(self):
        root = Path(__file__).resolve().parents[2]
        source = (root / "scripts/autonomy/run_low_battery_acceptance.py").read_text()
        self.assertIn('session.get("scenario") != "adverse_combined"', source)
        self.assertIn("REASON_CODE_BATTERY_BELOW_THRESHOLD", source)

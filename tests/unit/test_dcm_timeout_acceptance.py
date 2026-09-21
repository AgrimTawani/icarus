import ast
import unittest
from pathlib import Path


class DcmTimeoutAcceptanceTests(unittest.TestCase):
    def test_timeout_acceptance_has_no_direct_mavlink_dependency(self):
        root = Path(__file__).resolve().parents[2]
        source = root / "scripts/autonomy/run_dcm_timeout_acceptance.py"
        tree = ast.parse(source.read_text())
        imports = {
            alias.name for node in ast.walk(tree)
            if isinstance(node, (ast.Import, ast.ImportFrom))
            for alias in getattr(node, "names", [])
        }
        self.assertFalse(any("mavlink" in item.lower() for item in imports))

    def test_timeout_runtime_is_followed_by_safe_none(self):
        root = Path(__file__).resolve().parents[2]
        source = (root / "scripts/autonomy/run_dcm_timeout_acceptance.py").read_text()
        self.assertIn("raise DeadlineExceeded", source)
        self.assertIn('"action":"none"', source)
        self.assertIn('result["counts"]["executed"] != 0', source)

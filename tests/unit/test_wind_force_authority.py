"""Keep Icarus's custom atmosphere as the only wind-force authority."""

from pathlib import Path
import unittest


ROOT = Path(__file__).resolve().parents[2]


class WindForceAuthorityTest(unittest.TestCase):
    def test_compact_model_does_not_opt_into_native_wind(self):
        source = (ROOT / "scripts/simulation/build_compact_flight.py").read_text()
        model = (ROOT / "simulation/models/akshu_compact_sitl/model.sdf").read_text()

        self.assertIn("IcarusTurbulentAtmosphere", source)
        self.assertIn("IcarusTurbulentAtmosphere", model)
        self.assertNotIn('ET.SubElement(base, "enable_wind")', source)
        self.assertNotIn('ET.SubElement(rotor, "enable_wind")', source)
        self.assertNotIn("<enable_wind>true</enable_wind>", model)


if __name__ == "__main__":
    unittest.main()

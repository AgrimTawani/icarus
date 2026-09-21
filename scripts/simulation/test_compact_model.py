#!/usr/bin/python3
"""Fast structural regression checks for the composed flight SDF."""

import json
import unittest
import xml.etree.ElementTree as ET

import numpy as np
from build_akshu_candidate import ROOT


class CompactModelTests(unittest.TestCase):
    def setUp(self):
        self.model = (
            ET.parse(ROOT / "simulation/models/akshu_compact_sitl/model.sdf")
            .getroot()
            .find("model")
        )

    def test_mass_and_positive_inertia(self):
        self.assertAlmostEqual(
            sum(float(x.text) for x in self.model.findall("link/inertial/mass")), 4.343
        )
        for link in self.model.findall("link"):
            inertia = link.find("inertial/inertia")
            values = {x.tag: float(x.text) for x in inertia}
            matrix = np.array(
                [
                    [values["ixx"], values["ixy"], values["ixz"]],
                    [values["ixy"], values["iyy"], values["iyz"]],
                    [values["ixz"], values["iyz"], values["izz"]],
                ]
            )
            eigen = np.linalg.eigvalsh(matrix)
            self.assertGreater(eigen.min(), 0)
            self.assertLessEqual(eigen.max(), sum(eigen) - eigen.max() + 1e-12)

    def test_rotor_layout_and_visual_positions(self):
        layout = json.loads(
            (ROOT / "simulation/models/akshu_reference/extraction.json").read_text()
        )["motor_layout"]
        for m in layout:
            link = self.model.find(f"link[@name='motor_{m['motor']:02d}']")
            pose = np.array([float(v) for v in link.findtext("pose").split()[:3]])
            np.testing.assert_allclose(pose, m["position_m"])
            visual = link.find("visual")
            offset = np.array([float(v) for v in visual.findtext("pose").split()[:3]])
            np.testing.assert_allclose(pose + offset, [0, 0, 0], atol=1e-12)
        self.assertEqual(len(self.model.findall("joint")), 4)
        self.assertFalse(
            any(
                v.get("name").startswith("source_prop")
                for v in self.model.findall("link[@name='base_link']/visual")
            )
        )

    def test_sensor_frames_and_control_mapping(self):
        base = self.model.find("link[@name='base_link']")
        self.assertEqual(len(base.findall("sensor")), 9)
        down_camera = base.find("sensor[@name='rgbd_down']")
        self.assertIsNotNone(down_camera)
        self.assertEqual(down_camera.get("type"), "rgbd_camera")
        self.assertEqual(down_camera.findtext("topic"), "/icarus/sensors/rgbd_down")
        self.assertAlmostEqual(
            float(down_camera.findtext("pose").split()[4]), np.pi / 2
        )
        self.assertEqual(
            base.find("sensor[@name='imu_sensor']/update_rate").text, "1000"
        )
        self.assertAlmostEqual(
            float(base.find("sensor[@name='imu_sensor']/pose").text.split()[3]), np.pi
        )
        self.assertEqual(
            base.find("sensor[@name='imu']/pose").text.split()[3:], ["0", "0", "0"]
        )
        bridge = self.model.find("plugin[@filename='ArduPilotPlugin']")
        for i, control in enumerate(bridge.findall("control")):
            self.assertEqual(control.get("channel"), str(i))
            self.assertEqual(control.findtext("jointName"), f"rotor_{i + 1:02d}_joint")
        self.assertEqual(self.model.findtext("static"), "false")


if __name__ == "__main__":
    unittest.main()

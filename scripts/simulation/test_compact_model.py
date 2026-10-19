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
            sum(float(x.text) for x in self.model.findall("link/inertial/mass")), 4.4745
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
        # 12 hardware-facing sensors are generated on the visual model, four
        # redundant controller/environment sensors are added by the physical
        # profile, and one FRD IMU is reserved for the ArduPilot bridge.
        self.assertEqual(len(base.findall("sensor")), 17)
        forward_camera = base.find("sensor[@name='d435i']")
        self.assertEqual(forward_camera.get("type"), "rgbd_camera")
        self.assertEqual(forward_camera.findtext("camera/image/width"), "848")
        self.assertEqual(forward_camera.findtext("camera/image/height"), "480")
        self.assertEqual(forward_camera.findtext("update_rate"), "30")
        self.assertEqual(forward_camera.findtext("camera/clip/far"), "100")
        self.assertEqual(
            forward_camera.findtext("camera/depth_camera/clip/far"), "10")
        down_camera = base.find("sensor[@name='brio_down']")
        self.assertIsNotNone(down_camera)
        self.assertEqual(down_camera.get("type"), "camera")
        self.assertEqual(down_camera.findtext("topic"), "/icarus/sensors/brio_down")
        self.assertEqual(down_camera.findtext("camera/image/width"), "1280")
        self.assertIsNone(base.find("sensor[@name='brio_down']/camera/depth_camera"))
        self.assertAlmostEqual(
            float(down_camera.findtext("pose").split()[4]), np.pi / 2
        )
        landing = base.find("sensor[@name='landing_depth_proxy']")
        self.assertEqual(landing.get("type"), "rgbd_camera")
        self.assertEqual(landing.findtext("camera/image/width"), "320")
        lidar = base.find("sensor[@name='mid360']/lidar/scan")
        self.assertEqual(lidar.findtext("horizontal/samples"), "500")
        self.assertEqual(lidar.findtext("vertical/samples"), "40")
        self.assertAlmostEqual(float(lidar.findtext("vertical/min_angle")), np.deg2rad(-7))
        self.assertAlmostEqual(float(lidar.findtext("vertical/max_angle")), np.deg2rad(52))
        ark = base.find("sensor[@name='ark_flow_range']/lidar")
        self.assertEqual(ark.findtext("scan/horizontal/samples"), "8")
        self.assertEqual(ark.findtext("scan/vertical/samples"), "4")
        self.assertEqual(ark.findtext("range/min"), "0.08")
        self.assertEqual(ark.findtext("range/max"), "30")
        for name, rate in (
            ("pixhawk_imu_primary", "500"),
            ("pixhawk_imu_secondary", "500"),
            ("pixhawk_imu_tertiary", "200"),
            ("d435i_imu", "200"),
            ("mid360_imu", "200"),
            ("ark_flow_imu", "200"),
            ("h_rtk_gnss", "8"),
            ("h_rtk_compass", "50"),
            ("pixhawk_magnetometer", "50"),
            ("pixhawk_barometer", "30"),
            ("pixhawk_barometer_secondary", "30"),
        ):
            self.assertEqual(base.find(f"sensor[@name='{name}']/update_rate").text, rate)
        self.assertEqual(
            base.find("sensor[@name='imu_sensor']/update_rate").text, "1000"
        )
        self.assertAlmostEqual(
            float(base.find("sensor[@name='imu_sensor']/pose").text.split()[3]), np.pi
        )
        self.assertEqual(
            base.find("sensor[@name='pixhawk_imu_primary']/pose").text.split()[3:], ["0", "0", "0"]
        )
        bridge = self.model.find("plugin[@filename='ArduPilotPlugin']")
        for i, control in enumerate(bridge.findall("control")):
            self.assertEqual(control.get("channel"), str(i))
            self.assertEqual(control.findtext("jointName"), f"rotor_{i + 1:02d}_joint")
        self.assertEqual(self.model.findtext("static"), "false")


if __name__ == "__main__":
    unittest.main()

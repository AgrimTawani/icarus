import json
import subprocess
import tempfile
import unittest
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
MODULE = ROOT / "simulation/airsim/vehicles/icarus7"
VALIDATOR = ROOT / "scripts/simulation/airsim/validate_icarus7_module.py"
START = ROOT / "scripts/simulation/airsim/start_icarus7.py"
PAWN_CPP = MODULE / "Plugins/Icarus7AirSim/Source/Icarus7AirSim/Private/Icarus7AirSimPawn.cpp"


def _load_validator():
    import importlib.util
    spec = importlib.util.spec_from_file_location("validate_icarus7_module", VALIDATOR)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


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

    def test_native_pawn_root_is_a_blocking_primitive(self):
        # AirSim moves the pawn with a swept SetActorLocationAndRotation on the
        # root component; a plain USceneComponent root falls through the world.
        validator = _load_validator()
        source = PAWN_CPP.read_text()
        validator.check_pawn_runtime_contract(source)
        broken = source.replace(
            'CreateDefaultSubobject<UBoxComponent>(TEXT("VehicleRoot"))',
            'CreateDefaultSubobject<USceneComponent>(TEXT("VehicleRoot"))')
        with self.assertRaises(SystemExit):
            validator.check_pawn_runtime_contract(broken)

    def test_native_pawn_cameras_use_the_airsim_blueprint(self):
        # Bare APIPCamera has no capture components and aborts at spawn with
        # "Cast of nullptr to SceneCaptureComponent2D failed".
        validator = _load_validator()
        source = PAWN_CPP.read_text()
        broken = source.replace("/AirSim/Blueprints/BP_PIPCamera", "/AirSim/Blueprints/Missing")
        with self.assertRaises(SystemExit):
            validator.check_pawn_runtime_contract(broken)

    def test_settings_do_not_stream_the_perception_lidar_to_ardupilot(self):
        # Mirror the proven realistic-city layout: a light externally streamed
        # LiDAR for ArduPilot plus a separate perception LiDAR. ArduPilot's
        # JSON parser aborts at an empty "lidar" point cloud and drops the
        # rc/rng keys that follow it, so the perception LiDAR must not be the
        # only one, and the streamed one must stay light.
        validator = _load_validator()
        for path in validator.SETTINGS:
            settings = json.loads(path.read_text())
            validator.check_external_sensor_load(settings)
            heavy = json.loads(path.read_text())
            lidar = heavy["Vehicles"]["Icarus"]["Sensors"]["Lidar1"]
            lidar["ExternalController"] = True
            lidar["PointsPerSecond"] = 50000
            with self.assertRaises(SystemExit):
                validator.check_external_sensor_load(heavy)

    def test_sitl_hover_tuning_matches_the_icarus7_thrust_model(self):
        # Icarus7: 4 x 45.6 N against 4.4745 kg (T/W 4.15). AirSim thrust is
        # linear in the PWM fraction, so hover is a PWM fraction of 0.241; the
        # ArduPilot throttle for that must clear the MOT_THST_HOVER floor of
        # 0.125, and PSC_ACCZ_P/I must scale with it (ArduPilot tuning guide).
        # Untuned, a 5 m GUIDED takeoff climbed to 57 m with throttle at 0.
        validator = _load_validator()
        params = validator.load_sitl_params(validator.SITL_PARAMS)
        validator.check_sitl_hover_tuning(params)
        hover_pwm = validator.ICARUS7_HOVER_PWM_FRACTION
        self.assertAlmostEqual(hover_pwm, 0.2407, places=3)
        spin_min = params["MOT_SPIN_MIN"]
        expected = (hover_pwm - spin_min) / (1.0 - spin_min)
        self.assertAlmostEqual(params["MOT_THST_HOVER"], expected, places=2)
        self.assertGreaterEqual(params["MOT_THST_HOVER"], 0.125)
        self.assertEqual(params["PSC_ACCZ_P"], params["MOT_THST_HOVER"])
        self.assertEqual(params["PSC_ACCZ_I"], 2 * params["MOT_THST_HOVER"])
        for broken in (
            {**params, "MOT_SPIN_MIN": 0.15},                       # hover < floor
            {**params, "PSC_ACCZ_P": 0.5},                          # untuned gain
            {k: v for k, v in params.items() if k != "MOT_THST_HOVER"},
        ):
            with self.assertRaises(SystemExit):
                validator.check_sitl_hover_tuning(broken)

    def test_launcher_passes_the_icarus7_sitl_params(self):
        text = START.read_text()
        self.assertIn("icarus7-sitl.parm", text)
        self.assertIn("--sitl-param-file", text)
        stack = (ROOT / "scripts/simulation/airsim/start_stack.py").read_text()
        self.assertIn("--sitl-param-file", stack)

    def test_packager_rejects_airlib_built_against_libstdcxx(self):
        # Unreal links with its bundled clang + libc++ (std::__1). AirLib built
        # with GCC (std::__cxx11) compiles fine and fails only at link time,
        # an hour into the build. Catch it before RunUAT starts.
        import importlib.util
        spec = importlib.util.spec_from_file_location(
            "package_icarus7_environment",
            ROOT / "scripts/simulation/airsim/package_icarus7_environment.py")
        packager = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(packager)
        with tempfile.TemporaryDirectory() as temp:
            gcc_lib = Path(temp) / "libAirLib.a"
            gcc_lib.write_bytes(b"!<arch>\n_ZN3msr6airlib3FooENSt7__cxx1112basic_stringE")
            with self.assertRaises(SystemExit):
                packager.check_airlib_abi([gcc_lib])
            clang_lib = Path(temp) / "libMavLinkCom.a"
            clang_lib.write_bytes(b"!<arch>\n_ZN3msr6airlib3FooENSt3__112basic_stringE")
            packager.check_airlib_abi([clang_lib])

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

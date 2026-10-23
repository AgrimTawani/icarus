"""AirSim camera calibration comes from the settings file the simulator runs with."""
import json
import math

import pytest

from python.dcm import camera_geometry
from python.perception.target_localization import CameraModel, airsim_localization_config, load_localization_config

SETTINGS = 'config/simulation/airsim/settings.realistic-city.json'


@pytest.fixture
def airsim():
    camera_geometry.use_airsim_settings(SETTINGS)
    yield
    camera_geometry.use_airsim_settings(None)


def test_airsim_mount_converts_frd_degrees_to_flu_radians(tmp_path):
    settings = {'Vehicles': {'Icarus': {'Cameras': {'0': {
        'X': 0.35, 'Y': 0.1, 'Z': 0.2, 'Pitch': -20, 'Roll': 0, 'Yaw': 5,
        'CaptureSettings': [{'ImageType': 1, 'Width': 32, 'Height': 18},
                            {'ImageType': 0, 'Width': 640, 'Height': 360, 'FOV_Degrees': 90}]}}}}}
    path = tmp_path / 'settings.json'
    path.write_text(json.dumps(settings))
    mount = camera_geometry.airsim_camera(path)
    assert (mount['width'], mount['height'], mount['horizontal_fov_deg']) == (640, 360, 90.0)
    x, y, z, _, pitch, yaw = mount['pose_flu']
    assert (x, y, z) == (0.35, -0.1, -0.2)
    assert math.degrees(pitch) == pytest.approx(20)  # looks down
    assert math.degrees(yaw) == pytest.approx(-5)    # looks right


def test_geometry_uses_the_airsim_camera_while_selected(airsim):
    cal = camera_geometry.camera_calibration()
    assert cal['available'] and (cal['width'], cal['height']) == (640, 360)
    assert math.degrees(cal['horizontal_fov_rad']) == pytest.approx(90)
    # A point 20 deg below the nose is at the image centre of the 20 deg down-tilted camera.
    state = {'local_position_ned': {'north_m': 0, 'east_m': 0, 'down_m': -5, 'origin_id': 'o'},
             'attitude': {'roll_rad': 0, 'pitch_rad': 0, 'yaw_rad': 0}}
    drop = (10 - 0.35) * math.tan(math.radians(20))  # camera sits 0.35 m ahead of the body centre
    point = camera_geometry.project_point(state, {'north_m': 10, 'east_m': 0, 'down_m': -5 + drop, 'origin_id': 'o'})
    assert point['in_frame'] and point['pixel_y'] == pytest.approx(180, abs=0.5)


def test_geometry_returns_to_gazebo_when_cleared(airsim):
    camera_geometry.use_airsim_settings(None)
    assert camera_geometry.camera_calibration()['width'] == 848


def test_localizer_uses_the_airsim_camera_and_keeps_association_policy():
    base = load_localization_config('config/perception/target_localization.json')
    config = airsim_localization_config(base, camera_geometry.airsim_camera(SETTINGS))
    model = CameraModel.from_config(config['camera'])
    assert (model.width_px, model.height_px, model.horizontal_fov_deg) == (640, 360, 90.0)
    assert model.pitch_offset_deg == pytest.approx(20)
    assert model.row_px(math.radians(20), 0.0) == pytest.approx(180)
    assert config['association'] == base['association']

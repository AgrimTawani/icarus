"""Pure contract tests for the Gazebo camera-viewer pipeline."""

import importlib.util
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
SPEC = importlib.util.spec_from_file_location(
    "icarus_view_camera", ROOT / "scripts/simulation/view_camera.py")
MODULE = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(MODULE)


def test_two_camera_grid_declares_normal_window_size_before_frames_arrive():
    streams = [
        {"port": 5600, "label": "Forward", "width": 640, "height": 480},
        {"port": 5601, "label": "Downward", "width": 640, "height": 480},
    ]
    pipeline = MODULE.receiver_pipeline(streams, 75)
    assert "video/x-raw,width=1280,height=480,pixel-aspect-ratio=1/1" in pipeline
    assert "sink_1::xpos=640" in pipeline


def test_single_camera_viewer_still_uses_full_frame_dimensions():
    pipeline = MODULE.receiver_pipeline(
        [{"port": 5600, "label": "Forward", "width": 848, "height": 480}], 75)
    assert "video/x-raw,width=848,height=480,pixel-aspect-ratio=1/1" in pipeline

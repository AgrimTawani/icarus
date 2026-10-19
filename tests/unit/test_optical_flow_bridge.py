import importlib.util
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[2]
SPEC = importlib.util.spec_from_file_location(
    "optical_flow_bridge", ROOT / "scripts/simulation/optical_flow_bridge.py")
MODULE = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(MODULE)


def test_grayscale_downsample_validates_and_bounds_resolution():
    rgb = np.zeros((72, 128, 3), dtype=np.uint8)
    rgb[..., 0] = 100
    gray = MODULE.grayscale_downsample(rgb.tobytes(), 128, 72, 128 * 3, 64)
    assert gray.shape == (36, 64)
    assert np.allclose(gray, 29.9, atol=1e-5)


def test_phase_correlation_recovers_known_image_displacement():
    rng = np.random.default_rng(42)
    previous = (rng.normal(size=(90, 160)) * 20).astype(np.float32)
    current = np.roll(previous, shift=(3, -4), axis=(0, 1))
    dx, dy, quality = MODULE.phase_correlation(previous, current)
    assert np.allclose((dx, dy), (-4.0, 3.0), atol=0.05)
    assert quality > 100


def test_textureless_frame_has_zero_quality():
    blank = np.zeros((30, 40), dtype=np.float32)
    assert MODULE.phase_correlation(blank, blank) == (0.0, 0.0, 0)


def test_sensor_noise_is_not_optical_motion():
    rng = np.random.default_rng(15)
    a, b = rng.normal(80, 1.3, size=(2, 90, 160)).astype(np.float32)
    assert MODULE.phase_correlation(a, b)[2] == 0


def test_flow_uses_pinhole_focal_length_and_body_frd_signs():
    import math
    focal = 160 / (2 * math.tan(MODULE.HORIZONTAL_FOV_RAD / 2))
    flow = MODULE.flow_integrals(2, -3, 160, 90)
    assert np.isclose(flow["x"], 2 / focal)
    assert np.isclose(flow["y"], -3 / focal)

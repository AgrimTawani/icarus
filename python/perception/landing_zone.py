"""Deterministic landing-surface assessment from a downward depth frame.

This is intentionally geometry, not an LLM or a VLM.  A landing surface is
only considered when the frame's optical axis is calibrated as downward in the
body-FRD frame.  Applying it to Icarus's present forward RGB-D camera would be
unsafe, so that configuration is rejected rather than producing a plausible
but meaningless quality score.
"""

from __future__ import annotations

import json
import math
import time
from pathlib import Path

import numpy as np


SCHEMA = "icarus.perception.landing_zone.v1"


def load_landing_source(path):
    """Load the versioned depth-source calibration used for assessment.

    The source is data rather than a hard-coded simulator assumption so a
    physical downward camera can replace only this adapter configuration.
    """
    source = json.loads(Path(path).read_text())
    if source.get("version") != 1 or set(source) != {"version", "active_source"}:
        raise ValueError("invalid landing source configuration")
    active = source["active_source"]
    required = {"sensor_id", "topic", "optical_axis_body_frd",
                "horizontal_fov_deg", "vertical_fov_deg"}
    if set(active) != required or not isinstance(active["sensor_id"], str):
        raise ValueError("invalid active landing source")
    if not isinstance(active["topic"], str) or not active["topic"].startswith("/"):
        raise ValueError("landing source topic must be absolute")
    _as_axis(active["optical_axis_body_frd"])
    for name in ("horizontal_fov_deg", "vertical_fov_deg"):
        if not isinstance(active[name], (int, float)) or not 1 <= active[name] <= 179:
            raise ValueError("invalid landing source " + name)
    return active


def _as_axis(values):
    axis = np.asarray(values, dtype=np.float64)
    if axis.shape != (3,) or not np.isfinite(axis).all():
        raise ValueError("optical_axis_body_frd must be three finite numbers")
    norm = float(np.linalg.norm(axis))
    if norm < 1e-9:
        raise ValueError("optical_axis_body_frd must not be zero")
    return axis / norm


def _camera_points(depth, horizontal_fov_deg, vertical_fov_deg):
    """Return finite optical-frame points from a depth map.

    Gazebo RGB-D uses float32 range along the optical z axis.  Invalid,
    nonfinite and out-of-range samples are discarded before plane fitting.
    """
    if depth.ndim != 2 or min(depth.shape) < 8:
        raise ValueError("depth_m must be a 2-D frame of at least 8 by 8 pixels")
    if not 1.0 <= horizontal_fov_deg <= 179.0:
        raise ValueError("horizontal_fov_deg must be in [1, 179]")
    if not 1.0 <= vertical_fov_deg <= 179.0:
        raise ValueError("vertical_fov_deg must be in [1, 179]")
    height, width = depth.shape
    fx = width / (2.0 * math.tan(math.radians(horizontal_fov_deg) / 2.0))
    fy = height / (2.0 * math.tan(math.radians(vertical_fov_deg) / 2.0))
    rows, cols = np.indices(depth.shape, dtype=np.float64)
    z = depth.astype(np.float64, copy=False)
    valid = np.isfinite(z) & (z > 0.05) & (z < 100.0)
    x = (cols - (width - 1) / 2.0) * z / fx
    y = (rows - (height - 1) / 2.0) * z / fy
    return np.column_stack((x[valid], y[valid], z[valid])), valid


def assess_landing_zone(
    depth_m,
    *,
    optical_axis_body_frd,
    horizontal_fov_deg=87.0,
    vertical_fov_deg=58.0,
    vehicle_radius_m=0.35,
    required_clearance_m=0.15,
    max_slope_deg=8.0,
    max_roughness_m=0.04,
):
    """Assess whether the central downward depth footprint is a landing zone.

    The result is an evidence record, not a command.  It measures valid depth
    coverage, least-squares plane slope and robust residual roughness.  The
    algorithm deliberately returns ``assessable: false`` for a forward or
    poorly calibrated camera, and ``suitable: false`` for insufficient data.
    """
    if vehicle_radius_m <= 0 or required_clearance_m < 0:
        raise ValueError("vehicle radius and clearance must be non-negative")
    if not 0 < max_slope_deg < 45 or max_roughness_m <= 0:
        raise ValueError("landing thresholds are outside supported bounds")
    axis = _as_axis(optical_axis_body_frd)
    downward_cosine = float(axis[2])  # body FRD +Z is down.
    base = {
        "schema": SCHEMA,
        "observed_at_unix_ms": int(time.time() * 1000),
        "vehicle_radius_m": float(vehicle_radius_m),
        "required_clearance_m": float(required_clearance_m),
        "optical_axis_body_frd": [round(float(item), 6) for item in axis],
    }
    # Fifteen degrees is deliberately generous for a calibrated gimbal while
    # still rejecting a nose/forward camera (cosine = 0).
    if downward_cosine < math.cos(math.radians(15.0)):
        return base | {
            "assessable": False,
            "suitable": False,
            "reason": "depth source is not calibrated downward",
            "quality": 0.0,
        }

    depth = np.asarray(depth_m, dtype=np.float64)
    _, valid = _camera_points(depth, horizontal_fov_deg, vertical_fov_deg)
    # Select the physical touchdown footprint, not a fixed fraction of the
    # image.  Its radius includes vehicle body and obstacle clearance, so a
    # result cannot silently assess a surface too small for the airframe.
    footprint_radius_m = vehicle_radius_m + required_clearance_m
    height, width = depth.shape
    reference_range_m = float(np.median(depth[valid])) if valid.any() else 1.0
    fx = width / (2.0 * math.tan(math.radians(horizontal_fov_deg) / 2.0))
    fy = height / (2.0 * math.tan(math.radians(vertical_fov_deg) / 2.0))
    rows, cols = np.indices(depth.shape, dtype=np.float64)
    nominal_x = (cols - (width - 1) / 2.0) * reference_range_m / fx
    nominal_y = (rows - (height - 1) / 2.0) * reference_range_m / fy
    footprint_pixels = np.hypot(nominal_x, nominal_y) <= footprint_radius_m
    footprint_valid = valid & footprint_pixels
    coverage = float(footprint_valid.sum()) / max(1, int(footprint_pixels.sum()))
    points, _ = _camera_points(np.where(footprint_valid, depth, np.nan),
                               horizontal_fov_deg, vertical_fov_deg)
    if len(points) < 128:
        return base | {
            "assessable": True, "suitable": False,
            "reason": "insufficient valid depth in touchdown footprint",
            "quality": 0.0, "footprint_coverage": round(coverage, 5),
            "footprint_radius_m": round(footprint_radius_m, 5),
        }

    # Plane z = ax + by + c in optical coordinates.  For a downward camera,
    # its optical axis is normal to level ground, so sqrt(a^2+b^2) is tan tilt.
    design = np.column_stack((points[:, 0], points[:, 1], np.ones(len(points))))
    coefficients, _, _, _ = np.linalg.lstsq(design, points[:, 2], rcond=None)
    residuals = points[:, 2] - design @ coefficients
    # A median-only statistic hides a discrete rock/step occupying less than
    # half the footprint.  The 95th percentile makes those hazards visible
    # while still resisting isolated invalid pixels already filtered above.
    roughness = float(np.percentile(np.abs(residuals - np.median(residuals)), 95))
    slope_deg = math.degrees(math.atan(math.hypot(coefficients[0], coefficients[1])))
    # Quality is explanatory only; suitability is determined by the individual
    # hard limits below, never by an arbitrary score cutoff.
    coverage_score = min(1.0, coverage / 0.95)
    slope_score = max(0.0, 1.0 - slope_deg / max_slope_deg)
    roughness_score = max(0.0, 1.0 - roughness / max_roughness_m)
    quality = round(coverage_score * slope_score * roughness_score, 5)
    suitable = coverage >= 0.90 and slope_deg <= max_slope_deg and roughness <= max_roughness_m
    reason = "flat touchdown footprint" if suitable else "surface exceeds landing limits"
    return base | {
        "assessable": True,
        "suitable": suitable,
        "reason": reason,
        "quality": quality,
        "footprint_coverage": round(coverage, 5),
        "footprint_radius_m": round(footprint_radius_m, 5),
        "slope_deg": round(slope_deg, 5),
        "roughness_m": round(roughness, 5),
        "sample_count": int(len(points)),
        "limits": {"max_slope_deg": max_slope_deg,
                   "max_roughness_m": max_roughness_m},
    }

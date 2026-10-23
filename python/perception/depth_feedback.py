"""Measured forward depth feedback; no object identity or simulator ground truth.

Hardware drivers can push metric optical-z frames into ``DepthFrameCache``.
Gazebo uses one persistent subscriber, so requesting feedback never waits for
an image or launches a capture process. Pixel regions can be measured against
an immutable snapshot retained with the RGB observation that selected them.
"""

import copy
import json
import math
import sys
import threading
import time
from dataclasses import dataclass
from pathlib import Path

import numpy as np


@dataclass(frozen=True)
class DepthSnapshot:
    depth_m: np.ndarray
    received_at_unix_ms: int
    sensor_timestamp_ns: int | None
    metadata: dict
    fx: float
    fy: float
    maximum_age_ms: int

    def _evidence(self, now_unix_ms=None):
        now = int(time.time() * 1000) if now_unix_ms is None else now_unix_ms
        age = now - self.received_at_unix_ms
        return {
            **copy.deepcopy(self.metadata),
            "received_at_unix_ms": self.received_at_unix_ms,
            "sensor_timestamp_ns": self.sensor_timestamp_ns,
            "age_ms": age,
            "fresh": 0 <= age <= self.maximum_age_ms,
            "width_px": self.depth_m.shape[1],
            "height_px": self.depth_m.shape[0],
        }

    def _region(self, rows, cols):
        region = self.depth_m[rows, cols]
        valid = np.isfinite(region)
        values = region[valid]
        result = {"valid_fraction": round(float(valid.mean()), 4),
                  "valid_pixels": int(valid.sum()),
                  "minimum_optical_z_m": None, "median_optical_z_m": None,
                  "p10_optical_z_m": None, "minimum_radial_range_m": None}
        if values.size:
            ys, xs = np.nonzero(valid)
            xs = xs + (cols.start or 0)
            ys = ys + (rows.start or 0)
            h, w = self.depth_m.shape
            rays = np.sqrt(1 + ((xs - (w - 1) / 2) / self.fx) ** 2
                           + ((ys - (h - 1) / 2) / self.fy) ** 2)
            result.update(
                minimum_optical_z_m=float(np.min(values)),
                median_optical_z_m=float(np.median(values)),
                p10_optical_z_m=float(np.percentile(values, 10)),
                minimum_radial_range_m=float(np.min(values * rays)),
            )
        return result

    def summary(self, now_unix_ms=None):
        result = self._evidence(now_unix_ms)
        h, w = self.depth_m.shape
        result.update(self._region(slice(0, h), slice(0, w)))
        # Horizontal thirds describe the whole image; central patch is a
        # separate 20% x 20% measurement, never an identified object's range.
        result["sectors"] = {
            name: self._region(slice(0, h), slice(i * w // 3, (i + 1) * w // 3))
            for i, name in enumerate(("left", "middle", "right"))
        }
        result["center"] = self._region(slice(2 * h // 5, 3 * h // 5),
                                        slice(2 * w // 5, 3 * w // 5))
        result["available"] = result["valid_pixels"] > 0
        if not result["available"]:
            result["reason"] = "No valid depth returns; this does not establish free space."
        result["object_associated"] = False
        return result

    def region(self, box_1000, now_unix_ms=None):
        """Measure [left, top, right, bottom], normalized to 0..1000.

        This is box geometry, not proof that every return belongs to its
        selected object. Caller must associate aligned RGB/depth timestamps.
        """
        if (len(box_1000) != 4 or any(not math.isfinite(float(x)) for x in box_1000)
                or not 0 <= box_1000[0] < box_1000[2] <= 1000
                or not 0 <= box_1000[1] < box_1000[3] <= 1000):
            raise ValueError("box_1000 must be ordered [left,top,right,bottom] within 0..1000")
        h, w = self.depth_m.shape
        left, top, right, bottom = box_1000
        result = self._evidence(now_unix_ms)
        result.update(self._region(
            slice(int(top * h / 1000), min(h, math.ceil(bottom * h / 1000))),
            slice(int(left * w / 1000), min(w, math.ceil(right * w / 1000))),
        ))
        result["box_1000"] = list(box_1000)
        result["available"] = result["valid_pixels"] > 0
        result["object_associated"] = False
        return result


class DepthFrameCache:
    """Thread-safe hardware-independent cache of metric optical-z frames.

    A physical adapter must convert its depth units to metres, supply measured
    intrinsics and mounting calibration, and use the actual sensor limits.
    """

    def __init__(self, *, source, simulation_only, horizontal_fov_deg,
                 minimum_depth_m, maximum_depth_m, calibration,
                 sensor_id="intel_realsense_d435i_depth", maximum_age_ms=750,
                 optical_axis_body_frd=(1.0, 0.0, 0.0), vertical_fov_deg=None,
                 sensor_clock="device"):
        numbers = (horizontal_fov_deg, minimum_depth_m, maximum_depth_m, maximum_age_ms)
        if (not all(math.isfinite(float(n)) for n in numbers)
                or not 0 < horizontal_fov_deg < 180
                or not 0 <= minimum_depth_m < maximum_depth_m or maximum_age_ms <= 0
                or (vertical_fov_deg is not None and not 0 < vertical_fov_deg < 180)):
            raise ValueError("invalid depth calibration or range")
        self.metadata = {
            "source": source, "sensor_id": sensor_id,
            "simulation_only": bool(simulation_only),
            "frame": "camera_optical (x right, y down, z forward)",
            "measurement": "optical_z_metres; radial_range is ray length from camera",
            "optical_axis_body_frd": list(optical_axis_body_frd),
            "calibration": calibration,
            "sensor_clock": sensor_clock,
            "valid_range_m": [minimum_depth_m, maximum_depth_m],
        }
        self.hfov = horizontal_fov_deg
        self.vfov = vertical_fov_deg
        self.maximum_age_ms = maximum_age_ms
        self._lock = threading.Lock()
        self._snapshot = None
        self._error = "Waiting for first depth frame."

    def update(self, depth_m, *, received_at_unix_ms=None, sensor_timestamp_ns=None):
        frame = np.array(depth_m, dtype=np.float32, copy=True)
        if frame.ndim != 2 or min(frame.shape) < 8:
            raise ValueError("depth_m must be a 2-D image at least 8 x 8 pixels")
        minimum, maximum = self.metadata["valid_range_m"]
        frame[~np.isfinite(frame) | (frame < minimum) | (frame > maximum) | (frame <= 0)] = np.nan
        frame.setflags(write=False)
        h, w = frame.shape
        fx = w / (2 * math.tan(math.radians(self.hfov) / 2))
        fy = h / (2 * math.tan(math.radians(self.vfov) / 2)) if self.vfov else fx
        snapshot = DepthSnapshot(
            frame, int(time.time() * 1000) if received_at_unix_ms is None else received_at_unix_ms,
            sensor_timestamp_ns, copy.deepcopy(self.metadata), fx, fy, self.maximum_age_ms,
        )
        with self._lock:
            self._snapshot = snapshot
            self._error = None

    def invalidate(self, reason):
        with self._lock:
            self._snapshot = None
            self._error = str(reason)

    def snapshot(self):
        with self._lock:
            return self._snapshot

    def summary(self, now_unix_ms=None):
        with self._lock:
            snapshot, error = self._snapshot, self._error
        if snapshot is None:
            return {**copy.deepcopy(self.metadata), "available": False,
                    "fresh": False, "reason": error}
        return snapshot.summary(now_unix_ms)


def decode_gazebo_depth(message):
    """Decode R_FLOAT32 image rows, including transport row padding."""
    w, h, step = int(message.width), int(message.height), int(message.step)
    if (int(message.pixel_format_type) != 13 or w < 8 or h < 8
            or step < w * 4 or len(message.data) != h * step):
        raise ValueError("depth image is not R_FLOAT32 optical-z metres")
    return np.ndarray((h, w), dtype=np.float32, buffer=message.data,
                      strides=(step, 4)).copy()


class GazeboDepthSource:
    """Persistent simulation adapter; bounded CPU sampling and no per-call I/O."""

    def __init__(self, session, *, calibration_path=None, update_hz=5.0):
        # Ubuntu distributes the native Gazebo bindings through apt rather
        # than pip. The agent's isolated venv keeps its own packages preferred.
        try:
            import gz.transport13  # noqa: F401
        except ModuleNotFoundError as error:
            distro_packages = Path("/usr/lib/python3/dist-packages")
            if not error.name.startswith("gz") or not (distro_packages / "gz").is_dir():
                raise
            if str(distro_packages) not in sys.path:
                sys.path.append(str(distro_packages))
        from gz.msgs10.image_pb2 import Image
        from gz.transport13 import Node, NodeOptions

        if not math.isfinite(update_hz) or update_hz <= 0:
            raise ValueError("update_hz must be positive")
        path = calibration_path or (Path(__file__).resolve().parents[2]
                                    / "config/perception/target_localization.json")
        config = json.loads(Path(path).read_text())
        camera = config["camera"]
        self.topic = "/icarus/sensors/d435i/depth_image"
        self.cache = DepthFrameCache(
            source="gazebo:" + self.topic, simulation_only=True,
            horizontal_fov_deg=camera["horizontal_fov_deg"],
            minimum_depth_m=camera.get("depth_minimum_m", 0.3),
            maximum_depth_m=camera.get("depth_maximum_m", 10.0),
            calibration=camera["calibration"],
            maximum_age_ms=config["maximum_perception_age_ms"],
            sensor_clock="gazebo_simulation",
        )
        self._period = 1.0 / update_hz
        self._last_update = float("-inf")
        self._closed = False
        self._callback_lock = threading.Lock()
        options = NodeOptions()
        options.partition = session["partition"]
        self._node = Node(options)
        if not self._node.subscribe(Image, self.topic, self._receive):
            raise RuntimeError("Could not subscribe to forward depth: " + self.topic)

    def _receive(self, message):
        if not self._callback_lock.acquire(blocking=False):
            return
        try:
            now = time.monotonic()
            if self._closed or now - self._last_update < self._period:
                return
            self._last_update = now
            received_at = int(time.time() * 1000)
            stamp = message.header.stamp
            self.cache.update(
                decode_gazebo_depth(message), received_at_unix_ms=received_at,
                sensor_timestamp_ns=int(stamp.sec) * 1_000_000_000 + int(stamp.nsec),
            )
        except Exception as error:  # transport callback cannot propagate exceptions
            self.cache.invalidate(str(error))
        finally:
            self._callback_lock.release()

    def summary(self):
        return self.cache.summary()

    def snapshot(self):
        return self.cache.snapshot()

    def close(self):
        self._closed = True
        self._node.unsubscribe(self.topic)
        with self._callback_lock:
            self.cache.invalidate("Depth source closed.")


class AirSimDepthSource:
    """AirSim forward planar depth, captured on demand over RPC (no subscriber).

    AirSim ImageType 1 (DepthPlanar) is optical-z metres, the same quantity
    the Gazebo source publishes, so regions and summaries are shared.
    """

    def __init__(self, settings_path, *, host="127.0.0.1", port=41451,
                 vehicle=None, camera="0", maximum_depth_m=100.0):
        settings = json.loads(Path(settings_path).read_text())
        vehicles = settings.get("Vehicles", {})
        self.vehicle = vehicle or next(iter(vehicles))
        mount = vehicles[self.vehicle]["Cameras"][camera]
        capture = next((c for c in mount.get("CaptureSettings", []) if c.get("ImageType") == 1), None)
        if capture is None:
            raise ValueError(f"AirSim camera {camera} has no depth capture settings")
        self.camera, self.host, self.port = camera, host, port
        width, height = int(capture["Width"]), int(capture["Height"])
        hfov = float(capture.get("FOV_Degrees", 90))
        vfov = math.degrees(2 * math.atan(math.tan(math.radians(hfov) / 2) * height / width))
        self.cache = DepthFrameCache(
            source="airsim_depth_planar_rpc", simulation_only=True,
            horizontal_fov_deg=hfov, vertical_fov_deg=vfov,
            minimum_depth_m=0.2, maximum_depth_m=maximum_depth_m,
            calibration="airsim_settings_capture", sensor_id="airsim_forward_depth",
            maximum_age_ms=1500, sensor_clock="simulator")
        self._rpc = None

    def _capture(self):
        from python.simulation.airsim_rpc import MsgpackRpcClient
        if self._rpc is None:
            self._rpc = MsgpackRpcClient(self.host, self.port, timeout=10)
        request = {"camera_name": self.camera, "image_type": 1,
                   "pixels_as_float": True, "compress": False}
        response = (self._rpc.images([request], self.vehicle) or [{}])[0]
        width, height = int(response.get("width", 0)), int(response.get("height", 0))
        values = np.asarray(response.get("image_data_float") or [], dtype=np.float32)
        if width <= 0 or height <= 0 or values.size != width * height:
            raise RuntimeError("AirSim returned an empty or malformed depth image")
        self.cache.update(values.reshape(height, width))

    def snapshot(self):
        try:
            self._capture()
        except (OSError, RuntimeError, ValueError) as error:
            self._rpc = None
            self.cache.invalidate(error)
        return self.cache.snapshot()

    def summary(self, now_unix_ms=None):
        self.snapshot()
        return self.cache.summary(now_unix_ms)

    def close(self):
        rpc, self._rpc = self._rpc, None
        if rpc is not None and hasattr(rpc, "close"):
            rpc.close()

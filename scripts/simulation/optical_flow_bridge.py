#!/usr/bin/python3
"""Derive simulation-only flow observations from downward RGB.

Gazebo Harmonic has no native optical-flow sensor.  This bridge uses phase
correlation on the hardware-specific BRIO simulation stream and publishes a
bounded JSON observation. autopilot_sensor_bridge forwards valid observations
to Gazebo SITL. This is not a driver or calibration for a physical ARK Flow.
"""

import argparse
import json
import math
import signal
import threading
import time
from pathlib import Path

import numpy as np

SOURCE_TOPIC = "/icarus/sensors/brio_down"
OUTPUT_TOPIC = "/icarus/sensors/ark_flow/optical_flow"
HORIZONTAL_FOV_RAD = math.radians(82.1492203076493)


def grayscale_downsample(payload, width, height, step, target_width=160):
    """Return a small float grayscale frame from one packed RGB8 image."""
    if width <= 0 or height <= 0 or step != width * 3:
        raise ValueError("expected a packed RGB8 image")
    data = np.frombuffer(payload, dtype=np.uint8)
    if data.size != height * step:
        raise ValueError("RGB8 payload size does not match image dimensions")
    rgb = data.reshape(height, width, 3)
    stride = max(1, math.ceil(width / target_width))
    sampled = rgb[::stride, ::stride]
    # ITU-R BT.601 luma is sufficient for texture motion; keep float32 to
    # bound FFT memory and avoid retaining the native camera frame.
    return (
        0.299 * sampled[..., 0]
        + 0.587 * sampled[..., 1]
        + 0.114 * sampled[..., 2]
    ).astype(np.float32)


def phase_correlation(previous, current):
    """Return current image displacement ``(dx, dy, quality)`` in pixels."""
    if previous.shape != current.shape or previous.ndim != 2:
        raise ValueError("phase-correlation frames must be equal 2-D arrays")
    height, width = previous.shape
    window = np.outer(np.hanning(height), np.hanning(width)).astype(np.float32)
    a = (previous - float(previous.mean())) * window
    b = (current - float(current.mean())) * window
    energy = float(np.linalg.norm(a) * np.linalg.norm(b))
    if energy < 1e-6:
        return 0.0, 0.0, 0
    cross = np.fft.fft2(b) * np.conj(np.fft.fft2(a))
    magnitude = np.abs(cross)
    cross /= np.maximum(magnitude, 1e-12)
    correlation = np.fft.ifft2(cross).real
    peak_y, peak_x = np.unravel_index(np.argmax(correlation), correlation.shape)
    if peak_x > width // 2:
        peak_x -= width
    if peak_y > height // 2:
        peak_y -= height
    peak = float(correlation.max())
    background = float(np.mean(np.abs(correlation))) + 1e-12
    quality = int(np.clip((peak / background - 1.0) * 4.0, 0, 255))
    # Parabolic interpolation avoids quantising slow flight to whole pixels.
    py, px = np.unravel_index(np.argmax(correlation), correlation.shape)
    def offset(a, b, c):
        denominator = a - 2 * b + c
        return float(np.clip(0.5 * (a - c) / denominator, -0.5, 0.5)) if abs(denominator) > 1e-12 else 0.0
    sub_x = offset(correlation[py, (px - 1) % width], correlation[py, px], correlation[py, (px + 1) % width])
    sub_y = offset(correlation[(py - 1) % height, px], correlation[py, px], correlation[(py + 1) % height, px])
    # Sensor noise on a uniform floor is NOT observable motion. A weak peak
    # or textureless frame must never acquire flight authority through flow.
    if min(float(previous.std()), float(current.std())) < 3.0 or peak < 0.15:
        quality = 0
    return float(peak_x) + sub_x, float(peak_y) + sub_y, quality


def flow_integrals(dx, dy, width, height):
    """Pinhole projection; BRIO is pitched down 90deg in Gazebo FLU.

    Image right = body right, image down = body backward (R_y(+90)).
    Forward translation produces positive image dy, rightward translation
    negative dx. ArduPilot expects (-v_right/h, v_forward/h), hence (dx,dy).
    A square-pixel camera has fy=fx, NOT vertical-FOV divided by height.
    """
    focal = width / (2.0 * math.tan(HORIZONTAL_FOV_RAD / 2.0))
    return {"x": dx / focal, "y": dy / focal}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--directory", required=True, type=Path)
    parser.add_argument("--target-width", type=int, default=320)
    args = parser.parse_args()
    if not 64 <= args.target_width <= 320:
        parser.error("--target-width must be in 64..320")

    from gz.msgs10.image_pb2 import Image
    from gz.msgs10.stringmsg_pb2 import StringMsg
    from gz.transport13 import Node

    stopping = threading.Event()
    lock = threading.Lock()
    health_path = args.directory / "optical_flow.json"
    state = {
        "schema": "icarus.sim.optical-flow-health.v1",
        "status": "starting",
        "source_topic": SOURCE_TOPIC,
        "output_topic": OUTPUT_TOPIC,
        "simulation_only_derived": True,
        "flight_authority": False,
        "autopilot_consumer": "autopilot_sensor_bridge",
        "frames_received": 0,
        "observations_published": 0,
        "invalid_frames": 0,
    }
    previous = {"frame": None, "stamp_s": None}

    def stop(_signum, _frame):
        stopping.set()

    signal.signal(signal.SIGINT, stop)
    signal.signal(signal.SIGTERM, stop)
    node = Node()
    publisher = node.advertise(OUTPUT_TOPIC, StringMsg)

    def receive(message):
        now = time.monotonic()
        stamp_s = message.header.stamp.sec + message.header.stamp.nsec * 1e-9
        try:
            frame = grayscale_downsample(
                message.data, int(message.width), int(message.height),
                int(message.step), args.target_width)
        except (ValueError, TypeError):
            with lock:
                state["invalid_frames"] += 1
                state["last_frame_monotonic_s"] = now
            return
        with lock:
            state["frames_received"] += 1
            state["last_frame_monotonic_s"] = now
            old_frame = previous["frame"]
            old_stamp = previous["stamp_s"]
            previous["frame"] = frame
            previous["stamp_s"] = stamp_s
        if old_frame is None or old_stamp is None:
            return
        dt = stamp_s - old_stamp
        if not 1e-4 <= dt <= 1.0:
            return
        dx, dy, quality = phase_correlation(old_frame, frame)
        result = {
            "schema": "icarus.sim.ark-flow-observation.v2",
            "source": "derived_phase_correlation_from_logitech_brio_rgb",
            "simulation_only_derived": True,
            "flight_authority": False,
            "frame": "body_frd",
            "observed_at_simulation_s": stamp_s,
            "integration_interval_s": dt,
            "image_displacement_px": {"x": dx, "y": dy},
            "flow_integral_rad": flow_integrals(dx, dy, frame.shape[1], frame.shape[0]),
            "quality": quality,
            "quality_scale": [0, 255],
            "processing_resolution": {
                "width": int(frame.shape[1]), "height": int(frame.shape[0])},
        }
        output = StringMsg(data=json.dumps(result, sort_keys=True))
        output.header.CopyFrom(message.header)
        if publisher.publish(output):
            with lock:
                state["observations_published"] += 1
                state["last_publish_monotonic_s"] = now
                state["last_quality"] = quality

    if not node.subscribe(Image, SOURCE_TOPIC, receive):
        raise RuntimeError("could not subscribe to " + SOURCE_TOPIC)

    def publish_health(final=False):
        with lock:
            now = time.monotonic()
            last = state.get("last_publish_monotonic_s")
            stale = last is not None and now - last > 2.0
            state["status"] = (
                "stopped" if final else "ready"
                if state["observations_published"] >= 3 and not stale
                else "stale" if stale else "starting")
            state["updated_monotonic_s"] = now
            snapshot = dict(state)
        temporary = health_path.with_suffix(".tmp")
        temporary.write_text(json.dumps(snapshot, indent=2) + "\n")
        temporary.replace(health_path)

    try:
        while not stopping.wait(0.25):
            publish_health()
    finally:
        stopping.set()
        publish_health(final=True)


if __name__ == "__main__":
    main()

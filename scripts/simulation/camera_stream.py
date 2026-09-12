#!/usr/bin/python3
# ruff: noqa: I001
"""Publish the normalized forward camera as a low-latency H.264 RTP stream."""

import argparse
import json
import signal
import threading
import time
from pathlib import Path

import gi

gi.require_version("Gst", "1.0")
from gi.repository import Gst


CAMERA_TOPIC = "/icarus/sensors/rgbd/image"


def encoder_pipeline(host, port, width, height, fps, bitrate_kbps):
    return (
        "appsrc name=source is-live=true format=time do-timestamp=true block=false "
        f"caps=video/x-raw,format=RGB,width={width},height={height},framerate={fps}/1 "
        "! queue leaky=downstream max-size-buffers=2 "
        "! videoconvert "
        f"! x264enc tune=zerolatency speed-preset=ultrafast bitrate={bitrate_kbps} "
        f"key-int-max={fps} bframes=0 byte-stream=true "
        "! rtph264pay config-interval=1 pt=96 "
        f"! udpsink host={host} port={port} sync=false async=false"
    )


def valid_rgb_frame(message, width, height):
    return (
        int(message.width) == width
        and int(message.height) == height
        and int(message.step) == width * 3
        and len(message.data) == width * height * 3
    )


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--directory", required=True, type=Path)
    parser.add_argument("--destination", default="127.0.0.1")
    parser.add_argument("--port", type=int, default=5600)
    parser.add_argument("--width", type=int, default=640)
    parser.add_argument("--height", type=int, default=480)
    parser.add_argument("--fps", type=int, default=15)
    parser.add_argument("--bitrate-kbps", type=int, default=1500)
    args = parser.parse_args()
    if not 1 <= args.port <= 65535:
        raise ValueError("port must be in 1..65535")
    if min(args.width, args.height, args.fps, args.bitrate_kbps) <= 0:
        raise ValueError("camera dimensions, FPS and bitrate must be positive")

    from gz.msgs10.image_pb2 import Image
    from gz.transport13 import Node

    Gst.init(None)
    pipeline = Gst.parse_launch(
        encoder_pipeline(
            args.destination,
            args.port,
            args.width,
            args.height,
            args.fps,
            args.bitrate_kbps,
        )
    )
    source = pipeline.get_by_name("source")
    bus = pipeline.get_bus()
    stopping = threading.Event()
    lock = threading.Lock()
    started = time.monotonic()
    state = {
        "status": "starting",
        "topic": CAMERA_TOPIC,
        "transport": "rtp-h264",
        "destination": args.destination,
        "port": args.port,
        "width": args.width,
        "height": args.height,
        "nominal_fps": args.fps,
        "bitrate_kbps": args.bitrate_kbps,
        "frames_received": 0,
        "frames_pushed": 0,
        "invalid_frames": 0,
        "push_failures": 0,
    }

    def stop(_signum, _frame):
        stopping.set()

    signal.signal(signal.SIGINT, stop)
    signal.signal(signal.SIGTERM, stop)

    def receive(message):
        now = time.monotonic()
        with lock:
            state["frames_received"] += 1
            state["last_frame_monotonic_s"] = now
        if not valid_rgb_frame(message, args.width, args.height):
            with lock:
                state["invalid_frames"] += 1
            return
        buffer = Gst.Buffer.new_allocate(None, len(message.data), None)
        buffer.fill(0, bytes(message.data))
        result = source.emit("push-buffer", buffer)
        with lock:
            if result == Gst.FlowReturn.OK:
                state["frames_pushed"] += 1
                state["last_push_monotonic_s"] = now
            else:
                state["push_failures"] += 1

    node = Node()
    if not node.subscribe(Image, CAMERA_TOPIC, receive):
        raise RuntimeError("Could not subscribe to " + CAMERA_TOPIC)

    health_path = args.directory / "camera_stream.json"

    def publish(final=False):
        with lock:
            now = time.monotonic()
            age = now - state.get("last_frame_monotonic_s", now)
            elapsed = max(now - started, 1e-6)
            state["measured_input_fps"] = state["frames_received"] / elapsed
            state["measured_output_fps"] = state["frames_pushed"] / elapsed
            state["last_frame_age_s"] = age
            state["updated_monotonic_s"] = now
            if final:
                state["status"] = "stopped"
            elif state["frames_pushed"] >= 3 and age <= 2.0:
                state["status"] = "ready"
            elif state["frames_pushed"] and age > 2.0:
                state["status"] = "stale"
            snapshot = dict(state)
        temporary = health_path.with_suffix(".tmp")
        temporary.write_text(json.dumps(snapshot, indent=2) + "\n")
        temporary.replace(health_path)

    if pipeline.set_state(Gst.State.PLAYING) == Gst.StateChangeReturn.FAILURE:
        raise RuntimeError("Could not start the H.264 camera pipeline")
    try:
        while not stopping.wait(0.25):
            message = bus.pop_filtered(Gst.MessageType.ERROR)
            if message:
                error, debug = message.parse_error()
                raise RuntimeError(f"GStreamer camera error: {error}; {debug}")
            publish()
    finally:
        stopping.set()
        source.emit("end-of-stream")
        pipeline.set_state(Gst.State.NULL)
        publish(final=True)


if __name__ == "__main__":
    main()

#!/usr/bin/python3
# ruff: noqa: I001
"""Publish every normalized RGB camera as a low-latency H.264 RTP stream."""

import argparse
import json
import signal
import threading
import time
from pathlib import Path

import gi

gi.require_version("Gst", "1.0")
from gi.repository import Gst


CAMERA_STREAMS = (
    {"id": "forward_d435i", "label": "Forward Intel RealSense D435i",
     "topic": "/icarus/sensors/d435i/image", "source_width": 848,
     "source_height": 480, "source_fps": 30},
    {"id": "downward_brio", "label": "Downward Logitech BRIO 4K",
     "topic": "/icarus/sensors/brio_down", "source_width": 1280,
     "source_height": 720, "source_fps": 30},
)


def encoder_pipeline(host, port, source_width, source_height, source_fps,
                     output_width, output_height, output_fps, bitrate_kbps):
    return (
        "appsrc name=source is-live=true format=time do-timestamp=true block=false "
        f"caps=video/x-raw,format=RGB,width={source_width},height={source_height},"
        f"framerate={source_fps}/1 "
        "! queue leaky=downstream max-size-buffers=2 "
        "! videoconvert ! videoscale add-borders=true ! videorate drop-only=true "
        f"! video/x-raw,format=I420,width={output_width},height={output_height},"
        f"pixel-aspect-ratio=1/1,framerate={output_fps}/1 "
        f"! x264enc tune=zerolatency speed-preset=ultrafast bitrate={bitrate_kbps} "
        f"key-int-max={output_fps} bframes=0 byte-stream=true "
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
    # Both installed RGB cameras publish at 30 Hz in the physical-fidelity
    # profile.  Keep the network stream at that rate by default instead of
    # silently halving it to the legacy 15 FPS preview.
    parser.add_argument("--fps", type=int, default=30)
    parser.add_argument("--bitrate-kbps", type=int, default=1500)
    args = parser.parse_args()
    if not 1 <= args.port <= 65535:
        raise ValueError("port must be in 1..65535")
    if min(args.width, args.height, args.fps, args.bitrate_kbps) <= 0:
        raise ValueError("camera dimensions, FPS and bitrate must be positive")

    from gz.msgs10.image_pb2 import Image
    from gz.transport13 import Node

    Gst.init(None)
    stopping = threading.Event()
    lock = threading.Lock()
    started = time.monotonic()
    streams = []
    for index, specification in enumerate(CAMERA_STREAMS):
        port = args.port + index
        if port > 65535:
            raise ValueError("camera stream port range exceeds 65535")
        pipeline = Gst.parse_launch(encoder_pipeline(
            args.destination, port, specification["source_width"],
            specification["source_height"], specification["source_fps"],
            args.width, args.height, args.fps, args.bitrate_kbps))
        streams.append({**specification, "port": port, "pipeline": pipeline,
                        "source": pipeline.get_by_name("source"), "bus": pipeline.get_bus(),
                        "frames_received": 0, "frames_pushed": 0, "invalid_frames": 0,
                        "push_failures": 0})

    def stop(_signum, _frame):
        stopping.set()

    signal.signal(signal.SIGINT, stop)
    signal.signal(signal.SIGTERM, stop)

    def receive(message, stream):
        now = time.monotonic()
        with lock:
            stream["frames_received"] += 1
            stream["last_frame_monotonic_s"] = now
        if not valid_rgb_frame(
                message, stream["source_width"], stream["source_height"]):
            with lock:
                stream["invalid_frames"] += 1
            return
        buffer = Gst.Buffer.new_allocate(None, len(message.data), None)
        buffer.fill(0, bytes(message.data))
        result = stream["source"].emit("push-buffer", buffer)
        with lock:
            if result == Gst.FlowReturn.OK:
                stream["frames_pushed"] += 1
                stream["last_push_monotonic_s"] = now
            else:
                stream["push_failures"] += 1

    node = Node()
    for stream in streams:
        if not node.subscribe(Image, stream["topic"],
                              lambda message, stream=stream: receive(message, stream)):
            raise RuntimeError("Could not subscribe to " + stream["topic"])

    health_path = args.directory / "camera_stream.json"

    def publish(final=False):
        with lock:
            now = time.monotonic()
            elapsed = max(now - started, 1e-6)
            snapshots = []
            for stream in streams:
                age = now - stream.get("last_frame_monotonic_s", now)
                snapshots.append({key: stream[key] for key in (
                    "id", "label", "topic", "port", "source_width",
                    "source_height", "source_fps", "frames_received", "frames_pushed",
                    "invalid_frames", "push_failures")})
                snapshots[-1].update({"measured_input_fps": stream["frames_received"] / elapsed,
                                      "measured_appsrc_push_fps": stream["frames_pushed"] / elapsed,
                                      "last_frame_age_s": age})
            ready = all(item["frames_pushed"] >= 3 and item["last_frame_age_s"] <= 2.0
                        for item in snapshots)
            stale = any(item["frames_pushed"] and item["last_frame_age_s"] > 2.0
                        for item in snapshots)
            snapshot = {"status": "stopped" if final else "ready" if ready else "stale" if stale else "starting",
                        "transport": "rtp-h264", "destination": args.destination,
                        "width": args.width, "height": args.height, "nominal_fps": args.fps,
                        "bitrate_kbps": args.bitrate_kbps, "streams": snapshots,
                        "updated_monotonic_s": now}
        temporary = health_path.with_suffix(".tmp")
        temporary.write_text(json.dumps(snapshot, indent=2) + "\n")
        temporary.replace(health_path)

    for stream in streams:
        if stream["pipeline"].set_state(Gst.State.PLAYING) == Gst.StateChangeReturn.FAILURE:
            raise RuntimeError("Could not start the H.264 camera pipeline for " + stream["id"])
    try:
        while not stopping.wait(0.25):
            for stream in streams:
                message = stream["bus"].pop_filtered(Gst.MessageType.ERROR)
                if message:
                    error, debug = message.parse_error()
                    raise RuntimeError(f"GStreamer camera error ({stream['id']}): {error}; {debug}")
            publish()
    finally:
        stopping.set()
        for stream in streams:
            stream["source"].emit("end-of-stream")
            stream["pipeline"].set_state(Gst.State.NULL)
        publish(final=True)


if __name__ == "__main__":
    main()

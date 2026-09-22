#!/usr/bin/python3
# ruff: noqa: I001
"""Ground-station grid viewer for the Icarus H.264 RTP camera contract."""

import argparse
import json
import os
import signal
import threading
import time
from pathlib import Path

import gi

gi.require_version("Gst", "1.0")
from gi.repository import Gst


ROOT = Path(__file__).resolve().parents[2]
ACTIVE_SESSION = ROOT / "logs/simulation/active_session.json"


def receiver_pipeline(streams, latency_ms):
    """Compose trusted session streams in a two-column labeled grid."""
    compositor = ["compositor name=grid background=black"]
    branches = []
    for index, stream in enumerate(streams):
        xpos, ypos = (index % 2) * stream["width"], (index // 2) * stream["height"]
        compositor.append(f"sink_{index}::xpos={xpos} sink_{index}::ypos={ypos}")
        branches.append(
            f"udpsrc port={stream['port']} "
            'caps="application/x-rtp,media=video,encoding-name=H264,payload=96" '
            f"! rtpjitterbuffer latency={latency_ms} drop-on-latency=true "
            "! rtph264depay ! avdec_h264 ! videoconvert "
            f"! textoverlay text=\"{stream['label']}\" valignment=top halignment=left shaded-background=true "
            f"! queue ! grid.sink_{index}")
    return " ".join(compositor) + " ! videoconvert ! autovideosink name=display sync=false " + " ".join(branches)


def session_video_streams():
    if not ACTIVE_SESSION.is_file():
        return []
    session = json.loads(ACTIVE_SESSION.read_text())
    try:
        os.kill(int(session["launcher_pid"]), 0)
    except (KeyError, ProcessLookupError, ValueError):
        return []
    video = session.get("video", {})
    if video.get("transport") != "rtp-h264":
        return []
    streams = video.get("streams") or [{"id": "forward_rgbd", "label": "Forward RGB-D", "port": video["port"]}]
    return [{"id": str(stream["id"]), "label": str(stream["label"]), "port": int(stream["port"]),
             "width": int(video.get("width", 640)), "height": int(video.get("height", 480))}
            for stream in streams]


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--port",
        type=int,
        help="Base local RTP listen port; defaults to the active vehicle session or 5600",
    )
    parser.add_argument("--latency-ms", type=int, default=75)
    args = parser.parse_args()
    streams = session_video_streams() or [
        {"id": "forward_rgbd", "label": "Forward RGB-D", "port": 5600, "width": 640, "height": 480},
        {"id": "downward_rgbd", "label": "Downward RGB-D", "port": 5601, "width": 640, "height": 480},
    ]
    if args.port is not None:
        for index, stream in enumerate(streams):
            stream["port"] = args.port + index
    if any(not 1 <= stream["port"] <= 65535 for stream in streams):
        raise ValueError("camera grid ports must be in 1..65535")
    if not 0 <= args.latency_ms <= 2000:
        raise ValueError("latency must be in 0..2000 ms")

    Gst.init(None)
    pipeline = Gst.parse_launch(receiver_pipeline(streams, args.latency_ms))
    decoder = pipeline.get_by_name("display").get_static_pad("sink")
    lock = threading.Lock()
    frame_count = 0
    last_frame = None

    def frame_probe(_pad, _info):
        nonlocal frame_count, last_frame
        with lock:
            frame_count += 1
            last_frame = time.monotonic()
        return Gst.PadProbeReturn.OK

    decoder.add_probe(Gst.PadProbeType.BUFFER, frame_probe)
    stopping = threading.Event()

    def stop(_signum, _frame):
        stopping.set()

    signal.signal(signal.SIGINT, stop)
    signal.signal(signal.SIGTERM, stop)
    bus = pipeline.get_bus()
    if pipeline.set_state(Gst.State.PLAYING) == Gst.StateChangeReturn.FAILURE:
        raise RuntimeError("Could not start the camera viewer")
    print("Listening for Icarus camera grid: " + ", ".join(
        f"{stream['label']} UDP {stream['port']}" for stream in streams))
    print("The viewer can start before the drone stream. Press Ctrl+C to close it.")
    previous_count = 0
    previous_time = time.monotonic()
    try:
        while not stopping.wait(1.0):
            message = bus.pop_filtered(Gst.MessageType.ERROR)
            if message:
                error, debug = message.parse_error()
                if "Output window was closed" in str(error):
                    break
                raise RuntimeError(f"GStreamer viewer error: {error}; {debug}")
            now = time.monotonic()
            with lock:
                count, last = frame_count, last_frame
            fps = (count - previous_count) / max(now - previous_time, 1e-6)
            status = "WAITING" if last is None else "LIVE" if now - last < 2 else "STALE"
            age = "n/a" if last is None else f"{now - last:.2f}s"
            print(
                f"\rVideo {status} | {fps:4.1f} fps | last frame {age} ago",
                end="",
                flush=True,
            )
            previous_count, previous_time = count, now
    finally:
        print()
        pipeline.set_state(Gst.State.NULL)


if __name__ == "__main__":
    main()

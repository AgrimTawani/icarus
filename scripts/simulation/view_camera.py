#!/usr/bin/python3
# ruff: noqa: I001
"""Ground-station viewer for the Icarus H.264 RTP camera contract."""

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


def receiver_pipeline(port, latency_ms):
    return (
        f"udpsrc port={port} "
        'caps="application/x-rtp,media=video,encoding-name=H264,payload=96" '
        f"! rtpjitterbuffer latency={latency_ms} drop-on-latency=true "
        "! rtph264depay ! avdec_h264 ! videoconvert "
        "! autovideosink name=display sync=false"
    )


def session_video_port():
    if not ACTIVE_SESSION.is_file():
        return None
    session = json.loads(ACTIVE_SESSION.read_text())
    try:
        os.kill(int(session["launcher_pid"]), 0)
    except (KeyError, ProcessLookupError, ValueError):
        return None
    video = session.get("video", {})
    if video.get("transport") != "rtp-h264":
        return None
    return int(video["port"])


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--port",
        type=int,
        help="Local RTP listen port; defaults to the active vehicle session or 5600",
    )
    parser.add_argument("--latency-ms", type=int, default=75)
    args = parser.parse_args()
    port = args.port or session_video_port() or 5600
    if not 1 <= port <= 65535:
        raise ValueError("port must be in 1..65535")
    if not 0 <= args.latency_ms <= 2000:
        raise ValueError("latency must be in 0..2000 ms")

    Gst.init(None)
    pipeline = Gst.parse_launch(receiver_pipeline(port, args.latency_ms))
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
    print(f"Listening for Icarus H.264/RTP video on UDP {port}")
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

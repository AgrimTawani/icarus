"""Bounded edge observer service skeleton used by the simulation profile.

The service is intentionally an observer: it can publish structured vision
evidence to a local consumer, but it has no Drone API or MAVLink dependency.
"""
import argparse
import hashlib
import json
import os
import subprocess
import tempfile
import time
from pathlib import Path

from .edge_vision import UniqueTrackCounter, normalize_yolo_result, select_device, validate_edge_manifest


# The detector gets coverage from both mounted RGB cameras.  This is a global
# round-robin schedule: combined inference remains capped at --max-fps, rather
# than silently running each camera at the full budget.
CAMERA_FEEDS = (
    {"id": "forward_rgbd", "topic": "/icarus/sensors/rgbd/image", "count_eligible": False},
    {"id": "downward_rgbd", "topic": "/icarus/sensors/rgbd_down/image", "count_eligible": True},
)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--profile", default="edge-vision", choices=("edge-vision",))
    parser.add_argument("--manifest", type=Path, default=Path.home() / "models/edge/MANIFEST.json")
    parser.add_argument("--max-fps", type=float, default=5)
    args = parser.parse_args()
    if not 0 < args.max_fps <= 5:
        parser.error("--max-fps must be between 0 and 5")
    manifest = json.loads(args.manifest.read_text())
    validate_edge_manifest(manifest)
    yolo_artifact = next(item for item in manifest["artifacts"] if item["role"] == "yolo")
    weights = Path(yolo_artifact["path"])
    if not weights.is_file() or hashlib.sha256(weights.read_bytes()).hexdigest() != yolo_artifact["sha256"]:
        raise RuntimeError("pinned yolo11n artifact is missing or has a checksum mismatch")
    import torch
    from ultralytics import YOLO
    device = select_device("auto", torch.cuda.is_available())
    model = YOLO(str(weights))
    trackers = {feed["id"]: UniqueTrackCounter() for feed in CAMERA_FEEDS}
    print(json.dumps({"profile": args.profile, "status": "observer-ready",
                      "max_fps": args.max_fps, "camera_feeds": [feed["id"] for feed in CAMERA_FEEDS],
                      "per_camera_target_fps": args.max_fps / len(CAMERA_FEEDS),
                      "manifest": str(args.manifest),
                      "device": device, "flight_authority": False,
                      "started_unix_ms": int(time.time() * 1000)}), flush=True)
    session_path = Path(__file__).resolve().parents[2] / "logs/simulation/active_session.json"
    root = Path(__file__).resolve().parents[2]
    next_frame = time.monotonic()
    feed_index = 0
    while True:
        try:
            session = json.loads(session_path.read_text())
            os.kill(int(session["launcher_pid"]), 0)
        except (OSError, ValueError, KeyError, json.JSONDecodeError):
            return
        now = time.monotonic()
        if now < next_frame:
            time.sleep(min(.05, next_frame - now))
            continue
        next_frame = now + 1 / args.max_fps
        feed = CAMERA_FEEDS[feed_index]
        feed_index = (feed_index + 1) % len(CAMERA_FEEDS)
        with tempfile.TemporaryDirectory(prefix="icarus-edge-frame-") as temporary:
            frame = Path(temporary) / "frame.ppm"
            inference_frame = Path(temporary) / "frame.png"
            try:
                subprocess.run(["/usr/bin/python3", str(root / "scripts/capture-camera-frame"),
                                "--output", str(frame), "--timeout", "2",
                                "--topic", feed["topic"]],
                               cwd=root, check=True, timeout=5, capture_output=True, text=True)
                from PIL import Image
                Image.open(frame).convert("RGB").save(inference_frame)
                started = time.monotonic()
                output = model.predict(source=str(inference_frame), imgsz=640, conf=.35,
                                       device=device, verbose=False)[0]
                names, boxes = output.names, output.boxes
                result = normalize_yolo_result(
                    {"labels": [names[int(value)] for value in boxes.cls.tolist()],
                     "scores": boxes.conf.tolist(), "boxes": boxes.xyxy.tolist(),
                     "image_dimensions": [int(output.orig_shape[1]), int(output.orig_shape[0])]},
                    frame, trackers[feed["id"]], image_size=640, max_fps=args.max_fps,
                    started_monotonic=started)
                result["runtime"] = {"device": device, "fallback": device == "cpu"}
            except RuntimeError as error:
                if device == "cuda" and "out of memory" in str(error).lower():
                    device = "cpu"
                    result = {"schema": "icarus.edge.yolo.v1", "observed_at_unix_ms": int(time.time() * 1000),
                              "event": "cuda_oom_cpu_fallback", "error": str(error)[-300:],
                              "max_fps": args.max_fps, "flight_authority": False}
                else:
                    result = {"schema": "icarus.edge.yolo.v1", "observed_at_unix_ms": int(time.time() * 1000),
                              "event": "observer_error", "error": str(error)[-300:],
                              "max_fps": args.max_fps, "flight_authority": False}
            except (OSError, subprocess.SubprocessError) as error:
                result = {"schema": "icarus.edge.yolo.v1", "observed_at_unix_ms": int(time.time() * 1000),
                          "event": "camera_capture_error", "error": str(error)[-300:],
                          "max_fps": args.max_fps, "flight_authority": False}
            result["camera"] = {"id": feed["id"], "topic": feed["topic"],
                                "count_eligible": feed["count_eligible"]}
        destination = Path(session.get("run_directory", root / "logs/simulation")) / "edge_vision.jsonl"
        with destination.open("a", encoding="utf-8") as stream:
            stream.write(json.dumps(result, sort_keys=True) + "\n")


if __name__ == "__main__":
    main()

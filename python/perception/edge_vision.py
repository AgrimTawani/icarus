"""Simulation-only edge vision primitives.

Detector and VLM outputs are observation evidence only.  This module imports
no Drone API, MAVLink, shell, or network client and exposes no flight action.
"""
from __future__ import annotations

import hashlib
import json
import tempfile
import time
from dataclasses import dataclass
from pathlib import Path


def _iou(a, b):
    ax1, ay1, ax2, ay2 = a
    bx1, by1, bx2, by2 = b
    ix1, iy1, ix2, iy2 = max(ax1, bx1), max(ay1, by1), min(ax2, bx2), min(ay2, by2)
    intersection = max(0.0, ix2 - ix1) * max(0.0, iy2 - iy1)
    aa = max(0.0, ax2 - ax1) * max(0.0, ay2 - ay1)
    ab = max(0.0, bx2 - bx1) * max(0.0, by2 - by1)
    return intersection / max(aa + ab - intersection, 1e-9)


@dataclass
class _Track:
    track_id: int
    label: str
    box: list[float]
    last_seen: int
    hits: int = 1


class UniqueTrackCounter:
    """Deterministic IoU tracker for compact unique-person counting."""
    def __init__(self, iou_threshold=0.35, max_missed_frames=15):
        if not 0 < iou_threshold <= 1:
            raise ValueError("iou_threshold must be in (0, 1]")
        self.iou_threshold = iou_threshold
        self.max_missed_frames = max_missed_frames
        self.frame_index = 0
        self.next_id = 1
        self.tracks = []
        self.seen_person_track_ids = set()

    def update(self, detections):
        self.frame_index += 1
        used, output = set(), []
        for detection in detections:
            label = str(detection["class"]).strip().lower()
            box = [float(v) for v in detection["bbox_xyxy_px"]]
            matches = [(_iou(box, track.box), track) for track in self.tracks
                       if track.label == label and track.track_id not in used]
            score, track = max(matches, key=lambda item: item[0], default=(0.0, None))
            if track is None or score < self.iou_threshold:
                track = _Track(self.next_id, label, box, self.frame_index)
                self.next_id += 1
                self.tracks.append(track)
            else:
                track.box, track.last_seen, track.hits = box, self.frame_index, track.hits + 1
            used.add(track.track_id)
            if label == "person":
                self.seen_person_track_ids.add(track.track_id)
            output.append({"track_id": track.track_id, "class": label,
                           "confidence": round(float(detection.get("confidence", 0)), 5),
                           "bbox_xyxy_px": [round(v, 2) for v in box], "hits": track.hits})
        self.tracks = [t for t in self.tracks
                       if self.frame_index - t.last_seen <= self.max_missed_frames]
        return output


def normalize_yolo_result(result, image_path, tracker, model_id="yolo11n",
                          image_size=640, max_fps=5.0, started_monotonic=None):
    """Normalize one Ultralytics-like result without retaining image pixels."""
    if image_size != 640 or not 0 < max_fps <= 5:
        raise ValueError("edge detector requires image_size=640 and max_fps in (0, 5]")
    labels, scores, boxes = (result.get(name, []) for name in ("labels", "scores", "boxes"))
    if not len(labels) == len(scores) == len(boxes):
        raise ValueError("YOLO result arrays must have equal length")
    detections = [{"class": str(label).strip().lower(), "confidence": round(float(score), 5),
                   "bbox_xyxy_px": [round(float(value), 2) for value in box]}
                  for label, score, box in zip(labels, scores, boxes)]
    tracks = tracker.update(detections)
    digest = hashlib.sha256(Path(image_path).read_bytes()).hexdigest()
    elapsed = None if started_monotonic is None else (time.monotonic() - started_monotonic) * 1000
    return {"schema": "icarus.edge.yolo.v1", "observed_at_unix_ms": int(time.time() * 1000),
            "source_frame_sha256": digest, "image_dimensions": result.get("image_dimensions"),
            "model": model_id, "image_size": image_size, "max_fps": max_fps,
            "classes": sorted({item["class"] for item in detections}), "detections": detections,
            "tracks": tracks, "unique_person_count": len(tracker.seen_person_track_ids),
            "confidence": round(sum(item["confidence"] for item in detections) / len(detections), 5) if detections else 0.0,
            "inference_latency_ms": round(elapsed, 3) if elapsed is not None else None,
            "flight_authority": False}


class RateLimiter:
    def __init__(self, interval_s=5.0, clock=time.monotonic):
        if interval_s < 0:
            raise ValueError("interval_s must not be negative")
        self.interval_s, self.clock, self.last = interval_s, clock, None

    def admit(self):
        now = self.clock()
        if self.last is not None and now - self.last < self.interval_s:
            raise RuntimeError("vision request rate limit exceeded")
        self.last = now


def select_device(requested="auto", cuda_available=False):
    """Choose an explicit edge detector backend; auto never hides fallback."""
    if requested not in ("auto", "cuda", "cpu"):
        raise ValueError("device must be auto, cuda, or cpu")
    if requested == "cuda" and not cuda_available:
        raise RuntimeError("CUDA requested for YOLO but unavailable")
    return "cuda" if requested == "cuda" or (requested == "auto" and cuda_available) else "cpu"


def detect_edge_image(image_path, manifest_path, tracker, *, device="auto",
                      max_fps=5.0, confidence=0.35):
    """Run pinned YOLO11n. Importing Ultralytics is deferred until execution."""
    manifest = json.loads(Path(manifest_path).read_text())
    validate_edge_manifest(manifest)
    yolo = next(item for item in manifest["artifacts"] if item["role"] == "yolo")
    weights = Path(yolo["path"])
    if not weights.is_file():
        raise FileNotFoundError("pinned yolo11n weights unavailable: " + str(weights))
    if hashlib.sha256(weights.read_bytes()).hexdigest() != yolo["sha256"]:
        raise ValueError("pinned yolo11n checksum mismatch")
    import torch
    from ultralytics import YOLO
    selected = select_device(device, torch.cuda.is_available())
    started = time.monotonic()
    # Gazebo's bounded capture adapter emits PPM, which Ultralytics does not
    # list as an accepted source extension. Convert only inside a temporary
    # directory: the structured result retains the original frame hash and no
    # image pixel data enters an episode.
    from PIL import Image
    with tempfile.TemporaryDirectory(prefix="icarus-yolo-input-") as temporary:
        inference_path = Path(temporary) / "frame.png"
        Image.open(image_path).convert("RGB").save(inference_path)
        output = YOLO(str(weights)).predict(source=str(inference_path), imgsz=640,
                                            conf=confidence, device=selected,
                                            verbose=False)[0]
    names = output.names
    boxes = output.boxes
    result = {"labels": [names[int(value)] for value in boxes.cls.tolist()],
              "scores": boxes.conf.tolist(), "boxes": boxes.xyxy.tolist(),
              "image_dimensions": [int(output.orig_shape[1]), int(output.orig_shape[0])]}
    normalized = normalize_yolo_result(result, image_path, tracker, image_size=640,
                                       max_fps=max_fps, started_monotonic=started)
    normalized["runtime"] = {"device": selected, "fallback": selected == "cpu",
                             "repository": yolo["repository"], "sha256": yolo["sha256"]}
    return normalized


def inspect_smolvlm(image_path, question, manifest_path, limiter, *, max_new_tokens=96):
    """One qualitative image question, loading then releasing accelerator memory."""
    if not isinstance(question, str) or not (1 <= len(" ".join(question.split())) <= 240):
        raise ValueError("visual question must be 1..240 compact characters")
    if not 1 <= max_new_tokens <= 96:
        raise ValueError("max_new_tokens must be in 1..96")
    limiter.admit()
    manifest = json.loads(Path(manifest_path).read_text())
    verify_edge_manifest_files(manifest)
    artifact = next(item for item in manifest["artifacts"] if item["role"] == "vlm")
    model_path = Path(artifact["path"])
    if not model_path.is_dir():
        raise FileNotFoundError("pinned SmolVLM snapshot unavailable: " + str(model_path))
    import torch
    from PIL import Image
    from transformers import AutoModelForImageTextToText, AutoProcessor
    selected = "cuda" if torch.cuda.is_available() else "cpu"
    started = time.monotonic()
    processor = AutoProcessor.from_pretrained(model_path, local_files_only=True)
    fallback = False
    dtype = torch.float16 if selected == "cuda" else torch.float32
    try:
        model = AutoModelForImageTextToText.from_pretrained(
            model_path, local_files_only=True, torch_dtype=dtype).to(selected).eval()
    except RuntimeError as error:
        if selected != "cuda" or "out of memory" not in str(error).lower():
            raise
        torch.cuda.empty_cache()
        selected, fallback, dtype = "cpu", True, torch.float32
        model = AutoModelForImageTextToText.from_pretrained(
            model_path, local_files_only=True, torch_dtype=dtype).to(selected).eval()
    try:
        image = Image.open(image_path).convert("RGB")
        messages = [{"role": "user", "content": [
            {"type": "image"}, {"type": "text", "text": (
                "Answer qualitatively in one concise sentence. Do not suggest flight actions, "
                "coordinates, landing areas, or safety decisions. Question: " + question)}]}]
        prompt = processor.apply_chat_template(messages, add_generation_prompt=True)
        inputs = processor(text=prompt, images=[image], return_tensors="pt").to(selected)
        with torch.inference_mode():
            generated = model.generate(**inputs, max_new_tokens=max_new_tokens, do_sample=False)
        answer = processor.batch_decode(generated[:, inputs["input_ids"].shape[1]:], skip_special_tokens=True)[0].strip()
    finally:
        del model
        if selected == "cuda":
            torch.cuda.empty_cache()
    if not answer:
        raise RuntimeError("SmolVLM returned no qualitative answer")
    return {"schema": "icarus.edge.smolvlm.v1", "observed_at_unix_ms": int(time.time() * 1000),
            "question": question, "answer": answer[:1200], "max_new_tokens": max_new_tokens,
            "latency_ms": round((time.monotonic() - started) * 1000, 3),
            "image": {"sha256": hashlib.sha256(Path(image_path).read_bytes()).hexdigest()},
            "runtime": {"device": selected, "fallback": fallback, "load_mode": "load-infer-release",
                        "repository": artifact["repository"], "revision": artifact["revision"]},
            "flight_authority": False}


def validate_edge_manifest(manifest):
    artifacts = {item.get("role"): item for item in manifest.get("artifacts", [])}
    required = ("qwen_edge_primary", "yolo", "vlm")
    missing = [role for role in required if role not in artifacts]
    if missing:
        raise ValueError("edge manifest missing roles: " + ", ".join(missing))
    if manifest.get("profile") != "edge-vision" or manifest.get("simulation_only") is not True:
        raise ValueError("edge manifest must be explicitly simulation-only")
    for role in required:
        item = artifacts[role]
        if any(not item.get(field) for field in ("path", "sha256", "repository", "revision", "runtime")):
            raise ValueError(f"edge manifest {role} is incomplete")
        if len(item["sha256"]) != 64:
            raise ValueError(f"edge manifest {role} has invalid sha256")
    if not manifest.get("llama_cpp_revision"):
        raise ValueError("edge manifest missing llama.cpp revision")
    return True


def verify_edge_manifest_files(manifest):
    """Verify pinned artifact bytes; intended for setup/preflight, not every frame."""
    validate_edge_manifest(manifest)
    artifacts = {item["role"]: item for item in manifest["artifacts"]}
    for role in ("qwen_edge_primary", "yolo"):
        path = Path(artifacts[role]["path"])
        if not path.is_file() or hashlib.sha256(path.read_bytes()).hexdigest() != artifacts[role]["sha256"]:
            raise ValueError(f"edge manifest {role} checksum mismatch")
    vlm = artifacts["vlm"]
    files = vlm.get("file_sha256", {})
    if not files:
        raise ValueError("edge manifest VLM files are not checksum-pinned")
    root = Path(vlm["path"]).resolve()
    for relative, expected in files.items():
        path = (root / relative).resolve()
        if root not in path.parents or not path.is_file() or hashlib.sha256(path.read_bytes()).hexdigest() != expected:
            raise ValueError("edge manifest VLM checksum mismatch: " + relative)
    return True

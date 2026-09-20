"""Open-vocabulary detector boundary for Icarus semantic perception.

This module deliberately accepts an image plus an explicit class list and
returns structured detections. It does not call an LLM, MAVLink or a flight
action. A future DCM ``detect`` tool can request classes through this narrow
boundary while flight-critical avoidance continues to use LiDAR.
"""

import hashlib
import json
import time
from collections import Counter
from pathlib import Path


def normalize_classes(values):
    """Return a stable, bounded, duplicate-free detector vocabulary."""
    if not isinstance(values, (list, tuple)) or not values:
        raise ValueError("at least one detection class is required")
    normalized = []
    for value in values:
        if not isinstance(value, str):
            raise ValueError("detection classes must be strings")
        item = " ".join(value.strip().lower().split())
        if not item or len(item) > 64:
            raise ValueError("detection class must contain 1..64 characters")
        if item not in normalized:
            normalized.append(item)
    if len(normalized) > 16:
        raise ValueError("at most 16 detection classes may be requested")
    return normalized


def _file_sha256(path):
    digest = hashlib.sha256()
    with Path(path).open("rb") as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def summarize_detections(results, classes, image_path, detector):
    """Normalize detector output without leaking model-specific tensors."""
    labels = results.get("labels", [])
    scores = results.get("scores", [])
    boxes = results.get("boxes", [])
    if not (len(labels) == len(scores) == len(boxes)):
        raise ValueError("detector returned mismatched labels, scores and boxes")
    detections = []
    for label, score, box in zip(labels, scores, boxes):
        label = str(label).strip().lower()
        if label not in classes:
            continue
        values = [round(float(value), 2) for value in box]
        if len(values) != 4:
            raise ValueError("detector box must have four coordinates")
        detections.append({"class": label, "confidence": round(float(score), 5),
                           "bbox_xyxy_px": values})
    counts = Counter(item["class"] for item in detections)
    return {
        "schema": "icarus.vision.detection.v1",
        "observed_at_unix_ms": int(time.time() * 1000),
        "image": {"path": str(Path(image_path).resolve()),
                  "sha256": _file_sha256(image_path)},
        "detector": detector,
        "requested_classes": list(classes),
        "counts": {name: counts.get(name, 0) for name in classes},
        "detections": detections,
    }


def detect_image(image_path, classes, manifest_path, threshold=0.35,
                 text_threshold=0.25, device="auto"):
    """Run the pinned Grounding-DINO artifact against one captured image."""
    classes = normalize_classes(classes)
    if not 0 < threshold <= 1 or not 0 < text_threshold <= 1:
        raise ValueError("thresholds must be in (0, 1]")
    manifest = json.loads(Path(manifest_path).read_text())
    detector = manifest.get("detector") or {}
    model_path = Path(detector.get("path", ""))
    if not (model_path / "config.json").is_file():
        raise FileNotFoundError("pinned detector is unavailable: " + str(model_path))

    import torch
    from PIL import Image
    from transformers import AutoModelForZeroShotObjectDetection, AutoProcessor

    selected_device = "cuda" if device == "auto" and torch.cuda.is_available() else (
        "cpu" if device == "auto" else device)
    processor = AutoProcessor.from_pretrained(model_path, local_files_only=True)
    model = AutoModelForZeroShotObjectDetection.from_pretrained(
        model_path, local_files_only=True).to(selected_device).eval()
    image = Image.open(image_path).convert("RGB")
    # Grounding-DINO uses full stops to delimit open-vocabulary noun phrases.
    prompt = ". ".join(classes) + "."
    inputs = processor(images=image, text=prompt, return_tensors="pt").to(selected_device)
    with torch.inference_mode():
        outputs = model(**inputs)
    processed = processor.post_process_grounded_object_detection(
        outputs, inputs.input_ids, threshold=threshold,
        text_threshold=text_threshold, target_sizes=[image.size[::-1]],
    )[0]
    result = summarize_detections(
        {"labels": processed.get("text_labels", processed["labels"]),
         "scores": processed["scores"],
         "boxes": processed["boxes"]},
        classes, image_path,
        {"repository": detector.get("repository"),
         "weights_sha256": detector.get("weights_sha256"),
         "threshold": threshold, "text_threshold": text_threshold,
         "device": selected_device},
    )
    result["image"]["width"] = image.width
    result["image"]["height"] = image.height
    return result

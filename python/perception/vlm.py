"""Bounded local VLM inspection; deliberately outside flight control.

The VLM receives one ephemeral image and one short question.  It cannot see a
Drone API client, MAVLink socket, mission state, filesystem paths from the
caller, or any action schema.  Its output is evidence for a human/DCM decision,
never a flight command or landing approval.
"""

from __future__ import annotations

import hashlib
import json
import subprocess
import time
from pathlib import Path


SCHEMA = "icarus.vision.qualitative.v1"
MAX_QUESTION_CHARS = 240
MAX_ANSWER_CHARS = 1200


def normalize_question(question: str) -> str:
    if not isinstance(question, str):
        raise ValueError("visual question must be text")
    compact = " ".join(question.strip().split())
    if not compact or len(compact) > MAX_QUESTION_CHARS:
        raise ValueError(f"visual question must be 1..{MAX_QUESTION_CHARS} characters")
    return compact


def _sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as source:
        for block in iter(lambda: source.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def load_vlm(manifest_path: Path) -> dict:
    manifest = json.loads(Path(manifest_path).read_text())
    for artifact in manifest.get("artifacts", []):
        if artifact.get("role") == "vlm":
            break
    else:
        raise FileNotFoundError("VLM artifact is not registered in manifest")
    required = {"path", "sha256", "mmproj_path", "mmproj_sha256", "runtime"}
    if not required <= set(artifact):
        raise ValueError("VLM manifest entry is incomplete")
    model, projector = Path(artifact["path"]), Path(artifact["mmproj_path"])
    if not model.is_file() or not projector.is_file():
        raise FileNotFoundError("VLM model or projector is unavailable")
    if _sha256(model) != artifact["sha256"] or _sha256(projector) != artifact["mmproj_sha256"]:
        raise ValueError("VLM artifact checksum mismatch")
    return artifact


def extract_answer(stdout: str) -> str:
    """Extract llama-cli's generated turn without retaining its footer."""
    lines = [line.strip() for line in stdout.splitlines() if line.strip()]
    timing = next((index for index, line in enumerate(lines)
                   if line.startswith("[ Prompt:")), None)
    if timing is not None:
        return lines[timing - 1] if timing else ""
    ignored = {"Exiting...", ">"}
    return next((line for line in reversed(lines)
                 if line not in ignored and not line.startswith("[")
                 and "=" not in line), "")


def inspect_image(image_path: Path, question: str, manifest_path: Path,
                  binary: Path | None = None, timeout_s: float = 45.0) -> dict:
    """Ask the pinned VLM one question about an image without retaining pixels."""
    question = normalize_question(question)
    image_path = Path(image_path)
    artifact = load_vlm(manifest_path)
    executable = binary or (Path(__file__).resolve().parents[2]
                            / "third_party/llama.cpp/build/bin/llama-cli")
    if not executable.is_file():
        raise FileNotFoundError("llama-cli is unavailable")
    if timeout_s <= 0:
        raise ValueError("VLM timeout must be positive")
    prompt = (
        "You are a visual inspection component. Describe only what is visible "
        "in the supplied image. Do not propose flight actions, landing sites, "
        "coordinates, or safety decisions. Answer in one concise sentence.\n"
        f"Question: {question}"
    )
    started = time.monotonic()
    completed = subprocess.run(
        [str(executable), "-m", artifact["path"], "--mmproj", artifact["mmproj_path"],
         "--image", str(image_path), "-ngl", "99", "-c", "1024", "--temp", "0",
         "-n", "96", "--single-turn", "--no-display-prompt", "-p", prompt],
        check=False, text=True, capture_output=True, timeout=timeout_s,
    )
    if completed.returncode:
        raise RuntimeError("VLM inference failed: " + completed.stderr[-500:])
    # llama-cli writes the generated response before its timing footer. Keep
    # the bounded, non-empty portion rather than pretending its prose is a
    # machine-validated measurement.
    answer = extract_answer(completed.stdout)
    if not answer:
        raise RuntimeError("VLM returned no inspection text")
    return {
        "schema": SCHEMA,
        "observed_at_unix_ms": int(time.time() * 1000),
        "question": question,
        "answer": answer[:MAX_ANSWER_CHARS],
        "latency_ms": round((time.monotonic() - started) * 1000, 3),
        "image": {"sha256": _sha256(image_path)},
        "runtime": {key: artifact.get(key) for key in
                    ("family", "quantization", "sha256", "mmproj_sha256", "repository", "revision")},
        "flight_authority": False,
    }

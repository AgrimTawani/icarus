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
        question = max((index for index, line in enumerate(lines[:timing])
                        if line.startswith("Question:")), default=None)
        if question is not None:
            # llama.cpp wraps longer generations across physical lines. Keep
            # the complete generation between the echoed question and timing
            # footer instead of returning only its last wrapped fragment.
            return " ".join(lines[question + 1:timing]).strip()
        return lines[timing - 1] if timing else ""
    ignored = {"Exiting...", ">"}
    return next((line for line in reversed(lines)
                 if line not in ignored and not line.startswith("[")
                 and "=" not in line), "")


def inspect_image(image_path: Path, question: str, manifest_path: Path,
                  binary: Path | None = None, timeout_s: float = 45.0,
                  gpu_layers: int = 99) -> dict:
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
    if type(gpu_layers) is not int or not 0 <= gpu_layers <= 999:
        raise ValueError("VLM gpu_layers must be an integer in 0..999")
    prompt = (
        "You are a visual inspection component. Describe only what is visible "
        "in the supplied image. Do not propose flight actions, landing sites, "
        "coordinates, or safety decisions. Answer in one concise sentence.\n"
        f"Question: {question}"
    )
    started = time.monotonic()
    command = [str(executable), "-m", artifact["path"], "--mmproj",
               artifact["mmproj_path"], "--image", str(image_path),
               "-ngl", str(gpu_layers)]
    if gpu_layers == 0:
        # ``-ngl 0`` keeps text layers on the CPU but llama.cpp may still put
        # the multimodal projector and host operations on CUDA. That is enough
        # to OOM beside the resident flight LLM on a 6 GB device. Make CPU-only
        # execution explicit for every backend component.
        command.extend(["--device", "none", "--no-op-offload",
                        "--no-mmproj-offload", "--mmproj-device", "none"])
    command.extend(["-c", "1024", "--temp", "0", "-n", "96",
                    "--single-turn", "--no-display-prompt", "-p", prompt])
    completed = subprocess.run(
        command,
        check=False, text=True, capture_output=True, timeout=timeout_s,
        stdin=subprocess.DEVNULL, start_new_session=True,
    )
    if completed.returncode:
        raise RuntimeError("VLM inference failed: " + completed.stderr[-500:])
    # llama-cli writes the generated response before its timing footer. Keep
    # the bounded, non-empty portion rather than pretending its prose is a
    # machine-validated measurement.
    answer = extract_answer(completed.stdout) or extract_answer(completed.stderr)
    if not answer:
        raise RuntimeError("VLM returned no inspection text")
    return {
        "schema": SCHEMA,
        "observed_at_unix_ms": int(time.time() * 1000),
        "question": question,
        "answer": answer[:MAX_ANSWER_CHARS],
        "latency_ms": round((time.monotonic() - started) * 1000, 3),
        "image": {"sha256": _sha256(image_path)},
        "runtime": ({key: artifact.get(key) for key in
                    ("family", "quantization", "sha256", "mmproj_sha256",
                     "repository", "revision")} | {"gpu_layers": gpu_layers}),
        "flight_authority": False,
    }

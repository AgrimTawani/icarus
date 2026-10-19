"""Bounded local VLM inspection; deliberately outside flight control.

The VLM receives one ephemeral image and one short question.  It cannot see a
Drone API client, MAVLink socket, mission state, filesystem paths from the
caller, or any action schema.  Its output is evidence for a human/DCM decision,
never a flight command or landing approval.
"""

from __future__ import annotations

import base64
import hashlib
import json
import math
import os
import signal
import socket
import subprocess
import time
import urllib.error
import urllib.request
from pathlib import Path

SCHEMA = "icarus.vision.qualitative.v1"
GROUNDING_SCHEMA = "icarus.vision.grounding.v1"
MAX_QUESTION_CHARS = 240
CONDITION_SCHEMA = {
    "type": "object", "properties": {
        "found": {"type": ["boolean", "null"]},
        "answer": {"type": "string", "maxLength": 300}},
    "required": ["found", "answer"], "additionalProperties": False,
}
COUNT_SCHEMA = {
    "type": "object", "properties": {
        "count": {"type": ["integer", "null"], "minimum": 0},
        "answer": {"type": "string", "maxLength": 300}},
    "required": ["count", "answer"], "additionalProperties": False,
}


class VlmInferenceError(RuntimeError):
    """Inference failed before producing usable visual evidence."""

    def __init__(self, code, detail):
        self.code = code
        super().__init__(detail)


def decode_measurement(answer, result_type="boolean"):
    """Validate evidence without coercing text, booleans, or missing counts."""
    if result_type not in {"boolean", "count"}:
        raise ValueError("VLM result_type must be boolean or count")
    key = "found" if result_type == "boolean" else "count"
    base = {"result_type": result_type, key: None, "value": None}
    if result_type == "count":
        base["count_scope"] = "visible_frame"
    try:
        value = json.loads(answer)
        if not isinstance(value, dict) or set(value) != {key, "answer"}:
            raise ValueError("missing or unexpected measurement fields")
        measured = value[key]
        if not isinstance(value["answer"], str) or len(value["answer"]) > 300:
            raise ValueError("measurement explanation must be bounded text")
        if measured is not None and not (
                type(measured) is bool if key == "found" else
                type(measured) is int and measured >= 0):
            raise ValueError("incorrect measurement type")
        return dict(base, **{key: measured, "value": measured,
                            "answer": value["answer"],
                            "validity": "unknown" if measured is None else "valid"})
    except (ValueError, TypeError):
        return dict(base, validity="unknown", error_code="malformed_output",
                    uncertainty="VLM did not return the requested typed measurement")


MAX_ANSWER_CHARS = 1200
MAX_DESCRIPTION_CHARS = 120


def measurement_grammar(result_type):
    """Use the pinned converter's grammar without assistant-template prefill.

    --json-schema is OUTPUT_FORMAT in common/arg.cpp; sampling.cpp prefills
    generation_prompt tokens into output-format grammars. --grammar is USER,
    whose contract explicitly starts at generated output. Both constrain the
    same JSON language; no free-text inference fallback is permitted.
    Regenerate with scripts/autonomy/generate_vlm_grammars.py after schema edits.
    """
    schemas = {"boolean": CONDITION_SCHEMA, "count": COUNT_SCHEMA}
    if result_type not in schemas:
        raise ValueError("VLM result_type must be boolean or count")
    artifact = json.loads(Path(__file__).with_name("vlm_grammars.json").read_text())
    encoded = json.dumps(schemas[result_type], sort_keys=True, separators=(",", ":"))
    entry = artifact[result_type]
    if hashlib.sha256(encoded.encode()).hexdigest() != entry["schema_sha256"]:
        raise VlmInferenceError("grammar_contract_mismatch",
                                "VLM schema changed; regenerate its constrained grammar")
    return entry["grammar"]


def normalize_question(question: str) -> str:
    if not isinstance(question, str):
        raise ValueError("visual question must be text")  # noqa: TRY004 - public validation contract
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


def extract_condition_answer(stdout, result_type="boolean"):
    """Decode a complete typed JSON reply, including multi-line CLI output."""
    decoder = json.JSONDecoder()
    for index, char in enumerate(stdout):
        if char != "{":
            continue
        try:
            value, _ = decoder.raw_decode(stdout[index:])
        except ValueError:
            continue
        encoded = json.dumps(value)
        if decode_measurement(encoded, result_type).get("error_code") is None:
            return encoded
    return extract_answer(stdout)  # caller records malformed output as unknown


def viewing_context_text(view_context):
    """Only measured viewing metadata, never world landmarks or target labels."""
    if not view_context:
        return ''
    heading = view_context.get('aircraft_heading_deg')
    if type(heading) not in (int, float) or not math.isfinite(heading):
        raise ValueError('VLM viewing heading must be finite')
    camera = view_context.get('camera')
    if camera not in ('forward_d435i', 'downward_brio'):
        raise ValueError('VLM viewing camera must be a known mounted camera')
    return (f'Measured aircraft compass heading at capture: {heading % 360:.1f} degrees '
            '(0 north, 90 east, 180 south, 270 west). '
            f'Camera: {camera}; mounted {"forward" if camera == "forward_d435i" else "downward"}. '
            'Compass labels refer to this measured view, not image up. '
            'This metadata is not evidence that an object is present.\n')


def inspect_image(image_path: Path, question: str, manifest_path: Path,
                  binary: Path | None = None, timeout_s: float = 45.0,
                  gpu_layers: int = 99, structured=False, result_type=None,
                  view_context=None) -> dict:
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
    if result_type is not None and result_type not in {"boolean", "count"}:
        raise ValueError("VLM result_type must be boolean or count")
    structured = structured or result_type is not None
    result_type = result_type or "boolean"
    answer_format = (
        "First describe the visible colors, surfaces and their image placement, then answer briefly. "
        "A cropped or partial surface does not establish the whole object's shape, height or distance. "
        "State which part is visible and what cannot be determined; do not complete unseen parts.")
    if structured:
        answer_format = (
            'Return JSON only: {"found":true or false or null,"answer":"brief visible evidence"}. '
            'Use null if uncertain.' if result_type == "boolean" else
            'Count only individually visible instances in this single image. Return JSON only: '
            '{"count":nonnegative integer or null,"answer":"brief visible evidence"}. '
            'Use null if the count is uncertain; never infer invisible instances.')
    prompt = (
        "You are a visual inspection component. Describe only what is visible "
        "in the supplied image. The question may assume a subject that is absent. "
        "First check whether the subject is actually visible; if absent or uncertain, say so "
        "instead of describing an assumed object. Do not infer visibility from the question, "
        "geographic labels or prior context. Do not propose flight actions, landing sites, "
        "coordinates, or safety decisions. " + answer_format + "\n"
        + viewing_context_text(view_context) + f"Question: {question}"
    )
    started = time.monotonic()
    cpu_threads = artifact.get('cpu_threads', 4)
    command = [str(executable), "-m", artifact["path"], "--mmproj",
               artifact["mmproj_path"], "--image", str(image_path),
               *_cpu_only_arguments(gpu_layers, cpu_threads)]
    if structured:
        command.extend(["--grammar", measurement_grammar(result_type)])
    # Image tokens share the language model's context with the prompt and
    # answer. Bound image resolution explicitly, and reserve a conservative
    # UTF-8 byte budget for text plus template/image framing. A fixed 1024-token
    # context can be smaller than the image alone.
    image_tokens = 1024
    answer_tokens = 160 if structured else 96
    required_context = image_tokens + len(prompt.encode("utf-8")) + answer_tokens + 256
    context_tokens = max(2048, ((required_context + 255) // 256) * 256)
    command.extend(["--image-max-tokens", str(image_tokens),
                    "-c", str(context_tokens), "--temp", "0", "-n", str(answer_tokens),
                    "--single-turn", "--no-display-prompt", "-p", prompt])
    try:
        completed = subprocess.run(
            command, check=False, text=True, capture_output=True, timeout=timeout_s,
            stdin=subprocess.DEVNULL, start_new_session=True,
        )
    except subprocess.TimeoutExpired as exc:
        raise VlmInferenceError("timeout", "VLM inference exceeded its deadline") from exc
    # The pinned CLI can report a server error and still exit zero. Diagnostics
    # and stderr logs are never a successful visual answer.
    diagnostics = completed.stdout + "\n" + completed.stderr
    error_lines = [line.strip() for line in diagnostics.splitlines()
                   if line.strip().startswith("Error:")]
    if completed.returncode or error_lines:
        code = ("sampler_initialization" if "Failed to initialize samplers" in diagnostics
                else "inference_failed")
        raise VlmInferenceError(code, "VLM inference failed: " +
                                (error_lines[-1] if error_lines else completed.stderr[-500:]))
    answer = (extract_condition_answer(completed.stdout, result_type) if structured
              else extract_answer(completed.stdout))
    if not answer:
        raise VlmInferenceError("empty_output", "VLM returned no inspection text")
    measurement = decode_measurement(answer, result_type) if structured else {}
    return {
        **measurement,
        "schema": SCHEMA,
        "observed_at_unix_ms": int(time.time() * 1000),
        "question": question,
        "answer": answer[:MAX_ANSWER_CHARS],
        "latency_ms": round((time.monotonic() - started) * 1000, 3),
        "image": {"sha256": _sha256(image_path)},
        "runtime": ({key: artifact.get(key) for key in
                    ("family", "quantization", "sha256", "mmproj_sha256",
                     "repository", "revision")} | {"gpu_layers": gpu_layers,
                     "cpu_threads": cpu_threads if gpu_layers == 0 else None,
                     "context_tokens": context_tokens, "image_max_tokens": image_tokens}),
        "view_context": view_context,
        "flight_authority": False,
    }


def _cpu_only_arguments(gpu_layers: int, cpu_threads: int = 4) -> list[str]:
    if type(cpu_threads) is not int or cpu_threads <= 0:
        raise ValueError('VLM cpu_threads must be a positive integer')
    arguments = ["-ngl", str(gpu_layers)]
    if gpu_layers == 0:
        # CPU offload must also leave scheduling capacity for perception and
        # control. The artifact can override the per-process worker budget.
        arguments.extend(["--device", "none", "--no-op-offload",
                          "--no-mmproj-offload", "--mmproj-device", "none",
                          "--threads", str(cpu_threads), "--threads-batch", str(cpu_threads)])
    return arguments


def _free_loopback_port() -> int:
    with socket.socket() as probe:
        probe.bind(("127.0.0.1", 0))
        return probe.getsockname()[1]


class _GroundingServer:
    """One short-lived, loopback-only llama-server for one image.

    A server (rather than llama-cli) lets the presence gate and the grounding
    question share one image encoding: the second question reuses the cached
    image prefix and costs about a second instead of a second full encode.
    The process lives only inside one ``ground_description`` call.
    """

    def __init__(self, executable: Path, artifact: dict, gpu_layers: int,
                 deadline: float):
        self.deadline = deadline
        self.port = _free_loopback_port()
        self.url = f"http://127.0.0.1:{self.port}"
        self.command = [str(executable), "-m", artifact["path"], "--mmproj",
                        artifact["mmproj_path"], *_cpu_only_arguments(gpu_layers, artifact.get('cpu_threads', 4)),
                        "-c", "2048", "-np", "1", "--host", "127.0.0.1",
                        "--port", str(self.port), "--no-webui", "--offline",
                        "--log-disable"]
        self.process = None
        self.marker = None

    def _remaining(self) -> float:
        remaining = self.deadline - time.monotonic()
        if remaining <= 0:
            raise TimeoutError("VLM grounding deadline exceeded")
        return remaining

    def _get(self, path: str):
        with urllib.request.urlopen(self.url + path,
                                    timeout=min(5.0, self._remaining())) as response:
            return json.loads(response.read())

    def __enter__(self):
        self.process = subprocess.Popen(
            self.command, stdin=subprocess.DEVNULL, stdout=subprocess.DEVNULL,
            stderr=subprocess.DEVNULL, start_new_session=True)
        try:
            while True:
                self._remaining()
                if self.process.poll() is not None:
                    raise RuntimeError("VLM server exited during startup")
                try:
                    if self._get("/health").get("status") == "ok":
                        break
                except (urllib.error.URLError, ConnectionError, OSError, ValueError):
                    pass
                time.sleep(0.2)
            self.marker = self._get("/props")["media_marker"]
            return self
        except BaseException:
            self.__exit__()
            raise

    def __exit__(self, *_):
        if self.process is None or self.process.poll() is not None:
            return
        try:
            os.killpg(self.process.pid, signal.SIGTERM)
            self.process.wait(timeout=5)
        except (ProcessLookupError, subprocess.TimeoutExpired):
            self.process.kill()
            self.process.wait()

    def complete(self, image_b64: str, question: str, prefill: str,
                 n_predict: int) -> str:
        # Qwen2-VL's own chat layout; the image marker is expanded by mtmd.
        prompt = ("<|im_start|>system\nYou are a helpful assistant.<|im_end|>\n"
                  "<|im_start|>user\n" + self.marker + question +
                  "<|im_end|>\n<|im_start|>assistant\n" + prefill)
        payload = {
            "prompt": {"prompt_string": prompt, "multimodal_data": [image_b64]},
            "temperature": 0, "n_predict": n_predict, "cache_prompt": True,
        }
        body = json.dumps(payload).encode()
        request = urllib.request.Request(
            self.url + "/completion", body, {"Content-Type": "application/json"})
        with urllib.request.urlopen(request, timeout=self._remaining()) as response:
            return json.loads(response.read()).get("content", "")


def normalize_description(description: str) -> str:
    if not isinstance(description, str):
        raise ValueError("target description must be text")  # noqa: TRY004 - public validation contract
    compact = " ".join(description.strip().split())
    if not compact or len(compact) > MAX_DESCRIPTION_CHARS:
        raise ValueError(
            f"target description must be 1..{MAX_DESCRIPTION_CHARS} characters")
    return compact


def presence_answer_is_yes(answer: str) -> bool:
    words = answer.strip().lower().replace(".", " ").replace(",", " ").split()
    return bool(words) and words[0] == "yes"


def ground_description(image_path: Path, description: str, manifest_path: Path,
                       binary: Path | None = None, timeout_s: float = 90.0,
                       gpu_layers: int = 0, view_context=None) -> dict:
    """Ask the pinned VLM whether a described thing is visible, and where.

    Two bounded questions on one image: a yes/no presence gate, then (only on
    yes) Qwen2-VL's native grounding format, forced by prefilling
    ``<|object_ref_start|>...<|box_start|>(`` so the answer is a box in the
    model's 0..1000 image coordinates.  The box is image evidence only; this
    function never produces metric coordinates or a confidence.
    """
    description = normalize_description(description)
    image_path = Path(image_path)
    artifact = load_vlm(manifest_path)
    if artifact.get("family") != "qwen2-vl":
        # The prefilled box tokens are Qwen2-VL's grounding format; another
        # model family would return an unvalidated imitation of a box.
        raise ValueError("VLM grounding requires the pinned qwen2-vl artifact")
    executable = binary or (Path(__file__).resolve().parents[2]
                            / "third_party/llama.cpp/build/bin/llama-server")
    if not executable.is_file():
        raise FileNotFoundError("llama-server is unavailable")
    if timeout_s <= 0:
        raise ValueError("VLM timeout must be positive")
    if type(gpu_layers) is not int or not 0 <= gpu_layers <= 999:
        raise ValueError("VLM gpu_layers must be an integer in 0..999")
    image_b64 = base64.b64encode(image_path.read_bytes()).decode()
    # Keep navigation metadata in the receipt for the pilot/geometry layer.
    # The visual referent task needs pixels and the target expression only:
    # the small VLM mistakes long heading instructions for visual constraints.
    # Validate supplied metadata without turning it into an image assertion.
    viewing_context_text(view_context)
    started = time.monotonic()
    with _GroundingServer(executable, artifact, gpu_layers,
                          started + timeout_s) as server:
        loaded = time.monotonic()
        presence = server.complete(
            image_b64, f"The requested target is: {description}. "
            "Can you identify the physical object referred to in the image? "
            "Answer yes or no.", "", 4).strip()
        first_word = presence.lower().replace('.', ' ').replace(',', ' ').split()
        if not first_word or first_word[0] not in ('yes', 'no'):
            raise VlmInferenceError('invalid_presence',
                                    'VLM did not provide a usable yes/no observation; presence is unknown')
        present = presence_answer_is_yes(presence)
        grounding = None
        if present:
            grounding = "(" + server.complete(
                image_b64, f"Locate the physical object referred to as: {description}.",
                f"<|object_ref_start|>the {description}<|object_ref_end|>"
                "<|box_start|>(", 24)
    finished = time.monotonic()
    return {
        "schema": GROUNDING_SCHEMA,
        "observed_at_unix_ms": int(time.time() * 1000),
        "description": description,
        "present": present,
        "presence_answer": presence[:MAX_ANSWER_CHARS],
        "grounding_text": None if grounding is None else grounding[:MAX_ANSWER_CHARS],
        "view_context": view_context,
        "latency_ms": round((finished - started) * 1000, 3),
        "model_load_ms": round((loaded - started) * 1000, 3),
        "image": {"sha256": _sha256(image_path)},
        "runtime": ({key: artifact.get(key) for key in
                    ("family", "quantization", "sha256", "mmproj_sha256",
                     "repository", "revision")} | {"gpu_layers": gpu_layers,
                                                   "cpu_threads": artifact.get('cpu_threads', 4) if gpu_layers == 0 else None,
                                                   "server": "llama-server"}),
        "flight_authority": False,
    }

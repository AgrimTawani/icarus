"""Model identity and process settings; independent of mission interpretation."""
import dataclasses
import hashlib
import json
import math
from pathlib import Path

DIRECT_PROMPT_VERSION = 'direct-agent-v3-retained-evidence'
DIRECT_TOOL_VERSION = 'direct-tools-v3-retained-results'


class DeadlineExceeded(Exception):
    """A model call exceeded its wall-clock deadline; no response is executed."""


class ContextBudgetExceeded(RuntimeError):
    """Token counting could not fit a request; inference has not started."""


@dataclasses.dataclass(frozen=True)
class RuntimeDescriptor:
    """Everything that must be recorded to make a decision reproducible."""

    model_id: str
    family: str
    quantization: str
    artifact_path: str
    artifact_sha256: str
    runtime: str
    runtime_revision: str
    context_length: int
    temperature: float
    deadline_ms: int
    prompt_version: str = DIRECT_PROMPT_VERSION
    contract_version: str = "direct-agent-v1"
    vocabulary_version: str = DIRECT_TOOL_VERSION
    reasoning_mode: str | None = None
    reasoning_budget: int | None = None
    gpu_layers: int | str = "auto"
    gpu_memory_margin_mib: int | None = None
    batch_size: int | None = None
    micro_batch_size: int | None = None
    # None leaves the pinned server's default unchanged for legacy manifests.
    top_p: float | None = None
    top_k: int | None = None
    min_p: float | None = None
    presence_penalty: float | None = None
    repetition_penalty: float | None = None
    # Native uint32 maximum is llama.cpp's random-seed sentinel.
    seed: int | None = None

    def __post_init__(self):
        for name in ("temperature", "top_p", "min_p", "presence_penalty", "repetition_penalty"):
            value = getattr(self, name)
            if value is None and name != "temperature":
                continue
            if type(value) not in (int, float) or not math.isfinite(value):
                raise ValueError(f"{name} must be a finite number")
            if name in ("top_p", "min_p") and not 0 <= value <= 1:
                raise ValueError(f"{name} must be between 0 and 1")
            if name == "temperature" and value < 0:
                raise ValueError("temperature must be nonnegative")
            if name == "repetition_penalty" and value <= 0:
                raise ValueError("repetition_penalty must be positive")
        if self.top_k is not None and (
                type(self.top_k) is not int or not 0 <= self.top_k <= 2**31 - 1):
            raise ValueError("top_k must be a nonnegative 32-bit integer or None")
        if self.seed is not None and (
                type(self.seed) is not int or not 0 <= self.seed <= 2**32 - 1):
            raise ValueError("seed must be an unsigned 32-bit integer or None")
        for name in ("context_length", "deadline_ms"):
            value = getattr(self, name)
            if type(value) is not int or value <= 0:
                raise ValueError(f"{name} must be a positive integer")
        if self.reasoning_budget is not None and (
                type(self.reasoning_budget) is not int or self.reasoning_budget < 0):
            raise ValueError("reasoning_budget must be a nonnegative integer or None")
        if self.reasoning_mode not in (None, "on", "off", "auto"):
            raise ValueError("reasoning_mode must be on, off, auto or None")
        if not ((type(self.gpu_layers) is int and self.gpu_layers >= 0)
                or (type(self.gpu_layers) is str and self.gpu_layers in ("auto", "all"))):
            raise ValueError("gpu_layers must be a nonnegative integer, auto or all")
        if self.gpu_memory_margin_mib is not None and (
                type(self.gpu_memory_margin_mib) is not int or self.gpu_memory_margin_mib <= 0):
            raise ValueError("gpu_memory_margin_mib must be a positive integer or None")
        for name in ("batch_size", "micro_batch_size"):
            value = getattr(self, name)
            if value is not None and (type(value) is not int or value <= 0):
                raise ValueError(f"{name} must be a positive integer or None")
        if self.batch_size and self.micro_batch_size and self.micro_batch_size > self.batch_size:
            raise ValueError("micro_batch_size must not exceed batch_size")

    @classmethod
    def from_manifest(cls, manifest_path, role="primary", default_context_length=8192, **overrides):
        """Build a descriptor from a setup-model-runtime manifest."""
        manifest = json.loads(Path(manifest_path).read_text())
        for artifact in manifest["artifacts"]:
            if artifact["role"] == role:
                break
        else:
            raise ValueError("no artifact with role " + role)
        fields = {
            "model_id": Path(artifact["path"]).stem,
            # A manifest may name its own family (needed once a manifest
            # holds more than one model family, as the vision manifest does);
            # falling back to "qwen" keeps existing single-family manifests
            # working without every one of them needing updating.
            "family": artifact.get("family", manifest.get("family", "qwen")),
            "quantization": artifact["quantization"],
            "artifact_path": artifact["path"],
            "artifact_sha256": artifact["sha256"],
            "runtime": "llama_cpp",
            "runtime_revision": manifest["llama_cpp_revision"],
            "reasoning_mode": artifact.get("reasoning_mode", manifest.get("reasoning_mode")),
            "reasoning_budget": artifact.get("reasoning_budget", manifest.get("reasoning_budget")),
            "gpu_layers": artifact.get("gpu_layers", manifest.get("gpu_layers", "auto")),
            "gpu_memory_margin_mib": artifact.get(
                "gpu_memory_margin_mib", manifest.get("gpu_memory_margin_mib")),
            "context_length": artifact.get("context_length", manifest.get("context_length", default_context_length)),
            "temperature": artifact.get("temperature", manifest.get("temperature", 0.0)),
            "deadline_ms": artifact.get("deadline_ms", manifest.get("deadline_ms", 15_000)),
        }
        for name in ("top_p", "top_k", "min_p", "presence_penalty", "repetition_penalty", "seed",
                     "batch_size", "micro_batch_size"):
            fields[name] = artifact.get(name, manifest.get(name))
        fields.update(overrides)
        return cls(**fields)

    def verify_artifact(self):
        """Re-hash the weights so a swapped or truncated file fails loudly."""
        path = Path(self.artifact_path)
        if not path.is_file():
            raise FileNotFoundError("model artifact missing: " + str(path))
        digest = hashlib.sha256()
        with path.open("rb") as handle:
            for block in iter(lambda: handle.read(1024 * 1024), b""):
                digest.update(block)
        actual = digest.hexdigest()
        if actual != self.artifact_sha256:
            raise ValueError(
                f"model artifact checksum mismatch for {path}: "
                f"expected {self.artifact_sha256}, found {actual}")
        return actual

    def as_record(self):
        """The provenance block written into every observe report."""
        return dataclasses.asdict(self)

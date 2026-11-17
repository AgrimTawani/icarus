"""Choose the mission runtime: a hosted model or the local llama.cpp server."""
from pathlib import Path

KINDS = ("cloud", "local")
# `llama` was the only runtime name before the hosted model existed.
ALIASES = {"llama": "local"}


def runtime_kind(name):
    """Canonical kind for a --runtime value (`llama` means `local`)."""
    kind = ALIASES.get(name, name)
    if kind not in KINDS:
        raise ValueError(f"unknown runtime {name!r}; use one of {', '.join(KINDS)}")
    return kind


def make_runtime(kind, manifest: Path, **overrides):
    """Return `(runtime, record)` for `kind`.

    local: `(LlamaCppRuntime(descriptor), descriptor)`, overrides go to
    `RuntimeDescriptor.from_manifest`. cloud: `(CloudRuntime(settings),
    settings.as_record())`, overrides go to `CloudSettings.from_manifest` and
    may raise `MissingCloudSetting`. The cloud record never holds a key.
    """
    kind = runtime_kind(kind)
    if kind == "local":
        from python.dcm.llama_runtime import LlamaCppRuntime
        from python.dcm.runtime_config import RuntimeDescriptor

        descriptor = RuntimeDescriptor.from_manifest(manifest, **overrides)
        return LlamaCppRuntime(descriptor), descriptor
    from python.model_runtime.cloud import CloudRuntime
    from python.model_runtime.cloud_config import CloudSettings

    settings = CloudSettings.from_manifest(manifest, **overrides)
    return CloudRuntime(settings), settings.as_record()

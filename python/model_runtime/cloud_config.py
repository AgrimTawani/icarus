"""Settings for the cloud (OpenAI-compatible) model runtime.

Limits come from the runtime manifest; endpoints, keys and model ids come from the
environment, so no secret is ever written to a file the repository tracks.
"""
import dataclasses
import json
import os
from collections.abc import Mapping
from pathlib import Path
from urllib.parse import urlparse


class MissingCloudSetting(ValueError):
    """A required cloud environment variable is unset or blank."""


class InvalidCloudSetting(MissingCloudSetting):
    """A cloud setting is present but unusable; callers handle it like a missing one."""


LOOPBACK_HOSTS = frozenset({"localhost", "127.0.0.1", "::1"})


def _require_https(name, url):
    """`url` unchanged if it is https (or plain http to this machine); else InvalidCloudSetting naming `name`.

    The API key travels in a header, so a plain-HTTP URL to another host would send it in clear.
    """
    parsed = urlparse(url)
    if parsed.scheme == "https" and parsed.hostname:
        return url
    if parsed.scheme == "http" and parsed.hostname in LOOPBACK_HOSTS:
        return url
    raise InvalidCloudSetting(
        f"{name} must be an https:// URL (plain http:// is allowed only for localhost, 127.0.0.1 or ::1)")


@dataclasses.dataclass(frozen=True)
class CloudSettings:
    base_url: str
    api_key: str = dataclasses.field(repr=False)
    model: str
    vision_base_url: str
    vision_api_key: str = dataclasses.field(repr=False)
    vision_model: str
    context_length: int
    deadline_ms: int
    max_retries: int
    vision: dict

    @classmethod
    def from_manifest(cls, path, environ: Mapping[str, str] = os.environ, **overrides):
        """Build settings from the manifest's `cloud` block plus ICARUS_LLM_* / ICARUS_VLM_*."""
        block = json.loads(Path(path).read_text())["cloud"]

        def value(name, default=None):
            found = (environ.get(name) or "").strip()
            if found:
                return found
            if default is None:
                raise MissingCloudSetting(f"{name} is not set")
            return default

        base_url = _require_https("ICARUS_LLM_BASE_URL", value("ICARUS_LLM_BASE_URL"))
        api_key = value("ICARUS_LLM_API_KEY")
        model = value("ICARUS_LLM_MODEL")
        vision = dict(block["vision"])
        vision["grounding_format"] = value("ICARUS_VLM_GROUNDING_FORMAT", vision["grounding_format"])
        from python.perception.vlm import JSON_BOX_FORMATS

        if vision["grounding_format"] not in JSON_BOX_FORMATS:
            source = ("ICARUS_VLM_GROUNDING_FORMAT" if (environ.get("ICARUS_VLM_GROUNDING_FORMAT") or "").strip()
                      else "cloud.vision.grounding_format in the manifest")
            raise InvalidCloudSetting(
                f"{source} is {vision['grounding_format']!r}; use one of {', '.join(JSON_BOX_FORMATS)}")
        fields = {
            "base_url": base_url.rstrip("/"),
            "api_key": api_key,
            "model": model,
            "vision_base_url": _require_https("ICARUS_VLM_BASE_URL", value("ICARUS_VLM_BASE_URL", base_url)).rstrip("/"),
            "vision_api_key": value("ICARUS_VLM_API_KEY", api_key),
            "vision_model": value("ICARUS_VLM_MODEL", model),
            "context_length": block["context_length"],
            "deadline_ms": block["deadline_ms"],
            "max_retries": block["max_retries"],
            "vision": vision,
        }
        fields.update(overrides)
        return cls(**fields)

    def as_record(self):
        """Provenance for reports: provider, models and limits; never a key."""
        return {
            "provider_host": urlparse(self.base_url).hostname,
            "model": self.model,
            "vision_provider_host": urlparse(self.vision_base_url).hostname,
            "vision_model": self.vision_model,
            "context_length": self.context_length,
            "deadline_ms": self.deadline_ms,
            "max_retries": self.max_retries,
            "vision": dict(self.vision),
        }

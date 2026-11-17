import json
from pathlib import Path

import pytest

from python.model_runtime.cloud_config import CloudSettings, MissingCloudSetting

REPO_MANIFEST = Path(__file__).resolve().parents[2] / "config" / "dcm" / "agent-runtime.json"
ENV = {"ICARUS_LLM_BASE_URL": "https://api.sarvam.ai/v2", "ICARUS_LLM_API_KEY": "sk-secret",
       "ICARUS_LLM_MODEL": "gemma4"}


def manifest(tmp_path, **cloud):
    block = {"context_length": 16384, "deadline_ms": 60000, "max_retries": 3,
             "vision": {"grounding_format": "json_norm1000_xyxy", "image_max_side_px": 1024,
                        "jpeg_quality": 85, "minimum_request_interval_s": 0.5}}
    block.update(cloud)
    path = tmp_path / "manifest.json"
    path.write_text(json.dumps({"artifacts": [], "cloud": block}))
    return path


def settings(tmp_path, environ=None, **overrides):
    return CloudSettings.from_manifest(manifest(tmp_path), environ=ENV if environ is None else environ, **overrides)


def settings_with_key(key):
    return CloudSettings.from_manifest(REPO_MANIFEST, environ={**ENV, "ICARUS_LLM_API_KEY": key})


def test_reads_env_and_defaults_vision_to_llm(tmp_path):
    s = settings(tmp_path)
    assert (s.base_url, s.api_key, s.model) == ("https://api.sarvam.ai/v2", "sk-secret", "gemma4")
    assert (s.vision_base_url, s.vision_api_key, s.vision_model) == ("https://api.sarvam.ai/v2", "sk-secret", "gemma4")
    assert s.deadline_ms == 60000 and s.context_length == 16384 and s.max_retries == 3
    assert s.vision["grounding_format"] == "json_norm1000_xyxy"
    assert s.vision["image_max_side_px"] == 1024


def test_vision_env_overrides(tmp_path):
    s = settings(tmp_path, environ={
        **ENV, "ICARUS_VLM_BASE_URL": "https://vision.example/v1", "ICARUS_VLM_API_KEY": "sk-vision",
        "ICARUS_VLM_MODEL": "qwen-vl", "ICARUS_VLM_GROUNDING_FORMAT": "json_pixel_xyxy"})
    assert (s.vision_base_url, s.vision_api_key, s.vision_model) == (
        "https://vision.example/v1", "sk-vision", "qwen-vl")
    assert (s.base_url, s.api_key, s.model) == ("https://api.sarvam.ai/v2", "sk-secret", "gemma4")
    assert s.vision["grounding_format"] == "json_pixel_xyxy"
    assert s.vision["jpeg_quality"] == 85


@pytest.mark.parametrize("name", ["ICARUS_LLM_BASE_URL", "ICARUS_LLM_API_KEY", "ICARUS_LLM_MODEL"])
def test_missing_setting_names_the_variable(tmp_path, name):
    environ = {key: value for key, value in ENV.items() if key != name}
    with pytest.raises(MissingCloudSetting, match=name):
        settings(tmp_path, environ=environ)


def test_blank_setting_counts_as_missing(tmp_path):
    with pytest.raises(MissingCloudSetting, match="ICARUS_LLM_API_KEY"):
        settings(tmp_path, environ={**ENV, "ICARUS_LLM_API_KEY": "  "})


def test_missing_setting_error_does_not_echo_the_key(tmp_path):
    environ = {key: value for key, value in ENV.items() if key != "ICARUS_LLM_MODEL"}
    with pytest.raises(MissingCloudSetting) as raised:
        settings(tmp_path, environ=environ)
    assert "sk-secret" not in str(raised.value)


def test_key_never_appears_in_repr_or_record(tmp_path):
    s = settings_with_key("sk-secret")
    assert "sk-secret" not in repr(s) and "sk-secret" not in json.dumps(s.as_record())
    assert s.as_record()["provider_host"] == "api.sarvam.ai"
    s = settings(tmp_path, environ={**ENV, "ICARUS_VLM_API_KEY": "sk-vision"})
    assert "sk-vision" not in repr(s) and "sk-vision" not in json.dumps(s.as_record())


def test_record_names_models_and_limits(tmp_path):
    record = settings(tmp_path).as_record()
    assert record["model"] == "gemma4" and record["vision_model"] == "gemma4"
    assert (record["context_length"], record["deadline_ms"], record["max_retries"]) == (16384, 60000, 3)
    assert not any("key" in name for name in record)


def test_deadline_override(tmp_path):
    assert settings(tmp_path, deadline_ms=5000).deadline_ms == 5000


def test_trailing_slash_is_stripped(tmp_path):
    s = settings(tmp_path, environ={**ENV, "ICARUS_LLM_BASE_URL": "https://api.sarvam.ai/v2/",
                                    "ICARUS_VLM_BASE_URL": "https://vision.example/v1//"})
    assert (s.base_url, s.vision_base_url) == ("https://api.sarvam.ai/v2", "https://vision.example/v1")


def test_shipped_manifest_has_the_cloud_block():
    s = settings_with_key("sk-secret")
    assert (s.deadline_ms, s.context_length, s.max_retries) == (60000, 16384, 3)
    assert s.vision == {"grounding_format": "json_norm1000_xyxy", "image_max_side_px": 1024,
                        "jpeg_quality": 85, "minimum_request_interval_s": 0.5}


@pytest.mark.parametrize("url", ["http://api.example.com/v1", "api.example.com/v1", "ftp://api.example.com", "https://"])
def test_a_non_https_base_url_is_refused_naming_the_variable(tmp_path, url):
    with pytest.raises(MissingCloudSetting, match="ICARUS_LLM_BASE_URL.*https"):
        settings(tmp_path, environ={**ENV, "ICARUS_LLM_BASE_URL": url})


@pytest.mark.parametrize("url", ["http://127.0.0.1:8080/v1", "http://localhost:8000/v1", "http://[::1]:8000/v1",
                                 "https://api.example.com/v1"])
def test_https_and_loopback_http_base_urls_are_accepted(tmp_path, url):
    assert settings(tmp_path, environ={**ENV, "ICARUS_LLM_BASE_URL": url}).base_url == url


def test_a_plain_http_vision_url_is_refused_naming_its_variable(tmp_path):
    with pytest.raises(MissingCloudSetting, match="ICARUS_VLM_BASE_URL"):
        settings(tmp_path, environ={**ENV, "ICARUS_VLM_BASE_URL": "http://vision.example.com/v1"})


def test_a_loopback_http_vision_url_is_accepted(tmp_path):
    s = settings(tmp_path, environ={**ENV, "ICARUS_VLM_BASE_URL": "http://127.0.0.1:8081/v1"})
    assert s.vision_base_url == "http://127.0.0.1:8081/v1"


def test_the_refusal_never_echoes_the_key(tmp_path):
    with pytest.raises(MissingCloudSetting) as raised:
        settings(tmp_path, environ={**ENV, "ICARUS_LLM_BASE_URL": "http://api.example.com/v1"})
    assert "sk-secret" not in str(raised.value)


@pytest.mark.parametrize("name", ["json_pixel_xyxy", "json_norm1000_xyxy"])
def test_known_grounding_formats_are_accepted_from_the_environment(tmp_path, name):
    assert settings(tmp_path, environ={**ENV, "ICARUS_VLM_GROUNDING_FORMAT": name}).vision["grounding_format"] == name


def test_an_unknown_grounding_format_from_the_environment_is_refused(tmp_path):
    with pytest.raises(MissingCloudSetting, match="ICARUS_VLM_GROUNDING_FORMAT.*json_pixel_xyxy"):
        settings(tmp_path, environ={**ENV, "ICARUS_VLM_GROUNDING_FORMAT": "qwen2vl_box_1000"})


def test_an_unknown_grounding_format_in_the_manifest_is_refused(tmp_path):
    path = manifest(tmp_path, vision={"grounding_format": "bogus", "image_max_side_px": 1024,
                                      "jpeg_quality": 85, "minimum_request_interval_s": 0.5})
    with pytest.raises(MissingCloudSetting, match="manifest.*bogus"):
        CloudSettings.from_manifest(path, environ=ENV)

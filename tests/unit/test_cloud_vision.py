import base64
import io
import json
import threading
import time
import urllib.error

import pytest
from PIL import Image
from test_cloud_runtime import KEY, FakeHttp, completion, http_error, settings

from python.model_runtime.cloud import CloudRuntime, CloudVision
from python.perception import vlm
from python.perception.vlm import (
    CONDITION_SCHEMA,
    COUNT_SCHEMA,
    DESCRIPTION_SCHEMA,
    PIXEL_BOX_SCHEMA,
    PIXEL_BOXES_SCHEMA,
    measurement_grammar,
)


def ppm_bytes(size=(848, 480)):
    buffer = io.BytesIO()
    Image.new("RGB", size, (10, 200, 30)).save(buffer, format="PPM")
    return buffer.getvalue()


PPM = ppm_bytes()
PPM_B64 = base64.b64encode(PPM).decode()


def vision_settings(**vision):
    block = {"grounding_format": "json_norm1000_xyxy", "image_max_side_px": 512, "jpeg_quality": 85,
             "minimum_request_interval_s": 0.5}
    block.update(vision)
    return settings(vision=block, vision_model="qwen-vl", vision_base_url="https://vision.example.com/v1",
                    vision_api_key="vk-secret")


def make_vision(**vision):
    return CloudRuntime(vision_settings(**vision)).vision()


@pytest.fixture
def fake_http(monkeypatch):
    http = FakeHttp()
    http.script = [completion({"role": "assistant", "content": '{"seen": "a house", "found": true}'},
                              finish="stop")]
    monkeypatch.setattr("python.model_runtime.cloud._open", http)
    return http


def sent_image(http):
    content = http.last_json["messages"][0]["content"]
    part = next(item for item in content if item["type"] == "image_url")
    url = part["image_url"]["url"]
    assert url.startswith("data:image/jpeg;base64,")
    return Image.open(io.BytesIO(base64.b64decode(url.split(",", 1)[1])))


def sent_text(http):
    content = http.last_json["messages"][0]["content"]
    return next(item for item in content if item["type"] == "text")["text"]


def test_artifact_and_shared_instance():
    runtime = CloudRuntime(vision_settings())
    vision = runtime.vision()
    assert vision is runtime.vision() and isinstance(vision, CloudVision)
    assert vision.artifact == {"family": "cloud", "model": "qwen-vl", "provider_host": "vision.example.com",
                               "resident": True, "grounding_format": "json_norm1000_xyxy",
                               "minimum_request_interval_s": 0.5}
    assert "vk-secret" not in repr(vision.artifact) and "vk-secret" not in repr(vision)


def test_request_uses_vision_endpoint_key_and_model(fake_http):
    vision = make_vision()
    answer = vision.complete(PPM_B64, "Is a house visible?", "", 160, grammar=measurement_grammar("boolean"))
    assert answer == '{"seen": "a house", "found": true}'
    request = fake_http.requests[-1][0]
    assert request.full_url == "https://vision.example.com/v1/chat/completions"
    assert dict(request.header_items())["Authorization"] == "Bearer vk-secret"
    body = fake_http.last_json
    assert body["model"] == "qwen-vl" and body["temperature"] == 0 and body["max_tokens"] == 160
    assert len(body["messages"]) == 1 and body["messages"][0]["role"] == "user"
    assert sent_text(fake_http) == "Is a house visible?"
    assert "tools" not in body and "tool_choice" not in body


def test_grammar_maps_back_to_its_schema(fake_http):
    vision = make_vision()
    cases = {"boolean": CONDITION_SCHEMA, "count": COUNT_SCHEMA, "description": DESCRIPTION_SCHEMA,
             "pixel_box": PIXEL_BOX_SCHEMA, "pixel_boxes": PIXEL_BOXES_SCHEMA}
    for result_type, schema in cases.items():
        vision.complete(PPM_B64, "Q?", "", 160, grammar=measurement_grammar(result_type))
        response_format = fake_http.last_json["response_format"]
        assert response_format["type"] == "json_schema"
        assert response_format["json_schema"]["schema"] == schema
        assert response_format["json_schema"]["strict"] is True and response_format["json_schema"]["name"]


def test_unknown_grammar_raises_valueerror(fake_http):
    vision = make_vision()
    with pytest.raises(ValueError, match="grammar"):
        vision.complete(PPM_B64, "Q?", "", 160, grammar='root ::= "x"')
    assert fake_http.requests == []


def test_no_grammar_sends_no_response_format(fake_http):
    make_vision().complete(PPM_B64, "Q?", "", 160)
    assert "response_format" not in fake_http.last_json


def test_ppm_is_sent_as_jpeg_data_url_and_downscaled(fake_http):
    make_vision().complete(PPM_B64, "Q?", "", 160, grammar=measurement_grammar("boolean"))
    image = sent_image(fake_http)
    assert image.format == "JPEG" and image.size == (512, 290)


def test_png_is_accepted_too(fake_http):
    buffer = io.BytesIO()
    Image.new("RGBA", (200, 100), (1, 2, 3, 255)).save(buffer, format="PNG")
    make_vision().complete(base64.b64encode(buffer.getvalue()).decode(), "Q?", "", 160)
    assert sent_image(fake_http).size == (200, 100)


def test_pixel_format_is_never_resized(fake_http):
    vision = make_vision(grounding_format="json_pixel_xyxy")
    vision.complete(PPM_B64, "Find it", "", 512, grammar=measurement_grammar("pixel_boxes"))
    assert sent_image(fake_http).size == (848, 480)


def test_undecodable_image_raises_valueerror(fake_http):
    with pytest.raises(ValueError, match="image"):
        make_vision().complete(base64.b64encode(b"not an image").decode(), "Q?", "", 160)
    assert fake_http.requests == []


def test_strict_schema_rejected_falls_back_to_json_object(fake_http):
    reply = completion({"role": "assistant", "content": '{"seen": "x", "found": false}'}, finish="stop")
    fake_http.script = [http_error(400, json.dumps({"error": {"message": "Unsupported response_format json_schema"}})),
                        reply]
    vision = make_vision()
    assert vision.complete(PPM_B64, "Q?", "", 160, grammar=measurement_grammar("boolean")) \
        == '{"seen": "x", "found": false}'
    assert len(fake_http.requests) == 2
    assert fake_http.last_json["response_format"] == {"type": "json_object"}
    assert json.dumps(CONDITION_SCHEMA) in sent_text(fake_http)
    # The fallback is remembered: the next call goes straight to json_object.
    fake_http.script = [reply]
    vision.complete(PPM_B64, "Q?", "", 160, grammar=measurement_grammar("boolean"))
    assert len(fake_http.requests) == 3 and fake_http.last_json["response_format"] == {"type": "json_object"}


def test_any_400_to_a_schema_request_falls_back_to_json_object(fake_http):
    reply = completion({"role": "assistant", "content": '{"seen": "x", "found": false}'}, finish="stop")
    fake_http.script = [http_error(400, json.dumps({"error": {"message": "bad request"}})), reply]
    vision = make_vision()
    assert vision.complete(PPM_B64, "Q?", "", 160, grammar=measurement_grammar("boolean")) \
        == '{"seen": "x", "found": false}'
    assert len(fake_http.requests) == 2 and fake_http.last_json["response_format"] == {"type": "json_object"}


@pytest.mark.parametrize("status", [404, 422])
def test_other_rejections_raise_without_fallback(fake_http, status):
    fake_http.script = [http_error(status, "response_format json_schema")]
    with pytest.raises(RuntimeError, match=f"HTTP {status}") as raised:
        make_vision().complete(PPM_B64, "Q?", "", 160, grammar=measurement_grammar("boolean"))
    assert len(fake_http.requests) == 1
    assert "json_schema" not in str(raised.value) and "vk-secret" not in str(raised.value)


def test_failed_fallback_does_not_drop_strict_schemas(fake_http):
    reply = completion({"role": "assistant", "content": '{"seen": "x", "found": true}'}, finish="stop")
    fake_http.script = [http_error(400, "image too large"), http_error(400, "image too large")]
    vision = make_vision()
    with pytest.raises(RuntimeError, match="HTTP 400") as raised:
        vision.complete(PPM_B64, "Q?", "", 160, grammar=measurement_grammar("boolean"))
    assert "image too large" not in str(raised.value) and len(fake_http.requests) == 2
    fake_http.script = [reply]
    vision.complete(PPM_B64, "Q?", "", 160, grammar=measurement_grammar("boolean"))
    assert len(fake_http.requests) == 3 and fake_http.last_json["response_format"]["type"] == "json_schema"


def test_second_rejection_after_fallback_raises(fake_http):
    fake_http.script = [http_error(400, "response_format is not supported")]
    with pytest.raises(RuntimeError, match="HTTP 400"):
        make_vision().complete(PPM_B64, "Q?", "", 160, grammar=measurement_grammar("boolean"))
    assert len(fake_http.requests) == 2


def test_prefill_is_prepended_to_question_not_sent_as_assistant(fake_http):
    make_vision().complete(PPM_B64, "Is a house visible?", '{"seen": "', 160)
    body = fake_http.last_json
    assert [m["role"] for m in body["messages"]] == ["user"]
    text = sent_text(fake_http)
    assert text.startswith("Is a house visible?") and text.endswith('{"seen": "')


def test_missing_content_is_empty_string(fake_http):
    fake_http.script = [{"choices": [{"message": {"role": "assistant"}, "finish_reason": "stop"}]}]
    assert make_vision().complete(PPM_B64, "Q?", "", 160) == ""
    fake_http.script = [{"choices": []}]
    assert make_vision().complete(PPM_B64, "Q?", "", 160) == ""


def test_resident_callers_work_unchanged(fake_http, tmp_path):
    fake_http.script = [completion({"role": "assistant", "content": '{"bboxes": [[100, 100, 200, 200]]}'},
                                   finish="stop")]
    path = tmp_path / "frame.ppm"
    path.write_bytes(PPM)
    vision = make_vision(grounding_format="json_pixel_xyxy")
    result = vlm.ground_all(path, "house", tmp_path / "unused-manifest.json", server=vision)
    assert result["boxes"] == [[100, 100, 200, 200]]
    assert result["source_width"] == 848 and sent_image(fake_http).size == (848, 480)


def test_resident_inspect_image_reads_the_answer(fake_http, tmp_path):
    path = tmp_path / "frame.ppm"
    path.write_bytes(PPM)
    result = vlm.inspect_image(path, "Is a house visible?", tmp_path / "unused-manifest.json",
                               structured=True, result_type="boolean", server=make_vision())
    assert result["found"] is True and result["value"] is True


def test_timeout_raises_timeouterror(monkeypatch):
    release = threading.Event()

    def slow(request, timeout=None):
        release.wait(5)
        raise AssertionError("late call must have been abandoned")

    monkeypatch.setattr("python.model_runtime.cloud._open", slow)
    started = time.monotonic()
    try:
        with pytest.raises(TimeoutError):
            make_vision().complete(PPM_B64, "Q?", "", 160, timeout_s=0.1)
        assert time.monotonic() - started < 2
    finally:
        release.set()


def test_provider_deadline_becomes_timeouterror(monkeypatch):
    def refuse(request, timeout=None):
        raise urllib.error.URLError(TimeoutError("timed out"))

    monkeypatch.setattr("python.model_runtime.cloud._open", refuse)
    with pytest.raises(TimeoutError):
        make_vision().complete(PPM_B64, "Q?", "", 160, timeout_s=5)


def test_failures_never_carry_the_key(monkeypatch):
    def reject(request, timeout=None):
        raise http_error(401)

    monkeypatch.setattr("python.model_runtime.cloud._open", reject)
    with pytest.raises(RuntimeError, match="API key") as raised:
        make_vision().complete(PPM_B64, "Q?", "", 160)
    assert "vk-secret" not in str(raised.value) and KEY not in str(raised.value)

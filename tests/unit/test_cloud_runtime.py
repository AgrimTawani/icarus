import io
import json
import threading
import time
import urllib.error

import pytest

from python.dcm.runtime_config import (
    ContextBudgetExceeded,
    DeadlineExceeded,
    IncompleteModelResponse,
)
from python.model_runtime.cloud import CloudRuntime
from python.model_runtime.cloud_config import CloudSettings

TOOLS = [{"type": "function", "function": {"name": "takeoff", "parameters": {"type": "object"}}}]
USER = [{"role": "user", "content": "go"}]
KEY = "sk-secret"


def settings(**overrides):
    fields = {"base_url": "https://api.sarvam.ai/v2", "api_key": KEY, "model": "gemma4",
              "vision_base_url": "https://api.sarvam.ai/v2", "vision_api_key": KEY, "vision_model": "gemma4",
              "context_length": 16384, "deadline_ms": 60000, "max_retries": 3,
              "vision": {"grounding_format": "json_norm1000_xyxy"}}
    fields.update(overrides)
    return CloudSettings(**fields)


def completion(message=None, finish="tool_calls", usage=None):
    message = message or {"role": "assistant", "content": None, "tool_calls": [
        {"id": "c1", "type": "function", "function": {"name": "takeoff", "arguments": "{}"}}]}
    usage = usage or {"prompt_tokens": 8400, "completion_tokens": 90, "completion_time": 0.3,
                      "prompt_tokens_details": {"cached_tokens": 8000}}
    return {"choices": [{"message": message, "finish_reason": finish}], "usage": usage}


class Reply:
    def __init__(self, body):
        self.body = json.dumps(body).encode()

    def read(self, *_):
        return self.body

    def __enter__(self):
        return self

    def __exit__(self, *_):
        return False


def http_error(code, body="", retry_after=None):
    headers = {"Retry-After": str(retry_after)} if retry_after is not None else {}
    return urllib.error.HTTPError("https://api.sarvam.ai/v2/chat/completions", code, "err", headers,
                                  io.BytesIO(body.encode()))


class FakeHttp:
    """Scripted urlopen: each item is a JSON body to return or an exception to raise."""

    def __init__(self):
        self.script = []
        self.requests = []

    def __call__(self, request, timeout=None):
        self.requests.append((request, timeout))
        item = self.script.pop(0) if len(self.script) > 1 else self.script[0]
        if isinstance(item, BaseException):
            raise item
        return Reply(item)

    @property
    def last_json(self):
        return json.loads(self.requests[-1][0].data.decode())

    @property
    def last_headers(self):
        return dict(self.requests[-1][0].header_items())


@pytest.fixture
def fake_http(monkeypatch):
    http = FakeHttp()
    http.script = [completion()]
    monkeypatch.setattr("python.model_runtime.cloud.urllib.request.urlopen", http)
    return http


class FakeClock:
    def __init__(self):
        self.now = 1000.0
        self.sleeps = []

    def monotonic(self):
        return self.now

    def sleep(self, seconds):
        self.sleeps.append(seconds)
        self.now += seconds


@pytest.fixture
def fake_clock():
    return FakeClock()


def runtime(clock=None, **overrides):
    rt = CloudRuntime(settings(**overrides))
    if clock is not None:
        rt._clock, rt._sleep = clock.monotonic, clock.sleep
    return rt


def test_request_body_is_plain_openai(fake_http):
    runtime().decide(USER, TOOLS)
    body = fake_http.last_json
    assert body["model"] == "gemma4" and body["tool_choice"] == "required" and body["temperature"] == 0
    assert body["messages"] == USER and body["tools"] == TOOLS
    for llama_only in ("cache_prompt", "id_slot", "chat_template_kwargs", "reasoning_budget_tokens",
                       "top_k", "min_p", "repeat_penalty", "parallel_tool_calls"):
        assert llama_only not in body
    assert fake_http.last_headers["Authorization"] == f"Bearer {KEY}"
    assert fake_http.requests[-1][0].full_url == "https://api.sarvam.ai/v2/chat/completions"


def test_returns_message_and_stats(fake_http):
    rt = runtime()
    msg = rt.decide(USER, TOOLS)
    assert msg["tool_calls"][0]["function"]["name"] == "takeoff"
    assert rt.last_stats == {"tokens_prompt": 8400, "tokens_cached": 8000, "tokens_prefilled": 400,
                             "tokens_predicted": 90, "predicted_per_second": 300.0, "retries": 0,
                             "provider_host": "api.sarvam.ai", "model": "gemma4"}
    assert rt.descriptor_record == settings().as_record()


def test_stats_without_provider_timing_or_cache(fake_http):
    fake_http.script = [completion(usage={"prompt_tokens": 100, "completion_tokens": 5})]
    rt = runtime()
    rt.decide(USER, TOOLS)
    assert rt.last_stats["predicted_per_second"] is None
    assert (rt.last_stats["tokens_cached"], rt.last_stats["tokens_prefilled"]) == (0, 100)


def test_concise_is_accepted_and_changes_nothing(fake_http):
    rt = runtime()
    rt.decide(USER, TOOLS, concise=True)
    assert fake_http.last_json["messages"] == USER
    assert "concise" not in rt.last_stats


def test_content_only_reply_is_returned_without_tool_calls(fake_http):
    fake_http.script = [completion({"role": "assistant", "content": "Which building?"}, finish="stop")]
    msg = runtime().decide(USER, TOOLS)
    assert msg["content"] == "Which building?" and not msg.get("tool_calls")


@pytest.mark.parametrize("text", ["context_length_exceeded", "This model's Maximum Context Length is 8192",
                                  "too many tokens", "exceeds the context window"])
def test_context_overflow_becomes_context_budget_exceeded(fake_http, text):
    fake_http.script = [http_error(400, json.dumps({"error": {"message": text}}))]
    with pytest.raises(ContextBudgetExceeded):
        runtime().decide(USER, TOOLS)
    assert len(fake_http.requests) == 1


def test_length_finish_becomes_incomplete(fake_http):
    fake_http.script = [completion({"role": "assistant", "content": "partial"}, finish="length")]
    with pytest.raises(IncompleteModelResponse):
        runtime().decide(USER, TOOLS)


@pytest.mark.parametrize("text", ["tool_use_failed", "Failed to call a function."])
def test_tool_use_failed_becomes_incomplete(fake_http, text):
    fake_http.script = [http_error(400, json.dumps({"error": {"message": text}}))]
    with pytest.raises(IncompleteModelResponse):
        runtime().decide(USER, TOOLS)


def test_429_retries_then_succeeds(fake_http, fake_clock):
    fake_http.script = [http_error(429, retry_after=1), completion()]
    rt = runtime(fake_clock)
    rt.decide(USER, TOOLS)
    assert rt.last_stats["retries"] == 1 and fake_clock.sleeps == [1.0]
    assert len(fake_http.requests) == 2


def test_server_error_backs_off_exponentially_then_gives_up(fake_http, fake_clock):
    fake_http.script = [http_error(503)]
    rt = runtime(fake_clock, max_retries=2)
    with pytest.raises(RuntimeError, match="503") as raised:
        rt.decide(USER, TOOLS)
    assert not isinstance(raised.value, DeadlineExceeded)
    assert fake_clock.sleeps == [1, 2] and len(fake_http.requests) == 3
    assert "api.sarvam.ai" in str(raised.value) and KEY not in str(raised.value)


def test_unreachable_provider_retries_then_names_host(fake_http, fake_clock):
    fake_http.script = [urllib.error.URLError("connection refused")]
    rt = runtime(fake_clock, max_retries=1)
    with pytest.raises(RuntimeError, match="api.sarvam.ai"):
        rt.decide(USER, TOOLS)
    assert len(fake_http.requests) == 2


def test_retry_after_past_deadline_raises_deadline_without_sleeping(fake_http, fake_clock):
    fake_http.script = [http_error(429, retry_after=120)]
    rt = runtime(fake_clock, deadline_ms=2000)
    with pytest.raises(DeadlineExceeded):
        rt.decide(USER, TOOLS)
    assert sum(fake_clock.sleeps) < 2 and len(fake_http.requests) == 1


def test_each_attempt_is_bounded_by_the_time_left(fake_http, fake_clock):
    fake_http.script = [http_error(429, retry_after=1), completion()]
    runtime(fake_clock, deadline_ms=10000).decide(USER, TOOLS)
    first, second = (timeout for _, timeout in fake_http.requests)
    assert first == pytest.approx(10.0) and second == pytest.approx(9.0)


def test_socket_timeout_is_a_missed_deadline(fake_http):
    fake_http.script = [urllib.error.URLError(TimeoutError("timed out"))]
    with pytest.raises(DeadlineExceeded):
        runtime().decide(USER, TOOLS)


@pytest.mark.parametrize("code", [401, 403])
def test_auth_error_raises_runtimeerror_without_key(fake_http, code):
    fake_http.script = [http_error(code, json.dumps({"error": {"message": f"bad key {KEY}"}}))]
    with pytest.raises(RuntimeError) as raised:
        runtime().decide(USER, TOOLS)
    assert str(code) in str(raised.value) and "api.sarvam.ai" in str(raised.value)
    assert KEY not in str(raised.value) and KEY not in repr(raised.value)
    assert len(fake_http.requests) == 1


def test_stats_and_record_never_hold_the_key(fake_http):
    rt = runtime()
    rt.decide(USER, TOOLS)
    assert KEY not in json.dumps(rt.last_stats) and KEY not in json.dumps(rt.descriptor_record)


class SlowHttp:
    def __init__(self):
        self.entered = threading.Event()
        self.release = threading.Event()

    def __call__(self, request, timeout=None):
        self.entered.set()
        self.release.wait(5)
        return Reply(completion())


def test_stop_abandons_inflight_and_discards_late_reply(monkeypatch):
    slow = SlowHttp()
    monkeypatch.setattr("python.model_runtime.cloud.urllib.request.urlopen", slow)
    rt = runtime()
    outcome = {}

    def call():
        try:
            outcome["message"] = rt.decide(USER, TOOLS)
        except BaseException as error:  # noqa: BLE001
            outcome["error"] = error

    caller = threading.Thread(target=call)
    caller.start()
    assert slow.entered.wait(2)
    started = time.monotonic()
    rt.stop()
    assert time.monotonic() - started < 0.2
    caller.join(0.5)
    assert not caller.is_alive()
    assert isinstance(outcome.get("error"), (InterruptedError, DeadlineExceeded)) and "message" not in outcome
    slow.release.set()  # the late reply arrives after the abandon and is dropped
    time.sleep(0.05)
    assert rt.last_stats is None and "message" not in outcome


def test_a_stopped_runtime_serves_the_next_decision(fake_http):
    rt = runtime()
    rt.stop()
    assert rt.decide(USER, TOOLS)["tool_calls"]


def test_deadline_abandons_a_hung_call(monkeypatch):
    slow = SlowHttp()
    monkeypatch.setattr("python.model_runtime.cloud.urllib.request.urlopen", slow)
    started = time.monotonic()
    with pytest.raises(DeadlineExceeded):
        runtime(deadline_ms=100).decide(USER, TOOLS)
    assert time.monotonic() - started < 0.5
    slow.release.set()


def test_start_checks_models_endpoint(fake_http):
    fake_http.script = [{"data": []}]
    CloudRuntime(settings()).start()
    request, timeout = fake_http.requests[-1]
    assert request.full_url == "https://api.sarvam.ai/v2/models" and timeout == 5
    assert dict(request.header_items())["Authorization"] == f"Bearer {KEY}"


def test_warm_starts(fake_http):
    fake_http.script = [{"data": []}]
    CloudRuntime(settings()).warm(TOOLS)
    assert fake_http.requests[-1][0].full_url.endswith("/models")


def test_start_unreachable_names_host_and_local_fallback(fake_http):
    fake_http.script = [urllib.error.URLError("no route")]
    with pytest.raises(RuntimeError) as raised:
        CloudRuntime(settings()).start()
    assert "api.sarvam.ai" in str(raised.value) and "--runtime local" in str(raised.value)


def test_start_rejected_key_hides_key(fake_http):
    fake_http.script = [http_error(401, KEY)]
    with pytest.raises(RuntimeError) as raised:
        CloudRuntime(settings()).start()
    assert "401" in str(raised.value) and KEY not in str(raised.value)


def test_start_without_scheme_asks_for_https():
    with pytest.raises(RuntimeError, match="https://"):
        CloudRuntime(settings(base_url="api.sarvam.ai/v2")).start()

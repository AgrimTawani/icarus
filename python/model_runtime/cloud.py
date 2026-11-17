"""Cloud (OpenAI-compatible) runtime for mission decisions.

It speaks plain chat-completions to a hosted model and exposes the same
duck-typed surface `fly_mission` uses on `LlamaCppRuntime`: `decide`, `stop`,
`last_stats`. The request carries no llama.cpp-only fields, so any provider that
follows the OpenAI wire format can serve it.

The deadline is wall-clock and covers retries and waits, not just one socket
operation: the whole attempt loop runs on a daemon worker thread that the caller
joins until the deadline, and `stop()` or a missed deadline abandons it so a late
reply is dropped. The API key is sent only in the Authorization header; it is
never put in a message, a stat or a record.
"""
import base64
import http.client
import io
import json
import threading
import time
import urllib.error
import urllib.request
from urllib.parse import urlparse

from python.dcm.runtime_config import (
    ContextBudgetExceeded,
    DeadlineExceeded,
    IncompleteModelResponse,
)
from python.perception import vlm

START_TIMEOUT_S = 5
RETRYABLE_STATUS = frozenset({429, 500, 502, 503, 504})
AUTH_STATUS = frozenset({401, 403})
CONTEXT_MARKERS = ("context_length_exceeded", "maximum context length", "too many tokens", "context window")
INCOMPLETE_MARKERS = ("tool_use_failed", "failed to call a function")
ERROR_BODY_LIMIT = 4096
MAX_TOKENS = 768  # the reply cap LlamaCppRuntime.decide uses


class RejectedRequest(RuntimeError):
    """The provider refused the request itself (a non-retryable 4xx).

    `detail` is the provider's error text, lower-cased and size-limited, with the
    API key removed; callers classify on it. It is never part of the message.
    """

    def __init__(self, message, status, detail):
        super().__init__(message)
        self.status = status
        self.detail = detail


class _NoRedirect(urllib.request.HTTPRedirectHandler):
    """Never follow a redirect: urllib would copy the Authorization header to the new host."""

    def redirect_request(self, *_args, **_kwargs):
        return None


_OPENER = urllib.request.build_opener(_NoRedirect)


def _open(request, timeout):
    """urlopen without redirect following; every cloud request goes through here."""
    return _OPENER.open(request, timeout=timeout)


def _redirected(host, status):
    return RuntimeError(f"{host} redirected the request (HTTP {status}); redirects are not followed so the "
                        "API key is never sent elsewhere; use the provider's final base URL")


def _host(url):
    return urlparse(url).hostname or url


def _retry_after(failure):
    value = failure.headers.get("Retry-After") if failure.headers is not None else None
    try:
        seconds = float(value)
    except (TypeError, ValueError):
        return None
    return seconds if seconds >= 0 else None


def _error_text(failure):
    try:
        return failure.read(ERROR_BODY_LIMIT).decode("utf-8", errors="replace").lower()
    except Exception:  # noqa: BLE001 - an unreadable error body is classified as an ordinary failure
        return ""


def _post_json(url, key, body, deadline_at, max_retries, *, clock=time.monotonic, sleep=time.sleep,
               abandoned=None, stats=None):
    """POST `body` as JSON and return the decoded reply, within `deadline_at`.

    Retries 429/5xx and connection failures up to `max_retries` times, waiting
    Retry-After (else 2**attempt seconds) between attempts; a wait that would pass
    the deadline raises DeadlineExceeded instead of sleeping. Every failure message
    names the status and provider host only: never a header or the request body.
    The number of retries used is written to `stats["retries"]`.
    """
    host = _host(url)
    data = json.dumps(body).encode("utf-8")
    headers = {"Content-Type": "application/json", "Authorization": f"Bearer {key}"}
    try:
        request = urllib.request.Request(url, data=data, headers=headers)
    except ValueError:
        raise RuntimeError(f"{host} is not a usable URL") from None
    if stats is not None:
        stats["retries"] = 0
    attempt = 0
    while True:
        remaining = deadline_at - clock()
        if remaining <= 0:
            raise DeadlineExceeded(f"no response from {host} before the deadline")
        wait = None
        try:
            with _open(request, timeout=remaining) as response:
                raw = response.read()
        except urllib.error.HTTPError as failure:
            status = failure.code
            text = _error_text(failure)
            failure.close()
            if status in AUTH_STATUS:
                raise RuntimeError(f"{host} rejected the API key (HTTP {status})") from None
            if 300 <= status < 400:
                raise _redirected(host, status) from None
            if status not in RETRYABLE_STATUS:
                if any(marker in text for marker in CONTEXT_MARKERS):
                    raise ContextBudgetExceeded(
                        f"{host} reported the request exceeds its context window (HTTP {status})") from None
                if any(marker in text for marker in INCOMPLETE_MARKERS):
                    raise IncompleteModelResponse(
                        f"{host} could not form a valid tool call (HTTP {status})") from None
                raise RejectedRequest(f"{host} returned HTTP {status}", status,
                                      text.replace(str(key).lower(), "") if key else text) from None
            failed = RuntimeError(f"{host} returned HTTP {status} after {attempt} retries")
            wait = _retry_after(failure)
        except urllib.error.URLError as failure:
            if isinstance(failure.reason, TimeoutError):
                raise DeadlineExceeded(f"no response from {host} before the deadline") from None
            failed = RuntimeError(f"cannot reach {host}: {' '.join(str(failure.reason).split())[:200]}")
        except TimeoutError:
            raise DeadlineExceeded(f"no response from {host} before the deadline") from None
        except http.client.InvalidURL:
            raise RuntimeError(f"{host} is not a usable URL") from None
        except (http.client.HTTPException, ConnectionError) as failure:
            # A reset, early disconnect or short body part-way through the reply: transient.
            failed = RuntimeError(f"connection to {host} broke mid-response ({type(failure).__name__})")
        else:
            try:
                return json.loads(raw.decode("utf-8"))
            except ValueError:
                raise RuntimeError(f"{host} returned a reply that is not JSON") from None
        if attempt >= max_retries:
            raise failed
        wait = float(2 ** attempt) if wait is None else wait
        if wait > deadline_at - clock():
            raise DeadlineExceeded(f"{host} asked to wait {wait:g} s, which would pass the deadline")
        sleep(wait)
        if abandoned is not None and abandoned.is_set():
            raise InterruptedError("model call abandoned")
        attempt += 1
        if stats is not None:
            stats["retries"] = attempt


class CloudRuntime:
    """Mission decisions from a hosted OpenAI-compatible model."""

    name = "cloud"

    def __init__(self, settings):
        self.settings = settings
        self.last_stats = None
        self.descriptor_record = settings.as_record()
        self._clock = time.monotonic
        self._sleep = None  # None: wait on the abandon flag, so stop() cuts a backoff short
        self._current = None
        self._lock = threading.Lock()
        self._vision = None

    @property
    def _provider_host(self):
        return self.descriptor_record["provider_host"]

    def start(self):
        """Check the provider answers and accepts the key; no model call is made."""
        host = self._provider_host
        if host is None:
            raise RuntimeError(f"cloud base URL {self.settings.base_url!r} needs a scheme such as https://")
        request = urllib.request.Request(
            self.settings.base_url + "/models", headers={"Authorization": f"Bearer {self.settings.api_key}"})
        try:
            with _open(request, timeout=START_TIMEOUT_S) as response:
                response.read()
        except urllib.error.HTTPError as failure:
            status = failure.code
            failure.close()
            if status in AUTH_STATUS:
                raise RuntimeError(f"cloud provider {host} rejected the API key (HTTP {status}); "
                                   "use --runtime local to fly offline") from None
            if 300 <= status < 400:
                raise _redirected(f"cloud provider {host}", status) from None
            # Any other status means the provider answered; some have no /models route.
        except (urllib.error.URLError, TimeoutError, http.client.HTTPException, ConnectionError) as failure:
            reason = failure.reason if isinstance(failure, urllib.error.URLError) else failure
            raise RuntimeError(f"cannot reach cloud provider {host} ({' '.join(str(reason).split())[:200]}); "
                               "use --runtime local to fly offline") from None

    def warm(self, tools):
        self.start()

    def vision(self):
        """The one shared server for camera questions (a plain HTTP call: no lock with decisions)."""
        with self._lock:
            if self._vision is None:
                self._vision = CloudVision(self)
            return self._vision

    def stop(self):
        """Abandon the in-flight call, if any; its late reply is dropped."""
        with self._lock:
            current = self._current
        if current is not None:
            abandoned, finished = current
            abandoned.set()
            finished.set()

    def decide(self, messages, tools, *, concise=False):
        """Return one assistant message (content and/or tool_calls), or raise.

        `concise` is accepted for the loop's retry and changes nothing: a hosted
        model has no thinking switch to turn off.
        """
        body = {"model": self.settings.model, "messages": messages, "tools": tools,
                "tool_choice": "required", "temperature": 0, "max_tokens": MAX_TOKENS}
        timeout_s = self.settings.deadline_ms / 1000
        abandoned, finished = threading.Event(), threading.Event()
        outcome = {}
        stats = {}
        sleep = self._sleep or (lambda seconds: abandoned.wait(seconds))
        deadline_at = self._clock() + timeout_s

        def call():
            try:
                outcome["payload"] = _post_json(
                    self.settings.base_url + "/chat/completions", self.settings.api_key, body, deadline_at,
                    self.settings.max_retries, clock=self._clock, sleep=sleep, abandoned=abandoned, stats=stats)
            except BaseException as failure:  # noqa: BLE001 - relayed to the deciding thread
                outcome["failure"] = failure
            finally:
                finished.set()

        with self._lock:
            self._current = (abandoned, finished)
        worker = threading.Thread(target=call, daemon=True)
        worker.start()
        finished.wait(timeout_s)
        # Decide the verdict before reading the outcome: whatever the worker writes
        # after this point must not change a decision already made.
        if abandoned.is_set():
            raise InterruptedError("model call abandoned")
        if not finished.is_set():
            abandoned.set()
            raise DeadlineExceeded(f"no response within {self.settings.deadline_ms} ms")
        with self._lock:
            if self._current == (abandoned, finished):
                self._current = None
        if "failure" in outcome:
            raise outcome["failure"]
        return self._message(outcome["payload"], stats)

    def _message(self, payload, stats):
        usage = payload.get("usage") or {}
        prompt = usage.get("prompt_tokens")
        completion = usage.get("completion_tokens")
        cached = ((usage.get("prompt_tokens_details") or {}).get("cached_tokens")) or 0
        seconds = usage.get("completion_time")
        self.last_stats = {
            "tokens_prompt": prompt,
            "tokens_cached": cached,
            "tokens_prefilled": prompt - cached if prompt is not None else None,
            "tokens_predicted": completion,
            "predicted_per_second": completion / seconds if completion and seconds else None,
            "retries": stats.get("retries", 0),
            "provider_host": self._provider_host,
            "model": self.settings.model,
        }
        choices = payload.get("choices") or []
        if not choices:
            raise RuntimeError(f"{self._provider_host} returned no choices")
        choice = choices[0]
        if choice.get("finish_reason") == "length":
            raise IncompleteModelResponse(
                f"{self._provider_host} output was truncated at the token limit; "
                f"prompt tokens={prompt}, generated tokens={completion}. No complete tool response was returned.")
        message = choice.get("message") or {}
        return {key: message[key] for key in ("role", "content", "tool_calls") if key in message}


RESULT_TYPES = ("boolean", "count", "description", "pixel_box", "pixel_boxes")
PIXEL_FORMAT = "json_pixel_xyxy"


class CloudVision:
    """Camera questions answered by the hosted multimodal model.

    Same `complete` surface as `ResidentVision`, so the camera tools keep their
    prompts and answer schemas. The tools hand over a llama.cpp grammar; it is
    mapped back to the JSON schema it was generated from and sent as a strict
    `json_schema` response format. A provider that answers that with a 400 is retried once
    with `json_object` plus the schema in the prompt; that fallback is remembered only after
    it succeeds, so an unrelated 400 changes nothing.
    """

    def __init__(self, runtime):
        self.runtime = runtime
        settings = runtime.settings
        self._settings = settings
        self._schemas = {vlm.measurement_grammar(kind): (kind, schema) for kind, schema in (
            ("boolean", vlm.CONDITION_SCHEMA), ("count", vlm.COUNT_SCHEMA),
            ("description", vlm.DESCRIPTION_SCHEMA), ("pixel_box", vlm.PIXEL_BOX_SCHEMA),
            ("pixel_boxes", vlm.PIXEL_BOXES_SCHEMA))}
        self._json_object = False  # set once the provider refuses json_schema
        record = runtime.descriptor_record
        self.artifact = {"family": "cloud", "model": settings.vision_model,
                         "provider_host": record["vision_provider_host"], "resident": True,
                         "grounding_format": settings.vision["grounding_format"],
                         "minimum_request_interval_s": settings.vision["minimum_request_interval_s"]}

    def __repr__(self):
        return f"CloudVision(model={self.artifact['model']!r}, provider_host={self.artifact['provider_host']!r})"

    def _jpeg_data_url(self, image_b64):
        """The camera file as a JPEG data URL; shrunk only when the box format is resolution-free.

        Pixel-box answers are coordinates in the image the model saw, and the
        callers read them against the source frame, so that image is never resized.
        """
        from PIL import Image
        try:
            with Image.open(io.BytesIO(base64.b64decode(image_b64))) as source:
                image = source.convert("RGB")
        except Exception:  # noqa: BLE001 - PIL raises many types for corrupt input
            raise ValueError("camera image could not be decoded") from None
        if self._settings.vision["grounding_format"] != PIXEL_FORMAT:
            side = self._settings.vision["image_max_side_px"]
            image.thumbnail((side, side))
        buffer = io.BytesIO()
        image.save(buffer, format="JPEG", quality=self._settings.vision["jpeg_quality"])
        return "data:image/jpeg;base64," + base64.b64encode(buffer.getvalue()).decode()

    def _body(self, text, data_url, n_predict, response_format):
        body = {"model": self._settings.vision_model, "temperature": 0, "max_tokens": n_predict,
                "messages": [{"role": "user", "content": [
                    {"type": "text", "text": text}, {"type": "image_url", "image_url": {"url": data_url}}]}]}
        if response_format is not None:
            body["response_format"] = response_format
        return body

    def complete(self, image_b64, question, prefill, n_predict, *, grammar=None, timeout_s=45.0):
        """The model's text answer; TimeoutError if no answer arrives within `timeout_s`."""
        entry = None
        if grammar is not None:
            entry = self._schemas.get(grammar)
            if entry is None:
                raise ValueError("grammar is not one of the measurement grammars")
        data_url = self._jpeg_data_url(image_b64)
        text = question + (f"\nStart your answer with: {prefill}" if prefill else "")
        url = self._settings.vision_base_url + "/chat/completions"
        runtime = self.runtime
        abandoned, finished = threading.Event(), threading.Event()
        outcome = {}
        deadline_at = runtime._clock() + timeout_s

        def post(response_format, prompt):
            return _post_json(url, self._settings.vision_api_key,
                              self._body(prompt, data_url, n_predict, response_format), deadline_at,
                              self._settings.max_retries, clock=runtime._clock,
                              sleep=lambda seconds: abandoned.wait(seconds), abandoned=abandoned)

        def call():
            try:
                if entry is None:
                    outcome["payload"] = post(None, text)
                    return
                kind, schema = entry
                fallback = (f"{text}\nReply with only a JSON object that matches this JSON schema: "
                            f"{json.dumps(schema)}")
                if not self._json_object:
                    try:
                        outcome["payload"] = post({"type": "json_schema", "json_schema": {
                            "name": f"icarus_{kind}", "strict": True, "schema": schema}}, text)
                        return
                    except RejectedRequest as failure:
                        if failure.status != 400:
                            raise
                        # Any 400 to a json_schema request may be the schema; json_object is
                        # remembered only if it succeeds, so an unrelated 400 changes nothing.
                        outcome["payload"] = post({"type": "json_object"}, fallback)
                        self._json_object = True
                        return
                outcome["payload"] = post({"type": "json_object"}, fallback)
            except BaseException as failure:  # noqa: BLE001 - relayed to the asking thread
                outcome["failure"] = failure
            finally:
                finished.set()

        threading.Thread(target=call, daemon=True).start()
        finished.wait(timeout_s)
        if not finished.is_set():
            abandoned.set()
            raise TimeoutError(f"no answer from {self.artifact['provider_host']} within {timeout_s:g} s")
        failure = outcome.get("failure")
        if isinstance(failure, DeadlineExceeded):
            raise TimeoutError(str(failure)) from None
        if failure is not None:
            raise failure
        choices = outcome["payload"].get("choices") or []
        message = (choices[0].get("message") if choices else None) or {}
        return message.get("content") or ""

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
import json
import socket
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

START_TIMEOUT_S = 5
RETRYABLE_STATUS = frozenset({429, 500, 502, 503, 504})
AUTH_STATUS = frozenset({401, 403})
CONTEXT_MARKERS = ("context_length_exceeded", "maximum context length", "too many tokens", "context window")
INCOMPLETE_MARKERS = ("tool_use_failed", "failed to call a function")
ERROR_BODY_LIMIT = 4096


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
    if stats is not None:
        stats["retries"] = 0
    attempt = 0
    while True:
        remaining = deadline_at - clock()
        if remaining <= 0:
            raise DeadlineExceeded(f"no response from {host} before the deadline")
        wait = None
        try:
            request = urllib.request.Request(url, data=data, headers=headers)
            with urllib.request.urlopen(request, timeout=remaining) as response:
                return json.loads(response.read().decode("utf-8"))
        except urllib.error.HTTPError as failure:
            status = failure.code
            text = _error_text(failure)
            failure.close()
            if status in AUTH_STATUS:
                raise RuntimeError(f"{host} rejected the API key (HTTP {status})") from None
            if status not in RETRYABLE_STATUS:
                if any(marker in text for marker in CONTEXT_MARKERS):
                    raise ContextBudgetExceeded(
                        f"{host} reported the request exceeds its context window (HTTP {status})") from None
                if any(marker in text for marker in INCOMPLETE_MARKERS):
                    raise IncompleteModelResponse(
                        f"{host} could not form a valid tool call (HTTP {status})") from None
                raise RuntimeError(f"{host} returned HTTP {status}") from None
            failed = RuntimeError(f"{host} returned HTTP {status} after {attempt} retries")
            wait = _retry_after(failure)
        except urllib.error.URLError as failure:
            if isinstance(failure.reason, (TimeoutError, socket.timeout)):
                raise DeadlineExceeded(f"no response from {host} before the deadline") from None
            failed = RuntimeError(f"cannot reach {host}: {' '.join(str(failure.reason).split())[:200]}")
        except TimeoutError:
            raise DeadlineExceeded(f"no response from {host} before the deadline") from None
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
            with urllib.request.urlopen(request, timeout=START_TIMEOUT_S) as response:
                response.read()
        except urllib.error.HTTPError as failure:
            status = failure.code
            failure.close()
            reason = f"rejected the API key (HTTP {status})" if status in AUTH_STATUS else f"returned HTTP {status}"
            raise RuntimeError(f"cloud provider {host} {reason}; use --runtime local to fly offline") from None
        except (urllib.error.URLError, TimeoutError) as failure:
            reason = failure.reason if isinstance(failure, urllib.error.URLError) else failure
            raise RuntimeError(f"cannot reach cloud provider {host} ({' '.join(str(reason).split())[:200]}); "
                               "use --runtime local to fly offline") from None

    def warm(self, tools):
        self.start()

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
                "tool_choice": "required", "temperature": 0}
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

"""llama.cpp adapter for the DCM, with a deadline enforced at the boundary.

The runtime owns a `llama-server` subprocess and speaks to it over HTTP. It
implements `contract.ModelRuntime` and nothing more: it does not decide what
is observable, what is allowed, or what counts as fresh.

Why a server rather than one `llama-cli` per decision: a decision loop asks
many times, and reloading multi-gigabyte weights per question would dominate
the measured latency and tell us nothing about the model.

The deadline is real and measured in wall-clock time. The request runs on a
worker thread and is abandoned when `deadline_ms` passes, and the server is
then terminated and restarted. A socket timeout alone is not enough: urllib
applies it per socket operation, so a server that keeps trickling bytes can
run well past the deadline without ever raising. This is the property the
first observe slice lacked, and a per-operation timeout would not have
supplied it either.

Live missions use native tools and a measured feedback loop. The process and
HTTP transport do not interpret, compile, admit, or review missions.
"""

import contextlib
import dataclasses
import json
import os
import re
import select
import socket
import subprocess
import threading
import time
import urllib.error
import urllib.request
from pathlib import Path

from python.dcm.runtime_config import ContextBudgetExceeded, DeadlineExceeded, RuntimeDescriptor

DEFAULT_BINARY = (Path(__file__).resolve().parents[2]
                  / "third_party/llama.cpp/build/bin/llama-server")
STARTUP_TIMEOUT_S = 180
STDERR_TAIL_BYTES = 64 * 1024


class IncompleteModelResponse(RuntimeError):
    """No tool executed; one concise retry may repair unusable model output."""


VISION_IMAGE_MAX_TOKENS = 1024
# CPU encoding time scales with image tokens. On 848x480 Gazebo frames, 256
# tokens answered the same 10/10 building questions at 3.2 s instead of 5.1 s,
# with grounding boxes within ~15 px of the 1024-token boxes; 160 loosened them.
VISION_CPU_IMAGE_MAX_TOKENS = 256
# The mission conversation and camera questions each own a server slot, so an
# image never displaces the conversation. With one shared slot llama-server
# copied the displaced prompt into its RAM cache on every switch: measured
# ~110 MiB per camera question (1.9 -> 4.2 GiB after 15), and an 88-frame
# survey evicted the mission conversation, costing ~9 s of prefill afterwards.
MISSION_SLOT, VISION_SLOT = 0, 1
VISION_SLOT_TEXT_TOKENS = 256  # question, prefill and answer around the image


class ResidentVision:
    """Image questions answered by the mission runtime's own multimodal server.

    Same ``complete`` interface as the short-lived VLM server in
    python/perception/vlm.py, so the camera tools keep their prompts,
    grammars and decoding unchanged. Requests are serialized with mission
    decisions on the runtime's inference lock.
    """

    # Qwen3.5 boxes are 0-1000 normalized whatever the prompt says.
    artifact_fields = {"grounding_format": "json_norm1000_xyxy", "minimum_request_interval_s": 0.5}

    def __init__(self, runtime):
        self.runtime = runtime
        descriptor = runtime.descriptor
        self.artifact = {"family": descriptor.family, "quantization": descriptor.quantization,
                         "sha256": descriptor.artifact_sha256, "mmproj_sha256": descriptor.mmproj_sha256,
                         "path": descriptor.artifact_path, "mmproj_path": descriptor.mmproj_path,
                         "resident": True, **self.artifact_fields}
        self._marker = None

    @property
    def marker(self):
        if self._marker is None:
            with urllib.request.urlopen(self.runtime.endpoint + "/props", timeout=5) as response:
                self._marker = json.loads(response.read())["media_marker"]
        return self._marker

    def complete(self, image_b64, question, prefill, n_predict, *, grammar=None, timeout_s=45.0):
        runtime = self.runtime
        if not runtime._inference_lock.acquire(timeout=timeout_s):
            raise TimeoutError("mission model busy; image question not sent")
        try:
            if runtime.process is None:
                runtime.start()
            # Thinking stays off for camera answers, as for decisions.
            prompt = ("<|im_start|>user\n" + self.marker + question + "<|im_end|>\n"
                      "<|im_start|>assistant\n<think>\n\n</think>\n\n" + prefill)
            payload = {"prompt": {"prompt_string": prompt, "multimodal_data": [image_b64]},
                       "temperature": 0, "n_predict": n_predict, "cache_prompt": True,
                       "id_slot": VISION_SLOT}
            if grammar is not None:
                payload["grammar"] = grammar
            request = urllib.request.Request(runtime.endpoint + "/completion", json.dumps(payload).encode(),
                                             {"Content-Type": "application/json"})
            with urllib.request.urlopen(request, timeout=timeout_s) as response:
                return json.loads(response.read()).get("content", "")
        finally:
            runtime._inference_lock.release()


CPU_PROJECTOR_NICE = 10


def _yield_cpu_to_flight_stack():
    os.nice(CPU_PROJECTOR_NICE)


def _free_port():
    with socket.socket() as probe:
        probe.bind(("127.0.0.1", 0))
        return probe.getsockname()[1]


class LlamaCppRuntime:
    """Run a pinned local GGUF behind the provider-neutral contract."""

    mission_protocol = "direct-agent-v1"

    def __init__(self, descriptor, binary=None, host="127.0.0.1", port=None,
                 gpu_layers=None, verify_artifact=True, extra_args=(),
                 ending_guidance=False):
        if not isinstance(descriptor, RuntimeDescriptor):
            raise TypeError("descriptor must be a RuntimeDescriptor")
        self.descriptor = (descriptor if gpu_layers is None else
                           dataclasses.replace(descriptor, gpu_layers=gpu_layers))
        self.binary = Path(binary or DEFAULT_BINARY)
        self.host = host
        self.port = port or _free_port()
        self.gpu_layers = self.descriptor.gpu_layers
        self.verify_artifact = verify_artifact
        # Explicit experimental CLI flags override manifest settings. Normalize
        # them into the descriptor so provenance reflects the effective budget,
        # and emit a single server flag rather than relying on duplicate flags.
        self.extra_args = []
        arguments = iter(extra_args)
        for argument in arguments:
            if argument == "--reasoning-budget":
                value = next(arguments, None)
                if value is None:
                    raise ValueError("--reasoning-budget requires a nonnegative integer")
            elif isinstance(argument, str) and argument.startswith("--reasoning-budget="):
                value = argument.split("=", 1)[1]
            else:
                self.extra_args.append(argument)
                continue
            if not isinstance(value, str) or not value.isascii() or not value.isdecimal():
                raise ValueError("--reasoning-budget requires a nonnegative integer")
            self.descriptor = dataclasses.replace(self.descriptor, reasoning_budget=int(value))
        # The runtime renders the prompt, so it owns which variant is used.
        self.ending_guidance = ending_guidance
        self.process = None
        # The model identity belongs in the name so a report cannot be
        # mistaken for one produced by a different artifact.
        self.name = f"llama_cpp:{descriptor.model_id}"
        # Speed and token counts from the most recent propose(), as reported
        # by llama-server itself. None until a decision has actually run.
        self.last_stats = None
        self._inference_lock = threading.Lock()
        self._stderr_tail = bytearray()
        self._stderr_lock = threading.Lock()
        self._stderr_stop = threading.Event()
        self._stderr_thread = None
        # The session's native tool catalogue, when known before start(). The
        # chat template renders it at the top of every prompt, so warming
        # without it caches nothing a decision can reuse.
        self.warm_tools = None
        # None until the first start measures whether model and projector both fit.
        self.projector_on_gpu = None

    # ---------------------------------------------------------------- server
    @property
    def endpoint(self):
        return f"http://{self.host}:{self.port}"

    def start(self):
        if self.process is not None:
            return self
        if not self.binary.is_file():
            raise FileNotFoundError(
                f"llama-server missing at {self.binary}; "
                "run ./scripts/setup-model-runtime")
        if self.binary.resolve() == Path(DEFAULT_BINARY).resolve():
            from python.dcm.runtime_patches import verify_runtime_build
            verify_runtime_build(self.binary, self.descriptor.runtime_revision,
                Path(__file__).resolve().parents[2] / 'config/dependencies/patches/llama-tagged-argument-order.patch')
        if self.verify_artifact:
            # Fail before loading rather than after a whole evaluation run.
            self.descriptor.verify_artifact()
        command = [
            str(self.binary),
            "-m", self.descriptor.artifact_path,
            "--host", self.host,
            "--port", str(self.port),
            # _inference_lock permits one in-flight request. Automatic parallel
            # slots allocate unused recurrent/KV state and fragment context; a
            # declared projector gets exactly one extra slot for camera questions,
            # sharing one KV pool, and no RAM prompt cache (nothing is displaced).
            *(("--parallel", "2", "--kv-unified", "--cache-ram", "0") if self.descriptor.mmproj_path
              else ("--parallel", "1")),
            "-c", str(self.descriptor.context_length),
            "-ngl", str(self.gpu_layers),
            "--fit", "on",
            *self.extra_args,
        ]
        if self.descriptor.gpu_memory_margin_mib is not None:
            command.extend(["--fit-target", str(self.descriptor.gpu_memory_margin_mib)])
        if self.descriptor.mmproj_path:
            on_cpu = self.projector_on_gpu is False
            command.extend(["--mmproj", self.descriptor.mmproj_path, "--image-max-tokens",
                            str(VISION_CPU_IMAGE_MAX_TOKENS if on_cpu else VISION_IMAGE_MAX_TOKENS)])
            if on_cpu:
                command.append("--no-mmproj-offload")
            # Info level carries llama.cpp's "offloaded N/M layers" report that
            # decides projector placement (~2.5 KB of stderr per request).
            command.extend(["-lv", "4"])
        # Bound individual prefill kernels when rendering and inference share
        # an accelerator; the manifest records the effective scheduling policy.
        for name, flag in (("batch_size", "--batch-size"),
                           ("micro_batch_size", "--ubatch-size")):
            if getattr(self.descriptor, name) is not None:
                command.extend([flag, str(getattr(self.descriptor, name))])
        if self.descriptor.reasoning_budget is not None:
            command.extend(["--reasoning-budget", str(self.descriptor.reasoning_budget)])
        if self.descriptor.reasoning_mode is not None:
            command.extend(["--reasoning", self.descriptor.reasoning_mode])
        self.process = subprocess.Popen(
            command, stdout=subprocess.DEVNULL, stderr=subprocess.PIPE,
            # A CPU projector encodes every camera frame on the CPU. The flight
            # stack's sensor bridges must win that contention: measured AirSim
            # surveys aborted twice on stale perception without this.
            preexec_fn=_yield_cpu_to_flight_stack if self.projector_on_gpu is False else None)
        self._start_stderr_drain()
        try:
            self._await_ready()
            if self._projector_displaced_layers():
                # Decisions come first: every model layer on the GPU halves
                # decision time (measured 20 -> 42 tok/s beside Unreal), while
                # the projector on the CPU adds ~1.5 s per camera question.
                self.stop()
                self.projector_on_gpu = False
                return self.start()
            self._warm()
        except BaseException:
            self.stop()
            raise
        return self

    def _projector_displaced_layers(self):
        """True when the GPU projector pushed model layers to the CPU (llama.cpp's own report)."""
        if not self.descriptor.mmproj_path or self.projector_on_gpu is False or self.gpu_layers == 0:
            return False
        match = re.search(r"offloaded (\d+)/(\d+) layers to GPU", self._stderr_text())
        if match is None:
            return False
        self.projector_on_gpu = int(match.group(1)) == int(match.group(2))
        return not self.projector_on_gpu

    def _start_stderr_drain(self):
        """Drain diagnostics continuously without unbounded RAM or log files."""
        self._stderr_tail = bytearray()
        self._stderr_stop.clear()
        descriptor = self.process.stderr.fileno()
        os.set_blocking(descriptor, False)
        def drain():
            shutdown_bytes = 0
            while True:
                try:
                    block = os.read(descriptor, 8192)
                    if not block:
                        return
                    with self._stderr_lock:
                        self._stderr_tail.extend(block)
                        del self._stderr_tail[:-STDERR_TAIL_BYTES]
                    if self._stderr_stop.is_set():
                        shutdown_bytes += len(block)
                        if shutdown_bytes >= STDERR_TAIL_BYTES:
                            return
                except BlockingIOError:
                    if self._stderr_stop.is_set():
                        return
                    select.select([descriptor], [], [], .1)
                except OSError:
                    return
        self._stderr_thread = threading.Thread(target=drain, name="llama-stderr", daemon=True)
        self._stderr_thread.start()

    def _stderr_text(self):
        with self._stderr_lock:
            return bytes(self._stderr_tail).decode("utf-8", "replace")

    def _warm(self):
        """Prefill the fixed prompt prefix before the mission clock starts.

        The prefix is the system prompt plus the native tool catalogue, which
        the chat template renders first; ``warm_tools`` must be the exact
        catalogue the mission loop will send, or the first decision re-prefills
        everything (~8 s on the edge GPU instead of ~1 s). Warming is
        legitimate: the prefix is fixed and known before any mission is typed.
        """
        from python.dcm.agent_context import system_prompt
        request_body = {
            "messages": [{"role": "system", "content": system_prompt()},
                         {"role": "user", "content": "Awaiting an operator mission."}],
            "temperature": 0.0, "stream": False, "max_tokens": 1, "cache_prompt": True,
            **self._mission_slot(),
        }
        if self.warm_tools:
            request_body.update(tools=self.warm_tools,
                                chat_template_kwargs={"enable_thinking": False})
        request = urllib.request.Request(
            self.endpoint + "/v1/chat/completions", data=json.dumps(request_body).encode("utf-8"),
            headers={"Content-Type": "application/json"})
        # A failed warmup is not fatal: the first decision simply pays the
        # cost it would have paid anyway.
        with contextlib.suppress(Exception), \
                urllib.request.urlopen(request, timeout=STARTUP_TIMEOUT_S) as r:
            r.read()

    def warm(self, tools):
        """Cache the session's system prompt and tool catalogue on the running server."""
        self.warm_tools = tools
        if self.process is None:
            self.start()
        else:
            with self._inference_lock:
                self._warm()

    def _mission_slot(self):
        return {"id_slot": MISSION_SLOT} if self.descriptor.mmproj_path else {}

    def _vision_slot_tokens(self):
        """Context the camera slot can hold in the shared KV pool."""
        if not self.descriptor.mmproj_path:
            return 0
        image = VISION_CPU_IMAGE_MAX_TOKENS if self.projector_on_gpu is False else VISION_IMAGE_MAX_TOKENS
        return image + VISION_SLOT_TEXT_TOKENS

    def vision(self):
        """The camera-question interface of this resident server, or None.

        Only a primary artifact declared with its multimodal projector serves
        images. Camera questions then share the mission model's server: no
        second model, no unload. They use their own slot (VISION_SLOT), so the
        mission conversation stays in place whatever the camera is asked.
        """
        return ResidentVision(self) if self.descriptor.mmproj_path else None

    def _await_ready(self):
        deadline = time.monotonic() + STARTUP_TIMEOUT_S
        while time.monotonic() < deadline:
            if self.process.poll() is not None:
                self.stop()
                stderr = self._stderr_text()
                raise RuntimeError("llama-server exited during startup:\n"
                                   + stderr[-2000:])
            try:
                with urllib.request.urlopen(self.endpoint + "/health",
                                            timeout=2) as response:
                    if response.status == 200:
                        return
            except (urllib.error.URLError, TimeoutError, ConnectionError, OSError):
                time.sleep(0.25)
        self.stop()
        raise TimeoutError(
            f"llama-server not ready within {STARTUP_TIMEOUT_S}s\n"
            + self._stderr_text()[-2000:])

    def stop(self):
        process = self.process
        if process is None:
            return
        if process.poll() is None:
            with contextlib.suppress(ProcessLookupError):
                process.terminate()
            try:
                process.wait(timeout=10)
            except subprocess.TimeoutExpired:
                process.kill()
                process.wait(timeout=10)
        # Drain already-written bytes before closing. Nonblocking reads and a
        # stop event also bound cleanup if a descendant inherited the pipe.
        self._stderr_stop.set()
        if self._stderr_thread is not None:
            self._stderr_thread.join(timeout=2)
            self._stderr_thread = None
        with contextlib.suppress(Exception):
            process.stderr.close()
        self.process = None

    def __enter__(self):
        return self.start()

    def __exit__(self, *_):
        self.stop()

    # -------------------------------------------------------------- decision
    def propose(self, observation):
        """Return one raw model response, or raise before the deadline passes.

        Raises DeadlineExceeded when the model does not answer in time. The
        request is abandoned and the server is restarted, because a server that
        missed one deadline is usually still working on that answer and would
        make the next decision late as well.
        """
        from python.dcm.contract import render_prompt
        prompt = render_prompt(observation,
                               ending_guidance=self.ending_guidance)
        return self._chat(prompt)

    def decide(self, messages, tools, *, concise=False):
        """Return one native assistant message; no whole-mission schema."""
        return self._chat({"messages": messages, "tools": tools, "concise": concise}, max_tokens=768)

    @contextlib.contextmanager
    def visual_inference(self):
        """Serialize accelerator ownership with ephemeral onboard vision.

        Stop only this runtime's owned server. The next decision lazily reloads
        it; failures and cancellation never leave a second model racing it.
        CPU-configured deployments remain CPU-only.
        """
        if not self._inference_lock.acquire(blocking=False):
            raise RuntimeError("previous model inference is still completing")
        try:
            layers = 0 if self.gpu_layers == 0 else 99
            if layers:
                self.stop()
            yield layers
        finally:
            self._inference_lock.release()

    def _chat(self, prompt, max_tokens=None, response_format=None):
        """One session-owned request, including deadline recovery.

        A cancelled mission can abandon waiting while inference winds down.
        A new request must not race that worker's server restart or statistics.
        """
        if not self._inference_lock.acquire(blocking=False):
            raise RuntimeError("previous model inference is still completing")
        try:
            return self._chat_owned(prompt, max_tokens, response_format)
        finally:
            self._inference_lock.release()

    def _chat_owned(self, prompt, max_tokens=None, response_format=None):
        """Send one system/user pair with the runtime's hard wall deadline."""
        if self.process is None:
            self.start()
        request_body = {
            "messages": [
                {"role": "system", "content": prompt.get("system", "")},
                {"role": "user", "content": prompt.get("user", "")},
            ],
            "temperature": self.descriptor.temperature,
            "stream": False,
            "cache_prompt": True,
            **self._mission_slot(),
        }
        for field, parameter in (("top_p", "top_p"), ("top_k", "top_k"),
                                 ("min_p", "min_p"), ("presence_penalty", "presence_penalty"),
                                 ("repetition_penalty", "repeat_penalty"), ("seed", "seed")):
            value = getattr(self.descriptor, field)
            if value is not None:
                request_body[parameter] = value
        if max_tokens is not None:
            request_body["max_tokens"] = max_tokens
        if response_format is not None:
            request_body["response_format"] = response_format
        native = "messages" in prompt
        if native:
            request_body.update(messages=prompt["messages"], tools=prompt["tools"],
                                # Accept complete native proposals from the
                                # parser. The agent executes only the first,
                                # then obtains fresh feedback; no flight tools
                                # run in parallel. Strict single-call parsing
                                # otherwise turns extra proposals into HTTP500.
                                tool_choice="required", parallel_tool_calls=True,
                                chat_template_kwargs={"enable_thinking": bool(self.descriptor.reasoning_budget) and not prompt.get("concise")})
        timeout_s = self.descriptor.deadline_ms / 1000
        outcome = {}
        abandoned = threading.Event()

        def call():
            # urllib's own timeout is per socket operation, not total elapsed
            # time, so a server that dribbles data can run past the deadline
            # without ever raising. It is kept only as a backstop; the real
            # deadline is the join() below.
            try:
                if self.descriptor.reasoning_budget is not None and max_tokens is not None:
                    # Count with the inference chat template and schema, without
                    # model inference, inside the same hard wall deadline.
                    request_body["reasoning_budget_tokens"] = self.descriptor.reasoning_budget
                    count_request = urllib.request.Request(
                        self.endpoint + "/v1/chat/completions/input_tokens",
                        data=json.dumps(request_body).encode("utf-8"),
                        headers={"Content-Type": "application/json"})
                    with urllib.request.urlopen(count_request, timeout=timeout_s) as response:
                        count = json.loads(response.read().decode("utf-8")).get("input_tokens")
                    if type(count) is not int or count < 0:
                        raise RuntimeError("llama-server returned an invalid prompt token count")
                    # Keep the full JSON allowance. Only optional reasoning may
                    # shrink; never truncate context or rely on context shifting.
                    camera = self._vision_slot_tokens()
                    available = self.descriptor.context_length - camera - count - max_tokens - 32
                    if available < 0:
                        raise ContextBudgetExceeded(
                            f"mission context needs {count} prompt tokens plus {max_tokens} "
                            f"reply tokens and 32 boundary tokens, exceeding configured "
                            f"context={self.descriptor.context_length}"
                            + (f" less {camera} held for camera questions" if camera else "")
                            + "; no prompt was truncated")
                    effective = 0 if prompt.get("concise") else min(self.descriptor.reasoning_budget, available)
                    request_body["reasoning_budget_tokens"] = effective
                    request_body["max_tokens"] = max_tokens + effective
                    outcome["allocation"] = {
                        "tokens_prompt_counted": count,
                        "reasoning_budget_requested": self.descriptor.reasoning_budget,
                        "reasoning_budget_effective": effective,
                        "reply_tokens_reserved": max_tokens,
                        "max_tokens_requested": max_tokens + effective,
                    }
                if abandoned.is_set():
                    return
                request = urllib.request.Request(
                    self.endpoint + "/v1/chat/completions", data=json.dumps(request_body).encode("utf-8"),
                    headers={"Content-Type": "application/json"})
                with urllib.request.urlopen(request, timeout=timeout_s) as response:
                    outcome["payload"] = json.loads(response.read().decode("utf-8"))
            except urllib.error.HTTPError as failure:
                # Read a bounded error while still inside the same deadline.
                # HTTPError subclasses URLError but is a completed response,
                # not evidence that inference timed out.
                try:
                    detail = failure.read(4096).decode("utf-8", errors="replace")
                    try:
                        error_body = json.loads(detail)
                        error = error_body.get("error", error_body) if isinstance(error_body, dict) else detail
                        detail = error.get("message", detail) if isinstance(error, dict) else str(error)
                    except (ValueError, TypeError):
                        pass
                    detail = " ".join(str(detail).split())[:512]
                    # llama.cpp reports generated tool syntax it cannot parse
                    # as HTTP 500. It is still unexecuted model output, so use
                    # the same bounded repair as a truncated response. Other
                    # server errors remain transport failures, not blind retries.
                    error_type = (
                        IncompleteModelResponse
                        if native and failure.code == 500
                        and "does not match the expected peg-native format" in detail
                        else RuntimeError
                    )
                    outcome["failure"] = error_type(
                        f"llama-server HTTP {failure.code}: {detail or failure.reason}")
                except BaseException as read_failure:  # noqa: BLE001 - same deadline owns error reads
                    outcome["failure"] = read_failure
                finally:
                    failure.close()
            except BaseException as failure:  # noqa: BLE001
                outcome["failure"] = failure

        worker = threading.Thread(target=call, daemon=True)
        started = time.monotonic()
        worker.start()
        worker.join(timeout_s)
        # Snapshot the verdict before restarting. Tearing the server down can
        # unblock the worker, which would then record a late answer or a
        # decode error; whatever it writes afterwards must not change a
        # decision the deadline has already made.
        expired = worker.is_alive()
        elapsed_ms = (time.monotonic() - started) * 1000
        payload = outcome.get("payload")
        failure = outcome.get("failure")
        timeout_failure = failure.reason if isinstance(failure, urllib.error.URLError) else failure
        if not expired and payload is None and isinstance(timeout_failure, (TimeoutError, socket.timeout)):
            expired = True
        if expired:
            abandoned.set()
            # Abandon the answer and take the server down with it. A worker
            # still blocked on a dead socket cannot outlive the process it was
            # talking to, and a server that missed one deadline is usually
            # still computing that answer and would make the next decision
            # late too.
            self.stop()
            raise DeadlineExceeded(
                f"no response within {self.descriptor.deadline_ms} ms "
                f"(waited {elapsed_ms:.0f} ms)")
        if isinstance(failure, urllib.error.URLError):
            detail = " ".join(str(failure.reason).split())[:512]
            raise RuntimeError(f"llama-server connection failed: {detail}") from failure  # noqa: TRY004 - transport error, not an argument type error
        if failure is not None:
            raise failure
        if payload is None:
            raise RuntimeError("llama-server returned no payload")
        # llama-server already reports generation speed and token counts on
        # every non-streaming response; record rather than recompute them.
        self.last_stats = {
            "tokens_predicted": (payload.get("usage") or {}).get(
                "completion_tokens"),
            "tokens_prompt": (payload.get("usage") or {}).get("prompt_tokens"),
            "predicted_per_second": (payload.get("timings") or {}).get(
                "predicted_per_second"),
            "prompt_per_second": (payload.get("timings") or {}).get(
                "prompt_per_second"),
            # Tokens actually prefilled vs served from the prompt cache: the
            # number that says whether the cache held for this decision.
            "tokens_prefilled": (payload.get("timings") or {}).get("prompt_n"),
            "tokens_cached": (payload.get("timings") or {}).get("cache_n"),
        }
        self.last_stats.update(outcome.get("allocation", {}))
        choice = payload["choices"][0]
        self.last_response = {key: value for key, value in choice.get("message", {}).items()
                              if key in ("content", "tool_calls")}
        if choice.get("finish_reason") == "length":
            raise IncompleteModelResponse(
                "llama-server output was truncated at the token limit; "
                f"context={self.descriptor.context_length}, requested reply={request_body.get('max_tokens')}, "
                f"prompt tokens={self.last_stats['tokens_prompt']}, "
                f"generated tokens={self.last_stats['tokens_predicted']}. "
                "No complete tool response was returned.")
        return choice["message"] if native else choice["message"]["content"]

    def restart(self):
        self.stop()
        self.start()

    def as_record(self):
        return {"runtime": self.name, "endpoint": self.endpoint,
                **self.descriptor.as_record()}

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

Output is deliberately *not* grammar-constrained. llama.cpp can force
schema-valid JSON, which would drive the invalid-action rate to zero by
construction and hide exactly the differences Phase 12 exists to measure.
Constraining output is a legitimate deployment choice, but it must be made
knowingly and recorded, not switched on by default in an evaluation harness.
"""

import contextlib
import json
import socket
import subprocess
import threading
import time
import urllib.error
import urllib.request
from pathlib import Path

from python.dcm.contract import DeadlineExceeded, RuntimeDescriptor, render_prompt

DEFAULT_BINARY = (Path(__file__).resolve().parents[2]
                  / "third_party/llama.cpp/build/bin/llama-server")
STARTUP_TIMEOUT_S = 180


def _free_port():
    with socket.socket() as probe:
        probe.bind(("127.0.0.1", 0))
        return probe.getsockname()[1]


class LlamaCppRuntime:
    """Run a pinned local GGUF behind the provider-neutral contract."""

    def __init__(self, descriptor, binary=None, host="127.0.0.1", port=None,
                 gpu_layers=99, verify_artifact=True, extra_args=(),
                 ending_guidance=False):
        if not isinstance(descriptor, RuntimeDescriptor):
            raise TypeError("descriptor must be a RuntimeDescriptor")
        self.descriptor = descriptor
        self.binary = Path(binary or DEFAULT_BINARY)
        self.host = host
        self.port = port or _free_port()
        self.gpu_layers = gpu_layers
        self.verify_artifact = verify_artifact
        self.extra_args = list(extra_args)
        # The runtime renders the prompt, so it owns which variant is used.
        self.ending_guidance = ending_guidance
        self.process = None
        # The model identity belongs in the name so a report cannot be
        # mistaken for one produced by a different artifact.
        self.name = f"llama_cpp:{descriptor.model_id}"

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
        if self.verify_artifact:
            # Fail before loading rather than after a whole evaluation run.
            self.descriptor.verify_artifact()
        command = [
            str(self.binary),
            "-m", self.descriptor.artifact_path,
            "--host", self.host,
            "--port", str(self.port),
            "-c", str(self.descriptor.context_length),
            "-ngl", str(self.gpu_layers),
            *self.extra_args,
        ]
        self.process = subprocess.Popen(
            command, stdout=subprocess.DEVNULL, stderr=subprocess.PIPE)
        self._await_ready()
        self._warm()
        return self

    def _warm(self):
        """Prefill the fixed system prompt before the decision clock starts.

        /health reports ready once weights are loaded, but the first request
        still pays to prefill the shared prompt prefix that `cache_prompt`
        serves to every later decision. Warming with the real system prompt is
        legitimate rather than flattering: it is fixed and known before any
        episode runs, so what remains is the steady-state cost a deployed loop
        would actually see.

        This reduces the first decision's cost but does not remove it; see
        docs/architecture/DCM-OBSERVE-V1.md for the measured spread and why the
        deadline must be set with it in mind.
        """
        prompt = render_prompt({"warmup": True},
                               ending_guidance=self.ending_guidance)
        body = json.dumps({
            "messages": [
                {"role": "system", "content": prompt["system"]},
                {"role": "user", "content": prompt["user"]},
            ],
            "temperature": 0.0, "stream": False, "max_tokens": 1,
            "cache_prompt": True,
        }).encode("utf-8")
        request = urllib.request.Request(
            self.endpoint + "/v1/chat/completions", data=body,
            headers={"Content-Type": "application/json"})
        # A failed warmup is not fatal: the first decision simply pays the
        # cost it would have paid anyway.
        with contextlib.suppress(Exception), \
                urllib.request.urlopen(request, timeout=STARTUP_TIMEOUT_S) as r:
            r.read()

    def _await_ready(self):
        deadline = time.monotonic() + STARTUP_TIMEOUT_S
        while time.monotonic() < deadline:
            if self.process.poll() is not None:
                stderr = self.process.stderr.read().decode("utf-8", "replace")
                self.process = None
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
            f"llama-server not ready within {STARTUP_TIMEOUT_S}s")

    def stop(self):
        if self.process is None:
            return
        self.process.terminate()
        try:
            self.process.wait(timeout=10)
        except subprocess.TimeoutExpired:
            self.process.kill()
            self.process.wait(timeout=10)
        with contextlib.suppress(Exception):
            self.process.stderr.close()
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
        if self.process is None:
            self.start()
        prompt = render_prompt(observation,
                               ending_guidance=self.ending_guidance)
        body = json.dumps({
            "messages": [
                {"role": "system", "content": prompt["system"]},
                {"role": "user", "content": prompt["user"]},
            ],
            "temperature": self.descriptor.temperature,
            "stream": False,
            "cache_prompt": True,
        }).encode("utf-8")
        timeout_s = self.descriptor.deadline_ms / 1000
        outcome = {}

        def call():
            # urllib's own timeout is per socket operation, not total elapsed
            # time, so a server that dribbles data can run past the deadline
            # without ever raising. It is kept only as a backstop; the real
            # deadline is the join() below.
            try:
                request = urllib.request.Request(
                    self.endpoint + "/v1/chat/completions", data=body,
                    headers={"Content-Type": "application/json"})
                with urllib.request.urlopen(request, timeout=timeout_s) as response:
                    outcome["payload"] = json.loads(response.read().decode("utf-8"))
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
        if not expired and payload is None and isinstance(
                failure, (TimeoutError, socket.timeout, urllib.error.URLError)):
            expired = True
        if expired:
            # Abandon the answer and take the server down with it. A worker
            # still blocked on a dead socket cannot outlive the process it was
            # talking to, and a server that missed one deadline is usually
            # still computing that answer and would make the next decision
            # late too.
            self.restart()
            raise DeadlineExceeded(
                f"no response within {self.descriptor.deadline_ms} ms "
                f"(waited {elapsed_ms:.0f} ms)")
        if failure is not None:
            raise failure
        if payload is None:
            raise RuntimeError("llama-server returned no payload")
        return payload["choices"][0]["message"]["content"]

    def restart(self):
        self.stop()
        self.start()

    def as_record(self):
        return {"runtime": self.name, "endpoint": self.endpoint,
                **self.descriptor.as_record()}

"""Adapter tests: the deadline must be wall-clock and must abandon the request.

These run against a stub HTTP server, not the real model, so they stay fast and
work on a machine with no GGUF present. The behaviour under test is the
adapter's, not the model's.
"""

import contextlib
import hashlib
import json
import tempfile
import threading
import time
import unittest
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path

from python.dcm.contract import DeadlineExceeded, RuntimeDescriptor
from python.dcm.llama_runtime import LlamaCppRuntime


class StubHandler(BaseHTTPRequestHandler):
    """Answers /health immediately and /v1/chat/completions after a delay."""

    delay_s = 0.0
    reply = '{"action":"none","arguments":{}}'
    trickle = False

    def log_message(self, *_):
        pass

    def do_GET(self):
        self.send_response(200)
        self.end_headers()
        self.wfile.write(b'{"status":"ok"}')

    def do_POST(self):
        self.rfile.read(int(self.headers.get("Content-Length", 0)))
        if self.trickle:
            # Send headers, then dribble bytes forever. Each individual socket
            # read succeeds, so a per-operation timeout never fires; only a
            # wall-clock deadline catches this.
            self.send_response(200)
            self.send_header("Content-Type", "application/json")
            self.end_headers()
            for _ in range(200):
                try:
                    self.wfile.write(b" ")
                    self.wfile.flush()
                except OSError:
                    return
                time.sleep(0.05)
            return
        time.sleep(self.delay_s)
        body = json.dumps({
            "choices": [{"message": {"content": self.reply}}]
        }).encode()
        # The client abandons slow requests by design, so writing to a closed
        # socket is the expected outcome here, not a failure.
        with contextlib.suppress(OSError):
            self.send_response(200)
            self.send_header("Content-Type", "application/json")
            self.send_header("Content-Length", str(len(body)))
            self.end_headers()
            self.wfile.write(body)


class StubRuntime(LlamaCppRuntime):
    """A runtime whose 'server' is an in-process stub, so no GGUF is needed."""

    def __init__(self, descriptor, handler):
        super().__init__(descriptor, binary=__file__, verify_artifact=False)
        self.handler = handler
        self.server = None
        self.restarts = 0

    def start(self):
        if self.server is not None:
            return self
        # Threaded, so shutting down does not block on an in-flight slow
        # handler; that would hide how promptly the deadline returns control.
        self.server = ThreadingHTTPServer(("127.0.0.1", self.port), self.handler)
        self.server.daemon_threads = True
        self.thread = threading.Thread(target=self.server.serve_forever,
                                       daemon=True)
        self.thread.start()
        self.process = self.server
        return self

    def stop(self):
        if self.server is None:
            return
        self.server.shutdown()
        self.server.server_close()
        self.server = None
        self.process = None

    def restart(self):
        self.restarts += 1
        super().restart()


def descriptor(deadline_ms=500):
    return RuntimeDescriptor(
        model_id="stub", family="qwen", quantization="q5_k_m",
        artifact_path="/nonexistent.gguf", artifact_sha256="0" * 64,
        runtime="llama_cpp", runtime_revision="deadbeef",
        context_length=8192, temperature=0.0, deadline_ms=deadline_ms)


class DeadlineTests(unittest.TestCase):
    def test_prompt_response_is_returned_when_in_time(self):
        class Fast(StubHandler):
            delay_s = 0.0
        with StubRuntime(descriptor(2000), Fast) as runtime:
            raw = runtime.propose({"state": {}})
            self.assertEqual(json.loads(raw)["action"], "none")

    def test_slow_response_raises_deadline_exceeded(self):
        class Slow(StubHandler):
            delay_s = 3.0
        with StubRuntime(descriptor(400), Slow) as runtime:
            started = time.monotonic()
            with self.assertRaises(DeadlineExceeded):
                runtime.propose({"state": {}})
            elapsed = time.monotonic() - started
            # The point of the deadline is that it returns control promptly,
            # not that it eventually notices the answer was late.
            self.assertLess(elapsed, 2.5)

    def test_trickling_server_still_hits_the_wall_clock_deadline(self):
        # A per-socket-operation timeout does not fire here: every read
        # succeeds. Only a wall-clock deadline catches it. This is the exact
        # case that made a real 7107 ms decision pass a 5000 ms socket timeout.
        class Trickle(StubHandler):
            trickle = True
        with StubRuntime(descriptor(400), Trickle) as runtime:
            started = time.monotonic()
            with self.assertRaises(DeadlineExceeded):
                runtime.propose({"state": {}})
            self.assertLess(time.monotonic() - started, 2.5)

    def test_missed_deadline_restarts_the_server(self):
        class Slow(StubHandler):
            delay_s = 3.0
        runtime = StubRuntime(descriptor(300), Slow)
        runtime.start()
        try:
            with self.assertRaises(DeadlineExceeded):
                runtime.propose({"state": {}})
            self.assertEqual(runtime.restarts, 1)
        finally:
            runtime.stop()

    def test_deadline_message_reports_the_configured_limit(self):
        class Slow(StubHandler):
            delay_s = 2.0
        with StubRuntime(descriptor(250), Slow) as runtime:
            with self.assertRaises(DeadlineExceeded) as caught:
                runtime.propose({"state": {}})
            self.assertIn("250 ms", str(caught.exception))


class ConfigurationTests(unittest.TestCase):
    def test_descriptor_type_is_enforced(self):
        with self.assertRaises(TypeError):
            LlamaCppRuntime({"model_id": "not-a-descriptor"})

    def test_runtime_name_carries_the_model_identity(self):
        runtime = LlamaCppRuntime(descriptor(), binary=__file__,
                                  verify_artifact=False)
        self.assertEqual(runtime.name, "llama_cpp:stub")

    def test_missing_binary_is_reported_clearly(self):
        runtime = LlamaCppRuntime(descriptor(), binary="/nonexistent/llama-server",
                                  verify_artifact=False)
        with self.assertRaises(FileNotFoundError) as caught:
            runtime.start()
        self.assertIn("setup-model-runtime", str(caught.exception))

    def test_checksum_is_verified_before_the_model_is_loaded(self):
        with tempfile.TemporaryDirectory() as root:
            weights = Path(root) / "model.gguf"
            weights.write_bytes(b"weights")
            wrong = RuntimeDescriptor(
                model_id="stub", family="qwen", quantization="q5_k_m",
                artifact_path=str(weights), artifact_sha256="0" * 64,
                runtime="llama_cpp", runtime_revision="deadbeef",
                context_length=8192, temperature=0.0, deadline_ms=500)
            runtime = LlamaCppRuntime(wrong, binary=__file__)
            with self.assertRaises(ValueError) as caught:
                runtime.start()
            self.assertIn("checksum mismatch", str(caught.exception))
            right = RuntimeDescriptor(
                **{**wrong.as_record(),
                   "artifact_sha256": hashlib.sha256(b"weights").hexdigest()})
            # Verification passes; startup then fails for an unrelated reason
            # (the stub binary is this test file), which is the point.
            self.assertEqual(right.verify_artifact(),
                             hashlib.sha256(b"weights").hexdigest())


if __name__ == "__main__":
    unittest.main()

"""The state/perception sampler must outlive one slow Drone API reply."""
import json
import tempfile
import time
from pathlib import Path
from types import SimpleNamespace

import grpc

from python.dataset_tools.episode import Episode


class _Timeout(grpc.RpcError):
    def code(self):
        return grpc.StatusCode.DEADLINE_EXCEEDED


class _Message:
    def __init__(self, n):
        self.n = n


def _fake_safe_message(message):
    return {"n": getattr(message, "n", None)}


def test_one_deadline_exceeded_does_not_end_state_sampling(monkeypatch, tmp_path):
    import python.dataset_tools.episode as module
    monkeypatch.setattr(module, "_safe_message", _fake_safe_message)
    pb2 = SimpleNamespace(GetStateRequest=lambda **kw: "state-req",
                          GetPerceptionRequest=lambda **kw: "perc-req",
                          WatchEventsRequest=lambda **kw: "events-req")

    calls = {"n": 0}

    def get_state(request, timeout):
        calls["n"] += 1
        if calls["n"] == 3:  # one slow reply mid-mission
            raise _Timeout()
        return _Message(calls["n"])

    def watch_events(request, timeout):
        time.sleep(0.05)
        raise _Timeout()

    class Stub:
        def __init__(self, channel):
            pass

        def GetPerception(self, request, timeout):
            return _Message(0)

    import sys
    grpc_module = SimpleNamespace(PerceptionServiceStub=Stub)
    v1 = SimpleNamespace(drone_api_pb2=pb2, drone_api_pb2_grpc=grpc_module)
    monkeypatch.setitem(sys.modules, "icarus", SimpleNamespace(v1=v1))
    monkeypatch.setitem(sys.modules, "icarus.v1", v1)
    monkeypatch.setitem(sys.modules, "icarus.v1.drone_api_pb2", pb2)
    monkeypatch.setitem(sys.modules, "icarus.v1.drone_api_pb2_grpc", grpc_module)
    client = SimpleNamespace(channel=None, session_id="s",
                             state_api=SimpleNamespace(GetState=get_state, WatchEvents=watch_events))
    episode = Episode(tmp_path, "test", "127.0.0.1:50051", None)
    episode.start_sampling(client, maximum_rate_hz=50)
    deadline = time.time() + 3
    while calls["n"] < 8 and time.time() < deadline:
        time.sleep(0.02)
    episode.seal()
    assert calls["n"] >= 8, "sampling stopped after the first timeout"
    manifest = json.loads((episode.directory / "manifest.json").read_text())
    assert "state:StatusCode.DEADLINE_EXCEEDED" in manifest["stream_errors"]
    states = [json.loads(l) for l in (episode.directory / "events.jsonl").open() if '"state"' in l]
    assert len(states) >= 7

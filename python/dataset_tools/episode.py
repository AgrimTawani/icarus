"""Compact, append-only flight episode capture at the Drone API boundary.

This records the control contract, excludes operator credentials, and snapshots
the bounded simulator sensor recording when one exists. Camera pixels remain
excluded; the simulator's RGB/depth streams contain metadata only. Every record
has a common monotonic sequence and wall-clock timestamp; a final manifest seals
the stream hash so incomplete/tampered episodes fail replay.
"""

import hashlib
import json
import os
import shutil
import subprocess
import threading
import time
import uuid
from pathlib import Path

import grpc
from google.protobuf.json_format import MessageToDict

ACTION_NAMES = {
    "Arm", "Disarm", "Takeoff", "Goto", "ExecuteRoute", "Hold",
    "ReturnHome", "Land", "Orbit", "CancelAction",
}
PRIVATE_FIELDS = {"session_id", "control_lease_id", "client_id", "lease_id"}


def _safe_message(message):
    value = MessageToDict(message, preserving_proto_field_name=True)

    def clean(item):
        if isinstance(item, dict):
            return {key: clean(val) for key, val in item.items()
                    if key not in PRIVATE_FIELDS}
        if isinstance(item, list):
            return [clean(val) for val in item]
        return item

    return clean(value)


def _replay_message(message):
    """Retain command semantics while replacing volatile session identifiers."""
    value = MessageToDict(message, preserving_proto_field_name=True)

    def clean(item):
        if isinstance(item, dict):
            return {key: ("redacted" if key in PRIVATE_FIELDS else clean(val))
                    for key, val in item.items()}
        if isinstance(item, list):
            return [clean(val) for val in item]
        return item

    return clean(value)


def _sha256(path):
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def _git_revision(root):
    result = subprocess.run(["git", "rev-parse", "HEAD"], cwd=root,
                            capture_output=True, text=True, check=False)
    return result.stdout.strip() if result.returncode == 0 else "unavailable"


def _source_fingerprint(root):
    """Fingerprint the actual source bytes, including uncommitted changes."""
    digest = hashlib.sha256()
    roots = [root / name for name in ("cpp", "perception", "proto",
                                     "python/perception", "python/dataset_tools", "python/dcm",
                                     "scripts/autonomy", "scripts/simulation")]
    files = [root / "CMakeLists.txt", root / "scripts/start-autonomy"]
    for source_root in roots:
        files.extend(source_root.rglob("*.cpp"))
        files.extend(source_root.rglob("*.hpp"))
        files.extend(source_root.rglob("*.py"))
        files.extend(source_root.rglob("*.proto"))
    for path in sorted(set(files)):
        if path.is_file():
            digest.update(str(path.relative_to(root)).encode())
            digest.update(b"\0")
            digest.update(bytes.fromhex(_sha256(path)))
    return digest.hexdigest()


def _snapshot_simulator_sensors(session, destination):
    """Copy the bounded sensor stream into an episode, if this is a sim run.

    The recorder owns the live source files and may still append while a
    mission client closes. A copy is therefore a valid point-in-time snapshot,
    not a move or a hard link to a mutable simulator directory. Image streams
    have already had pixel payloads removed by ``record_compact_sensors.py``.
    """
    if not session or not session.get("run_directory"):
        return None
    source = Path(session["run_directory"]) / "sensors"
    if not (source / "schema.json").is_file() or not (source / "index.jsonl").is_file():
        return None
    target = destination / "raw_sensors"
    shutil.copytree(source, target)
    hashes = {
        str(path.relative_to(destination)): _sha256(path)
        for path in sorted(target.rglob("*")) if path.is_file()
    }
    schema = json.loads((target / "schema.json").read_text())
    return {
        "format": schema.get("format"),
        "image_pixels_saved": schema.get("image_pixels_saved"),
        "channels": sorted((schema.get("channels") or {}).keys()),
        "files": hashes,
    }


def _snapshot_edge_vision(session, destination):
    """Seal structured edge-observer evidence without copying camera pixels."""
    if not session or not session.get("run_directory"):
        return None
    source = Path(session["run_directory"]) / "edge_vision.jsonl"
    if not source.is_file():
        return None
    target = destination / "edge_vision.jsonl"
    shutil.copyfile(source, target)
    target.chmod(0o444)
    return {"path": str(target.relative_to(destination)), "sha256": _sha256(target),
            "raw_pixels_saved": False}


class ActionRecorder:
    """Transparent action stub wrapper; captures requests and actual replies."""

    def __init__(self, stub, episode, client):
        self._stub = stub
        self._episode = episode
        self._client = client

    def __getattr__(self, name):
        method = getattr(self._stub, name)
        if name == "WatchActionStatus":
            def watch(request, **kwargs):
                for status in method(request, **kwargs):
                    self._episode.record("action_status", _safe_message(status))
                    yield status
            return watch
        if name not in ACTION_NAMES:
            return method

        def call(request, **kwargs):
            if name != "CancelAction":
                from icarus.v1 import action_pb2, drone_api_pb2
                field = "goto" if name == "Goto" else (
                    "execute_route" if name == "ExecuteRoute" else
                    "return_home" if name == "ReturnHome" else name.lower())
                command = action_pb2.ActionCommand()
                getattr(command, field).CopyFrom(request)
                snapshot = self._client.state()
                evaluated_at = int(time.time() * 1000)
                result = self._stub.ValidateAction(
                    drone_api_pb2.ValidateActionRequest(
                        session_id=self._client.session_id, command=command),
                    timeout=5)
                self._episode.record("guardrail_validation", {
                    "command": _replay_message(command),
                    "state": _replay_message(snapshot),
                    "authorized": bool(self._client.lease_id),
                    "evaluated_at_unix_ms": evaluated_at,
                    "result": _safe_message(result),
                })
            self._episode.record("action_request", {
                "method": name, "request": _safe_message(request),
            })
            try:
                receipt = method(request, **kwargs)
            except grpc.RpcError as error:
                self._episode.record("action_rpc_error", {
                    "method": name, "code": str(error.code()),
                    "details": error.details(),
                })
                raise
            self._episode.record("action_receipt", {
                "method": name, "receipt": _safe_message(receipt),
            })
            return receipt
        return call


class Episode:
    def __init__(self, root, mission, endpoint, session):
        self.root = Path(root)
        self.id = f"{time.strftime('%Y%m%dT%H%M%S')}_{uuid.uuid4().hex[:12]}"
        self.directory = self.root / "logs" / "episodes" / self.id
        self.directory.mkdir(parents=True, exist_ok=False)
        self.stream_path = self.directory / "events.jsonl"
        self.stream = self.stream_path.open("x", encoding="utf-8")
        self.lock = threading.Lock()
        self.stop = threading.Event()
        self.threads = []
        self.seq = 0
        self.counts = {}
        self.started_ms = int(time.time() * 1000)
        self.mission = mission
        self.endpoint = endpoint
        self.session = session
        self.failed_streams = []
        self.model = None
        self.record("episode_start", {"mission": mission})

    def set_model(self, descriptor):
        """Attach the immutable runtime descriptor before the episode seals."""
        if not isinstance(descriptor, dict):
            raise TypeError("model descriptor must be a dictionary")
        with self.lock:
            self.model = json.loads(json.dumps(descriptor, sort_keys=True))

    def record(self, kind, payload):
        with self.lock:
            if self.stream.closed:
                return
            self.seq += 1
            self.counts[kind] = self.counts.get(kind, 0) + 1
            item = {"seq": self.seq, "unix_ms": int(time.time() * 1000),
                    "monotonic_ns": time.monotonic_ns(), "kind": kind,
                    "payload": payload}
            self.stream.write(json.dumps(item, sort_keys=True,
                                         separators=(",", ":")) + "\n")

    def start_sampling(self, client, maximum_rate_hz=10):
        from icarus.v1 import drone_api_pb2, drone_api_pb2_grpc

        perception_api = drone_api_pb2_grpc.PerceptionServiceStub(client.channel)
        state_request = drone_api_pb2.GetStateRequest(
            session_id=client.session_id, vehicle_id="icarus-01")
        perception_request = drone_api_pb2.GetPerceptionRequest(
            session_id=client.session_id, vehicle_id="icarus-01")
        self.record("state", _safe_message(
            client.state_api.GetState(state_request, timeout=2)))
        self.record("perception", _safe_message(
            perception_api.GetPerception(perception_request, timeout=2)))

        def sample():
            interval = 1 / maximum_rate_hz
            while not self.stop.wait(interval):
                for kind, call, request in (
                    ("state", client.state_api.GetState, state_request),
                    ("perception", perception_api.GetPerception, perception_request),
                ):
                    try:
                        self.record(kind, _safe_message(call(request, timeout=2)))
                    except grpc.RpcError as error:
                        self.failed_streams.append(kind + ":" + str(error.code()))
                        return

        def events():
            request = drone_api_pb2.WatchEventsRequest(
                session_id=client.session_id, vehicle_id="icarus-01")
            seen = set()
            while not self.stop.is_set():
                try:
                    for event in client.state_api.WatchEvents(request, timeout=2):
                        if event.event_id not in seen:
                            self.record("vehicle_event", _safe_message(event))
                            seen.add(event.event_id)
                        if self.stop.is_set():
                            return
                except grpc.RpcError as error:
                    if error.code() != grpc.StatusCode.DEADLINE_EXCEEDED:
                        self.failed_streams.append("events:" + str(error.code()))
                        return

        for target in (sample, events):
            thread = threading.Thread(target=target, daemon=True)
            thread.start()
            self.threads.append(thread)

    def seal(self, outcome="completed", score=None):
        self.stop.set()
        for thread in self.threads:
            thread.join(timeout=3)
        self.record("episode_end", {"outcome": outcome, "score": score})
        with self.lock:
            self.stream.flush()
            os.fsync(self.stream.fileno())
            self.stream.close()
        session = self.session
        config_paths = [self.root / "config/safety/v1.yaml"]
        if session:
            scenario = session.get("scenario")
            if scenario:
                config_paths.append(self.root / "simulation/scenarios" /
                                    (scenario + ".json"))
            if session.get("profile") == "edge-vision":
                config_paths.append(Path.home() / "models/edge/MANIFEST.json")
        snapshots = self.directory / "config"
        snapshots.mkdir()
        config_hashes = {}
        for source in config_paths:
            if source.is_file():
                target = snapshots / ("edge-model-manifest.json" if source.name == "MANIFEST.json"
                                      and source.parent.name == "edge" else source.name)
                shutil.copyfile(source, target)
                target.chmod(0o444)
                config_hashes[str(target.relative_to(self.directory))] = _sha256(target)
        raw_sensor_snapshot = _snapshot_simulator_sensors(
            session, self.directory)
        edge_vision_snapshot = _snapshot_edge_vision(session, self.directory)
        manifest = {
            "schema": "icarus.episode.v1", "episode_id": self.id,
            "mission": self.mission, "source": "simulation" if session else "physical",
            "started_unix_ms": self.started_ms,
            "finished_unix_ms": int(time.time() * 1000),
            "outcome": outcome, "score": score,
            "code_revision": _git_revision(self.root),
            "source_tree_sha256": _source_fingerprint(self.root),
            "api_endpoint": self.endpoint,
            "scenario": session.get("scenario") if session else None,
            "seed": session.get("seed") if session else None,
            "vehicle_id": "icarus-01",
            "raw_sensor_payloads": raw_sensor_snapshot is not None,
            "raw_sensor_snapshot": raw_sensor_snapshot,
            "edge_vision_snapshot": edge_vision_snapshot,
            "model": self.model,
            "privacy": {"operator_identifiers": "excluded",
                        "training_status": "unreviewed_do_not_train"},
            "config_sha256": config_hashes,
            "streams": {"events.jsonl": {
                "sha256": _sha256(self.stream_path), "records": self.seq,
                "counts": self.counts,
            }},
            "stream_errors": list(self.failed_streams),
        }
        path = self.directory / "manifest.json"
        with path.open("x", encoding="utf-8") as output:
            json.dump(manifest, output, sort_keys=True, indent=2)
            output.write("\n")
            output.flush()
            os.fsync(output.fileno())
        self.stream_path.chmod(0o444)
        path.chmod(0o444)
        print("Episode:", self.directory, flush=True)
        return self.directory

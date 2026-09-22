#!/usr/bin/env python3
"""Deterministic Phase 8 reference client; this file never imports MAVLink."""

import argparse
import json
import re
import subprocess
import tempfile
import sys
import threading
import time
import uuid
from pathlib import Path

import grpc
from google.protobuf.json_format import MessageToDict

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT))
sys.path.insert(0, str(ROOT / "build/generated/python"))

from icarus.v1 import (
    action_pb2,
    drone_api_pb2,
    drone_api_pb2_grpc,
    state_pb2,
)

from python.dataset_tools.episode import ActionRecorder, Episode

TERMINAL = {
    action_pb2.ACTION_STATE_REJECTED,
    action_pb2.ACTION_STATE_SUCCEEDED,
    action_pb2.ACTION_STATE_CANCELLED,
    action_pb2.ACTION_STATE_TIMED_OUT,
    action_pb2.ACTION_STATE_FAILED,
    action_pb2.ACTION_STATE_PREEMPTED,
    action_pb2.ACTION_STATE_ABORTED_BY_SAFETY,
}


def _asks_count_of(text, noun):
    """Match an explicit count request, not a nearby unrelated noun."""
    return bool(re.search(r"\bcount(?:\s+(?:the|a|an))?(?:\s+(?:number|amount))?(?:\s+of)?(?:\s+unique)?\s+"
                          + re.escape(noun) + r"\b", text.lower()))


class MissionClient:
    def __init__(self, endpoint: str, mission_name: str = "drone_api_mission"):
        self.endpoint = endpoint
        self.channel = grpc.insecure_channel(endpoint)
        grpc.channel_ready_future(self.channel).result(timeout=15)
        self.session_api = drone_api_pb2_grpc.SessionServiceStub(self.channel)
        self.authority_api = drone_api_pb2_grpc.AuthorityServiceStub(self.channel)
        self.state_api = drone_api_pb2_grpc.StateServiceStub(self.channel)
        self.action_api = drone_api_pb2_grpc.ActionServiceStub(self.channel)
        self.client_id = "phase8-reference-client"
        self.session_id = ""
        self.lease_id = ""
        self.stop_renewal = threading.Event()
        self.renewal_thread = None
        self.trace_id = uuid.uuid4().hex
        self.mission_name = mission_name
        self.episode = None
        self.episode_outcome = "completed"
        self.episode_score = None
        self.edge_tracker = None
        self.edge_detection_limiter = None
        self.edge_vlm_limiter = None
        self.simulation_session = None

    def connect(self):
        response = self.session_api.Connect(
            drone_api_pb2.ConnectRequest(
                client_id=self.client_id,
                client_name="Icarus deterministic mission baseline",
                requested_api_major="1",
            ),
            timeout=5,
        )
        self.session_id = response.session_id
        capabilities = self.session_api.GetCapabilities(
            drone_api_pb2.GetCapabilitiesRequest(
                session_id=self.session_id, vehicle_id="icarus-01"
            ),
            timeout=5,
        )
        required = {"arm", "takeoff", "hold", "land", "state_stream"}
        available = {item.name for item in capabilities.capabilities if item.supported}
        missing = required - available
        if missing:
            raise RuntimeError("Drone API lacks required capabilities: " + str(missing))
        session_path = ROOT / "logs/simulation/active_session.json"
        session = None
        if session_path.is_file():
            candidate = json.loads(session_path.read_text())
            try:
                import os
                os.kill(int(candidate["launcher_pid"]), 0)
                session = candidate
            except (KeyError, ValueError, ProcessLookupError):
                pass
        self.simulation_session = session
        self.episode = Episode(ROOT, self.mission_name, self.endpoint, session)
        self.action_api = ActionRecorder(self.action_api, self.episode, self)
        self.episode.start_sampling(self)

    def mission_reference(self):
        """Return committed edge landmarks, never hidden ground-truth counts."""
        if not self.simulation_session or self.simulation_session.get("profile") != "edge-vision":
            return {}
        scenario_path = ROOT / "simulation/scenarios" / (self.simulation_session["scenario"] + ".json")
        scenario = json.loads(scenario_path.read_text())
        truth = scenario.get("ground_truth", {})
        return {"known_landmarks": truth.get("known_landmarks", []),
                "count_window_s": truth.get("count_window_s"),
                "visual_requirements": {
                    "person_count_requires_detect": True,
                    "detector_classes": ["person"],
                    "known_landmarks_are_not_detector_classes": True,
                }}

    def assess_mission_completion(self, mission, executed):
        """Score requested edge evidence after landing without exposing it to the model."""
        if not self.simulation_session or self.simulation_session.get("profile") != "edge-vision":
            return None
        text = mission.lower()
        asks_people = _asks_count_of(text, "people") or _asks_count_of(text, "person")
        asks_buildings = _asks_count_of(text, "building") or _asks_count_of(text, "buildings")
        report = {"required": [], "satisfied": [], "reasons": []}
        person_results = [item.get("result") for item in executed
                          if item["action"] == "detect" and item["outcome"] == "SUCCEEDED"
                          and "person" in item.get("arguments", {}).get("classes", [])]
        if asks_people:
            report["required"].append("unique_person_count")
            if not person_results:
                report["reasons"].append("requested person count has no successful detect evidence")
            else:
                scenario_path = ROOT / "simulation/scenarios" / (self.simulation_session["scenario"] + ".json")
                expected = json.loads(scenario_path.read_text())["ground_truth"]["unique_person_count"]
                observed = person_results[-1].get("unique_person_count")
                if observed == expected:
                    report["satisfied"].append("unique_person_count")
                else:
                    report["reasons"].append(
                        f"person count is {observed!r}; expected {expected} in this deterministic scenario")
            if "orbit" in text:
                orbit_index = next((index for index, item in enumerate(executed)
                                    if item["action"] == "orbit" and item["outcome"] == "SUCCEEDED"), None)
                detect_index = next((index for index, item in enumerate(executed)
                                     if item["action"] == "detect" and item["outcome"] == "SUCCEEDED"
                                     and "person" in item.get("arguments", {}).get("classes", [])), None)
                if orbit_index is None or detect_index is None or detect_index < orbit_index:
                    report["reasons"].append("requested person count must be collected after the requested orbit")
        if asks_buildings:
            report["required"].append("visual_building_count")
            report["reasons"].append(
                "stock YOLO11n has no building class; use the committed north_building landmark instead")
        report["success"] = not report["reasons"]
        return report

    def acquire(self):
        response = self.authority_api.AcquireControl(
            drone_api_pb2.AcquireControlRequest(
                session_id=self.session_id,
                vehicle_id="icarus-01",
                requested_role=state_pb2.CONTROL_ROLE_SCRIPTED_AUTONOMY,
                requested_lease_ms=30_000,
            ),
            timeout=5,
        )
        if not response.granted:
            raise RuntimeError("Control denied: " + response.message)
        self.lease_id = response.authority.lease_id
        self.renewal_thread = threading.Thread(target=self._renew, daemon=True)
        self.renewal_thread.start()

    def _renew(self):
        while not self.stop_renewal.wait(10):
            try:
                response = self.authority_api.RenewControl(
                    drone_api_pb2.RenewControlRequest(
                        session_id=self.session_id,
                        lease_id=self.lease_id,
                        requested_lease_ms=30_000,
                    ),
                    timeout=5,
                )
                if not response.granted:
                    self.stop_renewal.set()
            except grpc.RpcError:
                self.stop_renewal.set()

    def state(self):
        return self.state_api.GetState(
            drone_api_pb2.GetStateRequest(
                session_id=self.session_id, vehicle_id="icarus-01"
            ),
            timeout=5,
        )

    def context(self, name: str):
        now = int(time.time() * 1000)
        state = self.state()
        return action_pb2.CommandContext(
            request_id=uuid.uuid4().hex,
            idempotency_key=f"{self.trace_id}:{name}",
            vehicle_id="icarus-01",
            client_id=self.client_id,
            control_lease_id=self.lease_id,
            session_id=self.session_id,
            issued_at_unix_ms=now,
            expires_at_unix_ms=now + 5_000,
            minimum_state_sequence=state.sequence,
            trace_id=self.trace_id,
        )

    def detect(self, classes):
        """Run a simulator camera inspection; never sends a flight command."""
        if self.episode is None or not self.episode.session:
            raise RuntimeError("semantic detection requires an active simulator episode")
        from python.perception.vision import detect_image, normalize_classes

        classes = normalize_classes(classes)
        if self.episode.session.get("profile") == "edge-vision" and "person" in classes:
            return self._edge_count_window(classes)
        # Pixels are inspection input, not episode data. Capture into an
        # ephemeral directory, retain only the image hash and structured
        # detector output, then remove the frame before returning.
        with tempfile.TemporaryDirectory(prefix="icarus-vision-") as temporary:
            output = Path(temporary) / "frame.ppm"
            subprocess.run(
                [str(ROOT / "scripts/capture-camera-frame"), "--output", str(output),
                 "--topic", "/icarus/sensors/rgbd_down/image"],
                cwd=ROOT, check=True, timeout=15)
            if self.episode.session.get("profile") == "edge-vision":
                from python.perception.edge_vision import RateLimiter, UniqueTrackCounter, detect_edge_image
                if self.edge_tracker is None:
                    self.edge_tracker = UniqueTrackCounter()
                    self.edge_detection_limiter = RateLimiter(interval_s=0.2)
                self.edge_detection_limiter.admit()
                result = detect_edge_image(output, Path.home() / "models/edge/MANIFEST.json",
                                           self.edge_tracker, max_fps=5)
            else:
                result = detect_image(output, classes, Path.home() / "models/vision/MANIFEST.json")
        if "image" in result:
            result["image"].pop("path", None)
            result["image"]["source"] = "simulator_ephemeral_capture"
        result["source"] = "simulator_ephemeral_capture"
        # Keep the model's next observation bounded: it needs count evidence,
        # not a potentially large box list or image payload.
        summary = ({"unique_person_count": result["unique_person_count"],
                    "classes": result["classes"], "observed_at_unix_ms": result["observed_at_unix_ms"],
                    "flight_authority": False}
                   if result.get("schema") == "icarus.edge.yolo.v1" else
                   {"counts": result["counts"], "requested_classes": result["requested_classes"],
                    "observed_at_unix_ms": result["observed_at_unix_ms"]})
        self.episode.record("semantic_detection", result)
        return summary

    def _edge_count_window(self, classes):
        """Read the capped observer for the scenario's declared count window."""
        from python.perception.edge_vision import summarize_observer_window

        scenario_path = ROOT / "simulation/scenarios" / (self.simulation_session["scenario"] + ".json")
        duration_s = float(json.loads(scenario_path.read_text())["ground_truth"]["count_window_s"])
        observer_path = Path(self.simulation_session["run_directory"]) / "edge_vision.jsonl"
        started_ms = int(time.time() * 1000)
        deadline = time.monotonic() + duration_s
        while time.monotonic() < deadline:
            time.sleep(min(.25, deadline - time.monotonic()))
        finished_ms = int(time.time() * 1000)
        if not observer_path.is_file():
            raise RuntimeError("edge observer evidence is unavailable; restart ./scripts/start-autonomy --profile edge-vision")
        records = []
        for line in observer_path.read_text().splitlines():
            item = json.loads(line)
            observed_at = item.get("observed_at_unix_ms", 0)
            if started_ms <= observed_at <= finished_ms:
                records.append(item)
        result = summarize_observer_window(records, classes)
        if not result["camera_evidence"]:
            raise RuntimeError("edge observer produced no usable frames during the count window")
        result.update({"observed_at_unix_ms": finished_ms, "count_window_s": duration_s,
                       "window_started_unix_ms": started_ms, "window_finished_unix_ms": finished_ms})
        self.episode.record("semantic_detection", result)
        return {"unique_person_count": result["unique_person_count"], "classes": list(classes),
                "observed_at_unix_ms": finished_ms, "count_window_s": duration_s,
                "count_method": result["count_method"], "camera_evidence": result["camera_evidence"],
                "flight_authority": False}

    def assess_landing_zone(self):
        """Run the non-flight depth boundary on the active simulator source.

        The active source is a calibrated belly RGB-D camera.  This remains
        evidence gathering only: even a suitable result cannot issue a land
        command or bypass landing guardrails.
        """
        if self.episode is None or not self.episode.session:
            raise RuntimeError("landing assessment requires an active simulator episode")
        import numpy as np
        from python.perception.landing_zone import assess_landing_zone, load_landing_source

        source = load_landing_source(ROOT / "config/perception/landing_zone.json")

        with tempfile.TemporaryDirectory(prefix="icarus-depth-") as temporary:
            output = Path(temporary) / "depth.npy"
            subprocess.run(
                [str(ROOT / "scripts/capture-depth-frame"), "--output", str(output),
                 "--topic", source["topic"]],
                cwd=ROOT, check=True, timeout=15)
            depth = np.load(output, allow_pickle=False)
            result = assess_landing_zone(
                depth, optical_axis_body_frd=source["optical_axis_body_frd"],
                horizontal_fov_deg=source["horizontal_fov_deg"],
                vertical_fov_deg=source["vertical_fov_deg"])
            result["depth"] = {"source": "simulator_ephemeral_capture",
                               "sensor_id": source["sensor_id"],
                               "width": int(depth.shape[1]),
                               "height": int(depth.shape[0])}
        self.episode.record("semantic_landing_assessment", result)
        return {key: result[key] for key in ("assessable", "suitable", "reason",
                                              "quality", "observed_at_unix_ms")}

    def inspect_scene(self, question):
        """Run the pinned VLM on one ephemeral simulator camera frame.

        This method intentionally owns neither a command context nor an action
        RPC. It is a visual evidence boundary only.
        """
        if self.episode is None or not self.episode.session:
            raise RuntimeError("visual inspection requires an active simulator episode")
        from python.perception.vlm import inspect_image, normalize_question

        question = normalize_question(question)
        with tempfile.TemporaryDirectory(prefix="icarus-vlm-") as temporary:
            output = Path(temporary) / "frame.ppm"
            subprocess.run(
                [str(ROOT / "scripts/capture-camera-frame"), "--output", str(output)],
                cwd=ROOT, check=True, timeout=15)
            if self.episode.session.get("profile") == "edge-vision":
                from python.perception.edge_vision import RateLimiter, inspect_smolvlm
                if self.edge_vlm_limiter is None:
                    self.edge_vlm_limiter = RateLimiter(interval_s=5.0)
                result = inspect_smolvlm(output, question, Path.home() / "models/edge/MANIFEST.json",
                                         self.edge_vlm_limiter)
            else:
                result = inspect_image(output, question, Path.home() / "models/vision/MANIFEST.json")
        result["image"]["source"] = "simulator_ephemeral_capture"
        self.episode.record("semantic_visual_inspection", result)
        return {key: result[key] for key in ("question", "answer", "latency_ms",
                                              "observed_at_unix_ms", "flight_authority")}

    def wait_action(self, receipt, timeout: int):
        if receipt.disposition == action_pb2.ACTION_STATE_REJECTED:
            raise RuntimeError(f"Action rejected: {receipt.message}")
        deadline = time.monotonic() + timeout
        latest = None
        for status in self.action_api.WatchActionStatus(
            drone_api_pb2.WatchActionStatusRequest(
                session_id=self.session_id, action_id=receipt.action_id
            ),
            timeout=timeout,
        ):
            latest = status
            print(
                f"{action_pb2.ActionType.Name(status.type)}: "
                f"{action_pb2.ActionState.Name(status.state)} — {status.message}",
                flush=True,
            )
            if status.state in TERMINAL:
                break
            if time.monotonic() > deadline:
                raise TimeoutError("Action status deadline exceeded")
        if latest is None or latest.state != action_pb2.ACTION_STATE_SUCCEEDED:
            detail = latest.message if latest else "no terminal status"
            raise RuntimeError("Action failed: " + detail)
        return latest

    def close(self):
        self.stop_renewal.set()
        if self.renewal_thread:
            self.renewal_thread.join(timeout=2)
        if self.episode:
            self.episode.seal(self.episode_outcome, self.episode_score)
            self.episode = None
        if self.lease_id:
            try:
                self.authority_api.ReleaseControl(
                    drone_api_pb2.ReleaseControlRequest(
                        session_id=self.session_id, lease_id=self.lease_id
                    ),
                    timeout=3,
                )
            except grpc.RpcError:
                pass
        if self.session_id:
            try:
                self.session_api.CloseSession(
                    drone_api_pb2.CloseSessionRequest(session_id=self.session_id),
                    timeout=3,
                )
            except grpc.RpcError:
                pass
        self.channel.close()


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--mission",
        choices=("takeoff_hover_land", "preflight"),
        default="takeoff_hover_land",
    )
    parser.add_argument("--altitude", type=float, default=3.0)
    parser.add_argument("--hover-seconds", type=float, default=10.0)
    parser.add_argument("--endpoint", default="127.0.0.1:50051")
    args = parser.parse_args()
    if not 1 <= args.altitude <= 30:
        parser.error("--altitude must be in 1..30 metres")
    if not 1 <= args.hover_seconds <= 120:
        parser.error("--hover-seconds must be in 1..120")

    client = MissionClient(args.endpoint, args.mission)
    result = {"status": "failed", "mission": args.mission}
    started = time.monotonic()
    try:
        client.connect()
        health = client.state_api.GetHealth(
            drone_api_pb2.GetHealthRequest(
                session_id=client.session_id, vehicle_id="icarus-01"
            ),
            timeout=5,
        )
        result["initial_health"] = MessageToDict(
            health, preserving_proto_field_name=True
        )
        if args.mission == "preflight":
            if not health.ready_to_arm:
                raise RuntimeError(
                    "Preflight not ready: " + ", ".join(health.blocking_reason_codes)
                )
            result["status"] = "passed"
            return 0

        client.acquire()
        arm = client.action_api.Arm(
            action_pb2.ArmRequest(context=client.context("arm")), timeout=5
        )
        client.wait_action(arm, 55)
        takeoff = client.action_api.Takeoff(
            action_pb2.TakeoffRequest(
                context=client.context("takeoff"),
                target_altitude_agl_m=args.altitude,
                heading=action_pb2.HeadingPolicy(
                    mode=action_pb2.HEADING_MODE_KEEP_CURRENT
                ),
                limits=action_pb2.ActionLimits(
                    maximum_climb_rate_mps=2,
                    execution_timeout_ms=60_000,
                ),
            ),
            timeout=5,
        )
        client.wait_action(takeoff, 70)
        hold = client.action_api.Hold(
            action_pb2.HoldRequest(
                context=client.context("hold"),
                duration_ms=int(args.hover_seconds * 1000),
                heading=action_pb2.HeadingPolicy(
                    mode=action_pb2.HEADING_MODE_KEEP_CURRENT
                ),
            ),
            timeout=5,
        )
        client.wait_action(hold, int(args.hover_seconds) + 15)
        land = client.action_api.Land(
            action_pb2.LandRequest(context=client.context("land")), timeout=5
        )
        terminal = client.wait_action(land, 100)
        result["terminal_state"] = MessageToDict(
            terminal.terminal_state, preserving_proto_field_name=True
        )
        result["status"] = "passed"
        return 0
    except Exception as error:  # noqa: BLE001 - persist deterministic failure evidence
        result["error"] = str(error)
        print("FAIL:", error, flush=True)
        return 1
    finally:
        client.episode_outcome = result["status"]
        client.episode_score = result
        result["duration_s"] = time.monotonic() - started
        output = ROOT / "logs/phase8"
        output.mkdir(parents=True, exist_ok=True)
        artifact = output / (
            f"mission_{time.strftime('%Y%m%dT%H%M%S')}_{uuid.uuid4().hex[:6]}.json"
        )
        artifact.write_text(json.dumps(result, indent=2) + "\n")
        print("Result:", artifact, flush=True)
        client.close()


if __name__ == "__main__":
    raise SystemExit(main())

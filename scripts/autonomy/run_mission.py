#!/usr/bin/env python3
"""Deterministic Phase 8 reference client; this file never imports MAVLink."""

import argparse
import json
import os
import re
import subprocess
import sys
import tempfile
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
    return bool(re.search(r"\bcount(?:\s+(?:the|a|an))?(?:\s+(?:number|amount|no\.?))?(?:\s+of)?(?:\s+unique)?\s+"
                          + re.escape(noun) + r"\b", text.lower()))


def _asks_building_inspection(text):
    return bool(re.search(
        r"\b(?:check|inspect|examine|observe|survey|look\s+at)\b[^.]{0,80}"
        r"\b(?:building|structure|tower)\b", text.lower()))


def _asks_hover_over_building(text):
    return bool(re.search(
        r"\bhover\b[^.]{0,50}\b(?:over|above)\b[^.]{0,30}"
        r"\b(?:it|building|structure|tower)\b", text.lower()))


class MissionClient:
    def __init__(self, endpoint: str, mission_name: str = "drone_api_mission",
                 vision_profile: str = "default"):
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
        self.vision_profile = vision_profile
        self.capabilities = None

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
        self.capabilities = capabilities
        session = None
        candidates = (
            (ROOT / "logs/simulation/active_session.json", "gazebo", "launcher_pid"),
            (ROOT / "logs/simulation/airsim_active_session.json", "airsim", "pid"),
        )
        for session_path, simulator, pid_key in candidates:
            if not session_path.is_file():
                continue
            candidate = json.loads(session_path.read_text())
            try:
                os.kill(int(candidate[pid_key]), 0)
            except (KeyError, ValueError, ProcessLookupError):
                continue
            candidate["simulator"] = simulator
            if simulator == "airsim":
                candidate.setdefault("profile", "airsim")
                candidate.setdefault(
                    "run_directory",
                    str((ROOT / Path(candidate["runtime_home"])).parent))
            session = candidate
            break
        self.simulation_session = session
        self.episode = Episode(ROOT, self.mission_name, self.endpoint, session)
        self.action_api = ActionRecorder(self.action_api, self.episode, self)
        self.episode.start_sampling(self)

    def mission_reference(self):
        """Return committed edge landmarks, never hidden ground-truth counts."""
        if not self.simulation_session or self.vision_profile != "edge-vision":
            return {}
        truth = {}
        if self.simulation_session.get("simulator") == "gazebo":
            scenario_path = ROOT / "simulation/scenarios" / (
                self.simulation_session["scenario"] + ".json")
            truth = json.loads(scenario_path.read_text()).get("ground_truth", {})
        reference = self.simulation_session.get("mission_reference", {})
        result = {"known_landmarks": reference.get(
                    "known_landmarks", truth.get("known_landmarks", [])),
                "count_window_s": reference.get(
                    "count_window_s", truth.get("count_window_s", 1)),
                "visual_requirements": {
                    "person_count_requires_detect": True,
                    "building_inspection_requires_inspect_scene": True,
                    "detector_classes": ["person"],
                    "known_landmarks_are_not_detector_classes": True,
                }}
        if reference.get("primary_target_id"):
            result["primary_target_id"] = reference["primary_target_id"]
        return result

    def mission_obligations(self, mission, history, observation):
        """Describe measurable completion obligations without choosing actions.

        Unlike the former demo state machine, this method never emits a
        required next action.  The DCM independently decomposes the mission;
        deterministic code only states what evidence completion must contain.
        """
        successful_items = [item for item in history
                            if item.get("outcome") == "SUCCEEDED"]
        successful = [item["action"] for item in successful_items]
        reference = self.mission_reference()
        primary = next((item for item in reference.get("known_landmarks", [])
                        if item.get("id") == reference.get("primary_target_id")), None)
        requirements = []
        if re.search(r"\b(?:arm|take\s*off|takeoff|hover|fly)\b", mission.lower()):
            altitude = re.search(
                r"\btake\s*off(?:\s+to)?\s+(\d+(?:\.\d+)?)\s*"
                r"(?:m|metre|metres|meter|meters)\b", mission.lower())
            requirements.append({"id": "flight", "status": "satisfied"
                                 if "takeoff" in successful else "pending",
                                 "target_altitude_agl_m":
                                     float(altitude.group(1)) if altitude else None})
        if _asks_building_inspection(mission):
            requirements.append({"id": "building_visual_inspection",
                                 "status": "satisfied" if "inspect_scene" in successful
                                 else "pending", "target_id": reference.get(
                                     "primary_target_id")})
        if _asks_hover_over_building(mission):
            waypoint = (primary or {}).get("hover_waypoint")
            def reached_reviewed_waypoint(item):
                arguments = item.get("arguments") or {}
                return (item.get("action") == "goto" and waypoint and
                        all(type(arguments.get(key)) in (int, float) and
                            abs(float(arguments[key]) - float(waypoint[key])) <= 1.0
                            for key in ("north_m", "east_m", "altitude_agl_m")))

            goto_index = next((index for index, item in enumerate(successful_items)
                               if reached_reviewed_waypoint(item)), None)
            hold_after = (goto_index is not None and
                          any(item["action"] == "hold"
                              for item in successful_items[goto_index + 1:]))
            seconds = re.search(r"\bhover\b[^.]{0,35}\bfor\s+(\d+(?:\.\d+)?)\s*seconds?\b",
                                mission.lower())
            requirements.append({"id": "building_hover",
                                 "status": "satisfied" if hold_after else "pending",
                                 "reviewed_waypoint": waypoint,
                                 "duration_ms": int(float(seconds.group(1)) * 1000)
                                 if seconds else 3000})
            evidence_start = (next((index for index, action in enumerate(successful)
                                    if action == "hold" and goto_index is not None
                                    and index > goto_index), -1) + 1)
        else:
            evidence_start = 0
        asks_people = (_asks_count_of(mission, "people") or
                       _asks_count_of(mission, "person"))
        if asks_people:
            requirements.append({"id": "person_count",
                                 "status": "satisfied"
                                 if "detect" in successful[evidence_start:] else "pending",
                                 "detector_classes": ["person"]})
        if re.search(r"\breturn\s+(?:to\s+)?home\b", mission.lower()):
            requirements.append({"id": "return_home",
                                 "status": "satisfied" if "return_home" in successful
                                 else "pending"})
        if not requirements:
            return {}
        return {"schema": "icarus.dcm.mission-obligations.v1",
                "requirements": requirements,
                "pending": [item["id"] for item in requirements
                            if item["status"] == "pending"],
                "authority": "completion_evidence_only"}

    # Compatibility for older callers; semantics are now obligation-only.
    mission_guidance = mission_obligations

    def capability_profile(self):
        """Return bounded facts introspected from the connected deployment."""
        session = self.simulation_session or {}
        capabilities = []
        if self.capabilities is not None:
            capabilities = sorted(item.name for item in self.capabilities.capabilities
                                  if item.supported)
        profile = {
            "schema": "icarus.dcm.live-body.v1",
            "vehicle_id": "icarus-01",
            "vehicle_type": getattr(self.capabilities, "vehicle_type", None),
            "autopilot": getattr(self.capabilities, "autopilot", None),
            "autopilot_version": getattr(self.capabilities, "autopilot_version", None),
            "capabilities": capabilities,
            "simulator": session.get("simulator", "physical"),
            "simulation_profile": session.get("profile"),
            "controller": session.get("controller", "ArduPilot"),
            "environment": session.get("environment"),
        }
        if session.get("simulator") == "airsim":
            settings_path = session.get("settings")
            if settings_path:
                path = ROOT / settings_path
                if path.is_file():
                    settings = json.loads(path.read_text())
                    vehicle = (settings.get("Vehicles") or {}).get("Icarus", {})
                    profile["airframe"] = {
                        "id": "airsim-default-multirotor",
                        "gazebo_airframe_equivalent": False,
                        "vehicle_type": vehicle.get("VehicleType"),
                    }
                    profile["sensors"] = sorted((vehicle.get("Sensors") or {}).keys())
                    profile["cameras"] = vehicle.get("Cameras", {})
            parameters = {}
            parameter_path = ROOT / "config/simulation/airsim/icarus-sitl.parm"
            if parameter_path.is_file():
                for line in parameter_path.read_text().splitlines():
                    line = line.strip()
                    if line and not line.startswith("#"):
                        name, value = line.split(None, 1)
                        parameters[name] = value
            profile["autopilot_parameter_overrides"] = parameters
        elif session.get("simulator") == "gazebo":
            mass_path = ROOT / "simulation/models/akshu_compact_sitl/mass_properties.json"
            mass = json.loads(mass_path.read_text()) if mass_path.is_file() else {}
            profile["airframe"] = {
                "id": "akshu_compact_sitl",
                "mass_kg": mass.get("total_mass_kg"),
                "gazebo_airframe_equivalent": True,
            }
            profile["sensors"] = ["forward_rgbd", "downward_rgbd", "lidar",
                                  "rangefinder", "imu", "gps", "magnetometer",
                                  "barometer"]
        return profile

    def assess_mission_completion(self, mission, executed):
        """Score requested edge evidence after landing without exposing it to the model."""
        if not self.simulation_session or self.vision_profile != "edge-vision":
            return None
        text = mission.lower()
        asks_people = _asks_count_of(text, "people") or _asks_count_of(text, "person")
        asks_buildings = _asks_count_of(text, "building") or _asks_count_of(text, "buildings")
        asks_building_inspection = _asks_building_inspection(text)
        asks_hover_over_building = _asks_hover_over_building(text)
        asks_return_home = bool(re.search(r"\breturn\s+(?:to\s+)?home\b", text))
        report = {"required": [], "satisfied": [], "reasons": []}
        person_results = [item.get("result") for item in executed
                          if item["action"] == "detect" and item["outcome"] == "SUCCEEDED"
                          and "person" in item.get("arguments", {}).get("classes", [])]
        if asks_people:
            report["required"].append("unique_person_count")
            if not person_results:
                report["reasons"].append("requested person count has no successful detect evidence")
            elif self.simulation_session.get("simulator") == "gazebo":
                scenario_path = ROOT / "simulation/scenarios" / (self.simulation_session["scenario"] + ".json")
                expected = json.loads(scenario_path.read_text())["ground_truth"]["unique_person_count"]
                observed = person_results[-1].get("unique_person_count")
                if observed == expected:
                    report["satisfied"].append("unique_person_count")
                else:
                    report["reasons"].append(
                        f"person count is {observed!r}; expected {expected} in this deterministic scenario")
            else:
                observed = person_results[-1].get("unique_person_count")
                truth = (self.simulation_session.get("mission_reference", {})
                         .get("person_count_ground_truth") or {})
                expected = truth.get("expected_visible_count")
                report["observed_person_count"] = observed
                if (type(expected) is int and expected >= 0
                        and truth.get("review_status") == "reviewed"
                        and isinstance(truth.get("camera_id"), str)
                        and truth.get("camera_id")):
                    report["expected_person_count"] = expected
                    report["ground_truth_camera_id"] = truth["camera_id"]
                    if observed == expected:
                        report["satisfied"].append("verified_person_count")
                    else:
                        report["reasons"].append(
                            f"person count is {observed!r}; reviewed AirSim truth is {expected}")
                else:
                    report["verification"] = "inconclusive_no_airsim_ground_truth"
                    report["reasons"].append(
                        "AirSim person count cannot be verified without reviewed scene ground truth")
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
                "stock YOLO11n has no building class; use a committed building landmark instead")
        if asks_building_inspection:
            report["required"].append("building_visual_inspection")
            inspected = [item.get("result") for item in executed
                         if item["action"] == "inspect_scene"
                         and item["outcome"] == "SUCCEEDED"]
            if inspected:
                report["satisfied"].append("building_visual_inspection")
            else:
                report["reasons"].append(
                    "requested building inspection has no successful inspect_scene evidence")
        if asks_hover_over_building:
            report["required"].append("building_hover")
            reference = self.simulation_session.get("mission_reference", {})
            primary_id = reference.get("primary_target_id")
            primary = next((item for item in reference.get("known_landmarks", [])
                            if item.get("id") == primary_id), None)
            waypoint = (primary or {}).get("hover_waypoint")
            if not waypoint:
                report["reasons"].append(
                    "requested building hover has no reviewed hover waypoint")
            else:
                goto_index = next((index for index, item in enumerate(executed)
                                   if item["action"] == "goto"
                                   and item["outcome"] == "SUCCEEDED"
                                   and all(abs(float(item["arguments"].get(key, 1e9))
                                               - float(waypoint[key])) <= 1.0
                                           for key in ("north_m", "east_m",
                                                       "altitude_agl_m"))), None)
                hold_index = next((index for index, item in enumerate(executed)
                                   if item["action"] == "hold"
                                   and item["outcome"] == "SUCCEEDED"
                                   and goto_index is not None and index > goto_index), None)
                if goto_index is None or hold_index is None:
                    report["reasons"].append(
                        "requested building hover did not reach the reviewed waypoint and hold")
                else:
                    report["satisfied"].append("building_hover")
                    if asks_people:
                        detect_after = any(
                            index > hold_index and item["action"] == "detect"
                            and item["outcome"] == "SUCCEEDED"
                            and "person" in item.get("arguments", {}).get("classes", [])
                            for index, item in enumerate(executed))
                        if not detect_after:
                            report["reasons"].append(
                                "requested person count was not observed after the building hover")
        if asks_return_home:
            report["required"].append("return_home")
            if any(item["action"] == "return_home" and item["outcome"] == "SUCCEEDED"
                   for item in executed):
                report["satisfied"].append("return_home")
            else:
                report["reasons"].append("requested return home did not complete")
        report["success"] = not report["reasons"]
        return report

    def _capture_rgb(self, output, *, camera="0"):
        """Capture one ephemeral frame from the active simulator."""
        if self.simulation_session.get("simulator") == "airsim":
            command = [str(ROOT / "scripts/capture-airsim-frame"),
                       "--output", str(output), "--camera", camera]
            source = "airsim_ephemeral_capture"
        else:
            command = [str(ROOT / "scripts/capture-camera-frame"),
                       "--output", str(output),
                       "--topic", "/icarus/sensors/rgbd_down/image"]
            source = "gazebo_ephemeral_capture"
        subprocess.run(command, cwd=ROOT, check=True, timeout=15,
                       capture_output=True, text=True)
        return source

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
        # AuthorityManager caps leases at 10 s. Renew well inside that window;
        # waiting exactly 10 s races expiry and would make a healthy client
        # indistinguishable from a dead DCM to the safety supervisor.
        while not self.stop_renewal.wait(3):
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
        if (self.vision_profile == "edge-vision" and
                self.simulation_session.get("simulator") == "gazebo" and
                "person" in classes):
            return self._edge_count_window(classes)
        # Pixels are inspection input, not episode data. Capture into an
        # ephemeral directory, retain only the image hash and structured
        # detector output, then remove the frame before returning.
        with tempfile.TemporaryDirectory(prefix="icarus-vision-") as temporary:
            suffix = ".png" if self.simulation_session.get("simulator") == "airsim" else ".ppm"
            output = Path(temporary) / ("frame" + suffix)
            source = self._capture_rgb(output)
            if self.vision_profile == "edge-vision":
                from python.perception.edge_vision import (
                    RateLimiter,
                    UniqueTrackCounter,
                    detect_edge_image,
                )
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
        result["source"] = source
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

        from python.perception.landing_zone import (
            assess_landing_zone,
            load_landing_source,
        )

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
            suffix = ".png" if self.simulation_session.get("simulator") == "airsim" else ".ppm"
            output = Path(temporary) / ("frame" + suffix)
            source = self._capture_rgb(output)
            if self.vision_profile == "edge-vision":
                from python.perception.edge_vision import RateLimiter, inspect_smolvlm
                if self.edge_vlm_limiter is None:
                    self.edge_vlm_limiter = RateLimiter(interval_s=5.0)
                result = inspect_smolvlm(output, question, Path.home() / "models/edge/MANIFEST.json",
                                         self.edge_vlm_limiter)
            else:
                result = inspect_image(output, question, Path.home() / "models/vision/MANIFEST.json")
        result["image"]["source"] = source
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

#!/usr/bin/env python3
"""Deterministic Phase 8 reference client; this file never imports MAVLink."""

import argparse
import json
import math
import os
import re
import shutil
import subprocess
import sys
import tempfile
import threading
import time
import uuid
from functools import wraps
from contextlib import nullcontext
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

_INFERENCE_LOCK_INIT = threading.Lock()


def _visual_view_context(pose, camera='forward_d435i'):
    yaw = pose.get('attitude', {}).get('yaw_rad')
    if type(yaw) not in (int, float) or not math.isfinite(yaw):
        return None
    return {'camera': camera, 'aircraft_heading_deg': math.degrees(yaw) % 360}


def _exclusive_inference(method):
    """Keep a cancelled mission's in-flight visual call from overlapping the next.

    The lock belongs to the client/session, not one mission graph. Reentrant
    calls (observe -> inspect) share the same worker and are allowed.
    """
    @wraps(method)
    def guarded(self, *args, **kwargs):
        with _INFERENCE_LOCK_INIT:
            if not hasattr(self, "_inference_lock"):
                self._inference_lock = threading.RLock()
            lock = self._inference_lock
        if not lock.acquire(blocking=False):
            raise RuntimeError("previous visual inference is still completing; no overlapping inference started")
        try:
            return method(self, *args, **kwargs)
        finally:
            lock.release()
    return guarded


def _asks_count_of(text, noun):
    """Match an explicit count request, not a nearby unrelated noun."""
    normalized = re.sub(r"\bno\.", "number", text.lower())
    # Natural operator requests often insert useful qualifiers ("all visible",
    # "unique", "in front of the building") between the counting verb and
    # noun. Keep the match within one clause so an unrelated noun in a later
    # instruction cannot silently create a detector obligation.
    direct = (r"\b(?:count|determine|identify|report)\b"
              r"[^.!?;,]{0,80}\b" + re.escape(noun) + r"\b")
    how_many = r"\bhow\s+many\s+" + re.escape(noun) + r"\b"
    return bool(re.search(direct, normalized) or re.search(how_many, normalized))


def _asks_person_detection(text):
    return bool(re.search(
        r"\bdetect\b[^.!?;,]{0,50}\b(?:people|person)\b", text.lower()))


def _asks_building_inspection(text):
    normalized = text.lower()
    inspection = re.search(
        r"\b(?:check|detect|inspect|examine|observe|survey|look\s+at)\b",
        normalized)
    target = re.search(r"\b(?:building|structure|tower)\b", normalized)
    # Resolve a simple reviewed-target pronoun such as "fly to the building,
    # visually inspect it". The target still has to be named by the operator;
    # this does not infer arbitrary scene objects.
    return bool(inspection and target)


def _asks_hover_over_building(text):
    return bool(re.search(
        r"\bhover\b[^.]{0,50}\b(?:over|above)\b[^.]{0,30}"
        r"\b(?:it|building|structure|tower)\b", text.lower()))


def _requested_hold_duration_ms(text):
    match = re.search(
        r"\b(?:hold(?:\s+position)?(?:\s+there)?|hover(?:\s+there)?)\s+for\s+"
        r"(\d+(?:\.\d+)?)\s*seconds?\b", text.lower())
    return int(float(match.group(1)) * 1000) if match else None


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
        self.edge_trackers = {}
        self.edge_detector_runtime = None
        self.edge_detection_limiter = None
        self.edge_vlm_limiter = None
        self.target_localizer = None
        self.target_registry = None
        self.depth_source = None
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
        required = {"arm", "takeoff", "hold", "land", "state_stream", "home_relative_local_targets"}
        available = {item.name for item in capabilities.capabilities if item.supported}
        missing = required - available
        if missing:
            guidance = (". Update and restart the Drone API backend before using this client; "
                        "the old backend cannot preserve above-home target altitude."
                        if "home_relative_local_targets" in missing else "")
            raise RuntimeError("Drone API lacks required capabilities: " + str(sorted(missing)) + guidance)
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
            profile["sensors"] = [
                "intel_realsense_d435i_rgbd_and_imu",
                "logitech_brio_4k_downward_rgb_only",
                "livox_mid360_lidar_and_imu_uniform_scan_approximation",
                "ark_flow_range_imu_and_derived_optical_flow",
                "pixhawk_6x_pro_triple_imu_dual_barometer",
                "holybro_h_rtk_f9p_gnss_and_compass",
                "battery_power_model",
            ]
            profile["sensor_limitations"] = [
                "landing depth is a simulation-only proxy, not a BRIO capability",
                "MID-360 uses a uniform Gazebo lattice, not the Livox non-repetitive pattern",
                "derived optical flow is recorded but not fused into the ArduPilot EKF",
                "H-RTK centimeter noise assumes an RTK fix; correction transport is not simulated",
            ]
            parameters = {}
            parameter_path = ROOT / "simulation/parameters/mark4_v2_base.parm"
            if parameter_path.is_file():
                for raw_line in parameter_path.read_text().splitlines():
                    line = raw_line.split("#", 1)[0].strip()
                    if not line:
                        continue
                    name, value = line.split(None, 1)
                    parameters[name] = value
            profile["autopilot_parameter_overrides"] = parameters

        parameters = profile.get("autopilot_parameter_overrides") or {}

        def parameter(name, default):
            try:
                return float(parameters.get(name, default))
            except (TypeError, ValueError):
                return float(default)

        # These are the only parameter-derived numbers useful to the decision
        # model. The complete parameter snapshot remains in BODY for audit,
        # while the model sees this compact, unit-labelled interpretation.
        yaw_ceiling = parameter("ATC_RATE_WPY_MAX", 60.0) or 60.0
        profile["motion_constraints"] = {
            "yaw_rate_deg_s": {
                "minimum_command": 5.0,
                "slow": 5.0,
                "default": 30.0,
                "maximum": min(60.0, yaw_ceiling),
                "parameter": "ATC_RATE_WPY_MAX",
            },
            "climb_rate_mps": {
                "minimum_command": 0.3,
                "slow": 0.5,
                "default": min(2.0, parameter("WP_SPD_UP", 2.0)),
                "maximum": min(2.0, parameter("WP_SPD_UP", 2.0)),
                "parameter": "WP_SPD_UP",
            },
            "ground_speed_mps": {
                "minimum_command": 0.5,
                "slow": 0.5,
                "default": min(3.0, parameter("WP_SPD", 3.0)),
                "maximum": min(3.0, parameter("WP_SPD", 3.0)),
                "parameter": "WP_SPD",
            },
        }
        return profile

    def assess_mission_completion(self, mission, executed):
        """Score requested evidence after landing without exposing it to the model."""
        if not self.simulation_session:
            return None
        text = mission.lower()
        asks_people = (_asks_count_of(text, "people") or
                       _asks_count_of(text, "person") or
                       _asks_person_detection(text))
        asks_buildings = _asks_count_of(text, "building") or _asks_count_of(text, "buildings")
        asks_building_inspection = _asks_building_inspection(text)
        asks_hover_over_building = _asks_hover_over_building(text)
        asks_return_home = bool(re.search(r"\breturn\s+(?:to\s+)?home\b", text))
        hold_duration_ms = _requested_hold_duration_ms(text)
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
                report["observed_person_count"] = observed
                report["verification"] = "inconclusive_no_airsim_ground_truth"
                report["reasons"].append(
                    "AirSim person count requires independent post-mission scoring truth")
            detect_index = max((index for index, item in enumerate(executed)
                                if item["action"] == "detect" and
                                item["outcome"] == "SUCCEEDED" and
                                "person" in item.get("arguments", {}).get("classes", [])),
                               default=None)
            if "orbit" in text:
                report["required"].append("inspection_orbit")
                orbit_index = next((index for index, item in enumerate(executed)
                                    if item["action"] == "orbit" and item["outcome"] == "SUCCEEDED"), None)
                if orbit_index is None or detect_index is None or detect_index < orbit_index:
                    report["reasons"].append("requested person count must be collected after the requested orbit")
                else:
                    report["satisfied"].append("inspection_orbit")
        if asks_buildings:
            report["required"].append("visual_building_count")
            report["reasons"].append(
                "stock YOLO11n has no building class; use current-camera VLM inspection")
        if asks_building_inspection:
            report["required"].append("building_visual_inspection")
            inspect_index = next((index for index, item in enumerate(executed)
                                  if item["action"] == "inspect_scene" and
                                  item["outcome"] == "SUCCEEDED"), None)
            if inspect_index is not None:
                report["satisfied"].append("building_visual_inspection")
            else:
                report["reasons"].append(
                    "requested building inspection lacks current-camera inspect_scene evidence")
        if asks_hover_over_building:
            report["required"].append("building_hover")
            report["reasons"].append(
                "visual-target hover requires operator-grounded 3-D coordinates; "
                "no environment waypoint is injected")
        if hold_duration_ms is not None:
            report["required"].append("hold_position")
            detect_index = max((index for index, item in enumerate(executed)
                                if item["action"] == "detect" and
                                item["outcome"] == "SUCCEEDED"), default=-1)
            held = any(index > detect_index and item["action"] == "hold" and
                       item["outcome"] == "SUCCEEDED" and
                       int(item.get("arguments", {}).get("duration_ms", 0)) >=
                       hold_duration_ms
                       for index, item in enumerate(executed))
            if held:
                report["satisfied"].append("hold_position")
            else:
                report["reasons"].append(
                    f"requested {hold_duration_ms} ms hold did not complete after evidence collection")
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
            camera = {"forward_d435i": "0", "downward_brio": "3"}.get(camera, camera)
            command = [str(ROOT / "scripts/capture-airsim-frame"),
                       "--output", str(output), "--camera", camera]
            source = "airsim_ephemeral_capture"
        else:
            topics = {
                "0": ("forward_d435i", "/icarus/sensors/d435i/image"),
                "forward_d435i": ("forward_d435i", "/icarus/sensors/d435i/image"),
                "forward_rgbd": ("forward_d435i", "/icarus/sensors/d435i/image"),
                "3": ("downward_brio", "/icarus/sensors/brio_down"),
                "downward_brio": ("downward_brio", "/icarus/sensors/brio_down"),
                "downward_rgbd": ("downward_brio", "/icarus/sensors/brio_down"),
            }
            if camera not in topics:
                raise ValueError(
                    "Gazebo camera must be 0/forward_d435i or 3/downward_brio")
            camera_id, topic = topics[camera]
            command = [str(ROOT / "scripts/capture-camera-frame"),
                       "--output", str(output),
                       "--topic", topic]
            source = f"gazebo_{camera_id}_ephemeral_capture"
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
        self.lease_expires_at_unix_ms = response.authority.expires_at_unix_ms
        self.renewal_error = None
        self.renewal_thread = threading.Thread(target=self._renew, daemon=True)
        self.renewal_thread.start()

    def _renew(self):
        # AuthorityManager caps leases at 10 s. Renew well inside that window;
        # waiting exactly 10 s races expiry and would make a healthy client
        # indistinguishable from a dead DCM to the safety supervisor.
        delay_s = 3.0
        while not self.stop_renewal.wait(delay_s):
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
                    self.renewal_error = response.message
                    self.stop_renewal.set()
                else:
                    self.lease_expires_at_unix_ms = response.authority.expires_at_unix_ms
                    self.renewal_error = None
                    delay_s = 3.0
            except grpc.RpcError as error:
                self.renewal_error = str(error)
                # A transient RPC failure does not mean a still-valid lease
                # was revoked. Retry only within the server-confirmed window;
                # never reacquire control over a manual operator.
                remaining_ms = self.lease_expires_at_unix_ms - int(time.time() * 1000)
                if remaining_ms <= 250 or error.code() not in (
                        grpc.StatusCode.UNAVAILABLE, grpc.StatusCode.DEADLINE_EXCEEDED):
                    self.stop_renewal.set()
                else:
                    delay_s = min(0.25, remaining_ms / 2000.0)

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

    def _edge_manifest(self):
        return Path(os.environ.get("EDGE_VISION_MANIFEST") or (Path.home() / "models/edge/MANIFEST.json"))

    def _edge_detector(self):
        from python.perception.edge_vision import EdgeDetectorRuntime
        if getattr(self, "edge_detector_runtime", None) is None:
            self.edge_detector_runtime = EdgeDetectorRuntime(self._edge_manifest())
        return self.edge_detector_runtime

    @_exclusive_inference
    def prepare_camera_detector(self):
        """Warm the advertised camera tool before accepting a flight mission.

        Importing the ML stack and initializing kernels in flight competes
        with the sensor/control services. This performs no camera observation
        and does not interpret the operator's mission.
        """
        if self.vision_profile != "edge-vision":
            return
        started = time.monotonic()
        result = self._edge_detector().prepare()
        if self.episode is not None:
            self.episode.record("mission_resource_preparation", {
                "tool": "yolo", "seconds": time.monotonic() - started, **result})
        return result

    @_exclusive_inference
    def prepare_mission_resources(self, plan, echo=print):
        """Prepare declared detector operations, never infer mission meaning."""
        def needs_detector(value):
            if isinstance(value, dict):
                return (value.get("tool") == "yolo" or value.get("action") == "detect"
                        or value.get("kind") == "detect"
                        or any(needs_detector(child) for child in value.values()))
            return isinstance(value, list) and any(needs_detector(child) for child in value)
        operations = plan.get("adaptive_nodes") or plan.get("objectives") or []
        if self.vision_profile != "edge-vision" or not needs_detector(operations):
            return
        echo("Preparing the camera detector before mission execution…")
        started = time.monotonic()
        result = self._edge_detector().prepare()
        if self.episode is not None:
            self.episode.record("mission_resource_preparation", {
                "tool": "yolo", "seconds": time.monotonic()-started, **result})
        echo("Camera detector ready.")

    def camera_detector_classes(self):
        """Configured image-detector vocabulary, independent of mission text."""
        if self.vision_profile != 'edge-vision':
            return []
        from python.perception.classes import YOLO_CLASSES
        return sorted(YOLO_CLASSES)

    @_exclusive_inference
    def detect(self, classes, *, window_s=None, camera=None):
        """Run a simulator camera inspection; never sends a flight command."""
        if self.episode is None or not self.episode.session:
            raise RuntimeError("semantic detection requires an active simulator episode")
        from python.perception.classes import YOLO_CLASSES
        from python.perception.vision import detect_image, normalize_classes

        # A fixed-vocabulary detector computes all configured classes in one
        # pass. The open-vocabulary backend's text-query limit does not apply.
        classes = normalize_classes(classes, maximum_classes=(
            len(self.camera_detector_classes()) if self.vision_profile == 'edge-vision' else 16))
        if self.vision_profile == "edge-vision":
            unsupported = sorted(set(classes) - YOLO_CLASSES)
            if unsupported:
                raise ValueError(
                    "Configured YOLO detector does not support classes: "
                    + ", ".join(unsupported)
                    + "; route these requests to inspect_scene/VLM")
        if (self.vision_profile == "edge-vision" and
                self.simulation_session.get("simulator") == "gazebo" and
                "person" in classes and camera is None):
            return self._edge_count_window(classes, duration_s=window_s)
        # Pixels are inspection input, not episode data. Capture into an
        # ephemeral directory, retain only the image hash and structured
        # detector output, then remove the frame before returning.
        with tempfile.TemporaryDirectory(prefix="icarus-vision-") as temporary:
            suffix = ".png" if self.simulation_session.get("simulator") == "airsim" else ".ppm"
            output = Path(temporary) / ("frame" + suffix)
            pose_before = MessageToDict(self.state(), preserving_proto_field_name=True) if camera else None
            source = self._capture_rgb(output, camera=camera or "0")
            captured_ms = int(time.time() * 1000)
            pose_after = MessageToDict(self.state(), preserving_proto_field_name=True) if camera else None
            metadata_path = output.with_suffix(output.suffix + ".json")
            if metadata_path.is_file():
                captured_ms = json.loads(metadata_path.read_text()).get("captured_at_unix_ms", captured_ms)
            if self.vision_profile == "edge-vision":
                from python.perception.edge_vision import (
                    RateLimiter,
                    UniqueTrackCounter,
                    detect_edge_image,
                )
                if self.edge_detection_limiter is None:
                    self.edge_detection_limiter = RateLimiter(interval_s=0.2)
                self.edge_detection_limiter.admit()
                trackers = self.edge_trackers
                tracker = trackers.setdefault(camera or "forward_d435i", UniqueTrackCounter())
                result = detect_edge_image(output, self._edge_manifest(),
                                           tracker, max_fps=5, runtime=self._edge_detector())
            else:
                result = detect_image(output, classes, Path.home() / "models/vision/MANIFEST.json")
        if "image" in result:
            result["image"].pop("path", None)
            result["image"]["source"] = "simulator_ephemeral_capture"
        result["source"] = source
        # Keep the model's next observation bounded: it needs count evidence,
        # not a potentially large box list or image payload.
        counts = {}
        for detection in result.get("detections", []):
            label = detection.get("class")
            if label in classes:
                counts[label] = counts.get(label, 0) + 1
        # Historical tracker totals remain in semantic_detection logs, never in
        # this single-view result: they cannot establish a current condition.
        summary = ({"count_scope": "current_camera_frame",
                "counts": {name: counts.get(name, 0) for name in classes},
                "classes": result["classes"], "requested_classes": classes,
                "observed_at_unix_ms": result["observed_at_unix_ms"],
                "flight_authority": False}
               if result.get("schema") == "icarus.edge.yolo.v1" else
               {"counts": result["counts"], "requested_classes": result["requested_classes"],
                "observed_at_unix_ms": result["observed_at_unix_ms"]})
        self.episode.record("semantic_detection", result)
        if "confidence_threshold" in result:
            summary["confidence_threshold"] = result["confidence_threshold"]
        summary["captured_at_unix_ms"] = captured_ms
        summary["camera"] = camera or "forward_d435i"
        if camera:
            from python.perception.capture_pose import bracket_pose
            summary["capture_pose"], summary["capture_pose_quality"] = bracket_pose(pose_before, pose_after)
        return summary

    def _edge_count_window(self, classes, duration_s=None):
        """Read the capped observer for the scenario's declared count window."""
        from python.perception.edge_vision import summarize_observer_window

        scenario_path = ROOT / "simulation/scenarios" / (self.simulation_session["scenario"] + ".json")
        if duration_s is None:
            duration_s = float(json.loads(scenario_path.read_text())["ground_truth"]["count_window_s"])
        duration_s = float(duration_s)
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
        return {"unique_person_count": result["unique_person_count"],
                "counts": result.get("counts", {"person": result["unique_person_count"]}),
                "classes": list(classes), "requested_classes": list(classes),
                "observed_at_unix_ms": finished_ms, "count_window_s": duration_s,
                "count_method": result["count_method"], "camera_evidence": result["camera_evidence"],
                "flight_authority": False}

    @_exclusive_inference
    def assess_landing_zone(self):
        """Run the non-flight depth boundary on the active simulator source.

        The active source is a calibrated belly RGB-D camera.  This remains
        evidence gathering only: even a suitable result cannot issue a land
        command or bypass landing guardrails.
        """
        if self.episode is None or not self.episode.session:
            raise RuntimeError("landing assessment requires an active simulator episode")
        if self.simulation_session.get("simulator") != "gazebo":
            raise RuntimeError("calibrated landing depth proxy is Gazebo-only")
        import numpy as np

        from python.perception.landing_zone import (
            assess_landing_zone,
            load_landing_source,
        )

        source = load_landing_source(ROOT / "config/perception/landing_zone.json")

        with tempfile.TemporaryDirectory(prefix="icarus-depth-") as temporary:
            output = Path(temporary) / "depth.npy"
            pose_before = MessageToDict(self.state(), preserving_proto_field_name=True)
            subprocess.run(
                [str(ROOT / "scripts/capture-depth-frame"), "--output", str(output),
                 "--topic", source["topic"]],
                cwd=ROOT, check=True, timeout=15)
            depth = np.load(output, allow_pickle=False)
            captured_ms = int(time.time() * 1000)
            pose_after = MessageToDict(self.state(), preserving_proto_field_name=True)
            result = assess_landing_zone(
                depth, optical_axis_body_frd=source["optical_axis_body_frd"],
                horizontal_fov_deg=source["horizontal_fov_deg"],
                vertical_fov_deg=source["vertical_fov_deg"])
            result["depth"] = {"source": "simulator_ephemeral_capture",
                               "sensor_id": source["sensor_id"],
                               "width": int(depth.shape[1]),
                               "height": int(depth.shape[0])}
        self.episode.record("semantic_landing_assessment", result)
        result["captured_at_unix_ms"] = captured_ms
        result["simulation_only"] = True
        from python.perception.capture_pose import bracket_pose
        result["capture_pose"], result["capture_pose_quality"] = bracket_pose(pose_before, pose_after)
        return result

    def capture_view(self, camera="forward_d435i"):
        """Capture one forward frame now for a later ``inspect_scene(view=...)``.

        Used at orbit viewpoints: the frame is taken while the vehicle holds
        the viewpoint and the (slow) VLM question about it runs afterwards.
        The pixels stay in a private temporary directory until that call
        consumes them; only the image hash and the answer reach the episode.
        """
        if self.episode is None or not self.episode.session:
            raise RuntimeError("view capture requires an active simulator episode")
        directory = tempfile.mkdtemp(prefix="icarus-view-")
        suffix = ".png" if self.simulation_session.get("simulator") == "airsim" else ".ppm"
        output = Path(directory) / ("frame" + suffix)
        try:
            capture_started_ms = int(time.time() * 1000)
            pose = MessageToDict(self.state(), preserving_proto_field_name=True)
            source = self._capture_rgb(output, camera=camera)
            after = MessageToDict(self.state(), preserving_proto_field_name=True)
        except BaseException:
            shutil.rmtree(directory, ignore_errors=True)
            raise
        from python.perception.capture_pose import bracket_pose
        capture_pose, pose_quality = bracket_pose(pose, after)
        metadata_path = output.with_suffix(output.suffix + ".json")
        metadata = json.loads(metadata_path.read_text()) if metadata_path.is_file() else {}
        return {"path": str(output), "directory": directory, "source": source,
                "camera": camera, "capture_started_at_unix_ms": capture_started_ms,
                "capture_pose": capture_pose, "capture_pose_quality": pose_quality,
                "captured_at_unix_ms": metadata.get("captured_at_unix_ms", int(time.time() * 1000))}

    @_exclusive_inference
    def inspect_scene(self, question, *, view=None, structured=False, result_type=None):
        """Run the pinned VLM on one ephemeral simulator camera frame.

        This method intentionally owns neither a command context nor an action
        RPC. It is a visual evidence boundary only.  ``view`` (from
        ``capture_view``) answers about that earlier frame, then deletes it.
        """
        if self.episode is None or not self.episode.session:
            raise RuntimeError("visual inspection requires an active simulator episode")
        from python.perception.vlm import inspect_image, normalize_question

        question = normalize_question(question)
        if not structured and result_type is None:
            result_type = 'description'
        with tempfile.TemporaryDirectory(prefix="icarus-vlm-") as temporary:
            suffix = ".png" if self.simulation_session.get("simulator") == "airsim" else ".ppm"
            output = Path(temporary) / ("frame" + suffix)
            if view is not None:
                shutil.move(view["path"], output)
                shutil.rmtree(view["directory"], ignore_errors=True)
                source = view["source"]
            else:
                source = self._capture_rgb(output)
            if self.vision_profile == "edge-vision":
                from python.perception.edge_vision import RateLimiter
                if self.edge_vlm_limiter is None:
                    self.edge_vlm_limiter = RateLimiter(interval_s=5.0)
                self.edge_vlm_limiter.admit()
                # Release the owned flight model before using the shared GPU.
                # Standalone callers without a runtime lease remain CPU-only.
                lease = getattr(self, 'visual_inference', lambda: nullcontext(0))
                with lease() as gpu_layers:
                    result = inspect_image(
                        output, question, self._edge_manifest(),
                        timeout_s=45.0, gpu_layers=gpu_layers,
                        view_context=_visual_view_context(view.get('capture_pose', {}), view.get('camera', 'forward_d435i')) if view else None,
                        **({"structured": True, "result_type": result_type} if structured or result_type else {}))
            else:
                result = inspect_image(output, question, Path.home() / "models/vision/MANIFEST.json",
                                       **({"structured": True, "result_type": result_type} if structured or result_type else {}))
        result["image"]["source"] = source
        if view is not None:
            result["image"]["captured_at_unix_ms"] = view["captured_at_unix_ms"]
        self.episode.record("semantic_visual_inspection", result)
        summary = {key: result[key] for key in ("question", "answer", "latency_ms",
                                              "observed_at_unix_ms", "flight_authority")}
        summary.update({key: result[key] for key in (
            "result_type", "answerable", "found", "count", "value", "validity", "error_code",
            "uncertainty", "count_scope") if key in result})
        summary["image"] = result["image"]
        if view is not None:
            summary.update({key: view[key] for key in (
                "captured_at_unix_ms", "camera", "capture_pose",
                "capture_started_at_unix_ms", "capture_pose_quality") if key in view})
        return summary

    @_exclusive_inference
    def observe_mission(self, observation):
        """Camera-specific evidence for mission conditions; no flight authority.

        A VLM returns a typed boolean/count or null in JSON. Malformed output
        stays unknown; prose is never converted into a flight condition.
        """
        tool, camera = observation["tool"], observation["camera"]
        capture_motion = observation.get("capture_motion")
        if capture_motion not in {None, "stationary"}:
            raise ValueError("observation capture_motion must be stationary when specified")
        if capture_motion == "stationary":
            # A stationary observation is a capture constraint, not a Hold
            # action. Confirm that any earlier flight command reached a
            # terminal state, then capture through the normal serialized
            # evidence boundary without issuing a new control command.
            self.ensure_action_settled()
        result_type = observation.get("result_type", "boolean")
        if result_type not in {"boolean", "count"}:
            raise ValueError("observation result_type must be boolean or count")
        if result_type == "count" and observation.get("count_scope", "visible_frame") != "visible_frame":
            raise ValueError("count observations support only a single visible frame")
        if tool == "landing" and result_type != "boolean":
            raise ValueError("landing assessment does not produce a count")
        if tool == "landing":
            if camera != "downward_brio":
                raise ValueError("landing assessment requires the downward view")
            pose = MessageToDict(self.state(), preserving_proto_field_name=True)
            result = self.assess_landing_zone()
            result.setdefault("capture_pose", {key: pose.get(key) for key in (
                "local_position_ned", "attitude", "relative_altitude_m",
                "observed_at_unix_ms")})
        elif tool == "yolo":
            from python.perception.classes import YOLO_CLASSES
            label = observation["query"]
            if label not in YOLO_CLASSES:
                raise ValueError("YOLO observation query must be a supported class; use VLM otherwise")
            pose = MessageToDict(self.state(), preserving_proto_field_name=True)
            result = self.detect([label], camera=camera)
            result.setdefault("capture_pose", {key: pose.get(key) for key in (
                "local_position_ned", "attitude", "relative_altitude_m",
                "observed_at_unix_ms")})
            count = result.get("counts", {}).get(label)
            valid = type(count) is int and count >= 0
            result.update({"count": count if valid else None,
                           "found": count > 0 if valid else None,
                           "value": (count if observation.get("result_type") == "count"
                                     else count > 0) if valid else None,
                           "result_type": observation.get("result_type", "boolean"),
                           "validity": "valid" if valid else "unknown",
                           "count_scope": "visible_frame"})
            if not valid:
                result["error_code"] = "missing_count"
        elif tool == "vlm":
            from python.perception.vlm import VlmInferenceError, decode_measurement
            result_type = observation.get("result_type", "boolean")
            view = self.capture_view(camera)
            try:
                result = self.inspect_scene(observation["query"], view=view,
                                            structured=True, result_type=result_type)
                # Decode at this boundary as well for alternate inspection backends.
                if "validity" not in result:
                    result.update(decode_measurement(result.get("answer"), result_type))
                elif result.get("validity") == "valid":
                    decoded = decode_measurement(result.get("answer"), result_type)
                    result["answer"] = decoded.get("answer", result.get("answer", ""))
            except (VlmInferenceError, FileNotFoundError, TimeoutError) as exc:
                result = {"validity": "error", "result_type": result_type,
                          "found" if result_type == "boolean" else "count": None,
                          "value": None, "error_code": getattr(exc, "code", "unavailable"),
                          "uncertainty": str(exc),
                          "observed_at_unix_ms": int(time.time() * 1000)}
                result.update({key: view[key] for key in (
                    "captured_at_unix_ms", "camera", "capture_pose",
                    "capture_started_at_unix_ms", "capture_pose_quality") if key in view})
            finally:
                shutil.rmtree(view["directory"], ignore_errors=True)
        else:
            raise ValueError("unknown observation tool")
        result.update({"tool": tool, "camera": camera, "query": observation["query"],
                       "flight_authority": False, "ground_truth_verified": False})
        if capture_motion == "stationary":
            result["capture_motion"] = "stationary"
            result["capture_constraint"] = {
                "enforcement": "serialized_capture_after_terminal_flight_action",
                "issued_flight_command": False,
                **({"pose_bracket": result["capture_pose_quality"]}
                   if "capture_pose_quality" in result else {}),
            }
        self.episode.record("mission_observation", result)
        return result

    def _depth_source(self):
        if getattr(self, "depth_source", None) is None:
            if not self.simulation_session or self.simulation_session.get("simulator") != "gazebo":
                return None
            from python.perception.depth_feedback import GazeboDepthSource
            self.depth_source = GazeboDepthSource(self.simulation_session,
                calibration_path=ROOT / "config/perception/target_localization.json")
        return self.depth_source

    def depth_feedback(self):
        source = self._depth_source()
        if source is None:
            return {"available": False, "reason": "No calibrated depth transport connected"}
        return source.summary()

    def _targets(self):
        if getattr(self, "target_registry", None) is None:
            from python.perception.target_registry import TargetRegistry
            self.target_registry = TargetRegistry()
        return self.target_registry

    def refresh_target(self, target_id):
        result = self._targets().refresh(target_id, self._perception_with_obstacles(),
            MessageToDict(self.state(), preserving_proto_field_name=True))
        if self.episode is not None:
            self.episode.record("target_tracking", result)
        return result

    def target_feedback(self):
        return self._targets().summaries()

    def perception_with_obstacles(self):
        """PerceptionSummary with the live LiDAR clusters, as a dict."""
        return self._perception_with_obstacles()

    def _perception_with_obstacles(self):
        perception_api = drone_api_pb2_grpc.PerceptionServiceStub(self.channel)
        return MessageToDict(perception_api.GetPerception(
            drone_api_pb2.GetPerceptionRequest(
                session_id=self.session_id, vehicle_id="icarus-01",
                include_nearby_obstacles=True), timeout=5),
            preserving_proto_field_name=True)

    @_exclusive_inference
    def localize_target(self, description, *, new_target=False):
        """Measure where an operator-described visible object is.

        The pinned VLM only decides which image region matches the description
        (a grounding box, or no match).  Bearing comes from camera geometry;
        range, centre and footprint come from Drone API LiDAR clusters and the
        vehicle state.  Like inspect_scene this owns no command context or
        action RPC, so it cannot move the vehicle.  Repeated calls for the same
        description fuse views (e.g. after separately authorized yaw actions)
        into agreement and spread metrics; ``new_target`` discards old views.
        """
        if self.episode is None or not self.episode.session:
            raise RuntimeError("target localization requires an active simulator episode")
        if self.simulation_session.get("simulator") != "gazebo":
            raise RuntimeError("target localization camera calibration is defined "
                               "only for the Gazebo D435i")
        from python.perception.target_localization import (
            TargetLocalizer,
            load_localization_config,
            normalize_description,
            visual_grounding_description,
        )
        from python.perception.vlm import ground_description

        description = normalize_description(description)
        grounding_target = visual_grounding_description(description)
        if getattr(self, "target_localizer", None) is None:
            self.target_localizer = TargetLocalizer(load_localization_config(
                ROOT / "config/perception/target_localization.json"))
        if new_target:
            self.target_localizer.reset(description)
        camera = self.target_localizer.config["camera"]
        manifest = (self._edge_manifest()
                    if self.vision_profile == "edge-vision"
                    else Path.home() / "models/vision/MANIFEST.json")
        with tempfile.TemporaryDirectory(prefix="icarus-target-") as temporary:
            output = Path(temporary) / "frame.ppm"
            # The pose and LiDAR snapshot bracket the frame so geometry refers
            # to the moment the pixels were taken, not the end of VLM inference.
            state = MessageToDict(self.state(), preserving_proto_field_name=True)
            source = self._capture_rgb(output, camera=camera["gazebo_camera"])
            captured_ms = int(time.time() * 1000)
            try:
                depth_source = self._depth_source()
                depth_frame = depth_source.snapshot() if depth_source else None
            except (ImportError, ValueError, KeyError, RuntimeError):
                depth_frame = None  # LiDAR localization remains available; context reports missing depth.
            perception = self._perception_with_obstacles()
            from python.perception.edge_vision import RateLimiter
            if getattr(self, "edge_vlm_limiter", None) is None:
                self.edge_vlm_limiter = RateLimiter(interval_s=5.0)
            self.edge_vlm_limiter.admit()
            lease = getattr(self, 'visual_inference', lambda: nullcontext(0))
            with lease() as gpu_layers:
                grounding = ground_description(
                    output, grounding_target["visual_query"], manifest,
                    timeout_s=90.0, gpu_layers=gpu_layers, view_context=_visual_view_context(state))
        estimate = self.target_localizer.observe(
            description,
            vlm_text=grounding["grounding_text"] if grounding["present"] else None,
            state=state, perception=perception,
            image_captured_at_unix_ms=captured_ms,
            refreshed_perception=self._perception_with_obstacles(),
            provenance=[
                {"sensor": camera["sensor_id"], "source": source,
                 "image_sha256": grounding["image"]["sha256"]},
                {"sensor": self.target_localizer.config["lidar"]["sensor_id"],
                 "source": "drone_api_perception_nearby_obstacles"},
                {"model": "vlm", "runtime": grounding["runtime"]},
            ])
        estimate["vlm"] = {key: grounding[key] for key in (
            "present", "presence_answer", "grounding_text", "latency_ms",
            "model_load_ms")}
        estimate["vlm"]["visual_query"] = grounding_target["visual_query"]
        estimate["vlm"]["view_selector"] = grounding_target["view"]
        estimate["vlm"]["view_context"] = grounding.get('view_context')
        estimate["latency_ms"] = int(time.time() * 1000) - captured_ms
        if depth_frame is not None and estimate.get("box_1000"):
            estimate["depth_at_capture"] = depth_frame.region(estimate["box_1000"], now_unix_ms=captured_ms)
            estimate["depth_at_capture"]["association"] = "VLM-selected RGB region; calibration alignment required; mixed foreground/background possible"
        estimate = self._targets().register(estimate, new_target=new_target)
        self.episode.record("semantic_target_localization", estimate)
        return estimate

    def ensure_action_settled(self):
        """Reconcile a broken status stream before any subsequent command.

        A stream deadline is not an action cancellation. Query first so a
        completed action is never interrupted; otherwise establish safe hold
        with CancelAction and confirm the original identity is terminal.
        """
        receipt = getattr(self, "_unsettled_action", None)
        if receipt is None:
            return None
        request = drone_api_pb2.GetActionStatusRequest(
            session_id=self.session_id, action_id=receipt.action_id)
        diagnostic = {"action_id": receipt.action_id, "settled": False}
        try:
            status = self.action_api.GetActionStatus(request, timeout=3)
            if status.action_id != receipt.action_id:
                raise RuntimeError("status identity mismatch during action reconciliation")
            if status.state not in TERMINAL:
                reply = self.action_api.CancelAction(action_pb2.CancelActionRequest(
                    context=self.context(f"stream-cancel-{uuid.uuid4().hex[:8]}"),
                    action_id=receipt.action_id), timeout=12)
                diagnostic["cancel_disposition"] = action_pb2.ActionState.Name(reply.disposition)
                # Cancellation receipts are not a replacement for the original
                # action's terminal status; a racing completion is valid too.
                deadline = time.monotonic() + 5
                while True:
                    status = self.action_api.GetActionStatus(request, timeout=3)
                    if status.action_id != receipt.action_id:
                        raise RuntimeError("status identity mismatch after cancellation")
                    if status.state in TERMINAL:
                        break
                    if time.monotonic() >= deadline:
                        raise RuntimeError("original action remains active after cancellation")
                    time.sleep(.1)
            diagnostic["terminal_state"] = action_pb2.ActionState.Name(status.state)
            diagnostic["settled"] = True
            self._unsettled_action = None
            return status
        except Exception as failure:
            diagnostic["error"] = str(failure)
            raise RuntimeError(
                f"Action {receipt.action_id} is not confirmed settled; "
                f"no replacement flight command will be sent: {failure}") from failure
        finally:
            episode = getattr(self, "episode", None)
            if episode is not None:
                episode.record("action_stream_reconciliation", diagnostic)

    def wait_action(self, receipt, timeout: int, on_status=None,
                    observer_errors_fatal=False, return_failure=False):
        """Wait through execution plus transport/mode-transition allowance.

        Broken streams are reconciled against the server and the original
        command is settled before the caller can attempt recovery.
        """
        if receipt.disposition == action_pb2.ACTION_STATE_REJECTED:
            raise RuntimeError(f"Action rejected: {receipt.message}")
        # Requested execution budgets can equal the executor's physical timer
        # (RTL:120s). Allow mode transition (10s) and delivery overhead without
        # expiring the stream at the same instant as the physical timeout.
        stream_timeout = timeout + 15
        deadline = time.monotonic() + stream_timeout
        latest = None
        last_printed = None
        failure = None
        self._unsettled_action = receipt
        try:
            for status in self.action_api.WatchActionStatus(
                drone_api_pb2.WatchActionStatusRequest(
                    session_id=self.session_id, action_id=receipt.action_id,
                    periodic_updates=observer_errors_fatal), timeout=stream_timeout,
            ):
                latest = status
                if on_status is not None:
                    try:
                        on_status(status)
                    except Exception as observer_failure:
                        if observer_errors_fatal:
                            raise
                        print(f"status observer failed: {observer_failure}", flush=True)
                printed = (status.type, status.state, status.message)
                if printed != last_printed:
                    print(f"{action_pb2.ActionType.Name(status.type)}: "
                          f"{action_pb2.ActionState.Name(status.state)} — {status.message}", flush=True)
                    last_printed = printed
                if status.state in TERMINAL:
                    self._unsettled_action = None
                    break
                if time.monotonic() > deadline:
                    raise TimeoutError("Action status deadline exceeded")
        except Exception as error:  # noqa: BLE001 - settle flight before propagating any stream/observer failure
            failure = error
        if self._unsettled_action is not None:
            latest = self.ensure_action_settled()
            if on_status is not None:
                try:
                    on_status(latest)
                except Exception as observer_failure:  # noqa: BLE001 - observer cannot bypass settlement
                    if observer_errors_fatal:
                        failure = failure or observer_failure
                    else:
                        print(f"status observer failed: {observer_failure}", flush=True)
        if return_failure and latest is not None and latest.state in TERMINAL:
            return latest
        if latest is None or latest.state != action_pb2.ACTION_STATE_SUCCEEDED:
            detail = latest.message if latest else "no terminal status"
            if failure is not None:
                detail += f" (status stream interrupted: {failure})"
            raise RuntimeError("Action failed: " + detail)
        if failure is not None and observer_errors_fatal and not isinstance(failure, grpc.RpcError):
            raise failure
        return latest

    def close(self):
        failures = []
        def cleanup(name, operation):
            try:
                operation()
            except Exception as failure:  # noqa: BLE001 - finish all owned cleanup before reporting
                failures.append(f"{name}: {failure}")
        settled = True
        try:
            self.ensure_action_settled()
        except RuntimeError as failure:
            settled = False
            print(f"Closing with unresolved action: {failure}. Renewal stops; "
                  "server lease-expiry failsafe remains responsible for control.", flush=True)
            self.episode_outcome = "unresolved_action"
        depth_source = getattr(self, "depth_source", None)
        if depth_source is not None:
            cleanup("depth source", depth_source.close)
            self.depth_source = None
        detector = getattr(self, "edge_detector_runtime", None)
        if detector is not None:
            cleanup("detector", detector.close)
            self.edge_detector_runtime = None
        self.stop_renewal.set()
        if self.renewal_thread:
            cleanup("renewal thread", lambda: self.renewal_thread.join(timeout=2))
        if self.episode:
            cleanup("episode seal", lambda: self.episode.seal(self.episode_outcome, self.episode_score))
            self.episode = None
        if self.lease_id and settled:
            try:
                self.authority_api.ReleaseControl(
                    drone_api_pb2.ReleaseControlRequest(
                        session_id=self.session_id, lease_id=self.lease_id
                    ),
                    timeout=3,
                )
            except grpc.RpcError:
                pass
        if self.session_id and settled:
            try:
                self.session_api.CloseSession(
                    drone_api_pb2.CloseSessionRequest(session_id=self.session_id),
                    timeout=3,
                )
            except grpc.RpcError:
                pass
        cleanup("channel", self.channel.close)
        if failures:
            raise RuntimeError("Session cleanup failed: " + "; ".join(failures))


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

#!/usr/bin/env python3
"""Deterministic Phase 8 reference client; this file never imports MAVLink."""

import argparse
import json
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
        self.episode = Episode(ROOT, self.mission_name, self.endpoint, session)
        self.action_api = ActionRecorder(self.action_api, self.episode, self)
        self.episode.start_sampling(self)

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

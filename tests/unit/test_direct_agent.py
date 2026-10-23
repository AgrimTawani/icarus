"""Behavior tests for the direct agent, independent of model and simulator."""

import io
import json
import math
import threading
import time
from types import SimpleNamespace

import pytest

from python.dcm._paths import ensure_generated_proto_on_path

ensure_generated_proto_on_path()
from icarus.v1 import action_pb2 as pb
from icarus.v1 import state_pb2 as sp

from python.dcm.agent_tools import ToolExecutor, validate_call
from python.dcm.fly import AgentSession, fly_mission
from python.dcm.operator_control import OperatorInterrupt


def call(name, **arguments):
    return {
        "role": "assistant",
        "tool_calls": [
            {
                "id": "test-call",
                "type": "function",
                "function": {"name": name, "arguments": json.dumps(arguments)},
            }
        ],
    }


def test_model_completion_is_labelled_as_assessment_not_independent_verification():
    from python.dcm.fly import summarize, session_episode_summary
    result = {'status': 'completed', 'message': 'Done.', 'decisions': 1,
              'assessment_source': 'mission_model'}
    assert 'Agent assessment: completed' in summarize(result)
    _, score = session_episode_summary([result])
    assert score['independent_mission_verification'] is False
    assert score['success_source'] == 'mission_model_completion_reports'
    result.pop('assessment_source')
    result['status'] = 'cancelled'
    assert 'Status: cancelled' in summarize(result)


def test_summary_surfaces_failed_attempts_without_overruling_recovery():
    from python.dcm.fly import summarize
    result = {'status': 'completed', 'message': 'Done.', 'decisions': 4,
              'assessment_source': 'mission_model', 'executed': [
                  {'action': 'orbit_target', 'result': {'status': 'ABORTED_BY_SAFETY'}},
                  {'action': 'orbit_target', 'result': {'status': 'SUCCEEDED'}},
                  {'action': 'return_home', 'result': {'status': 'SUCCEEDED'}}]}
    text = summarize(result)
    assert 'Agent assessment: completed' in text
    assert 'Recorded unsuccessful tool attempts (including retries): orbit_target [ABORTED_BY_SAFETY].' in text
    assert 'return_home [SUCCEEDED]' not in text
    assert result['status'] == 'completed'


class FakeClient:
    def __init__(self):
        self.sent = []
        self.episode = None
        self.fail = None
        self.vehicle = sp.DroneState(
            landed=True,
            armed=False,
            relative_altitude_m=0,
            observed_at_unix_ms=int(time.time() * 1000),
            attitude_received_at_unix_ms=int(time.time() * 1000),
            local_position_ned=sp.LocalPositionNed(origin_id="ekf-origin"),
            attitude=sp.Attitude(yaw_rad=0),
        )
        self.action_api = self
        self.index = 0

    def state(self):
        return self.vehicle

    def context(self, name):
        return pb.CommandContext(request_id=name)

    def __getattr__(self, name):
        if name not in (
            "Arm",
            "Takeoff",
            "Goto",
            "Hold",
            "Yaw",
            "Land",
            "ReturnHome",
            "Orbit",
            "Disarm",
            "ExecuteRoute",
            "CancelAction",
        ):
            raise AttributeError(name)

        def send(request, timeout):
            self.sent.append((name, request))
            return pb.ActionReceipt(
                action_id=str(len(self.sent)),
                disposition=pb.ACTION_STATE_REJECTED
                if self.fail == name
                else pb.ACTION_STATE_ACCEPTED,
                message="blocked by backend" if self.fail == name else "",
            )

        return send

    def wait_action(self, receipt, timeout, return_failure=False):
        name, request = self.sent[int(receipt.action_id) - 1]
        if name == "Arm":
            self.vehicle.armed = True
        if name == "Takeoff":
            self.vehicle.landed = False
            self.vehicle.local_position_ned.down_m -= (
                request.target_altitude_agl_m - self.vehicle.relative_altitude_m)
            self.vehicle.relative_altitude_m = request.target_altitude_agl_m
        if name == "Yaw":
            self.vehicle.attitude.yaw_rad += math.radians(request.relative_angle_deg)
        if name == "Goto":
            self.vehicle.local_position_ned.CopyFrom(request.destination.local_ned)
            self.vehicle.relative_altitude_m = (
                request.destination.local_ned.altitude_above_home_m
            )
        if name in ("Land", "ReturnHome"):
            self.vehicle.landed = True
            self.vehicle.armed = False
        self.vehicle.attitude_received_at_unix_ms = int(time.time() * 1000)
        return pb.ActionStatus(
            action_id=receipt.action_id,
            state=pb.ACTION_STATE_SUCCEEDED,
            terminal_state=self.vehicle,
            message="done",
        )

    def detect(self, **args):
        return {"count": 2, "fresh": True}


class Runtime:
    def __init__(self, *responses):
        self.responses = iter(responses)
        self.inputs = []
        self.tools = []

    def decide(self, messages, tools):
        self.inputs.append(messages)
        self.tools.append(tools)
        return next(self.responses)


def context(client):
    from python.dcm.agent_tools import message_dict

    return {
        "state": message_dict(client.state()),
        "perception": {"fresh": True},
        "defaults": {
            "takeoff_altitude_agl_m": 3,
            "yaw_rate_deg_s": 30,
            "maximum_ground_speed_mps": 3,
        },
    }


def run(client, runtime, mission="mission", **kw):
    kw.setdefault('session', AgentSession(mission))
    return fly_mission(
        client, runtime, mission, context_provider=context, echo=lambda *a: None, **kw
    )


def test_takeoff_arms_without_separate_arm_decision_then_reports_result():
    c = FakeClient()
    r = Runtime(call("takeoff", target_altitude_agl_m=5),
                call('finish', outcome='completed', message='Took off to 5 meters.'))
    result = run(c, r)
    assert [x[0] for x in c.sent] == ["Arm", "Takeoff"]
    assert result["decisions"] == 2 and result["status"] == "completed"


def test_completion_hint_without_final_report_cannot_claim_mission_success():
    c = FakeClient()
    r = Runtime(call('takeoff', target_altitude_agl_m=3))
    result = run(c, r, mission='Take off then orbit and land.', max_decisions=1)
    assert result['status'] == 'incomplete'
    assert 'Orbit' not in [name for name, _ in c.sent]


@pytest.mark.parametrize('heading', [0, 90, 180, 270, 359.5])
def test_face_heading_uses_native_yaw_without_replacing_the_position_target(heading):
    c = FakeClient()
    c.vehicle.armed = True
    c.vehicle.landed = False
    c.vehicle.relative_altitude_m = 4.2
    c.vehicle.local_position_ned.north_m = 12.3
    c.vehicle.local_position_ned.east_m = -8.4
    c.vehicle.attitude.yaw_rad = math.radians(45.754)
    result = ToolExecutor(c).execute('face_heading', {'heading_deg': heading})
    assert result['status'] == 'SUCCEEDED'
    assert [name for name, _ in c.sent] == ['Yaw']
    request = c.sent[0][1]
    assert request.relative_angle_deg == pytest.approx(math.remainder(heading-45.754,360))
    assert c.vehicle.local_position_ned.north_m == 12.3
    assert c.vehicle.local_position_ned.east_m == -8.4
    assert c.vehicle.relative_altitude_m == 4.2
    assert result['evidence']['absolute_heading_error_deg'] == pytest.approx(0)


@pytest.mark.parametrize('fault', ['landed', 'unarmed', 'missing_stamp', 'nonfinite', 'stale'])
def test_face_heading_requires_fresh_attitude_and_airborne_control_for_a_turn(fault):
    c = FakeClient()
    c.vehicle.armed = True
    c.vehicle.landed = False
    c.vehicle.attitude.yaw_rad = math.radians(90)
    if fault == 'landed': c.vehicle.landed = True
    if fault == 'unarmed': c.vehicle.armed = False
    if fault == 'missing_stamp': c.vehicle.attitude_received_at_unix_ms = 0
    if fault == 'nonfinite': c.vehicle.attitude.yaw_rad = float('nan')
    if fault == 'stale': c.vehicle.attitude_received_at_unix_ms -= 2000
    result = ToolExecutor(c).execute('face_heading', {'heading_deg': 0})
    assert result['status'] == 'FAILED' and not c.sent


def test_clarification_preserves_original_mission_and_answer():
    c = FakeClient()
    s = AgentSession("Take off to the altitude I choose, then land")
    r = Runtime(
        call("ask_user", question="What altitude?"),
        call("takeoff", target_altitude_agl_m=5),
        call("land"),
        call('finish', outcome='completed', message='Took off to the chosen altitude and landed.'),
    )
    assert run(c, r, session=s)["status"] == "awaiting_user"
    s.answer("5m")
    assert run(c, r, session=s)["status"] == "completed"
    assert r.inputs[-1][1]["content"].startswith(s.mission)
    assert any(m.get("content", "").startswith("5m") for m in r.inputs[-1])
    assert [a for a, _ in c.sent] == ["Arm", "Takeoff", "Land"]


def test_detection_result_is_available_before_conditional_move():
    c = FakeClient()
    c.vehicle.landed = False
    c.vehicle.armed = True
    c.vehicle.relative_altitude_m = 5
    r = Runtime(
        call("detect", classes=["person"]),
        call("move_relative", forward_m=5),
        call("finish", outcome="completed", message="Detected two people and moved forward 5 meters."),
    )
    assert run(c, r)["status"] == "completed"
    assert '"count":2' in r.inputs[1][-1]["content"]
    assert len(r.inputs) == 3


@pytest.mark.parametrize(
    "name,args",
    [
        ("yaw", {"angle_deg": True}),
        ("yaw", {"angle_deg": float("nan")}),
        ("goto", {"north_m": 1, "east_m": 2}),
        ("takeoff", {"target_altitude_agl_m": "5m"}),
        ("land", {"mission_complete": True}),
    ],
)
def test_only_api_types_are_validated(name, args):
    with pytest.raises(ValueError):
        validate_call(name, args)


def test_invalid_call_is_repairable_without_clarification_or_flight():
    c = FakeClient()
    r = Runtime(
        call("yaw", angle_deg="ninety"),
        call("yaw", angle_deg=90),
        call('finish', outcome='completed', message='Rotated 90 degrees.'),
    )
    assert run(c, r)["status"] == "completed"
    assert len(c.sent) == 1 and "INVALID_ARGUMENT" in r.inputs[1][-1]["content"]


def test_model_timeout_reports_failure_and_holds_airborne():
    c = FakeClient()
    c.vehicle.landed = False

    class Broken:
        def decide(self, *args):
            raise TimeoutError("model timed out")

    result = run(c, Broken())
    assert result["status"] == "incomplete"
    assert [a for a, _ in c.sent] == ["Hold"]


def test_operator_interrupt_during_inference_never_executes_late_answer():
    c = FakeClient()
    c.vehicle.landed = False
    interrupt = OperatorInterrupt(echo=lambda *a: None)
    release = threading.Event()

    class Slow:
        def decide(self, *args):
            release.wait(2)
            return call("takeoff", target_altitude_agl_m=5)

        def stop(self):
            release.set()

    timer = threading.Timer(0.1, lambda: interrupt.submit("/land"))
    timer.start()
    try:
        result = run(c, Slow(), interrupt=interrupt)
    finally:
        timer.join()
        release.set()
    assert result["status"] == "cancelled"
    assert [a for a, _ in c.sent] == ["Land"]


def test_approval_decline_does_not_move():
    c = FakeClient()
    r = Runtime(call("takeoff", target_altitude_agl_m=5))
    assert run(c, r, mode="approval", stream=io.StringIO("\n"))["status"] == "cancelled"
    assert not c.sent


def test_model_plain_text_does_not_claim_machine_verified_completion():
    c = FakeClient()
    r = Runtime({"content": "Mission completed."})
    result = run(c, r)
    assert result["status"] == "awaiting_user" and not c.sent


def test_hold_without_duration_finishes_command_but_keeps_hover_mode():
    c = FakeClient()
    c.vehicle.landed = False
    result = ToolExecutor(c).execute("hold", {})
    assert result["status"] == "SUCCEEDED"
    assert c.sent[0][1].duration_ms == 1000


def test_new_operator_command_supersedes_recovery():
    c = FakeClient()
    c.vehicle.landed = False
    interrupt = OperatorInterrupt(echo=lambda *a: None)
    original = c.wait_action

    def wait(receipt, timeout, return_failure=False):
        if c.sent[-1][0] == "ReturnHome":
            interrupt.submit("/land")
        return original(receipt, timeout, return_failure)

    c.wait_action = wait
    interrupt.enable_safety(c)
    interrupt.submit("/rtl")
    result = run(c, Runtime(), interrupt=interrupt)
    assert result["status"] == "cancelled"
    assert c.vehicle.landed and not c.vehicle.armed
    assert "land" in result["message"]
    assert not any(name == "Arm" for name, _ in c.sent)


def test_runtime_stop_latency_does_not_delay_operator_action():
    c = FakeClient()
    c.vehicle.landed = False
    interrupt = OperatorInterrupt(echo=lambda *a: None)
    release = threading.Event()

    class SlowStop:
        def decide(self, *a):
            release.wait(3)
            return call("land")

        def stop(self):
            release.wait(2)

    timer = threading.Timer(0.05, lambda: interrupt.submit("/hold"))
    timer.start()
    started = time.monotonic()
    try:
        assert run(c, SlowStop(), interrupt=interrupt)["status"] == "cancelled"
        assert time.monotonic() - started < 0.5
    finally:
        release.set()
        timer.join()


def test_nonzero_local_origin_and_above_home_altitude_are_not_conflated():
    c = FakeClient()
    c.vehicle.landed = False
    c.vehicle.armed = True
    c.vehicle.local_position_ned.north_m = 20
    c.vehicle.local_position_ned.east_m = -4
    c.vehicle.local_position_ned.down_m = -25
    c.vehicle.relative_altitude_m = 5
    result = ToolExecutor(c).execute("move_relative", {"forward_m": 2, "up_m": 1})
    assert result["status"] == "SUCCEEDED"
    target = c.sent[-1][1].destination.local_ned
    assert (
        target.north_m == 22
        and target.east_m == -4
        and target.altitude_above_home_m == 6
    )


def test_successful_takeoff_then_clarification_does_not_repeat_flight():
    c = FakeClient()
    s = AgentSession("take off, then inspect the one I choose")
    r = Runtime(
        call("takeoff", target_altitude_agl_m=3),
        call("ask_user", question="Which target?"),
        call(
            "finish",
            outcome="incomplete",
            message="Target inspection is not available in this test.",
        ),
    )
    assert run(c, r, session=s)["status"] == "awaiting_user"
    s.answer("the blue container")
    assert run(c, r, session=s)["status"] == "incomplete"
    assert [a for a, _ in c.sent] == ["Arm", "Takeoff"]


def test_route_preserves_speed_heading_dwell_and_completion():
    c = FakeClient()
    result = ToolExecutor(c).execute(
        "execute_route",
        {
            "points": [
                {
                    "north_m": 4,
                    "east_m": 2,
                    "altitude_agl_m": 6,
                    "maximum_ground_speed_mps": 1.3,
                    "heading_deg": 55,
                    "hold_time_s": 2.5,
                }
            ],
            "completion": "land",
        },
    )
    request = c.sent[-1][1]
    point = request.points[0]
    assert result["status"] == "SUCCEEDED"
    assert point.position.local_ned.altitude_above_home_m == 6
    assert point.limits.maximum_ground_speed_mps == 1.3
    assert point.heading.heading_deg == 55 and point.hold_time_ms == 2500
    assert request.completion_behavior == pb.ROUTE_COMPLETION_BEHAVIOR_LAND


def test_small_route_cannot_accept_one_stationary_pose_at_all_corners():
    c = FakeClient()
    c.vehicle.relative_altitude_m = 3
    points = [{'north_m': n, 'east_m': e, 'altitude_agl_m': 3}
              for n, e in [(0, 0), (0, .1), (.1, .1), (.1, 0), (0, 0)]]
    result = ToolExecutor(c).execute('execute_route', {'points': points})
    assert result['status'] == 'SUCCEEDED'
    route = c.sent[-1][1].points
    for left, right in zip(route, route[1:]):
        a, b = left.position.local_ned, right.position.local_ned
        separation = math.hypot(a.north_m-b.north_m, a.east_m-b.east_m)
        assert left.acceptance_radius_m + right.acceptance_radius_m < separation


def test_pattern_start_tolerance_is_not_tightened_by_altitude_settling_noise():
    c = FakeClient()
    c.vehicle.relative_altitude_m = 2.979
    result = ToolExecutor(c).execute('fly_pattern', {
        'shape': 'square', 'coverage': 'perimeter', 'width_m': 1, 'altitude_agl_m': 3})
    assert result['status'] == 'SUCCEEDED'
    assert c.sent[-1][1].points[0].acceptance_radius_m == 0.25


def test_native_protocol_preserves_proposals_and_full_mission_context():
    from unittest.mock import MagicMock, patch

    from python.dcm.contract import RuntimeDescriptor
    from python.dcm.llama_runtime import LlamaCppRuntime

    descriptor = RuntimeDescriptor(
        "fake", "qwen", "q4", "/unused", "unused", "llama_cpp", "test", 8192, 0, 5000
    )
    runtime = LlamaCppRuntime(descriptor, verify_artifact=False)
    runtime.process = object()
    response = MagicMock()
    response.__enter__.return_value.read.return_value = json.dumps(
        {
            "choices": [
                {
                    "finish_reason": "tool_calls",
                    "message": call("land"),
                }
            ],
            "usage": {"prompt_tokens": 12, "completion_tokens": 4},
        }
    ).encode()
    messages = [
        {"role": "system", "content": "rules"},
        {"role": "user", "content": "land"},
    ]
    with patch("urllib.request.urlopen", return_value=response) as post:
        result = runtime.decide(messages, [])
    body = json.loads(post.call_args.args[0].data)
    assert body["messages"] == messages
    # Backend may parse several proposals; the agent serializes actual execution.
    assert body["tool_choice"] == "required" and body["parallel_tool_calls"] is True
    assert result["tool_calls"][0]["function"]["name"] == "land"


def test_truncated_output_retries_once_without_repeating_an_action():
    from python.dcm.llama_runtime import IncompleteModelResponse

    class Truncated:
        def __init__(self):
            self.concise = []

        def decide(self, messages, tools, concise=False):
            self.concise.append(concise)
            if len(self.concise) == 3:
                return call('finish', outcome='completed', message='Took off to 3 meters.')
            if not concise:
                raise IncompleteModelResponse("cut off")
            return call("takeoff", target_altitude_agl_m=3)

    c = FakeClient()
    r = Truncated()
    result = run(c, r)
    assert result["status"] == "completed" and result["decisions"] == 2
    assert r.concise == [False, True, False]
    assert [n for n, _ in c.sent] == ["Arm", "Takeoff"]


def test_live_context_is_bounded_and_preserves_zero_and_missing_data(monkeypatch):
    from icarus.v1 import drone_api_pb2_grpc as rpc
    from icarus.v1 import perception_pb2 as pp

    from python.dcm.agent_context import capture_context

    c = FakeClient()
    c.session_id = "session"
    c.channel = object()
    c.capabilities = None
    c.state_api = SimpleNamespace(
        GetHealth=lambda *a, **k: sp.HealthReport(ready_to_arm=True)
    )
    perception = pp.PerceptionSummary(
        observed_at_unix_ms=int(time.time() * 1000), local_map_available=True
    )
    for i in range(40):
        o = perception.nearby_obstacles.add(
            obstacle_id=str(i), distance_m=i + 1, point_count=100
        )
        for j in range(100):
            o.hull.add(north_m=j, east_m=j)
    monkeypatch.setattr(
        rpc,
        "PerceptionServiceStub",
        lambda _: SimpleNamespace(GetPerception=lambda *a, **k: perception),
    )
    result = capture_context(c)
    assert result['registered_waypoints']['entries'] == []
    assert (
        result["state"]["armed"] is False
        and result["state"]["relative_altitude_m"] == 0
    )
    assert result["perception"]["fresh"] is True
    assert result["perception"]["camera_detection_evidence"]["inline_classification_present"] is False
    assert "available" not in result["perception"]["camera_detection_evidence"]
    assert "inspect_scene" in result["perception"]["camera_detection_evidence"]["obtain_current_camera_observation_with"]
    assert "vehicle_features" in result and "capabilities" not in result
    assert (
        result["perception"]["camera_detection_evidence"]["absence_confirmed"] is False
    )
    assert "detections_total" not in result["perception"]
    assert "detections" not in result["perception"]
    assert result["perception"]["nearby_obstacles_truncated"] is True
    assert result["perception"]["nearby_obstacles_total"] == 40
    assert len(result["perception"]["nearby_obstacles"]) == 8
    assert "hull" not in result["perception"]["nearby_obstacles"][0]
    assert len(json.dumps(result)) < 6500
    c.state_api.GetHealth = lambda *a, **k: (_ for _ in ()).throw(
        RuntimeError("offline")
    )
    assert capture_context(c)["health"]["available"] is False


@pytest.mark.parametrize("tool", ["scan_area", "fly_pattern"])
def test_scan_dispatch_uses_native_route_and_preserves_fixed_frame(tool):
    c = FakeClient()
    c.vehicle.relative_altitude_m = 3
    c.vehicle.local_position_ned.north_m = 10
    c.vehicle.local_position_ned.east_m = 20
    arguments = {
        "shape": "square", "coverage": "perimeter", "width_m": 1,
        
    }
    if tool == "scan_area":
        arguments["observation"] = "none"
    result = ToolExecutor(c).execute(tool, arguments)
    assert result["status"] == "SUCCEEDED"
    assert len(c.sent) == 1
    name, request = c.sent[0]
    assert name == "ExecuteRoute" and request.delegate_to_autopilot
    assert [(p.position.local_ned.north_m, p.position.local_ned.east_m)
            for p in request.points] == [(10, 20), (10, 21), (11, 21), (11, 20), (10, 20)]


def test_detection_pause_cannot_shortcut_remaining_mission():
    c = FakeClient()
    c.vehicle.relative_altitude_m = 3
    c.detect = lambda **kwargs: {"counts": {"person": 1}}
    executor = ToolExecutor(c)
    result = executor.execute("scan_area", {
        "shape": "square", "coverage": "perimeter", "width_m": 2,
        "observation": "detect", "classes": ["person"],
        "stop_on_detection": "person",
    })
    assert result["status"] == "SUCCEEDED"
    assert result["evidence"]["event"] == "condition_met"
    assert not result.get("workflow_complete")
    c.detect = lambda **kwargs: {"counts": {"person": 0}}
    resumed = executor.execute("resume_scan", {
        "scan_id": result["evidence"]["scan_id"],
    })
    assert resumed["workflow_complete"]


def test_local_origin_is_taken_from_current_aircraft_not_hardcoded():
    c = FakeClient()
    c.vehicle.local_position_ned.origin_id = "vehicle-42-origin"
    assert (
        ToolExecutor(c).execute(
            "goto", {"north_m": 1, "east_m": 2, "altitude_agl_m": 3}
        )["status"]
        == "SUCCEEDED"
    )
    assert c.sent[-1][1].destination.local_ned.origin_id == "vehicle-42-origin"


def test_already_disarmed_is_success_without_redundant_rpc():
    c = FakeClient()
    assert ToolExecutor(c).execute("disarm", {})["status"] == "SUCCEEDED"
    assert not c.sent


def test_exit_disarms_a_landed_armed_vehicle():
    from python.dcm.fly import safe_exit_recovery

    c = FakeClient()
    c.vehicle.armed = True
    wait = c.wait_action
    def completed(receipt, *args, **kwargs):
        result = wait(receipt, *args, **kwargs)
        if c.sent[-1][0] == 'Disarm':
            c.vehicle.armed = False
        return result
    c.wait_action = completed
    def measured_state():
        c.vehicle.observed_at_unix_ms = int(time.time() * 1000)
        c.vehicle.attitude_received_at_unix_ms = c.vehicle.observed_at_unix_ms
        c.vehicle.velocity_ned_mps.z = 0
        return c.vehicle
    c.state = measured_state
    assert safe_exit_recovery(c)["status"] == "SUCCEEDED"
    assert [name for name, _ in c.sent] == ['Disarm']


def test_exit_lands_here_if_return_home_is_rejected():
    from python.dcm.fly import safe_exit_recovery
    c = FakeClient()
    c.vehicle.armed = True
    c.vehicle.landed = False
    c.fail = 'ReturnHome'
    result = safe_exit_recovery(c)
    assert result['status'] == 'SUCCEEDED'
    assert result['return_home_attempt']['status'] == 'REJECTED'
    assert [name for name, _ in c.sent] == ['ReturnHome', 'Land']


def test_detector_default_is_current_forward_view_not_legacy_count_window():
    c = FakeClient()
    observed = []
    c.detect = lambda **kw: observed.append(kw) or {"counts": {"person": 0}}
    assert (
        ToolExecutor(c).execute("detect", {"classes": ["person"]})["status"]
        == "SUCCEEDED"
    )
    assert observed == [{"classes": ["person"], "camera": "forward_d435i"}]
    ToolExecutor(c).execute(
        "detect", {"classes": ["person"], "camera": "downward_brio"}
    )
    assert observed[-1]["camera"] == "downward_brio"


def test_cancel_final_call_reports_cancellation_not_mission_success():
    c = FakeClient()
    result = run(c, Runtime(call("cancel")))
    assert result["status"] == "cancelled"
    assert "cancelled" in result["message"].lower()
    assert not c.sent


def test_target_coordinates_cannot_be_relabelled_after_origin_change():
    c = FakeClient()
    with pytest.raises(ValueError, match="origin changed"):
        ToolExecutor(c)._flight("goto", {"north_m":1,"east_m":2,"altitude_agl_m":4},
                                runtime_target=True, expected_origin_id="old-origin")
    assert not c.sent


def test_named_launch_orbit_does_not_use_current_position():
    c = FakeClient()
    c._takeoff_reference = {'local_position_ned': {'north_m': 7, 'east_m': 2, 'down_m': 0, 'origin_id': 'ekf-origin'}}
    r = ToolExecutor(c).execute('orbit', {'center_reference': 'launch', 'radius_m': 3,
        'altitude_agl_m': 8, 'face_center': True})
    assert r['status'] == 'SUCCEEDED'
    request = c.sent[-1][1]
    assert request.center.local_ned.north_m == 7
    assert request.center.local_ned.east_m == 2
    assert not r['evidence']['camera_visibility_verified']


def test_named_orbit_reference_overrides_redundant_model_coordinates():
    c = FakeClient()
    c._takeoff_reference = {'local_position_ned': {
        'north_m': 7, 'east_m': 2, 'down_m': 0, 'origin_id': 'ekf-origin'}}
    r = ToolExecutor(c).execute('orbit', {'center_reference': 'launch',
        'center_north_m': 999, 'center_east_m': -999, 'radius_m': 3,
        'altitude_agl_m': 8, 'face_center': True})
    assert r['status'] == 'SUCCEEDED'
    assert c.sent[-1][1].center.local_ned.north_m == 7
    assert c.sent[-1][1].center.local_ned.east_m == 2
    assert r['evidence']['center_reference'] == 'launch'
    assert r['evidence']['center_north_m'] == 7


def test_launch_reference_survives_airborne_mission_and_rejected_takeoff():
    c = FakeClient()
    c.vehicle.local_position_ned.north_m = 5
    ex = ToolExecutor(c)
    assert ex.execute('takeoff', {'target_altitude_agl_m': 3})['status'] == 'SUCCEEDED'
    c.vehicle.local_position_ned.north_m = 12
    ex.execute('takeoff', {'target_altitude_agl_m': 4})
    assert c._takeoff_reference['local_position_ned']['north_m'] == 5
    failed = FakeClient(); failed.fail = 'Takeoff'
    ToolExecutor(failed).execute('takeoff', {'target_altitude_agl_m': 3})
    assert not hasattr(failed, '_takeoff_reference')


def test_impossible_required_camera_orbit_does_not_dispatch():
    c = FakeClient()
    c._takeoff_reference = {'local_position_ned': {'north_m': 0, 'east_m': 0, 'down_m': 0, 'origin_id': 'ekf-origin'}}
    r = ToolExecutor(c).execute('orbit', {'center_reference': 'launch', 'radius_m': 3,
        'altitude_agl_m': 8, 'face_center': True, 'camera': 'forward_d435i'})
    assert r['status'] == 'FAILED' and not c.sent
    assert r['evidence']['no_flight_executed']


def test_camera_incompatibility_pauses_before_model_can_change_parameters():
    c = FakeClient()
    session = AgentSession('Take off to 8m and orbit launch at 3m keeping the front camera on it.')
    r = Runtime(
        call('takeoff', target_altitude_agl_m=8),
        call('orbit', destination={'type':'launch'}, radius_m=3,
             altitude_agl_m=8, face_center=True, camera='forward_d435i'),
        call('land'),
        call('finish', outcome='incomplete', message='Cancelled the incompatible camera orbit and landed.'),
    )
    result = run(c, r, session=session)
    assert result['status'] == 'awaiting_user' and session.awaiting_user
    assert len(r.inputs) == 2
    assert [name for name, _ in c.sent] == ['Arm', 'Takeoff']
    assert '8 m' in result['message'] and '3 m radius' in result['message']
    session.answer('Cancel the orbit and land here.')
    result = run(c, r, session=session)
    assert result['status'] == 'incomplete'
    assert [name for name, _ in c.sent] == ['Arm', 'Takeoff', 'Land']
    assert 'Cancel the orbit and land here.' in json.dumps(r.inputs[2])


def test_unknown_camera_target_height_requires_measurement_not_user_choice():
    c = FakeClient()
    result = ToolExecutor(c).execute('orbit', {'center_north_m': 7, 'center_east_m': 2,
        'radius_m': 3, 'altitude_agl_m': 8, 'face_center': True, 'camera': 'forward_d435i'})
    assert result['status'] == 'FAILED' and not c.sent
    assert result['evidence']['camera_geometry']['feasible'] is None
    assert not result.get('requires_user_choice')


def test_unfittable_original_input_stops_without_flight_or_unbounded_packing_retries():
    from python.dcm.runtime_config import ContextBudgetExceeded
    class Unfittable:
        calls = 0
        def decide(self,messages,tools):
            self.calls += 1
            raise ContextBudgetExceeded('original request cannot fit')
    c=FakeClient();runtime=Unfittable()
    result=run(c,runtime)
    assert result['status']=='incomplete' and 'cannot fit' in result['message']
    assert runtime.calls==1 and not c.sent


def test_api_rejection_reason_survives_tool_result_and_execution_context(monkeypatch):
    from python.dcm.fly import brief
    client=FakeClient()
    monkeypatch.setattr(client,'Arm',lambda request,timeout:pb.ActionReceipt(
        action_id='denied',disposition=pb.ACTION_STATE_REJECTED,
        reason_code=pb.REASON_CODE_BATTERY_BELOW_THRESHOLD,message='Cannot arm.'))
    result=ToolExecutor(client).execute('arm',{})
    assert result['reason_code']=='REASON_CODE_BATTERY_BELOW_THRESHOLD'
    assert brief(result)['reason_code']=='REASON_CODE_BATTERY_BELOW_THRESHOLD'


def test_terminal_failure_preserves_api_reason_without_parsing_message(monkeypatch):
    client=FakeClient()
    monkeypatch.setattr(client,'wait_action',lambda *args,**kwargs:pb.ActionStatus(
        action_id='1',state=pb.ACTION_STATE_ABORTED_BY_SAFETY,
        reason_code=pb.REASON_CODE_STATE_STALE,message='Stopped.',terminal_state=client.vehicle))
    result=ToolExecutor(client).execute('hold',{'duration_s':1})
    assert result['status']=='ABORTED_BY_SAFETY'
    assert result['reason_code']=='REASON_CODE_STATE_STALE'


def test_absent_terminal_state_cannot_become_fabricated_airborne_state(monkeypatch):
    client=FakeClient();client.vehicle.armed=True
    monkeypatch.setattr(client,'wait_action',lambda *args,**kwargs:pb.ActionStatus(
        action_id='1',state=pb.ACTION_STATE_FAILED,reason_code=pb.REASON_CODE_INTERNAL_ERROR,
        message='No terminal telemetry supplied.'))
    result=ToolExecutor(client).execute('takeoff',{'target_altitude_agl_m':4})
    assert result['status']=='FAILED'
    assert 'state' not in result
    assert not hasattr(client,'_takeoff_reference')


@pytest.mark.parametrize('second_succeeds',[True,False])
def test_takeoff_recovers_once_only_after_confirmed_disarmed_ground(monkeypatch,second_succeeds):
    client=FakeClient();client.vehicle.armed=True
    ordinary_wait=client.wait_action
    attempts=0
    def wait(receipt,*args,**kwargs):
        nonlocal attempts
        name=client.sent[int(receipt.action_id)-1][0]
        if name=='Takeoff':
            attempts+=1
            if attempts==1 or not second_succeeds:
                client.vehicle.armed=False
                return pb.ActionStatus(action_id=receipt.action_id,state=pb.ACTION_STATE_FAILED,
                    reason_code=pb.REASON_CODE_AUTOPILOT_REJECTED,message='Rejected while disarmed.')
        return ordinary_wait(receipt,*args,**kwargs)
    monkeypatch.setattr(client,'wait_action',wait)
    result=ToolExecutor(client).execute('takeoff',{'target_altitude_agl_m':4})
    assert result['status']==('SUCCEEDED' if second_succeeds else 'FAILED')
    assert [name for name,_ in client.sent]==['Takeoff','Arm','Takeoff']
    assert result['takeoff_recovery']['failed_attempt']['reason_code']=='REASON_CODE_AUTOPILOT_REJECTED'
    assert hasattr(client,'_takeoff_reference')==second_succeeds


def test_interrupt_after_rearming_prevents_the_recovery_takeoff(monkeypatch):
    client=FakeClient();client.vehicle.armed=True
    executor=ToolExecutor(client)
    stopped=False;sent=[]
    def flight(name,args,**kwargs):
        nonlocal stopped
        sent.append(name)
        if name=='arm':
            stopped=True
            return {'status':'SUCCEEDED'}
        return {'status':'FAILED','reason_code':'REASON_CODE_AUTOPILOT_REJECTED'}
    monkeypatch.setattr(executor,'_flight',flight)
    monkeypatch.setattr(executor,'_confirm_disarmed_ground',lambda:{'landed':True,'armed':False})
    monkeypatch.setattr(executor,'interrupted',lambda:stopped)
    result=executor.execute('takeoff',{'target_altitude_agl_m':4})
    assert result['status']=='CANCELLED'
    assert sent==['takeoff','arm']


@pytest.mark.parametrize('reason,manual,landed',[
    (pb.REASON_CODE_AUTOPILOT_TIMEOUT,False,True),
    (pb.REASON_CODE_AUTOPILOT_REJECTED,True,True),
    (pb.REASON_CODE_AUTOPILOT_REJECTED,False,False)])
def test_takeoff_recovery_never_rearms_after_timeout_manual_control_or_airborne_state(monkeypatch,reason,manual,landed):
    client=FakeClient();client.vehicle.armed=True
    def wait(receipt,*args,**kwargs):
        client.vehicle.armed=False
        client.vehicle.manual_override_active=manual
        client.vehicle.landed=landed
        return pb.ActionStatus(action_id=receipt.action_id,state=pb.ACTION_STATE_FAILED,
            reason_code=reason,message='Stopped.')
    monkeypatch.setattr(client,'wait_action',wait)
    result=ToolExecutor(client).execute('takeoff',{'target_altitude_agl_m':4})
    assert result['status']=='FAILED'
    assert [name for name,_ in client.sent]==['Takeoff']


def test_near_stationary_goto_has_achievable_completion_tolerance():
    from python.dcm.actions import navigation_acceptance_radius
    assert navigation_acceptance_radius(0, {}) == .1
    assert navigation_acceptance_radius(.029435, {}) == .1
    assert navigation_acceptance_radius(1, {}) == .25
    assert navigation_acceptance_radius(30, {}) == 1.5
    assert navigation_acceptance_radius(.02, {"goto_acceptance_min_m": .05}) == .05
    with pytest.raises(ValueError):
        navigation_acceptance_radius(1, {"goto_acceptance_min_m": 2})


@pytest.mark.parametrize('recovery, expected_action', [('land', 'Land'), ('cancel', 'Hold')])
def test_repeated_camera_choices_preserve_operator_recovery_on_resume(recovery, expected_action):
    c = FakeClient()
    c.vehicle.landed = False
    c.vehicle.armed = True
    c.vehicle.relative_altitude_m = 8
    c.vehicle.local_position_ned.down_m = -8
    c._takeoff_reference = {'local_position_ned': {
        'north_m': 0, 'east_m': 0, 'down_m': 0, 'origin_id': 'ekf-origin'}}
    session = AgentSession('Orbit launch with the ground point in the front camera.')
    runtime = Runtime(*(
        call('orbit', destination={'type':'launch'}, radius_m=radius, altitude_agl_m=8,
             camera='forward_d435i', face_center=True) for radius in (3, 4, 5)),
        call(recovery),
        call('finish', outcome='cancelled', message='Cancelled the orbit and completed recovery.'))
    for index in range(3):
        if index:
            session.answer(f'Try radius {3+index} meters.')
        result = run(c, runtime, session=session)
        assert result['status'] == 'awaiting_user'
        assert not c.sent
    session.answer('Cancel the orbit and ' + ('land here.' if recovery == 'land' else 'hold here.'))
    result = run(c, runtime, session=session)
    assert result['status'] == 'cancelled'
    assert [name for name, _ in c.sent] == [expected_action]
    assert len(runtime.inputs) > 3


def test_camera_negotiation_does_not_erase_real_backend_failure_budget():
    session = AgentSession('Continue after operator clarification')
    for _ in range(3):
        session.actions.extend([
            {'action': 'arm', 'arguments': {}, 'result': {'status': 'REJECTED', 'message': 'backend rejection'}},
            {'action': 'orbit', 'arguments': {}, 'result': {'status': 'FAILED', 'requires_user_choice': True,
                                                         'question': 'Choose another camera.'}},
        ])
    c = FakeClient()
    result = run(c, None, session=session)
    assert result['status'] == 'incomplete'
    assert not c.sent


def test_orbit_defaults_to_facing_center_but_explicit_fixed_heading_is_preserved():
    for requested, expected in ((None, 'HEADING_MODE_FACE_POINT'), (False, 'HEADING_MODE_KEEP_CURRENT')):
        c = FakeClient()
        arguments = {'center_north_m': 0, 'center_east_m': 0, 'radius_m': 3, 'altitude_agl_m': 8,
                     'camera': 'none'}
        if requested is not None:
            arguments['face_center'] = requested
        result = ToolExecutor(c).execute('orbit', arguments)
        assert result['status'] == 'SUCCEEDED'
        assert c.sent[-1][1].heading.mode == getattr(pb, expected)


def test_console_shows_sensor_count_without_inventing_mission_success():
    c = FakeClient()
    c.detect = lambda **kwargs: {'counts': {'person': 5}, 'camera': 'forward_d435i'}
    messages = []
    result = fly_mission(c, Runtime(call('detect', classes=['person']),
                         call('finish', outcome='completed', message='Reported detections.')),
                         'Tell me the detections.', context_provider=context, echo=messages.append)
    assert result['status'] == 'completed'
    assert any('accepted detections person=5' in message for message in messages)
    assert not c.sent


def test_uncertain_visual_condition_cannot_approach_without_measured_localization():
    # Measured localization in the destination backend, not the VLM answer,
    # authorizes object flight.
    c = FakeClient()
    session = AgentSession('Observe then approach.')
    result = ToolExecutor(c, session=session).execute('approach_target', {
        'destination': {'type': 'visible_object', 'visual_query': 'landmark', 'view': 'current', 'standoff_m': 5}})
    assert result['status'] != 'SUCCEEDED'
    assert not c.sent


def camera_client(tmp_path, answers, localizations):
    """Real dispatch over a stubbed camera: every capture/VLM/localizer call is recorded."""
    c = FakeClient()
    c.vehicle.landed = False
    c.vehicle.armed = True
    c.camera_calls = []
    def capture_view(camera):
        c.camera_calls.append('capture')
        directory = tmp_path / f'view{len(c.camera_calls)}'
        directory.mkdir()
        return {'directory': str(directory)}
    def inspect_scene(question, view, **_):
        return {'validity': 'valid', 'found': answers.pop(0)}
    def localize_target(**_):
        c.camera_calls.append('localize')
        return localizations.pop(0)
    c.capture_view, c.inspect_scene, c.localize_target = capture_view, inspect_scene, localize_target
    return c


def test_contradictory_localization_blocks_same_view_reacquisition_without_camera_or_motion(tmp_path):
    c = camera_client(tmp_path, [True], [{'found': False}])
    session = AgentSession('Search for an object.')
    executor = ToolExecutor(c, session=session)
    inspect = {'question': 'Visible?', 'question_type': 'condition'}
    executor.execute('inspect_scene', inspect)
    executor.execute('localize_target', {'visual_query': 'landmark'})
    camera_calls = list(c.camera_calls)
    for name, args in [('inspect_scene', inspect),
                       ('inspect_scene', {'question': 'Describe the view.'}),
                       ('localize_target', {'visual_query': 'landmark', 'new_target': True}),
                       ('approach_target', {'destination': {'type': 'visible_object',
                           'visual_query': 'landmark', 'view': 'current', 'standoff_m': 4}})]:
        result = executor.execute(name, args)
        assert result['status'] == 'VIEWPOINT_REQUIRED' and result['no_action_executed']
    assert c.camera_calls == camera_calls
    assert not c.sent


def test_same_view_contradiction_loop_stops_without_motion(tmp_path):
    # The recorded live failure: positive VLM answer, failed localization, then
    # the model alternates inspection and localization without turning.
    c = camera_client(tmp_path, [True], [{'found': False}])
    question = {'question': 'Is the object visible?', 'question_type': 'condition'}
    session = AgentSession('Turn clockwise to search; approach the object when seen.')
    runtime = Runtime(call('inspect_scene', **question),
                      call('localize_target', visual_query='object', view='current', new_target=True),
                      call('inspect_scene', **question),
                      call('localize_target', visual_query='object', view='current', new_target=False),
                      call('inspect_scene', **question),
                      call('inspect_scene', **question))
    result = run(c, runtime, session=session)
    assert result['status'] == 'incomplete'
    assert [a['result']['status'] for a in session.actions] == [
        'SUCCEEDED', 'SUCCEEDED', 'VIEWPOINT_REQUIRED', 'VIEWPOINT_REQUIRED', 'VIEWPOINT_REQUIRED']
    assert c.camera_calls == ['capture', 'localize']
    assert not c.sent


def test_requested_turn_reopens_observation_from_the_new_view(tmp_path, monkeypatch):
    c = camera_client(tmp_path, [True], [])
    attempts = []
    def execute_destination(executor, operation, arguments):
        attempts.append(arguments)
        return ({'status': 'FAILED', 'evidence': {'target_localization': {'found': False}}}
                if len(attempts) == 1 else {'status': 'SUCCEEDED'})
    monkeypatch.setattr('python.dcm.destinations.execute_destination', execute_destination)
    destination = {'type': 'visible_object', 'visual_query': 'object', 'view': 'current', 'standoff_m': 4}
    turn = {'angle_deg': 17, 'direction': 'counterclockwise', 'rate_deg_s': 12}
    runtime = Runtime(call('inspect_scene', question='Visible?', question_type='condition'),
                      call('approach_target', destination=destination),
                      call('inspect_scene', question='Visible?', question_type='condition'),
                      call('yaw', **turn),
                      call('approach_target', destination=destination),
                      call('finish', outcome='completed', message='Approached after the requested turn.'))
    result = run(c, runtime)
    assert result['status'] == 'completed'
    assert [name for name, _ in c.sent] == ['Yaw']
    assert len(attempts) == 2
    assert [(e['action'], e['result']['status']) for e in result['executed']] == [
        ('inspect_scene', 'SUCCEEDED'), ('approach_target', 'FAILED'),
        ('inspect_scene', 'VIEWPOINT_REQUIRED'), ('yaw', 'SUCCEEDED'),
        ('approach_target', 'SUCCEEDED')]
    assert c.camera_calls == ['capture']  # no automatic re-inspection


def test_invalid_nonfinite_output_does_not_poison_later_feedback_json():
    c = FakeClient()
    session = AgentSession('mission')
    r = Runtime(call('yaw', angle_deg=float('inf')),
                call('finish', outcome='incomplete', message='Bad generated argument.'))
    result = run(c, r, session=session)
    assert result['status'] == 'incomplete' and not c.sent
    assert result['executed'] == []
    rejection = next(m for m in session.messages if m.get('role') == 'tool')
    assert json.loads(rejection['content'].split('\n')[0])['status'] == 'INVALID_ARGUMENT'


@pytest.mark.parametrize('heading,north,east', [(0, 5, 0), (90, 0, 5), (270, 0, -5)])
def test_relative_movement_receipt_reports_world_target_and_measured_displacement(heading, north, east):
    c = FakeClient()
    c.vehicle.landed = False
    c.vehicle.armed = True
    c.vehicle.attitude.yaw_rad = math.radians(heading)
    result = ToolExecutor(c).execute('move_relative', {'forward_m': 5})
    motion = result['motion']
    assert result['status'] == 'SUCCEEDED'
    assert motion['resolved_target']['north_m'] == pytest.approx(north, abs=1e-6)
    assert motion['resolved_target']['east_m'] == pytest.approx(east, abs=1e-6)
    assert motion['measured_displacement_ned_m']['north_m'] == pytest.approx(north, abs=1e-6)
    assert motion['measured_displacement_ned_m']['east_m'] == pytest.approx(east, abs=1e-6)
    assert motion['start']['heading_deg'] == pytest.approx(heading)
    assert motion['end']['origin_id'] == 'ekf-origin'


def test_rejected_goto_does_not_claim_measured_displacement():
    c = FakeClient()
    c.fail = 'Goto'
    result = ToolExecutor(c).execute('goto', {'north_m': 5, 'east_m': 2, 'altitude_agl_m': 3})
    assert result['status'] == 'REJECTED'
    assert result['motion']['resolved_target']['north_m'] == 5
    assert result['motion']['end'] is None
    assert result['motion']['measured_displacement_ned_m'] is None


def test_failed_goto_reports_actual_partial_motion_and_retains_it_in_feedback():
    c = FakeClient()
    def fail(receipt, timeout, return_failure=False):
        end = sp.DroneState()
        end.CopyFrom(c.vehicle)
        end.local_position_ned.north_m = 2
        return pb.ActionStatus(state=pb.ACTION_STATE_FAILED, message='stopped', terminal_state=end)
    c.wait_action = fail
    args = {'north_m': 10, 'east_m': 0, 'altitude_agl_m': 0, 'heading_deg': 90}
    result = ToolExecutor(c).execute('goto', args)
    assert result['status'] == 'FAILED'
    assert result['motion']['resolved_target']['north_m'] == 10
    assert result['motion']['resolved_target']['heading_deg'] == 90
    assert result['motion']['measured_displacement_ned_m']['north_m'] == 2


def test_changed_origin_makes_displacement_unknown_even_after_success():
    c = FakeClient()
    original_wait = c.wait_action
    def changed(receipt, timeout, return_failure=False):
        terminal = original_wait(receipt, timeout, return_failure)
        terminal.terminal_state.local_position_ned.origin_id = 'new-origin'
        return terminal
    c.wait_action = changed
    result = ToolExecutor(c).execute('goto', {'north_m': 5, 'east_m': 0, 'altitude_agl_m': 0})
    assert result['status'] == 'SUCCEEDED'
    assert result['motion']['start']['origin_id'] == 'ekf-origin'
    assert result['motion']['end']['origin_id'] == 'new-origin'
    assert result['motion']['measured_displacement_ned_m'] is None
    assert 'origin' in result['motion']['displacement_reason']


@pytest.mark.parametrize('yaw_deg,heading', [(359,1),(1,359),(90,90)])
def test_face_heading_already_aligned_is_fresh_non_actuating_even_while_landed(yaw_deg,heading):
    client=FakeClient()
    client.vehicle.attitude.yaw_rad=math.radians(yaw_deg)
    result=ToolExecutor(client).execute('face_heading',{'heading_deg':heading})
    assert result['status']=='SUCCEEDED'
    assert result['evidence']['no_flight_command_sent'] is True
    assert not client.sent


def test_face_heading_needs_no_new_navigation_coordinate_reference():
    client=FakeClient()
    client.vehicle.armed=True;client.vehicle.landed=False
    client.vehicle.local_position_ned.origin_id=''
    result=ToolExecutor(client).execute('face_heading',{'heading_deg':90})
    assert result['status']=='SUCCEEDED'
    assert [name for name,_ in client.sent]==['Yaw']


def test_face_heading_does_not_accept_a_yaw_receipt_without_actual_heading():
    client=FakeClient();client.vehicle.armed=True;client.vehicle.landed=False
    def unverified(receipt,timeout,return_failure=False):
        return pb.ActionStatus(action_id=receipt.action_id,state=pb.ACTION_STATE_SUCCEEDED)
    client.wait_action=unverified
    result=ToolExecutor(client).execute('face_heading',{'heading_deg':90})
    assert result['status']=='FAILED'
    assert result['evidence']['absolute_heading_error_deg'] is None


def test_multiple_proposals_execute_only_first_then_decide_from_observation():
    client=FakeClient()
    client.detect=lambda **kwargs:{'count':0,'fresh':True}
    first=call('detect',classes=['person'])
    first['tool_calls'].extend(call('takeoff',target_altitude_agl_m=8)['tool_calls'])
    runtime=Runtime(first,call('finish',outcome='completed',message='No people detected; stayed grounded.'))
    result=run(client,runtime,mission='Look for people before flying; stay grounded if none are detected.')
    assert result['status']=='completed'
    assert not client.sent
    assert [entry['action'] for entry in result['executed']]==['detect']
    assert len(first['tool_calls'])==2  # Raw proposal remains auditable.
    history=[c['function']['name'] for m in runtime.inputs[-1] for c in m.get('tool_calls',[])]
    assert history==['detect']
    assert '"person":0' in runtime.inputs[-1][-1]['content'] or '"count":0' in runtime.inputs[-1][-1]['content']


def test_multiple_flight_proposals_never_queue_a_later_action():
    client=FakeClient()
    first=call('arm')
    first['tool_calls'].extend(call('takeoff',target_altitude_agl_m=8)['tool_calls'])
    runtime=Runtime(first,call('ask_user',question='Continue with flight?'))
    result=run(client,runtime)
    assert result['status']=='awaiting_user'
    assert [name for name,_ in client.sent]==['Arm']



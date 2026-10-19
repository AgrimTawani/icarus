"""Capability feedback handles old servers and malformed optional metadata."""
from types import SimpleNamespace

from python.dcm.agent_context import advertised_motion_limits


def capabilities(**attributes):
    return SimpleNamespace(capabilities=[SimpleNamespace(
        name="motion_limits", supported=True, attributes=attributes)])


def test_limits_follow_connected_server_instead_of_machine_defaults():
    first = advertised_motion_limits(capabilities(
        maximum_climb_speed_mps="2.5", maximum_horizontal_speed_mps="4.0",
        maximum_descent_speed_mps="1.25"))
    other = advertised_motion_limits(capabilities(maximum_climb_speed_mps="1.75"))
    assert first == {"maximum_climb_speed_mps": 2.5,
                     "maximum_horizontal_speed_mps": 4.0,
                     "maximum_descent_speed_mps": 1.25}
    assert other == {"maximum_climb_speed_mps": 1.75}


def test_missing_or_unsupported_metadata_does_not_invent_limits():
    assert advertised_motion_limits(None) == {}
    old = SimpleNamespace(capabilities=[SimpleNamespace(name="takeoff", supported=True)])
    assert advertised_motion_limits(old) == {}
    disabled = capabilities(maximum_climb_speed_mps="2.5")
    disabled.capabilities[0].supported = False
    assert advertised_motion_limits(disabled) == {}


def test_malformed_limits_do_not_leak_nonfinite_json_or_fake_defaults():
    assert advertised_motion_limits(capabilities(
        maximum_climb_speed_mps="nan", maximum_horizontal_speed_mps="inf",
        maximum_descent_speed_mps="bad", unknown_field="1")) == {}
    assert advertised_motion_limits(capabilities(
        maximum_climb_speed_mps="-1", maximum_horizontal_speed_mps="0")) == {}


def test_context_exposes_advertised_policy_without_rounding_bounds(monkeypatch):
    from python.dcm._paths import ensure_generated_proto_on_path
    ensure_generated_proto_on_path()
    from icarus.v1 import drone_api_pb2 as api
    from icarus.v1 import drone_api_pb2_grpc as rpc
    from icarus.v1 import state_pb2 as state
    from icarus.v1 import perception_pb2 as perception
    from python.dcm.agent_context import capture_context

    caps = api.VehicleCapabilities()
    capability = caps.capabilities.add(name="motion_limits", supported=True)
    capability.attributes["maximum_climb_speed_mps"] = "1.23456789"
    client = SimpleNamespace(
        session_id="test", channel=object(), capabilities=caps,
        state=lambda: state.DroneState(),
        state_api=SimpleNamespace(GetHealth=lambda *a, **k: state.HealthReport()))
    monkeypatch.setattr(rpc, "PerceptionServiceStub", lambda channel: SimpleNamespace(
        GetPerception=lambda *a, **k: perception.PerceptionSummary()))
    feedback = capture_context(client)
    assert feedback["motion_limits"] == {"maximum_climb_speed_mps": 1.23456789}


def test_spatial_discovery_preserves_distinct_overall_and_runtime_target_fences():
    result = advertised_motion_limits(capabilities(
        maximum_distance_from_home_m='100', maximum_runtime_target_distance_from_home_m='30'))
    assert result == {'maximum_distance_from_home_m': 100,
                      'maximum_runtime_target_distance_from_home_m': 30}
    changed = advertised_motion_limits(capabilities(
        maximum_distance_from_home_m='57.125', maximum_runtime_target_distance_from_home_m='22.75'))
    assert changed == {'maximum_distance_from_home_m': 57.125,
                       'maximum_runtime_target_distance_from_home_m': 22.75}

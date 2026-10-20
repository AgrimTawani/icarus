"""Recovery must prove posture and motion, not just autopilot ground flags."""
import copy
import math
from types import SimpleNamespace

import pytest

from python.dcm import ground_recovery as ground
from python.dcm.agent_tools import message_dict, ToolExecutor
from python.dcm.fly import safe_exit_recovery
from test_direct_agent import FakeClient


@pytest.fixture
def clock(monkeypatch):
    current = [1000.0]
    monkeypatch.setattr(ground, 'time', SimpleNamespace(
        time=lambda: current[0], monotonic=lambda: current[0],
        sleep=lambda seconds: current.__setitem__(0, current[0] + seconds)))
    return current


def sample(clock):
    return {'landed': True, 'armed': False, 'attitude': {'roll_rad': 0, 'pitch_rad': 0},
            'ground_speed_mps': 0, 'velocity_ned_mps': {'z': 0},
            'observed_at_unix_ms': int(clock[0] * 1000),
            'attitude_received_at_unix_ms': int(clock[0] * 1000)}


def verify(clock, transform=lambda state: state):
    client = SimpleNamespace(state=lambda: transform(sample(clock)))
    return ground.verify_ground_recovery(client, message_dict)


def test_sustained_upright_stationary_ground_is_verified(clock):
    result = verify(clock)
    assert result['status'] == 'SUCCEEDED'
    assert result['evidence']['settled_duration_s'] >= 1
    assert result['evidence']['no_flight_command_sent']


def test_exact_crash_posture_is_failed_without_waiting_or_actuation(clock):
    def crash(state):
        state['attitude'].update(roll_rad=3.1378247738, pitch_rad=-1.2784944773)
        return state
    result = verify(clock, crash)
    assert result['status'] == 'FAILED'
    assert 'not upright' in result['message']
    assert clock[0] == 1000


@pytest.mark.parametrize('field,value', [
    ('ground_speed_mps', 1), ('ground_speed_mps', math.nan),
    ('ground_speed_mps', -1), ('attitude_received_at_unix_ms', 0),
    ('attitude', {}), ('velocity_ned_mps', {}),
    ('velocity_ned_mps', {'z': 1}), ('velocity_ned_mps', {'z': math.inf}),
    ('observed_at_unix_ms', 2000000),
])
def test_missing_stale_nonfinite_or_moving_state_never_proves_recovery(clock, field, value):
    assert verify(clock, lambda state: dict(state, **{field: value}))['status'] == 'FAILED'


def test_repeated_single_sample_is_not_sustained_evidence(clock):
    frozen = sample(clock)
    client = SimpleNamespace(state=lambda: copy.deepcopy(frozen))
    assert ground.verify_ground_recovery(client, message_dict)['status'] == 'FAILED'


def test_manual_override_remains_owned_by_operator(clock):
    assert verify(clock, lambda state: dict(state, manual_override_active=True))['status'] == 'manual_override'


@pytest.mark.parametrize('entry', ['land', 'exit'])
def test_both_public_paths_reject_crashed_disarmed_aircraft_without_rpc(clock, entry):
    c = FakeClient()
    c.state = lambda: dict(sample(clock), attitude={'roll_rad': math.pi, 'pitch_rad': 0})
    result = ToolExecutor(c).execute('land', {}) if entry == 'land' else safe_exit_recovery(c)
    assert result['status'] == 'FAILED'
    assert not c.sent


def test_settling_dwell_restarts_after_motion(clock):
    def unstable(state):
        if 1000.6 < clock[0] < 1000.8:
            state['ground_speed_mps'] = 1
        return state
    result = verify(clock, unstable)
    assert result['status'] == 'SUCCEEDED'
    assert clock[0] >= 1001.8

"""Public controller intent reaches native commands without model sign arithmetic."""
import copy

import pytest

from python.dcm.agent_tools import ToolExecutor, decision_tools, validate_call
from python.dcm.scan_workflow import prepare_pattern


@pytest.mark.parametrize('direction,angle,expected', [
    ('clockwise', 30, 30), ('counterclockwise', 30, -30),
    ('clockwise', 270, 270), ('counterclockwise', 0, 0),
])
def test_explicit_turn_direction_reaches_native_yaw(direction, angle, expected):
    from test_direct_agent import FakeClient
    client = FakeClient()
    client.vehicle.armed = True
    client.vehicle.landed = False
    client.vehicle.relative_altitude_m = 3
    args = {'angle_deg': angle, 'direction': direction}
    original = copy.deepcopy(args)
    validate_call('yaw', args, tools=decision_tools({}))
    result = ToolExecutor(client).execute('yaw', args)
    assert result['status'] == 'SUCCEEDED'
    assert client.sent[-1][0] == 'Yaw'
    assert client.sent[-1][1].relative_angle_deg == expected
    assert args == original


@pytest.mark.parametrize('args', [
    {'direction': 'counterclockwise', 'angle_deg': -30},
    {'direction': 'sideways', 'angle_deg': 30},
    {'direction': 'clockwise', 'angle_deg': True},
])
def test_invalid_turn_intent_never_dispatches(args):
    from test_direct_agent import FakeClient
    client = FakeClient()
    with pytest.raises(ValueError):
        ToolExecutor(client).execute('yaw', args)
    assert not client.sent


def test_native_horizontal_pattern_inherits_height_without_a_second_altitude_goal():
    tools = decision_tools({})
    args = {'shape': 'rectangle', 'coverage': 'perimeter', 'width_m': 2,
            'length_m': 1, 'heading_deg': 180}
    validate_call('fly_pattern', args, tools=tools)
    state = {'local_position_ned': {'north_m': 4, 'east_m': 7, 'origin_id': 'measured'},
             'attitude': {'yaw_rad': 0}, 'relative_altitude_m': 3}
    _, points = prepare_pattern(state, args)
    assert {point['altitude_agl_m'] for point in points} == {3}
    with pytest.raises(ValueError):
        validate_call('fly_pattern', {**args, 'altitude_agl_m': 8}, tools=tools)


def test_extra_native_field_is_rejected_before_sensor_or_flight_dispatch():
    from types import SimpleNamespace
    args = {'destination': {'type': 'visible_object', 'visual_query': 'structure',
                            'view': 'current'}, 'camera': 'forward_d435i'}
    with pytest.raises(ValueError, match='camera'):
        ToolExecutor(SimpleNamespace()).execute('overfly_target', args)

"""Object descriptions resolve measured identities without a planning layer."""
import pytest
from jsonschema import Draft202012Validator

from python.dcm.agent_tools import ToolExecutor, decision_tools, validate_call
from test_target_motion import Client


class DescribedClient(Client):
    def __init__(self, measurement=None, registered=True):
        super().__init__()
        self.measurement = measurement if measurement is not None else {'found': True, 'target_id': 'measured_123'}
        self.registered = registered
        self.localized = []
        self.refreshed = []

    def localize_target(self, description):
        self.localized.append(description)
        return self.measurement

    def target_feedback(self):
        return [{'target_id': 'measured_123'}] if self.registered else []

    def refresh_target(self, target_id):
        self.refreshed.append(target_id)
        return super().refresh_target(target_id)


def executor(client):
    result = ToolExecutor(client)
    result._flight = client.flight
    return result


@pytest.mark.parametrize('name,extra', [('approach_target', {'standoff_m': 3}),
                                      ('orbit_target', {'standoff_m': 3, 'camera': 'none'}),
                                      ('overfly_target', {})])
@pytest.mark.parametrize('measurement,registered', [
    ({'found': False, 'reason': 'not visible'}, True),
    ({'found': None, 'reason': 'depth unavailable'}, True),
    ({'found': True}, True),
    ({'found': True, 'target_id': 'invented'}, True),
    ({'found': True, 'target_id': 'measured_123'}, False),
])
def test_unmeasured_description_never_dispatches_flight(name, extra, measurement, registered):
    client = DescribedClient(measurement, registered)
    result = executor(client).execute(name, {'description': 'grey object', **extra})
    assert result['status'] == 'FAILED'
    assert result['evidence']['target_localization'] == measurement
    assert client.calls == client.refreshed == []


def test_description_localizes_then_real_controller_refreshes_and_moves():
    client = DescribedClient()
    result = executor(client).execute('approach_target', {'description': 'grey object', 'standoff_m': 3})
    assert result['status'] == 'SUCCEEDED'
    # depth first (no depth here), then LiDAR acquisition; both ground the nearest instance
    assert client.localized == ['nearest grey object', 'nearest grey object']
    assert set(client.refreshed) == {'measured_123'}
    assert client.north == pytest.approx(7)
    assert result['evidence']['target_localization']['target_id'] == 'measured_123'


def test_known_identity_does_not_repeat_visual_localization():
    client = DescribedClient()
    result = executor(client).execute('approach_target', {'target_id': 'existing', 'standoff_m': 3})
    assert result['status'] == 'SUCCEEDED'
    assert client.localized == []
    assert set(client.refreshed) == {'existing'}


@pytest.mark.parametrize('name,extra', [('approach_target', {'standoff_m': 3}),
                                        ('orbit_target', {'standoff_m': 3, 'camera': 'none'}),
                                        ('overfly_target', {})])
def test_exactly_one_reference_in_static_and_live_catalogs(name, extra):
    from python.dcm.agent_tools import SCHEMAS
    static_validator = Draft202012Validator(SCHEMAS[name])
    for reference in ({'target_id': 'known'}, {'description': 'visible object'}):
        validate_call(name, {**extra, **reference})
        assert static_validator.is_valid({**extra, **reference})
    for reference in ({}, {'target_id': 'known', 'description': 'visible object'}):
        with pytest.raises(ValueError):
            validate_call(name, {**extra, **reference})
        assert not static_validator.is_valid({**extra, **reference})
    operation='orbit' if name=='orbit_target' else 'overfly_target' if name=='overfly_target' else 'approach_target'
    args=({'camera':'none','standoff_m':3} if operation=='orbit' else
          {} if name=='overfly_target' else
          {'object_position':'standoff','standoff_m':3})
    for context in ({},{'targets':[{'target_id':'known'}]}):
        catalog=decision_tools(context)
        validate_call(operation,{**args,'destination':{'type':'visible_object',
            'visual_query':'visible object','view_heading_deg':0}},tools=catalog)
        if context:
            validate_call(operation,{**args,'destination':{'type':'registered_target','target_id':'known'}},tools=catalog)
        for reference in ({},{'type':'registered_target','target_id':'invented'},
                          {'type':'registered_target','target_id':'known','visual_query':'visible object'}):
            with pytest.raises(ValueError):
                validate_call(operation,{**args,'destination':reference},tools=catalog)


def test_object_alternatives_have_complete_typed_grammar_branches():
    from python.dcm.agent_tools import SCHEMAS
    for name in ('approach_target', 'orbit_target', 'overfly_target'):
        for branch in SCHEMAS[name]['oneOf']:
            reference = 'target_id' if 'target_id' in branch['properties'] else 'description'
            assert branch['type'] == 'object'
            assert branch['additionalProperties'] is False
            assert reference in branch['required']
            assert not Draft202012Validator(branch).is_valid({})
            if name != 'overfly_target':
                assert 'standoff_m' in branch['required']


class StripClient(DescribedClient):
    """The far fix ('strip') is lost up close; the house is measured again from there."""
    def __init__(self):
        super().__init__({'found': True, 'target_id': 'strip'})

    def target_feedback(self):
        return [{'target_id': 'strip', 'description': 'house'}, {'target_id': 'house'}]

    def localize_target(self, description):
        self.localized.append(description)
        if len(self.localized) > 2:  # after the depth try and the far LiDAR fix
            self.measurement = {'found': True, 'target_id': 'house'}
        return self.measurement

    def refresh_target(self, target_id):
        target = super().refresh_target(target_id)
        if target_id == 'strip' and len(self.refreshed) > 1:
            target.update(found=False, reason='target_not_observed')
        return target


def test_lost_far_fix_is_measured_again_once_and_the_approach_continues():
    client = StripClient()
    result = executor(client).execute('approach_target', {'description': 'house', 'standoff_m': 3})
    assert result['status'] == 'SUCCEEDED'
    assert result['evidence']['target_id'] == 'house'
    assert 'remeasured_after_target_lost' in result['evidence']
    assert client.localized == ['nearest house'] * 3  # depth try, LiDAR fix, re-measure


def test_orbit_falls_back_to_depth_when_lidar_geometry_is_partial():
    client = DescribedClient()
    client.partial = True
    client.target_feedback = lambda: [{'target_id': 'measured_123', 'description': 'house'}]
    client.measurement = {'found': False, 'reason': 'target_association_ambiguous',
                          'bearing_deg': {'image_absolute': 0.0, 'image_span_relative': [-20.0, 20.0]},
                          'depth_at_capture': {'p10_optical_z_m': 6.0, 'median_optical_z_m': 9.0}}
    result = executor(client).execute('orbit_target', {'target_id': 'measured_123', 'standoff_m': 5, 'camera': 'none'})
    assert result['status'] == 'SUCCEEDED'
    assert result['evidence']['orbit_centre_measured_by'] == 'vlm_box_bearing_and_forward_depth'
    assert 'lidar_measurement_failed' in result['evidence']
    orbit = [args for name, args, kw in client.calls if name == 'orbit'][-1]
    assert orbit['face_center'] and orbit['revolutions'] == 1.0


def test_safety_abort_never_falls_back_to_depth():
    client = DescribedClient()
    def blocked(name, args, **kw):
        client.calls.append((name, args, kw))
        return {'status': 'ABORTED_BY_SAFETY', 'reason_code': 'REASON_CODE_OBSTACLE', 'message': 'blocked'}
    client.flight = blocked
    result = executor(client).execute('approach_target', {'target_id': 'measured_123', 'standoff_m': 3})
    assert result['status'] == 'ABORTED_BY_SAFETY'
    assert client.localized == []

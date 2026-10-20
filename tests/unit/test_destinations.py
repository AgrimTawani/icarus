"""Reference types choose measured controllers; no mission-specific aliases."""
import copy
import json
import time
from types import SimpleNamespace
import pytest
from python.dcm.agent_tools import ToolExecutor, decision_tools, validate_call
from python.dcm.destinations import execute_destination, waypoint_catalogue
from test_direct_agent import FakeClient


class Executor:
    def __init__(self):
        self.value={'local_position_ned':{'north_m':100,'east_m':200,'down_m':-3,'origin_id':'frame'},
                    'relative_altitude_m':3,'observed_at_unix_ms':2000,
                    'home_local_position_ned':{'north_m':10,'east_m':20,'down_m':0,'origin_id':'frame'},
                    'home_position_received_at_unix_ms':1000}
        self.client=SimpleNamespace(state=lambda:copy.deepcopy(self.value))
        self.calls=[];self.failure=None
    def execute(self,name,args):
        self.calls.append((name,copy.deepcopy(args)))
        return {'status':self.failure or 'SUCCEEDED','evidence':{'measured':True}}
    def _flight(self,name,args,expected_origin_id=None):
        if expected_origin_id!=self.value['local_position_ned']['origin_id']:
            raise ValueError('origin changed')
        return self.execute(name,args)


def checked(operation,args,ex=None):
    validate_call(operation,args,tools=decision_tools({'targets':[{'target_id':'known'}]}))
    return execute_destination(ex or Executor(),'goto' if operation=='approach_target' else operation,args)


def test_visible_destination_orients_then_localizes_through_existing_controller():
    ex=Executor()
    result=checked('approach_target',{'destination':{'type':'visible_object','visual_query':'visible structure',
                 'view_heading_deg':0},'object_position':'standoff','standoff_m':4},ex)
    assert ex.calls==[('face_heading',{'heading_deg':0}),
                     ('approach_target',{'visual_query':'visible structure','standoff_m':4})]
    assert result['evidence']['destination']['controller']=='approach_target'
    assert result['evidence']['measured']


def test_above_object_uses_overflight_controller_not_altitude_only_goto():
    ex=Executor()
    args={'destination':{'type':'registered_target','target_id':'known'},'clearance_above_m':2}
    validate_call('overfly_target',args,tools=decision_tools({'targets':[{'target_id':'known'}]}))
    execute_destination(ex,'goto',{**args,'object_position':'above'})
    assert ex.calls==[('overfly_target',{'target_id':'known','clearance_above_m':2})]


def test_object_orbit_uses_surface_controller_not_coordinate_radius():
    ex=Executor()
    checked('orbit',{'destination':{'type':'registered_target','target_id':'known'},
                      'standoff_m':3,'camera':'forward_d435i'},ex)
    assert ex.calls==[('orbit_target',{'target_id':'known','standoff_m':3,'camera':'forward_d435i'})]


def test_failed_orientation_cannot_continue_to_object_motion():
    ex=Executor();ex.failure='FAILED'
    result=checked('approach_target',{'destination':{'type':'visible_object','visual_query':'visible structure',
        'view_heading_deg':0},'object_position':'standoff','standoff_m':4},ex)
    assert result['status']=='FAILED' and len(ex.calls)==1


@pytest.mark.parametrize('operation',['goto','orbit'])
def test_home_relative_coordinates_are_transformed_and_origin_bound(operation):
    ex=Executor();args={'destination':{'type':'coordinates','coordinate_frame':'home_relative_ned',
                                     'north_m':2,'east_m':-3},'altitude_agl_m':5}
    if operation=='orbit':args.update(radius_m=4,camera='none')
    checked(operation,args,ex)
    _,actual=ex.calls[0]
    assert actual['north_m' if operation=='goto' else 'center_north_m']==12
    assert actual['east_m' if operation=='goto' else 'center_east_m']==17


def test_local_coordinates_with_stale_origin_never_move():
    ex=Executor()
    with pytest.raises(ValueError,match='origin'):
        checked('goto',{'destination':{'type':'coordinates','coordinate_frame':'local_ned',
            'origin_id':'old','north_m':1,'east_m':2},'altitude_agl_m':5},ex)
    assert not ex.calls


def test_unknown_named_waypoint_requests_coordinates_without_movement(monkeypatch):
    monkeypatch.delenv('ICARUS_WAYPOINTS_FILE',raising=False)
    ex=Executor()
    result=checked('goto',{'destination':{'type':'named_waypoint','waypoint_name':'operator point'}},ex)
    assert result['requires_user_choice'] and 'operator point' in result['question']
    assert not ex.calls


def test_operator_catalogue_waypoint_is_data_not_a_mission_alias(tmp_path,monkeypatch):
    p=tmp_path/'points.json';p.write_text(json.dumps({'coordinate_frame':'home_relative_ned',
        'entries':[{'name':'Survey A','north_m':2,'east_m':-3,'altitude_agl_m':5,'heading_deg':123}]}))
    monkeypatch.setenv('ICARUS_WAYPOINTS_FILE',str(p))
    ex=Executor();result=checked('goto',{'destination':{'type':'named_waypoint','waypoint_name':'survey a'}},ex)
    assert ex.calls==[('goto',{'north_m':12,'east_m':17,'altitude_agl_m':5,'heading_deg':123})]
    assert result['evidence']['destination']['catalogue_sha256']==waypoint_catalogue()['sha256']


@pytest.mark.parametrize('entries',[[{'name':'a','north_m':float('nan'),'east_m':0}],
    [{'name':'a','north_m':0,'east_m':0},{'name':'A','north_m':1,'east_m':1}]])
def test_invalid_catalogue_remains_unavailable_not_guessed(tmp_path,monkeypatch,entries):
    p=tmp_path/'points.json';p.write_text(json.dumps({'coordinate_frame':'home_relative_ned','entries':entries}))
    monkeypatch.setenv('ICARUS_WAYPOINTS_FILE',str(p))
    assert waypoint_catalogue()['available'] is False


@pytest.mark.parametrize('args',[
    {'destination':{'type':'visible_object','visual_query':'object'},'object_position':'standoff','standoff_m':4},
    {'destination':{'type':'coordinates','coordinate_frame':'local_ned','north_m':1,'east_m':2},'altitude_agl_m':5},
    {'destination':{'type':'home','north_m':1},'altitude_agl_m':5},
    {'destination':{'type':'registered_target','target_id':'known'},'north_m':1,'east_m':2,'object_position':'above'},
])
def test_reference_kinds_cannot_mix_fields_or_omit_required_measurements(args):
    with pytest.raises(ValueError):validate_call('goto',args,tools=decision_tools({'targets':[{'target_id':'known'}]}))


def test_real_executor_accepts_typed_coordinates_and_keeps_legacy_api_compatible():
    client=FakeClient();executor=ToolExecutor(client)
    result=executor.execute('goto',{'destination':{'type':'coordinates','coordinate_frame':'local_ned',
        'origin_id':'ekf-origin','north_m':1,'east_m':2},'altitude_agl_m':3})
    assert result['status']=='SUCCEEDED'
    result=executor.execute('goto',{'north_m':2,'east_m':3,'altitude_agl_m':4})
    assert result['status']=='SUCCEEDED' and len(client.sent)==2


def test_object_orbit_explicit_face_center_selects_tracking_controller():
    ex=Executor()
    checked('orbit',{'destination':{'type':'registered_target','target_id':'known'},
        'standoff_m':3,'camera':'forward_d435i','face_center':True},ex)
    assert ex.calls==[('orbit_target',{'target_id':'known','standoff_m':3,'camera':'forward_d435i'})]
    with pytest.raises(ValueError):
        checked('orbit',{'destination':{'type':'registered_target','target_id':'known'},
            'standoff_m':3,'camera':'forward_d435i','face_center':False},ex)


def test_observation_pose_is_not_advertised_as_an_orbit_center(tmp_path,monkeypatch):
    p=tmp_path/'points.json';p.write_text(json.dumps({'coordinate_frame':'home_relative_ned',
        'entries':[{'name':'View A','north_m':2,'east_m':3,'role':'observation_pose'}]}))
    monkeypatch.setenv('ICARUS_WAYPOINTS_FILE',str(p))
    ctx={'registered_waypoints':waypoint_catalogue()}
    args={'destination':{'type':'named_waypoint','waypoint_name':'View A'},'radius_m':4,
          'altitude_agl_m':5,'camera':'none'}
    with pytest.raises(ValueError):validate_call('orbit',args,tools=decision_tools(ctx))
    ex=Executor()
    with pytest.raises(ValueError,match='not an orbit center'):execute_destination(ex,'orbit',args)
    assert not ex.calls
    result=execute_destination(ex,'goto',{'destination':args['destination']})
    receipt=result['evidence']['destination']
    assert receipt['reference_role']=='observation_pose'
    assert receipt['object_relative_position_established'] is False


def test_native_overflight_dispatches_existing_measured_controller(monkeypatch):
    from python.dcm.target_motion import TargetMotion
    seen=[]
    def overfly(self,**args):
        seen.append(args);return {'status':'SUCCEEDED','evidence':{'measured_top':True}}
    monkeypatch.setattr(TargetMotion,'overfly_target',overfly)
    client=FakeClient()
    result=ToolExecutor(client).execute('overfly_target',
        {'destination':{'type':'registered_target','target_id':'known'},'clearance_above_m':2})
    assert result['status']=='SUCCEEDED' and seen==[{'target_id':'known','clearance_above_m':2}]


def test_landing_after_landed_return_home_never_rearms_or_sends_another_command():
    client=FakeClient();ex=ToolExecutor(client)
    assert ex.execute('return_home',{})['status']=='SUCCEEDED'
    def measured_state():
        client.vehicle.observed_at_unix_ms = int(time.time() * 1000)
        client.vehicle.attitude_received_at_unix_ms = client.vehicle.observed_at_unix_ms
        client.vehicle.velocity_ned_mps.z = 0
        return client.vehicle
    client.state = measured_state
    sent=len(client.sent)
    result=ex.execute('land',{})
    assert result['status']=='SUCCEEDED' and result['evidence']['end_state_already_satisfied']
    assert len(client.sent)==sent and not client.vehicle.armed


@pytest.mark.parametrize('condition',['stale','armed','airborne'])
def test_landing_cannot_infer_an_already_satisfied_end_state(condition):
    client=FakeClient()
    if condition=='stale':client.vehicle.observed_at_unix_ms=1
    if condition=='armed':client.vehicle.armed=True
    if condition=='airborne':client.vehicle.landed=False
    result = ToolExecutor(client).execute('land',{})
    if condition == 'stale':
        assert result['status'] == 'FAILED'
        assert not client.sent  # stale telemetry must not trigger a mode change on disarmed aircraft
    else:
        assert [n for n,_ in client.sent]==['Land']


@pytest.mark.parametrize('height',[2,5,8])
def test_airborne_altitude_request_preserves_xy_without_arming_or_launch_reset(height):
    client=FakeClient();client.vehicle.landed=False;client.vehicle.armed=True
    client.vehicle.relative_altitude_m=3
    client.vehicle.local_position_ned.north_m=7;client.vehicle.local_position_ned.east_m=-4
    client._takeoff_reference={'sentinel':'original measured launch'}
    result=ToolExecutor(client).execute('takeoff',{'target_altitude_agl_m':height})
    assert result['status']=='SUCCEEDED'
    assert [n for n,_ in client.sent]==['Goto']
    request=client.sent[0][1]
    assert request.destination.local_ned.north_m==7 and request.destination.local_ned.east_m==-4
    assert request.destination.local_ned.altitude_above_home_m==height
    assert client._takeoff_reference=={'sentinel':'original measured launch'}


def test_airborne_altitude_cannot_drop_an_explicit_rate_requirement():
    client=FakeClient();client.vehicle.landed=False;client.vehicle.armed=True
    result=ToolExecutor(client).execute('takeoff',{'target_altitude_agl_m':5,'maximum_climb_rate_mps':1})
    assert result['requires_user_choice'] and not client.sent


def test_airborne_altitude_cannot_invent_an_origin():
    client=FakeClient();client.vehicle.landed=False;client.vehicle.armed=True
    client.vehicle.local_position_ned.origin_id=''
    result=ToolExecutor(client).execute('takeoff',{'target_altitude_agl_m':5})
    assert result['status']=='FAILED' and not client.sent


def test_airborne_tool_catalogue_only_offers_supported_altitude_options():
    tools=decision_tools({'state':{'landed':False,'armed':True}})
    schema=next(t['function']['parameters'] for t in tools if t['function']['name']=='takeoff')
    assert 'maximum_climb_rate_mps' not in schema['properties']
    validate_call('takeoff',{'target_altitude_agl_m':5},tools=tools)
    with pytest.raises(ValueError):validate_call('takeoff',
        {'target_altitude_agl_m':5,'maximum_climb_rate_mps':1},tools=tools)


def test_airborne_batch_cannot_reintroduce_unsupported_departure_rate():
    tools=decision_tools({'state':{'landed':False,'armed':True}})
    args={'actions':[{'name':'takeoff','arguments':{'target_altitude_agl_m':5,'maximum_climb_rate_mps':1}}]}
    with pytest.raises(ValueError):validate_call('execute_sequence',args,tools=tools)

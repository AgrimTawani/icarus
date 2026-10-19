"""Reference types choose measured controllers; no mission-specific aliases."""
import copy
import json
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
    return execute_destination(ex or Executor(),operation,args)


def test_visible_destination_orients_then_localizes_through_existing_controller():
    ex=Executor()
    result=checked('goto',{'destination':{'type':'visible_object','visual_query':'visible structure',
                 'view_heading_deg':0},'object_position':'standoff','standoff_m':4},ex)
    assert ex.calls==[('face_heading',{'heading_deg':0}),
                     ('approach_target',{'visual_query':'visible structure','standoff_m':4})]
    assert result['evidence']['destination']['controller']=='approach_target'
    assert result['evidence']['measured']


def test_above_object_uses_overflight_controller_not_altitude_only_goto():
    ex=Executor()
    checked('goto',{'destination':{'type':'registered_target','target_id':'known'},
                    'object_position':'above','clearance_above_m':2},ex)
    assert ex.calls==[('overfly_target',{'target_id':'known','clearance_above_m':2})]


def test_object_orbit_uses_surface_controller_not_coordinate_radius():
    ex=Executor()
    checked('orbit',{'destination':{'type':'registered_target','target_id':'known'},
                      'standoff_m':3,'camera':'forward_d435i'},ex)
    assert ex.calls==[('orbit_target',{'target_id':'known','standoff_m':3,'camera':'forward_d435i'})]


def test_failed_orientation_cannot_continue_to_object_motion():
    ex=Executor();ex.failure='FAILED'
    result=checked('goto',{'destination':{'type':'visible_object','visual_query':'visible structure',
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
        'entries':[{'name':'Survey A','north_m':2,'east_m':-3,'altitude_agl_m':5}]}))
    monkeypatch.setenv('ICARUS_WAYPOINTS_FILE',str(p))
    ex=Executor();result=checked('goto',{'destination':{'type':'named_waypoint','waypoint_name':'survey a'}},ex)
    assert ex.calls==[('goto',{'north_m':12,'east_m':17,'altitude_agl_m':5})]
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

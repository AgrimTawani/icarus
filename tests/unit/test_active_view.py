"""Active camera views delegate compass geometry without implicit launch."""
import math
import pytest
from python.dcm.viewing import resolve_view,COMPASS_HEADINGS
from python.dcm.agent_tools import ToolExecutor,decision_tools,validate_call
from python.perception.vlm import VlmInferenceError
from test_direct_agent import FakeClient,context


@pytest.mark.parametrize('direction,expected',list(COMPASS_HEADINGS.items()))
def test_compass_labels_resolve_without_model_heading_math(direction,expected):
    assert resolve_view(direction,{})==expected


@pytest.mark.parametrize('bad',[True,None,'unknown',-1,360,float('nan')])
def test_invalid_view_never_becomes_a_heading(bad):
    with pytest.raises(ValueError):resolve_view(bad,{})


def camera_client(tmp_path,*,landed=False,yaw=90):
    client=FakeClient();client.vehicle.landed=landed;client.vehicle.armed=not landed
    client.vehicle.relative_altitude_m=0 if landed else 3
    client.vehicle.attitude.yaw_rad=math.radians(yaw)
    captures=[]
    def capture(**kwargs):
        captures.append(len(client.sent));return {'directory':str(tmp_path/'capture')}
    client.capture_view=capture
    client.inspect_scene=lambda *args,**kwargs:{'answerable':True,'answer':'Observed surface.','validity':'valid'}
    return client,captures


def test_requested_scene_view_turns_before_capture_in_one_tool_call(tmp_path):
    client,captures=camera_client(tmp_path)
    result=ToolExecutor(client).execute('inspect_scene',{'question':'Describe this view.','view':'north'})
    assert result['status']=='SUCCEEDED' and captures==[1]
    assert [name for name,_ in client.sent]==['Yaw']
    assert client.sent[0][1].relative_angle_deg==pytest.approx(-90)
    assert math.degrees(client.vehicle.attitude.yaw_rad)%360==pytest.approx(0)
    assert result['evidence']['motion_command_issued'] is True
    assert result['evidence']['requested_view_heading_deg']==0


@pytest.mark.parametrize('view,capture_expected',[('north',False),('current',True)])
def test_ground_observation_never_arms_or_turns(tmp_path,view,capture_expected):
    client,captures=camera_client(tmp_path,landed=True)
    result=ToolExecutor(client).execute('inspect_scene',{'question':'Describe this view.','view':view})
    assert not client.sent and bool(captures)==capture_expected
    assert result['status']==('SUCCEEDED' if capture_expected else 'NEEDS_CLARIFICATION')


def test_failed_capture_after_turn_invalidates_old_view_counts(tmp_path):
    client,_=camera_client(tmp_path)
    def fail(**kwargs):raise VlmInferenceError('capture_failed','Camera acquisition failed')
    client.capture_view=fail
    result=ToolExecutor(client).execute('inspect_scene',{'question':'Describe this view.','view':'north'})
    assert result['status']=='FAILED' and result['error_code']=='capture_failed'
    assert result['evidence']['motion_command_issued'] is True


def test_face_heading_native_and_legacy_numeric_forms_share_controller(tmp_path):
    client,_=camera_client(tmp_path)
    result=ToolExecutor(client).execute('face_heading',{'view':'west'})
    assert result['status']=='SUCCEEDED'
    assert client.sent[-1][0]=='Yaw'
    assert abs(client.sent[-1][1].relative_angle_deg)==pytest.approx(180)
    assert math.degrees(client.vehicle.attitude.yaw_rad)%360==pytest.approx(270)
    tools=decision_tools(context(client))
    validate_call('face_heading',{'heading_deg':0},tools=tools)


def test_detect_can_aim_and_capture_without_a_separate_model_call(tmp_path):
    client,_=camera_client(tmp_path)
    def detect(**kwargs):
        assert len(client.sent)==1
        return {'camera':'forward_d435i','counts':{'person':0},'captured_at_unix_ms':1000}
    client.detect=detect
    result=ToolExecutor(client).execute('detect',{'classes':['person'],'view':'north'})
    assert result['status']=='SUCCEEDED' and result['evidence']['counts']=={'person':0}
    assert result['evidence']['motion_command_issued'] is True

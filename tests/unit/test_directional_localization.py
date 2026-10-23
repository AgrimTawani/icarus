"""Directional image acquisition owns orientation, never implicit departure."""
import pytest
from python.dcm.agent_tools import ToolExecutor, decision_tools, validate_call
from test_direct_agent import FakeClient, context


def test_airborne_orientation_finishes_before_localization():
    client=FakeClient();client.vehicle.armed=True;client.vehicle.landed=False
    client.vehicle.relative_altitude_m=3
    captures=[]
    client.localize_target=lambda **args: captures.append((len(client.sent),args)) or {'found':True}
    result=ToolExecutor(client).execute('localize_target',{
        'visual_query':'visible structure','view_heading_deg':90})
    assert result['status']=='SUCCEEDED'
    assert captures==[(1,{'description':'visible structure'})]
    assert [name for name,_ in client.sent]==['Yaw']
    assert result['evidence']['requested_view_heading_deg']==90
    assert result['evidence']['motion_command_issued'] is True


def test_failed_orientation_never_captures():
    client=FakeClient();client.vehicle.armed=True;client.vehicle.landed=False
    client.fail='Yaw'
    client.localize_target=lambda **args: pytest.fail('capture after failed turn')
    result=ToolExecutor(client).execute('localize_target',{
        'visual_query':'visible structure','view_heading_deg':90})
    assert result['status']!='SUCCEEDED'
    assert result['evidence']['motion_command_issued'] is True


@pytest.mark.parametrize('heading,expected',[(90,'NEEDS_CLARIFICATION'),(0,'SUCCEEDED')])
def test_ground_camera_request_never_arms_or_departs(heading,expected):
    client=FakeClient();captures=[]
    client.localize_target=lambda **args: captures.append(args) or {'found':False}
    result=ToolExecutor(client).execute('localize_target',{
        'visual_query':'visible structure','view_heading_deg':heading})
    assert result['status']==expected and not client.sent
    assert bool(captures)==(heading==0)


def test_native_localization_requires_direction():
    tools=decision_tools(context(FakeClient()))
    with pytest.raises(ValueError):
        validate_call('localize_target',{'visual_query':'structure'},tools=tools)
    validate_call('localize_target',{'visual_query':'structure','view_heading_deg':0},tools=tools)


def test_failed_inference_after_turn_invalidates_old_counts_and_keeps_error_code():
    from python.perception.vlm import VlmInferenceError
    client=FakeClient();client.vehicle.armed=True;client.vehicle.landed=False
    client.vehicle.relative_altitude_m=3
    def fail(**kwargs):raise VlmInferenceError('invalid_pixel_box','Bad measured image box')
    client.localize_target=fail
    result=ToolExecutor(client).execute('localize_target',{
        'visual_query':'visible structure','view_heading_deg':90})
    assert result['status']=='FAILED' and result['error_code']=='invalid_pixel_box'
    assert result['evidence']['motion_command_issued'] is True
    assert [name for name,_ in client.sent]==['Yaw']

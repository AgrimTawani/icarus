"""Optional schema disclosure is explicit, retained and never a flight action."""
from types import SimpleNamespace
import pytest
from python.dcm.agent_tools import decision_tools, validate_call, ToolExecutor, OPTIONAL_TOOLS
from python.dcm.fly import AgentSession


def names(tools):
    return {t['function']['name'] for t in tools}


def test_default_catalogue_is_smaller_and_keeps_all_core_actions():
    import json
    full = decision_tools({})
    core = decision_tools({}, loaded_tools=set())
    assert names(full) - names(core) == set(OPTIONAL_TOOLS)
    assert {'inspect_scene', 'detect', 'takeoff', 'face_heading', 'return_home', 'land', 'overfly_target', 'approach_target'} <= names(core)
    assert len(json.dumps(core)) < len(json.dumps(full)) * .55
    assert decision_tools({}) == full  # no mutation across sessions


def test_loading_only_changes_the_requested_session_catalogue():
    session = AgentSession('operator request')
    executor = ToolExecutor(SimpleNamespace(), session=session)
    result = executor.execute('load_tools', {'names': ['scan_heading', 'fly_pattern']})
    assert result['status'] == 'SUCCEEDED'
    assert session.actions == []
    offered = decision_tools({}, loaded_tools=session.loaded_tools)
    assert 'scan_heading' in names(offered)
    assert 'fly_pattern' in names(offered)
    assert 'scan_area' not in names(offered)
    assert AgentSession('another').loaded_tools == set()
    with pytest.raises(ValueError, match='unavailable'):
        validate_call('scan_area', {}, tools=offered)


def test_completed_and_pending_workflows_keep_their_own_resume_contract():
    tools = decision_tools({}, {'angular-1': {'completed': False}},
                           {'area-1': {'completed': True}}, loaded_tools=set())
    assert 'resume_heading_scan' in names(tools)
    assert 'resume_scan' not in names(tools)
    assert 'scan_heading' not in names(tools)


def test_loader_rejects_unknown_names_before_any_client_access():
    with pytest.raises(ValueError):
        ToolExecutor(SimpleNamespace(), session=AgentSession('request')).execute(
            'load_tools', {'names': ['invented_autonomy']})


def test_fly_loop_discloses_loaded_tool_on_next_decision_without_flight():
    from test_direct_agent import FakeClient, call, context
    from python.dcm.fly import fly_mission
    catalogs = []
    class Runtime:
        def decide(self, messages, tools):
            catalogs.append(names(tools))
            return (call('load_tools', names=['localize_target']) if len(catalogs)==1
                    else call('finish', outcome='incomplete', message='Discovery test only.'))
    client = FakeClient()
    session = AgentSession('Read-only discovery test.')
    result = fly_mission(client, Runtime(), session.mission, session=session,
                         context_provider=context, echo=lambda *args: None)
    assert result['status']=='incomplete'
    assert 'localize_target' not in catalogs[0]
    assert 'localize_target' in catalogs[1]
    assert 'orbit' not in catalogs[1]
    assert not client.sent
    assert session.actions[0]['action']=='load_tools'


def test_explicit_legacy_catalogue_can_still_require_diagnostic_memory():
    tools = decision_tools({}, loaded_tools=set(), require_progress=True)
    for name, args in [('takeoff', {'target_altitude_agl_m':3}),
                       ('finish', {'outcome':'incomplete','message':'No action.'})]:
        with pytest.raises(ValueError, match='mission_progress'):
            validate_call(name, args, tools=tools)
        with pytest.raises(ValueError):
            validate_call(name, {**args,'mission_progress':''}, tools=tools)
        validate_call(name, {**args,'mission_progress':'Preserve unresolved operator requirements.'}, tools=tools)
    for tool in tools:
        assert next(iter(tool['function']['parameters']['properties']))=='mission_progress'


def test_loaded_waypoint_handles_expose_navigation_without_second_discovery():
    context = {'registered_waypoints': {'loaded': True, 'entries': [
        {'name': 'configured observation pose', 'role': 'observation_pose'}]}}
    offered = decision_tools(context, loaded_tools=set())
    assert 'goto' in names(offered)
    validate_call('goto', {'destination': {'type': 'named_waypoint',
        'waypoint_name': 'configured observation pose'}}, tools=offered)
    assert 'orbit' not in names(offered)
    assert 'goto' not in names(decision_tools({}, loaded_tools=set()))


def test_live_catalogue_does_not_advertise_self_written_plan_notes():
    for tool in decision_tools({}, loaded_tools=set()):
        assert 'mission_progress' not in tool['function']['parameters']['properties']
    # Old recorded diagnostic metadata is harmless and not a physical argument.
    validate_call('takeoff', {'target_altitude_agl_m': 3, 'mission_progress': 'Old note'},
                  tools=decision_tools({}, loaded_tools=set()))

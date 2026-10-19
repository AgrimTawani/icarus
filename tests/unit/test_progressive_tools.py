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
    assert {'overfly_target', 'approach_target', 'inspect_scene', 'orbit', 'takeoff'} <= names(core)
    assert len(json.dumps(core)) < len(json.dumps(full)) * .8
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

"""The mission loop: one tool per step, brief results, no hidden extra work."""
import json

from python.dcm.agent_tools import AGENT_TOOLS, decision_tools
from python.dcm.fly import AgentSession, brief, drop_oldest_exchange, fly_mission
from test_direct_agent import FakeClient, Runtime, call, context, run


def airborne():
    client = FakeClient()
    client.vehicle.landed = False
    client.vehicle.armed = True
    client.vehicle.relative_altitude_m = 5
    return client


def test_conversation_is_append_only_so_the_model_cache_holds():
    runtime = Runtime(call('yaw', angle_deg=30, direction='clockwise'),
                      call('yaw', angle_deg=30, direction='clockwise'),
                      call('finish', outcome='completed', message='Turned twice.'))
    assert run(airborne(), runtime)['status'] == 'completed'
    for earlier, later in zip(runtime.inputs, runtime.inputs[1:]):
        assert later[:len(earlier)] == earlier


def test_each_result_is_brief_and_carries_current_state():
    runtime = Runtime(call('yaw', angle_deg=30, direction='clockwise'),
                      call('finish', outcome='completed', message='Turned.'))
    run(airborne(), runtime)
    result = runtime.inputs[1][-1]
    assert result['role'] == 'tool'
    summary, state = result['content'].split('\n')
    assert json.loads(summary)['status'] == 'SUCCEEDED'
    assert state.startswith('Vehicle state:') and 'altitude 5.0 m above home' in state
    assert len(result['content']) < 400


def test_no_perception_runs_unless_the_model_asks_for_it():
    client = airborne()
    captures = []
    client.capture_view = lambda **kwargs: captures.append(kwargs)
    client.detect = lambda **kwargs: captures.append(kwargs)
    runtime = Runtime(call('yaw', angle_deg=30, direction='clockwise'),
                      call('finish', outcome='completed', message='Turned.'))
    run(client, runtime)
    assert captures == []
    assert [name for name, _ in client.sent] == ['Yaw']


def test_brief_keeps_measured_facts_and_drops_bulk():
    result = {'status': 'SUCCEEDED', 'message': 'Measured target surface standoff reached.',
              'state': {'big': 'telemetry'},
              'evidence': {'target_id': 't1', 'final_surface_range_m': 5.0, 'provenance': [{'x': 1}] * 50}}
    assert brief(result) == {'status': 'SUCCEEDED', 'message': 'Measured target surface standoff reached.',
                             'evidence': {'target_id': 't1', 'final_surface_range_m': 5.0}}


def test_context_overflow_drops_the_oldest_exchange_but_keeps_the_request():
    session = AgentSession('Turn twice.')
    for index in range(2):
        session.messages += [{'role': 'assistant', 'tool_calls': [{'id': str(index)}]},
                             {'role': 'tool', 'content': str(index)}]
    assert drop_oldest_exchange(session.messages)
    assert [m['role'] for m in session.messages] == ['system', 'user', 'assistant', 'tool']
    assert session.messages[-1]['content'] == '1'
    assert not drop_oldest_exchange(session.messages[:2])


def test_catalogue_is_fixed_and_has_no_meta_tools():
    names = [tool['function']['name'] for tool in decision_tools({})]
    assert sorted(names) == sorted(AGENT_TOOLS)
    for tool in decision_tools({}):
        assert 'mission_complete' not in json.dumps(tool['function']['parameters'])


def test_cancel_ends_the_mission():
    client = airborne()
    result = run(client, Runtime(call('cancel')))
    assert result['status'] == 'cancelled'


def test_three_consecutive_failures_stop_with_the_reason():
    client = airborne()
    client.fail = 'Yaw'
    turn = call('yaw', angle_deg=30, direction='clockwise')
    result = run(client, Runtime(turn, turn, turn, call('finish', outcome='completed', message='x')))
    assert result['status'] == 'incomplete'
    assert result['message'].startswith('Stopped after 3 unsuccessful calls in a row.')


def test_the_same_failing_call_stops_even_with_holds_in_between():
    """Live run: approach_target on a lost target failed eight times with a
    successful hold between each; the consecutive-failure stop never fired."""
    client = airborne()
    client.fail = 'Yaw'
    turn = call('yaw', angle_deg=30, direction='clockwise')
    pause = call('hold', duration_s=1)
    result = run(client, Runtime(turn, pause, turn, pause, turn, pause, turn,
                                 call('finish', outcome='completed', message='x')))
    assert result['status'] == 'incomplete'
    assert 'same call failed 3 times' in result['message']
    assert [name for name, _ in client.sent].count('Yaw') == 3


def test_every_advertised_tool_has_a_property_for_the_grammar_compiler():
    # The pinned llama-server rejects grammars for objects without properties.
    for tool in decision_tools({}):
        parameters = tool['function']['parameters']
        assert parameters.get('properties') or parameters.get('oneOf'), tool['function']['name']


def test_grammar_padding_never_reaches_the_executor():
    client = airborne()
    result = run(client, Runtime(call('land', reason='mission over'),
                                 call('finish', outcome='completed', message='Landed.')))
    assert result['status'] == 'completed'
    assert result['executed'][0]['arguments'] == {}
    assert [name for name, _ in client.sent] == ['Land']


def test_completed_is_checked_once_against_unresolved_failures():
    from python.dcm.fly import unresolved_failures
    client = airborne()
    client.fail = 'Yaw'
    runtime = Runtime(call('yaw', angle_deg=30, direction='clockwise'),
                      call('finish', outcome='completed', message='Turned.'),
                      call('finish', outcome='incomplete', message='The turn did not run.'))
    result = run(client, runtime)
    assert result['status'] == 'incomplete' and result['decisions'] == 3
    check = json.loads(runtime.inputs[2][-1]['content'].split('\n')[0])
    assert check['status'] == 'REJECTED' and 'yaw (' in check['message']
    assert unresolved_failures(result['executed']) == [check['message'].split('latest call of ')[1].split(' did not')[0]]


def test_a_retried_step_that_succeeded_is_resolved_and_a_repeated_claim_is_delivered():
    from python.dcm.fly import unresolved_failures
    ok, failed = {'status': 'SUCCEEDED'}, {'status': 'FAILED', 'message': 'route stale'}
    assert unresolved_failures([{'action': 'survey_count', 'result': failed},
                                {'action': 'survey_count', 'result': ok}]) == []
    assert unresolved_failures([{'action': 'land', 'result': failed, 'operator': True}]) == []
    assert unresolved_failures([{'action': 'inspect_scene', 'result': {'status': 'VIEWPOINT_REQUIRED'}}]) == []
    assert unresolved_failures([{'action': 'survey_count', 'result': failed}]) == ['survey_count (FAILED: route stale)']
    runtime = Runtime(call('yaw', angle_deg=30, direction='clockwise'),
                      call('finish', outcome='completed', message='Turned.'),
                      call('finish', outcome='completed', message='Turned, I insist.'))
    client = airborne()
    client.fail = 'Yaw'
    result = run(client, runtime)
    assert result['status'] == 'completed' and result['unresolved_failures'][0].startswith('yaw (')


def test_completed_without_failures_needs_no_extra_decision():
    runtime = Runtime(call('yaw', angle_deg=30, direction='clockwise'),
                      call('finish', outcome='completed', message='Turned.'))
    result = run(airborne(), runtime)
    assert result['decisions'] == 2 and 'unresolved_failures' not in result


def test_failed_step_partial_evidence_says_the_path_was_not_completed():
    summary = brief({'status': 'FAILED', 'message': 'route stale',
                     'evidence': {'count': 9, 'path_completed': False, 'objects': []}})
    assert summary['evidence']['path_completed'] is False

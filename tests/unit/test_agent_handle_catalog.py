"""Opaque tool handles are supplied as exact choices, never guessed/repaired."""
import copy

from python.dcm.agent_tools import TOOLS, decision_tools, validate_call
from python.dcm.fly import AgentSession, fly_mission
from test_direct_agent import FakeClient, call, context


def prop(catalog, tool, field):
    schema = next(item['function']['parameters'] for item in catalog
                  if item['function']['name'] == tool)
    def find(value):
        if isinstance(value, dict):
            if field in value.get('properties', {}):
                return value['properties'][field]
            for child in value.values():
                found = find(child)
                if found is not None: return found
        elif isinstance(value, list):
            for child in value:
                found = find(child)
                if found is not None: return found
    result = find(schema)
    assert result is not None, (tool, field)
    return result



def test_object_standoff_default_is_context_data_and_does_not_override_explicit_values():
    catalog = decision_tools({'defaults': {'object_standoff_m': 4.5}})
    for name in ('approach_target', 'orbit'):
        assert prop(catalog, name, 'standoff_m')['default'] == 4.5
        validate_call(name, {'destination': {'type':'visible_object', 'visual_query':'visible structure', 'view_heading_deg':0}, 'standoff_m':3, **({'camera':'none'} if name=='orbit' else {'object_position':'standoff'})}, tools=catalog)
    assert 'default' not in prop(decision_tools({}), 'orbit', 'standoff_m')


def test_registered_target_handles_include_stale_and_update_without_mutating_static():
    before = copy.deepcopy(TOOLS)
    ctx = {'targets': [{'target_id': 'target_long_abc', 'fresh': False},
                       {'target_id': 'target_other_def', 'fresh': True}]}
    first = decision_tools(ctx)
    for tool in ('approach_target', 'orbit'):
        assert prop(first, tool, 'target_id')['enum'] == ['target_long_abc', 'target_other_def']
    ctx['targets'].append({'target_id': 'target_new_xyz'})
    second = decision_tools(ctx)
    assert 'target_new_xyz' in prop(second, 'orbit', 'target_id')['enum']
    assert 'target_new_xyz' not in prop(first, 'orbit', 'target_id')['enum']
    assert TOOLS == before
    # Call validation remains typed; the backend, not this schema helper,
    # decides whether a referenced object is usable or requires new evidence.
    validate_call('orbit_target', {'target_id': 'unknown', 'standoff_m': 3})


def test_workflow_handle_choices_are_separate_and_missing_ids_hide_dependent_tools():
    catalog = decision_tools({}, {'turn_123': {}}, {'area_456': {}})
    assert prop(catalog, 'resume_heading_scan', 'scan_id')['enum'] == ['turn_123']
    assert prop(catalog, 'resume_scan', 'scan_id')['enum'] == ['area_456']
    empty = decision_tools({})
    names = {tool['function']['name'] for tool in empty}
    assert not names.intersection({'resume_heading_scan', 'resume_scan'})
    for name in ('approach_target', 'orbit'):
        schema = next(t['function']['parameters'] for t in empty if t['function']['name'] == name)
        variants = schema['properties']['destination']['oneOf']
        assert 'registered_target' not in [v['properties']['type']['const'] for v in variants]
        visible = next(v for v in variants if v['properties']['type']['const']=='visible_object')
        assert {'visual_query','view'} <= set(visible['required'])
    assert 'orbit_target' not in names and 'approach_target' in names
    assert 'overfly_target' in names
    assert {'localize_target', 'orbit', 'goto', 'face_heading', 'scan_area', 'scan_heading'} <= names
    assert 'enum' not in prop(TOOLS, 'resume_heading_scan', 'scan_id')
    assert 'enum' not in prop(TOOLS, 'orbit_target', 'target_id')


def test_live_decisions_rebind_new_targets_and_use_only_current_mission_workflows():
    client = FakeClient()
    client._angular_scans = {'previous_turn': {}}
    client._scan_workflows = {'previous_area': {}}
    session = AgentSession('Hold and report.')
    # Handle-contract tests exercise already-discovered tools.
    session.loaded_tools.update({"goto", "orbit", "localize_target", "overfly_target", "approach_target", "execute_sequence"})
    session.angular_scans = {'current_turn': {'completed': False, 'phase': 'observe',
                                            'swept_deg': 30, 'observations': []}}
    session.area_scans = {'current_area': {'completed': False}}
    catalogs = []
    calls = 0

    def capture(c):
        nonlocal calls
        calls += 1
        return {**context(c), 'targets': [{'target_id': 'target_' + str(calls)}]}

    class Runtime:
        def decide(self, messages, tools):
            catalogs.append(tools)
            return (call('hold', duration_s=1) if len(catalogs) == 1 else
                    call('finish', outcome='completed', message='Held.'))

    result = fly_mission(client, Runtime(), session.mission, session=session,
                         context_provider=capture, echo=lambda *args: None)
    assert result['status'] == 'completed'
    assert prop(catalogs[0], 'orbit', 'target_id')['enum'] == ['target_1']
    assert prop(catalogs[1], 'orbit', 'target_id')['enum'] == ['target_2']
    for catalog in catalogs:
        assert prop(catalog, 'resume_heading_scan', 'scan_id')['enum'] == ['current_turn']
        assert prop(catalog, 'resume_scan', 'scan_id')['enum'] == ['current_area']


def test_completed_workflows_disappear_but_paused_and_open_ended_remain():
    angular = {'done': {'completed': True},
               'paused': {'completed': False, 'phase': 'observe'},
               'open': {'completed': False, 'args': {'repeat_until_match': True}}}
    area = {'done_area': {'completed': True}, 'paused_area': {'completed': False}}
    catalog = decision_tools({}, angular, area)
    assert prop(catalog, 'resume_heading_scan', 'scan_id')['enum'] == ['open', 'paused']
    assert prop(catalog, 'resume_scan', 'scan_id')['enum'] == ['paused_area']
    angular['paused']['completed'] = angular['open']['completed'] = True
    area['paused_area']['completed'] = True
    next_catalog = decision_tools({}, angular, area)
    assert not {'resume_heading_scan', 'resume_scan'}.intersection(
        tool['function']['name'] for tool in next_catalog)
    assert prop(catalog, 'resume_heading_scan', 'scan_id')['enum'] == ['open', 'paused']
    assert 'enum' not in prop(TOOLS, 'resume_heading_scan', 'scan_id')


def test_truncation_retry_uses_same_handle_bindings():
    from python.dcm.llama_runtime import IncompleteModelResponse
    client = FakeClient()
    catalogs = []

    class Runtime:
        def decide(self, messages, tools, concise=False):
            catalogs.append(tools)
            if not concise:
                raise IncompleteModelResponse('truncated')
            return call('finish', outcome='incomplete', message='No flight needed.')

    session = AgentSession('Report.')
    session.loaded_tools.add('orbit')
    fly_mission(client, Runtime(), 'Report.', session=session,
                context_provider=lambda c: {**context(c), 'targets': [{'target_id': 'exact_123'}]},
                echo=lambda *args: None)
    assert len(catalogs) == 2 and catalogs[0] == catalogs[1]
    assert prop(catalogs[1], 'orbit', 'target_id')['enum'] == ['exact_123']


def test_server_motion_limits_apply_to_nested_arguments_without_affecting_other_numbers():
    before = copy.deepcopy(TOOLS)
    catalog = decision_tools({'motion_limits': {
        'maximum_climb_speed_mps': 1.25, 'maximum_horizontal_speed_mps': 4.5}})
    assert prop(catalog, 'takeoff', 'maximum_climb_rate_mps')['maximum'] == 1.25
    assert prop(catalog, 'goto', 'maximum_ground_speed_mps')['maximum'] == 4.5
    assert 'maximum' not in prop(catalog, 'goto', 'north_m')
    points = prop(catalog, 'execute_route', 'points')['items']['properties']
    assert points['maximum_ground_speed_mps']['maximum'] == 4.5
    branches = prop(catalog, 'execute_sequence', 'actions')['items']['oneOf']
    takeoff = next(b for b in branches if b['properties']['name']['const'] == 'takeoff')
    assert takeoff['properties']['arguments']['properties']['maximum_climb_rate_mps']['maximum'] == 1.25
    assert TOOLS == before
    # An older server or unavailable/malformed capability must not invent a limit.
    for value in (None, {}, {'maximum_climb_speed_mps': float('nan')},
                  {'maximum_climb_speed_mps': -1}, {'maximum_climb_speed_mps': True}):
        unbound = decision_tools({'motion_limits': value})
        assert 'maximum' not in prop(unbound, 'takeoff', 'maximum_climb_rate_mps')


def test_orbit_catalog_requires_visual_intent_and_reference_workflows_are_not_batched():
    catalog = decision_tools({'targets': [{'target_id': 'measured_target'}]})
    schema = next(t['function']['parameters'] for t in catalog if t['function']['name']=='orbit')
    assert {'camera','destination'} <= set(schema['required'])
    assert schema['properties']['camera']['enum'] == ['forward_d435i','downward_brio','none']
    variants = prop(catalog, 'execute_sequence', 'actions')['items']['oneOf']
    assert not {'goto','orbit'}.intersection(b['properties']['name']['const'] for b in variants)
    validate_call('orbit', {'center_reference':'home','radius_m':3,'altitude_agl_m':8})


def test_exact_decision_contract_rejects_invalid_calls_before_any_executor(monkeypatch):
    from python.dcm.agent_tools import ToolExecutor
    from test_direct_agent import Runtime
    cases = [
        ('orbit', {'destination': {'type':'home'}, 'radius_m':3, 'altitude_agl_m':8}, {}, 'camera'),
        ('takeoff', {'target_altitude_agl_m': 3, 'maximum_climb_rate_mps': 3},
         {'motion_limits': {'maximum_climb_speed_mps': 2.5}}, 'maximum'),
        ('orbit', {'destination': {'type':'registered_target','target_id':'invented'}, 'standoff_m':3, 'camera':'forward_d435i'},
         {'targets':[{'target_id':'registered'}]}, 'destination'),
        ('resume_heading_scan', {'scan_id': 'completed'}, {}, 'unavailable'),
        ('resume_scan', {'scan_id': 'completed_area'}, {}, 'unavailable'),
        ('execute_sequence', {'actions': [{'name': 'arm', 'arguments': {}},
            {'name': 'takeoff', 'arguments': {'target_altitude_agl_m': 3,
                                            'maximum_climb_rate_mps': 3}}]},
         {'motion_limits': {'maximum_climb_speed_mps': 2.5}}, 'maximum'),
        ('execute_sequence', {'actions': [{'name': 'orbit', 'arguments': {
            'center_reference': 'home', 'radius_m': 3, 'altitude_agl_m': 8}}]}, {}, 'actions'),
    ]
    def unexpected_execution(*args, **kwargs):
        raise AssertionError('Invalid advertised call reached ToolExecutor')
    monkeypatch.setattr(ToolExecutor, 'execute', unexpected_execution)
    for name, arguments, extra_context, diagnostic in cases:
        client = FakeClient()
        session = AgentSession('Exercise tool contract.')
        # Handle-contract tests exercise already-discovered tools.
        session.loaded_tools.update({"goto", "orbit", "localize_target", "overfly_target", "approach_target", "execute_sequence"})
        session.angular_scans = {'completed': {'completed': True}}
        session.area_scans = {'completed_area': {'completed': True}}
        runtime = Runtime(call(name, **arguments),
                          call('finish', outcome='incomplete', message='Invalid arguments.'))
        fly_mission(client, runtime, session.mission, session=session,
                    context_provider=lambda c: {**context(c), **extra_context},
                    echo=lambda *args: None)
        assert not client.sent
        assert not session.actions
        assert len(session.rejected_validation_attempts) == 1
        rejected = session.rejected_validation_attempts[0]
        assert rejected['no_action_executed'] is True
        assert diagnostic in rejected['message'], (name, rejected)


def test_exact_contract_is_also_used_after_truncation_retry():
    from python.dcm.llama_runtime import IncompleteModelResponse
    client = FakeClient()
    session = AgentSession('Check retry contract.')
    # Handle-contract tests exercise already-discovered tools.
    session.loaded_tools.update({"goto", "orbit", "localize_target", "overfly_target", "approach_target", "execute_sequence"})
    catalogs = []
    class Runtime:
        def decide(self, messages, tools, concise=False):
            catalogs.append(tools)
            if len(catalogs) == 1:
                raise IncompleteModelResponse('truncated')
            if len(catalogs) == 2:
                assert concise
                return call('takeoff', target_altitude_agl_m=3, maximum_climb_rate_mps=3)
            return call('finish', outcome='incomplete', message='Stopped.')
    fly_mission(client, Runtime(), session.mission, session=session,
                context_provider=lambda c: {**context(c), 'motion_limits': {'maximum_climb_speed_mps': 2.5}},
                echo=lambda *args: None)
    assert catalogs[0] is catalogs[1]
    assert not client.sent
    assert 'maximum' in session.rejected_validation_attempts[0]['message']


def test_direct_executor_retains_static_api_contract():
    from python.dcm.agent_tools import ToolExecutor
    client = FakeClient()
    validate_call('orbit', {'center_reference': 'home', 'radius_m': 3, 'altitude_agl_m': 8})
    result = ToolExecutor(client).execute('takeoff', {'target_altitude_agl_m': 3,
                                                   'maximum_climb_rate_mps': 3})
    assert result['status'] == 'SUCCEEDED'
    assert [name for name, _ in client.sent] == ['Arm', 'Takeoff']


def test_native_tagged_destinations_keep_root_fields_and_exact_nested_variants():
    import pytest
    catalog=decision_tools({'targets':[{'target_id':'known'}],
                            'motion_limits':{'maximum_horizontal_speed_mps':2}})
    for name,common in [('approach_target',{'object_position':'standoff','standoff_m':3}),
                        ('orbit',{'standoff_m':3,'camera':'forward_d435i'})]:
        schema=next(t['function']['parameters'] for t in catalog if t['function']['name']==name)
        assert 'destination' in schema['properties'] and 'oneOf' not in schema
        assert prop(catalog,name,'target_id')['enum']==['known']
        assert prop(catalog,name,'maximum_ground_speed_mps')['maximum']==2
        valid=[{'type':'registered_target','target_id':'known'},
               {'type':'visible_object','visual_query':'visible structure','view_heading_deg':0}]
        for destination in valid:
            validate_call(name,{**common,'destination':destination},tools=catalog)
        for destination in [{}, {'type':'registered_target','target_id':'invented'},
                            {'type':'registered_target','target_id':'known','visual_query':'x'},
                            {'type':'visible_object','visual_query':'x'}]:
            with pytest.raises(ValueError):
                validate_call(name,{**common,'destination':destination},tools=catalog)
        with pytest.raises(ValueError):
            validate_call(name,{**common,'destination':valid[0],'maximum_ground_speed_mps':3},tools=catalog)

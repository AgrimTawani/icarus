import copy
import pytest
from python.dcm.action_memory import archive_older_observations, read_action_result


def receipts():
    return [{'action': 'execute_sequence', 'result': {'actions': [
        {'name': 'arm', 'result': {'status': 'SUCCEEDED'}},
        {'name': 'takeoff', 'arguments': {'target_altitude_agl_m': 3},
         'result': {'status': 'SUCCEEDED'}}]}},
        {'action': 'inspect_scene', 'arguments': {}, 'result': {'status': 'SUCCEEDED',
         'evidence': {'answer': 'grey structure', 'samples': [{'height': i} for i in range(800)]}}}]


def test_archive_keeps_condition_evidence_and_lookup_preserves_full_receipt():
    actions = receipts(); original = copy.deepcopy(actions)
    evidence = actions[-1]['result']['evidence']
    observations = [{'after_action_index': 3, 'status': 'SUCCEEDED', 'evidence': evidence},
                    {'after_action_index': 4, 'evidence': {'counts': {'person': 0}}},
                    {'after_action_index': 5, 'evidence': {'answer': 'uncertain'}}]
    packed = archive_older_observations(observations)
    assert packed[0]['evidence']['answer'] == 'grey structure'
    assert packed[1:] == observations[1:]
    reference = packed[0]['retained_details']
    assert reference['action_index'] == 3
    field = reference['fields'][0]
    first = read_action_result(actions, 3, field['path'], limit=3)
    assert first['data'] == [{'height': 0}, {'height': 1}, {'height': 2}]
    assert first['next_offset'] == 3 and first['total_items'] == 800
    assert read_action_result(actions, 3, [*field['path'], 799, 'height'])['data'] == 799
    assert actions == original and observations[0]['evidence'] == original[-1]['result']['evidence']


def test_lookup_indices_match_flattened_execution_ledger():
    assert read_action_result(receipts(), 2, ['arguments'])['data'] == {'target_altitude_agl_m': 3}
    assert read_action_result(receipts(), 1, ['action'])['data'] == 'arm'


@pytest.mark.parametrize('index,path', [(0, []), (99, [])])
def test_bad_lookup_cannot_read_other_data(index, path):
    with pytest.raises(ValueError):read_action_result(receipts(), index, path)


def test_large_payload_has_explicit_drill_down_instead_of_an_unbounded_reply():
    result = read_action_result(receipts(), 3)
    assert 'data' not in result and result['fields']
    large = [{'action':'inspect_scene', 'result': {'answer': 'x'*8000}}]
    a = read_action_result(large, 1, ['result','answer'])
    b = read_action_result(large, 1, ['result','answer'], offset=a['next_offset'])
    assert len(a['data']) == 400 and b['next_offset'] == 800


def test_many_observations_page_history_without_losing_current_condition_or_source_data():
    from python.dcm.action_memory import observation_memory_view
    observations = [{'after_action_index': i, 'evidence': {'answer': 'view '+str(i),
                     'diagnostic': {'raw': 'x'*5000}}} for i in range(1, 31)]
    original = copy.deepcopy(observations)
    view = observation_memory_view(observations, history_bytes=1000)
    assert [item['evidence']['answer'] for item in view['observations'][-2:]] == ['view 29', 'view 30']
    assert view['archived_observations']['action_indices'][0] == 1
    represented = view['archived_observations']['action_indices'] + [
        entry['after_action_index'] for entry in view['observations']]
    assert represented == list(range(1,31)) and observations == original


def test_large_current_scan_exposes_the_actual_latest_station_for_bounded_lookup():
    import json
    from python.dcm.action_memory import observation_memory_view
    stations = [{'station': i, 'evidence': {'counts': {'person': i % 4}}} for i in range(800)]
    actions = [{'action': 'resume_heading_scan', 'result': {'status': 'SUCCEEDED',
                'evidence': {'scan_id': 'scan', 'observations': stations}}}]
    view = observation_memory_view([{'after_action_index': 1,
                                    'evidence': actions[0]['result']['evidence']}])
    assert len(json.dumps(view)) < 2000
    field = view['observations'][0]['retained_details']['fields'][0]
    latest = read_action_result(actions, 1, field['latest_path'])
    assert latest['data']['station'] == 799
    assert latest['data']['evidence']['counts']['person'] == 3
    assert latest['current_evidence'] is False


def test_invalid_receipt_path_returns_actual_structure_without_claiming_data():
    actions=receipts();original=copy.deepcopy(actions)
    result=read_action_result(actions,3,['result','observation','counts'])
    assert result['path_found'] is False and result['path']==['result']
    evidence=next(f for f in result['fields'] if f['key']=='evidence')
    assert evidence['path']==['result','evidence']
    assert 'data' not in result and result['current_evidence'] is False
    answer=read_action_result(actions,3,[*evidence['path'],'answer'])
    assert answer['data']=='grey structure' and actions==original
    bad_index=read_action_result(actions,3,['result','evidence','samples',-1])
    assert bad_index['path_found'] is False and bad_index['next_path']==['result','evidence','samples',0]


def test_execution_history_bounds_repeated_geometry_without_deleting_action_identities():
    import json
    from python.dcm.action_memory import execution_memory_view
    entries = [{'index': i, 'action': 'observe', 'source': 'model_tool_call',
                'verification_scope': 'Repeated description of result scope.',
                'arguments': {'sensor': 'camera'}, 'status': 'SUCCEEDED',
                'outcome_facts': {'found': False, 'count': 0, 'range_m': None,
                                 'camera_geometry': {'raw': 'x' * 10000}}}
               for i in range(1, 65)]
    original = copy.deepcopy(entries)
    packed = execution_memory_view(entries)
    assert [(e['index'], e['action'], e['status']) for e in packed] == [
        (i, 'observe', 'SUCCEEDED') for i in range(1, 65)]
    assert len(json.dumps(packed)) < 13000
    assert len(json.dumps(packed)) < len(json.dumps(entries)) / 20
    assert packed[-1]['outcome_facts'] == {'found': False, 'count': 0, 'range_m': None}
    assert packed[-1]['retained_details']['paths'] == [['result', 'evidence']]
    assert packed[0]['details_paged'] is True and entries == original


def test_execution_projection_preserves_small_condition_facts_and_real_lookup_path():
    from python.dcm.action_memory import execution_memory_view
    actions = [{'action': 'inspect_scene', 'arguments': {'question': 'Measure the view'},
                'result': {'status': 'SUCCEEDED', 'evidence': {
                    'found': False, 'counts': {'object': 0}, 'answerable': None,
                    'details': {'measurements': list(range(900))}}}}]
    entry = {'index': 1, 'action': 'inspect_scene', 'arguments': actions[0]['arguments'],
             'status': 'SUCCEEDED', 'outcome_facts': copy.deepcopy(actions[0]['result']['evidence'])}
    packed = execution_memory_view([entry])[0]
    assert packed['outcome_facts'] == {'found': False, 'counts': {'object': 0}, 'answerable': None}
    path = packed['retained_details']['paths'][0]
    assert path == ['result', 'evidence']
    resolved = read_action_result(actions, packed['retained_details']['action_index'], [*path, 'details', 'measurements', 899])
    assert resolved['data'] == 899


def test_grouped_receipt_projection_retains_both_range_and_actual_lookup_index():
    from python.dcm.action_memory import execution_memory_view
    entry = {'index': 2, 'through_index': 9, 'action': 'resume_scan', 'executions': 8,
             'status': 'FAILED', 'message': 'x' * 800, 'outcome_facts': {'found': False}}
    row = execution_memory_view([entry])[0]
    assert row['index'] == 2 and row['through_index'] == 9 and row['executions'] == 8
    assert row['status'] == 'FAILED' and row['outcome_facts']['found'] is False
    assert row['retained_details']['action_index'] == 9

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


@pytest.mark.parametrize('index,path', [(0, []), (99, []), (3, ['missing']), (3, ['result','evidence','samples',-1])])
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

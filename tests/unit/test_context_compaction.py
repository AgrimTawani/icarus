import copy

import pytest

from python.dcm.context_compaction import compact_scan_receipts


def test_nested_target_observations_keep_historical_measurements_without_replaying_old_geometry():
    import copy
    from python.dcm.context_compaction import reference_registered_target_geometry
    target = {'schema': 'icarus.perception.target_estimate.v1', 'target_id': 'a',
              'found': True, 'observed_at_unix_ms': 100, 'surface_range_m': 8,
              'bearing_deg': {'absolute': 20}, 'quality': {'age_ms': 30},
              'geometry': {'height_m': 6, 'height_status': 'measured',
                           'center': {'north_m': 12, 'east_m': 3},
                           'measurement_view_candidates': [{'north_m': i} for i in range(100)]}}
    receipt = {'requested_standoff_m': 3, 'target_localization': target}
    original = copy.deepcopy(receipt)
    packed = reference_registered_target_geometry(receipt, {'a'})
    observation = packed['target_localization']
    assert observation['surface_range_m'] == 8 and observation['observed_at_unix_ms'] == 100
    assert observation['historical_height'] == {'height_m': 6, 'height_status': 'measured'}
    assert observation['bearing_deg'] == {'absolute': 20} and observation['quality']['age_ms'] == 30
    assert observation['current_geometry_reference'] == 'targets' and 'geometry' not in observation
    assert reference_registered_target_geometry(receipt, set()) == original
    assert receipt == original


def entry(index, *, scan='scan_a', action='resume_heading_scan', status='SUCCEEDED', source='model_tool_call'):
    return {'index': index, 'action': action, 'arguments': {'scan_id': scan},
            'source': source, 'status': status, 'message': f'{index} observed stations'}


def test_consecutive_resumes_keep_original_range_and_latest_progress_without_mutation():
    raw = [entry(i) for i in range(4, 9)]
    original = copy.deepcopy(raw)
    grouped, = compact_scan_receipts(raw)
    assert raw == original
    assert grouped['index'] == 4 and grouped['through_index'] == 8
    assert grouped['executions'] == 5 and grouped['message'] == raw[-1]['message']
    assert grouped['arguments'] == {'scan_id': 'scan_a'}
    assert grouped['evidence_reference']['scan_id'] == 'scan_a'
    assert 'cumulative' in grouped['evidence_reference']['location']


@pytest.mark.parametrize('barrier', [
    {'status': 'FAILED'}, {'action': 'detect'}, {'scan': 'scan_b'},
    {'source': 'perception_feedback'}, {'action': 'resume_scan'}, {'scan': ''},
])
def test_no_grouping_across_failure_other_action_workflow_or_source(barrier):
    raw = [entry(1), entry(2), entry(3, **barrier), entry(4), entry(5)]
    compact = compact_scan_receipts(raw)
    assert [e['index'] for e in compact] == [1, 3, 4]
    assert compact[0]['through_index'] == 2
    assert compact[1] == raw[2]
    assert compact[2]['through_index'] == 5


def test_nonconsecutive_indices_and_changed_arguments_are_not_merged():
    first, second = entry(1), entry(3)
    assert compact_scan_receipts([first, second]) == [first, second]
    second = entry(2)
    second['arguments']['extra'] = True
    assert compact_scan_receipts([first, second]) == [first, second]


def test_area_scan_resumes_are_supported_and_single_receipt_is_unchanged():
    raw = [entry(1, action='resume_scan'), entry(2, action='resume_scan')]
    assert compact_scan_receipts(raw)[0]['executions'] == 2
    assert compact_scan_receipts(raw[:1]) == raw[:1]


def scan_evidence(count=12):
    return {'scan_id': 'scan_a', 'event': 'observation_ready', 'observations': [
        {'station': i + 1, 'heading_deg': i * 30, 'evidence': {
            'counts': {'person': 4 if i == 0 else 0, **{name: 0 for name in
                ('car', 'truck', 'bicycle', 'chair', 'bed', 'couch', 'tv', 'laptop')}},
            'camera': 'forward_d435i', 'confidence_threshold': .35,
            'count_scope': 'current_camera_frame', 'captured_at_unix_ms': 1000 + i}}
        for i in range(count)]}


def unpack_scan(packed):
    restored = copy.deepcopy(packed)
    shared = restored.pop('shared_station_evidence', {})
    restored.pop('station_evidence_merge', None)
    for station in restored['observations']:
        item = station['evidence']
        merged = {**copy.deepcopy(shared), **item}
        if 'counts' in shared:
            merged['counts'] = {**shared['counts'], **item.get('counts', {})}
        station['evidence'] = merged
    return restored


def test_scan_packing_roundtrips_all_evidence_and_keeps_latest_zero_explicit():
    import json
    from python.dcm.context_compaction import compact_scan_evidence
    original = scan_evidence()
    before = copy.deepcopy(original)
    packed = compact_scan_evidence(original)
    assert unpack_scan(packed) == original == before
    assert packed['observations'][-1]['evidence'] == original['observations'][-1]['evidence']
    assert packed['observations'][-1]['evidence']['counts']['person'] == 0
    assert 'person' not in packed['shared_station_evidence']['counts']
    assert all('person' in s['evidence']['counts'] for s in packed['observations'])
    assert packed['shared_station_evidence']['counts']['car'] == 0
    assert len(json.dumps(packed, separators=(',', ':'))) < len(json.dumps(original, separators=(',', ':')))
    assert compact_scan_evidence(packed) == packed


def test_differing_camera_and_threshold_remain_per_station():
    from python.dcm.context_compaction import compact_scan_evidence
    original = scan_evidence()
    original['observations'][3]['evidence'].update(camera='downward_brio', confidence_threshold=.6)
    packed = compact_scan_evidence(original)
    assert 'camera' not in packed['shared_station_evidence']
    assert 'confidence_threshold' not in packed['shared_station_evidence']
    assert unpack_scan(packed) == original


def test_missing_class_or_unknown_count_never_acquires_shared_zero():
    from python.dcm.context_compaction import compact_scan_evidence
    original = scan_evidence()
    del original['observations'][2]['evidence']['counts']['car']
    original['observations'][3]['evidence']['counts']['truck'] = None
    original['observations'][4]['evidence']['counts']['chair'] = False
    packed = compact_scan_evidence(original)
    zeros = packed['shared_station_evidence']['counts']
    assert not {'car', 'truck', 'chair'} & zeros.keys()
    assert unpack_scan(packed) == original


@pytest.mark.parametrize('field', [{'error': 'capture failed'}, {'fresh': False},
                                  {'validity': 'unknown'}, {'status': 'FAILED'}])
def test_failed_or_stale_station_prevents_shared_evidence_packing(field):
    from python.dcm.context_compaction import compact_scan_evidence
    original = scan_evidence()
    original['observations'][3]['evidence'].update(field)
    assert compact_scan_evidence(original) == original


def test_missing_counts_prevents_any_shared_zero_inference_and_small_pack_is_not_used():
    from python.dcm.context_compaction import compact_scan_evidence
    original = scan_evidence()
    del original['observations'][2]['evidence']['counts']
    packed = compact_scan_evidence(original)
    assert 'counts' not in packed.get('shared_station_evidence', {})
    assert unpack_scan(packed) == original
    small = {'scan_id': 'a', 'observations': [
        {'evidence': {'counts': {'person': 0}}}, {'evidence': {'counts': {'person': 0}}}]}
    assert compact_scan_evidence(small) == small

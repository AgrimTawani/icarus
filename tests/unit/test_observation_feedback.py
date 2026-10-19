import pytest
from python.dcm.observation_feedback import observation_feedback


def ok(evidence):
    return {'status': 'SUCCEEDED', 'evidence': evidence}


def test_detection_reports_actual_accepted_counts_and_camera():
    lines = observation_feedback('detect', ok({'counts': {'person': 5, 'car': 0}, 'camera': 'forward_d435i'}))
    assert lines == ['forward_d435i: accepted detections person=5, car=0 (not a guaranteed total)']


@pytest.mark.parametrize('evidence', [{}, {'counts': {}}, {'unique_person_count': 5}])
def test_missing_counts_never_invent_zero_or_treat_lifetime_tracks_as_current(evidence):
    assert observation_feedback('detect', ok(evidence)) == ['camera unspecified: accepted detection counts unavailable']


@pytest.mark.parametrize('invalid', [{'validity': 'unknown'}, {'fresh': False}, {'validity': 'error'}])
def test_invalid_evidence_does_not_print_stale_counts(invalid):
    text = ' '.join(observation_feedback('detect', ok({'counts': {'person': 0}, **invalid})))
    assert 'unavailable or stale' in text and 'person=0' not in text


def test_bad_counts_unknown_not_zero():
    text = observation_feedback('detect', ok({'counts': {'person': float('nan'), 'car': -1, 'dog': True}}))[0]
    assert 'person=unknown, car=unknown, dog=unknown' in text


def test_failed_observation_ignores_any_evidence_payload():
    assert observation_feedback('detect', {'status': 'FAILED', 'message': 'camera disconnected', 'evidence': {'counts': {'person': 0}}}) == ['detect: FAILED: camera disconnected']


def test_scene_answer_is_attributed_bounded_and_does_not_assert_truth():
    text = observation_feedback('inspect_scene', ok({'answer': 'Maybe a grey building.\nNot certain.'}), {'camera': 'forward_d435i'})[0]
    assert text == 'forward_d435i: VLM answer (uncertain): Maybe a grey building. Not certain.'
    assert len(observation_feedback('inspect_scene', ok({'answer': 'x' * 2000}))[0]) < 360
    assert 'unavailable' in observation_feedback('inspect_scene', ok({}))[0]


def test_target_reports_measured_surface_not_candidate_or_footprint_radius():
    text = observation_feedback('localize_target', ok({'found': True, 'target_id': 'target_a', 'surface_range_m': 3.156, 'footprint': {'radius_m': 9}}))[0]
    assert text == 'Target target_a: localized; measured surface range 3.16 m'
    assert 'unavailable' in observation_feedback('localize_target', ok({'found': True, 'surface_range_m': None}))[0]
    assert 'not localized: no association' in observation_feedback('localize_target', ok({'found': False, 'reason': 'no association', 'surface_range_m': 3}))[0]


@pytest.mark.parametrize('tool', ['scan_heading', 'resume_heading_scan', 'scan_area', 'resume_scan'])
def test_scan_cumulative_observations_show_only_new_stations(tool):
    observations = [{'station': i, 'heading_deg': i * 30, 'evidence': {'counts': {'person': i}, 'camera': 'forward_d435i'}} for i in range(1, 5)]
    lines = observation_feedback(tool, ok({'observations': observations, 'event': 'observation_ready'}), previous_observation_count=3)
    assert len(lines) == 2 and lines[0].startswith('Station 4 at 120°') and 'person=4' in lines[0]
    assert observation_feedback(tool, ok({'observations': observations}), previous_observation_count=4) == []


def test_large_scan_is_bounded_with_explicit_omissions_and_scene_answers():
    observations = [{'station': i, 'evidence': {'answer': 'Grey wall', 'camera': 'downward_brio'}} for i in range(20)]
    lines = observation_feedback('scan_area', ok({'observations': observations}))
    assert len(lines) == 4 and '20 new observations; showing last 3' in lines[0]
    assert all('VLM answer (uncertain): Grey wall' in x for x in lines[1:])


def test_failed_scan_can_show_completed_observation_but_not_claim_scan_success():
    r = {'status': 'FAILED', 'message': 'turn timed out', 'evidence': {'observations': [{'station': 1, 'evidence': {'counts': {'person': 0}}}], 'event': 'turn_failed'}}
    lines = observation_feedback('scan_heading', r)
    assert lines[0] == 'scan_heading: FAILED: turn timed out'
    assert 'person=0' in lines[1] and lines[-1] == 'scan_heading: turn_failed'


def test_non_observation_and_malformed_payloads_are_safe():
    assert observation_feedback('yaw', ok({})) == []
    assert observation_feedback('detect', None) == []
    assert 'unavailable' in observation_feedback('detect', ok(None))[0]
    assert 'unavailable' in observation_feedback('scan_area', ok({'observations': None}))[0]

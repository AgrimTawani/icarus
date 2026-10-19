"""Measurement validity is separate from condition truth and arrival order."""
import pytest

from python.dcm.evidence import usable_observation, valid_capture_timestamp


@pytest.mark.parametrize('value', [None, False, True, 0, -1, '123', float('nan'), float('inf')])
def test_invalid_capture_timestamps(value):
    assert not valid_capture_timestamp(value)
    assert not usable_observation({'found': True, 'captured_at_unix_ms': value})


@pytest.mark.parametrize('value', [False, True])
def test_both_boolean_outcomes_are_usable(value):
    assert usable_observation({'found': value, 'captured_at_unix_ms': 123})


@pytest.mark.parametrize('value', [0, 2, 99])
def test_zero_is_a_valid_count(value):
    assert usable_observation({'count': value, 'captured_at_unix_ms': 123}, {'result_type': 'count'})


@pytest.mark.parametrize('value', [None, True, -1, 2.5, '2'])
def test_noncounts_are_rejected(value):
    assert not usable_observation({'count': value, 'captured_at_unix_ms': 123}, {'result_type': 'count'})


@pytest.mark.parametrize('extra', [
    {'validity': 'unknown'}, {'validity': 'error'}, {'error': 'failure'},
    {'error_code': 'malformed_output'}, {'result_type': 'count'},
])
def test_errors_or_type_mismatch_cannot_be_masked_by_truthy_measurement(extra):
    result = dict(found=True, captured_at_unix_ms=123, **extra)
    assert not usable_observation(result, {'result_type': 'boolean'})


@pytest.mark.parametrize('stamp,expected', [(99, False), (100, True), (150, True), (200, True), (201, False)])
def test_capture_window_checks_both_ends(stamp, expected):
    assert usable_observation({'found': False, 'captured_at_unix_ms': stamp},
                              captured_after_ms=100, captured_before_ms=200) is expected


def test_unsuitable_but_assessable_surface_is_valid_negative_evidence():
    result = {'tool': 'landing', 'assessable': True, 'suitable': False, 'captured_at_unix_ms': 123}
    assert usable_observation(result)
    result['assessable'] = False
    assert not usable_observation(result)


def test_unique_survey_not_claimed_from_single_frame_count():
    result = {'count': 2, 'captured_at_unix_ms': 123}
    assert not usable_observation(result, {'result_type': 'count', 'count_scope': 'unique_survey'})

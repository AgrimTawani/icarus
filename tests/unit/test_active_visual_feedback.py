import pytest

from python.dcm.active_visual_feedback import ActiveVisualFeedback


def answer(found=None, status='SUCCEEDED'):
    return {'status': status, 'evidence': {
        'validity': 'valid' if type(found) is bool else 'unknown', 'found': found}}


def requested(found=None):
    feedback = ActiveVisualFeedback()
    feedback.observe('inspect_scene', {'question': 'Is the requested landmark visible?',
                     'question_type': 'condition', 'view': 'north'}, answer(found))
    return feedback


@pytest.mark.parametrize('found', [True, False, None])
def test_each_turn_refreshes_original_question_in_current_view(found):
    f = requested(found)
    calls = []
    class Executor:
        def interrupted(self): return False
        def execute(self, name, arguments):
            calls.append((name, arguments))
            return answer(False)
    for _ in range(3):
        f.observe('yaw', {}, {'status': 'SUCCEEDED'})
        assert f.snapshot()['found'] is None
        entry = f.refresh(Executor())
        assert entry['source'] == 'visual_feedback'
        assert f.snapshot()['found'] is False
        assert f.refresh(Executor()) is None
    assert len(calls) == 3
    assert all(args['view'] == 'current' and args['question'] == f.interest['question'] for _, args in calls)


def test_detector_cannot_resolve_visual_question():
    f = requested()
    f.observe('detect', {'classes': ['person']}, answer(True))
    assert f.snapshot()['found'] is None
    assert f.unresolved_approach('approach_target', {'destination': {'type': 'visible_object'}})
    assert not f.unresolved_approach('localize_target', {})
    assert not f.unresolved_approach('goto', {'destination': {'type': 'home'}})


def test_negative_condition_is_not_globally_interpreted_as_forbidden_branch():
    # E.g. an operator can request an approach when an object is NOT red.
    assert not requested(False).unresolved_approach('approach_target', {'destination': {'type': 'visible_object'}})


def test_failed_localization_keeps_search_successful_localization_retires_it():
    f = requested(False)
    f.observe('localize_target', {}, answer(False))
    assert f.interest
    f.observe('localize_target', {}, {'status': 'SUCCEEDED', 'evidence': {'found': True, 'target_id': 'measured'}})
    f.observe('yaw', {}, {'status': 'SUCCEEDED'})
    assert f.snapshot() is None and not f.pending


def test_localization_does_not_answer_an_unknown_appearance_question():
    f = requested()
    f.observe('localize_target', {}, {'status': 'SUCCEEDED', 'evidence': {'found': True, 'target_id': 'measured'}})
    assert f.snapshot()['found'] is None


@pytest.mark.parametrize('name', ['get_context', 'arm', 'hold', 'load_tools'])
def test_non_viewpoint_tools_do_not_schedule_vlm(name):
    f = requested(False)
    f.observe(name, {}, {'status': 'SUCCEEDED'})
    assert not f.pending


def test_no_automatic_retry_loop_after_observation_failure():
    f = requested(False)
    f.observe('yaw', {}, {'status': 'SUCCEEDED'})
    class Executor:
        def interrupted(self): return False
        def execute(self, *_): raise RuntimeError('camera unavailable')
    assert f.refresh(Executor())['result']['status'] == 'OBSERVATION_UNAVAILABLE'
    assert f.snapshot()['found'] is None
    assert f.refresh(Executor()) is None

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


@pytest.mark.parametrize('tool', ['localize_target', 'approach_target', 'orbit'])
def test_contradictory_localization_needs_new_view_and_new_observation(tool):
    f = requested(True)
    args = ({'visual_query': 'landmark'} if tool == 'localize_target' else
            {'destination': {'type': 'visible_object', 'visual_query': 'landmark'}})
    evidence = {'found': False, 'reason': 'no_match'}
    result = {'status': 'SUCCEEDED' if tool == 'localize_target' else 'FAILED',
              'evidence': evidence if tool == 'localize_target' else {'target_localization': evidence}}
    f.observe(tool, args, result)
    assert f.snapshot()['found'] is None
    assert f.recovery_required(tool, args)
    # Merely asking again or resetting association cannot erase the conflict.
    f.observe('inspect_scene', {'question': 'Visible?', 'question_type': 'condition'}, answer(True))
    assert f.recovery_required('localize_target', {'description': 'landmark', 'new_target': True})
    f.observe('hold', {}, {'status': 'SUCCEEDED'})
    f.observe('face_heading', {}, {'status': 'SUCCEEDED',
                                 'evidence': {'no_flight_command_sent': True}})
    f.observe('yaw', {}, {'status': 'FAILED'})
    f.observe('inspect_scene', {'question': 'Visible?', 'question_type': 'condition'}, answer(True))
    assert f.recovery_required(tool, args)
    f.observe('yaw', {'angle_deg': 17}, {'status': 'SUCCEEDED'})
    assert f.recovery_required(tool, args)  # movement alone is insufficient
    f.observe('inspect_scene', {'question': 'Visible?', 'question_type': 'condition'}, answer(None))
    assert f.recovery_required(tool, args)  # unknown still insufficient
    f.observe('inspect_scene', {'question': 'Visible?', 'question_type': 'condition'}, answer(False))
    assert not f.recovery_required(tool, args)
    assert f.snapshot()['found'] is False


def test_recovery_is_scoped_to_failed_subject_not_home_or_another_target():
    f = requested(True)
    f.observe('localize_target', {'visual_query': 'tower'}, answer(False))
    assert not f.recovery_required('goto', {'destination': {'type': 'home'}})
    assert not f.recovery_required('localize_target', {'description': 'vehicle'})
    assert f.recovery_required('localize_target', {'description': ' TOWER '})


def test_post_motion_refresh_resolves_recovery_without_an_extra_model_call():
    f = requested(True)
    f.observe('localize_target', {'visual_query': 'landmark'}, answer(False))
    f.observe('yaw', {'angle_deg': 13}, {'status': 'SUCCEEDED'})
    class Executor:
        def interrupted(self): return False
        def execute(self, *_): return answer(False)
    f.refresh(Executor())
    assert not f.recovery and f.snapshot()['found'] is False

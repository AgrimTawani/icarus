from copy import deepcopy
import pytest
from python.dcm.active_detection_feedback import ActiveDetectionFeedback


def detected(count=2, camera='forward_d435i', **extra):
    return {'status': 'SUCCEEDED', 'evidence': {'camera': camera, 'counts': {'person': count},
        'captured_at_unix_ms': 1000, 'capture_pose': {'local_position_ned': {'origin_id': 'origin'}},
        'unique_person_count': 99, **extra}}


class Executor:
    def __init__(self, result=None):
        self.result = result or detected(0)
        self.calls = []
        self.stopped = False
    def interrupted(self):
        return self.stopped
    def execute(self, name, args):
        self.calls.append((name, args))
        if isinstance(self.result, Exception):
            raise self.result
        return self.result


def active():
    f = ActiveDetectionFeedback(clock_ms=lambda: 1100)
    f.observe('detect', {'classes': ['person']}, detected())
    return f


def test_no_interest_never_dispatches_even_after_motion():
    f = ActiveDetectionFeedback(); ex = Executor()
    f.observe('yaw', {'angle_deg': 30}, {'status': 'SUCCEEDED'})
    assert f.snapshot() is None and not f.needs_refresh
    assert f.refresh(ex) is None and not ex.calls


def inspection_pair(count=2, camera='forward_d435i', failed=False):
    return {'status':'SUCCEEDED','evidence':{'answer':'A qualitative scene answer.',
        'camera_detection':{'arguments':{'classes':['person'],'camera':camera},
            'result':{'status':'FAILED','message':'Detector unavailable'} if failed else detected(count,camera)}}}


def test_inspection_detector_starts_feedback_and_refreshes_after_motion():
    f=ActiveDetectionFeedback(clock_ms=lambda:1100)
    original=inspection_pair(2)
    f.observe('inspect_scene',{},original)
    assert f.snapshot()['observation']['counts']=={'person':2}
    assert f.snapshot()['observation']['source_tool']=='inspection_detector'
    f.observe('orbit_target',{}, {'status':'SUCCEEDED'})
    ex=Executor(detected(3))
    f.refresh(ex)
    assert f.snapshot()['observation']['counts']=={'person':3}
    assert original['evidence']['camera_detection']['result']['evidence']['counts']=={'person':2}
    assert len(ex.calls)==1


def test_inspection_does_not_replace_an_explicit_camera_interest():
    f=active()
    f.observe('inspect_scene',{},inspection_pair(5,camera='downward_brio'))
    assert f.interest=={'camera':'forward_d435i','classes':['person']}
    assert f.snapshot()['observation']['counts']=={'person':2}


def test_failed_inspection_detector_is_unknown_and_can_refresh_after_motion():
    f=ActiveDetectionFeedback(clock_ms=lambda:1100)
    f.observe('inspect_scene',{},inspection_pair(failed=True))
    assert f.snapshot()['observation']['validity']=='unknown'
    assert 'counts' not in f.snapshot()['observation']
    f.observe('hold',{}, {'status':'SUCCEEDED'})
    f.refresh(Executor(detected(1)))
    assert f.snapshot()['observation']['counts']=={'person':1}


def test_motion_refreshes_once_with_requested_interest_preserving_history():
    f = active(); ex = Executor(); original = detected(); before = deepcopy(original)
    f.observe('detect', {'classes': ['person']}, original)
    f.observe('yaw', {'angle_deg': 30}, {'status': 'SUCCEEDED'})
    assert f.snapshot()['observation']['validity'] == 'unknown'
    assert 'counts' not in f.snapshot()['observation']
    assert f.needs_refresh
    assert f.refresh(ex) == ex.result
    assert f.refresh(ex) is None
    assert ex.calls == [('detect', {'camera': 'forward_d435i', 'classes': ['person']})]
    current = f.snapshot()
    assert current['observation']['counts'] == {'person': 0}
    assert current['observation']['capture_pose']['local_position_ned']['origin_id'] == 'origin'
    assert current['motion_action'] == 'yaw' and current['after_motion_revision'] == 1
    assert 'unique_person_count' not in current['observation'] and original == before


@pytest.mark.parametrize('tool', ['get_context', 'ask_user', 'finish', 'arm', 'disarm', 'inspect_scene'])
def test_reads_and_nonmoving_tools_do_not_refresh(tool):
    f = active(); ex = Executor()
    f.observe(tool, {}, {'status': 'SUCCEEDED'})
    assert not f.needs_refresh and f.refresh(ex) is None


def test_hold_refreshes_environment_without_automatic_loop():
    f = active(); ex = Executor()
    f.observe('hold', {'duration_s': 2}, {'status': 'SUCCEEDED'})
    f.refresh(ex); f.snapshot(); f.refresh(ex)
    assert len(ex.calls) == 1


def test_batch_refreshes_once_after_last_successful_motion():
    f = active(); ex = Executor()
    f.observe('execute_sequence', {}, {'status': 'SUCCEEDED', 'actions': [
        {'name': 'takeoff', 'arguments': {'target_altitude_agl_m': 3}, 'result': {'status': 'SUCCEEDED'}},
        {'name': 'yaw', 'arguments': {'angle_deg': 90}, 'result': {'status': 'SUCCEEDED'}}]})
    f.refresh(ex)
    assert len(ex.calls) == 1 and f.snapshot()['motion_action'] == 'yaw'


@pytest.mark.parametrize('result', [{'status': 'FAILED', 'message': 'camera missing'}, RuntimeError('offline')])
def test_failed_refresh_erases_old_count_and_never_retries_on_reads(result):
    f = active(); ex = Executor(result)
    f.observe('goto', {}, {'status': 'SUCCEEDED'}); f.refresh(ex)
    assert 'counts' not in f.snapshot()['observation']
    assert f.snapshot()['observation']['validity'] == 'unknown'
    assert f.refresh(ex) is None and len(ex.calls) == 1


def test_failed_motion_invalidates_without_capture():
    f = active(); ex = Executor()
    f.observe('goto', {}, {'status': 'FAILED'})
    assert not f.needs_refresh and f.refresh(ex) is None
    assert 'counts' not in f.snapshot()['observation']


def test_latest_camera_replaces_interest_and_deepcopies():
    f = active(); ex = Executor(detected(1, 'downward_brio'))
    args = {'classes': ['person'], 'camera': 'downward_brio'}
    f.observe('detect', args, detected(1, 'downward_brio')); args['classes'].append('car')
    f.observe('land', {}, {'status': 'SUCCEEDED'}); f.refresh(ex)
    assert ex.calls[0][1] == {'camera': 'downward_brio', 'classes': ['person']}


def test_scan_reuses_station_observation_without_duplicate_capture():
    f = active(); ex = Executor()
    f.observe('scan_heading', {}, {'status': 'SUCCEEDED', 'evidence': {
        'event': 'observation_ready', 'observations': [{'evidence': detected(3)['evidence']}]}})
    assert f.snapshot()['observation']['counts'] == {'person': 3}
    assert f.refresh(ex) is None


def test_completed_scan_does_not_misrepresent_earlier_station_as_current_view():
    f = active(); ex = Executor()
    f.observe('resume_scan', {}, {'status': 'SUCCEEDED', 'evidence': {
        'event': 'completed', 'observations': [{'evidence': detected(3)['evidence']}]}})
    assert 'counts' not in f.snapshot()['observation'] and f.needs_refresh
    f.refresh(ex)
    assert len(ex.calls) == 1 and f.snapshot()['observation']['counts'] == {'person': 0}


def test_stale_and_wrong_camera_are_unknown():
    f = active(); f.clock_ms = lambda: 99999
    assert 'counts' not in f.snapshot()['observation']
    f.observe('detect', {'classes': ['person']}, detected(3, 'downward_brio'))
    assert f.snapshot()['observation']['validity'] == 'unknown'


def test_interrupt_before_and_during_capture_never_keeps_counts():
    f = active(); ex = Executor(); f.observe('yaw', {}, {'status': 'SUCCEEDED'})
    ex.stopped = True; assert f.refresh(ex) is None and not ex.calls
    f.observe('yaw', {}, {'status': 'SUCCEEDED'}); ex.stopped = False
    def execute(*args):
        ex.stopped = True
        return detected(5)
    ex.execute = execute; f.refresh(ex)
    assert 'counts' not in f.snapshot()['observation']


def test_operator_choice_does_not_schedule_refresh():
    f = active(); ex = Executor()
    f.observe('orbit', {}, {'status': 'FAILED', 'requires_user_choice': True})
    assert not f.needs_refresh and f.refresh(ex) is None


def test_explicit_failed_detect_replaces_old_interest_but_never_fabricates_zero():
    f = active()
    f.observe('detect', {'classes': ['car'], 'camera': 'downward_brio'}, {'status': 'FAILED', 'message': 'no frame'})
    assert f.snapshot()['interest'] == {'classes': ['car'], 'camera': 'downward_brio'}
    assert f.snapshot()['observation']['status'] == 'FAILED'
    assert 'counts' not in f.snapshot()['observation']


def test_flight_only_area_scan_refreshes_existing_interest_once():
    f = active(); ex = Executor()
    f.observe('scan_area', {'observation': 'none'}, {'status': 'SUCCEEDED', 'evidence': {'backend': 'ardupilot_auto'}})
    assert f.needs_refresh
    f.refresh(ex); f.refresh(ex)
    assert len(ex.calls) == 1


@pytest.mark.parametrize('event', ['completed', 'sweep_exhausted'])
def test_angular_scan_final_station_capture_is_reused(event):
    f = active(); ex = Executor()
    f.observe('resume_heading_scan', {}, {'status': 'SUCCEEDED', 'evidence': {
        'event': event, 'observations': [{'evidence': detected(3)['evidence']}]}})
    assert f.snapshot()['observation']['counts'] == {'person': 3}
    assert not f.needs_refresh and f.refresh(ex) is None


def test_failed_scan_never_refreshes_without_settled_success():
    f = active(); ex = Executor()
    f.observe('resume_scan', {}, {'status': 'FAILED', 'evidence': {
        'event': 'flight_failed', 'observations': [{'evidence': detected(3)['evidence']}]}})
    assert not f.needs_refresh and f.refresh(ex) is None
    assert 'counts' not in f.snapshot()['observation']

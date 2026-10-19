import math
from types import SimpleNamespace
from python.dcm.angular_scan import AngularScan


class Executor:
    def __init__(self, results=()):
        self.state = {'local_position_ned': {'north_m': 0, 'east_m': 0, 'origin_id': 'x'},
                      'attitude': {'yaw_rad': 0}}
        self.client = SimpleNamespace(state=lambda: self.state)
        self.calls = []
        self.results = iter(results)
        self.stopped = False
    def interrupted(self): return self.stopped
    def _flight(self, name, args):
        self.calls.append((name, dict(args)))
        self.state['attitude']['yaw_rad'] += math.radians(args['angle_deg'])
        return {'status': 'SUCCEEDED'}
    def execute(self, name, args):
        self.calls.append((name, dict(args)))
        return next(self.results, {'status': 'SUCCEEDED', 'evidence': {'counts': {'person': 0}}})


def test_every_turn_has_observation_and_open_search_not_completed():
    ex = Executor(); w = AngularScan(ex)
    r = w.scan_heading(step_deg=30, classes=['person'], stop_on_detection='person', repeat_until_match=True)
    assert [x[0] for x in ex.calls] == ['yaw', 'detect'] * 12
    assert not r['workflow_complete'] and r['evidence']['event'] == 'sweep_exhausted'
    w.resume_heading_scan(r['evidence']['scan_id'])
    assert [x[0] for x in ex.calls] == ['yaw', 'detect'] * 24


def test_failed_capture_is_retried_without_another_turn():
    ex = Executor([{'status': 'FAILED', 'message': 'no frame'}]); w = AngularScan(ex)
    r = w.scan_heading(step_deg=30, sweep_deg=60, classes=['person'], return_on='sweep')
    assert r['status'] == 'FAILED' and r['evidence']['pending_phase'] == 'observe'
    done = w.resume_heading_scan(r['evidence']['scan_id'])
    assert [x[0] for x in ex.calls] == ['yaw', 'detect', 'detect', 'yaw', 'detect']
    assert done['workflow_complete']


def test_match_returns_before_next_turn_and_resume_continues_remaining_sweep():
    ex = Executor([{'status': 'SUCCEEDED', 'evidence': {'counts': {'person': 4}}}]); w = AngularScan(ex)
    r = w.scan_heading(step_deg=45, sweep_deg=90, classes=['person'], stop_on_detection='person', minimum_count=4)
    assert r['evidence']['event'] == 'condition_met'
    assert len(ex.calls) == 2
    r = w.resume_heading_scan(r['evidence']['scan_id'])
    assert r['workflow_complete'] and len(ex.calls) == 4


def test_scene_returns_each_station_for_nested_condition():
    ex = Executor(); w = AngularScan(ex)
    r = w.scan_heading(step_deg=30, sweep_deg=60, observation='scene', question='What is visible?')
    assert r['evidence']['event'] == 'observation_ready' and len(ex.calls) == 2
    r = w.resume_heading_scan(r['evidence']['scan_id'])
    assert len(ex.calls) == 4
    r = w.resume_heading_scan(r['evidence']['scan_id'])
    assert r['workflow_complete'] and len(ex.calls) == 4


def test_scene_without_specific_question_still_returns_one_observed_station():
    ex = Executor()
    result = AngularScan(ex).scan_heading(step_deg=30, observation='scene')
    assert [name for name, _ in ex.calls] == ['yaw', 'inspect_scene']
    assert ex.calls[-1][1]['question']
    assert result['evidence']['event'] == 'observation_ready'
    assert not result['workflow_complete']


def test_interrupt_never_dispatches_next_station():
    ex = Executor(); ex.stopped = True
    r = AngularScan(ex).scan_heading(step_deg=30, classes=['person'])
    assert r['status'] == 'CANCELLED' and not ex.calls


def test_unspecified_predicate_returns_observation_before_next_movement():
    ex = Executor([{'status': 'SUCCEEDED', 'evidence': {'counts': {'person': 3}}}])
    r = AngularScan(ex).scan_heading(step_deg=30, classes=['person'])
    assert r['evidence']['event'] == 'observation_ready'
    assert [x[0] for x in ex.calls] == ['yaw', 'detect']


def test_single_large_turn_observes_only_after_full_requested_arc():
    ex = Executor(); workflow = AngularScan(ex)
    result = workflow.scan_heading(step_deg=270, sweep_deg=270, clockwise=True, classes=['person'])
    assert [name for name, _ in ex.calls] == ['yaw', 'detect']
    assert ex.calls[0][1]['angle_deg'] == 270
    assert result['evidence']['observations'][0]['heading_deg'] == 270
    assert result['workflow_complete']
    completed = workflow.resume_heading_scan(result['evidence']['scan_id'])
    assert completed['workflow_complete']
    assert len(ex.calls) == 2

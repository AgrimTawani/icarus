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


def test_explicit_finish_at_match_is_terminal_before_next_turn():
    ex = Executor([{'status': 'SUCCEEDED', 'evidence': {'counts': {'person': n}}}
                   for n in (0, 0, 1)])
    workflow = AngularScan(ex)
    result = workflow.scan_heading(step_deg=30, classes=['person'],
        stop_on_detection='person', exit_policy='finish_at_match')
    assert [name for name, _ in ex.calls] == ['yaw', 'detect'] * 3
    assert result['workflow_complete']
    assert result['evidence']['event'] == 'condition_met'
    assert not result['evidence']['coverage_complete']
    count = len(ex.calls)
    assert workflow.resume_heading_scan(result['evidence']['scan_id'])['workflow_complete']
    assert len(ex.calls) == count


def test_explicit_pause_at_match_preserves_remaining_scan_for_nested_branch():
    ex = Executor([{'status': 'SUCCEEDED', 'evidence': {'counts': {'person': 1}}}])
    workflow = AngularScan(ex)
    result = workflow.scan_heading(step_deg=30, sweep_deg=60, classes=['person'],
        stop_on_detection='person', exit_policy='pause_at_match')
    assert not result['workflow_complete'] and len(ex.calls) == 2
    done = workflow.resume_heading_scan(result['evidence']['scan_id'])
    assert done['workflow_complete'] and done['evidence']['coverage_complete']
    assert len(ex.calls) == 4


def test_finish_policy_retries_unknown_observation_without_extra_turn():
    ex = Executor([{'status': 'SUCCEEDED', 'evidence': {'counts': {}}},
                   {'status': 'SUCCEEDED', 'evidence': {'counts': {'person': 1}}}])
    workflow = AngularScan(ex)
    first = workflow.scan_heading(step_deg=30, classes=['person'],
        stop_on_detection='person', exit_policy='finish_at_match')
    assert first['status'] == 'FAILED' and not first['workflow_complete']
    done = workflow.resume_heading_scan(first['evidence']['scan_id'])
    assert done['workflow_complete'] and done['evidence']['event'] == 'condition_met'
    assert [name for name, _ in ex.calls] == ['yaw', 'detect', 'detect']


def test_explicit_unconditional_scene_scan_covers_all_stations_without_model_calls():
    ex = Executor([{'status': 'SUCCEEDED', 'evidence': {'answer': 'Visible terrain.'}}] * 2)
    result = AngularScan(ex).scan_heading(step_deg=30, sweep_deg=60,
        observation='scene', exit_policy='complete_coverage')
    assert [name for name, _ in ex.calls] == ['yaw', 'inspect_scene'] * 2
    assert result['workflow_complete'] and result['evidence']['coverage_complete']
    assert result['evidence']['termination_reason'] == 'coverage_complete'


def test_final_turn_without_valid_observation_is_not_completed_coverage():
    ex = Executor([{'status': 'FAILED', 'message': 'camera unavailable'}])
    workflow = AngularScan(ex)
    result = workflow.scan_heading(step_deg=30, sweep_deg=30, classes=['person'],
                                   exit_policy='complete_coverage')
    assert not result['workflow_complete'] and not result['evidence']['coverage_complete']
    done = workflow.resume_heading_scan(result['evidence']['scan_id'])
    assert done['workflow_complete'] and done['evidence']['coverage_complete']
    assert [name for name, _ in ex.calls] == ['yaw', 'detect', 'detect']

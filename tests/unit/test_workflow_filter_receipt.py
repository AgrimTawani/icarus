"""Workflow stop filters disclose their own predicate, never mission verdicts."""
import pytest
from python.dcm.angular_scan import AngularScan
from python.dcm.scan_workflow import ScanWorkflow
from test_angular_scan import Executor as AngularExecutor
from test_scan_workflow import Executor as AreaExecutor


def run(kind, evidence, minimum=1):
    result = {'status': 'SUCCEEDED', 'evidence': evidence}
    ex = AngularExecutor([result]) if kind == 'angular' else AreaExecutor([result])
    args = {'classes': ['person'], 'stop_on_detection': 'person', 'minimum_count': minimum}
    if kind == 'angular':
        w = AngularScan(ex)
        r = w.scan_heading(step_deg=30, sweep_deg=30, **args)
    else:
        w = ScanWorkflow(ex)
        r = w.scan_area(shape='square', width_m=1, observation='detect', observation_count=1, **args)
    return ex, w, r


@pytest.mark.parametrize('kind', ['angular', 'area'])
@pytest.mark.parametrize('count,minimum,match', [(3, 1, True), (3, 4, False), (0, 1, False)])
def test_receipt_expresses_only_actual_filter_and_survives_compaction(kind, count, minimum, match):
    _, _, r = run(kind, {'counts': {'person': count}}, minimum)
    assert r['status'] == 'SUCCEEDED'
    receipt = r['evidence']['workflow_stop_filter']
    assert receipt == {'class': 'person', 'comparison': '>=', 'minimum_count': minimum,
                       'observed_count': count, 'matched': match, 'station': 1,
                       'scope': 'workflow_stop_filter_not_mission_verdict'}
    assert (r['evidence']['event'] == 'condition_met') == match
    assert r['evidence']['workflow_stop_filter'] == receipt
    assert all(item['workflow_stop_filter'] == receipt
               for item in r['evidence']['observations'])


@pytest.mark.parametrize('kind', ['angular', 'area'])
@pytest.mark.parametrize('evidence', [
    {'counts': {}}, {'counts': {'person': None}},
    {'counts': {'person': 3}, 'validity': 'unknown'},
    {'counts': {'person': 3}, 'fresh': False},
    {'counts': {'person': 3}, 'available': False},
    {'counts': {'person': 3}, 'stale': True},
])
def test_unavailable_capture_has_no_filter_verdict_and_remains_resumable(kind, evidence):
    ex, w, r = run(kind, evidence)
    assert r['status'] == 'FAILED'
    assert 'workflow_stop_filter' not in r['evidence']
    assert r['evidence']['observations'] == []
    ident = r['evidence']['scan_id']
    ex.execute = lambda *args: {'status': 'SUCCEEDED', 'evidence': {'counts': {'person': 3}}}
    done = w.resume_heading_scan(ident) if kind == 'angular' else w.resume_scan(ident)
    assert done['evidence']['workflow_stop_filter']['observed_count'] == 3
    assert done['evidence']['workflow_stop_filter']['matched'] is True

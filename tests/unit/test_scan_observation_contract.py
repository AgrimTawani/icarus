"""Embedded sensor calls must share the public contract before any motion."""
import pytest
from python.dcm.agent_tools import SCHEMAS, validate_call
from python.dcm.angular_scan import AngularScan
from python.dcm.scan_workflow import ScanWorkflow
from python.perception.classes import YOLO_CLASSES
from test_angular_scan import Executor as AngularExecutor
from test_scan_workflow import Executor as AreaExecutor


@pytest.mark.parametrize('kind', ['angular', 'area'])
def test_full_supported_class_set_passes_workflow_and_embedded_detect(kind):
    classes = sorted(YOLO_CLASSES)
    counts = {name: 0 for name in classes}
    response = {'status': 'SUCCEEDED', 'evidence': {'counts': counts}}
    ex = AngularExecutor([response]) if kind == 'angular' else AreaExecutor([response])
    original = ex.execute
    def checked(name, args):
        validate_call(name, args)
        return original(name, args)
    ex.execute = checked
    if kind == 'angular':
        args = {'step_deg': 30, 'sweep_deg': 30, 'classes': classes}
        validate_call('scan_heading', args)
        result = AngularScan(ex).scan_heading(**args)
    else:
        args = {'shape': 'square', 'coverage': 'perimeter', 'width_m': 1,
                'observation': 'detect', 'observation_count': 1, 'classes': classes}
        validate_call('scan_area', args)
        result = ScanWorkflow(ex).scan_area(**args)
    assert result['status'] == 'SUCCEEDED'
    assert result['evidence']['observations'][0]['evidence']['counts'] == counts


@pytest.mark.parametrize('bad', [
    {'observation': 'detect', 'classes': ['person', 'person']},
    {'observation': 'detect', 'classes': ['not_a_class']},
    {'observation': 'detect', 'classes': []},
    {'observation': 'detect'},
    {'observation': 'scene', 'question': ''},
    {'observation': 'scene', 'question': 'x' * 241},
    {'observation': 'scene', 'camera': 'invented_camera'},
])
@pytest.mark.parametrize('kind', ['angular', 'area'])
def test_bad_embedded_observation_rejected_before_motion_or_saved_workflow(kind, bad):
    ex = AngularExecutor() if kind == 'angular' else AreaExecutor()
    with pytest.raises(ValueError):
        if kind == 'angular':
            AngularScan(ex).scan_heading(step_deg=30, **bad)
        else:
            ScanWorkflow(ex).scan_area(shape='square', width_m=1, **bad)
    assert (ex.calls if kind == 'angular' else ex.routes) == []
    assert getattr(ex.client, '_angular_scans' if kind == 'angular' else '_scan_workflows') == {}


def test_all_public_detection_lists_share_identical_capability_contract():
    expected = SCHEMAS['detect']['properties']['classes']
    assert expected['maxItems'] == len(YOLO_CLASSES)
    for name in ('scan_heading', 'scan_area'):
        assert SCHEMAS[name]['properties']['classes'] == expected

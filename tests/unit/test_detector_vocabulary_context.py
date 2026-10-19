"""Publish sensor vocabulary once while validating against actual capabilities."""
import pytest
from python.dcm.agent_tools import ToolExecutor, decision_tools, validate_call
from test_direct_agent import FakeClient


def test_native_schemas_reference_one_context_vocabulary_without_repeating_it():
    tools=decision_tools({'detector_classes':['model-specific-label']})
    for name in ('detect','scan_heading','scan_area'):
        schema=next(t['function']['parameters'] for t in tools if t['function']['name']==name)
        assert 'enum' not in schema['properties']['classes']['items']
        assert 'context.detector_classes' in schema['properties']['classes']['description']
    validate_call('detect',{'classes':['model-specific-label']},tools=tools,
                  detector_classes=['model-specific-label'])
    with pytest.raises(ValueError):
        validate_call('detect',{'classes':['person']},tools=tools,
                      detector_classes=['model-specific-label'])


def test_unsupported_scan_class_is_rejected_before_movement(monkeypatch):
    client=FakeClient()
    monkeypatch.setattr(client,'camera_detector_classes',lambda:['model-specific-label'],raising=False)
    with pytest.raises(ValueError):
        ToolExecutor(client).execute('scan_heading',{'step_deg':30,'classes':['person']})
    assert not client.sent


def test_executor_accepts_configured_labels_outside_the_default_detector(monkeypatch):
    client=FakeClient()
    monkeypatch.setattr(client,'camera_detector_classes',lambda:['model-specific-label'],raising=False)
    monkeypatch.setattr(client,'detect',lambda **args:{'counts':{'model-specific-label':0}},raising=False)
    result=ToolExecutor(client).execute('detect',{'classes':['model-specific-label']})
    assert result['status']=='SUCCEEDED' and result['evidence']['counts']=={'model-specific-label':0}
    assert not client.sent


def test_scan_observation_uses_the_same_configured_vocabulary(monkeypatch):
    client=FakeClient();client.vehicle.armed=True;client.vehicle.landed=False
    monkeypatch.setattr(client,'camera_detector_classes',lambda:['model-specific-label'],raising=False)
    monkeypatch.setattr(client,'detect',lambda **args:{'counts':{'model-specific-label':0},'validity':'valid'},raising=False)
    result=ToolExecutor(client).execute('scan_heading',{'step_deg':30,'exit_policy':'each_observation',
        'observation':'detect','classes':['model-specific-label']})
    assert result['status']=='SUCCEEDED' and [n for n,_ in client.sent]==['Yaw']

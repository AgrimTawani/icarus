"""Native generation and dispatch must agree on configured sensor labels."""
import pytest
from jsonschema import Draft202012Validator
from python.dcm.agent_tools import ToolExecutor, decision_tools, validate_call
from test_direct_agent import FakeClient


def test_native_schemas_constrain_labels_before_generation_and_dispatch():
    tools=decision_tools({'detector_classes':['model-specific-label']})
    for name in ('detect','scan_heading','scan_area'):
        schema=next(t['function']['parameters'] for t in tools if t['function']['name']==name)
        assert schema['properties']['classes']['items']['enum'] == ['model-specific-label']
        # Inspect the advertised schema directly: validate_call must not hide
        # a missing generation constraint by adding its own enum afterwards.
        classes = schema['properties']['classes']
        assert Draft202012Validator(classes).is_valid(['model-specific-label'])
        assert not Draft202012Validator(classes).is_valid(['unsupported-label'])
        assert not Draft202012Validator(classes).is_valid(['person'])
    validate_call('detect',{'classes':['model-specific-label']},tools=tools)
    validate_call('detect',{'classes':['model-specific-label']},tools=tools,
                  detector_classes=['model-specific-label'])
    with pytest.raises(ValueError):
        validate_call('detect',{'classes':['person']},tools=tools,
                      detector_classes=['model-specific-label'])


def test_changed_backend_cannot_expand_an_already_advertised_call_contract():
    tools=decision_tools({'detector_classes':['old-label']})
    with pytest.raises(ValueError):
        validate_call('detect',{'classes':['new-label']},tools=tools,
                      detector_classes=['old-label','new-label'])
    with pytest.raises(ValueError):
        validate_call('detect',{'classes':['old-label']},tools=tools,
                      detector_classes=['new-label'])


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

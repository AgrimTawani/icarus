"""Question answerability is separate from condition truth and transport success."""
import json
import pytest
from python.perception.vlm import decode_measurement, measurement_grammar
from python.dcm.agent_tools import ToolExecutor
from test_direct_agent import FakeClient, Runtime, call, run


@pytest.mark.parametrize('value,validity', [(True,'valid'),(False,'unknown'),(None,'unknown')])
def test_description_answerability_preserves_unknown(value,validity):
    result=decode_measurement(json.dumps({'answerable':value,'answer':'Visible evidence.'}),'description')
    assert result['answerable'] is value and result['validity']==validity
    assert 'answerable' in measurement_grammar('description')


@pytest.mark.parametrize('payload',[
    {'answer':'Yes'}, {'answerable':1,'answer':'Yes'},
    {'answerable':'true','answer':'Yes'}, {'answerable':True,'answer':'Yes','extra':True},
])
def test_description_cannot_coerce_missing_or_forged_answerability(payload):
    result=decode_measurement(json.dumps(payload),'description')
    assert result['answerable'] is None and result['error_code']=='malformed_output'


def camera_client(tmp_path,monkeypatch,payload):
    client=FakeClient()
    def capture(camera):
        path=tmp_path/'capture';path.mkdir(exist_ok=True)
        return {'camera':camera,'directory':str(path)}
    monkeypatch.setattr(client,'capture_view',capture,raising=False)
    monkeypatch.setattr(client,'inspect_scene',lambda question,view,**kwargs:dict(payload),raising=False)
    return client


@pytest.mark.parametrize('payload',[
    {'answerable':False,'answer':'The requested subject is not visible.'},
    {'answerable':None,'answer':'Too little evidence.'},
    {'answer':'An untyped confident description.'},
])
def test_successful_capture_cannot_satisfy_an_unanswered_question(tmp_path,monkeypatch,payload):
    client=camera_client(tmp_path,monkeypatch,payload)
    result=ToolExecutor(client).execute('inspect_scene',{'question':'Describe the selected subject.'})
    assert result['status']=='OBSERVATION_UNAVAILABLE' and not client.sent
    assert result['evidence']['answer']==payload['answer']


def test_answerable_negative_presence_is_a_successful_observation(tmp_path,monkeypatch):
    client=camera_client(tmp_path,monkeypatch,{'answerable':True,'answer':'No subject visible.'})
    result=ToolExecutor(client).execute('inspect_scene',{'question':'Is the subject visible?'})
    assert result['status']=='SUCCEEDED' and not client.sent


def test_typed_false_visual_condition_does_not_require_positive_visibility(tmp_path,monkeypatch):
    client=camera_client(tmp_path,monkeypatch,{'found':False,'validity':'valid','answer':'No subject visible.'})
    result=ToolExecutor(client).execute('inspect_scene',
        {'question':'Is the subject visible?','question_type':'condition'})
    assert result['status']=='SUCCEEDED' and result['evidence']['found'] is False
    assert not client.sent


def test_unknown_visual_condition_cannot_be_coerced_to_negative_branch(tmp_path,monkeypatch):
    client=camera_client(tmp_path,monkeypatch,{'found':None,'validity':'unknown','answer':'Cannot decide.'})
    result=ToolExecutor(client).execute('inspect_scene',
        {'question':'Is the subject visible?','question_type':'condition'})
    assert result['status']=='OBSERVATION_UNAVAILABLE' and not client.sent



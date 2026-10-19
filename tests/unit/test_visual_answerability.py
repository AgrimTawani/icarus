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


def test_agent_cannot_finish_completed_after_unanswerable_inspection(tmp_path,monkeypatch):
    client=camera_client(tmp_path,monkeypatch,{'answerable':False,'answer':'View is inadequate.'})
    runtime=Runtime(call('inspect_scene',question='Describe the subject.',question_type='description'),
        call('finish',outcome='completed',message='Unsupported success.'),
        call('finish',outcome='incomplete',message='The camera view could not answer the question.'))
    result=run(client,runtime)
    assert result['status']=='incomplete'
    assert result['unresolved_tool_failures'][0]['action']=='inspect_scene'
    assert 'UNRESOLVED_TOOL_FAILURES' in str(runtime.inputs[2])
    assert not client.sent


def test_camera_clarification_requires_new_usable_evidence_before_completion(tmp_path,monkeypatch):
    from python.dcm.fly import AgentSession
    client=camera_client(tmp_path,monkeypatch,{})
    replies=iter([{'answerable':False,'answer':'Subject outside this view.'},
                  {'answerable':True,'answer':'A brown surface is visible.'}])
    cameras=[]
    def inspect(question,view,**kwargs):
        cameras.append(view['camera']);return next(replies)
    monkeypatch.setattr(client,'inspect_scene',inspect)
    session=AgentSession('Describe the selected subject.')
    first=run(client,Runtime(call('inspect_scene',question='Describe the subject.',question_type='description',camera='forward_d435i'),
        call('ask_user',question='May I use the other camera?')),session=session)
    assert first['status']=='awaiting_user' and session.unresolved_tool_failures
    session.answer('Use the downward camera.')
    assert session.unresolved_tool_failures
    final=run(client,Runtime(call('inspect_scene',question='Describe the subject.',question_type='description',camera='downward_brio'),
        call('finish',outcome='completed',message='Visible surface described.')),session=session)
    assert final['status']=='completed' and not session.unresolved_tool_failures
    assert cameras==['forward_d435i','downward_brio'] and not client.sent


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


def test_successful_condition_cannot_erase_an_unanswered_description():
    from python.dcm.fly import AgentSession
    session=AgentSession('Describe a target.')
    session.remember_tool_result('inspect_scene',{'question_type':'description'},
        {'status':'OBSERVATION_UNAVAILABLE','message':'Subject unseen.'})
    session.remember_tool_result('inspect_scene',{'question_type':'condition'},
        {'status':'SUCCEEDED','evidence':{'found':False}})
    assert len(session.unresolved_tool_failures)==1
    session.remember_tool_result('inspect_scene',{'question_type':'description'},
        {'status':'SUCCEEDED','evidence':{'answerable':True}})
    assert not session.unresolved_tool_failures

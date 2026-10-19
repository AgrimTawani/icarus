"""Saved navigation poses are retrieved explicitly, not visual object context."""
import json
from python.dcm.destinations import catalogue_context
from python.dcm.fly import AgentSession, fly_mission
from test_direct_agent import FakeClient, Runtime, call, context, run


def catalogue(tmp_path,monkeypatch):
    path=tmp_path/'poses.json'
    path.write_text(json.dumps({'coordinate_frame':'home_relative_ned','entries':[
        {'name':'Saved view A','north_m':2,'east_m':-3,'role':'observation_pose','heading_deg':45}]}))
    monkeypatch.setenv('ICARUS_WAYPOINTS_FILE',str(path))
    return path


def test_default_index_omits_names_coordinates_and_filesystem_path(tmp_path,monkeypatch):
    path=catalogue(tmp_path,monkeypatch)
    index=catalogue_context()
    assert index['count']==1 and index['entries']==[] and not index['loaded']
    assert 'Saved view A' not in json.dumps(index) and str(path) not in json.dumps(index)
    loaded=catalogue_context(True)
    assert loaded['entries'][0]['name']=='Saved view A' and loaded['loaded']
    assert loaded['entries'][0]['role']=='observation_pose'


def test_explicit_retrieval_persists_only_in_the_current_mission(tmp_path,monkeypatch):
    catalogue(tmp_path,monkeypatch)
    def capture(client,include_waypoints=False,**kwargs):
        return {**context(client),'registered_waypoints':catalogue_context(include_waypoints)}
    monkeypatch.setattr('python.dcm.agent_context.capture_context',capture)
    client=FakeClient();session=AgentSession('List the saved navigation poses.')
    runtime=Runtime(call('get_context',include_waypoints=True),
                    call('finish',outcome='completed',message='Retrieved saved poses.'))
    result=fly_mission(client,runtime,session.mission,session=session,context_provider=capture,echo=lambda *a:None)
    assert result['status']=='completed' and not client.sent
    assert 'Saved view A' not in json.dumps(runtime.inputs[0])
    assert 'Saved view A' in json.dumps(runtime.inputs[1])
    assert session.include_waypoints
    fresh=AgentSession('Observe the current scene.')
    assert not fresh.include_waypoints

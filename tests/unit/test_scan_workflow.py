"""Deterministic geometry/observation boundaries; no mission-text templates."""
import copy
from types import SimpleNamespace
import pytest
from python.dcm.scan_workflow import ScanWorkflow, prepare_pattern


def state():
    return {'local_position_ned':{'north_m':10,'east_m':20,'origin_id':'local'},
            'relative_altitude_m':3,'attitude':{'yaw_rad':0}}


class Executor:
    def __init__(self, observations=()):
        self.value=state();self.client=SimpleNamespace(state=lambda:self.value)
        self.routes=[];self.observations=iter(observations);self.stopped=False;self.failure=None
    def interrupted(self): return self.stopped
    def _flight(self,name,args,expected_origin_id=None):
        assert name=='execute_route'
        if expected_origin_id!=self.value['local_position_ned']['origin_id']:raise ValueError('origin changed')
        self.routes.append(copy.deepcopy(args))
        p=args['points'][-1]
        self.value['local_position_ned'].update(north_m=p['north_m'],east_m=p['east_m'])
        return {'status':self.failure or 'SUCCEEDED'}
    def execute(self,name,args):
        assert name in ('detect','inspect_scene')
        return next(self.observations,{'status':'SUCCEEDED','evidence':{'counts':{'person':0}}})


def test_pattern_anchor_stays_fixed_and_right_first():
    anchor,points=prepare_pattern(state(),{'shape':'square','width_m':1})
    assert anchor['north_m']==10
    assert [(p['north_m'],p['east_m']) for p in points]==[(10,20),(10,21),(11,21),(11,20),(10,20)]


def test_native_pattern_is_one_route_not_one_model_call_per_corner():
    ex=Executor();r=ScanWorkflow(ex).fly_pattern(shape='rectangle',width_m=2,length_m=3)
    assert len(ex.routes)==1 and len(ex.routes[0]['points'])==5
    assert r['evidence']['backend']=='ardupilot_auto'
    assert r['evidence']['sensor_coverage_verified'] is False


def test_detection_condition_pauses_and_resumes_original_geometry_after_nested_inspection():
    ex=Executor([{'status':'SUCCEEDED','evidence':{'counts':{'person':1}}}])
    workflow=ScanWorkflow(ex)
    first=workflow.scan_area(shape='square',coverage='perimeter',width_m=2,
        observation='detect',classes=['person'],stop_on_detection='person')
    assert first['evidence']['event']=='condition_met' and not first['workflow_complete']
    assert len(ex.routes)==1
    # A nested inspection branch may move the aircraft before scan resumption.
    ex.value['local_position_ned'].update(north_m=50,east_m=50)
    done=workflow.resume_scan(first['evidence']['scan_id'])
    assert done['workflow_complete'] and done['evidence']['completed_stations']==4
    assert ex.routes[-1]['points'][-1]['north_m']==10
    assert ex.routes[-1]['points'][-1]['east_m']==20
    count=len(ex.routes)
    assert workflow.resume_scan(first['evidence']['scan_id'])['workflow_complete']
    assert len(ex.routes)==count


def test_no_detection_does_all_stations_without_agent_calls():
    ex=Executor();r=ScanWorkflow(ex).scan_area(shape='square',width_m=1,observation='detect',classes=['person'],return_on='completion')
    assert r['workflow_complete'] and len(r['evidence']['observations'])==4
    assert len(ex.routes)==5  # four distinct corners, then close the perimeter
    positions=[(o['position']['north_m'],o['position']['east_m']) for o in r['evidence']['observations']]
    assert len(set(positions))==4


def test_observation_chunks_keep_tolerance_from_full_small_pattern():
    ex=Executor();r=ScanWorkflow(ex).scan_area(shape='square',width_m=.1,
        observation='detect',classes=['person'],return_on='completion')
    assert r['workflow_complete']
    assert all(p['acceptance_radius_m'] < .05 for route in ex.routes for p in route['points'])


def test_failed_observation_does_not_skip_station_on_resume():
    ex=Executor([{'status':'FAILED','message':'camera unavailable'}]);w=ScanWorkflow(ex)
    r=w.scan_area(shape='square',width_m=1,return_on='completion')
    assert r['status']=='FAILED' and r['evidence']['completed_stations']==0
    r2=w.resume_scan(r['evidence']['scan_id'])
    assert r2['workflow_complete']
    assert ex.routes[0]['points']==ex.routes[1]['points']


def test_default_scan_yields_each_view_for_semantic_branching():
    ex=Executor();w=ScanWorkflow(ex)
    r=w.scan_area(shape='square',width_m=1,question='Describe this view')
    assert r['evidence']['event']=='observation_ready' and len(ex.routes)==1
    r=w.resume_scan(r['evidence']['scan_id'])
    assert r['evidence']['completed_stations']==2 and len(ex.routes)==2


def test_unknown_detection_is_not_a_negative_condition():
    ex=Executor([{'status':'SUCCEEDED','evidence':{'counts':{}}}]);w=ScanWorkflow(ex)
    r=w.scan_area(shape='square',width_m=1,observation='detect',classes=['person'],stop_on_detection='person')
    assert r['status']=='FAILED' and r['evidence']['completed_stations']==0
    assert len(ex.routes)==1


def test_failed_flight_has_no_successful_observations():
    ex=Executor();ex.failure='FAILED'
    r=ScanWorkflow(ex).scan_area(shape='square',width_m=1)
    assert r['status']=='FAILED' and r['evidence']['observations']==[]


def test_cancel_before_scan_does_not_dispatch():
    ex=Executor();ex.stopped=True
    r=ScanWorkflow(ex).scan_area(shape='square',width_m=1)
    assert r['status']=='CANCELLED' and not ex.routes


def test_origin_change_rejects_resume():
    ex=Executor([{'status':'SUCCEEDED','evidence':{'counts':{'person':1}}}]);w=ScanWorkflow(ex)
    r=w.scan_area(shape='square',width_m=1,observation='detect',classes=['person'],stop_on_detection='person')
    ex.value['local_position_ned']['origin_id']='reset'
    with pytest.raises(ValueError,match='origin changed'):w.resume_scan(r['evidence']['scan_id'])


@pytest.mark.parametrize('kwargs',[{'observation':'detect'},{'observation':'detect','classes':['building']},
    {'classes':['person'],'stop_on_detection':'person'}, {'coverage':'area'}])
def test_incomplete_contract_does_not_fly(kwargs):
    ex=Executor()
    with pytest.raises(ValueError):ScanWorkflow(ex).scan_area(shape='square',width_m=1,**kwargs)
    assert not ex.routes


def test_area_finish_at_match_stops_route_and_cannot_resume():
    ex=Executor([{'status':'SUCCEEDED','evidence':{'counts':{'person':2}}}])
    workflow=ScanWorkflow(ex)
    result=workflow.scan_area(shape='square',width_m=2,observation='detect',
        classes=['person'],stop_on_detection='person',minimum_count=2,exit_policy='finish_at_match')
    assert result['workflow_complete'] and result['evidence']['event']=='condition_met'
    assert not result['evidence']['coverage_complete']
    assert len(ex.routes)==1
    workflow.resume_scan(result['evidence']['scan_id'])
    assert len(ex.routes)==1


def test_area_explicit_pause_allows_resuming_after_match():
    ex=Executor([{'status':'SUCCEEDED','evidence':{'counts':{'person':1}}}])
    workflow=ScanWorkflow(ex)
    result=workflow.scan_area(shape='square',width_m=2,observation='detect',
        classes=['person'],stop_on_detection='person',exit_policy='pause_at_match')
    assert not result['workflow_complete']
    done=workflow.resume_scan(result['evidence']['scan_id'])
    assert done['workflow_complete'] and done['evidence']['coverage_complete']
    assert len(ex.routes)==5


@pytest.mark.parametrize('extra', [
    {'exit_policy':'finish_at_match'},
    {'exit_policy':'pause_at_match','stop_on_detection':'car','classes':['person']},
    {'exit_policy':'complete_coverage','stop_on_detection':'person','classes':['person']},
    {'exit_policy':'each_observation','minimum_count':2},
    {'exit_policy':'finish_at_match','stop_on_detection':'person','classes':['person'],'observation':'scene'},
])
def test_conflicting_exit_policies_fail_before_any_route(extra):
    ex=Executor()
    with pytest.raises(ValueError):
        ScanWorkflow(ex).scan_area(shape='square',width_m=2,**extra)
    assert not ex.routes

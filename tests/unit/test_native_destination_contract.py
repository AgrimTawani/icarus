"""The generated JSON contract must include the executor's prerequisites."""
import copy
import pytest
from jsonschema import Draft202012Validator
from python.dcm.agent_tools import decision_tools,validate_call
from python.dcm.destinations import canonical_destination_arguments

CONTEXT={'targets':[{'target_id':'measured'}],'registered_waypoints':{
    'entries':[{'name':'Center A','role':'orbit_center'}]}}
TOOLS=decision_tools(CONTEXT)


def schema(name):
    return next(t['function']['parameters'] for t in TOOLS if t['function']['name']==name)


CASES=[
 ('goto',{'type':'coordinates','coordinate_frame':'home_relative_ned','north_m':1,'east_m':2,'altitude_agl_m':3}),
 ('goto',{'type':'coordinates','coordinate_frame':'local_ned','origin_id':'measured-origin','north_m':1,'east_m':2,'altitude_agl_m':3}),
 ('goto',{'type':'home'}),('goto',{'type':'launch'}),
 ('goto',{'type':'named_waypoint','waypoint_name':'Center A'}),
 ('approach_target',{'type':'visible_object','visual_query':'visible structure','view':'north','standoff_m':3}),
 ('approach_target',{'type':'registered_target','target_id':'measured','standoff_m':3}),
 ('orbit',{'type':'coordinates','coordinate_frame':'home_relative_ned','north_m':1,'east_m':2,'altitude_agl_m':3,'radius_m':4}),
 ('orbit',{'type':'home','radius_m':4,'altitude_agl_m':3}),
 ('orbit',{'type':'launch','radius_m':4,'altitude_agl_m':3}),
 ('orbit',{'type':'named_waypoint','waypoint_name':'Center A','radius_m':4,'altitude_agl_m':3}),
 ('orbit',{'type':'visible_object','visual_query':'visible structure','view':'north','standoff_m':3}),
 ('orbit',{'type':'registered_target','target_id':'measured','standoff_m':3}),
 ('overfly_target',{'type':'visible_object','visual_query':'visible structure','view':'north'}),
 ('overfly_target',{'type':'registered_target','target_id':'measured'}),
]


@pytest.mark.parametrize('name,destination',CASES)
def test_advertised_variant_satisfies_real_dispatch_requirements(name,destination):
    args={'destination':destination}
    if name=='orbit':args['camera']='none'
    Draft202012Validator(schema(name)).validate(args)
    validate_call(name,args,tools=TOOLS)


@pytest.mark.parametrize('name,destination',[
 ('approach_target',{'type':'visible_object','visual_query':'structure','view':'north'}),
 ('goto',{'type':'coordinates','coordinate_frame':'local_ned','north_m':1,'east_m':2,'altitude_agl_m':3}),
 ('orbit',{'type':'registered_target','target_id':'measured','radius_m':4}),
 ('orbit',{'type':'home','radius_m':4}),
])
def test_missing_requirements_are_rejected_by_generation_schema(name,destination):
    args={'destination':destination}
    if name=='orbit':args['camera']='none'
    assert not Draft202012Validator(schema(name)).is_valid(args)


def test_legacy_keyword_placement_is_a_lossless_adapter_not_a_model_requirement():
    args={'destination':{'type':'registered_target','target_id':'measured'},
          'standoff_m':4,'object_position':'standoff'}
    canonical=canonical_destination_arguments('approach_target',args)
    assert canonical=={'destination':{'type':'registered_target','target_id':'measured','standoff_m':4}}
    validate_call('approach_target',args,tools=TOOLS)
    assert 'standoff_m' in args and args['object_position']=='standoff'


def test_conflicting_duplicate_geometry_is_never_silently_overridden():
    with pytest.raises(ValueError,match='once'):
        validate_call('approach_target',{'destination':{'type':'registered_target','target_id':'measured','standoff_m':3},'standoff_m':8},tools=TOOLS)


def test_goto_does_not_advertise_ambiguous_object_relative_motion():
    kinds={v['properties']['type']['const'] for v in schema('goto')['properties']['destination']['oneOf']}
    assert kinds=={'coordinates','home','launch','named_waypoint'}
    for name in ('approach_target','overfly_target'):
        assert {v['properties']['type']['const'] for v in schema(name)['properties']['destination']['oneOf']}=={'visible_object','registered_target'}


def test_saved_pose_orientation_is_atomic_and_can_be_changed_by_a_later_heading_action():
    args={'destination':{'type':'named_waypoint','waypoint_name':'Center A','heading_deg':0}}
    assert not Draft202012Validator(schema('goto')).is_valid(args)
    with pytest.raises(ValueError):validate_call('goto',args,tools=TOOLS)
    validate_call('face_heading',{'heading_deg':0},tools=TOOLS)

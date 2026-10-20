"""Current physical frame conversion never interprets the operator's mission."""
import copy
import math
import pytest
from python.dcm.camera_geometry import target_reference_feedback


def state(north=0, east=0, down=-5, yaw=0, origin='map'):
    return {'local_position_ned': {'north_m': north, 'east_m': east, 'down_m': down, 'origin_id': origin},
            'attitude': {'roll_rad': 0, 'pitch_rad': 0, 'yaw_rad': math.radians(yaw)}}


def targets(north=10, east=0, top=-5, origin='map'):
    return [{'target_id': 'registered', 'origin_id': origin, 'reference_available': True,
             'reference_age_ms': 24000, 'current_observation_available': False,
             'geometry': {'center': {'north_m': north, 'east_m': east, 'origin_id': origin},
                          'top_down_m': top}}]


@pytest.mark.parametrize('yaw,forward,right', [(0,10,0),(90,0,-10),(180,-10,0),(270,0,10)])
def test_map_reference_offsets_follow_current_heading(yaw, forward, right):
    raw=targets();before=copy.deepcopy(raw)
    result=target_reference_feedback(state(yaw=yaw),raw,['forward_d435i'])
    relative=result[0]['relative_reference']
    assert relative['forward_m']==pytest.approx(forward)
    assert relative['right_m']==pytest.approx(right)
    assert relative['horizontal_center_distance_m']==10
    assert relative['current_surface_range_measurement'] is False
    assert result[0]['reference_age_ms']==24000 and result[0]['current_observation_available'] is False
    assert result[0]['camera_reference_projection']['forward_d435i']['image_proof'] is False
    assert raw==before


def test_reference_below_aircraft_requires_more_than_yaw_for_fixed_forward_camera():
    item=target_reference_feedback(state(north=10,down=-9),targets(top=-6),
                                   ['forward_d435i','downward_brio'])[0]
    assert item['relative_reference']['horizontal_center_distance_m']==0
    assert item['relative_reference']['up_m']==-3
    front=item['camera_reference_projection']['forward_d435i']
    assert front['in_frame'] is False and front['yaw_alone_can_resolve'] is False
    assert item['camera_reference_projection']['downward_brio']['in_frame'] is True


def test_reference_moves_behind_vehicle_after_passing_it_without_relabeling_object():
    raw=targets(north=0)
    result=target_reference_feedback(state(north=10),raw,['forward_d435i'])[0]
    assert result['relative_reference']['forward_m']==-10
    assert result['camera_reference_projection']['forward_d435i']['in_frame'] is False


def test_missing_height_is_unknown_not_ground_or_zero_altitude():
    item=target_reference_feedback(state(),targets(top=None),['forward_d435i'])[0]
    assert item['relative_reference']['available'] is True
    assert item['relative_reference']['up_m'] is None
    assert item['camera_reference_projection']['forward_d435i']['available'] is False
    assert item['camera_reference_projection']['forward_d435i']['in_frame'] is None


def test_changed_or_unknown_origin_cannot_supply_body_offsets():
    for origin in ('other',''):
        item=target_reference_feedback(state(origin=origin),targets(),['forward_d435i'])[0]
        assert item['relative_reference']['available'] is False
        assert 'forward_m' not in item['relative_reference']

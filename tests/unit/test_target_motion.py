import math
from copy import deepcopy

import pytest

from python.dcm.agent_tools import validate_call, SCHEMAS, TARGET_FLIGHT
from python.dcm.target_motion import TargetMotion, OrbitMeasurements


class Client:
    def __init__(self, north=0):
        self.north = north
        self.east = 0
        self.alt = 4
        self.lost = False
        self.partial = False
        self.calls = []

    def state(self):
        return {'local_position_ned': {'north_m': self.north, 'east_m': self.east,
                                      'origin_id': 'test'},
                'relative_altitude_m': self.alt, 'attitude': {'yaw_rad': 0}}

    def refresh_target(self, target_id):
        return {'origin_id': 'test', 'found': not self.lost, 'reason': 'association lost' if self.lost else '',
                'surface_range_m': abs(10-self.north),
                'surface_point': {'north_m': 10, 'east_m': 0},
                'geometry': {'orbit_usable': not self.partial,
                             'center': {'north_m': 12, 'east_m': 0}, 'radius_m': 2},
                'footprint': {'height_m': 6, 'height_clipped': False}}

    def flight(self, name, args, **kw):
        # Intermediate target-facing arc legs (0.5 m acceptance) are kept apart
        # so call sequences still show each meaningful move once.
        legs = self.__dict__.setdefault('arc_legs', [])
        (legs if name == 'goto' and kw.get('acceptance_radius_m') == 0.5 else self.calls).append(
            (name, deepcopy(args), kw))
        if name == 'goto':
            self.north, self.east, self.alt = args['north_m'], args['east_m'], args['altitude_agl_m']
        return {'status': 'SUCCEEDED', 'state': self.state()}


def test_approach_uses_surface_distance_and_moves_toward_object():
    c = Client()
    result = TargetMotion(c, c.flight).execute('approach_target', {'target_id': 'a', 'standoff_m': 3})
    assert result['status'] == 'SUCCEEDED'
    assert c.north == pytest.approx(7)
    positions = [args['north_m'] for name, args, kw in c.calls]
    assert positions == sorted(positions)
    assert all(kw['runtime_target'] for name, args, kw in c.calls)
    assert result['evidence']['final_surface_range_m'] == 3


def test_approach_recedes_only_if_inside_standoff():
    c = Client(north=8)
    result = TargetMotion(c, c.flight).execute('approach_target', {'target_id': 'a', 'standoff_m': 3})
    assert result['status'] == 'SUCCEEDED' and c.north == 7


def test_lost_target_holds_without_moving():
    c = Client(); c.lost = True
    result = TargetMotion(c, c.flight).execute('approach_target', {'target_id': 'a', 'standoff_m': 3})
    assert result['status'] == 'FAILED'
    assert [x[0] for x in c.calls] == ['hold']


class AnchorClient(Client):
    def __init__(self):
        super().__init__()
        self.reference = self.refresh_target('a')
        self.reference.update(reference_available=True, found=False,
                              current_observation_available=False)
        self.lost = True

    def target_anchor(self, target_id):
        return deepcopy(self.reference)


def test_overflight_can_use_stationary_map_geometry_outside_the_current_sensor_view():
    c = AnchorClient()
    result = TargetMotion(c, c.flight).execute('overfly_target', {'target_id': 'a'})
    assert result['status'] == 'SUCCEEDED'
    assert [name for name, _, _ in c.calls] == ['goto', 'goto']
    assert c.north == 12 and c.alt == 9
    assert result['evidence']['overhead_waypoint_reached'] is True
    assert result['evidence']['clearance_above_measured_top_m'] == 3
    assert result['evidence']['target_observation_available_before_maneuver'] is False
    assert 'camera inspection is separate' in result['message']


def test_orbit_can_dispatch_from_a_retained_anchor_but_never_fakes_coverage():
    c = AnchorClient()
    result = TargetMotion(c, c.flight).execute('orbit_target', {'target_id': 'a', 'standoff_m': 3})
    assert any(name == 'orbit' for name, _, _ in c.calls)
    # A mock that never flies cannot prove a completed circle.
    assert result['status'] != 'SUCCEEDED'
    assert result['evidence']['camera_visibility_verified'] is False


def test_approach_still_requires_live_surface_distance_even_with_a_static_anchor():
    c = AnchorClient()
    result = TargetMotion(c, c.flight).execute('approach_target', {'target_id': 'a', 'standoff_m': 3})
    assert result['status'] == 'FAILED'
    assert [name for name, _, _ in c.calls] == ['hold']


def test_overflight_default_is_published_and_shared_with_controller_policy(monkeypatch):
    from python.dcm.agent_tools import decision_tools
    defaults={'overflight_clearance_m':4.5,'object_standoff_m':7}
    monkeypatch.setattr('python.dcm.target_motion.operating_defaults',lambda:defaults)
    tool=next(t['function'] for t in decision_tools({'defaults':defaults})
              if t['function']['name']=='overfly_target')
    assert tool['parameters']['properties']['clearance_above_m']['default']==4.5
    c=AnchorClient()
    result=TargetMotion(c,c.flight).execute('overfly_target',{'target_id':'a'})
    assert result['status']=='SUCCEEDED' and c.alt==10.5
    c=AnchorClient()
    result=TargetMotion(c,c.flight).execute('overfly_target',{'target_id':'a','clearance_above_m':1})
    assert result['status']=='SUCCEEDED' and c.alt==7  # Explicit operator value is retained.


def test_operator_stop_does_not_override_recovery():
    c = Client()
    result = TargetMotion(c, c.flight, lambda: True).execute('approach_target', {'target_id': 'a', 'standoff_m': 3})
    assert result['status'] == 'CANCELLED' and not c.calls


def test_partial_footprint_cannot_be_reported_as_a_measured_orbit():
    c = Client(); c.partial = True
    result = TargetMotion(c, c.flight).execute('orbit_target', {'target_id': 'a', 'standoff_m': 3})
    assert result['status'] == 'FAILED'
    assert [x[0] for x in c.calls] == ['hold']


def test_overflight_climbs_before_translating_and_does_not_land():
    c = Client()
    result = TargetMotion(c, c.flight).execute('overfly_target', {'target_id': 'a', 'clearance_above_m': 3})
    assert result['status'] == 'SUCCEEDED'
    assert [(x[1]['north_m'], x[1]['altitude_agl_m']) for x in c.calls] == [(0,9),(12,9)]
    assert result['evidence']['landing_assessed'] is False


def test_orbit_measurements_cover_clockwise_and_counterclockwise():
    for clockwise in (True, False):
        m = OrbitMeasurements({'north_m': 0, 'east_m': 0}, clockwise)
        for i in range(361):
            a = math.radians(i) * (1 if clockwise else -1)
            m.observe({'local_position_ned': {'north_m': 5*math.cos(a), 'east_m': 5*math.sin(a), 'origin_id':'x'},
                       'relative_altitude_m': 6, 'attitude': {'yaw_rad': a+math.pi}})
        result = m.result()
        assert result['angular_coverage_deg'] == 360
        assert result['maximum_heading_error_deg'] == 0
        assert result['minimum_center_radius_m'] == 5


def test_target_tools_accept_a_registered_target():
    for name in TARGET_FLIGHT:
        validate_call(name, {'target_id': 'a', **({'standoff_m': 3} if name != 'overfly_target' else {})})


def test_detector_schema_is_actual_capability():
    validate_call('detect', {'classes': ['person']})
    with pytest.raises(ValueError):
        validate_call('detect', {'classes': ['building']})


class AcquisitionClient(Client):
    def __init__(self, succeed_after=1):
        super().__init__()
        self.partial = True
        self.succeed_after = succeed_after
        self.captures = []
        self.cancel = False
        self.capture_identity = 'a'
        self.capture_found = True
        self.capture_error = None
        self.cancel_after_capture = False
        self.cancel_after_move = False
        self.views = [
            {'north_m': 3, 'east_m': -6, 'origin_id': 'test', 'face_observed_midpoint_heading_deg': 40, 'observation_standoff_m': 4},
            {'north_m': 3, 'east_m': 4, 'origin_id': 'test', 'face_observed_midpoint_heading_deg': 320, 'observation_standoff_m': 4}]

    def refresh_target(self, target_id):
        target = super().refresh_target(target_id)
        target.update(target_id=target_id, description='the observed object')
        target['geometry']['measurement_view_candidates'] = deepcopy(self.views)
        return target

    def localize_target(self, description, *, new_target):
        self.captures.append((description, new_target))
        if self.capture_error:
            raise self.capture_error
        self.partial = len(self.captures) < self.succeed_after
        self.cancel = self.cancel_after_capture
        return {'found': self.capture_found, 'target_id': self.capture_identity,
                'reason': None if self.capture_found else 'insufficient depth'}

    def flight(self, name, args, **kw):
        result = super().flight(name, args, **kw)
        if name == 'goto':
            self.cancel = self.cancel_after_move
        return result


def fake_measured_orbit(monkeypatch):
    # Isolate acquisition orchestration; real angular accumulation is covered
    # by test_orbit_measurements_cover_clockwise_and_counterclockwise above.
    class Measurements:
        samples = 2
        def __init__(self, *args):
            pass
        def observe(self, state):
            pass
        def result(self):
            return {'angular_coverage_deg': 360}
    monkeypatch.setattr('python.dcm.target_motion.OrbitMeasurements', Measurements)


def test_orbit_acquires_closest_measured_side_view_and_preserves_identity(monkeypatch):
    fake_measured_orbit(monkeypatch)
    c = AcquisitionClient()
    result = TargetMotion(c, c.flight).execute('orbit_target', {'target_id': 'a', 'standoff_m': 3})
    assert result['status'] == 'SUCCEEDED'
    assert [call[0] for call in c.calls] == ['goto', 'orbit']
    move = c.calls[0]
    assert move[1]['north_m'] == 3 and move[1]['east_m'] == 4
    assert move[1]['heading_deg'] == 320 and move[1]['altitude_agl_m'] == 4
    assert move[2]['expected_origin_id'] == 'test'
    assert 'minimum_clearance_m' not in move[1]  # retain the independent API policy
    assert c.captures == [('the observed object', False)]
    assert [step['phase'] for step in result['evidence']['geometry_acquisition']] == ['move_to_measurement_view', 'localize_same_target']
    assert result['evidence']['camera_visibility_verified'] is False


def test_partial_geometry_after_two_views_stops_without_orbit():
    c = AcquisitionClient(succeed_after=3)
    result = TargetMotion(c, c.flight).execute('orbit_target', {'target_id': 'a', 'standoff_m': 3})
    assert result['status'] == 'FAILED'
    assert 'two additional' in result['message']
    assert [call[0] for call in c.calls] == ['goto', 'goto', 'hold']
    assert len(c.captures) == 2
    assert c.calls[0][1]['east_m'] != c.calls[1][1]['east_m']


def test_blocked_orbit_does_not_claim_acquisition_movement_never_happened(monkeypatch):
    fake_measured_orbit(monkeypatch)
    c = AcquisitionClient()
    def flight(name, args, **kwargs):
        if name == 'orbit':
            return {'status': 'FAILED', 'requires_user_choice': True, 'question': 'Choose another camera?',
                    'evidence': {'no_flight_executed': True}}
        return c.flight(name, args, **kwargs)
    result = TargetMotion(c, flight).execute('orbit_target', {'target_id': 'a', 'standoff_m': 3})
    assert c.calls[0][0] == 'goto'
    assert result['requires_user_choice']
    assert result['evidence']['orbit_not_dispatched'] is True
    assert 'no_flight_executed' not in result['evidence']


@pytest.mark.parametrize('fault', ['new_identity', 'not_found', 'exception'])
def test_geometry_acquisition_failure_keeps_history_and_does_not_orbit(fault):
    c = AcquisitionClient()
    if fault == 'new_identity':
        c.capture_identity = 'different'
    elif fault == 'not_found':
        c.capture_found = False
    else:
        c.capture_error = RuntimeError('camera unavailable')
    result = TargetMotion(c, c.flight).execute('orbit_target', {'target_id': 'a', 'standoff_m': 3})
    assert result['status'] == 'FAILED'
    assert [call[0] for call in c.calls] == ['goto', 'hold']
    assert len(result['evidence']['geometry_acquisition']) == 2


@pytest.mark.parametrize('when', ['before_move', 'after_move', 'after_capture'])
def test_operator_interruption_never_overrides_recovery_during_acquisition(when):
    c = AcquisitionClient()
    c.cancel = when == 'before_move'
    c.cancel_after_move = when == 'after_move'
    c.cancel_after_capture = when == 'after_capture'
    result = TargetMotion(c, c.flight, lambda: c.cancel).execute('orbit_target', {'target_id': 'a', 'standoff_m': 3})
    assert result['status'] == 'CANCELLED'
    assert all(call[0] not in ('orbit', 'hold') for call in c.calls)
    assert len(c.captures) == (1 if when == 'after_capture' else 0)


def test_invalid_or_repeated_view_candidates_do_not_authorize_movement():
    c = AcquisitionClient()
    c.views = [
        {'north_m': 3, 'east_m': 4, 'origin_id': 'other', 'face_observed_midpoint_heading_deg': 30},
        {'north_m': 0, 'east_m': 0, 'origin_id': 'test', 'face_observed_midpoint_heading_deg': 30},
        {'north_m': float('nan'), 'east_m': 0, 'origin_id': 'test', 'face_observed_midpoint_heading_deg': 30}]
    result = TargetMotion(c, c.flight).execute('orbit_target', {'target_id': 'a', 'standoff_m': 3})
    assert result['status'] == 'FAILED'
    assert [call[0] for call in c.calls] == ['hold']
    assert not c.captures


def test_second_distinct_view_can_complete_geometry(monkeypatch):
    fake_measured_orbit(monkeypatch)
    c = AcquisitionClient(succeed_after=2)
    result = TargetMotion(c, c.flight).execute('orbit_target', {'target_id': 'a', 'standoff_m': 3})
    assert result['status'] == 'SUCCEEDED'
    assert [call[0] for call in c.calls] == ['goto', 'goto', 'orbit']
    assert len(c.captures) == 2


def test_measurement_move_failure_does_not_capture_or_orbit():
    c = AcquisitionClient()
    def fail_move(name, args, **kw):
        c.calls.append((name, args, kw))
        return {'status': 'FAILED', 'message': 'clearance unavailable'}
    result = TargetMotion(c, fail_move).execute('orbit_target', {'target_id': 'a', 'standoff_m': 3})
    assert result['status'] == 'FAILED'
    assert result['message'] == 'clearance unavailable'
    assert [call[0] for call in c.calls] == ['goto']
    assert not c.captures


def test_origin_change_during_capture_stops_before_orbit():
    c = AcquisitionClient()
    original = c.refresh_target
    def refreshed(target_id):
        target = original(target_id)
        if c.captures:
            target['origin_id'] = 'reset-origin'
        return target
    c.refresh_target = refreshed
    result = TargetMotion(c, c.flight).execute('orbit_target', {'target_id': 'a', 'standoff_m': 3})
    assert result['status'] == 'FAILED'
    assert 'origin changed' in result['message']
    assert [call[0] for call in c.calls] == ['goto', 'hold']


def test_larger_requested_standoff_recomputes_views_from_measured_face(monkeypatch):
    fake_measured_orbit(monkeypatch)
    c = AcquisitionClient()
    original = c.refresh_target
    def with_hull(target_id):
        target = original(target_id)
        target['geometry']['observed_hull'] = [
            {'north_m': 10, 'east_m': -2, 'origin_id': 'test'},
            {'north_m': 10, 'east_m': 2, 'origin_id': 'test'}]
        return target
    c.refresh_target = with_hull
    result = TargetMotion(c, c.flight).execute('orbit_target', {'target_id': 'a', 'standoff_m': 10})
    assert result['status'] == 'SUCCEEDED'
    move = c.calls[0][1]
    assert move['north_m'] == 0 and abs(move['east_m']) == 12
    assert 'minimum_clearance_m' not in move
    assert result['evidence']['geometry_acquisition'][0]['observation_standoff_m'] == 10


def test_larger_standoff_without_measured_hull_cannot_use_closer_default_view():
    c = AcquisitionClient()
    result = TargetMotion(c, c.flight).execute('orbit_target', {'target_id': 'a', 'standoff_m': 10})
    assert result['status'] == 'FAILED'
    assert [call[0] for call in c.calls] == ['hold']
    assert not c.captures


def test_orbit_reserves_controller_error_without_lowering_clearance(monkeypatch):
    fake_measured_orbit(monkeypatch)
    c = Client()
    result = TargetMotion(c, c.flight).execute('orbit_target', {'target_id': 'a', 'standoff_m': 3})
    orbit = next(args for name, args, _ in c.calls if name == 'orbit')
    assert orbit['radius_m'] == 6  # measured radius2 + minimum3 + tracking reserve1
    assert 'minimum_clearance_m' not in orbit
    envelope = result['evidence']['orbit_envelope']
    assert envelope['requested_minimum_surface_clearance_m'] == 3
    assert envelope['nominal_measured_hull_clearance_lower_bound_m'] == 4
    assert envelope['position_uncertainty_reported'] is False
    assert envelope['unseen_shape_bounded'] is False


def test_target_standoff_does_not_become_an_exclusion_radius_for_other_obstacles(monkeypatch):
    fake_measured_orbit(monkeypatch)
    c = Client()
    normal_flight = c.flight
    def with_unrelated_obstacle(name, args, **kwargs):
        # Another object is 7m away: clear of the API's 2m collision floor,
        # but closer than the requested 10m distance from the selected target.
        if max(2, args.get('minimum_clearance_m', 0)) > 7:
            return {'status': 'ABORTED_BY_SAFETY', 'message': 'unrelated obstacle'}
        return normal_flight(name, args, **kwargs)
    result = TargetMotion(c, with_unrelated_obstacle).execute(
        'orbit_target', {'target_id': 'a', 'standoff_m': 10})
    assert result['status'] == 'SUCCEEDED'
    orbit = next(args for name, args, _ in c.calls if name == 'orbit')
    assert orbit['radius_m'] == 13  # target radius2 + standoff10 + tracking reserve1
    assert result['evidence']['orbit_envelope']['requested_minimum_surface_clearance_m'] == 10


def test_envelope_uses_reported_position_uncertainty_and_observed_hull():
    from python.dcm.target_motion import orbit_envelope
    geometry = {'center': {'north_m': 0, 'east_m': 0}, 'radius_m': 2,
                'observed_hull': [{'north_m': 3, 'east_m': 4}]}
    _, radius, evidence = orbit_envelope(geometry, 3, 0.4)
    assert radius == pytest.approx(9.4)
    assert evidence['measured_enclosure_radius_m'] == 5
    assert evidence['position_uncertainty_reserve_m'] == 0.4
    assert evidence['position_uncertainty_reported'] is True


@pytest.mark.parametrize('uncertainty', [None, 0, float('nan'), float('inf'), -1, True])
def test_missing_or_invalid_uncertainty_is_not_claimed_measured(uncertainty):
    from python.dcm.target_motion import orbit_envelope
    _, radius, evidence = orbit_envelope({'center': {'north_m': 0, 'east_m': 0}, 'radius_m': 2}, 3, uncertainty)
    assert radius == 6
    assert evidence['position_uncertainty_reported'] is False
    assert evidence['position_uncertainty_reserve_m'] == 0


def test_approach_delegates_final_goal_instead_of_blocked_short_ray_endpoint():
    c = Client()
    def routed(name, args, **kw):
        # An unrelated obstacle occupies the old 2m intermediate endpoint;
        # the native route can go around it to the valid final endpoint.
        if name == 'goto' and 1.5 <= args['north_m'] <= 2.5:
            return {'status': 'FAILED', 'message': 'destination intersects obstacle'}
        return c.flight(name, args, **kw)
    r = TargetMotion(c, routed).execute('approach_target', {'target_id': 'a', 'standoff_m': 3})
    assert r['status'] == 'SUCCEEDED'
    assert [(n, a['north_m']) for n, a, kw in c.calls] == [('goto', 7)]
    assert c.calls[0][2]['expected_origin_id'] == 'test'
    assert r['evidence']['navigation_method'] == 'native_goto_to_measured_horizontal_standoff'
    assert not r['evidence']['continuous_target_identity_tracking']


def test_approach_remeasures_after_arrival_and_corrects_geometry():
    c = Client()
    original = c.refresh_target
    count = 0
    def measured(target_id):
        nonlocal count
        count += 1
        result = original(target_id)
        surface = 10 if count == 1 else 10.7
        result.update(surface_range_m=abs(surface-c.north),
                      surface_point={'north_m': surface, 'east_m': 0})
        return result
    c.refresh_target = measured
    r = TargetMotion(c, c.flight).execute('approach_target', {'target_id': 'a', 'standoff_m': 3})
    assert r['status'] == 'SUCCEEDED'
    assert [a['north_m'] for n, a, kw in c.calls] == pytest.approx([7, 7.7])
    assert count == 3


@pytest.mark.parametrize('reason', ['target_not_observed', 'lidar_perception_stale', 'target_association_ambiguous'])
def test_approach_lost_measurement_after_arrival_cannot_continue(reason):
    c = Client()
    original = c.refresh_target
    def measured(target_id):
        return original(target_id) if not c.calls else {'found': False, 'reason': reason}
    c.refresh_target = measured
    r = TargetMotion(c, c.flight).execute('approach_target', {'target_id': 'a', 'standoff_m': 3})
    assert r['status'] == 'FAILED'
    assert [n for n, a, kw in c.calls] == ['goto', 'hold']
    assert r['evidence']['target_measurement']['reason'] == reason


def test_approach_changed_origin_prevents_dispatch():
    c = Client()
    original = c.refresh_target
    c.refresh_target = lambda ident: {**original(ident), 'origin_id': 'different'}
    r = TargetMotion(c, c.flight).execute('approach_target', {'target_id': 'a', 'standoff_m': 3})
    assert r['status'] == 'FAILED'
    assert [n for n, a, kw in c.calls] == ['hold']


def test_approach_interrupted_at_arrival_does_not_override_operator():
    c = Client()
    r = TargetMotion(c, c.flight, lambda: bool(c.calls)).execute(
        'approach_target', {'target_id': 'a', 'standoff_m': 3})
    assert r['status'] == 'CANCELLED'
    assert [n for n, a, kw in c.calls] == ['goto']


def test_approach_no_progress_stops_boundedly():
    c = Client()
    def stuck(name, args, **kw):
        c.calls.append((name, args, kw))
        return {'status': 'SUCCEEDED'}
    r = TargetMotion(c, stuck).execute('approach_target', {'target_id': 'a', 'standoff_m': 3})
    assert r['status'] == 'FAILED'
    assert 'No measurable progress' in r['message']
    assert [n for n, a, kw in c.calls] == ['goto']*4 + ['hold']


def test_overflight_reuses_measured_side_views_before_climbing_over_target():
    c=AcquisitionClient()
    for view in c.views:view['observation_standoff_m']=6
    result=TargetMotion(c,c.flight).execute('overfly_target',{'target_id':'a','clearance_above_m':3})
    assert result['status']=='SUCCEEDED'
    assert [call[0] for call in c.calls]==['goto','goto','goto']
    assert c.calls[0][1]['altitude_agl_m']==4  # measurement view, not overflight
    assert c.calls[1][1]['altitude_agl_m']==9
    assert c.calls[1][1]['north_m']==c.calls[0][1]['north_m']
    assert c.calls[2][1]['north_m']==12
    assert result['evidence']['geometry_acquisition']
    assert c.captures==[('the observed object',False)]


def test_overflight_does_not_invent_a_top_when_height_stays_clipped():
    class Clipped(AcquisitionClient):
        def refresh_target(self,target_id):
            result=super().refresh_target(target_id)
            result['footprint']['height_clipped']=True
            return result
    c=Clipped()
    for view in c.views:view['observation_standoff_m']=6
    result=TargetMotion(c,c.flight).execute('overfly_target',{'target_id':'a'})
    assert result['status']=='FAILED'
    assert all(args.get('altitude_agl_m',4)==4 for name,args,_ in c.calls)
    assert c.calls[-1][0]=='hold'


def test_overhead_point_is_outside_level_front_camera_but_in_downward_view():
    class WithPose(Client):
        def state(self):
            state=super().state();state['local_position_ned']['down_m']=-self.alt
            state['attitude'].update(roll_rad=0,pitch_rad=0)
            return state
    c=WithPose()
    result=TargetMotion(c,c.flight).execute('overfly_target',{'target_id':'a'})
    views=result['evidence']['camera_view_geometry']
    assert views['forward_d435i']['in_frame'] is False
    assert views['downward_brio']['in_frame'] is True
    assert not views['downward_brio']['image_proof']


def test_side_view_transit_keeps_the_nose_on_the_target_centre(monkeypatch):
    fake_measured_orbit(monkeypatch)
    c = AcquisitionClient()
    TargetMotion(c, c.flight).execute('orbit_target', {'target_id': 'a', 'standoff_m': 3})
    geometry = c.refresh_target('a')['geometry']
    centre = geometry.get('orbit_center') or geometry['center']
    assert c.arc_legs, 'a side-view move should be flown as short arc legs'
    for _, leg, _ in c.arc_legs:
        bearing = math.degrees(math.atan2(centre['east_m'] - leg['east_m'], centre['north_m'] - leg['north_m'])) % 360
        assert abs((leg['heading_deg'] - bearing + 180) % 360 - 180) < 1e-6

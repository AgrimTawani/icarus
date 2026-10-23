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
        # Target-facing transit arcs are kept apart so call sequences still
        # show each meaningful move once.
        arcs = self.__dict__.setdefault('arcs', [])
        transit = name == 'orbit' and args.get('revolutions', 1) < 1
        (arcs if transit else self.calls).append((name, deepcopy(args), kw))
        if transit:
            angle = math.atan2(self.east - args['center_east_m'], self.north - args['center_north_m'])
            angle += (1 if args['clockwise'] else -1) * args['revolutions'] * 2 * math.pi
            self.north = args['center_north_m'] + args['radius_m'] * math.cos(angle)
            self.east = args['center_east_m'] + args['radius_m'] * math.sin(angle)
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


def test_partial_footprint_orbits_with_a_stated_assumption(monkeypatch):
    fake_measured_orbit(monkeypatch)
    c = Client(); c.partial = True
    result = TargetMotion(c, c.flight).execute('orbit_target', {'target_id': 'a', 'standoff_m': 3})
    assert result['status'] == 'SUCCEEDED'
    assert [x[0] for x in c.calls] == ['orbit']
    assert result['evidence']['orbit_envelope']['unseen_shape_bounded'] != False


def test_object_without_lidar_hull_cannot_be_orbited():
    class Unmeasured(Client):
        def refresh_target(self, target_id):
            return {'found': False, 'reason': 'unknown_target_id', 'origin_id': 'test'}
    c = Unmeasured()
    result = TargetMotion(c, c.flight).execute('orbit_target', {'target_id': 'ghost', 'standoff_m': 3})
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


def test_a_single_missed_lidar_scan_does_not_lose_the_target(monkeypatch):
    import python.dcm.target_motion as target_motion
    monkeypatch.setattr(target_motion, 'REFRESH_WAIT_S', 0)
    seen = iter([{'found': False, 'reason': 'target_not_observed'}, {'found': True, 'target_id': 't'}])

    class Client:
        def refresh_target(self, target_id):
            return next(seen)

    assert target_motion.TargetMotion(Client(), None)._target('t')['found'] is True


def test_a_lost_target_still_fails_after_bounded_rescans(monkeypatch):
    import python.dcm.target_motion as target_motion
    monkeypatch.setattr(target_motion, 'REFRESH_WAIT_S', 0)
    calls = []

    class Client:
        def refresh_target(self, target_id):
            calls.append(target_id)
            return {'found': False, 'reason': 'target_not_observed'}

    with pytest.raises(target_motion.TargetMeasurementError):
        target_motion.TargetMotion(Client(), None)._target('t')
    assert len(calls) == target_motion.REFRESH_ATTEMPTS


def test_orbit_camera_aims_level_when_the_target_top_was_not_measured():
    from python.dcm.target_motion import _aim_height
    state = {'relative_altitude_m': 5.0}
    assert _aim_height({'footprint': {'height_m': 6.5}}, state) == 6.5
    assert _aim_height({'footprint': {'height_m': None, 'height_clipped': True}}, state) == 5.0
    assert _aim_height({}, state) == 5.0


def test_approach_stops_if_remeasurement_jumps_implausibly_closer():
    """Clutter can re-associate the target with a different, nearer surface.

    This must be reported as a measurement fault, not treated as progress to
    chase: otherwise approach can creep far past the requested standoff.
    """
    class DriftingClient(Client):
        def __init__(self):
            super().__init__()
            self.readings = iter([10.0, 0.3])  # second reading jumps far below standoff*0.5

        def refresh_target(self, target_id):
            base = super().refresh_target(target_id)
            base['surface_range_m'] = next(self.readings)
            return base

    c = DriftingClient()
    result = TargetMotion(c, c.flight).execute('approach_target', {'target_id': 'a', 'standoff_m': 3})
    assert result['status'] == 'FAILED'
    assert 'implausibly closer' in result['message']
    goto_calls = [call for call in c.calls if call[0] == 'goto']
    assert len(goto_calls) == 1  # stopped before compounding a second unsafe move


class LosingClient(Client):
    """LiDAR loses the target after the first leg; the forward depth sees something near."""
    def refresh_target(self, target_id):
        self.lost = bool(self.calls)
        target = super().refresh_target(target_id)
        target['reason'] = 'target_not_observed' if self.lost else ''
        return target

    def depth_feedback(self):
        return {'fresh': True, 'center': {'p10_optical_z_m': 3.0}}


def test_lost_lidar_fails_even_when_forward_depth_is_close():
    """Depth under the nose is not the target; at roofline height it is the roof below."""
    c = LosingClient()
    result = TargetMotion(c, c.flight).execute('approach_target', {'target_id': 'a', 'standoff_m': 5})
    assert result['status'] == 'FAILED'
    assert 'standoff_measured_by' not in result['evidence']
    assert result['evidence']['target_measurement']['reason'] == 'target_not_observed'
    assert result['evidence']['last_measured_range_m'] == 10
    assert [n for n, a, kw in c.calls] == ['goto', 'hold']


def test_approach_stops_if_remeasurement_jumps_farther():
    """Re-association with a surface behind the target must not be chased."""
    class DriftingClient(Client):
        def __init__(self):
            super().__init__()
            self.readings = iter([10.0, 9.0])  # after moving 7 m closer the range barely changed

        def refresh_target(self, target_id):
            base = super().refresh_target(target_id)
            base['surface_range_m'] = next(self.readings)
            return base

    c = DriftingClient()
    result = TargetMotion(c, c.flight).execute('approach_target', {'target_id': 'a', 'standoff_m': 3})
    assert result['status'] == 'FAILED'
    assert 'different' in result['message']
    assert [n for n, a, kw in c.calls] == ['goto', 'hold']


def test_depth_already_measured_is_not_assumed_again():
    """A 20 m x 10 m hull seen from a corner has its depth measured; only the
    remainder up to the visible width is assumed, not a 22 m square."""
    from python.dcm.target_motion import single_face_enclosure
    hull = [{'north_m': n, 'east_m': e} for n, e in ((-20, -3), (-20, 7), (-40, 7), (-40, -3))]
    geometry = {'center': {'north_m': -30, 'east_m': 2}, 'radius_m': 11.2, 'observed_hull': hull}
    viewer = (-30, 30)  # due east, looking west along the 10 m side
    enclosure = single_face_enclosure(geometry, viewer)
    # across = north span 20 m (the width seen); along = east span 10 m already observed.
    # unseen depth assumed up to the width: far edge at 20 m behind the near face.
    assert enclosure['center']['east_m'] == pytest.approx(7 - 10)
    assert enclosure['center']['north_m'] == pytest.approx(-30)
    assert enclosure['radius_m'] == pytest.approx(math.hypot(10, 10))


def test_single_face_orbit_encloses_an_assumed_square_footprint(monkeypatch):
    """One measured face: the circle must also clear a building as deep as it is wide."""
    fake_measured_orbit(monkeypatch)
    c = Client(); c.partial = True
    original = c.refresh_target
    def one_face(target_id):
        target = original(target_id)
        target['geometry'].update(center={'north_m': 10, 'east_m': 0}, radius_m=4,
                                  observed_hull=[{'north_m': 10, 'east_m': -4}, {'north_m': 10, 'east_m': 4}])
        return target
    c.refresh_target = one_face
    result = TargetMotion(c, c.flight).execute('orbit_target', {'target_id': 'a', 'standoff_m': 3})
    assert result['status'] == 'SUCCEEDED'
    assert [name for name, _, _ in c.calls] == ['orbit']  # no measurement arcs, no side-view goto
    orbit = c.calls[0][1]
    # centre pushed back by half the 8 m face, radius encloses the 8x8 square plus standoff plus reserve
    assert orbit['center_north_m'] == pytest.approx(14) and orbit['center_east_m'] == pytest.approx(0)
    assert orbit['radius_m'] == pytest.approx(math.hypot(4, 4) + 3 + 1)
    assert result['evidence']['orbit_envelope']['unseen_shape_bounded'] == 'assumed square footprint from one measured face'

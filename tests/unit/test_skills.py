"""Drone skills: bounded loops over ordinary executor actions."""
import math
import pytest

from python.dcm import skills


class Client:
    def __init__(self, localizations=(), obstacles=()):
        self.heading = 0.0
        self.localizations = list(localizations)
        self.obstacles = list(obstacles)
        self.registered = set()

    def state(self):
        return {'attitude': {'yaw_rad': math.radians(self.heading)}}

    def localize_target(self, description):
        result = self.localizations.pop(0)
        if result.get('target_id'):
            self.registered.add(result['target_id'])
        return result

    def target_feedback(self):
        return [{'target_id': target} for target in self.registered]

    def perception_with_obstacles(self):
        return {'nearby_obstacles': self.obstacles}


class Executor:
    """Records calls; turns change the heading, inspections follow a script."""
    def __init__(self, client, answers=(), failing_turns=0):
        self.client, self.answers, self.failing_turns = client, list(answers), failing_turns
        self.calls = []

    def interrupted(self):
        return False

    def execute(self, name, arguments):
        self.calls.append((name, dict(arguments)))
        if name == 'inspect_scene':
            found = self.answers.pop(0)
            return {'status': 'SUCCEEDED', 'evidence': {'found': found, 'answer': 'scene'}}
        if name == 'yaw':
            if self.failing_turns:
                self.failing_turns -= 1
                return {'status': 'ABORTED_BY_SAFETY', 'message': 'perception stale'}
            sign = 1 if arguments['direction'] == 'clockwise' else -1
            self.client.heading = (self.client.heading + sign * arguments['angle_deg']) % 360
        if name == 'face_heading':
            self.client.heading = arguments['heading_deg']
        return {'status': 'SUCCEEDED'}


def names(executor):
    return [name for name, _ in executor.calls]


def test_search_inspects_each_view_and_stops_when_found():
    executor = Executor(Client(), answers=[False, False, True])
    result = skills.search_for_object(executor, 'building', step_deg=30, rate_deg_s=30)
    assert result['status'] == 'SUCCEEDED' and result['evidence']['found'] is True
    assert names(executor) == ['inspect_scene', 'yaw', 'inspect_scene', 'yaw', 'inspect_scene']
    turn = executor.calls[1][1]
    assert turn == {'angle_deg': 30, 'direction': 'clockwise', 'rate_deg_s': 30}
    assert executor.client.heading == 60


def test_search_honours_direction_and_bounded_sweep():
    executor = Executor(Client(), answers=[False] * 4)
    result = skills.search_for_object(executor, 'tree', clockwise=False, step_deg=90, max_sweep_deg=270)
    assert result['evidence']['found'] is False
    assert names(executor).count('inspect_scene') == 4 and names(executor).count('yaw') == 3
    assert all(args['direction'] == 'counterclockwise' for name, args in executor.calls if name == 'yaw')


def test_search_retries_a_turn_once_after_a_safety_abort():
    executor = Executor(Client(), answers=[False, True], failing_turns=1)
    result = skills.search_for_object(executor, 'building')
    assert result['evidence']['found'] is True
    assert names(executor) == ['inspect_scene', 'yaw', 'yaw', 'inspect_scene']


def test_search_stops_when_a_turn_keeps_failing():
    executor = Executor(Client(), answers=[False], failing_turns=2)
    result = skills.search_for_object(executor, 'building')
    assert result['status'] == 'ABORTED_BY_SAFETY'
    assert names(executor) == ['inspect_scene', 'yaw', 'yaw']


def test_acquire_centres_an_off_centre_object_before_measuring_again():
    client = Client([{'found': False, 'reason': 'no_lidar_cluster_in_selected_region',
                      'bearing_deg': {'image_absolute': 23.0}},
                     {'found': True, 'target_id': 't1', 'bearing_deg': {'image_absolute': 23.0}}])
    executor = Executor(client)
    localization, attempts = skills.acquire(executor, 'building')
    assert localization['target_id'] == 't1'
    assert executor.calls == [('face_heading', {'heading_deg': 23.0})]
    assert len(attempts) == 2


def test_acquire_closes_in_when_centred_but_unranged():
    client = Client([{'found': False, 'reason': 'no_lidar_cluster_in_selected_region',
                      'bearing_deg': {'image_absolute': 2.0}},
                     {'found': True, 'target_id': 't1', 'bearing_deg': {'image_absolute': 2.0}}],
                    obstacles=[{'relative_bearing_deg': 1.0, 'distance_m': 30.0, 'estimated_radius_m': 4.0}])
    executor = Executor(client)
    localization, _ = skills.acquire(executor, 'building')
    assert localization['target_id'] == 't1'
    assert executor.calls == [('move_relative', {'forward_m': skills.STEP_M,
                                                 'maximum_ground_speed_mps': skills.STEP_SPEED_MPS})]


def test_acquire_never_steps_close_to_something_ahead():
    client = Client([{'found': False, 'reason': 'no_lidar_cluster_in_selected_region',
                      'bearing_deg': {'image_absolute': 0.0}}],
                    obstacles=[{'relative_bearing_deg': 0.0, 'distance_m': 9.0, 'estimated_radius_m': 1.5}])
    executor = Executor(client)
    localization, _ = skills.acquire(executor, 'building')
    assert localization['found'] is False and executor.calls == []


def test_acquire_does_not_move_when_the_camera_cannot_see_the_object():
    executor = Executor(Client([{'found': False, 'reason': 'vlm_reported_no_match'}]))
    localization, attempts = skills.acquire(executor, 'building')
    assert localization['found'] is False and executor.calls == [] and len(attempts) == 1


def test_acquire_is_bounded():
    unranged = {'found': False, 'reason': 'no_lidar_cluster_in_selected_region', 'bearing_deg': {'image_absolute': 0.0}}
    executor = Executor(Client([dict(unranged) for _ in range(skills.ACQUIRE_ATTEMPTS)]))
    _, attempts = skills.acquire(executor, 'building')
    assert len(attempts) == skills.ACQUIRE_ATTEMPTS
    assert names(executor).count('move_relative') == skills.ACQUIRE_ATTEMPTS


def test_survey_counts_while_flying_and_merges_repeat_sightings(monkeypatch):
    import threading
    import python.dcm.camera_geometry as geometry
    calls = []
    houses = iter([(5.0, 5.0), (5.5, 5.2), (20.0, 5.0), (300.0, 300.0)])  # 2nd repeats the 1st; last is outside

    class SurveyClient:
        def state(self):
            return {'attitude': {'yaw_rad': 0.0}, 'relative_altitude_m': 20.0,
                    'local_position_ned': {'north_m': 0, 'east_m': 0, 'down_m': -20, 'origin_id': 'o'}}

        def capture_view(self, camera):
            return {'capture_pose': self.state()}

        def locate_all(self, query, view):
            try:
                calls.append(next(houses))
            except StopIteration:
                flying.wait(5)
                return {'boxes': [], 'source_width': 640, 'source_height': 360}
            return {'boxes': [[0, 0, 2, 2]], 'source_width': 640, 'source_height': 360}

    flying = threading.Event()

    class SurveyExecutor:
        client = SurveyClient()
        def interrupted(self):
            return False
        def execute(self, name, args):
            assert name == 'fly_pattern' and args['coverage'] == 'area' and args['lane_spacing_m'] > 0
            while len(calls) < 4:  # the worker looks while the route is still flying
                threading.Event().wait(0.01)
            flying.set()
            return {'status': 'SUCCEEDED', 'evidence': {'anchor': {'north_m': 0, 'east_m': 0, 'heading_deg': 0}}}

    monkeypatch.setattr(geometry, 'camera_calibration', lambda camera: {'available': True, 'vertical_fov_rad': 1.0})
    monkeypatch.setattr(geometry, 'ground_point', lambda pose, camera, u, v, w, h: dict(zip(('north_m', 'east_m'), calls[-1])))
    result = skills.survey_count(SurveyExecutor(), 'house', shape='rectangle', width_m=40, length_m=40)
    evidence = result['evidence']
    assert result['status'] == 'SUCCEEDED'
    assert evidence['count'] == 2 and evidence['sightings'] == 4 and evidence['sightings_inside_area'] == 3
    assert [o['nearest_neighbour_m'] for o in evidence['objects']] == [pytest.approx(14.8, abs=0.2)] * 2


def test_circle_survey_area_is_centred_one_radius_ahead():
    anchor = {'north_m': 0, 'east_m': 0, 'heading_deg': 90}  # facing east
    assert skills._in_area({'north_m': 0, 'east_m': 10}, anchor, 'circle', None, None, 10)
    assert not skills._in_area({'north_m': 0, 'east_m': -1}, anchor, 'circle', None, None, 10)


def test_acquisition_grounds_the_nearest_instance_by_default():
    assert skills.nearest('house') == 'nearest house'
    assert skills.nearest('closest red car') == 'closest red car'
    assert skills.nearest('Nearest tree') == 'Nearest tree'

"""Drone skills: bounded loops over ordinary executor actions."""
import math

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

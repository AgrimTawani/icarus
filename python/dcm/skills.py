"""Drone skills: short, bounded behaviours built from ordinary tools.

Each skill runs existing executor actions in a fixed loop so the mission
model makes one decision instead of many. Every motion still goes through
ToolExecutor and the Drone API's safety checks.

- search_for_object: turn in steps, asking the VLM after each turn, until it
  says the object is visible or the requested sweep is used up.
- acquire: before flying to a visible object, centre it in the camera,
  measure it, and step closer if depth/LiDAR cannot range it yet.
"""
import math

CENTRE_TOLERANCE_DEG = 8.0
ACQUIRE_ATTEMPTS = 5
STEP_M = 4.0
STEP_SPEED_MPS = 1.5
# Never step to within this distance of a LiDAR return in the step direction.
STEP_CLEARANCE_M = 7.0


def _heading_deg(client):
    from python.dcm.agent_tools import message_dict
    return math.degrees(message_dict(client.state())['attitude']['yaw_rad']) % 360


def search_for_object(executor, visual_query, clockwise=True, step_deg=30.0, rate_deg_s=30.0,
                      max_sweep_deg=360.0, camera='forward_d435i'):
    """Inspect the current view, then turn step_deg and inspect again, until found."""
    question = f'Is there a {visual_query} visible in this image?'
    direction = 'clockwise' if clockwise else 'counterclockwise'
    turn = {'angle_deg': step_deg, 'direction': direction, 'rate_deg_s': rate_deg_s}
    views, swept = [], 0.0
    while True:
        if executor.interrupted():
            return {'status': 'CANCELLED', 'message': 'Operator interrupted the search.', 'evidence': {'views': views}}
        look = executor.execute('inspect_scene', {'question': question, 'question_type': 'condition',
                                                  'camera': camera, 'view': 'current'})
        evidence = look.get('evidence', {})
        views.append({'heading_deg': round(_heading_deg(executor.client)), 'status': look.get('status'),
                      'found': evidence.get('found'), 'answer': evidence.get('answer')})
        if look.get('status') == 'SUCCEEDED' and evidence.get('found') is True:
            return {'status': 'SUCCEEDED',
                    'message': f'{visual_query} visible at heading {views[-1]["heading_deg"]} deg after turning {swept:.0f} deg.',
                    'evidence': {'found': True, 'answer': evidence.get('answer'), 'views': views}}
        if swept + step_deg > max_sweep_deg + 1e-6:
            return {'status': 'SUCCEEDED',
                    'message': f'{visual_query} not seen in {len(views)} views over {swept:.0f} deg.',
                    'evidence': {'found': False, 'views': views}}
        moved = executor.execute('yaw', dict(turn))
        if moved.get('status') != 'SUCCEEDED':
            moved = executor.execute('yaw', dict(turn))  # one retry, e.g. after a transient safety brake
        if moved.get('status') != 'SUCCEEDED':
            return {'status': moved.get('status', 'FAILED'),
                    'message': 'Search stopped: the turn failed. ' + moved.get('message', ''),
                    'evidence': {'found': None, 'views': views}}
        swept += step_deg


def _image_bearing(localization):
    bearing = localization.get('bearing_deg')
    if not isinstance(bearing, dict):
        return None
    value = bearing.get('image_absolute', bearing.get('absolute'))
    return value if isinstance(value, (int, float)) and math.isfinite(value) else None


def _clear_ahead_m(perception, half_width_deg=20.0):
    """Nearest LiDAR surface within the forward cone, if any."""
    nearest = math.inf
    for obstacle in perception.get('nearby_obstacles', []):
        bearing, distance = obstacle.get('relative_bearing_deg'), obstacle.get('distance_m')
        if not isinstance(bearing, (int, float)) or not isinstance(distance, (int, float)):
            continue
        if abs((bearing + 180) % 360 - 180) <= half_width_deg:
            nearest = min(nearest, distance - obstacle.get('estimated_radius_m', 0.0))
    return nearest


def registered(client, localization):
    target_id = localization.get('target_id')
    return (localization.get('found') is True and isinstance(target_id, str) and bool(target_id)
            and any(item.get('target_id') == target_id for item in client.target_feedback()))


def acquire(executor, description):
    """Return (last localization, attempt log). Moves only to centre or close in on the object."""
    client = executor.client
    attempts = []
    localization = {'found': False, 'reason': 'not_attempted'}
    for _ in range(ACQUIRE_ATTEMPTS):
        if executor.interrupted():
            break
        localization = client.localize_target(description)
        attempt = {key: localization.get(key) for key in ('found', 'reason', 'target_id', 'surface_range_m')}
        attempts.append(attempt)
        if registered(client, localization):
            break
        bearing = _image_bearing(localization)
        if bearing is None:
            break  # the camera does not see it here: nothing to centre on or approach
        offset = (bearing - _heading_deg(client) + 180) % 360 - 180
        if abs(offset) > CENTRE_TOLERANCE_DEG:
            action = ('face_heading', {'heading_deg': round(bearing % 360, 1)})
        else:
            perception = client.perception_with_obstacles() if hasattr(client, 'perception_with_obstacles') else {}
            step = min(STEP_M, _clear_ahead_m(perception) - STEP_CLEARANCE_M)
            if step < 1.0:
                break  # already close to something; closing further is not safe
            action = ('move_relative', {'forward_m': round(step, 1), 'maximum_ground_speed_mps': STEP_SPEED_MPS})
        result = executor.execute(*action)
        attempt['then'] = {'action': action[0], 'arguments': action[1], 'status': result.get('status')}
        if result.get('status') != 'SUCCEEDED':
            break
    return localization, attempts

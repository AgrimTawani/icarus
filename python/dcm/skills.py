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
import re

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


def nearest(description):
    """Ground the nearest instance by default: with several in view the VLM otherwise
    picks the most prominent one, often too far for LiDAR to range."""
    return description if re.search(r'\b(nearest|closest)\b', description, re.I) else f'nearest {description}'


def acquire(executor, description):
    """Return (last localization, attempt log). Moves only to centre or close in on the object."""
    description = nearest(description)
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


MERGE_RADIUS_M = 8.0
SURVEY_LANE_OVERLAP = 0.8  # lane spacing as a fraction of the camera's ground swath


def _in_area(point, anchor, shape, width_m, length_m, radius_m):
    """Is a ground point inside the surveyed rectangle/circle (fly_pattern's frame)?"""
    heading = math.radians(anchor['heading_deg'])
    dn, de = point['north_m'] - anchor['north_m'], point['east_m'] - anchor['east_m']
    forward = dn*math.cos(heading) + de*math.sin(heading)
    right = -dn*math.sin(heading) + de*math.cos(heading)
    if shape == 'circle':
        return math.hypot(forward - radius_m, right) <= radius_m
    length = length_m if length_m is not None else width_m
    return 0 <= forward <= length and 0 <= right <= width_m


def merge_sightings(points, radius_m=MERGE_RADIUS_M):
    """Group ground sightings of the same object seen from overlapping frames."""
    groups = []
    for point in points:
        for group in groups:
            if math.hypot(point['north_m'] - group['north_m'], point['east_m'] - group['east_m']) <= radius_m:
                group['sightings'] += 1
                k = group['sightings']
                group['north_m'] += (point['north_m'] - group['north_m']) / k
                group['east_m'] += (point['east_m'] - group['east_m']) / k
                break
        else:
            groups.append({'north_m': point['north_m'], 'east_m': point['east_m'], 'sightings': 1})
    for group in groups:
        others = [math.hypot(group['north_m'] - o['north_m'], group['east_m'] - o['east_m'])
                  for o in groups if o is not group]
        group['nearest_neighbour_m'] = round(min(others), 1) if others else None
        group['north_m'], group['east_m'] = round(group['north_m'], 1), round(group['east_m'], 1)
    return groups


def survey_count(executor, visual_query, shape='rectangle', width_m=None, length_m=None,
                 radius_m=None, altitude_agl_m=None, maximum_ground_speed_mps=2.0,
                 camera='downward_brio'):
    """Fly an area once and count a visual object class while flying.

    The flight is one continuous route. A parallel worker keeps taking frames,
    asks the VLM for every instance, and projects each onto the ground, so
    nothing stops to look. Sightings from overlapping frames are merged by
    position; distances between counted objects come from the same positions.
    """
    import threading
    from python.dcm.agent_tools import message_dict
    from python.dcm.camera_geometry import camera_calibration, ground_point
    client = executor.client
    state = message_dict(client.state())
    altitude = altitude_agl_m or state['relative_altitude_m']
    calibration = camera_calibration(camera)
    if not calibration.get('available'):
        return {'status': 'FAILED', 'message': f'{camera} calibration unavailable: {calibration.get("reason")}'}
    # The camera holds the start heading, so image height spans the lane step.
    swath = 2 * altitude * math.tan(calibration['vertical_fov_rad'] / 2)
    pattern = {'shape': shape, 'coverage': 'area', 'altitude_agl_m': altitude,
               'lane_spacing_m': round(max(2.0, SURVEY_LANE_OVERLAP * swath), 1),
               'maximum_ground_speed_mps': maximum_ground_speed_mps,
               'camera_heading_deg': round(math.degrees(state['attitude']['yaw_rad']) % 360, 1),
               **({'radius_m': radius_m} if shape == 'circle' else
                  {'width_m': width_m, **({'length_m': length_m} if length_m is not None else {})})}
    sightings, frames, errors, done = [], [], [], threading.Event()

    def look():
        while not done.is_set() and not executor.interrupted():
            try:
                view = client.capture_view(camera=camera)
                pose = view['capture_pose']
                found = client.locate_all(visual_query, view=view)
                frames.append({'boxes': len(found['boxes']), 'altitude_m': pose.get('relative_altitude_m')})
                for x1, y1, x2, y2 in found['boxes']:
                    point = ground_point(pose, camera, (x1+x2)/2, (y1+y2)/2,
                                         found['source_width'], found['source_height'])
                    if point is not None:
                        sightings.append(point)
            except Exception as error:  # one bad frame must not stop the survey
                errors.append(str(error)[:200])
                done.wait(1.0)

    worker = threading.Thread(target=look, daemon=True)
    worker.start()
    try:
        flight = executor.execute('fly_pattern', pattern)
    finally:
        done.set()
        worker.join(timeout=120)
    anchor = flight.get('evidence', {}).get('anchor')
    inside = [p for p in sightings if anchor and _in_area(p, anchor, shape, width_m, length_m, radius_m)]
    objects = merge_sightings(inside)
    evidence = {'visual_query': visual_query, 'count': len(objects), 'objects': objects,
                'frames_inspected': len(frames), 'sightings': len(sightings),
                'sightings_inside_area': len(inside), 'lane_spacing_m': pattern['lane_spacing_m'],
                'altitude_agl_m': altitude, 'camera': camera, 'path_completed': flight['status'] == 'SUCCEEDED',
                'method': 'VLM boxes projected to flat ground, merged within %.0f m; positions approximate' % MERGE_RADIUS_M,
                **({'frame_errors': errors[:3]} if errors else {})}
    status = flight['status'] if flight['status'] != 'SUCCEEDED' or frames else 'OBSERVATION_UNAVAILABLE'
    return {'status': status, 'message': flight.get('message', ''), 'evidence': evidence}

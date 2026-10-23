"""Measured reference resolution and fixed-camera geometry; never image proof.

State uses NED and autopilot FRD Euler attitude. SDF cameras use FLU with
optical forward +X, image right -Y, image down -Z. Unknown inputs stay unknown.
"""
import json
import math
from functools import lru_cache
from pathlib import Path
import xml.etree.ElementTree as ET

_MODEL = Path(__file__).resolve().parents[2] / 'simulation/models/akshu_compact_sitl/model.sdf'
_NAMES = {'forward_d435i': 'd435i', 'downward_brio': 'brio_down'}
_AIRSIM_CAMERAS = {'forward_d435i': '0', 'downward_brio': '3'}
_airsim_settings = None  # set once per process when the active simulator is AirSim


def use_airsim_settings(path):
    """Take camera calibration from the AirSim settings the simulator runs with."""
    global _airsim_settings
    _airsim_settings = None if path is None else str(path)
    _calibration.cache_clear()


def airsim_camera(settings_path, camera='forward_d435i'):
    """Scene camera from AirSim settings: AirSim pose is FRD metres/degrees."""
    settings = json.loads(Path(settings_path).read_text())
    vehicle = next(iter(settings['Vehicles'].values()))
    mount = vehicle.get('Cameras', {}).get(_AIRSIM_CAMERAS.get(camera, ''))
    if mount is None:
        raise ValueError(f'AirSim settings define no {camera} camera')
    scene = next(item for item in mount['CaptureSettings'] if item.get('ImageType', 0) == 0)
    # FRD -> FLU flips y, z, pitch and yaw; AirSim Pitch<0 looks down.
    return {'width': int(scene['Width']), 'height': int(scene['Height']),
            'horizontal_fov_deg': float(scene.get('FOV_Degrees', 90)),
            'pose_flu': (float(mount.get('X', 0)), -float(mount.get('Y', 0)), -float(mount.get('Z', 0)),
                         math.radians(float(mount.get('Roll', 0))), -math.radians(float(mount.get('Pitch', 0))),
                         -math.radians(float(mount.get('Yaw', 0))))}


def _number(value):
    if isinstance(value, bool) or not isinstance(value, (int, float)) or not math.isfinite(value):
        raise ValueError('Missing or non-finite measured coordinate')
    return float(value)


def _rotation(roll, pitch, yaw):
    cr, sr, cp, sp, cy, sy = math.cos(roll), math.sin(roll), math.cos(pitch), math.sin(pitch), math.cos(yaw), math.sin(yaw)
    return ((cy*cp, cy*sp*sr-sy*cr, cy*sp*cr+sy*sr),
            (sy*cp, sy*sp*sr+cy*cr, sy*sp*cr-cy*sr), (-sp, cp*sr, cp*cr))


def _inverse_rotate(matrix, vector):
    return tuple(sum(matrix[j][i]*vector[j] for j in range(3)) for i in range(3))


def _local(state):
    local = state.get('local_position_ned', {})
    if not local.get('origin_id'):
        raise ValueError('Measured local origin is unavailable')
    return {**{k: _number(local.get(k)) for k in ('north_m', 'east_m', 'down_m')}, 'origin_id': local['origin_id']}


def resolve_reference(state, reference='home', launch_state=None):
    """Resolve home or a saved launch pose into the current local NED frame.

    Global-to-local conversion uses a co-measured position pair, WGS84 local
    curvature, and AMSL heights. Reject distant transforms beyond the local
    tangent approximation. Caller must supply fresh, coherent state snapshots.
    """
    try:
        local = _local(state)
        if reference == 'launch':
            if launch_state is None:
                raise ValueError('No launch snapshot was recorded')
            saved = _local(launch_state)
            if saved['origin_id'] == local['origin_id']:
                return {'available': True, 'reference': reference, **saved}
            target = launch_state.get('position', {}).get('geo', {})
        elif reference == 'home':
            measured = state.get('home_local_position_ned')
            if measured:
                saved = _local({'local_position_ned': measured})
                stamp = int(state.get('home_position_received_at_unix_ms', 0))
                if stamp <= 0 or stamp > int(state.get('observed_at_unix_ms', 0)):
                    raise ValueError('Autopilot home measurement timestamp is unavailable')
                if saved['origin_id'] != local['origin_id']:
                    raise ValueError('Autopilot home origin differs from current origin')
                return {'available': True, 'reference': reference,
                        'source': 'autopilot_HOME_POSITION', **saved}
            home = state.get('home', {})
            if 'local_ned' in home:
                saved = _local({'local_position_ned': home['local_ned']})
                if saved['origin_id'] != local['origin_id']:
                    raise ValueError('Home local origin differs from current origin')
                return {'available': True, 'reference': reference, **saved}
            target = home.get('geo', {})
        else:
            raise ValueError('Unsupported named reference; use home or launch')
        current = state.get('position', {}).get('geo', {})
        for point in (current, target):
            if point.get('frame') not in ('COORDINATE_FRAME_WGS84_AMSL', 1):
                raise ValueError('Reference transform requires measured AMSL global coordinates')
            if abs(_number(point.get('latitude_deg'))) >= 89 or abs(_number(point.get('longitude_deg'))) > 180:
                raise ValueError('Global coordinates outside supported local tangent range')
        lat = math.radians(current['latitude_deg'])
        dlat = math.radians(target['latitude_deg']-current['latitude_deg'])
        dlon = math.radians((target['longitude_deg']-current['longitude_deg']+180)%360-180)
        e2, a = 0.00669437999014, 6378137.0
        q = 1-e2*math.sin(lat)**2
        north = dlat*a*(1-e2)/q**1.5
        east = dlon*a/math.sqrt(q)*math.cos(lat)
        if math.hypot(north, east) > 10000:
            raise ValueError('Reference is beyond supported 10 km local tangent transform')
        return {'available': True, 'reference': reference, 'origin_id': local['origin_id'],
                'north_m': local['north_m']+north, 'east_m': local['east_m']+east,
                'down_m': local['down_m']+_number(current.get('altitude_m'))-_number(target.get('altitude_m'))}
    except (ValueError, TypeError, KeyError) as error:
        return {'available': False, 'reference': reference, 'reason': str(error)}


@lru_cache(maxsize=4)
def _calibration(camera):
    if camera not in _NAMES:
        raise ValueError('Unsupported camera calibration')
    if _airsim_settings is not None:
        mount = airsim_camera(_airsim_settings, camera)
        hfov = math.radians(mount['horizontal_fov_deg'])
        return {'camera': camera, 'pose_flu': mount['pose_flu'], 'width': mount['width'], 'height': mount['height'],
                'horizontal_fov_rad': hfov, 'vertical_fov_rad': 2*math.atan(math.tan(hfov/2)*mount['height']/mount['width']),
                'near_m': 0.1, 'far_m': 1000.0, 'fixed_mount': True, 'tilt_control_available': False,
                'source': _airsim_settings, 'simulation_only': True}
    sensor = ET.parse(_MODEL).find(f".//sensor[@name='{_NAMES[camera]}']")
    pose = tuple(float(v) for v in sensor.findtext('pose').split())
    width, height = int(sensor.findtext('camera/image/width')), int(sensor.findtext('camera/image/height'))
    hfov = float(sensor.findtext('camera/horizontal_fov'))
    return {'camera': camera, 'pose_flu': pose, 'width': width, 'height': height,
            'horizontal_fov_rad': hfov, 'vertical_fov_rad': 2*math.atan(math.tan(hfov/2)*height/width),
            'near_m': float(sensor.findtext('camera/clip/near')), 'far_m': float(sensor.findtext('camera/clip/far')),
            'fixed_mount': True, 'tilt_control_available': False, 'source': str(_MODEL), 'simulation_only': True}


def camera_calibration(camera='forward_d435i'):
    try:
        return {'available': True, **_calibration(camera)}
    except (ValueError, KeyError, StopIteration, OSError, ET.ParseError, AttributeError) as error:
        return {'available': False, 'reason': str(error)}


def _yaw_elevation_limit(rotation, ned, calibration):
    """Prove unreachable elevation using a conservative cone around the frustum.

    Yaw rotates the camera origin and optical axis around world vertical. Their
    heights and axis elevation stay fixed. Bound every possible point elevation
    using the camera-origin circle; only declare impossible for disjoint bounds.
    Overlap is unknown, not proof that some yaw or an unoccluded view exists.
    """
    pose = calibration['pose_flu']
    mount = _rotation(*pose[3:])
    axis_frd = (mount[0][0], -mount[1][0], -mount[2][0])
    origin_frd = (pose[0], -pose[1], -pose[2])
    world = lambda vector: tuple(sum(rotation[i][j]*vector[j] for j in range(3)) for i in range(3))
    axis, offset = world(axis_frd), world(origin_frd)
    axis_elevation = math.atan2(-axis[2], math.hypot(*axis[:2]))
    cone = math.atan(math.hypot(math.tan(calibration['horizontal_fov_rad']/2),
                               math.tan(calibration['vertical_fov_rad']/2)))
    horizontal, radius = math.hypot(*ned[:2]), math.hypot(*offset[:2])
    vertical = ned[2]-offset[2]
    elevations = sorted(math.atan2(-vertical, distance)
                        for distance in (abs(horizontal-radius), horizontal+radius))
    outside = elevations[1] < axis_elevation-cone or elevations[0] > axis_elevation+cone
    return {'yaw_alone_can_resolve': False if outside else None,
            **({'yaw_limit_reason': 'Point elevation is outside this fixed camera reach; change camera or vehicle position.'}
               if outside else {})}


def project_point(state, target, camera='forward_d435i'):
    """Project a point into RGB pixels. In-frame does not prove unoccluded visibility."""
    try:
        cal = _calibration(camera)
        local = _local(state)
        if target.get('origin_id') != local['origin_id']:
            raise ValueError('Target and camera pose must share a measured origin')
        attitude = state.get('attitude', {})
        rotation = _rotation(*(_number(attitude.get(k)) for k in ('roll_rad', 'pitch_rad', 'yaw_rad')))
        ned = tuple(_number(target.get(k))-local[k] for k in ('north_m', 'east_m', 'down_m'))
        frd = _inverse_rotate(rotation, ned)
        flu = (frd[0], -frd[1], -frd[2])
        pose = cal['pose_flu']
        sensor = _inverse_rotate(_rotation(*pose[3:]), tuple(flu[i]-pose[i] for i in range(3)))
        x, y, z = sensor
        result = {'available': True, 'camera': camera, 'image_proof': False,
                  'occlusion_checked': False, 'optical_depth_m': x,
                  **_yaw_elevation_limit(rotation, ned, cal)}
        if x <= 0:
            return {**result, 'in_frame': False, 'reason': result.get('yaw_limit_reason', 'Target is behind the camera')}
        focal = cal['width']/(2*math.tan(cal['horizontal_fov_rad']/2))
        u, v = cal['width']/2-focal*y/x, cal['height']/2-focal*z/x
        inside = 0 <= u < cal['width'] and 0 <= v < cal['height'] and cal['near_m'] <= x <= cal['far_m']
        if inside:
            result['yaw_alone_can_resolve'] = True
        return {**result, 'in_frame': inside, 'pixel_x': u, 'pixel_y': v,
                'reason': 'Geometrically in RGB frustum; image evidence still required' if inside else result.get('yaw_limit_reason', 'Target is outside RGB field of view or clipping range')}
    except (ValueError, TypeError, KeyError, StopIteration, OSError, ET.ParseError, AttributeError) as error:
        return {'available': False, 'in_frame': None, 'image_proof': False, 'reason': str(error)}


def orbit_camera_feasibility(radius_m, altitude_agl_m, target_altitude_agl_m=0, camera='forward_d435i'):
    """Level, center-facing orbit geometry, not a guarantee during banking/occlusion.

    Heights here mean above-home, matching the existing API's altitude_agl_m.
    A target's actual height must be supplied; buildings have no implied height.
    """
    try:
        radius, altitude, target_altitude = map(_number, (radius_m, altitude_agl_m, target_altitude_agl_m))
        if radius <= 0:
            raise ValueError('Orbit radius must be positive')
        state = {'local_position_ned': {'north_m': -radius, 'east_m': 0, 'down_m': -altitude, 'origin_id': 'orbit_geometry'},
                 'attitude': {'roll_rad': 0, 'pitch_rad': 0, 'yaw_rad': 0}}
        target = {'north_m': 0, 'east_m': 0, 'down_m': -target_altitude, 'origin_id': 'orbit_geometry'}
        projection = project_point(state, target, camera)
        return {**projection, 'feasible': projection.get('in_frame'), 'assumption': 'Level aircraft, yaw continuously facing orbit center; target point only',
                'target_depression_deg': math.degrees(math.atan2(altitude-target_altitude, radius))}
    except (ValueError, TypeError) as error:
        return {'available': False, 'feasible': None, 'reason': str(error), 'image_proof': False}


def target_reference_feedback(state, targets, cameras):
    """Express retained map references at the current vehicle pose, read-only.

    Offsets use the heading-aligned horizontal frame of move_relative. A
    projected point is not a new object detection, surface range or clearance.
    """
    from copy import deepcopy
    result = deepcopy(targets)
    if not isinstance(result, list):
        return result
    for target in result:
        if not isinstance(target, dict) or target.get('reference_available') is False:
            continue
        geometry = target.get('geometry', {})
        center = geometry.get('center') if isinstance(geometry, dict) else None
        if not isinstance(center, dict) or not center:
            continue  # An identity-only reference supplies no coordinates to project.
        try:
            local = _local(state)
            origin = center.get('origin_id', target.get('origin_id'))
            if not origin or origin != local['origin_id']:
                raise ValueError('Reference and vehicle pose must share a measured origin')
            north = _number(center.get('north_m')) - local['north_m']
            east = _number(center.get('east_m')) - local['east_m']
            yaw = _number(state.get('attitude', {}).get('yaw_rad'))
            top = geometry.get('top_down_m')
            height_known = type(top) in (int, float) and math.isfinite(top)
            target['relative_reference'] = {
                'available': True, 'frame': 'heading_aligned_horizontal',
                'reference_point': 'measured_footprint_center_at_retained_top_height',
                'forward_m': round(north*math.cos(yaw) + east*math.sin(yaw), 3),
                'right_m': round(-north*math.sin(yaw) + east*math.cos(yaw), 3),
                'up_m': round(local['down_m'] - top, 3) if height_known else None,
                'horizontal_center_distance_m': round(math.hypot(north, east), 3),
                'current_surface_range_measurement': False}
            point = {'north_m': center['north_m'], 'east_m': center['east_m'],
                     'down_m': top, 'origin_id': origin}
            projections = {}
            for camera in cameras:
                projection = (project_point(state, point, camera) if height_known else
                    {'available': False, 'in_frame': None,
                     'reason': 'Measured reference height unavailable; no vertical coordinate invented.'})
                projections[camera] = {key: projection[key] for key in (
                    'available', 'in_frame', 'yaw_alone_can_resolve', 'reason') if key in projection}
                projections[camera]['image_proof'] = False
            target['camera_reference_projection'] = projections
        except (ValueError, TypeError, AttributeError) as error:
            target['relative_reference'] = {'available': False, 'reason': str(error)}
    return result

"""Measured movement receipts; no interpretation of requested missions."""
import math


def _finite(value):
    return type(value) in (int, float) and math.isfinite(value)


def _pose(state):
    local = state.get('local_position_ned', {})
    pose = {k: local[k] for k in ('north_m', 'east_m', 'down_m')
            if _finite(local.get(k))}
    if local.get('origin_id'):
        pose['origin_id'] = local['origin_id']
    altitude = state.get('relative_altitude_m')
    if _finite(altitude):
        pose['altitude_above_home_m'] = altitude
    yaw = state.get('attitude', {}).get('yaw_rad')
    if _finite(yaw):
        pose['heading_deg'] = math.degrees(yaw) % 360
    return pose


def movement_feedback(start, arguments, end=None):
    """Keep commanded target separate from actual displacement, even on failure."""
    before = _pose(start)
    target = {k: arguments[k] for k in ('north_m', 'east_m', 'altitude_agl_m')
              if _finite(arguments.get(k))}
    target['origin_id'] = arguments.get('origin_id')
    target['altitude_reference'] = 'above_home'
    target['heading_mode'] = ('absolute' if 'heading_deg' in arguments else
                              'face_point' if 'face_point' in arguments else 'keep_current')
    if _finite(arguments.get('heading_deg')):
        target['heading_deg'] = arguments['heading_deg'] % 360
    result = {'resolved_target': target, 'start': before, 'end': _pose(end) if end else None,
              'measured_displacement_ned_m': None}
    after = result['end']
    if after is None:
        result['displacement_reason'] = 'No terminal measurement; displacement is unknown.'
    elif not before.get('origin_id') or before.get('origin_id') != after.get('origin_id'):
        result['displacement_reason'] = 'Missing or changed local origin; displacement is unknown.'
    elif any(k not in before or k not in after for k in ('north_m', 'east_m', 'down_m')):
        result['displacement_reason'] = 'Incomplete measured local positions; displacement is unknown.'
    else:
        result['measured_displacement_ned_m'] = {
            k: after[k] - before[k] for k in ('north_m', 'east_m', 'down_m')}
    return result

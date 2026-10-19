"""Typed destination selection over existing controllers; no mission text parsing."""
import copy
import hashlib
import json
import math
import os
from pathlib import Path

KINDS = ['coordinates', 'home', 'launch', 'named_waypoint', 'visible_object', 'registered_target']
OBJECTS = {'visible_object', 'registered_target'}


def destination_schema(operation, legacy):
    # Keep a real tagged union inside a root property. llama.cpp's native
    # tagged-tools parser needs root properties, while nested unions already
    # support exact per-variant fields (as used by execute_sequence).
    schema = copy.deepcopy(legacy)
    props = schema['properties']
    for name in ('north_m', 'east_m', 'center_north_m', 'center_east_m', 'center_reference'):
        props.pop(name, None)
    text = {'type': 'string', 'minLength': 1}
    number = {'type': 'number'}
    def ref(kind, properties=None, required=()):
        return {'type': 'object', 'properties': {'type': {'const': kind}, **(properties or {})},
                'required': ['type', *required], 'additionalProperties': False}
    props['destination'] = {'oneOf': [
        ref('coordinates', {'north_m': number, 'east_m': number,
            'coordinate_frame': {'enum': ['home_relative_ned', 'local_ned']}, 'origin_id': text},
            ['north_m', 'east_m', 'coordinate_frame']),
        ref('home'), ref('launch'),
        ref('named_waypoint', {'waypoint_name': text}, ['waypoint_name']),
        ref('visible_object', {'visual_query': {**text, 'description': 'Visible appearance only.'},
            'view_heading_deg': {'type': 'number', 'minimum': 0, 'exclusiveMaximum': 360,
                'description': 'Absolute camera heading before localization; current heading for an object already in front.'}},
            ['visual_query', 'view_heading_deg']),
        ref('registered_target', {'target_id': text}, ['target_id'])]}
    props['standoff_m'] = {'type': 'number', 'exclusiveMinimum': 0,
                           'description': 'Distance from measured object surface, not center radius.'}
    if operation == 'goto':
        props.update(object_position={'enum': ['standoff', 'above']},
                     clearance_above_m={'type': 'number', 'exclusiveMinimum': 0})
    schema['required'] = ['destination'] + (['camera'] if operation == 'orbit' else [])
    return schema


def overflight_schema(legacy):
    base = copy.deepcopy(legacy.get('oneOf', [legacy])[0])
    for key in ('target_id', 'description', 'visual_query'):
        base['properties'].pop(key, None)
    base['required'] = [key for key in base.get('required', [])
                        if key not in ('target_id', 'description', 'visual_query')]
    base['properties']['altitude_agl_m'] = {'type': 'number'}
    schema = destination_schema('goto', base)
    props = schema['properties']
    props['destination']['oneOf'] = [v for v in props['destination']['oneOf']
        if v['properties']['type']['const'] in OBJECTS]
    props.pop('object_position', None)
    props.pop('standoff_m', None)
    return schema


def unpack_destination(operation, arguments):
    args = copy.deepcopy(arguments)
    reference = args.pop('destination')
    kind = reference.pop('type')
    if operation == 'orbit' and kind == 'coordinates':
        reference['center_north_m'] = reference.pop('north_m')
        reference['center_east_m'] = reference.pop('east_m')
    return {**args, **reference, 'destination_type': kind}


def validate_destination(operation, args):
    kind = args['destination_type']
    coords = {'north_m', 'east_m'} if operation == 'goto' else {'center_north_m', 'center_east_m'}
    required = set()
    allowed = {'destination_type', 'mission_complete', 'mission_progress',
               'maximum_ground_speed_mps', 'altitude_agl_m'}
    if operation == 'orbit':
        allowed |= {'camera', 'clockwise', 'revolutions'}
        if kind in OBJECTS and args.get('face_center') is True:
            allowed.add('face_center')
    if kind not in OBJECTS:
        allowed.add('minimum_clearance_m')
        allowed |= ({'face_center', 'viewpoints', 'viewpoint_dwell_s'} if operation == 'orbit' else {'heading_deg'})
    if kind == 'coordinates':
        required |= coords | {'coordinate_frame', 'altitude_agl_m'}
        allowed |= coords | {'coordinate_frame', 'origin_id'}
        if args.get('coordinate_frame') == 'local_ned':
            required.add('origin_id')
    elif kind == 'named_waypoint':
        required.add('waypoint_name'); allowed.add('waypoint_name')
    elif kind in OBJECTS:
        required.add('visual_query' if kind == 'visible_object' else 'target_id')
        allowed |= {'visual_query' if kind == 'visible_object' else 'target_id', 'view_heading_deg', 'standoff_m'}
        if kind == 'visible_object':
            required.add('view_heading_deg')
        if operation == 'goto':
            required.add('object_position'); allowed.add('object_position')
            if args.get('object_position') == 'above':
                allowed.add('clearance_above_m')
        if operation == 'orbit' or args.get('object_position') == 'standoff':
            required.add('standoff_m')
        if operation == 'goto' and args.get('object_position') == 'above' and 'standoff_m' in args:
            raise ValueError('Above-object positioning uses clearance_above_m, not standoff_m')
    if operation == 'orbit' and kind not in OBJECTS:
        required |= {'radius_m', 'altitude_agl_m'}
        allowed |= {'radius_m', 'target_altitude_agl_m'}
    missing = required - args.keys()
    extra = args.keys() - allowed
    if missing:
        raise ValueError(f'{operation} {kind} requires {sorted(missing)}')
    if extra:
        raise ValueError(f'{operation} {kind} does not use {sorted(extra)}')


def waypoint_catalogue():
    path = os.environ.get('ICARUS_WAYPOINTS_FILE')
    if not path:
        return {'available': True, 'entries': [], 'source': 'No named waypoint catalogue is configured.'}
    try:
        raw = Path(path).read_bytes(); data = json.loads(raw)
        if data.get('coordinate_frame') != 'home_relative_ned' or not isinstance(data.get('entries'), list):
            raise ValueError('Catalogue requires coordinate_frame=home_relative_ned and entries')
        names = set(); entries = []
        for entry in data['entries']:
            name = entry['name'].strip()
            if not name or name.casefold() in names:
                raise ValueError('Waypoint names must be nonempty and unique')
            role = entry.get('role', 'navigation_pose')
            if role not in ('navigation_pose', 'observation_pose', 'orbit_center'):
                raise ValueError('Unknown waypoint role')
            point = {'name': name, 'role': role}
            for key in ('north_m', 'east_m', 'altitude_agl_m', 'heading_deg', 'target_altitude_agl_m'):
                if key not in entry and key in ('altitude_agl_m', 'heading_deg', 'target_altitude_agl_m'):
                    continue
                value = entry[key]
                if type(value) not in (int, float) or not math.isfinite(value):
                    raise ValueError('Waypoint coordinates must be finite numbers')
                point[key] = value
            entries.append(point); names.add(name.casefold())
        return {'available': True, 'entries': entries, 'coordinate_frame': 'home_relative_ned',
                'source': path, 'sha256': hashlib.sha256(raw).hexdigest()}
    except (OSError, ValueError, TypeError, KeyError, AttributeError) as error:
        return {'available': False, 'entries': [], 'source': path, 'reason': str(error)}


def catalogue_context(include_entries=False):
    catalogue = waypoint_catalogue()
    if include_entries:
        # A saved pose is an executable reference, not a coordinate recipe.
        # Keep its full position/heading together in the resolver; copying only
        # numeric position into a coordinates call loses pose semantics.
        entries = [{**{key:value for key,value in point.items()
                       if key not in ('north_m', 'east_m')},
                    'destination': {'type':'named_waypoint', 'waypoint_name':point['name']},
                    'position_resolved_by_tool': True} for point in catalogue['entries']]
        return {**{key:value for key,value in catalogue.items() if key != 'source'},
                'source': 'operator_configured_catalogue', 'entries':entries, 'loaded': True}
    return {**{key:value for key,value in catalogue.items() if key not in ('entries', 'source')},
        'source': 'operator_configured_catalogue' if os.environ.get('ICARUS_WAYPOINTS_FILE') else catalogue['source'],
        'entries': [], 'loaded': False, 'count': len(catalogue['entries']),
        'kind': 'saved_vehicle_navigation_poses',
        'load_with': {'tool':'get_context','arguments':{'include_waypoints':True}}}


def execute_destination(executor, operation, arguments):
    from python.dcm.agent_tools import message_dict
    from python.dcm.camera_geometry import resolve_reference
    args = unpack_destination(operation, arguments); kind = args.pop('destination_type')
    state = message_dict(executor.client.state())
    evidence = {'destination_type': kind}
    if kind in OBJECTS:
        preparation = []
        if 'altitude_agl_m' in args:
            if 'relative_altitude_m' not in state:
                raise ValueError('Measured altitude is required to change height')
            move = {'up_m': args.pop('altitude_agl_m') - state['relative_altitude_m']}
            result = executor.execute('move_relative', move)
            preparation.append({'name': 'move_relative', 'arguments': move, 'result': result})
            if result['status'] != 'SUCCEEDED':
                return result
        if 'view_heading_deg' in args:
            heading = {'heading_deg': args.pop('view_heading_deg')}
            result = executor.execute('face_heading', heading)
            preparation.append({'name': 'face_heading', 'arguments': heading, 'result': result})
            if result['status'] != 'SUCCEEDED':
                return result
        # Object orbit always tracks its measured target. Explicit false is
        # rejected by validation; true is redundant with this controller.
        args.pop('face_center', None)
        position = args.pop('object_position', None)
        method = 'orbit_target' if operation == 'orbit' else ('overfly_target' if position == 'above' else 'approach_target')
        result = executor.execute(method, args)
        result.setdefault('evidence', {})['destination'] = {**evidence, 'controller': method}
        if preparation:
            result['evidence']['preparatory_actions'] = preparation
        return result
    origin = state.get('local_position_ned', {}).get('origin_id')
    if operation == 'orbit' and kind in ('home', 'launch'):
        result = executor._flight('orbit', {'center_reference': kind, **args}, expected_origin_id=origin)
        result.setdefault('evidence', {})['destination'] = evidence
        return result
    if kind == 'named_waypoint':
        name = args.pop('waypoint_name'); catalogue = waypoint_catalogue()
        point = next((p for p in catalogue['entries'] if p['name'].casefold() == name.strip().casefold()), None)
        if point is None:
            question = f"What north/east coordinates relative to home should '{name}' identify?"
            return {'status': 'NEEDS_CLARIFICATION', 'requires_user_choice': True,
                    'question': question, 'message': 'No coordinates are registered for this waypoint; no movement executed.',
                    'evidence': {**evidence, 'waypoint_name': name, 'catalogue': catalogue}}
        if operation == 'orbit' and point['role'] != 'orbit_center':
            raise ValueError('This waypoint is a vehicle navigation/observation pose, not an orbit center. Localize the physical target instead.')
        evidence.update(reference_role=point['role'],
            established_relationship='arrived_at_navigation_pose' if operation == 'goto' else 'orbiting_declared_coordinate_center',
            object_relative_position_established=False)
        if operation == 'orbit' and 'target_altitude_agl_m' in point:
            args.setdefault('target_altitude_agl_m', point['target_altitude_agl_m'])
        args.setdefault('altitude_agl_m', point.get('altitude_agl_m', state.get('relative_altitude_m')))
        if operation == 'goto':
            args.update(north_m=point['north_m'], east_m=point['east_m'])
            if 'heading_deg' in point:
                args.setdefault('heading_deg', point['heading_deg'])
        else:
            args.update(center_north_m=point['north_m'], center_east_m=point['east_m'])
        args['coordinate_frame'] = 'home_relative_ned'
        evidence.update(waypoint_name=point['name'], catalogue_sha256=catalogue.get('sha256'))
    if kind in ('home', 'launch') or args.get('coordinate_frame') == 'home_relative_ned':
        ref = resolve_reference(state, kind if kind in ('home', 'launch') else 'home',
                                getattr(executor.client, '_takeoff_reference', None))
        if not ref.get('available'):
            raise ValueError(ref.get('reason', 'Measured reference unavailable'))
        origin = ref['origin_id']
        north, east = ('north_m', 'east_m') if operation == 'goto' else ('center_north_m', 'center_east_m')
        args[north] = ref['north_m'] + args.get(north, 0)
        args[east] = ref['east_m'] + args.get(east, 0)
        if kind in ('home', 'launch'):
            args.setdefault('altitude_agl_m', state.get('relative_altitude_m'))
    elif args.get('coordinate_frame') == 'local_ned':
        origin = args.pop('origin_id')
    args.pop('coordinate_frame', None); args.pop('origin_id', None)
    result = executor._flight(operation, args, expected_origin_id=origin)
    result.setdefault('evidence', {})['destination'] = evidence
    return result

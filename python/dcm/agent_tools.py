"""Native tool definitions and thin Drone API dispatch. No mission parser."""

import copy
import math
import time
import uuid

from jsonschema import Draft202012Validator
from python.perception.classes import YOLO_CLASSES

from python.dcm._paths import ensure_generated_proto_on_path
from python.dcm.actions import (
    ACTION_METHODS,
    build_request,
    local_position,
    navigation_acceptance_radius,
)
from python.dcm.validation_diagnostics import format_schema_error
from python.dcm.viewing import VIEW_SCHEMA, canonical_view_arguments, resolve_view

# Native ActionExecutor heading/attitude contracts, independent of mission text.
HEADING_TOLERANCE_DEG = 5.0
ATTITUDE_MAX_AGE_MS = 1000

def obj(properties=None, required=()):
    return {
        "type": "object",
        "properties": properties or {},
        "required": list(required),
        "additionalProperties": False,
    }


NUMBER = {"type": "number"}
POSITIVE = {"type": "number", "exclusiveMinimum": 0}
BOOL = {"type": "boolean"}
TEXT = {"type": "string", "minLength": 1}
CAMERA = {"type": "string", "enum": ["forward_d435i", "downward_brio"]}
DETECTION_CLASSES = {"type": "array", "items": {"type": "string", "enum": sorted(YOLO_CLASSES)},
                     "minItems": 1, "maxItems": len(YOLO_CLASSES), "uniqueItems": True}
ORBIT_CAMERA = {"type": "string", "enum": ["forward_d435i", "downward_brio", "none"],
                "description": "Requested target view: front/forward camera=forward_d435i; downward camera=downward_brio. none means the user requested only flight/heading, with no camera visibility requirement."}
NAV = {"north_m": NUMBER, "east_m": NUMBER, "altitude_agl_m": NUMBER}
SPEED = {"maximum_ground_speed_mps": POSITIVE, "minimum_clearance_m": {**POSITIVE, "description":"Optional obstacle-path clearance; omit for controller policy. Not target standoff."}}
POINT = obj(
    {
        **NAV,
        **SPEED,
        "heading_deg": NUMBER,
        "hold_time_s": {"type": "number", "minimum": 0},
        "fly_through": BOOL,
    },
    NAV,
)
FLIGHT = {
    "arm": obj(),
    "disarm": obj(),
    "takeoff": obj(
        {"target_altitude_agl_m": POSITIVE, "maximum_climb_rate_mps": POSITIVE},
        ["target_altitude_agl_m"],
    ),
    "goto": obj({**NAV, **SPEED, "heading_deg": NUMBER}, NAV),
    "move_relative": obj(
        {"forward_m": NUMBER, "right_m": NUMBER, "up_m": NUMBER, **SPEED}
    ),
    "execute_route": obj(
        {
            "points": {"type": "array", "items": POINT, "minItems": 1, "maxItems": 256},
            "completion": {"enum": ["hold", "return_home", "land"]},
        },
        ["points"],
    ),
    "hold": obj({"duration_s": POSITIVE}),
    "yaw": obj({"angle_deg": {**NUMBER, "description": "Relative turn from current heading in degrees; positive clockwise, negative counterclockwise. Not an absolute heading."}, "rate_deg_s": {"type": "number", "minimum": 5, "maximum": 60}}, ["angle_deg"]),
    "face_heading": obj({"view": VIEW_SCHEMA}, ["view"]),
    "orbit": obj(
        {
            "center_north_m": NUMBER,
            "center_east_m": NUMBER,
            "center_reference": {"enum": ["home", "launch"], "description": "home is the autopilot home location; launch is the recorded physical takeoff position. These can differ; use launch for the takeoff point."},
            "camera": ORBIT_CAMERA,
            "target_altitude_agl_m": NUMBER,
            "radius_m": POSITIVE,
            "altitude_agl_m": NUMBER,
            "clockwise": BOOL,
            "revolutions": POSITIVE,
            **SPEED,
            "face_center": {**BOOL, "default": True,
                            "description": "Default true: continuously point the nose at the orbit center. False explicitly keeps the entry heading and is incompatible with a requested target camera view."},
            "viewpoints": {"type": "integer", "minimum": 0},
            "viewpoint_dwell_s": POSITIVE,
        },
        ["radius_m", "altitude_agl_m"],
    ),
    "return_home": obj({"land_at_home": {**BOOL, "description": "True(default) lands; false holds airborne at home."}}),
    "land": obj(),
    "cancel": obj(),
}
def object_motion_schema(properties, required=()):
    # Complete typed alternatives are required by llama.cpp's JSON grammar;
    # a root object plus required-only oneOf branches can generate empty calls.
    visual_description = {**TEXT, 'description':
        'Visible object appearance or image-relative relationships. Compass directions and waypoint names are not visual attributes; orient with face_heading first. Do not include an unobservable geographic label.'}
    return {"oneOf": [obj({reference: visual_description if reference == 'description' else TEXT, **properties}, [reference, *required])
                      for reference in ("target_id", "description")]}



TARGET_FLIGHT = {
    "approach_target": object_motion_schema({"standoff_m": POSITIVE,
                            "maximum_ground_speed_mps": POSITIVE}, ["standoff_m"]),
    "orbit_target": object_motion_schema({"standoff_m": POSITIVE,
                         "revolutions": POSITIVE, "clockwise": BOOL,
                         "camera": ORBIT_CAMERA,
                         "maximum_ground_speed_mps": POSITIVE}, ["standoff_m"]),
    "overfly_target": object_motion_schema({"clearance_above_m": POSITIVE,
                           "maximum_ground_speed_mps": POSITIVE}),
}
from python.dcm.scan_policy import EXIT_POLICY

WORKFLOW_FLIGHT = {
    "scan_heading": obj({
        "exit_policy": copy.deepcopy(EXIT_POLICY),
        "step_deg": {"type": "number", "exclusiveMinimum": 0, "maximum": 360},
        "sweep_deg": {"type": "number", "exclusiveMinimum": 0, "maximum": 360},
        "clockwise": BOOL, "rate_deg_s": {"type": "number", "minimum": 5, "maximum": 60},
        "observation": {"enum": ["detect", "scene"]}, "camera": CAMERA,
        "classes": DETECTION_CLASSES,
        "question": {**TEXT, "maxLength": 240}, "question_type": {"enum": ["condition", "description"]}, "stop_on_detection": TEXT,
        "minimum_count": {"type": "integer", "minimum": 1},
        "repeat_until_match": BOOL,
        "return_on": {"enum": ["observation", "sweep"]},
    }, ["step_deg"]),
    "resume_heading_scan": obj({"scan_id": TEXT}, ["scan_id"]),
    "scan_area": obj({
        "exit_policy": copy.deepcopy(EXIT_POLICY),
        "shape": {"enum": ["square", "rectangle", "circle"],
                  "description": "Square has equal sides; rectangle has independent width_m and length_m; circle uses radius_m."},
        "coverage": {"enum": ["perimeter", "area"]},
        "width_m": {**POSITIVE, "description": "Square side or rectangle side across the heading."},
        "length_m": {**POSITIVE, "description": "Rectangle side along the heading; omit for square and circle."},
        "radius_m": POSITIVE,
        "lane_spacing_m": POSITIVE, "altitude_agl_m": POSITIVE,
        "heading_deg": NUMBER,
        "clockwise": BOOL, "maximum_ground_speed_mps": POSITIVE,
        "camera_heading_deg": NUMBER, "hold_time_s": {"type": "number", "minimum": 0},
        "repetitions": {"type": "integer", "minimum": 1, "maximum": 10},
        "observation": {"enum": ["none", "scene", "detect"]}, "camera": CAMERA,
        "observation_count": {"type": "integer", "minimum": 1, "maximum": 16},
        "question": {**TEXT, "maxLength": 240}, "question_type": {"enum": ["condition", "description"]},
        "classes": DETECTION_CLASSES,
        "stop_on_detection": TEXT, "minimum_count": {"type": "integer", "minimum": 1},
        "return_on": {"enum": ["observation", "completion"]},
    }, ["shape", "coverage"]),
    "resume_scan": obj({"scan_id": TEXT}, ["scan_id"]),
}
# Plain geometric flight and observation scans have distinct intents. Keep
# one shared parameter contract/implementation without making camera-free
# flight discover a special option buried in a perception tool.
WORKFLOW_FLIGHT["fly_pattern"] = obj({
    name: copy.deepcopy(schema)
    for name, schema in WORKFLOW_FLIGHT["scan_area"]["properties"].items()
    if name not in {"observation", "camera", "observation_count", "question", "question_type",
                    "classes", "stop_on_detection", "minimum_count", "return_on", "exit_policy"}
}, ["shape", "coverage"])
SCHEMAS = copy.deepcopy({**FLIGHT, **TARGET_FLIGHT, **WORKFLOW_FLIGHT})
SEQUENCE_FLIGHT = {name: schema for name, schema in FLIGHT.items()
                   if name != 'execute_route'}
# Ordinary geometric maneuvers are directly callable; parameter-heavy
# batching and observation workflows remain discoverable on demand.
DEFAULT_CONTROLLER_TOOLS = frozenset({'orbit', 'fly_pattern'})
OPTIONAL_TOOL_SUMMARIES = {
    'execute_sequence': 'Batch two or three unconditional primitive actions after loading this schema.',
    'goto': 'Fly to supplied coordinates, home/launch or a saved navigation pose.',
    'orbit': 'Circle an object or coordinate center; supports target-facing camera intent.',
    'localize_target': 'Acquire/update object geometry from camera, depth and LiDAR.',
    'scan_heading': 'Turn-settle-observe cycles, conditional stopping and resumable scans.',
    'scan_area': 'Geometric area/perimeter flight with sampled camera observations.',
    'fly_pattern': 'Camera-free geometric perimeter/area flight delegated to ArduPilot.',
    'execute_route': 'Explicit waypoint list delegated to ArduPilot AUTO.'}
OPTIONAL_TOOLS = tuple(OPTIONAL_TOOL_SUMMARIES)
SCHEMAS.update(
    {
        "load_tools": obj({"names": {"type": "array", "items": {"enum": list(OPTIONAL_TOOLS)}, "minItems": 1, "uniqueItems": True}}, ["names"]),
        "execute_sequence": obj(
            {
                "actions": {
                    "type": "array",
                    "minItems": 2,
                    "maxItems": 3,
                    # Each action carries exactly its own contract. A union of
                    # all argument properties invites models to fill irrelevant
                    # fields and loses the name/arguments relationship.
                    "items": {"oneOf": [
                        obj({"name": {"const": name},
                             "arguments": copy.deepcopy(schema)},
                            ["name", "arguments"])
                        for name, schema in SEQUENCE_FLIGHT.items()
                    ]},
                }
            },
            ["actions"],
        ),
        "detect": obj(
            {
                "classes": DETECTION_CLASSES,
                "camera": CAMERA,
            },
            ["classes"],
        ),
        "inspect_scene": obj(
            {"question": {**TEXT, "maxLength": 240}, "camera": CAMERA,
             "question_type": {"enum": ["condition", "description"], "description": "condition: answer a yes/no visual question; false is a valid result. description: describe visible appearance or scene; an unseen subject is unanswerable."}}, ["question"]
        ),
        "assess_landing_zone": obj(),
        "localize_target": obj(
            {"view": VIEW_SCHEMA, "view_heading_deg": {"type":"number", "minimum":0, "exclusiveMaximum":360}, "description": {**TEXT, 'description': 'Visual subject in the current image. Compass labels are not observable in pixels: face the requested direction first, then describe the object by visible features.'}, "new_target": {**BOOL, "description": "Default false: add measurements to the same physical object across viewpoints. True explicitly discards prior views and starts a distinct target; do not use true just to refresh a measurement."}}, ["description"]
        ),
        "get_context": obj({"include_geometry": BOOL, "include_waypoints": {**BOOL, "description": "Load saved vehicle navigation poses when a named waypoint is requested; these are not physical object positions."}}),
        "read_action_result": obj({
            "action_index": {"type": "integer", "minimum": 1},
            "path": {"type": "array", "maxItems": 16,
                     "items": {"oneOf": [{"type": "string"}, {"type": "integer", "minimum": 0}]}},
            "offset": {"type": "integer", "minimum": 0},
            "limit": {"type": "integer", "minimum": 1, "maximum": 16}}, ['action_index']),
        "ask_user": obj({"question": TEXT}, ["question"]),
        "finish": obj(
            {
                "outcome": {
                    "enum": ["completed", "incomplete", "rejected", "cancelled"]
                },
                "message": TEXT,
            },
            ["outcome", "message"],
        ),
    }
)
for name in SCHEMAS:
    for branch in SCHEMAS[name].get("oneOf", [SCHEMAS[name]]):
        branch["properties"]["mission_complete"] = {
            "type": "boolean",
            "description": "Optional completion hint, never a mission verdict.",
        }
DESCRIPTIONS = {
    'load_tools': 'Load optional navigation/observation tool schemas for subsequent decisions. Read-only; performs no flight or observation. Names and purposes are listed in context.optional_tools.',
    'face_heading': 'Hold airborne position/altitude and face an absolute compass heading (north0/east90/south180/west270).',
    'orbit': 'Fly a circular path around a center; face it by default. Object destinations use standoff_m from measured hull; coordinate/home/launch destinations use radius_m and altitude_agl_m. Select camera explicitly; none requests heading-only flight. Cameras cannot tilt.',
    'scan_heading': 'Airborne turn–settle–observe workflow. step_deg positive; sweep_deg=step_deg makes one station. clockwise=false turns left. Select exit_policy explicitly. finish_at_match ends the scan on stop_on_detection/minimum_count; pause_at_match permits resuming after a nested branch. each_observation yields every view; complete_coverage is unconditional. repeat_until_match searches across sweeps. Resume with resume_heading_scan.',
    'resume_heading_scan': 'Resume a saved angular scan, completing any pending observation before turning again. Use its scan_id.',
    'yaw': 'One airborne relative turn without observation: positive clockwise, negative counterclockwise. Compass headings use face_heading; turn/observe cycles use scan_heading.',
    'goto': 'Fly to coordinates, home, launch or a saved navigation pose. Arrival geometry belongs inside destination. Object-relative movement uses approach_target for surface standoff or overfly_target for above-object travel.',
    'arm': 'Enable motors without takeoff. Detection works disarmed: satisfy pre-arming conditions first. takeoff already arms if needed.',
    'takeoff': 'Reach the requested altitude above home. From landed, arm if needed and take off. If already flying, change height vertically with native goto while holding current horizontal position. Explicit maximum_climb_rate_mps applies only to ground departure.',
    'return_home': 'Return to measured autopilot home and land by default. land_at_home=false returns and holds. Return flight may change yaw.',
    "move_relative": "Move a specified body-frame distance, or one derived from a current metric measurement. Never guess a distance to reach a named waypoint or visual object: ask about missing waypoints, or use object-motion tools. Forward/right follow current heading; up increases altitude; negative values reverse direction.",
    'execute_sequence': 'Execute 2–3 contiguous unconditional flight actions in order, stopping at failure. Do not cross an intervening observation or decision.',
    'execute_route': 'Upload local waypoints as one ArduPilot AUTO route. Stopped checkpoints honor heading/dwell; fly_through=true permits continuous course-facing motion without them. Altitudes are above home.',
    'fly_pattern': 'ArduPilot geometric flight without observations, anchored at current position. Square needs width; rectangle width extends right, length forward; circle center lies one radius forward; area needs lane_spacing_m. Heading defaults current; clockwise=false starts rightward. Perimeters return to start. repetitions repeats; hold_time_s dwells; camera_heading_deg fixes heading.',
    'scan_area': 'Geometric flight plus sampled camera stations, anchored at current position. Rectangle width extends right, length forward; square needs width, circle radius, area lane_spacing_m. observation defaults scene, camera downward_brio, observation_count4. Select exit_policy explicitly: each_observation for arbitrary branching, finish_at_match to end at a detector match, pause_at_match for nested branches that resume, complete_coverage for unconditional scans. Resume with resume_scan.',
    'resume_scan': 'Resume a saved area scan with its original anchor and remaining stations.',
    'detect': 'Detect/count supported class enum labels in a current image, including while disarmed. Unsupported objects use inspect_scene/localize_target. Counts are thresholded detections, not guaranteed totals; default camera forward_d435i.',
    'inspect_scene': 'Ask onboard VLM about visible appearance, objects or relationships. Samples configured detector counts separately when available; VLM prose is qualitative evidence, not metric geometry or measured counts.',
    'localize_target': 'Associate a visible object with measured LiDAR/depth. Returns surface range, footprint quality and reusable target_id; another viewpoint may be needed. Reuse the query to accumulate views.',
    'approach_target': 'Move to a measured surface standoff from a visible or registered object destination. Recede only if already closer. This does not establish being above it. Another viewpoint may be needed to measure its geometry.',
    'orbit_target': 'Circle a measured object with nose facing it. Supply target_id or visual_query. standoff_m is minimum distance from its observed hull, not center radius or global obstacle clearance; wall distance varies for non-round shapes. Partial geometry may require two measured side views first. Select camera for visual intent.',
    'overfly_target': 'Fly ABOVE a physical object: climb above its measured top, then move over its footprint center. destination selects visible_object with visual_query and view, or registered_target with target_id. An observation waypoint is not above the object. Requires measured height/footprint. Does not land; assess_landing_zone before landing.',
    'get_context': 'Refresh state/health/perception; include_geometry adds obstacle bounds. Normally refreshed automatically.',
    'read_action_result': 'Read retained history by executed_actions index. Omit path to discover receipt fields. Optional path lists exact returned keys/array indices, starting at action/arguments/result; follow retained_details paths or returned fields. An absent path returns path_found=false and valid paths, never the missing data. offset/limit page large data. Read-only: historical data never establishes a current condition.',
    'ask_user': 'Ask one concise question about operator intent or ambiguity left after available sensing. Use available tools yourself; do not ask the operator to perform their actions. The answer continues this mission.',
    'finish': 'Report measured outcome, operating defaults/assumptions used and limitations. completed requires all requested work; incomplete/rejected explains what prevents progress.',
}
SCHEMAS['inspect_scene']['properties']['view'] = VIEW_SCHEMA
SCHEMAS['detect']['properties']['view'] = VIEW_SCHEMA


TOOLS = [
    {
        "type": "function",
        "function": {
            "name": name,
            "description": DESCRIPTIONS.get(
                name, name.replace("_", " ") + "; see DRONE_API.md."
            ),
            "parameters": schema,
        },
    }
    for name, schema in SCHEMAS.items()
]


def directional_yaw_schema(schema):
    schema = copy.deepcopy(schema)
    schema['properties']['angle_deg'] = {
        'type': 'number', 'minimum': 0,
        'description': 'Magnitude of the relative turn in degrees.'}
    schema['properties']['direction'] = {
        'enum': ['clockwise', 'counterclockwise'],
        'description': 'Clockwise turns right; counterclockwise turns left.'}
    schema['required'] = list(dict.fromkeys([*schema.get('required', []), 'direction']))
    return schema


def turn_arguments(name, arguments, *, native=False):
    """Translate explicit turn intent at the API boundary, retaining legacy callers."""
    args = copy.deepcopy(arguments)
    if not isinstance(args, dict):
        return args
    if name == 'execute_sequence' and isinstance(args.get('actions'), list):
        for child in args['actions']:
            if isinstance(child, dict) and isinstance(child.get('arguments'), dict):
                child['arguments'] = turn_arguments(
                    child.get('name'), child['arguments'], native=native)
    if name != 'yaw':
        return args
    angle = args.get('angle_deg')
    if native and 'direction' not in args and type(angle) in (int, float) and math.isfinite(angle):
        args.update(angle_deg=abs(angle), direction='clockwise' if angle >= 0 else 'counterclockwise')
    elif not native and 'direction' in args:
        direction = args.pop('direction')
        if direction not in ('clockwise', 'counterclockwise'):
            raise ValueError('Yaw direction must be clockwise or counterclockwise')
        if type(angle) not in (int, float) or not math.isfinite(angle) or angle < 0:
            raise ValueError('Directional yaw needs a finite nonnegative angle magnitude')
        args['angle_deg'] = angle if direction == 'clockwise' else -angle
    return args


# A repeated observation carries only its immediate continuation, not a route
# or future branch. Null explicitly denotes a one-shot observation.
SCHEMAS['inspect_scene']['properties']['repeat_motion'] = {
    'description': 'The user-requested movement repeated between views; null for a one-shot inspection. '
                   'Retain the requested direction/rate. This declares intent and executes no movement.',
    'oneOf': [{'type': 'null'}, *[
        obj({'name': {'const': name}, 'arguments': schema}, ['name', 'arguments'])
        for name, schema in (
            ('yaw', directional_yaw_schema(SCHEMAS['yaw'])),
            ('move_relative', copy.deepcopy(SCHEMAS['move_relative'])))]]}


def decision_tools(context, angular_scans=None, area_scans=None, loaded_tools=None, require_progress=False):
    """Bind opaque handles to exact currently known values, without rewriting calls.

    Geometry validity and workflow ownership remain backend responsibilities.
    A stale registered target remains selectable so it can be refreshed.
    """
    targets = context.get('targets', [])
    target_ids = sorted({item['target_id'] for item in targets
                         if isinstance(item, dict) and isinstance(item.get('target_id'), str)
                         and item.get('reference_available', True) is not False
                         and item['target_id']}) if isinstance(targets, list) else []
    def resumable_ids(registry):
        return sorted(key for key, saved in (registry or {}).items()
                      if isinstance(key, str) and key and isinstance(saved, dict)
                      and not saved.get('completed', False))
    scan_ids = {
        'resume_heading_scan': resumable_ids(angular_scans),
        'resume_scan': resumable_ids(area_scans),
    }
    advertised = context.get('motion_limits', {})
    limits = {}
    for parameter, capability in (
        ('maximum_climb_rate_mps', 'maximum_climb_speed_mps'),
        ('maximum_ground_speed_mps', 'maximum_horizontal_speed_mps'),
    ):
        value = advertised.get(capability) if isinstance(advertised, dict) else None
        if isinstance(value, (int, float)) and not isinstance(value, bool) and math.isfinite(value) and value > 0:
            limits[parameter] = value

    def bind_limits(schema):
        # Rebuild nested schemas rather than mutating shared numeric aliases.
        # This also covers sequence variants and native-route point arguments.
        if isinstance(schema, list):
            return [bind_limits(item) for item in schema]
        if not isinstance(schema, dict):
            return schema
        result = {key: bind_limits(value) for key, value in schema.items()}
        properties = result.get('properties', {})
        if 'classes' in properties:
            # Native generation must see the same finite sensor vocabulary as
            # dispatch validation. A prose context list cannot constrain the
            # grammar and allowed repeated unsupported-class proposals.
            labels = context.get('detector_classes') or DETECTION_CLASSES['items']['enum']
            properties['classes']['items'] = {'type':'string', 'enum':list(labels)}
            properties['classes']['description'] = (
                'Supported fixed detector labels. For other visual subjects use inspect_scene or localize_target.')
            properties['classes']['maxItems'] = len(labels)
        if ('target_altitude_agl_m' in properties and context.get('state', {}).get('landed') is False
                and context.get('state', {}).get('armed') is True):
            properties.pop('maximum_climb_rate_mps', None)
        standoff = context.get('defaults', {}).get('object_standoff_m')
        if ('standoff_m' in properties and isinstance(standoff, (int, float))
                and not isinstance(standoff, bool) and math.isfinite(standoff) and standoff > 0):
            properties['standoff_m']['default'] = standoff
            properties['standoff_m']['description'] = (
                'Minimum distance from selected target surface. Preserve an explicit user value; '
                'otherwise use the configured default.')
        clearance = context.get('defaults', {}).get('overflight_clearance_m')
        if ('clearance_above_m' in properties and type(clearance) in (int,float)
                and math.isfinite(clearance) and clearance > 0):
            properties['clearance_above_m']['default'] = clearance
            properties['clearance_above_m']['description'] = (
                'Vertical clearance above the measured target top. Preserve an explicit user value; '
                'otherwise omit this argument to use the configured default.')
        if properties.get('camera', {}).get('enum') == ORBIT_CAMERA['enum']:
            # The model must express visual intent, including an explicit
            # "none" for geometric flight. This applies inside batches too.
            result['required'] = list(dict.fromkeys([*result.get('required', []), 'camera']))
        for name, maximum in limits.items():
            if name in properties:
                item = properties[name]
                item['maximum'] = min(item.get('maximum', maximum), maximum)
        return result

    catalog = []
    for tool in bind_limits(TOOLS):
        function = tool['function']
        # A loaded navigation catalogue must expose the tool that can consume
        # its handles; discovering a waypoint must not require guessing a second
        # unrelated tool-discovery action. This depends on data, not mission text.
        catalogue_navigation = (function['name'] == 'goto'
            and bool(context.get('registered_waypoints', {}).get('entries')))
        if (loaded_tools is not None and function['name'] in OPTIONAL_TOOLS
                and function['name'] not in DEFAULT_CONTROLLER_TOOLS
                and function['name'] not in loaded_tools and not catalogue_navigation):
            continue
        if function['name'] in TARGET_FLIGHT and function['name'] not in ('overfly_target','approach_target'):
            # Native tools distinguish approach from overflight; object orbit
            # still selects its measured reference through orbit.
            continue
        if function['name'] in ('goto', 'orbit', 'overfly_target','approach_target'):
            from python.dcm.destinations import destination_schema, overflight_schema, approach_schema
            function['parameters'] = bind_limits(overflight_schema(SCHEMAS['overfly_target']) if function['name'] == 'overfly_target'
                else approach_schema(SCHEMAS['approach_target']) if function['name']=='approach_target'
                else destination_schema(function['name'], SCHEMAS[function['name']]))
            variants = function['parameters']['properties']['destination']['oneOf']
            waypoints = context.get('registered_waypoints', {}).get('entries', [])
            waypoint_names = sorted({p['name'] for p in waypoints if isinstance(p, dict)
                                     and isinstance(p.get('name'), str) and p['name']
                                     and (function['name'] != 'orbit' or p.get('role') == 'orbit_center')})
            for branch in list(variants):
                if branch['properties']['type']['const'] == 'named_waypoint':
                    if function['name'] == 'orbit' and not waypoint_names:
                        variants.remove(branch)
                    elif waypoint_names:
                        branch['properties']['waypoint_name']['enum'] = waypoint_names
            for branch in list(variants):
                if branch['properties']['type']['const'] == 'registered_target':
                    if target_ids:
                        branch['properties']['target_id']['enum'] = target_ids.copy()
                    else:
                        variants.remove(branch)
        if function['name'] == 'inspect_scene':
            function['parameters']['required'].append('question_type')
        if function['name'] == 'yaw':
            function['parameters'] = directional_yaw_schema(function['parameters'])
            function['description'] = ('Turn by the angle magnitude in the explicit direction. '
                                       'This is relative, not an absolute compass heading.')
        if function['name'] == 'fly_pattern':
            function['parameters']['properties'].pop('altitude_agl_m', None)
            function['description'] += (' This horizontal pattern retains current altitude; '
                'use takeoff first for an explicitly requested altitude change.')
        if function['name'] == 'execute_sequence':
            variants = function['parameters']['properties']['actions']['items']['oneOf']
            function['parameters']['properties']['actions']['items']['oneOf'] = [
                branch for branch in variants if branch['properties']['name']['const'] not in ('goto', 'orbit')]
            for branch in function['parameters']['properties']['actions']['items']['oneOf']:
                if branch['properties']['name']['const'] == 'yaw':
                    branch['properties']['arguments'] = directional_yaw_schema(
                        branch['properties']['arguments'])
        if function['name'] == 'read_action_result':
            count = context.get('retained_action_result_count', 0)
            if type(count) is not int or count < 1:
                continue
            function['parameters']['properties']['action_index']['maximum'] = count
        properties = function['parameters'].get('properties', {})
        if function['name'] in scan_ids and not scan_ids[function['name']]:
            continue
        choices = scan_ids.get(function['name'])
        if choices and 'scan_id' in properties:
            properties['scan_id'] = {**properties['scan_id'], 'enum': choices.copy()}
        if function['name'] == 'localize_target' and 'description' in properties:
            # Give the model a typed image-space query rather than an
            # ambiguous target name. Existing Python callers retain their
            # description keyword through the compatibility adapter below.
            properties['visual_query'] = properties.pop('description')
            properties['visual_query']['description'] = (
                'Object class, visual description or image-relative referent. '
                'A class alone can begin acquisition; direction uses view.')
            function['parameters']['required'] = [
                'visual_query' if key == 'description' else key
                for key in function['parameters'].get('required', [])]
            function['parameters']['required'].append('view')
            function['parameters']['properties'].pop('view_heading_deg',None)
            function['description'] = function['description'].replace('description', 'visual_query')
            if isinstance(function['parameters'].get('description'), str):
                function['parameters']['description'] = function['parameters']['description'].replace('description', 'visual_query')
        # Explicit legacy callers can still request the old diagnostic. The
        # live catalogue carries no self-authored plan field.
        if require_progress:
            function['parameters']['properties'] = {
                'mission_progress': {'type': 'string', 'minLength': 1, 'maxLength': 600},
                **function['parameters']['properties']}
            function['parameters']['required'].append('mission_progress')
        if function['name'] in ('scan_heading', 'scan_area'):
            # Legacy scheduling aliases remain available to Python callers;
            # the agent makes one explicit lifecycle choice instead.
            function['parameters']['properties'].pop('return_on', None)
            function['parameters']['required'].append('exit_policy')
        if function['name'] == 'inspect_scene':
            function['parameters']['required'].append('repeat_motion')
        catalog.append(tool)
    from python.dcm.active_visual_feedback import bind_recovery_tools
    return bind_recovery_tools(catalog, context.get('current_visual_condition'))


def validate_call(name, arguments, *, tools=None, detector_classes=None):
    """Validate the advertised call contract, or the static API for direct callers."""
    if not isinstance(name, str) or name not in SCHEMAS:
        raise ValueError(f"Unknown tool {name!r}; use a listed tool")
    schema = SCHEMAS[name]
    reference_call = (name in ('goto', 'orbit', 'overfly_target','approach_target') and isinstance(arguments, dict)
                      and 'destination' in arguments)
    if reference_call:
        from python.dcm.destinations import destination_schema, overflight_schema, approach_schema
        schema = overflight_schema(schema) if name == 'overfly_target' else approach_schema(schema) if name=='approach_target' else destination_schema(name,schema)
    if tools is not None:
        schema = next((tool['function']['parameters'] for tool in tools
                       if tool['function']['name'] == name), None)
        if schema is None:
            raise ValueError(f"Tool {name!r} is unavailable in this decision; use a listed tool")
    if 'classes' in schema.get('properties', {}):
        schema = copy.deepcopy(schema)
        declared = schema['properties']['classes']['items'].get('enum')
        supported = (declared if declared is not None else DETECTION_CLASSES['items']['enum'])
        if detector_classes is not None:
            supported = ([label for label in supported if label in detector_classes]
                         if tools is not None else detector_classes)
        schema['properties']['classes']['items'] = {'type':'string', 'enum':list(supported)}
        schema['properties']['classes']['maxItems'] = len(supported)
    arguments = canonical_view_arguments(name, copy.deepcopy(arguments))
    if name == 'inspect_scene' and tools is not None:
        # Saved pre-continuation calls remain read-only by default. Live native
        # generation must explicitly supply the field in the advertised schema.
        arguments.setdefault('repeat_motion', None)
    arguments = turn_arguments(name, arguments, native=tools is not None)
    if 'mission_progress' not in schema.get('properties', {}):
        # Read old saved/native turns without forwarding diagnostic text to a
        # physical controller or advertising it to current model decisions.
        arguments.pop('mission_progress', None)
    if reference_call:
        from python.dcm.destinations import canonical_destination_arguments
        arguments = canonical_destination_arguments(name, arguments)
    if name in (*TARGET_FLIGHT, 'localize_target'):
        if 'description' in arguments and 'visual_query' in arguments:
            raise ValueError('Supply one visual_query, not both query aliases')
        properties = schema.get('properties', {})
        if tools is not None and 'visual_query' in properties and 'description' in arguments:
            arguments['visual_query'] = arguments.pop('description')
        elif tools is None and 'visual_query' in arguments:
            arguments['description'] = arguments.pop('visual_query')

    # JSON Schema accepts NaN/Infinity as numbers; physical quantities cannot.
    def finite(value):
        if isinstance(value, float) and not math.isfinite(value):
            raise ValueError("Tool arguments must contain finite numbers")
        if isinstance(value, dict):
            for child in value.values():
                finite(child)
        elif isinstance(value, list):
            for child in value:
                finite(child)

    finite(arguments)
    errors = sorted(
        Draft202012Validator(schema).iter_errors(arguments),
        key=lambda e: str(e.path),
    )
    if errors:
        error = errors[0]
        raise ValueError(format_schema_error(name, error))
    if reference_call:
        from python.dcm.destinations import unpack_destination, validate_destination
        operation = 'goto' if name in ('overfly_target','approach_target') else name
        bound = dict(arguments, object_position='above') if name == 'overfly_target' else arguments
        validate_destination(operation, unpack_destination(operation, bound))
    if tools is not None and name in TARGET_FLIGHT and not reference_call:
        references = [key for key in ('target_id', 'description', 'visual_query') if key in arguments]
        if len(references) != 1:
            raise ValueError(f"{name}: supply exactly one of target_id or description; received {references}")
    if name in ('scan_heading', 'scan_area'):
        from python.dcm.scan_policy import scan_arguments
        scan_arguments(arguments, default_observation='detect' if name == 'scan_heading' else 'scene',
                       coverage_return='sweep' if name == 'scan_heading' else 'completion')
    if name == "execute_sequence" and tools is None:
        for action in arguments["actions"]:
            errors = list(
                Draft202012Validator(FLIGHT[action["name"]]).iter_errors(
                    action["arguments"]
                )
            )
            if errors:
                raise ValueError(format_schema_error(action['name'], errors[0]))


def observation_call(args, *, default_observation, default_camera, client=None):
    """Validate the exact embedded sensor call before a workflow can move."""
    observation = args.get('observation', default_observation)
    if observation not in ('detect', 'scene'):
        raise ValueError('Observation workflow requires detect or scene')
    name = 'detect' if observation == 'detect' else 'inspect_scene'
    params = {'camera': args.get('camera', default_camera)}
    if name == 'detect':
        params['classes'] = args.get('classes')
    else:
        params['question'] = args.get('question', 'Describe visible objects and any uncertainty in this view.')
        params['question_type'] = args.get('question_type', 'description')
    getter = getattr(client, 'camera_detector_classes', None) if name == 'detect' else None
    validate_call(name, params, detector_classes=getter() if callable(getter) else None)
    return name, params


def message_dict(message):
    if isinstance(message, dict):
        return copy.deepcopy(message)
    from google.protobuf.json_format import MessageToDict

    return MessageToDict(
        message,
        preserving_proto_field_name=True,
        always_print_fields_with_no_presence=True,
    )


class ToolExecutor:
    def __init__(self, client, interrupt=None, session=None):
        ensure_generated_proto_on_path()
        from icarus.v1 import action_pb2

        self.pb = action_pb2
        self.client, self.interrupt = client, interrupt
        self.session = session
        self.expected_interrupt = interrupt.sequence() if interrupt else 0

    def interrupted(self):
        return (
            self.interrupt is not None
            and self.interrupt.sequence() != self.expected_interrupt
        )

    def execute(self, name, arguments):
        vocabulary = None
        if isinstance(arguments, dict) and 'classes' in arguments:
            getter = getattr(self.client, 'camera_detector_classes', None)
            if callable(getter):
                vocabulary = getter()
        validate_call(name, arguments, detector_classes=vocabulary)
        arguments = turn_arguments(name, arguments)
        if name in (*TARGET_FLIGHT, 'localize_target') and 'visual_query' in arguments:
            arguments['description'] = arguments.pop('visual_query')
        complete = arguments.pop("mission_complete", False)
        if self.interrupted():
            return {
                "status": "CANCELLED",
                "message": "Operator interrupted the mission.",
            }
        visual_feedback = getattr(self.session, 'visual_feedback', None)
        if visual_feedback and visual_feedback.recovery_required(name, arguments):
            return {'status': 'VIEWPOINT_REQUIRED', 'no_action_executed': True,
                    'message': 'Localization contradicted the visual answer for this subject. Resume its observation '
                               'continuation and check fresh evidence, or clarify if none exists. No movement sent.',
                    'evidence': {'visual_condition': visual_feedback.snapshot()}}
        if visual_feedback and not visual_feedback.permits_motion(name, arguments):
            return {'status': 'CONTINUATION_REQUIRED', 'no_action_executed': True,
                    'message': 'Observation recovery must resume its saved movement, not introduce a new maneuver. '
                               'Use the continuation or clarify with the operator. No movement sent.',
                    'evidence': {'visual_condition': visual_feedback.snapshot()}}
        if visual_feedback and visual_feedback.unresolved_approach(name, arguments):
            return {'status': 'OBSERVATION_UNAVAILABLE', 'no_action_executed': True,
                    'message': 'The current visual question is unanswered. Obtain a valid inspection or measured '
                               'localize_target result before approaching a visible object. No movement sent.',
                    'evidence': {'visual_condition': visual_feedback.snapshot()}}
        orientation = None
        requested_view = None
        heading = None
        try:
            if name == 'face_heading' and 'view' in arguments:
                arguments['heading_deg'] = resolve_view(arguments.pop('view'), message_dict(self.client.state()))
            if name in ('inspect_scene','localize_target','detect'):
                requested_view = arguments.pop('view', arguments.pop('view_heading_deg', None))
                if requested_view is not None and requested_view != 'current':
                    state = message_dict(self.client.state())
                    heading = resolve_view(requested_view, state)
                    yaw = state.get('attitude',{}).get('yaw_rad')
                    if type(yaw) not in (int,float) or not math.isfinite(yaw):
                        raise ValueError('Measured heading is required before directional observation')
                    if abs(math.remainder(heading-math.degrees(yaw),360)) > HEADING_TOLERANCE_DEG:
                        if state.get('landed') or not state.get('armed'):
                            return {'status':'NEEDS_CLARIFICATION','requires_user_choice':True,
                                'question':'The requested view needs the aircraft to turn, but it is landed. Should the view or flight requirement change?',
                                'message':'No motors or flight command were sent for the camera request.'}
                        orientation = self.execute('face_heading', {'view':heading})
                        if orientation['status'] != 'SUCCEEDED':
                            orientation.setdefault('evidence',{})['motion_command_issued']=True
                            return orientation
            if name in ('goto', 'orbit', 'overfly_target','approach_target') and 'destination' in arguments:
                from python.dcm.destinations import execute_destination
                operation = 'goto' if name in ('overfly_target','approach_target') else name
                bound = dict(arguments, object_position='above') if name == 'overfly_target' else arguments
                result = execute_destination(self, operation, bound)
            elif name == "execute_sequence":
                results = []
                for item in arguments["actions"]:
                    result = self.execute(item["name"], item["arguments"])
                    results.append({**item, "result": result})
                    if result["status"] != "SUCCEEDED":
                        break
                result = {
                    "status": results[-1]["result"]["status"],
                    "actions": results,
                    "message": f"Executed {len(results)} of {len(arguments['actions'])} actions.",
                }
                if results[-1]['result'].get('requires_user_choice'):
                    result.update(requires_user_choice=True,
                                  question=results[-1]['result']['question'])
            elif name == "takeoff":
                state = message_dict(self.client.state())
                if state.get('landed') is False and state.get('armed') is True:
                    if 'maximum_climb_rate_mps' in arguments:
                        result = {'status': 'NEEDS_CLARIFICATION', 'requires_user_choice': True,
                            'question': 'The aircraft is already flying. May it change altitude using the configured flight-controller climb rate?',
                            'message': 'An explicit takeoff climb rate cannot be applied to an airborne goto; no command sent.'}
                    else:
                        position = state.get('local_position_ned', {})
                        if not position.get('origin_id') or any(type(position.get(k)) not in (int,float)
                            or not math.isfinite(position[k]) for k in ('north_m','east_m')):
                            raise ValueError('Measured position and origin are required for an airborne altitude change')
                        result = self._flight('goto', {'north_m':position['north_m'],
                            'east_m':position['east_m'], 'altitude_agl_m':arguments['target_altitude_agl_m']},
                            expected_origin_id=position['origin_id'])
                        result.setdefault('evidence', {})['altitude_transition'] = {
                            'reference':'above_home', 'requested_altitude_m':arguments['target_altitude_agl_m'],
                            'already_airborne':True, 'controller':'native_goto', 'horizontal_goal':'hold_current_position'}
                else:
                    launch = copy.deepcopy(state) if state.get('landed') else None
                    if state.get("landed") and not state.get("armed"):
                        arm = self.execute("arm", {})
                        if arm["status"] != "SUCCEEDED":
                            return arm
                    result = self._flight(name, arguments)
                    if (launch is not None
                            and result.get('status') in ('FAILED', 'REJECTED')
                            and result.get('reason_code') in (
                                'REASON_CODE_AUTOPILOT_REJECTED', 'REASON_CODE_NOT_ARMED')
                            and not self.interrupted()):
                        # A ground arm can expire during a model decision. Its
                        # heartbeat may arrive after the explicit command reject.
                        # Re-arm only after confirming disarmed + landed, once;
                        # never retry a timeout or bypass a live preflight check.
                        disarmed = self._confirm_disarmed_ground()
                        if disarmed is not None:
                            failed = {k: result[k] for k in
                                      ('status', 'reason_code', 'message', 'action_id') if k in result}
                            arm = self.execute('arm', {})
                            if self.interrupted():
                                result = {'status': 'CANCELLED',
                                          'message': 'Operator interrupted takeoff recovery.'}
                            else:
                                result = self._flight(name, arguments) if arm['status'] == 'SUCCEEDED' else arm
                            result['takeoff_recovery'] = {
                                'trigger': 'confirmed_disarmed_ground_after_explicit_rejection',
                                'failed_attempt': failed, 'rearm_status': arm['status']}
                            launch = disarmed
                    if launch is not None and (result['status'] == 'SUCCEEDED'
                            or result.get('state', {}).get('landed') is False):
                        # Preserve the actual launch across subsequent airborne
                        # missions; a rejected takeoff does not establish one.
                        self.client._takeoff_reference = launch
            elif name in TARGET_FLIGHT:
                from python.dcm.target_motion import TargetMotion
                localization = None
                if 'description' in arguments:
                    localization = self.client.localize_target(arguments.pop('description'))
                    target_id = localization.get('target_id')
                    registered = (isinstance(target_id, str) and bool(target_id)
                                  and localization.get('found') is True
                                  and any(item.get('target_id') == target_id
                                          for item in self.client.target_feedback()))
                    if not registered:
                        return {'status': 'FAILED',
                                'message': 'No measured registered target was localized; no flight action executed.',
                                'evidence': {'target_localization': localization}}
                    arguments['target_id'] = target_id
                if self.interrupted():
                    return {'status': 'CANCELLED', 'message': 'Operator interrupted target localization.'}
                result = TargetMotion(self.client, self._flight, self.interrupted).execute(name, arguments)
                if localization is not None:
                    result.setdefault('evidence', {})['target_localization'] = localization
            elif name in WORKFLOW_FLIGHT:
                from python.dcm.scan_workflow import ScanWorkflow
                from python.dcm.angular_scan import AngularScan
                workflow = AngularScan(self) if name in ('scan_heading', 'resume_heading_scan') else ScanWorkflow(self)
                method = 'fly_pattern' if name == 'scan_area' and arguments.get('observation') == 'none' else name
                result = getattr(workflow, method)(**arguments)
            elif name in FLIGHT:
                result = self._flight(name, arguments)
            elif name == "detect":
                # The legacy client uses a multi-camera count window when
                # camera is omitted. The agent tool promises one current view.
                arguments.setdefault("camera", "forward_d435i")
                result = {
                    "status": "SUCCEEDED",
                    "evidence": self.client.detect(**arguments),
                }
            elif name == "inspect_scene":
                camera = arguments.pop("camera", "forward_d435i")
                question_type = arguments.pop("question_type", "description")
                view = self.client.capture_view(camera=camera)
                try:
                    result = {
                        "status": "SUCCEEDED",
                        "evidence": self.client.inspect_scene(
                            arguments["question"], view=view,
                            **({"structured":True, "result_type":"boolean"} if question_type == "condition" else {})
                        ),
                    }
                    result['evidence']['measurement_kind'] = 'qualitative_vlm_answer'
                    result['evidence']['question_type'] = question_type
                    usable = (result['evidence'].get('validity') == 'valid'
                              and type(result['evidence'].get('found')) is bool) if question_type == 'condition' else result['evidence'].get('answerable') is True
                    if not usable:
                        result.update(status='OBSERVATION_UNAVAILABLE', message=(
                            'The camera call completed, but this view cannot answer the requested question. '
                            'The requested inspection is unfinished. Obtain a usable view, or ask the operator '
                            'before changing an explicitly requested camera or viewing constraint. '
                            'Do not report the target description as completed.'))
                    vocabulary = getattr(self.client, 'camera_detector_classes', lambda: [])()
                    if vocabulary and not self.interrupted():
                        # Quantitative feedback comes from the configured
                        # detector, never numbers inferred from VLM prose.
                        # This is a separate current capture with its own pose
                        # and timestamp, not a same-frame sensor-fusion claim.
                        request = {'camera': camera, 'classes': vocabulary}
                        try:
                            detection = {'status': 'SUCCEEDED',
                                'evidence': self.client.detect(**request)}
                        except Exception as error:
                            detection = {'status': 'FAILED', 'message': str(error)}
                        result['evidence']['camera_detection'] = {
                            'arguments': request, 'result': detection,
                            'capture_relationship': 'separate_detector_capture_after_vlm_inspection'}
                finally:
                    # inspect_scene normally consumes the frame; clean up if inference fails early.
                    import shutil

                    shutil.rmtree(view["directory"], ignore_errors=True)
            elif name in ("assess_landing_zone", "localize_target"):
                result = {
                    "status": "SUCCEEDED",
                    "evidence": getattr(self.client, name)(**arguments),
                }
            elif name == 'read_action_result':
                from python.dcm.action_memory import read_action_result
                if self.session is None:
                    raise ValueError('Receipt lookup needs the active mission conversation')
                result = {'status': 'SUCCEEDED',
                          'retained_result': read_action_result(self.session.actions, **arguments)}
            elif name == 'load_tools':
                if self.session is None:
                    raise ValueError('Tool loading needs an active mission session')
                self.session.loaded_tools.update(arguments['names'])
                result = {'status': 'SUCCEEDED', 'loaded_tools': sorted(self.session.loaded_tools),
                          'message': 'Schemas available on the next decision; no flight executed.'}
            elif name == "get_context":
                from python.dcm.agent_context import capture_context

                result = {
                    "status": "SUCCEEDED",
                    "context": capture_context(self.client, **arguments),
                }
            else:
                raise ValueError(
                    "Communication tools must be handled by the conversation"
                )
        except Exception as error:  # noqa: BLE001 - report boundary failures without losing control cleanup
            result = {"status": "FAILED", "message": str(error)}
            if isinstance(getattr(error, 'code', None), str):
                result['error_code'] = error.code
            if orientation is not None:
                # A failed capture after a completed turn must not leave the
                # previous camera direction's detections marked current.
                result['evidence'] = {'motion_command_issued': True}
        if name in ('inspect_scene','localize_target','detect') and not isinstance(result.get('evidence'),dict):
            if result.get('status')=='SUCCEEDED':
                result.update(status='FAILED',message='Perception tool returned no structured evidence')
            result['evidence']={}
        if requested_view is not None:
            result.setdefault('evidence',{}).update(requested_view=requested_view,
                motion_command_issued=orientation is not None)
            if heading is not None:result['evidence']['requested_view_heading_deg']=heading
        if complete and result["status"] == "SUCCEEDED" and result.get("workflow_complete", True) and not self.interrupted():
            result["mission_complete"] = True
        return result

    def _confirm_disarmed_ground(self, timeout_s=2.5):
        deadline = time.monotonic() + timeout_s
        while not self.interrupted():
            state = message_dict(self.client.state())
            if state.get('manual_override_active') or not state.get('landed'):
                return None
            if not state.get('armed'):
                return copy.deepcopy(state)
            if time.monotonic() >= deadline:
                return None
            time.sleep(0.05)
        return None

    def _flight(self, name, args, runtime_target=False, acceptance_radius_m=None, expected_origin_id=None):
        client, pb = self.client, self.pb
        if hasattr(client, "ensure_action_settled"):
            client.ensure_action_settled()
        state = message_dict(client.state())
        if name == 'land' and state.get('landed') is True and state.get('armed') is False:
            from python.dcm.ground_recovery import verify_ground_recovery
            return verify_ground_recovery(client, message_dict)
        orbit_evidence = None
        absolute_heading = None
        if name == 'face_heading':
            yaw = state.get('attitude', {}).get('yaw_rad')
            stamp = int(state.get('attitude_received_at_unix_ms', 0))
            age = time.time() * 1000 - stamp
            if type(yaw) not in (int, float) or not math.isfinite(yaw) or stamp <= 0 or not 0 <= age <= ATTITUDE_MAX_AGE_MS:
                raise ValueError('Facing a heading requires fresh finite measured attitude.')
            absolute_heading = args['heading_deg'] % 360
            delta = math.remainder(absolute_heading - math.degrees(yaw), 360)
            if abs(delta) <= HEADING_TOLERANCE_DEG:
                return {'status': 'SUCCEEDED', 'message': 'Already facing the requested heading; no flight command sent.',
                        'state': state, 'evidence': {'requested_heading_deg': absolute_heading,
                        'measured_heading_deg': math.degrees(yaw) % 360,
                        'absolute_heading_error_deg': abs(delta), 'heading_tolerance_deg': HEADING_TOLERANCE_DEG,
                        'end_state_already_satisfied': True, 'no_flight_command_sent': True}}
            if state.get('landed') or not state.get('armed'):
                raise ValueError('Facing a heading requires an airborne aircraft; landed aircraft cannot rotate.')
            # Preserve the existing hold target. A new waypoint invents a
            # position-arrival task (and 10cm tolerance) for a heading request.
            args = {'angle_deg': delta, **({'rate_deg_s': args['rate_deg_s']} if 'rate_deg_s' in args else {})}
            name = 'yaw'
        if name == 'orbit':
            from python.dcm.camera_geometry import resolve_reference, orbit_camera_feasibility
            args = dict(args)
            args.setdefault('face_center', True)
            reference = args.pop('center_reference', None)
            target_altitude = args.pop('target_altitude_agl_m', None)
            camera = args.pop('camera', None)
            if camera == 'none':
                camera = None
            if reference:
                center = resolve_reference(state, reference, getattr(client, '_takeoff_reference', None))
                if not center['available']:
                    raise ValueError(center['reason'])
                args.update(center_north_m=center['north_m'], center_east_m=center['east_m'])
                # Convert local target down to above-home, without assuming
                # local origin and autopilot home have identical altitude.
                target_altitude = state['relative_altitude_m'] + state['local_position_ned']['down_m'] - center['down_m']
            if 'center_north_m' not in args or 'center_east_m' not in args:
                raise ValueError('Orbit needs a measured center or home/launch reference')
            view = orbit_camera_feasibility(args['radius_m'], args['altitude_agl_m'], target_altitude, camera or 'forward_d435i')
            if camera:
                if not args.get('face_center'):
                    raise ValueError('Camera-target orbit requires face_center=true')
                if not view.get('feasible'):
                    result = {'status': 'FAILED', 'message': 'Requested camera view cannot be established: ' + view['reason']
                        + '. Ask about another camera or geometry; do not silently change requested altitude/radius.',
                        'evidence': {'camera_geometry': view, 'no_flight_executed': True,
                                     'center_reference': reference or 'explicit_coordinates',
                                     'center_north_m': args['center_north_m'],
                                     'center_east_m': args['center_east_m']}}
                    if view.get('feasible') is False:
                        camera_label = {'forward_d435i': 'forward', 'downward_brio': 'downward'}.get(camera, camera)
                        result.update(requires_user_choice=True, question=(
                            f"The target is outside the fixed {camera_label} camera's expected view at "
                            f"{args['altitude_agl_m']:g} m above home and {args['radius_m']:g} m radius. "
                            "Which may change: camera, altitude, or radius? Or cancel this maneuver."))
                    return result
            orbit_evidence = {'center_reference': reference or 'explicit_coordinates',
                'center_north_m': args['center_north_m'], 'center_east_m': args['center_east_m'],
                'camera_geometry': view, 'camera_visibility_verified': False,
                'heading_contract': ('horizontal bearing only' if args['face_center'] else 'keep entry heading'),
                'completion_scope': 'flight trajectory; camera visibility requires image evidence'}
        if name == "move_relative":
            pose = state.get("local_position_ned")
            attitude = state.get("attitude", {})
            if (
                not pose
                or "yaw_rad" not in attitude
                or "relative_altitude_m" not in state
            ):
                raise ValueError(
                    "Current measured position, heading and altitude are required for relative movement"
                )
            f, r, u = (args.pop(k, 0) for k in ("forward_m", "right_m", "up_m"))
            yaw = attitude["yaw_rad"]
            args.update(
                north_m=pose.get("north_m", 0) + f * math.cos(yaw) - r * math.sin(yaw),
                east_m=pose.get("east_m", 0) + f * math.sin(yaw) + r * math.cos(yaw),
                altitude_agl_m=state["relative_altitude_m"] + u,
            )
            name = "goto"
        if (
            name in ("arm", "disarm")
            and state.get("landed")
            and state.get("armed") == (name == "arm")
        ):
            return {
                "status": "SUCCEEDED",
                "message": "Aircraft is already "
                + ("armed." if name == "arm" else "disarmed."),
            }
        origin = state.get("local_position_ned", {}).get("origin_id")
        if expected_origin_id is not None and origin != expected_origin_id:
            raise ValueError("Target coordinate origin changed before flight dispatch; localize again")
        if name in ("goto", "orbit", "execute_route"):
            if not origin:
                raise ValueError("The measured local coordinate origin is unavailable")
            args["origin_id"] = origin
        context = client.context(f"agent-{uuid.uuid4().hex}")
        method, timeout = ACTION_METHODS.get(name), 120
        if name == "disarm":
            request, method = pb.DisarmRequest(context=context), "Disarm"
        elif name == "cancel":
            active = state.get("active_action_id")
            if not active:
                return (
                    self._flight("hold", {})
                    if not state.get("landed")
                    else {
                        "status": "SUCCEEDED",
                        "message": "No active action; aircraft landed.",
                    }
                )
            request, method = (
                pb.CancelActionRequest(context=context, action_id=active),
                "CancelAction",
            )
        elif name == "hold":
            duration = args.pop("duration_s", 1.0)
            request = pb.HoldRequest(
                context=context, duration_ms=max(1, round(duration * 1000))
            )
            timeout = duration + 20
        elif name == "return_home":
            request = pb.ReturnHomeRequest(
                context=context,
                completion_behavior=(
                    pb.RETURN_COMPLETION_BEHAVIOR_LAND_AT_HOME
                    if args.get("land_at_home", True)
                    else pb.RETURN_COMPLETION_BEHAVIOR_HOLD_AT_HOME
                ),
            )
        elif name == "execute_route":
            from python.dcm.scan_workflow import route_tolerances
            points = []
            for point, tolerance in zip(args['points'], route_tolerances(args['points'])):
                # Arrival regions must stay distinct even for small patterns.
                # The backend must actually reach these tighter tolerances;
                # inability to resolve a small maneuver cannot count as success.
                tolerance = min(tolerance, point.get('acceptance_radius_m', tolerance))
                heading = pb.HeadingPolicy(mode=pb.HEADING_MODE_FACE_PATH if point.get("fly_through") else pb.HEADING_MODE_KEEP_CURRENT)
                if "heading_deg" in point:
                    heading = pb.HeadingPolicy(
                        mode=pb.HEADING_MODE_ABSOLUTE, heading_deg=point["heading_deg"]
                    )
                points.append(
                    pb.RoutePoint(
                        position=local_position(pb, {**point, "origin_id": origin}),
                        acceptance_radius_m=tolerance,
                        heading=heading,
                        hold_time_ms=round(point.get("hold_time_s", 0) * 1000),
                        fly_through=point.get("fly_through", False),
                        limits=pb.ActionLimits(
                            **{k: point[k] for k in SPEED if k in point}
                        ),
                    )
                )
            request = pb.ExecuteRouteRequest(
                context=context,
                route_id=uuid.uuid4().hex,
                points=points,
                delegate_to_autopilot=True,
                completion_behavior={
                    "hold": pb.ROUTE_COMPLETION_BEHAVIOR_HOLD,
                    "return_home": pb.ROUTE_COMPLETION_BEHAVIOR_RETURN_HOME,
                    "land": pb.ROUTE_COMPLETION_BEHAVIOR_LAND,
                }[args.get("completion", "hold")],
            )
            method, timeout = "ExecuteRoute", 600
        else:
            if name == "orbit" and "viewpoint_dwell_s" in args:
                args["viewpoint_dwell_ms"] = round(args.pop("viewpoint_dwell_s") * 1000)
            if name == "goto":
                pose = state.get("local_position_ned", {})
                distance = math.sqrt(
                    (args["north_m"] - pose.get("north_m", 0)) ** 2
                    + (args["east_m"] - pose.get("east_m", 0)) ** 2
                    + (args["altitude_agl_m"] - state.get("relative_altitude_m", 0))
                    ** 2
                )
                args["acceptance_radius_m"] = (acceptance_radius_m if acceptance_radius_m is not None
                                               else navigation_acceptance_radius(distance, {}))
            request, timeout = build_request(pb, name, args, context, runtime_target=runtime_target)
        if self.interrupted():
            return {
                "status": "CANCELLED",
                "message": "Operator interrupted before dispatch.",
            }
        since = self.interrupt.sequence() if self.interrupt else 0
        from python.dcm.motion_feedback import movement_feedback
        motion = movement_feedback(state, args) if name == 'goto' else None
        receipt = getattr(client.action_api, method)(request, timeout=5)
        if receipt.disposition == pb.ACTION_STATE_REJECTED:
            return {
                "status": "REJECTED",
                "message": receipt.message,
                "reason_code": pb.ReasonCode.Name(receipt.reason_code),
                "action_id": receipt.action_id,
                **({'motion': motion} if motion is not None else {}),
            }
        if self.interrupt:
            self.interrupt.track(receipt, since)
        try:
            terminal = client.wait_action(receipt, timeout, return_failure=True)
            # Accessing an absent proto message manufactures an all-default
            # DroneState (including landed=false). Absence is not telemetry.
            terminal_state = (message_dict(terminal.terminal_state)
                              if terminal.HasField('terminal_state') else None)
            status = terminal.state
            if isinstance(status, int):
                status = pb.ActionState.Name(status)
            status = status.removeprefix("ACTION_STATE_")
            if name == "cancel" and status == "CANCELLED":
                status = "SUCCEEDED"
            heading_evidence = None
            if absolute_heading is not None:
                measured = (terminal_state or {}).get('attitude', {}).get('yaw_rad')
                error = (abs(math.remainder(math.degrees(measured)-absolute_heading,360))
                         if type(measured) in (int,float) and math.isfinite(measured) else None)
                heading_evidence = {'requested_heading_deg': absolute_heading,
                    'measured_heading_deg': math.degrees(measured) % 360 if error is not None else None,
                    'absolute_heading_error_deg': error, 'heading_tolerance_deg': HEADING_TOLERANCE_DEG,
                    'controller': 'native_yaw_preserving_position_target'}
                if status == 'SUCCEEDED' and (error is None or error > HEADING_TOLERANCE_DEG):
                    status = 'FAILED'
            return {
                "status": status,
                "message": ('Requested absolute heading was not verified in terminal telemetry.'
                            if absolute_heading is not None and terminal.state == pb.ACTION_STATE_SUCCEEDED and status == 'FAILED'
                            else terminal.message),
                "reason_code": pb.ReasonCode.Name(terminal.reason_code),
                "action_id": receipt.action_id,
                **({'state': terminal_state} if terminal_state is not None else {}),
                **({'motion': movement_feedback(state, args, terminal_state)}
                   if motion is not None else {}),
                **({'evidence': orbit_evidence} if orbit_evidence is not None else {}),
                **({'evidence': heading_evidence} if heading_evidence is not None else {}),
            }
        finally:
            if self.interrupt:
                self.interrupt.untrack(receipt)

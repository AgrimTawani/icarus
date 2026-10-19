"""Reusable metric flight/observation workflows, independent of mission wording."""
import math
import uuid

from python.dcm.scan_geometry import generate_scan
from python.dcm.observation_receipt import stop_filter_receipt


def route_tolerances(points):
    """Disjoint arrival regions based on requested legs, not pose noise."""
    vertices = [(p['north_m'], p['east_m'], p['altitude_agl_m']) for p in points]
    tolerances = []
    for index, vertex in enumerate(vertices):
        legs = [math.dist(vertex, vertices[other]) for other in (index - 1, index + 1)
                if 0 <= other < len(vertices)]
        tolerances.append(min([0.25] + [distance / 4 for distance in legs if distance > 0]))
    return tolerances


def prepare_pattern(state, arguments):
    pose = state.get('local_position_ned', {})
    if not pose.get('origin_id') or 'yaw_rad' not in state.get('attitude', {}):
        raise ValueError('Pattern flight requires measured position, heading and local origin')
    if 'relative_altitude_m' not in state:
        raise ValueError('Pattern flight requires measured altitude above home')
    args = dict(arguments)
    heading = args.get('heading_deg', math.degrees(state['attitude']['yaw_rad']))
    anchor = {'north_m': pose.get('north_m', 0),
              'east_m': pose.get('east_m', 0),
              'heading_deg': heading, 'origin_id': pose['origin_id']}
    points = generate_scan(shape=args['shape'], coverage=args.get('coverage', 'perimeter'),
        start_north_m=anchor['north_m'], start_east_m=anchor['east_m'], heading_deg=heading,
        altitude_agl_m=args.get('altitude_agl_m', state['relative_altitude_m']),
        width_m=args.get('width_m'), length_m=args.get('length_m'), radius_m=args.get('radius_m'),
        lane_spacing_m=args.get('lane_spacing_m'), clockwise=args.get('clockwise', False))
    repetitions = args.get('repetitions', 1)
    repeated = points[1:] if args.get('coverage', 'perimeter') == 'perimeter' else points
    points = points + [dict(p) for _ in range(repetitions-1) for p in repeated]
    if len(points)>256:
        raise ValueError('Pattern exceeds 256 waypoints; request a smaller segment')
    for point in points:
        point['maximum_ground_speed_mps'] = args.get('maximum_ground_speed_mps', 1)
        point['hold_time_s'] = args.get('hold_time_s', 0)
        if args.get('camera_heading_deg') is not None:
            point['heading_deg'] = args['camera_heading_deg']
    # Keep full-path precision when observation stations split the route.
    for point, tolerance in zip(points, route_tolerances(points)):
        point['acceptance_radius_m'] = tolerance
    return anchor, points


class ScanWorkflow:
    def __init__(self, executor):
        self.executor, self.client = executor, executor.client
        if not hasattr(self.client, '_scan_workflows'):
            self.client._scan_workflows = {}

    def _route(self, points, origin, completion='hold'):
        return self.executor._flight('execute_route', {'points': points, 'completion': completion},
                                     expected_origin_id=origin)

    def fly_pattern(self, **args):
        from python.dcm.agent_tools import message_dict
        anchor, points = prepare_pattern(message_dict(self.client.state()), args)
        result = self._route(points, anchor['origin_id'], args.get('completion', 'hold'))
        return {**result, 'evidence': {'backend': 'ardupilot_auto', 'anchor': anchor,
            'waypoint_count': len(points), 'shape': args['shape'],
            'coverage': args.get('coverage', 'perimeter'), 'sensor_coverage_verified': False,
            'path_completed': result['status']=='SUCCEEDED'}}

    def scan_area(self, **args):
        from python.dcm.agent_tools import message_dict
        from python.dcm.agent_tools import observation_call
        observation_call(args, default_observation='scene', default_camera='downward_brio')
        stop = args.get('stop_on_detection')
        if stop and (args.get('observation', 'scene')!='detect' or stop not in args.get('classes', [])):
            raise ValueError('stop_on_detection must be one of the requested detection classes')
        anchor, points = prepare_pattern(message_dict(self.client.state()), args)
        # Checkpoints divide one geometric path; never regenerate from a moved
        # vehicle pose. The LLM is not invoked at each checkpoint.
        closed=args.get('coverage', 'perimeter')=='perimeter'
        count=min(args.get('observation_count', 4), len(points)-int(closed))
        last=len(points)-1
        indices=([round(i*last/count) for i in range(count)] if closed else
                 sorted({round(i*last/(count-1)) for i in range(count)}) if count>1 else [last])
        ident='scan_'+uuid.uuid4().hex[:16]
        self.client._scan_workflows[ident]={'anchor':anchor,'points':points,'args':args,
            'indices':indices,'next':0,'path_index':0,'observations':[],'completed':False}
        return self.resume_scan(scan_id=ident)

    def resume_scan(self, scan_id):
        saved=self.client._scan_workflows.get(scan_id)
        if saved is None:
            raise ValueError('Unknown scan_id; start a new scan')
        if saved['completed']:
            return self._result(scan_id, saved, 'completed')
        args=saved['args']
        from python.dcm.agent_tools import observation_call
        name, params = observation_call(args, default_observation='scene', default_camera='downward_brio')
        while saved['next']<len(saved['indices']):
            if self.executor.interrupted():
                return self._result(scan_id,saved,'interrupted','CANCELLED')
            end=saved['indices'][saved['next']]
            result=self._route(saved['points'][saved['path_index']:end+1],saved['anchor']['origin_id'])
            if result['status']!='SUCCEEDED':
                return {**result,'evidence':self._result(scan_id,saved,'flight_failed')['evidence']}
            # Observation failure retries the same station, not a later one.
            observed=self.executor.execute(name, params)
            if observed['status']!='SUCCEEDED':
                return {**observed,'evidence':self._result(scan_id,saved,'observation_failed')['evidence']}
            evidence=observed.get('evidence',{})
            if (evidence.get('validity') in ('unknown', 'error', 'invalid')
                    or evidence.get('fresh') is False
                    or (args.get('observation') == 'detect' and any(
                        c not in evidence.get('counts', {}) for c in args['classes']))):
                return self._result(scan_id, saved, 'observation_unavailable', 'FAILED')
            receipt = stop_filter_receipt(args, evidence, saved['next']+1)
            if args.get('stop_on_detection') and receipt is None:
                return self._result(scan_id, saved, 'observation_unavailable', 'FAILED')
            saved['observations'].append({'station':saved['next']+1,
                'position':saved['points'][end], 'evidence':evidence,
                **({'workflow_stop_filter': receipt} if receipt else {})})
            saved['next']+=1
            saved['path_index']=end+1
            # Native flight has already established a hold. A measured event
            # returns to the agent for arbitrary/nested semantic branching.
            if receipt and receipt['matched']:
                return self._result(scan_id,saved,'condition_met')
            if not args.get('stop_on_detection') and args.get('return_on', 'observation') != 'completion':
                return self._result(scan_id,saved,'observation_ready')
        if saved['path_index']<len(saved['points']):
            result=self._route(saved['points'][saved['path_index']:],saved['anchor']['origin_id'])
            if result['status']!='SUCCEEDED':
                return {**result,'evidence':self._result(scan_id,saved,'flight_failed')['evidence']}
            saved['path_index']=len(saved['points'])
        saved['completed']=True
        return self._result(scan_id,saved,'completed')

    @staticmethod
    def _result(ident,saved,event,status='SUCCEEDED'):
        return {'status':status,'workflow_complete':saved['completed'],
            'message':'Scan observation checkpoints completed.' if saved['completed'] else 'Scan paused: '+event,
            'evidence':{'scan_id':ident,'event':event,'backend':'ardupilot_auto',
                'anchor':saved['anchor'],'completed':saved['completed'],
                **({'workflow_stop_filter': dict(saved['observations'][-1]['workflow_stop_filter'])}
                   if status == 'SUCCEEDED' and saved['observations']
                   and 'workflow_stop_filter' in saved['observations'][-1] else {}),
                'completed_stations':saved['next'],'total_stations':len(saved['indices']),
                'observations':list(saved['observations']),
                'coverage_claim':'observations at recorded stations along the requested path; image coverage not verified',
                'sensor_coverage_verified':False}}

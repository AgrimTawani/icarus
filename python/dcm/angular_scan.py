"""Measured turn/observe cycles. The agent chooses intent, never frame scheduling."""
import copy
import math
import uuid

from python.dcm.observation_receipt import stop_filter_receipt
from python.dcm.scan_policy import failure_receipt


class AngularScan:
    def __init__(self, executor):
        self.executor = executor
        self.client = executor.client
        if not hasattr(self.client, '_angular_scans'):
            self.client._angular_scans = {}

    def scan_heading(self, **args):
        from python.dcm.agent_tools import message_dict, observation_call
        from python.dcm.scan_policy import scan_arguments
        args = scan_arguments(args, default_observation='detect', coverage_return='sweep')
        observation_call(args, default_observation='detect', default_camera='forward_d435i')
        if args.get('stop_on_detection') and (args.get('observation', 'detect') != 'detect'
                or args['stop_on_detection'] not in args.get('classes', [])):
            raise ValueError('Detection condition must name a requested class')
        if args.get('repeat_until_match') and args.get('observation', 'detect') == 'detect' and not args.get('stop_on_detection'):
            raise ValueError('An open-ended detection search needs stop_on_detection')
        state = message_dict(self.client.state())
        origin = state.get('local_position_ned', {}).get('origin_id')
        if not origin or 'yaw_rad' not in state.get('attitude', {}):
            raise ValueError('Measured heading and origin required')
        ident = 'angular_' + uuid.uuid4().hex[:16]
        self.client._angular_scans[ident] = dict(args=copy.deepcopy(args), origin=origin,
            initial_heading_deg=math.degrees(state['attitude']['yaw_rad']) % 360,
            swept_deg=0.0, phase='turn', observations=[], completed=False,
            heading_after_turn=None)
        return self.resume_heading_scan(ident)

    def resume_heading_scan(self, scan_id):
        from python.dcm.agent_tools import message_dict, observation_call
        s = self.client._angular_scans.get(scan_id)
        if s is None:
            raise ValueError('Unknown angular scan_id')
        a = s['args']
        name, params = observation_call(a, default_observation='detect', default_camera='forward_d435i')
        # One sweep is a scheduling chunk, not completion of an open-ended search.
        budget = a.get('sweep_deg', 360)
        chunk_start = s['swept_deg']
        while not s['completed']:
            if self.executor.interrupted():
                return self.result(scan_id, s, 'interrupted', 'CANCELLED')
            state = message_dict(self.client.state())
            if state.get('local_position_ned', {}).get('origin_id') != s['origin']:
                raise ValueError('Angular scan coordinate origin changed')
            if s['phase'] == 'turn':
                remaining = budget - (s['swept_deg'] - chunk_start if a.get('repeat_until_match') else s['swept_deg'])
                if remaining <= 1e-6:
                    s['completed'] = not a.get('repeat_until_match', False)
                    return self.result(scan_id, s, 'completed' if s['completed'] else 'sweep_exhausted')
                angle = min(a['step_deg'], remaining) * (1 if a.get('clockwise', False) else -1)
                r = self.executor._flight('yaw', {'angle_deg': angle, 'rate_deg_s': a.get('rate_deg_s', 10)})
                if r['status'] != 'SUCCEEDED':
                    return failure_receipt(r, self.result(scan_id, s, 'turn_failed'))
                s['swept_deg'] += abs(angle)
                s['phase'] = 'observe'
                s['heading_after_turn'] = math.degrees(message_dict(self.client.state())['attitude']['yaw_rad']) % 360
            # A failed observation leaves this exact obligation pending. Resume
            # must not silently move on to the next station or changed viewpoint.
            state = message_dict(self.client.state())
            heading = math.degrees(state['attitude']['yaw_rad']) % 360
            delta = math.remainder(s['heading_after_turn'] - heading, 360)
            if abs(delta) > 5:
                r = self.executor._flight('yaw', {'angle_deg': delta, 'rate_deg_s': a.get('rate_deg_s', 10)})
                if r['status'] != 'SUCCEEDED':
                    return failure_receipt(r, self.result(scan_id, s, 'viewpoint_restore_failed'))
            r = self.executor.execute(name, params)
            if r['status'] != 'SUCCEEDED':
                return failure_receipt(r, self.result(scan_id, s, 'observation_failed'))
            evidence = r.get('evidence', {})
            if (evidence.get('validity') in ('unknown', 'error', 'invalid')
                    or evidence.get('fresh') is False
                    or (name == 'detect' and any(c not in evidence.get('counts', {}) for c in a['classes']))):
                return self.result(scan_id, s, 'observation_unavailable', 'FAILED')
            receipt = stop_filter_receipt(a, evidence, len(s['observations']) + 1)
            if a.get('stop_on_detection') and receipt is None:
                return self.result(scan_id, s, 'observation_unavailable', 'FAILED')
            s['observations'].append({'station': len(s['observations']) + 1,
                'heading_deg': s['heading_after_turn'], 'evidence': evidence,
                **({'workflow_stop_filter': receipt} if receipt else {})})
            s['phase'] = 'turn'
            # The final captured station completes a bounded sweep immediately;
            # resuming solely to discover completion would waste a model call.
            s['completed'] = (not a.get('repeat_until_match', False)
                              and s['swept_deg'] >= budget - 1e-6)
            if receipt and receipt['matched']:
                s['completed'] = (s['completed'] or bool(a.get('repeat_until_match'))
                                  or a.get('exit_policy') == 'finish_at_match')
                return self.result(scan_id, s, 'condition_met')
            # Semantic observations return each station for arbitrary nested decisions.
            if ((name == 'inspect_scene' and a.get('exit_policy') != 'complete_coverage')
                    or (not a.get('stop_on_detection') and a.get('return_on', 'observation') != 'sweep')):
                return self.result(scan_id, s, 'observation_ready')
        return self.result(scan_id, s, 'completed')

    @staticmethod
    def result(ident, s, event, status='SUCCEEDED'):
        from python.dcm.scan_policy import completion_reason
        return {'status': status, 'workflow_complete': s['completed'],
            'message': f"Angular observation scan: {event}; {len(s['observations'])} observed stations; completed turn commands total {s['swept_deg']:g} degrees.",
            'evidence': {'scan_id': ident, 'event': event, 'completed': s['completed'],
                'pending_phase': s['phase'], 'swept_deg': s['swept_deg'],
                'exit_policy': s['args'].get('exit_policy', 'legacy'),
                'termination_reason': completion_reason(s['completed'], s['observations']),
                'coverage_complete': (s['phase'] != 'observe'
                                      and not s['args'].get('repeat_until_match', False)
                                      and s['swept_deg'] >= s['args'].get('sweep_deg', 360)),
                **({'workflow_stop_filter': copy.deepcopy(s['observations'][-1]['workflow_stop_filter'])}
                   if status == 'SUCCEEDED' and s['observations']
                   and 'workflow_stop_filter' in s['observations'][-1] else {}),
                'observations': copy.deepcopy(s['observations']),
                'coverage_claim': 'sampled camera views at recorded headings; not all-around absence'}}

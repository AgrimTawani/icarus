"""Retain one explicit visual question across viewpoint changes.

This is observation continuity, not mission parsing or branch selection. Only
the agent's own condition inspection registers a question; detector counts
cannot answer it. Refreshes are synchronous, once per completed movement.
"""
import copy

from python.dcm.active_detection_feedback import _MOTION


class ActiveVisualFeedback:
    def __init__(self):
        self.interest = None
        self.result = None
        self.pending = False
        self.revision = 0

    def observe(self, name, arguments, result):
        if name == 'execute_sequence':
            for child in result.get('actions', []):
                self.observe(child['name'], child.get('arguments', {}), child['result'])
            return
        if name == 'localize_target':
            evidence = result.get('evidence', {})
            if (result.get('status') == 'SUCCEEDED' and evidence.get('found') is True
                    and evidence.get('target_id') and self.snapshot()
                    and self.snapshot()['found'] is not None):
                # Retire an ANSWERED question when target work takes over.
                # Localization alone cannot answer arbitrary visual questions
                # (colour, damage, etc.) that are still unknown.
                self.interest = None
                self.pending = False
            return
        if (name in {'approach_target', 'orbit', 'overfly_target'}
                and arguments.get('destination', {}).get('type') == 'visible_object'
                and result.get('status') == 'SUCCEEDED'):
            self.interest = None
            self.pending = False
            return
        if name == 'inspect_scene' and arguments.get('question_type') == 'condition':
            self.interest = {key: copy.deepcopy(arguments[key]) for key in
                             ('question', 'camera', 'question_type') if key in arguments}
            self.interest.setdefault('camera', 'forward_d435i')
            # Refresh the new view, not the original compass direction.
            self.interest['view'] = 'current'
            self.result = copy.deepcopy(result)
            self.pending = False
        elif self.interest and name in _MOTION - {'hold'}:
            self.revision += 1
            self.result = None
            self.pending = result.get('status') == 'SUCCEEDED' and name not in {'land', 'return_home'}

    def refresh(self, executor):
        if not self.pending or not self.interest:
            return None
        self.pending = False
        if executor.interrupted():
            return None
        arguments = copy.deepcopy(self.interest)
        try:
            result = executor.execute('inspect_scene', arguments)
        except Exception as error:
            result = {'status': 'OBSERVATION_UNAVAILABLE', 'message': str(error)}
        self.result = copy.deepcopy(result)
        return {'action': 'inspect_scene', 'source': 'visual_feedback',
                'arguments': arguments, 'result': result}

    def snapshot(self):
        if not self.interest:
            return None
        result = self.result or {}
        evidence = result.get('evidence', {})
        known = result.get('status') == 'SUCCEEDED' and evidence.get('validity') == 'valid' and type(evidence.get('found')) is bool
        return {'question': self.interest['question'], 'camera': self.interest['camera'],
                'found': evidence['found'] if known else None,
                'validity': 'valid' if known else 'unknown',
                'motion_revision': self.revision, 'refresh_pending': self.pending,
                'meaning': 'This visual question is independent of detector class counts. Unknown is not permission to approach.'}

    def unresolved_approach(self, name, arguments):
        destination = arguments.get('destination', {})
        snapshot = self.snapshot()
        return bool(name in {'approach_target', 'orbit', 'overfly_target'}
                    and destination.get('type') == 'visible_object'
                    and snapshot and snapshot['found'] is None)

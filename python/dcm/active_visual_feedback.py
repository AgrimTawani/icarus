"""Retain one explicit visual question across viewpoint changes.

This is observation continuity, not mission parsing or branch selection. Only
the agent's own condition inspection registers a question; detector counts
cannot answer it. Refreshes are synchronous, once per completed movement.
"""
import copy

from python.dcm.active_detection_feedback import _MOTION, _SCANS


class ActiveVisualFeedback:
    def __init__(self):
        self.interest = None
        self.result = None
        self.pending = False
        self.revision = 0
        self.recovery = None
        self.continuation = None

    def permits_motion(self, name, arguments):
        """Recovery can resume existing work, never authorize a new maneuver."""
        if not self.recovery or name in {'hold', 'land', 'return_home', 'cancel'}:
            return True
        from python.dcm.agent_tools import FLIGHT, TARGET_FLIGHT, WORKFLOW_FLIGHT, turn_arguments
        if name not in {*FLIGHT, *TARGET_FLIGHT, *WORKFLOW_FLIGHT, 'execute_sequence'}:
            return True
        continuation = self.recovery.get('continuation')
        args = turn_arguments(name, arguments)
        args.pop('mission_complete', None)
        return bool(continuation and name == continuation['name'] and args == continuation['arguments'])

    @staticmethod
    def _subject(name, arguments):
        destination = arguments.get('destination', {})
        if destination.get('type') == 'visible_object':
            query = destination.get('visual_query')
        elif name == 'localize_target':
            query = arguments.get('visual_query', arguments.get('description'))
        else:
            return None
        return query.strip().casefold() if isinstance(query, str) else None

    def recovery_required(self, name, arguments):
        return bool(self.recovery and self._subject(name, arguments) == self.recovery['subject'])

    def _accept(self, result):
        self.result = copy.deepcopy(result)
        evidence = result.get('evidence', {})
        if (self.recovery and self.revision > self.recovery['motion_revision']
                and result.get('status') == 'SUCCEEDED' and evidence.get('validity') == 'valid'
                and type(evidence.get('found')) is bool):
            self.recovery = None

    def observe(self, name, arguments, result):
        if name == 'execute_sequence':
            for child in result.get('actions', []):
                self.observe(child['name'], child.get('arguments', {}), child['result'])
            return
        evidence = result.get('evidence', {})
        localization = evidence if name == 'localize_target' else evidence.get('target_localization', {})
        current = self.snapshot()
        subject = self._subject(name, arguments)
        if subject and localization.get('found') is False and current and current['found'] is True:
            self.recovery = {'subject': subject, 'motion_revision': self.revision,
                'continuation': copy.deepcopy(self.continuation),
                'reason': localization.get('reason', 'target_not_localized'),
                'next_step': 'Resume the saved observation continuation, then check the fresh evidence. '
                             'Recovery does not authorize a different motion. If no continuation exists or the user '
                             'did not request repetition, clarify rather than invent movement.'}
            self.result = None
            self.pending = False
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
            if 'repeat_motion' in arguments and not self.recovery:
                from python.dcm.agent_tools import turn_arguments
                repeat = arguments['repeat_motion']
                self.continuation = ({'name': repeat['name'],
                    'arguments': turn_arguments(repeat['name'], repeat['arguments'])}
                    if repeat else None)
            self.interest = {key: copy.deepcopy(arguments[key]) for key in
                             ('question', 'camera', 'question_type') if key in arguments}
            self.interest.setdefault('camera', 'forward_d435i')
            # Refresh the new view, not the original compass direction.
            self.interest['view'] = 'current'
            self._accept(result)
            self.pending = False
        elif self.interest and name in (_MOTION | _SCANS) - {'hold'}:
            if evidence.get('no_flight_command_sent') is True:
                return
            if result.get('status') == 'SUCCEEDED':
                self.revision += 1
            self.result = None
            self.pending = result.get('status') == 'SUCCEEDED' and name not in {'land', 'return_home'}
        if (name in _MOTION and name != 'hold' and result.get('status') == 'SUCCEEDED'
                and not evidence.get('no_flight_command_sent')):
            from python.dcm.agent_tools import turn_arguments
            args = turn_arguments(name, arguments)
            args.pop('mission_complete', None)
            # Only relative primitive movements can be continued unchanged.
            # Absolute destinations/headings would revisit the same view.
            self.continuation = ({'name': name, 'arguments': args}
                                 if name in {'yaw', 'move_relative'} else None)

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
        self._accept(result)
        return {'action': 'inspect_scene', 'source': 'visual_feedback',
                'arguments': arguments, 'result': result}

    def snapshot(self):
        if not self.interest:
            return None
        result = self.result or {}
        evidence = result.get('evidence', {})
        known = (not self.recovery and result.get('status') == 'SUCCEEDED'
                 and evidence.get('validity') == 'valid' and type(evidence.get('found')) is bool)
        return {'question': self.interest['question'], 'camera': self.interest['camera'],
                'continuation': copy.deepcopy(self.continuation),
                'found': evidence['found'] if known else None,
                'validity': 'valid' if known else 'unknown',
                'motion_revision': self.revision, 'refresh_pending': self.pending,
                **({'recovery': copy.deepcopy(self.recovery)} if self.recovery else {}),
                'meaning': 'This visual question is independent of detector class counts. Unknown is not permission to approach.'}

    def unresolved_approach(self, name, arguments):
        destination = arguments.get('destination', {})
        snapshot = self.snapshot()
        return bool(name in {'approach_target', 'orbit', 'overfly_target'}
                    and destination.get('type') == 'visible_object'
                    and snapshot and snapshot['found'] is None)


def bind_recovery_tools(tools, condition):
    """Advertise the same continuation contract enforced by dispatch."""
    recovery = (condition or {}).get('recovery')
    if not recovery:
        return tools
    from python.dcm.agent_tools import FLIGHT, TARGET_FLIGHT, WORKFLOW_FLIGHT, turn_arguments
    motion = {*FLIGHT, *TARGET_FLIGHT, *WORKFLOW_FLIGHT, 'execute_sequence'}
    continuation = recovery.get('continuation') or {}
    result = []
    for tool in tools:
        name = tool['function']['name']
        if name in motion and name not in {'hold', 'land', 'return_home', 'cancel'}:
            if name != continuation.get('name'):
                continue
            tool = copy.deepcopy(tool)
            args = turn_arguments(name, continuation['arguments'], native=True)
            schema = tool['function']['parameters']
            schema['properties'] = {key: {'const': value} for key, value in args.items()}
            schema['required'] = list(args)
            schema['additionalProperties'] = False
        result.append(tool)
    return result

"""Refuse same-view observation after a contradicted positive visual answer.

When the VLM says a condition is true but measured localization finds nothing,
looking at the same view again cannot resolve the conflict. Dispatch refuses
further observation until a movement succeeds. Which movement is the agent's
choice from the user's request; this only guarantees new evidence is new.
"""

VIEW_CHANGES = frozenset({
    'takeoff', 'goto', 'move_relative', 'execute_route', 'yaw', 'face_heading', 'orbit',
    'return_home', 'land', 'approach_target', 'orbit_target', 'land_on_target', 'overfly_target',
    'fly_pattern', 'scan_heading', 'resume_heading_scan', 'scan_area', 'resume_scan'})


def _visible_object(arguments):
    destination = arguments.get('destination')
    return isinstance(destination, dict) and destination.get('type') == 'visible_object'


class ViewpointGate:
    def __init__(self):
        self.positive = False  # latest condition inspection answered true
        self.blocked = False

    def blocks(self, name, arguments):
        return self.blocked and (name in {'inspect_scene', 'localize_target'} or _visible_object(arguments))

    def dispatched(self, name, arguments, result):
        evidence = result.get('evidence')
        evidence = evidence if isinstance(evidence, dict) else {}
        if (name in VIEW_CHANGES and result.get('status') == 'SUCCEEDED'
                and not evidence.get('no_flight_command_sent')):
            self.positive = self.blocked = False
        elif name == 'inspect_scene' and evidence.get('question_type') == 'condition':
            self.positive = result.get('status') == 'SUCCEEDED' and evidence.get('found') is True
        elif name == 'localize_target' or _visible_object(arguments):
            localization = evidence if name == 'localize_target' else evidence.get('target_localization', {})
            if self.positive and localization.get('found') is False:
                self.positive, self.blocked = False, True

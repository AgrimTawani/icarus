"""One bounded, requested detection interest refreshed after settled movement.

This remembers sensing requests, not mission conditions. It never dispatches
movement, evaluates a branch, refreshes a VLM, or creates background processes.
"""
import copy
import time
from collections.abc import Mapping

_MOTION = {'takeoff', 'goto', 'move_relative', 'execute_route', 'yaw', 'face_heading',
           'orbit', 'return_home', 'land', 'approach_target', 'orbit_target',
           'land_on_target', 'fly_pattern', 'hold'}
_SCANS = {'scan_heading', 'resume_heading_scan', 'scan_area', 'resume_scan'}


class ActiveDetectionFeedback:
    def __init__(self, *, clock_ms=None, max_age_ms=30000):
        self.clock_ms = clock_ms or (lambda: int(time.time() * 1000))
        # Maximum age of a sampled image; never a continuous-tracking guarantee.
        self.max_age_ms = max_age_ms
        self.interest = None
        self.revision = 0
        self._pending = False
        self._current = None
        self.motion_action = None

    def _invalidate(self, reason, status="unavailable"):
        self._current = {'validity': 'unknown', 'fresh': False, 'reason': reason,
                         'motion_revision': self.revision, 'status': status}

    def _register(self, arguments):
        classes = arguments.get('classes')
        camera = arguments.get('camera', 'forward_d435i')
        if not isinstance(classes, list) or not classes:
            return False
        if not all(isinstance(x, str) and x for x in classes) or not isinstance(camera, str):
            return False
        self.interest = {'camera': camera, 'classes': list(dict.fromkeys(classes))}
        return True

    def request_interest(self, arguments):
        """Start sampled camera feedback after localization, without inventing a capture."""
        if self.interest or not self._register(arguments):
            return False
        self._invalidate('Camera feedback requested; no detection capture yet')
        self._pending = True
        return True

    def _accept(self, result, source):
        self._pending = False
        if not isinstance(result, Mapping):
            self._invalidate('Detection returned an invalid result')
            return
        evidence = result.get('evidence', {})
        if not isinstance(evidence, Mapping):
            evidence = {}
        if result.get('status') != 'SUCCEEDED':
            self._invalidate(result.get('message') or 'Detection refresh failed', result.get('status', 'unavailable'))
            return
        counts = evidence.get('counts')
        if (evidence.get('validity') in ('unknown', 'error', 'invalid')
                or evidence.get('fresh') is False or evidence.get('available') is False
                or evidence.get('camera') != self.interest['camera']
                or not isinstance(counts, Mapping)
                or any(type(counts.get(c)) is not int or counts[c] < 0 for c in self.interest['classes'])):
            self._invalidate('Requested current camera counts unavailable')
            return
        captured = evidence.get('captured_at_unix_ms')
        if type(captured) is not int:
            self._invalidate('Detection capture timestamp unavailable')
            return
        # Include only current-view evidence, never cumulative track totals.
        self._current = {key: copy.deepcopy(evidence[key]) for key in (
            'camera', 'captured_at_unix_ms', 'observed_at_unix_ms', 'capture_pose',
            'capture_pose_quality', 'confidence_threshold', 'model', 'source') if key in evidence}
        self._current.update(counts={c: counts[c] for c in self.interest['classes']},
            validity='valid', fresh=True, motion_revision=self.revision,
            source_tool=source, status='SUCCEEDED', count_scope='current_camera_frame',
            count_meaning='accepted model detections, not guaranteed object totals')

    def _after_motion(self,tool,result):
        self.motion_action=tool
        self.revision+=1
        if self.interest:
            self._invalidate('Viewpoint changed after motion')
            self._pending=result.get('status')=='SUCCEEDED' and not result.get('requires_user_choice')

    def observe(self, tool, arguments, result):
        """Consume each completed top-level result once, before next decision.

        Keep this object on AgentSession to survive clarification. Do not feed
        refresh()'s own result back here: refresh already updates its snapshot.
        """
        if not isinstance(arguments, Mapping) or not isinstance(result, Mapping):
            return
        evidence=result.get('evidence',{})
        moved=isinstance(evidence,Mapping) and evidence.get('motion_command_issued') is True
        if tool not in _SCANS and (tool in _MOTION or moved):
            self._after_motion(tool,result)
        if tool == 'detect':
            if self._register(arguments):
                self._accept(result, 'requested_detect')
            return
        if tool == 'inspect_scene':
            evidence = result.get('evidence', {})
            paired = evidence.get('camera_detection', {}) if isinstance(evidence, Mapping) else {}
            request = paired.get('arguments', {}) if isinstance(paired, Mapping) else {}
            detection = paired.get('result', {}) if isinstance(paired, Mapping) else {}
            if not self.interest:
                self._register(request)
            # An inspection must not replace an existing requested camera or
            # class interest. Its detector can satisfy that interest only if
            # the camera matches; _accept also validates every requested count.
            if self.interest and request.get('camera') == self.interest['camera']:
                self._accept(detection, 'inspection_detector')
            return
        if tool == 'execute_sequence':
            # Observe only actions actually executed; refresh once after batch.
            for child in result.get('actions', []):
                if isinstance(child, Mapping):
                    self.observe(child.get('name'), child.get('arguments', {}), child.get('result', {}))
            if result.get('status') != 'SUCCEEDED':
                self._pending = False
            return
        if tool in _SCANS:
            self.motion_action = tool
            self.revision += 1
            self._pending = bool(self.interest and result.get('status') == 'SUCCEEDED'
                                 and not result.get('requires_user_choice'))
            if self.interest:
                self._invalidate('Scan changed viewpoint; use its recorded observations')
            evidence = result.get('evidence', {})
            observations = evidence.get('observations', []) if isinstance(evidence, Mapping) else []
            if (result.get('status') != 'SUCCEEDED' or not isinstance(observations, list)
                    or not observations or not isinstance(observations[-1], Mapping)):
                return
            # Area completion can fly a closing leg after the final image.
            # Angular completion ends at its last captured heading, so that
            # image is reusable without another capture.
            reusable = {'observation_ready', 'condition_met'}
            if tool in ('scan_heading', 'resume_heading_scan'):
                reusable.update({'completed', 'sweep_exhausted'})
            if evidence.get('event') not in reusable:
                return
            last = observations[-1].get('evidence', {})
            if isinstance(last, Mapping) and isinstance(last.get('counts'), Mapping):
                request = {'camera': last.get('camera', arguments.get('camera', 'forward_d435i')),
                           'classes': list(last['counts'])}
                if self._register(request):
                    self._accept({'status': 'SUCCEEDED', 'evidence': last}, 'scan_observation')
            return

    @property
    def needs_refresh(self):
        return bool(self._pending and self.interest)

    def refresh(self, executor):
        """At most one synchronous capture per pending motion revision.

        Returns the actual detect result for optional logging/display, or None.
        Call before context assembly, never from a background polling loop.
        """
        if not self._pending or not self.interest:
            return None
        self._pending = False
        if executor.interrupted():
            self._invalidate('Operator interrupted before detection refresh')
            return None
        try:
            result = executor.execute('detect', copy.deepcopy(self.interest))
        except Exception as error:
            result = {'status': 'FAILED', 'message': str(error)}
        if executor.interrupted():
            self._invalidate('Operator interrupted during detection refresh')
        else:
            self._accept(result, 'automatic_post_motion_detect')
        return result

    def snapshot(self):
        if not self.interest:
            return None
        current = copy.deepcopy(self._current or {'validity': 'unknown', 'fresh': False})
        captured = current.get('captured_at_unix_ms')
        if captured is not None:
            current['age_ms'] = self.clock_ms() - captured
            if not 0 <= current['age_ms'] <= self.max_age_ms:
                current.pop('counts', None)
                current.update(validity='unknown', fresh=False, reason='Detection capture is stale')
        return {'interest': copy.deepcopy(self.interest), 'observation': current,
                'after_motion_revision': self.revision, 'motion_action': self.motion_action,
                'feedback_scope': 'sampled requested-camera detections, not continuous tracking',
                'refresh_pending': self._pending}

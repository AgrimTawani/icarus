"""Measured object-relative flight skills; no mission interpretation.

The model selects a target and outcome. These bounded controllers consume
sensor geometry and dispatch ordinary Drone API actions with runtime provenance.
"""
import math
import threading
import time
import json
from pathlib import Path


def operating_defaults():
    return json.loads((Path(__file__).resolve().parents[2] / 'config/dcm/default_policy.json').read_text())

# ActionExecutor::ExecuteContinuousOrbit permits this radial deviation before its
# tracking-loss timeout. Keep trajectory reserve distinct from clearance floor.
# This is a controller contract, not a target shape or scenario assumption.
ORBIT_RADIAL_TRACKING_TOLERANCE_M = 1.0


def orbit_envelope(geometry, standoff_m, horizontal_uncertainty_m=None):
    center = geometry.get('orbit_center') or geometry['center']
    measured_radius = float(geometry.get('orbit_radius_m', geometry['radius_m']))
    if geometry.get('observed_hull'):
        measured_radius = max(measured_radius, max(math.dist(point_xy(center), point_xy(p))
                                                  for p in geometry['observed_hull']))
    uncertainty_known = (not isinstance(horizontal_uncertainty_m, bool)
                         and isinstance(horizontal_uncertainty_m, (int, float))
                         and math.isfinite(horizontal_uncertainty_m)
                         # Proto default zero also represents an unpopulated
                         # estimate; it is not evidence of perfect positioning.
                         and horizontal_uncertainty_m > 0)
    uncertainty = horizontal_uncertainty_m if uncertainty_known else 0.0
    if not math.isfinite(measured_radius) or measured_radius < 0:
        raise ValueError('Measured target enclosure radius is invalid')
    reserve = ORBIT_RADIAL_TRACKING_TOLERANCE_M + uncertainty
    return center, measured_radius + standoff_m + reserve, {
        'measured_enclosure_radius_m': measured_radius,
        'radial_tracking_reserve_m': ORBIT_RADIAL_TRACKING_TOLERANCE_M,
        'position_uncertainty_reserve_m': uncertainty,
        'position_uncertainty_reported': uncertainty_known,
        'total_radius_reserve_m': reserve,
        'requested_minimum_surface_clearance_m': standoff_m,
        'nominal_measured_hull_clearance_lower_bound_m': standoff_m + reserve,
        'reserve_basis': 'native orbit radial tracking tolerance plus reported horizontal position uncertainty',
        'unseen_shape_bounded': False,
    }


def point_xy(point):
    return float(point['north_m']), float(point['east_m'])


def pose(state):
    from python.dcm.agent_tools import message_dict
    state = message_dict(state)
    p = state.get('local_position_ned', {})
    if not p.get('origin_id') or 'relative_altitude_m' not in state:
        raise ValueError('Measured local position and above-home altitude are unavailable')
    return state, float(p.get('north_m', 0)), float(p.get('east_m', 0))


class OrbitMeasurements:
    """Accumulate actual aircraft movement, independent of requested setpoints."""
    def __init__(self, center, clockwise=True):
        self.center = point_xy(center)
        self.direction = 1 if clockwise else -1
        self.previous = None
        self.sweep = 0.0
        self.samples = 0
        self.max_heading_error_deg = 0.0
        self.radii = []

    def observe(self, state):
        state, n, e = pose(state)
        dn, de = n - self.center[0], e - self.center[1]
        angle = math.atan2(de, dn)
        if self.previous is not None:
            self.sweep += math.remainder(angle - self.previous, 2 * math.pi) * self.direction
        self.previous = angle
        self.samples += 1
        self.radii.append(math.hypot(dn, de))
        yaw = state.get('attitude', {}).get('yaw_rad')
        if yaw is not None:
            error = abs(math.degrees(math.remainder(math.atan2(-de, -dn) - yaw, 2 * math.pi)))
            self.max_heading_error_deg = max(self.max_heading_error_deg, error)

    def result(self):
        return {'samples': self.samples, 'angular_coverage_deg': round(math.degrees(self.sweep), 2),
                'maximum_heading_error_deg': round(self.max_heading_error_deg, 2),
                'minimum_center_radius_m': round(min(self.radii), 3) if self.radii else None,
                'maximum_center_radius_m': round(max(self.radii), 3) if self.radii else None}


def _aim_height(target, state):
    """Height the orbit camera should frame: the measured top, else our own height.

    An unmeasured top (e.g. a roof above the LiDAR's upper beam) still means the
    wall spans the camera's altitude, so the camera aims level at it.
    """
    height = target.get('footprint', {}).get('height_m')
    if isinstance(height, (int, float)) and not isinstance(height, bool) and math.isfinite(height):
        return height
    return state['relative_altitude_m']


REFRESH_ATTEMPTS = 3
REFRESH_WAIT_S = 0.3  # LiDAR publishes at 10 Hz


class TargetMeasurementError(ValueError):
    def __init__(self, measurement):
        super().__init__('Target measurement unavailable: ' + measurement.get('reason', 'target lost'))
        self.measurement = measurement


class TargetMotion:
    def __init__(self, client, flight, interrupted=lambda: False):
        self.client, self.flight, self.interrupted = client, flight, interrupted

    def _target(self, target_id, *, static_reference=False):
        anchor = getattr(self.client, 'target_anchor', None)
        if static_reference and callable(anchor):
            target = anchor(target_id)
            if not target.get('reference_available'):
                raise TargetMeasurementError(target)
            return target
        # One scan can miss a sparse surface (AirSim's 8-channel LiDAR splits a
        # house face differently each sweep); a few fresh scans settle it.
        for attempt in range(REFRESH_ATTEMPTS):
            target = self.client.refresh_target(target_id)
            if target.get('found') or target.get('reason') != 'target_not_observed':
                break
            if attempt + 1 < REFRESH_ATTEMPTS:
                time.sleep(REFRESH_WAIT_S)
        if not target.get('found'):
            raise TargetMeasurementError(target)
        return target

    def _stop(self, message, **evidence):
        # Operator recovery owns control after interruption; do not overwrite it.
        if not self.interrupted():
            self.flight('hold', {'duration_s': 0.5})
        return {'status': 'CANCELLED' if self.interrupted() else 'FAILED',
                'message': message, 'evidence': evidence}

    def execute(self, name, args):
        try:
            result = getattr(self, name)(**args)
        except TargetMeasurementError as error:
            result = self._stop(str(error), target_measurement=error.measurement)
        except (ValueError, KeyError, RuntimeError) as error:
            result = self._stop(str(error))
        if name == 'approach_target':
            result.setdefault('evidence', {}).update(
                navigation_method='native_goto_to_measured_horizontal_standoff',
                static_target_association=True,
                target_geometry_update_points='before each routed leg and after arrival',
                continuous_target_identity_tracking=False)
        return result

    def approach_target(self, target_id, standoff_m, maximum_ground_speed_mps=1.0):
        history = []
        for _ in range(24):
            if self.interrupted():
                return self._stop('Operator interrupted target approach.', measurements=history)
            target = self._target(target_id)
            distance = float(target['surface_range_m'])
            # A clean re-measurement should not jump far closer than the last
            # commanded step could explain. In a cluttered scene, LiDAR
            # association can drift onto a different nearby surface; treat an
            # implausible overshoot as a measurement fault, not progress.
            if history and distance < standoff_m * 0.5 and distance < history[-1] - 1.5:
                return self._stop(
                    'Re-measured target surface jumped implausibly closer; '
                    'likely re-associated with a different nearby surface. Stopping short of it.',
                    target_id=target_id, measurements=history + [distance])
            history.append(distance)
            error = distance - standoff_m
            if abs(error) <= 0.25:
                return {'status': 'SUCCEEDED', 'message': 'Measured target surface standoff reached.',
                        'evidence': {'target_id': target_id, 'requested_standoff_m': standoff_m,
                                     'final_surface_range_m': distance, 'tolerance_m': 0.25,
                                     'measurements_m': history}}
            state, n, e = pose(self.client.state())
            sn, se = point_xy(target['surface_point'])
            length = math.hypot(sn-n, se-e)
            if state['local_position_ned']['origin_id'] != target['origin_id']:
                raise ValueError('Target coordinate origin changed before approach dispatch')
            if not all(math.isfinite(value) for value in
                       (distance, sn, se, n, e, state['relative_altitude_m'], standoff_m)) or distance < 0:
                raise ValueError('Target approach requires finite measured geometry')
            if length < 0.05:
                raise ValueError('Target direction is not measurable at this position')
            # Recede only when already inside the requested standoff; never
            # convert visual prose into a guessed signed displacement.
            # Give native obstacle routing the valid final standoff endpoint.
            # Artificial short ray endpoints can fall inside another obstacle,
            # preventing the planner from routing around it. Re-measure after
            # arrival; this is static-object association, not moving-target tracking.
            travel = error
            result = self.flight('goto', {'north_m': n + travel*(sn-n)/length,
                'east_m': e + travel*(se-e)/length,
                'altitude_agl_m': state['relative_altitude_m'],
                'heading_deg': math.degrees(math.atan2(se-e, sn-n)) % 360,
                'maximum_ground_speed_mps': min(maximum_ground_speed_mps, max(0.3, abs(error)/2))},
                runtime_target=True, acceptance_radius_m=0.12, expected_origin_id=target["origin_id"])
            if result['status'] != 'SUCCEEDED':
                return {**result, 'evidence': {'target_id': target_id, 'measurements_m': history}}
            if len(history) >= 4 and max(history[-3:]) - min(history[-3:]) < 0.08:
                return self._stop('No measurable progress toward target distance.', measurements=history)
        return self._stop('Target approach did not converge within its bounded execution.', measurements=history)

    def _acquire_target_geometry(self, target_id, target, standoff_m, maximum_ground_speed_mps, require_height=False):
        """Measure at most two new face-edge views, preserving target identity.

        Candidates are measured viewing positions, not inferred unseen hulls.
        Ordinary goto owns segment clearance and measured arrival; localization
        must associate the new image with the same registry identity.
        """
        history, visited = [], []
        def ready(measurement):
            footprint = measurement.get('footprint', {})
            return (measurement['geometry'].get('orbit_usable') and
                    (not require_height or footprint.get('height_m') is not None
                     and not footprint.get('height_clipped')))
        def fail(message):
            return target, history, self._stop(message, target_id=target_id,
                                              geometry_acquisition=history)
        try:
            for _ in range(2):
                if self.interrupted():
                    return fail('Operator interrupted orbit geometry measurement.')
                if ready(target):
                    return target, history, None
                state, north, east = pose(self.client.state())
                origin = target['origin_id']
                if state['local_position_ned']['origin_id'] != origin:
                    return fail('Target origin changed before orbit geometry measurement.')
                description = target.get('description')
                if not isinstance(description, str) or not description.strip():
                    return fail('Target description is unavailable for another measured view.')
                raw_candidates = target['geometry'].get('measurement_view_candidates', [])
                if any(isinstance(c, dict) and isinstance(c.get('observation_standoff_m'), (int, float))
                       and c['observation_standoff_m'] < standoff_m for c in raw_candidates):
                    # Recompute the same measured face-edge construction at the
                    # requested distance; never shrink a user's standoff to the
                    # registry's default observation distance.
                    from python.perception.target_registry import _measurement_views
                    hull = target['geometry'].get('observed_hull', [])
                    if any(not isinstance(p, dict) or p.get('origin_id') != origin for p in hull):
                        return fail('Observed hull origin is unavailable for the requested measurement standoff.')
                    raw_candidates = _measurement_views([point_xy(p) for p in hull],
                        state['local_position_ned'], origin, standoff_m)
                candidates = []
                for candidate in raw_candidates:
                    if not isinstance(candidate, dict) or candidate.get('origin_id') != origin:
                        continue
                    keys = ('north_m', 'east_m', 'face_observed_midpoint_heading_deg', 'observation_standoff_m')
                    if any(isinstance(candidate.get(k), bool) or
                           not isinstance(candidate.get(k), (int, float)) or
                           not math.isfinite(candidate[k]) for k in keys):
                        continue
                    if candidate['observation_standoff_m'] < standoff_m:
                        continue
                    xy = point_xy(candidate)
                    if math.dist(xy, (north, east)) < 0.25 or any(math.dist(xy, p) < 0.5 for p in visited):
                        continue
                    candidates.append(candidate)
                if not candidates:
                    return fail('Only part of the target shape is measured and no distinct measured side-view candidate is available. No orbit was started.')
                view = min(candidates, key=lambda p: math.dist(point_xy(p), (north, east)))
                visited.append(point_xy(view))
                step = {'phase': 'move_to_measurement_view',
                        'north_m': view['north_m'], 'east_m': view['east_m'],
                        'heading_deg': view['face_observed_midpoint_heading_deg'],
                        'origin_id': origin, 'observation_standoff_m': view['observation_standoff_m']}
                history.append(step)
                if self.interrupted():
                    return fail('Operator interrupted before orbit measurement movement.')
                result = self._facing_transit(target, view, state['relative_altitude_m'],
                                              maximum_ground_speed_mps, origin)
                step['status'] = result['status']
                if self.interrupted():
                    return fail('Operator interrupted orbit geometry measurement.')
                if result['status'] != 'SUCCEEDED':
                    return target, history, {**result, 'evidence': {**result.get('evidence', {}),
                        'target_id': target_id, 'geometry_acquisition': history}}
                step = {'phase': 'localize_same_target', 'requested_target_id': target_id}
                history.append(step)
                if self.interrupted():
                    return fail('Operator interrupted before target camera observation.')
                observed = self.client.localize_target(description, new_target=False)
                if not isinstance(observed, dict):
                    raise ValueError('Camera localization returned no structured measurement')
                step.update(found=observed.get('found', False), target_id=observed.get('target_id'),
                            reason=observed.get('reason'))
                if self.interrupted():
                    return fail('Operator interrupted after target camera observation.')
                if not observed.get('found'):
                    return fail('Another target view did not produce a valid measurement: ' + str(observed.get('reason', 'target not found')))
                if observed.get('target_id') != target_id:
                    return fail('The new view did not preserve the original target identity. No orbit was started.')
                target = self._target(target_id)
                if target.get('origin_id') != origin:
                    return fail('Target coordinate origin changed during geometry acquisition. No orbit was started.')
                step['orbit_usable'] = bool(target['geometry'].get('orbit_usable'))
            if self.interrupted():
                return fail('Operator interrupted orbit geometry measurement.')
            if not ready(target):
                return fail('Required target shape/height remains partial after two additional measured views. No maneuver was started.')
            return target, history, None
        except (ValueError, KeyError, RuntimeError, OSError) as error:
            return fail('Orbit geometry measurement failed: ' + str(error))

    def _facing_transit(self, target, view, altitude, speed, origin):
        """Fly to a side view along one smooth arc round the target, nose on its centre.

        The arc is a native continuous orbit (a fraction of a revolution), so
        the camera keeps facing the target without stop-and-turn legs. A short
        final goto then settles on the exact measurement viewpoint.
        """
        geometry = target['geometry']
        center = point_xy(geometry.get('orbit_center') or geometry['center'])
        _, north, east = pose(self.client.state())
        start = math.atan2(east - center[1], north - center[0])
        end = math.atan2(view['east_m'] - center[1], view['north_m'] - center[0])
        sweep = math.remainder(end - start, 2 * math.pi)
        if abs(sweep) > math.radians(5):
            result = self.flight('orbit', {'center_north_m': center[0], 'center_east_m': center[1],
                'radius_m': math.hypot(north - center[0], east - center[1]),
                'altitude_agl_m': altitude, 'clockwise': sweep > 0,
                'revolutions': abs(sweep) / (2 * math.pi),
                'maximum_ground_speed_mps': speed, 'face_center': True},
                runtime_target=True, expected_origin_id=origin)
            if result['status'] != 'SUCCEEDED' or self.interrupted():
                return result
        return self.flight('goto', {'north_m': view['north_m'], 'east_m': view['east_m'],
                                    'altitude_agl_m': altitude,
                                    'heading_deg': view['face_observed_midpoint_heading_deg'],
                                    'maximum_ground_speed_mps': speed},
                           runtime_target=True, acceptance_radius_m=0.2, expected_origin_id=origin)

    def orbit_target(self, target_id, standoff_m, revolutions=1.0, clockwise=True,
                     maximum_ground_speed_mps=1.0, camera=None):
        if self.interrupted():
            return self._stop('Operator interrupted target orbit.')
        target = self._target(target_id, static_reference=True)
        target, acquisition, failure = self._acquire_target_geometry(target_id, target, standoff_m, maximum_ground_speed_mps)
        if failure is not None:
            return failure
        geometry = target['geometry']
        state, _, _ = pose(self.client.state())
        center, radius, envelope = orbit_envelope(geometry, standoff_m,
            state.get('estimator', {}).get('horizontal_uncertainty_m'))
        if self.interrupted():
            return self._stop('Operator interrupted before orbit movement.', geometry_acquisition=acquisition)
        metrics = OrbitMeasurements(center, clockwise)
        errors = []
        done = threading.Event()
        def sample():
            while not done.is_set():
                try:
                    metrics.observe(self.client.state())
                except Exception as error:
                    errors.append(str(error))
                done.wait(0.2)
        thread = threading.Thread(target=sample, daemon=True)
        thread.start()
        try:
            result = self.flight('orbit', {'center_north_m': center['north_m'],
                'center_east_m': center['east_m'], 'radius_m': radius,
                'altitude_agl_m': state['relative_altitude_m'],
                'clockwise': clockwise, 'revolutions': revolutions,
                'maximum_ground_speed_mps': maximum_ground_speed_mps, 'face_center': True,
                **({'camera': camera, 'target_altitude_agl_m': _aim_height(target, state)}
                   if camera and camera != 'none' else {})},
                runtime_target=True, expected_origin_id=target["origin_id"])
        finally:
            done.set()
            thread.join(timeout=3)
        evidence = metrics.result()
        evidence.update(result.get('evidence', {}))
        if 'no_flight_executed' in evidence:
            # The inner orbit can be blocked after this skill has already
            # moved to acquire geometry. Scope the flag to that operation.
            evidence['orbit_not_dispatched'] = evidence.pop('no_flight_executed')
        evidence['camera_visibility_verified'] = False
        evidence['geometry_acquisition'] = acquisition
        evidence['orbit_envelope'] = envelope
        # Object standoff sizes the measured target enclosure. It is not a
        # global exclusion radius around unrelated trees, walls or vehicles.
        # Ordinary Drone API policy continues to enforce collision clearance.
        evidence['other_obstacle_clearance'] = 'Drone API collision-clearance policy; independent of target standoff'
        evidence.update(target_id=target_id, requested_revolutions=revolutions,
                        circle_radius_m=radius, minimum_requested_surface_clearance_m=standoff_m,
                        clearance_tolerance_m=0.25,
                        geometry_basis='observed target hull; unseen surfaces remain unknown',
                        reference_kind=target.get('reference_kind', 'measured_static_geometry'),
                        target_observation_available_before_maneuver=target.get('current_observation_available', target.get('found', False)),
                        constant_surface_clearance=False, sampling_errors=len(errors))
        # Geometric coverage is measured, never inferred from action acceptance.
        if result['status'] == 'SUCCEEDED' and (metrics.samples < 2 or
                evidence['angular_coverage_deg'] < revolutions*360 - 8):
            result = {**result, 'status': 'FAILED', 'message': 'Orbit ended without measured requested angular coverage.'}
        elif result['status'] == 'SUCCEEDED':
            result = {**result, 'message': 'Completed the measured static-reference orbit. ' + result.get('message', '')}
        return {**result, 'evidence': evidence}

    def overfly_target(self, target_id, clearance_above_m=None, maximum_ground_speed_mps=1.0):
        defaults = operating_defaults()
        if clearance_above_m is None:
            clearance_above_m = defaults['overflight_clearance_m']
        if type(clearance_above_m) not in (int,float) or not math.isfinite(clearance_above_m) or clearance_above_m <= 0:
            raise ValueError('Overflight clearance must be a positive finite distance.')
        target = self._target(target_id, static_reference=True)
        acquisition = []
        if (not target['geometry'].get('orbit_usable') or target['footprint'].get('height_m') is None
                or target['footprint'].get('height_clipped')):
            target, acquisition, failure = self._acquire_target_geometry(
                target_id, target, defaults['object_standoff_m'], maximum_ground_speed_mps, require_height=True)
            if failure is not None:
                return failure
        footprint, geometry = target['footprint'], target['geometry']
        height = footprint.get('height_m')
        if height is None or footprint.get('height_clipped') or not geometry.get('orbit_usable'):
            raise ValueError('Target top height or footprint is incomplete. Observe its top and another side before overflight.')
        state, n, e = pose(self.client.state())
        altitude = max(state['relative_altitude_m'], float(height) + clearance_above_m)
        # Climb outside the structure before translating above the measured
        # footprint. This is a reusable maneuver, not a compiled user mission.
        result = self.flight('goto', {'north_m': n, 'east_m': e,
             'altitude_agl_m': altitude, 'maximum_ground_speed_mps': maximum_ground_speed_mps},
             runtime_target=True, acceptance_radius_m=0.2, expected_origin_id=target["origin_id"])
        if result['status'] != 'SUCCEEDED' or self.interrupted():
            return result
        # The forward/upward LiDAR can lose a static roof below its field of
        # view after climbing. Retain this maneuver's measured static geometry,
        # with the same origin bound on both legs; do not label it a new scan.
        center = geometry['center']
        result = self.flight('goto', {'north_m': center['north_m'], 'east_m': center['east_m'],
            'altitude_agl_m': altitude, 'maximum_ground_speed_mps': maximum_ground_speed_mps},
            runtime_target=True, acceptance_radius_m=0.2, expected_origin_id=target["origin_id"])
        observed_state, _, _ = pose(self.client.state())
        completion, completed_n, completed_e = pose(result.get('state') or observed_state)
        top_down = footprint.get('top_down_m')
        completed_local = completion['local_position_ned']
        clearance = (float(top_down) - completed_local['down_m']
                     if isinstance(top_down, (int, float)) and 'down_m' in completed_local
                     else completion['relative_altitude_m'] - float(height))
        horizontal_error = math.hypot(completed_n-center['north_m'], completed_e-center['east_m'])
        reached = result['status'] == 'SUCCEEDED'
        if result['status'] == 'SUCCEEDED':
            result = {**result, 'message': 'Reached the overhead waypoint at the measured target center; camera inspection is separate.'}
        local = observed_state['local_position_ned']
        camera_views = {}
        if 'down_m' in local:
            from python.dcm.camera_geometry import project_point
            top = {**center, 'origin_id': target['origin_id'],
                   'down_m': (float(top_down) if isinstance(top_down, (int, float))
                              else local['down_m'] + observed_state['relative_altitude_m'] - float(height))}
            camera_views = {camera: project_point(observed_state, top, camera)
                            for camera in ('forward_d435i', 'downward_brio')}
        return {**result, 'evidence': {'target_id': target_id, 'measured_top_above_home_m': height,
                'relationship': 'above_measured_static_target_center', 'overhead_waypoint_reached': reached,
                'requested_clearance_above_m': clearance_above_m,
                'clearance_above_measured_top_m': clearance,
                'clearance_basis': 'vehicle pose and retained static geometry; not a new range capture',
                'horizontal_error_to_measured_center_m': horizontal_error,
                'arrival_tolerance_m': 0.2,
                'reference_kind': target.get('reference_kind', 'measured_static_geometry'),
                'target_observation_available_before_maneuver': target.get('current_observation_available', target.get('found', False)),
                'geometry_acquisition': acquisition, 'camera_view_geometry': camera_views,
                'commanded_altitude_above_home_m': altitude, 'landing_assessed': False,
                'geometry_basis': 'static target snapshot measured before climb',
                'next_observation': 'Use a camera whose projection includes the measured target; geometry is not image proof. Assess landing zone only if landing there is requested.'}}

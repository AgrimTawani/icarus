"""Non-actuating verification for an aircraft already landed and disarmed.

Matches native landing completion tolerances; this verifies physical recovery,
not whether the user's mission was accomplished.
"""

import math
import time

# Native WaitForLanding uses the configured 30 degree tilt ceiling, 0.35 m/s
# settled speeds, and a one second dwell. These are conservative client proof
# thresholds, not permission to change the server's flight policy.
MAX_TILT_DEG = 30.0
MAX_SPEED_MPS = 0.35
MAX_AGE_MS = 500
SETTLE_SECONDS = 1.0


def verify_ground_recovery(client, state_to_dict, *, timeout_s=2.5):
    deadline = time.monotonic() + timeout_s
    stable_since = None
    first_attitude_stamp = None
    reason = 'fresh settled telemetry unavailable'
    while True:
        state = state_to_dict(client.state())
        if state.get('manual_override_active'):
            return {'status': 'manual_override'}
        if state.get('landed') is not True or state.get('armed') is not False:
            return {'status': 'FAILED', 'message': 'Ground recovery not confirmed: aircraft is not landed and disarmed.', 'state': state}
        now_ms = time.time() * 1000
        try:
            stamps = [int(state.get(key, 0)) for key in
                      ('observed_at_unix_ms', 'attitude_received_at_unix_ms')]
            fresh = all(stamp > 0 and 0 <= now_ms - stamp <= MAX_AGE_MS for stamp in stamps)
            roll = float(state['attitude']['roll_rad'])
            pitch = float(state['attitude']['pitch_rad'])
            horizontal = float(state['ground_speed_mps'])
            vertical = float(state['velocity_ned_mps']['z'])
            finite = all(math.isfinite(value) for value in (roll, pitch, horizontal, vertical))
        except (KeyError, TypeError, ValueError, OverflowError):
            fresh = finite = False
        if fresh and finite:
            if math.cos(roll) * math.cos(pitch) < math.cos(math.radians(MAX_TILT_DEG)):
                return {'status': 'FAILED', 'message': 'Abnormal landing: disarmed aircraft is not upright; operator inspection required.', 'state': state}
            settled = 0 <= horizontal <= MAX_SPEED_MPS and abs(vertical) <= MAX_SPEED_MPS
            reason = 'aircraft motion has not settled' if not settled else 'sustained fresh telemetry unavailable'
        else:
            settled = False
            reason = 'fresh finite attitude and velocity telemetry unavailable'
        now = time.monotonic()
        if not settled:
            stable_since = first_attitude_stamp = None
        elif stable_since is None:
            stable_since, first_attitude_stamp = now, stamps[1]
        elif now - stable_since >= SETTLE_SECONDS and stamps[1] > first_attitude_stamp:
            return {'status': 'SUCCEEDED', 'message': 'Aircraft landed and disarmed; upright and settled.',
                    'state': state, 'evidence': {'no_flight_command_sent': True,
                    'end_state_already_satisfied': True, 'settled_duration_s': now - stable_since}}
        if now >= deadline:
            return {'status': 'FAILED', 'message': 'Ground recovery not confirmed: ' + reason + '.', 'state': state}
        time.sleep(0.05)

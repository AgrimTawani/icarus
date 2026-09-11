# Icarus V1 Simulation Missions

## Scope

These are the frozen Phase 0 missions. They exercise the deterministic Drone
API and safety layer before an LLM is allowed to control the simulated vehicle.
All position tolerances are measured from simulator ground truth as well as the
vehicle estimate.

## Common Preconditions

- Vehicle starts landed, disarmed, and inside the geofence.
- EKF, attitude, altitude, and required position sources are healthy.
- Battery is at or above the configured takeoff threshold.
- MAVLink heartbeat and state timestamps are fresh.
- No unresolved safety fault is active.
- Every mission has a 120-second episode timeout unless stated otherwise.

## Mission Definitions

### M01 — Arm and Disarm

Command the vehicle to arm, remain stationary for 3 seconds, then disarm.

Success: the vehicle reports armed and then disarmed, never leaves the ground,
and produces no rejected or unacknowledged command.

### M02 — Take Off

Take off vertically to 5 m above home and hold for 5 seconds.

Success: altitude enters and remains within 0.5 m of the target, horizontal
drift stays below 1.0 m, and no safety intervention occurs.

### M03 — Hover

Take off to 5 m and hover for 20 seconds.

Success: at least 95% of samples stay within 0.5 m vertically and 1.0 m
horizontally of the hold point; roll and pitch remain within 15 degrees.

### M04 — One Local Waypoint

From a 5 m hover, fly 20 m north while retaining 5 m altitude and hold for
5 seconds.

Success: the final position is within 1.5 m horizontally and 0.5 m vertically
of the waypoint, without breaching configured speed or geofence limits.

### M05 — Short Route

Fly a 20 m square at 5 m altitude and return to the first corner.

Success: every waypoint is reached within 1.5 m, the route completes within
90 seconds, and the vehicle never cuts a corner outside the geofence.

### M06 — Return Home

From a point 30 m north and 10 m east at 8 m altitude, request return-to-launch.

Success: RTL is acknowledged, the aircraft returns within 2 m of home and lands,
and it disarms automatically within 10 seconds of touchdown.

### M07 — Land

From a stable 5 m hover, land at the current horizontal position.

Success: touchdown occurs within 1.0 m of the commanded point, vertical speed
does not exceed the configured descent limit, and the vehicle disarms.

### M08 — Cancel Active Action

Start a 40 m waypoint action and cancel it after the aircraft travels at least
5 m.

Success: the waypoint action becomes cancelled within 2 seconds, the vehicle
does not continue toward the old target, and it enters the configured safe hold.

### M09 — Reject Unsafe Action

Request an altitude, speed, or destination beyond a configured safety limit.

Success: guardrails reject the request before MAVLink transmission, emit a
machine-readable reason, and leave the current safe action unchanged.

### M10 — Recover from Action Failure

During a waypoint action, inject one deterministic failure such as a rejected
command acknowledgement or stale state update.

Success: the executor detects the failure, does not repeatedly issue the same
unsafe command, records the failed action, and transitions to hold or RTL as
defined by the safety policy.

## Required Episode Evidence

Each run records the scenario and seed, configuration revisions, commands,
acknowledgements, estimated and ground-truth state, safety decisions, action
transitions, timing, final outcome, and failure reason. Passing once is not a
release gate; the later test campaign defines repetition and seed requirements.

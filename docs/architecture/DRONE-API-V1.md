# Drone API V1 Contract

Status: Phase 8–9 implementation contract, 2026-09-16

## Purpose

The Drone API is Icarus's stable aircraft boundary. Deterministic clients and
later DCM runtimes use the same typed operations in simulation and on physical
hardware. Neither client type can access MAVLink, motors, raw attitude/rate
targets, arbitrary parameters, shell commands or safety-policy mutation.

The V1 contract deliberately implements a small flight kernel while reserving
typed extension points for richer navigation, perception, payload, fleet and
maintenance services. High-performance flight is obtained by deterministic
local planning and flight-control loops, not by asking a language model for
high-rate setpoints.

## Public Services

`SessionService` owns session negotiation and capability discovery:

- `Connect`
- `CloseSession`
- `GetCapabilities`

`AuthorityService` provides exclusive, expiring control leases:

- `AcquireControl`
- `RenewControl`
- `ReleaseControl`

`StateService` separates observation from mutation:

- `GetState`
- `WatchState`
- `GetHealth`
- `WatchEvents`

`PerceptionService` exposes compact deterministic outputs, never raw pixels or
point clouds:

- `GetPerception`
- `WatchPerception`

`ActionService` validates, submits and observes aircraft actions:

- `ValidateAction`
- `Arm`
- `Disarm`
- `Takeoff`
- `Goto`
- `ExecuteRoute`
- `Hold`
- `ReturnHome`
- `Land`
- `Orbit`
- `CancelAction`
- `GetActionStatus`
- `WatchActionStatus`

The source of truth for request and response fields is `proto/icarus/v1`.

## Authority

The order of flight authority is:

1. safety supervisor;
2. manual operator;
3. scripted deterministic autonomy;
4. DCM autonomy.

Observers never obtain control. Maintainer access is a separate disarmed/bench
role and does not outrank an active flight controller. Every lease expires
unless renewed. Higher-priority acquisition preempts the old lease; commands
carrying the old lease fail validation immediately.

The server will bind an authenticated principal to a session. Request fields do
not create authority merely because a caller supplies a privileged role name.

## Command Invariants

Every command includes a request ID, idempotency key, vehicle and client
identity, control lease, issue and expiry times, minimum observed state sequence and
trace ID. DCM arming will additionally require an approved mission
authorization.

Submission and physical completion are distinct. An accepted command proceeds
through legal action states and ends exactly once as succeeded, cancelled,
timed-out, failed, preempted or aborted by safety. A retry with the same
idempotency key returns the original action rather than moving the aircraft
twice. Each terminal record retains the final normalized state and a SHA-256
digest of the original request.

## Coordinates and Units

Units are SI. Angular state is radians; explicit heading requests are degrees.
Geographic positions identify whether altitude is AMSL, above home or above
terrain. Local positions use north-east-down and name their origin. Commands
with missing or incompatible frames are rejected rather than guessed.

## Continuous Validation

Validation occurs when a request is proposed, again when accepted for
execution, and continuously while it is active. The current simulation policy
checks at least:

- state freshness and expected sequence;
- current control lease and manual override;
- armed/landed preconditions;
- estimator and position availability;
- home availability;
- battery takeoff reserve;
- altitude, distance, horizontal speed, climb and descent limits;
- route size and every route point;
- cancellation and conflicting-action state.
- perception freshness, local obstacle clearance and bounded detour validity
  for local navigation.

Native ArduPilot arming checks, estimator failsafes and geofencing remain
enabled as independent downstream protection.

## Extension Rule

New product capabilities must be typed and capability-advertised. Public DCM
operations describe objectives such as orbit, follow path, inspect or survey.
High-rate velocity, spline and trajectory operations belong to a private local
planner interface protected by a watchdog. Payload actuation, configuration,
airspace, fleet coordination and exceptional safety mechanisms use separate
services and authority policies rather than a generic command string.

## Verification

Generate and compile the contracts and native foundation with:

```bash
./scripts/generate-proto
cmake -S . -B build/phase8 -G Ninja -DCMAKE_BUILD_TYPE=Debug
cmake --build build/phase8 -j 2
ctest --test-dir build/phase8 --output-on-failure
```

The broader development gate is:

```bash
./scripts/check-workspace --scope dev
```

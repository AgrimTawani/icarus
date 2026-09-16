# Phase 8 Acceptance

Status: passed on 2026-09-12

## What was tested

ArduPilot SITL and the canonical Icarus simulation were started independently.
The C++ `icarus-drone-api` process exclusively owned the TCP MAVLink connection.
The acceptance client connected only to `127.0.0.1:50051` with generated gRPC
bindings; it does not import or transmit MAVLink.

| Case | Result | Duration |
| --- | --- | ---: |
| M01 arm/disarm | Passed | 4.86 s |
| M02 takeoff and hold | Passed | 30.37 s |
| M03 20-second hover | Passed | 45.61 s |
| M04 local waypoint | Passed | 33.38 s |
| M05 short route and land | Passed | 53.67 s |
| M06 return home and land | Passed | 73.82 s |
| M07 land | Passed | 24.26 s |
| M08 cancel active action | Passed | 26.34 s |
| M09 reject unsafe destination | Passed | 2.01 s |
| X01 orbit | Passed | 34.38 s |
| M10 injected stale state | Passed (native deterministic test) | 0.06 s suite |

The machine-readable live report was
`logs/phase8/v1_acceptance_20260912T220649_132f4b.json`. Logs are intentionally
ignored because they are run artifacts; this document is the versioned evidence.

## Safety result

M09 was rejected by guardrails before vehicle transmission. M10 advances a fake
clock beyond the configured 500 ms state-age limit during an executing hold. It
verifies the terminal state `ACTION_STATE_ABORTED_BY_SAFETY`, reason
`REASON_CODE_STATE_STALE`, and an ArduPilot BRAKE-mode recovery command.

The independent safety-supervisor loop additionally watches an armed aircraft
outside the RPC execution path. Its policy is: BRAKE on stale state, RTL after
the configured delay, RTL on link loss or low battery/geofence breach, LAND on
critical battery, and autonomous-action preemption on manual takeover.

## Reproduce

In terminal 1:

```bash
./scripts/start-sim --profile simulation-empty
```

In terminal 2:

```bash
./scripts/start-autonomy
```

In terminal 3:

```bash
./scripts/test-phase8
```

For a short API-only mission instead of the full gate:

```bash
./scripts/run-mission --mission takeoff_hover_land
```

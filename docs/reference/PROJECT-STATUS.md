# Project Status

Last updated: 2026-09-19

## Executive Summary

Icarus has a verified custom Gazebo/ArduPilot simulation foundation. Its
canonical vehicle derives mass, centre of gravity and inertia from a versioned
20-component manifest. Its single physical-noise sensor model and seeded
three-axis turbulent atmosphere run in populated worlds, including shear,
aerodynamic forces/moments and obstacle wakes. It is not yet an autonomous
LLM-controlled drone stack.

| Capability | State | Evidence or source |
| --- | --- | --- |
| Vehicle/BOM/safety scope | Complete | `VEHICLE-MARK4-V2-BOM.md`, `V1-MISSIONS.md`, `config/safety/v1.yaml` |
| Official Iris foundation | Complete | `SIM-CONNECTION-BASELINE.md` |
| Project simulation workspace | Complete | `simulation/`, coordinate-frame validation |
| Custom chassis and dynamics | Component-derived and flight-tested | `config/simulation/vehicle_components.json` |
| ArduPilot propulsion bridge | Complete | automated Phase 3–4 acceptance |
| Native simulated sensors | One physical-noise model, connected and health-tested | Phase 4 physical test |
| Turbulent atmosphere | Seeded turbulence, shear, drag, moments and obstacle wakes | `config/simulation/atmosphere.json` |
| Deterministic Phase 5 scenarios | Complete with stochastic-weather replay seeds | Phase 5 acceptance |
| Separated simulator/control launch | Complete; named profiles and global readiness | Phase 6 |
| Keyboard manual control | Implemented; simulation-only | `scripts/manual-control` |
| Xbox manual control | Background polling hardware-verified | SDL Xbox 360 mapping |
| Forward video | H.264/RTP onboard stream and ground viewer verified | `scripts/view-camera` |
| C++ autonomy services | Phase 8 complete | Drone API, authority, guardrails, executor, state engine, safety supervisor and MAVLink gateway |
| Drone API protobuf | V1 flight contract defined and generated | 23 RPCs; C++/Python message and gRPC bindings |
| Perception/obstacle avoidance | Phase 9 complete | live Gazebo LiDAR, normalized map, gRPC summary, A* detours and BRAKE fail-safe |
| DCM/model integration | Local Qwen connected in observe mode behind a versioned contract and wall-clock deadline; not evaluated | `python/dcm/contract.py`, `python/dcm/llama_runtime.py` |
| Dataset/evaluation system | Episodes, replay, corpus builder and offline decision evaluation implemented; Phase 12 campaign not run | `python/dcm/evaluate.py`, `scripts/fly-episode-corpus` |
| Reproducible runtime | Complete | pinned sources/packages, bootstrap, containers and CI |
| Real hardware integration | Not started | deferred Phase 13 |

## Latest Verified Simulation Results

- The component-derived/single-sensor-model regression passed five consecutive
  flights with 0.967–0.984 real-time factor and clean teardown.
- All eight scenario builds, obstacle scoring and the integrated adverse-weather
  flight passed. The adverse run recorded 723 wind samples at 10 Hz and achieved
  0.967 real-time factor.
- The final populated light-wind acceptance passed with physical ArduPilot IMU,
  barometer, GPS and magnetometer uncertainty: 0.033 m maximum drift, 0.58°
  maximum tilt, 2.973–3.026 m altitude, landing and disarm.
- The physical sensor test measured non-zero noise at the intended rate on all
  nine sensors. Flight recording additionally verifies the atmosphere channel.
- The operator accepted the final Xbox manual-flight and camera test. The run
  exercised arming, STABILIZE/LOITER mode changes, RTL recovery and disarming;
  recorded all ten telemetry channels; streamed 3,977/3,977 video frames with
  zero push failures; and shut down without a stale active session. An altitude
  fence event was recovered during the flight and remains in the local log.
- Phase 6 passed 20 consecutive `simulation-empty` start/ready/stop cycles with
  session and port cleanup after every run. Failure injection passed for
  occupied ports, duplicate launchers, invalid scenarios, unavailable hardware
  profiles, interruption, stale sensors, recorder exit, Gazebo exit and SITL
  exit. The `simulation-wind` profile then passed a full takeoff-hover-land run
  at 0.034 m maximum drift and 0.56° maximum tilt.
- The 8 m/s limit scenario was rejected before processes started or motors armed.

Raw evidence remains local under ignored `logs/simulation/`; concise acceptance
reports are versioned as `SIM-*` documents.

## Known Technical Gaps

- Component design-target inputs and aerodynamic coefficients require as-built
  measurements before the simulator can be called a validated digital twin.
- Gazebo LiDAR is integrated with local avoidance; camera semantics and
  physical sensor fusion remain later work.
- The DCM has no mission orchestration, approval mode or closed-loop autonomous
  simulation mode, and no model has been evaluated. A local Qwen proposes in
  observe mode only, and at one recorded decision point it proposed climbing
  immediately after a safety abort where the flight landed.
- No real Pixhawk, Jetson or physical sensor adapter has passed a test.

## Current Gate

Phase 8 is complete. The local C++ daemon owns the ArduPilot MAVLink connection
and exposes typed gRPC session, authority, state, health and action services.
Commands are lease-controlled, idempotent, deadline-bound, validated against a
versioned policy and tracked to physical terminal conditions. An independent
safety monitor covers stale state, link loss, low/critical battery, geofence
breach and manual takeover. `run-mission` and `test-phase8` are pure Drone API
clients and never import a MAVLink library.

The 2026-09-12 acceptance ran M01–M09 plus Orbit through the Drone API against
ArduPilot SITL; every case passed. A native injected-failure test covers M10 by
making telemetry stale during an action and verifying an
`ABORTED_BY_SAFETY` result plus BRAKE recovery. See
`simulation/PHASE-8-ACCEPTANCE.md`.

Phase 9 is complete. The runtime consumes live 360° multilayer Gazebo LiDAR,
normalizes body-frame returns into an expiring local-NED map, exposes perception
health through gRPC, plans clearance-inflated local detours and independently
commands BRAKE for stale perception or immediate frontal hazards. The headless
stress acceptance reached its destination through a safe detour in turbulent
gusts. Independent ground truth measured zero collisions, 1.606 m minimum
clearance and a 0.540 m goal error. A live LiDAR dropout produced the required
safety abort/BRAKE and recovered before landing; see
`simulation/PHASE-9-ACCEPTANCE.md`.

Phase 10 simulation capture is implemented. Every Drone API mission client
creates a sealed episode with time-aligned state/perception, actions, validation
inputs/results, terminal statuses and configuration snapshots. Offline replay
verifies integrity and re-runs the exact C++ guardrails; see
`simulation/PHASE-10-EPISODES.md`. A fresh Phase 9 stress flight and its 677
record episode passed after the LiDAR map was corrected to use full vehicle
attitude, not yaw alone. Phase 11 can begin in observe mode. Real-flight
privacy/retention review and physical sensor drivers remain later work; their
normalized contracts and frame-parity tests already exist.

Phase 11 has a versioned model contract and a working llama.cpp adapter. The
contract declares the action table, the generated prompt, freshness limits and
the runtime descriptor; the adapter runs the pinned Qwen3-4B Q5_K_M behind a
wall-clock deadline that abandons the request and restarts the server. Local
Qwen now proposes against sealed episodes in observe mode, and has executed
nothing.

A fresh headless Phase 9 stress flight passed on 2026-09-19 with the corrected
full-attitude LiDAR transform: zero collisions, 1.804 m minimum clearance,
0.448 m goal error, 11.9 m/s peak wind, and a perception dropout that produced
`ABORTED_BY_SAFETY` and recovered. Its episode passed the Phase 10 replay gate.

A first corpus evaluation ran 10 episodes across empty, wind, obstacle,
adverse and perception-stress profiles, three runs each, 123 decision points.
The model produced no malformed output at all: zero invalid proposals, zero
runtime errors, one timeout, six stale refusals, and nothing executed.

Its 74.8% agreement over comparable points should not be read as a pass. The
per-action breakdown shows the model never once proposed `land`: at all 27
land decision points it deterministically proposed climbing (18) or holding
(9). Agreement on `arm`, `takeoff` and `hold` is total; on `land` it is zero.
A single aggregate score would have reported a passable result for a model
that cannot end a flight. The cause is not established and is a hypothesis
about the prompt and observation rather than a measured property of the model.

Latency: a 9370 ms cold start that exceeds the 5 s deadline outright, against
a 540 ms steady-state median. A deployed loop must make a throwaway decision
before the mission starts.

63 Python unit tests, both C++ suites and the Phase 10 replay gate passed on
2026-09-19, alongside five consecutive headless corpus flights. Phase 10's full exit gate and all Phase 11 evaluation and flight
gates remain open. See [`../NEXT-STEPS.md`](../NEXT-STEPS.md) for the handoff.

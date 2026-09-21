# Project Status

Last updated: 2026-09-21

## Executive Summary

Icarus has a verified custom Gazebo/ArduPilot simulation foundation. Its
canonical vehicle derives mass, centre of gravity and inertia from a versioned
20-component manifest. Its single physical-noise sensor model and seeded
three-axis turbulent atmosphere run in populated worlds, including shear,
aerodynamic forces/moments and obstacle wakes. It has a simulator-only DCM
control path, but it is not an LLM-controlled drone approved for unattended
operation.

Update: the typed live DCM path now exists (chat, approval mode and
simulator-only autonomous mode), but the evaluated language models do **not**
meet the defined unattended-flight threshold. Phase 12 now has replay-verified
simulator MAVLink loss and latency evidence; the full scenario campaign and
model-promotion gate are still open. Bounded semantic detection and a
conservative depth landing assessment are implemented; a VLM and a calibrated
downward depth source remain open.

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
| Low-battery preflight gate | Simulator API path verified | Scenario SOC 35% rejects Arm at the typed C++ guardrail; physical battery integration remains deferred |
| DCM timeout containment | Simulator API path verified | One live deadline was retained, made zero action calls, and replayed successfully |
| DCM/model integration | Live chat, observe, approval and simulator-only autonomous paths exist; evaluated Qwen/Llama candidates fail promotion thresholds | `python/dcm/fly.py`, `python/dcm/llama_runtime.py` |
| Semantic vision | Pinned Grounding-DINO supports bounded class detection; a footprint-aware depth analyzer safely refuses the current forward camera; VLM and calibrated downward-source landing geometry remain open | `python/perception/vision.py`, `python/perception/landing_zone.py` |
| Dataset/evaluation system | Episodes, replay, corpus builder, live DCM provenance and offline decision evaluation implemented; Phase 12 promotion campaign remains open | `python/dcm/evaluate.py`, `scripts/fly-episode-corpus` |
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
- The DCM has live mission orchestration, approval mode and simulator-only
  autonomous mode, but no evaluated model is approved for unattended control.
  The held-out model gate remains open; a local Qwen previously proposed
  climbing immediately after a safety abort where the flight landed.
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
attitude, not yaw alone. The live DCM console now seals successful, failed,
interrupted and operator-declined sessions truthfully; low-battery and timeout
episodes have also replayed. Real-flight privacy/retention review and physical
sensor drivers remain later work; their normalized contracts and frame-parity
tests already exist.

Phase 11 has a versioned model contract and a working llama.cpp adapter. The
contract declares the action table, the generated prompt, freshness limits and
the runtime descriptor; the adapter runs pinned local artifacts behind a
wall-clock deadline that abandons the request and restarts the server. The live
DCM supports chat, observe, approval and simulator-only autonomous operation;
flight actions still traverse the typed Drone API, native guardrails and the
independent executor. A bounded `detect` semantic tool invokes a pinned
Grounding-DINO artifact on an ephemeral simulator frame and never gains flight
authority. No language model is approved for unattended flight.

A fresh headless Phase 9 stress flight passed on 2026-09-19 with the corrected
full-attitude LiDAR transform: zero collisions, 1.804 m minimum clearance,
0.448 m goal error, 11.9 m/s peak wind, and a perception dropout that produced
`ABORTED_BY_SAFETY` and recovered. Its episode passed the Phase 10 replay gate.

A first corpus evaluation ran 10 episodes across empty, wind, obstacle,
adverse and perception-stress profiles, three runs each, 123 decision points.
The model produced no malformed output at all: zero invalid proposals, zero
runtime errors, one timeout, six stale refusals, and nothing executed.

Its 74.8% agreement over comparable points should not be read as a pass. The
per-action breakdown showed the model never once proposed `land`: at all 27
land decision points it proposed climbing (18) or holding (9), while agreeing
perfectly on `arm`, `takeoff` and `hold`.

A controlled experiment established the cause as the contract, not the model.
The v1 observation never said what had already been done. Contract v2 adds the
sequence of completed actions; on the same episodes, `land -> takeoff` fell
from 18 to 1, `land -> land` rose from 0 to 9, and every other action was
untouched. The fix is partial: only 9 of 27 land points are correct and `hold`
is now the dominant wrong answer at 17, so the failure changed from dangerous
to conservative rather than disappearing. Aggregate agreement moved 74.8% to
83.2%, which describes that change far less usefully than the breakdown.

Three further variants were run, each changing one thing. Mission elapsed time
made the result worse; a directive ending-guidance prompt scored highest but
produced the only invalid outputs seen in this work and increased climbing
after a safety abort; Q4 outscored Q5 on every axis, contradicting the
reasoning that selected Q5. None of those three is actionable: they separate by
two or three situations out of the nine mission-ending situations the corpus
contains. Growing the corpus is now the binding constraint on every open
question.

Temperature 0 did not guarantee determinism: one land decision flipped between
repeats on identical input, because prompt-cache reuse changes floating-point
reduction order enough to flip a near-tied argmax.

Latency: cold starts of 4257, 9370 and 13259 ms across sessions, against a
507-540 ms steady-state median. The cold start is not a stable quantity and
cannot be accommodated by choosing a deadline; a deployed loop must warm itself
before the mission begins.

The Python unit suite, both C++ suites and the Phase 10 replay gate have passed
against the current codebase. Phase 10's full physical-data exit gate and the
Phase 11/12 model-promotion gates remain open. See
[`../NEXT-STEPS.md`](../NEXT-STEPS.md) for the current handoff.

# Project Status

Last updated: 2026-09-12

## Executive Summary

Icarus has a verified custom Gazebo/ArduPilot simulation foundation through
Phase 5. It can generate deterministic populated worlds, launch the custom
vehicle against ArduPilot SITL, expose native Gazebo sensors, inject declared
wind/public-sensor faults, run an automated takeoff-hover-land mission and score
the result. It is not yet an autonomous LLM-controlled drone stack.

| Capability | State | Evidence or source |
| --- | --- | --- |
| Vehicle/BOM/safety scope | Complete | `VEHICLE-MARK4-V2-BOM.md`, `V1-MISSIONS.md`, `config/safety/v1.yaml` |
| Official Iris foundation | Complete | `SIM-CONNECTION-BASELINE.md` |
| Project simulation workspace | Complete | `simulation/`, coordinate-frame validation |
| Custom chassis and dynamics | Complete for current simulation gate | `SIM-PHASE-3-4-COMPLETION.md` |
| ArduPilot propulsion bridge | Complete | automated Phase 3–4 acceptance |
| Native simulated sensors | Connected and health-tested | Phase 4 acceptance |
| Deterministic Phase 5 scenarios | Complete | `SIM-PHASE-5-COMPLETION.md` |
| Separated simulator/control launch | Implemented; reliability gate pending | Phase 6 |
| Keyboard manual control | Implemented; simulation-only | `scripts/manual-control` |
| Xbox manual control | Implemented; hardware verification pending | SDL mapping |
| C++ autonomy services | Scaffold only | empty component directories |
| Drone API protobuf | Initial draft | `proto/icarus/v1/` |
| Perception/obstacle avoidance | Not implemented | directories are scaffolds |
| DCM/model integration | Not implemented | runtime directories are scaffolds |
| Dataset/evaluation system | Architecture only | planned Phases 10–12 |
| Container/CI reproducibility | Not implemented | Phase 7 |
| Real hardware integration | Not started | deferred Phase 13 |

## Latest Verified Simulation Results

- Phase 3–4 automated gate used five consecutive flights plus motor, landing and
  dynamics checks, as explicitly selected instead of a manual-flight gate.
- Phase 5 adverse-combined flight passed with 0.664 m maximum drift, 17.90°
  maximum tilt, 2.979–3.008 m hover altitude and 0.978 real-time factor.
- The populated light-wind flight passed at approximately 0.148 m maximum
  drift, 9° maximum tilt and 0.975 real-time factor.
- The separated Phase 6 server/client path passed a light-wind flight on
  2026-09-12: the server remained active across client connection, with 0.150 m
  maximum drift, 9.01° maximum tilt, successful landing and clean port release.
- The 8 m/s limit scenario was rejected before processes started or motors armed.
- The 5 m/s strong-wind stress run held altitude but failed its drift/tilt
  envelope (1.40 m and 20.41°). This remains an open calibration item.

Raw evidence remains local under ignored `logs/simulation/`; concise acceptance
reports are versioned as `SIM-*` documents.

## Known Technical Gaps

- Phase 6 still needs Xbox hardware verification and the 20-cycle lifecycle gate.
- Strong-wind force/aerodynamic calibration does not meet the declared envelope.
- Public Gazebo camera/lidar/range streams are health-tested but not yet fused
  into Icarus perception or the ArduPilot EKF.
- Obstacle routes have ground truth and scoring, but no route executor or local
  avoidance planner exists.
- External repositories are documented at known-good commits but the installer
  still follows upstream branches.
- Heavy Python ML dependencies share one environment; no runtime/container
  profile split exists.
- No real Pixhawk, Jetson or physical sensor adapter has passed a test.

## Next Gate

Phase 6 must now verify keyboard/Xbox flight, complete failure injection and pass
20 consecutive clean simulator lifecycle cycles. Only then should implementation
of the autonomy services begin.

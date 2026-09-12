# Project Status

Last updated: 2026-09-12

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
| Separated simulator/control launch | Implemented; long lifecycle soak remains | Phase 6 |
| Keyboard manual control | Implemented; simulation-only | `scripts/manual-control` |
| Xbox manual control | Background polling hardware-verified | SDL Xbox 360 mapping |
| Forward video | H.264/RTP onboard stream and ground viewer verified | `scripts/view-camera` |
| C++ autonomy services | Scaffold only | empty component directories |
| Drone API protobuf | Initial draft | `proto/icarus/v1/` |
| Perception/obstacle avoidance | Not implemented | directories are scaffolds |
| DCM/model integration | Not implemented | runtime directories are scaffolds |
| Dataset/evaluation system | Architecture only | planned Phases 10–12 |
| Container/CI reproducibility | Not implemented | Phase 7 |
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
- The 8 m/s limit scenario was rejected before processes started or motors armed.

Raw evidence remains local under ignored `logs/simulation/`; concise acceptance
reports are versioned as `SIM-*` documents.

## Known Technical Gaps

- The 20-cycle lifecycle soak remains pending; shorter launch/failure and five-
  flight teardown regressions pass.
- Component design-target inputs and aerodynamic coefficients require as-built
  measurements before the simulator can be called a validated digital twin.
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

Treat the current simulator interfaces and physics sources as frozen while DCM
work begins. Define the DCM's versioned state/perception input, schema-valid
high-level action output, model-provider boundary and deterministic mock-model
tests. The DCM must never connect directly to MAVLink; later integration passes
its proposed actions through the Drone API and guardrails.

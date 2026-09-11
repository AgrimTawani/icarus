# Icarus Simulation Operator Guide

## Standard Commands

Run from the repository root:

```bash
# Automated, headless takeoff-hover-land
./scripts/sim --scenario empty_validation

# Same mission with Gazebo GUI
./scripts/sim --scenario wind_light --gui

# Stress test; currently expected to report an envelope failure
./scripts/sim --scenario wind_strong --gui

# Validate the declared operating-limit rejection without starting Gazebo
./scripts/sim --scenario wind_limit_reject
```

Stop an active foreground launch with `Ctrl+C`. The launcher handles the signal,
stops owned children and releases TCP 5760 and UDP 9002. Do not kill individual
children first unless diagnosing a cleanup failure.

## Scenario Catalog

| Name | Environment and purpose | Current expectation |
| --- | --- | --- |
| `empty_validation` | Empty, still-air baseline | Pass |
| `wind_light` | Mixed village, constant 1.5 m/s wind | Pass |
| `wind_strong` | Mixed village, constant 5 m/s wind | Known failing stress case |
| `wind_gusting` | Mixed village, 3 m/s with 60% gust amplitude | Pass target |
| `wind_direction_change` | Mixed village, 2.5 m/s and ±90° swing | Pass target |
| `wind_limit_reject` | 8 m/s exceeds 6 m/s declared limit | Reject before launch |
| `obstacle_course` | Buildings, trees, walls and scored routes | Hover pass; route autonomy not built |
| `adverse_combined` | Wind, obstacles, noise, link faults and low battery | Integrated Phase 5 pass |

The strong-wind vehicle held altitude in the latest run but reached 1.40 m
drift and 20.41° tilt, beyond its 0.75 m/18° scenario envelope. The supervisor
correctly stopped the flight. Do not loosen the gate to hide this result; tune
wind-force/aerodynamic representation and controller parameters with evidence.

## What the Launcher Does

1. Loads and strictly validates the scenario JSON.
2. Rejects contradictory overrides and out-of-policy wind before arming.
3. Checks required ports.
4. Builds deterministic generated world/model artifacts.
5. Starts Gazebo, ArduPilot SITL, controller and recorders.
6. Arms, takes off, hovers, lands and disarms.
7. Scores dynamics, health, mission outcome and real-time factor.
8. Writes a structured run directory and shuts everything down.

## Manual Testing Status

The GUI currently displays an automated flight; it does not imply joystick or
Mission Planner/QGroundControl ownership. Phase 6 will add a supported manual
profile with a published MAVLink endpoint, safe control handover, preflight
checklist and cleanup behavior. Until that profile exists, use the automated
scenarios for acceptance and the GUI only for visual inspection.

## Build and Score Without Flying

```bash
/usr/bin/python3 scripts/simulation/build_phase5_world.py obstacle_course

/usr/bin/python3 scripts/simulation/score_phase5_trajectory.py \
  obstacle_course trajectory.json --route open
```

Generated world/model files are disposable. Edit the JSON scenario,
environment preset, vehicle source model or builder—not a generated copy.

## Evidence

Run artifacts are stored under `logs/simulation/` and ignored by Git. A result
should identify the scenario, input hashes, process logs, health record,
trajectory/dynamics metrics and terminal state. Copy only concise, dated
acceptance conclusions into a dated `SIM-*` report; do not commit bulk recordings.

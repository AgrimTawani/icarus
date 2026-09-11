# Icarus

Icarus is a safety-bounded drone autonomy platform being developed first in
Gazebo and ArduPilot SITL, then for a Pixhawk-controlled physical aircraft. The
long-term system lets interchangeable local language models plan missions
through a typed Drone API while deterministic flight control, safety limits and
emergency behavior remain outside the model.

The repository currently contains a custom 10-inch quadrotor digital twin,
realistic Gazebo environments, native simulated sensors, ArduPilot SITL
integration, repeatable wind/fault scenarios and automated acceptance tests.
The Drone API and autonomy services are architectural contracts and scaffolds;
they are not yet a flight-ready autonomy implementation.

## Start Here

1. Read the [project goal](docs/GOAL.md).
2. Check the [master plan](docs/MASTER-PLAN.md) and
   [current status](docs/PROJECT-STATUS.md).
3. Understand the [software architecture](docs/SOFTWARE-ARCHITECTURE.md)
   and [simulation connections](docs/SIMULATION-CONNECTIONS.md).
4. Follow [getting started](docs/GETTING-STARTED.md) and the
   [simulation operator guide](docs/SIMULATION-OPERATIONS.md).

The complete documentation index is [docs/README.md](docs/README.md).

## Current Simulation Entry Point

```bash
./scripts/sim --scenario wind_light --gui
```

Run without `--gui` for automated or headless validation. Generated worlds,
logs, external source checkouts and local environments are intentionally not
versioned.

## Safety Boundary

The language model is not a flight controller. It may request bounded,
high-level actions such as takeoff, waypoint travel, return-to-home or land.
ArduPilot closes the attitude/rate loops; Icarus guardrails validate requests;
and an independent safety supervisor may reject, cancel, land or RTL. No model
gets direct motor, MAVLink, shell or unrestricted operating-system access.

## Project State

Phases 0–5 are complete. Phase 6—the unified, operator-friendly launch and
manual-test surface—is next. The strong-wind scenario is deliberately retained
as a failing stress case until wind-force and control calibration are completed.
See [PROJECT-STATUS.md](docs/PROJECT-STATUS.md) for exact evidence and
known limitations.

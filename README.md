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
   [current status](docs/reference/PROJECT-STATUS.md).
3. Understand the [software architecture](docs/architecture/SOFTWARE-ARCHITECTURE.md)
   and [simulation connections](docs/architecture/SIMULATION-CONNECTIONS.md).
4. Follow [getting started](docs/guides/GETTING-STARTED.md) and the
   [simulation operator guide](docs/guides/SIMULATION-OPERATIONS.md).

The complete documentation index is [docs/README.md](docs/README.md).

## Simulation and Control Entry Points

```bash
# Start the world, vehicle and ArduPilot without taking control
./scripts/start-sim --profile simulation-wind --gui

# In a second terminal, choose one control client
./scripts/manual-control
./scripts/run-mission --mission takeoff_hover_land

# Legacy one-shot acceptance run
./scripts/sim --profile simulation-wind --gui
```

List every supported or reserved profile with
`./scripts/start-sim --list-profiles`. The simulator and control clients are
separate processes. This allows keyboard, Xbox, deterministic mission and future
autonomy clients to use the same local MAVLink endpoint without rebuilding the
world. Generated worlds, logs, external source checkouts and local environments
are intentionally not versioned.

## Safety Boundary

The language model is not a flight controller. It may request bounded,
high-level actions such as takeoff, waypoint travel, return-to-home or land.
ArduPilot closes the attitude/rate loops; Icarus guardrails validate requests;
and an independent safety supervisor may reject, cancel, land or RTL. No model
gets direct motor, MAVLink, shell or unrestricted operating-system access.

## Project State

Phases 0–7 are complete. The next implementation phase is the typed Drone API,
state engine, MAVLink gateway, missions and deterministic guardrails. Simulation
realism will be audited later against as-built measurements. See
[PROJECT-STATUS.md](docs/reference/PROJECT-STATUS.md) for exact evidence and
known limitations.

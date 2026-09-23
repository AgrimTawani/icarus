# Icarus

Icarus is a safety-bounded drone autonomy platform being developed first in
Gazebo and ArduPilot SITL, then for a Pixhawk-controlled physical aircraft. The
long-term system lets interchangeable local language models plan missions
through a typed Drone API while deterministic flight control, safety limits and
emergency behavior remain outside the model.

The repository contains a custom 10-inch quadrotor digital twin, populated
Gazebo environments, native simulated sensors, ArduPilot SITL integration,
repeatable wind/fault scenarios, and tested Drone API, safety and local
avoidance services. A first offline observe-mode DCM mock exists, but no LLM
controls the drone and the stack is not validated for physical autonomy.

## Start Here

1. Read the [project goal](docs/GOAL.md).
2. Check the [master plan](docs/MASTER-PLAN.md) and
   [current status](docs/reference/PROJECT-STATUS.md), then read the
   [agent handoff](docs/NEXT-STEPS.md).
3. Understand the [software architecture](docs/architecture/SOFTWARE-ARCHITECTURE.md)
   and [simulation connections](docs/architecture/SIMULATION-CONNECTIONS.md).
   The separate experimental AirSim ArduCopter simulation track is documented in
   [AirSim Simulation](docs/architecture/AIRSIM-SIMULATION.md); it does not
   replace Gazebo as the promotion baseline.
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

# Small AirSim compatibility track (after ./scripts/fetch-airsim-blocks)
./scripts/start-airsim-stack --binary \
  third_party/airsim/blocks-v1.8.0-linux/Blocks/LinuxNoEditor/Blocks.sh
# second terminal: ./scripts/start-autonomy-airsim

# High-detail city track (one-time CC-BY-NC asset installation first)
./scripts/fetch-airvln-environments
./scripts/start-airsim-realistic
# second terminal: ./scripts/start-autonomy-airsim
# third terminal:  ./scripts/view-airsim-camera
# fourth terminal: ./scripts/dcm-fly --runtime llama --profile edge-vision \
#   --mode autonomous --context-length 4096 --max-decisions 18 \
#   --mission "Arm, take off to 3 metres, check that building, hover over it \
#   for 3 seconds, count no. of people, then return home and land."
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

Phases 0–9 are reported complete; episode/replay, local Qwen DCM and closed-loop
simulation paths are implemented, but no language model is approved for
unattended flight. V2 Phase 1's AerialClaw-inspired candidate and AirSim
ArduCopter track are implemented and tested. The frozen promotion scorecard
retains the current DCM because V3 regressed timeout/latency and lacks paired
live Gazebo evidence. Simulation realism still requires as-built validation. See
[PROJECT-STATUS.md](docs/reference/PROJECT-STATUS.md) for exact evidence and
known limitations.

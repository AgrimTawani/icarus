# Icarus

Icarus is a safety-bounded drone autonomy platform being developed first in
Gazebo and ArduPilot SITL, then for a Pixhawk-controlled physical aircraft. The
long-term system lets interchangeable local language models plan missions
through a typed Drone API while deterministic flight control, safety limits and
emergency behavior remain outside the model.

The repository contains a custom 10-inch quadrotor digital twin, populated
Gazebo environments, native simulated sensors, ArduPilot SITL integration,
repeatable wind/fault scenarios, and tested Drone API, safety and local
avoidance services. A simulator-only local-LLM DCM can propose typed actions in
approval or closed-loop modes; it remains unapproved for unattended or physical
flight. The default mission agent uses Qwen 3.5 4B and a
[direct feedback loop](docs/architecture/DIRECT-MISSION-AGENT.md): read measured
state/perception, select one native tool or a short unconditional sequence,
wait for the result, then decide again. Clarifications stay in the same
conversation. The whole-mission semantic compiler, resolver, admission/review
paths and upfront mission readback have been removed.

## Start Here

1. Read the [project goal](docs/GOAL.md).
2. Check the [master plan](docs/MASTER-PLAN.md) and
   [current status](docs/reference/PROJECT-STATUS.md), then read the
   [agent handoff](docs/NEXT-STEPS.md).
3. Understand the [software architecture](docs/architecture/SOFTWARE-ARCHITECTURE.md)
   and [simulation connections](docs/architecture/SIMULATION-CONNECTIONS.md).
   The separate experimental AirSim ArduCopter simulation track is documented in
   [AirSim Simulation](docs/architecture/AIRSIM-SIMULATION.md); it does not
   replace Gazebo as the promotion baseline. The
   [custom Icarus7 readiness report](docs/simulation/AIRSIM-ICARUS7-READINESS.md)
   separates completed source integration from the remaining Unreal build,
   wind and end-to-end runtime gates.
4. Follow [getting started](docs/guides/GETTING-STARTED.md) and the
   [simulation operator guide](docs/guides/SIMULATION-OPERATIONS.md). The exact
   D435i, BRIO, MID-360, ARK Flow, H-RTK and Pixhawk simulation boundary is in
   [Gazebo hardware fidelity](docs/simulation/GAZEBO-HARDWARE-FIDELITY.md).
   Gazebo now also feeds MID-360 proximity, downward range and image-derived
   optical flow into ArduPilot. GPS/barometer remain primary; the linked guide
   documents fusion, control-mode limits and the headless sensor acceptance test.

The complete documentation index is [docs/README.md](docs/README.md).

## End-to-End Architecture

The model receives the four documents in `config/dcm/prompts`, the original
mission, conversation/tool history, and fresh compact telemetry. It selects
one tool. A thin adapter sends typed Drone API requests and returns terminal
results. The model interprets conditions at the point where evidence is
available; it does not compile a mission graph. Operator interrupts and the
Drone API's physical protections continue independently of inference.

See [Direct mission agent](docs/architecture/DIRECT-MISSION-AGENT.md) for the
current commands, model identity, tool contract, and migration limits.

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
#   --mode autonomous --max-decisions 18 \
#   --show-prompts --mission "Arm, take off to 3 metres, yaw right 90 degrees, \
#   check the building, count people, then return home."

# Custom Icarus7 pawn (after the documented UE4.27 source build/package step)
./scripts/start-airsim-icarus7 --gui
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
ArduCopter track are implemented and tested. V3 is now the operator-selected
default so live runs receive the compact reviewed context; `--context v2`
remains available for historical comparisons. The exact prompt composition,
relative-yaw path and ordered execution rules are documented in the
[DCM objective pipeline](docs/architecture/DCM-OBJECTIVE-PIPELINE.md). This
does not waive promotion:
the earlier feedback prompt regressed timeout/latency. The hardened compact
revision now has live Gazebo evidence for building inspection, reviewed orbit,
four-person detection from the forward camera, hover, RTL, gusting wind and
strict deterministic replay. The plan-once revision additionally completed a
76-step yaw/YOLO scan and a three-view yaw/VLM scan with zero per-action model
decisions; both landed through RTL and passed strict replay. It is still not approved for unattended or
physical flight; the full frozen promotion campaign remains open. AirSim's
packaged city uses a generic pawn, while custom Icarus7 Unreal compilation and
wind acceptance remain open. Simulation realism still requires as-built validation. See
[PROJECT-STATUS.md](docs/reference/PROJECT-STATUS.md) for exact evidence and
known limitations.

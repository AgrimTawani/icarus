# Icarus Software Architecture

## Scope and Maturity

This document defines the target architecture and clearly marks the implemented
subset. As of 2026-09-12, the simulation foundation through Phase 5 is working.
The protobuf files are initial contracts; most `cpp/`, `python/` and
`perception/` service directories are scaffolds. Do not confuse the complete
directory layout with a complete autonomy stack.

## System Boundary

```text
 Operator / mission request
            |
            v
  DCM + selected model runtime <------- state/perception summary
            | high-level typed action                  ^
            v                                          |
        Guardrails --------------------------+          |
            | approved action                | reject   |
            v                                v          |
     Mission Executor <-------------- Safety Supervisor|
            | command / cancel               ^          |
            v                                | health   |
        Drone API <-------------------- State Engine ---+
            |                                          ^
            v                                          |
     MAVLink Gateway                                   |
            |                                          |
      +-----+-------------------+                      |
      |                         |                      |
 ArduPilot SITL             Pixhawk 6X                 |
      |                         |                      |
    Gazebo                 Real aircraft               |
      | sensors                 | sensors              |
      +-------------------------+--> Perception -------+

 Every boundary emits events to the episode recorder and evaluation pipeline.
```

## Authority Order

Authority is intentionally asymmetric. A lower item may never override a higher
one:

1. ArduPilot flight-control failsafes and pilot kill/manual override.
2. Icarus safety supervisor emergency land, RTL, hold or termination.
3. Guardrail validation against state freshness, geofence and action limits.
4. Mission executor sequencing and cancellation.
5. DCM/model suggestions.

The DCM is an untrusted planner. It sees a curated state, selects from declared
actions and receives structured results. It cannot issue actuator values,
arbitrary MAVLink messages, shell commands or filesystem/network operations.

## Components

| Component | Repository area | Responsibility |
| --- | --- | --- |
| Protobuf contracts | `proto/icarus/v1` | Versioned state, action, perception and Drone API messages |
| Drone API | `cpp/drone_api` | Stable external action/state service; authentication and request lifecycle later |
| State engine | `cpp/state_engine` | Fresh, normalized vehicle state and health snapshot |
| MAVLink gateway | `cpp/mavlink_gateway` | Sole Icarus owner of MAVLink transport and command translation |
| Mission executor | `cpp/mission_executor` | Deterministic action state machines, deadlines and cancellation |
| Guardrails | `cpp/guardrails` | Precondition and policy validation before execution |
| Safety supervisor | `cpp/safety_supervisor` | Independent monitoring and safe recovery authority |
| Local planner | `cpp/local_planner` | Short-horizon collision-safe motion planning |
| Autonomy core | `cpp/autonomy_core` | Composition root and service lifecycle |
| Perception | `perception/` | Sensor adapters, calibration, detections and obstacle map |
| DCM | `python/dcm` | Model-facing reasoning loop and action selection |
| Model runtime | `python/model_runtime` | Qwen/Llama/provider abstraction and resource controls |
| Evaluation | `python/evaluation` | Frozen campaigns, metrics and comparisons |
| Dataset tools | `python/dataset_tools` | Episode validation, curation, export and replay preparation |
| Simulation | `simulation/`, `scripts/simulation` | Vehicle/world generation, SITL launch, faults and acceptance |

## Control Plane and Data Plane

The control plane contains bounded actions and their lifecycle: request,
validation, acceptance, execution, cancellation and terminal result. Every
request carries an action ID, deadline and contract version. Repeating an action
ID must be idempotent or return the existing result.

The data plane contains timestamped vehicle state, health, perception summaries
and optional sensor payload references. Each sample carries source time,
receive time, frame ID and quality/validity. State age is checked before actions
are accepted. Bulk camera or lidar data never travels through the action RPC.

## Simulation and Physical Parity

| Stable service contract | Simulation adapter | Physical adapter |
| --- | --- | --- |
| Vehicle commands/state | MAVLink to ArduPilot SITL | MAVLink to Pixhawk 6X |
| Dynamics/environment | Gazebo Harmonic | Physical airframe/environment |
| Camera | Gazebo camera/depth plugin | OAK-D or selected camera SDK |
| Lidar | Gazebo GPU lidar | Unitree MID-360 driver |
| Downward range | Gazebo lidar/range sensor | LightWare LW20 driver |
| Position | SITL truth/noisy GPS | GNSS and estimator output |
| Scenario/fault input | Versioned JSON and deterministic seed | Test plan and controlled fault injection |

Mission logic consumes normalized state and perception contracts, never Gazebo
topics or vendor SDK objects. Simulation truth is reserved for scoring and
debugging; it must not leak into the autonomy input path.

## Runtime Topology

The expected process topology after Phase 8 is:

- one Gazebo server plus optional GUI in simulation;
- one ArduPilot SITL process, or one physical Pixhawk transport;
- MAVLink gateway, state engine, safety supervisor and mission executor as
  deterministic native services;
- perception services that can be independently restarted;
- a Python DCM/model worker with strict deadlines and resource limits;
- an episode recorder subscribing to events without blocking control.

Phase 6 first standardizes launch and shutdown of the currently implemented
Gazebo/SITL/controller processes. The same supervisor contract will later own
the service graph.

## API Contract Rules

- Package names remain versioned (`icarus.v1`) until a deliberate migration.
- Actions are high level: arm, takeoff, hover, goto, route, RTL, land and cancel.
- Units are SI and coordinate frames are explicit.
- Every response distinguishes rejected, accepted, active, succeeded, cancelled,
  timed out and failed.
- Unknown fields and unsupported enum values fail safely at trust boundaries.
- Telemetry is streamed separately from commands and includes freshness.
- Safety decisions include machine-readable reason codes.

## Safety Design

Configuration in `config/safety/` is versioned and applied independently of the
model. Guardrails reject requests before execution; the safety supervisor
continuously evaluates state even after acceptance. Loss of the DCM must not
destabilize flight. Loss of companion compute or MAVLink follows an explicitly
tested hold/RTL/land policy, while Pixhawk-native failsafes remain enabled.

Obstacle avoidance is not currently implemented. Gazebo collision geometry and
trajectory scoring are test infrastructure, not proof that the drone can avoid
obstacles autonomously.

## Model Runtime and Evaluation

All model providers implement the same interface: load a named immutable model
revision, consume a bounded prompt/state bundle, return one schema-constrained
action and report timing/token/resource metadata. Provider-specific chat
templates and quantization belong in `python/model_runtime`, not the DCM.

Comparisons use identical prompts, tools, safety policy, scenarios and seeds.
Primary metrics are mission success and safety violations; latency, invalid
action rate, intervention count, path efficiency and compute cost are secondary.
See [`DATA-AND-EVALUATION.md`](DATA-AND-EVALUATION.md).

## Deployment Profiles

- **Development workstation:** Ubuntu 24.04 x86_64, Gazebo GUI or headless SITL,
  optional local inference.
- **Headless evaluator:** version-pinned x86_64 runtime, no GUI, repeated seeded
  campaigns and artifact export.
- **Aircraft companion:** ARM64 Jetson, perception and model runtime, connected
  to Pixhawk; no simulator dependencies.

Containers should provide separate simulation, autonomy and ML profiles rather
than one oversized image. Device, GPU, GUI and network privileges must be
explicit at launch.

## Change Discipline

An architecture change is complete only when its contract, configuration,
tests and this document agree. New dependencies require a documented purpose
and reproducible version. New model capabilities never silently expand control
authority.

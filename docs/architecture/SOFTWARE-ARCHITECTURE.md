# Icarus Software Architecture

## Scope and Maturity

As of 2026-09-19, the deterministic stack through Phase 9 is implemented:
simulation, typed flight services, safety supervision, normalized LiDAR
perception and local avoidance. Phase 10 simulation episode capture and native
guardrail replay are implemented, but its full exit gate remains open. Phase 11
has an offline observe-only mock replay, not a connected model or live DCM.
Physical hardware integration remains a later phase. See
[`../NEXT-STEPS.md`](../NEXT-STEPS.md) for the current handoff.

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

The operator-facing forward-video contract is H.264 over RTP/UDP. In simulation,
a vehicle-side adapter converts the Gazebo RGB topic into that contract; on the
physical companion computer, the camera SDK and hardware encoder replace only
that adapter. The ground-station viewer is identical in both cases. Raw RGB and
depth frames remain separate internal perception inputs and are not transported
through MAVLink or the command API.

Landing-surface assessment is a separate deterministic depth boundary. It
accepts a calibrated float32 depth frame and the camera optical axis in
body-FRD, then returns coverage, plane slope, residual roughness and a bounded
``suitable`` result. It refuses a source that is not calibrated within 15° of
downward; the present forward RGB-D camera therefore cannot certify a landing
area. `scripts/capture-depth-frame` and `scripts/assess-landing-zone` provide
the Gazebo adapter and offline analyzer respectively. A downward depth sensor
and live calibration evidence remain required before this becomes an
operational landing capability.

The adapter was exercised against a real headless `empty_validation` Gazebo
session on 2026-09-21: it captured the native `/icarus/sensors/rgbd/depth_image`
stream as a 640×480 float32-metres frame. Assessing it with the configured
forward optical axis `[1, 0, 0]` returned `assessable: false` and no quality
score. This is positive evidence for the transport and refusal paths only; it
is not evidence that the vehicle can assess or select a landing site.

## Runtime Topology

The expected process topology after Phase 8 is:

- one Gazebo server plus optional GUI in simulation;
- one ArduPilot SITL process, or one physical Pixhawk transport;
- MAVLink gateway, state engine, safety supervisor and mission executor as
  deterministic native services;
- perception services that can be independently restarted;
- a Python DCM/model worker with strict deadlines and resource limits;
- an episode recorder subscribing to events without blocking control.

Phase 6 standardized launch and shutdown of the Gazebo/SITL/controller
processes. The same supervisor contract will later own the service graph.

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

Obstacle avoidance is deterministic and independent of the DCM. A normalized
scan becomes a local-NED obstacle map; the planner inflates returns by policy
clearance, searches a bounded grid, simplifies only line-of-sight-safe segments
and sends those targets through the normal executor. The safety supervisor
commands BRAKE if perception becomes stale or a frontal obstacle enters the
emergency envelope. LiDAR is the V1 avoidance source; camera metadata is
normalized but semantic vision is not flight-critical.

The first semantic-vision boundary is intentionally separate: the Gazebo
adapter captures an RGB frame to a standard image, and the pinned
Grounding-DINO detector receives that image plus an explicit class list and
returns boxes/counts. It cannot call MAVLink or issue actions. A future DCM
tool may request `detect(classes=[...])`, but detection output remains advisory
until a dedicated perception/mission policy validates its use.

For the current simulator-only inspection path:

```bash
./scripts/start-sim --scenario wind_light
./scripts/capture-camera-frame --output logs/vision/frame.ppm
./scripts/detect-image logs/vision/frame.ppm --classes 'person,tree,building'
```

The capture command uses Gazebo transport only; the detector launcher uses the
project virtual environment and the pinned local model. Neither command starts
the Drone API or can arm the vehicle.

When called through the live DCM contract, `detect` follows the same
ephemeral-frame rule. A 2026-09-21 headless check stored its `semantic_detection`
record in episode `20260921T051331_708405511741` with counts, boxes, model hash
and image hash, but no PPM/JPEG/PNG file. This proves the boundary wiring, not
detector accuracy or any autonomous visual-flight capability.

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

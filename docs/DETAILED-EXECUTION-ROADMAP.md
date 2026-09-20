# Detailed Icarus Execution Roadmap

## Goal

Build an autonomy stack that can first operate a realistic Mark4 V2 vehicle in
Gazebo through ArduPilot SITL, and later operate the physical vehicle through a
Pixhawk 6X without changing the Drone API, mission executor, safety rules, DCM
contract, logging format, or evaluation system.

The project must also record complete, reproducible simulation and flight
episodes that can later be curated into training and evaluation datasets.

The repository must be portable: after cloning or pulling a pinned revision on
a supported machine, one documented bootstrap command must recreate the build
and runtime environment. Host installations may be used during early simulator
work, but the finished development and evaluation workflows must also have
containerized, version-pinned paths.

Qwen is only the first model family. The DCM layer must support interchangeable
Qwen, Llama, and future runtimes without changing the Drone API or simulator.
Every model must be compared through the same frozen scenarios, prompts, tool
contract, safety policy, seeds, and scoring system.

## Non-Negotiable Architecture

```text
Human mission
    |
    v
DCM / selected model runtime
    | one structured high-level action
    v
Guardrails and mission executor
    |
    v
Drone API
    |
    v
MAVLink gateway
    |
    +---- simulation ----> ArduPilot SITL ----> Gazebo
    |
    +---- real world ----> Pixhawk 6X --------> Mark4 V2

Gazebo or physical sensors
    |
    v
Perception and State Engine
    |
    +-----------------------------------------> DCM
```

The DCM never sends raw motor commands, never communicates directly with
MAVLink, and never receives shell or unrestricted operating-system access.

## Original Starting Point (historical, before Phase 0)

Already available:

- Ubuntu 24.04 development host
- Gazebo Harmonic
- Official ArduPilot source and ArduCopter SITL build
- Official ArduPilot Gazebo plugin build
- Isolated ArduPilot Python environment with MAVProxy and pymavlink
- Project Python environment with gRPC, Protobuf, testing, data, and ML tools
- C++ compiler, CMake, Ninja, gRPC C++, Protobuf, OpenCV, and GStreamer

Not yet built:

- Icarus-owned Gazebo worlds or vehicle models
- Mark4 V2 digital twin
- Unified launcher
- Icarus source-code structure
- Drone API contract or implementation
- MAVLink gateway
- State Engine, perception, mission executor, or guardrails
- DCM runtime integration
- Automated simulation missions
- Dataset recorder and evaluation harness

## Definition of Done for Every Task

A task is complete only when:

1. Its output is stored in the repository.
2. It has a repeatable command or automated test.
3. Its configuration is not hidden in a developer's shell history.
4. Its logs make failures diagnosable.
5. Simulation-specific behavior is isolated behind an adapter or profile.
6. Another developer can reproduce the result from a clean launch.
7. Dependency versions and external source revisions are pinned.
8. The task works through the documented local or container entry point.

Portability means reproducible on explicitly supported targets, not literally
every computer. Maintain and test a small support matrix:

- x86_64 Ubuntu development workstation, with optional NVIDIA GPU
- x86_64 headless runner for automated simulation and evaluation
- ARM64 NVIDIA Jetson target later, using a JetPack-compatible image

From the first implementation commit, avoid absolute host paths, undocumented
environment variables, and machine-specific configuration. Containers package
the environment; repository scripts remain the stable commands used both
inside and outside them.

---

## Phase 0: Freeze the First Vehicle and Mission Scope

### 0.1 Select the exact Mark4 V2 build

- [x] Select a reinforced, custom-center-chassis derivative of the 10-inch,
      427 mm Mark4 V2. The preferred 7-inch frame and stock Mark4 center plates
      are too constrained for the Thor, its supporting systems, Pixhawk, and
      perception payload.
- [x] Record the X-frame wheelbase and initial motor coordinates.
- [x] Select 4 x Hobbywing XRotor 3115 900 KV motors and record the published
      6S/HQ10x4.5x3 thrust curve.
- [x] Select two CW and two CCW HQ 10 x 4.5 x 3 propellers.
- [x] Select the recommended Hobbywing XRotor FPV G2 65 A 4-in-1 ESC and
      DShot600 hardware protocol.
- [x] Select a 6S 10,000 mAh, at least 30C LiPo reference battery.
- [x] Place the Pixhawk at the center of gravity, arrow forward, on vibration
      isolation.
- [x] Allocate up to 900 g for a flight-suitable Thor T5000 compute assembly;
      the 1.94 kg developer kit is for bench use only.
- [x] Select GNSS/compass, downward rangefinder, forward depth camera, and
      top-mounted 360-degree lidar as the initial sensor suite.
- [x] Set the initial takeoff mass to 4.343 kg, cap the design at 4.5 kg, require
      the horizontal center of gravity within 5 mm of frame center, and preserve
      at least 2.18:1 thrust-to-weight at the initial 70% output ceiling.

Deliverable: [`vehicle/MARK4-V2-BOM.md`](vehicle/MARK4-V2-BOM.md)

### 0.2 Freeze the first mission set

- [x] Arm and disarm.
- [x] Take off to a commanded altitude.
- [x] Hover for a commanded duration.
- [x] Fly to one local waypoint.
- [x] Complete a short waypoint route.
- [x] Return home.
- [x] Land.
- [x] Cancel an active action.
- [x] Reject an unsafe action.
- [x] Recover from one simulated action failure.

Deliverable: [`reference/V1-MISSIONS.md`](reference/V1-MISSIONS.md)

### 0.3 Freeze initial safety limits

- [x] Maximum altitude.
- [x] Maximum horizontal and vertical speed.
- [x] Maximum distance from home.
- [x] Minimum battery for takeoff.
- [x] Battery level that triggers RTL.
- [x] Battery level that triggers landing.
- [x] Maximum accepted state age.
- [x] Required sensor and EKF health for arming.
- [x] Behavior after DCM, Jetson, or MAVLink failure.
- [x] Manual override behavior.

Deliverable: [`config/safety/v1.yaml`](../config/safety/v1.yaml)

### Phase 0 exit gate

- [x] The exact drone configuration is known.
- [x] Every first-version mission has measurable success criteria.
- [x] Safety limits are written before autonomous commands are implemented.

---

## Phase 1: Prove the Installed Simulation Foundation

Do this before creating custom assets. It separates installation problems from
Mark4 model problems.

### 1.1 Run the official Iris world

- [x] Configure the Gazebo plugin and resource paths from the repository.
- [x] Start `iris_runway.sdf`.
- [x] Start ArduCopter SITL with the `gazebo-iris` frame.
- [x] Confirm the JSON physics connection in both directions.
- [x] Connect a deterministic pymavlink test client; retain MAVProxy for
      optional interactive debugging.
- [x] Arm, take off to 5 m, hover, land, and disarm automatically.
- [x] Save the complete commands, structured result, SITL state, and logs.

Deliverable: [`scripts/simulation/run_iris_smoke_test.sh`](../scripts/simulation/run_iris_smoke_test.sh)

### 1.2 Verify the individual connections

- [x] Gazebo starts without missing model or plugin errors.
- [x] Gazebo sends state to SITL on the expected JSON/UDP connection.
- [x] SITL returns four motor outputs to Gazebo.
- [x] SITL emits MAVLink heartbeats.
- [x] The pymavlink test client receives telemetry and issues commands.
- [x] Simulation time and SITL time remain synchronized through lockstep.
- [x] Clean shutdown leaves no stale simulator processes or occupied ports.

Deliverable: [`simulation/CONNECTION-BASELINE.md`](simulation/CONNECTION-BASELINE.md)

### Phase 1 exit gate

- [x] The stock Iris can complete takeoff-hover-land three consecutive times.
- [x] Startup and shutdown are repeatable.
- [x] Any later failure can be identified as an Icarus asset/code failure rather
      than a broken ArduPilot/Gazebo installation.

---

## Phase 2: Create the Icarus Simulation Workspace

### 2.1 Create project-owned directories

```text
simulation/
|-- models/mark4_v2/
|-- worlds/
|-- parameters/
|-- scenarios/
|-- launch/
`-- tests/
```

- [x] Keep custom assets outside `third_party`.
- [x] Add a README describing every asset and its source/license.
- [x] Add a validation command for SDF files.
- [x] Define naming and coordinate-frame conventions.

### 2.2 Establish coordinate conventions

- [x] Document Gazebo world coordinates.
- [x] Document body frame and motor numbering.
- [x] Document ArduPilot NED conversion.
- [x] Document camera and lidar optical frames.
- [x] Create a visible axis/debug model for verification.

Deliverable: [`simulation/COORDINATE-FRAMES.md`](simulation/COORDINATE-FRAMES.md)

### Phase 2 exit gate

- [x] Gazebo resolves all Icarus assets using repository-local paths.
- [x] No custom file modifies an official third-party checkout.
- [x] Coordinate and unit conventions are explicit.

---

## Phase 3: Build the Mark4 V2 Digital Twin

Completed simulation gate, 2026-09-12: [Phase 3–4 acceptance report](simulation/PHASE-3-4-COMPLETION.md).
The user explicitly approved five consecutive automated flights in place of
the original manual-flight gate. This is simulation acceptance, not measured
hardware calibration or real-aircraft approval.

Build it in layers. Do not add all sensors or visual detail at once.

### 3.1 Build the rigid-body model

- [x] Create a simple frame using primitive geometry.
- [x] Set total mass.
- [x] Set center of mass.
- [x] Calculate and set the inertia tensor.
- [x] Add simple collision geometry.
- [x] Verify the model falls and rests correctly under gravity.

Test: spawn, drop from a small height, and confirm stable collision behavior.

### 3.2 Add the four motors

- [x] Add four rotor links and joints.
- [x] Match physical motor positions.
- [x] Set clockwise/counter-clockwise directions.
- [x] Add motor thrust and reaction-torque models.
- [x] Set maximum RPM and response time.
- [x] Verify each motor independently.

Test: command each output separately and confirm the correct rotor and torque
direction.

Deliverable: `scripts/simulation/test_mark4_motors.py` and
`simulation/models/mark4_v2/model.sdf`. Five headless cases verify individual
channels, spin, force/moment directions, response times, balanced collective
thrust and the physical speed ceiling. Passive drop regression also passes.
These tests use direct Gazebo actuator commands; SITL integration is Phase 3.3.
The thrust model remains provisional. The final compact-model gate additionally
compares hover PWM against the reference thrust curve and checks collective
vertical response. Measured hardware torque, motor response and full-curve
calibration remain future hardware-fidelity work.

### 3.3 Connect the ArduPilot Gazebo plugin

- [x] Add the plugin to the Mark4 SDF.
- [x] Map ArduPilot channels 0-3 to the correct rotor joints.
- [x] Configure JSON/UDP ports.
- [x] Configure Gazebo-to-NED and model-to-aircraft transforms.
- [x] Enable lockstep.
- [x] Verify no channel is reversed or swapped.

### 3.4 Create the initial ArduPilot parameters

- [x] Select Quad X frame class/type.
- [x] Configure motor output ordering.
- [x] Configure flight modes required by the first missions.
- [x] Configure conservative speed and acceleration limits.
- [x] Configure geofence and RTL behavior.
- [x] Store settings in `simulation/parameters/mark4_v2_base.parm`.
- [x] Never depend on unrecorded `eeprom.bin` changes.

The profile is checked against live MAVLink parameter readback before arming.
Configuration is verified; geofence breaches and RTL mission behavior still
need dedicated scenario tests. See [Mark4 SITL bring-up](simulation/MARK4-SITL.md).

### 3.5 Validate basic flight dynamics

- [x] Arm without bypassing normal safety checks unnecessarily.
- [x] Take off in an empty world with no wind.
- [x] Hover without continual divergence.
- [x] Check roll, pitch, yaw, climb, and descent directions.
- [x] Land without sustained bounce or ground pass-through (bounded solver penetration documented).
- [x] Compare hover throttle and motor-force/vertical-motion response with the provisional build model.
- [x] Validate the model before tuning complex autonomy; current gates require no further gain changes.

### 3.6 Replace primitive visuals

- [x] Add optimized visual meshes.
- [x] Keep collision meshes simple.
- [x] Confirm visuals do not change mass or inertia.
- [x] Maintain acceptable real-time performance.

### Phase 3 exit gate

- [x] Compact Mark4 completes automated takeoff-hover-land five consecutive times (user-approved replacement for manual gate).
- [x] Motor order and coordinate frames have automated checks.
- [x] Dynamics are plausible and documented, even if not yet perfectly matched
      to measured hardware.

Five final compact-model flights passed, including a noisy-sensor flight:
2.984–3.003 m hover altitude, maximum drift 0.0336 m, maximum tilt 0.3523°.
Motor, landing, visual/physics equivalence and performance checks passed.
Hardware-calibrated torque, motor response and actual packaging remain future
work, not prerequisites of this explicitly provisional simulation gate.

---

## Phase 4: Add Sensors One at a Time

For every sensor, complete the same sequence: model, transport, ArduPilot or
perception configuration, health check, nominal test, noise test, failure test.

### 4.1 Flight-critical baseline

- [x] IMU.
- [x] GPS.
- [x] Compass.
- [x] Barometer.
- [x] Battery state simulation.

### 4.2 Navigation and obstacle sensors

- [x] Downward rangefinder.
- [x] Lidar and forward depth camera.
- Optical flow: not required for the initial GPS-guided mission set; deferred.
- [x] RGB transport prepared for scene-perception experiments; pixels are not recorded.

### 4.3 Sensor validation checklist

- [x] Position and orientation match the intended simulated mount.
- [x] Units and coordinate frame are documented.
- [x] Update rate is measured.
- [x] Latency is simulated at the consumer boundary.
- [x] Gaussian noise/bias are configurable; battery uses configurable electrical parameters.
- [x] Dropout can be injected at the consumer boundary.
- [x] Stale data is detected.
- [x] Health is visible through Gazebo telemetry.

### Phase 4 exit gate

- [x] Each sensor can be tested independently.
- [x] Dropping or freezing observations produces an explicit unhealthy/stale state.
- [x] The vehicle still passes basic flight tests after all sensors are added.

Public sensor transport/perception health is validated; these streams are not
all fused into ArduPilot. See the acceptance report for the exact SITL boundary.

---

## Phase 5: Build Worlds and Scenario Layers

Completed simulation gate, 2026-09-12: [Phase 5 acceptance report](simulation/PHASE-5-COMPLETION.md).

### 5.1 Empty validation world

- [x] Flat ground.
- [x] Known origin and home position.
- [x] No wind.
- [x] No obstacles.
- [x] Minimal environment rendering load.

### 5.2 Wind worlds

- [x] Constant light wind.
- [x] Constant strong wind.
- [x] Gusting wind.
- [x] Changing wind direction.
- [x] Wind limit that forces pre-arm mission rejection.

### 5.3 Obstacle world

- [x] Begin with boxes and walls with known dimensions.
- [x] Add buildings with simplified collision geometry.
- [x] Add trees with explicit trunks and canopy collision policy.
- [x] Create narrow and open navigation routes.
- [x] Define ground-truth obstacle positions for evaluation.

### 5.4 Combined adverse world

- [x] Wind plus obstacles.
- [x] Sensor noise.
- [x] Public Gazebo GPS degradation.
- [x] Public sensor-consumer delay and dropout.
- [x] Low battery condition.

### 5.5 Scenario definitions

Each scenario file must define:

- World and vehicle profile
- Initial position and heading
- Random seed
- Wind configuration
- Obstacle configuration
- Sensor fault schedule
- Mission input
- Maximum duration
- Success and failure conditions

### Phase 5 exit gate

- [x] Every scenario is reproducible from a file and seed.
- [x] World complexity does not prevent the simulator from meeting its required
      real-time factor.
- [x] Ground truth is available for automated scoring.

---

## Phase 6: Build the Bulletproof Simulation Launcher

The launcher is a process supervisor, not a long shell command.

Implemented control separation, 2026-09-12:

```text
./scripts/start-sim --profile simulation-wind --gui
./scripts/manual-control
./scripts/run-mission --mission takeoff_hover_land
```

The first command owns the simulator lifecycle but never commands the vehicle.
Manual control is a direct simulation-only MAVLink client; `run-mission` is now
the reference Drone API client. Keyboard control, the
standard SDL Xbox mapping and the independent H.264/RTP camera viewer are
implemented. The physical Xbox/manual-camera workflow was operator-accepted on
2026-09-12. The subsequent 20-cycle lifecycle and injected-failure gates passed.

### 6.1 Define launch profiles

- [x] `simulation-empty`
- [x] `simulation-wind`
- [x] `simulation-obstacles`
- [x] `simulation-adverse`
- [x] `hardware-bench` (reserved and deliberately fails closed)
- [x] `hardware-flight` (reserved and deliberately fails closed)

Profiles select adapters, endpoints, parameter overlays, sensors, safety policy,
logging level, and DCM operating mode.

### 6.2 Implement startup ordering

- [x] Validate configuration.
- [x] Allocate an episode ID and log directory.
- [x] Check ports and stale processes.
- [x] Start Gazebo headless, running and isolated by partition.
- [x] Spawn the selected vehicle.
- [x] Start SITL with the selected parameter file.
- [x] Wait for the Gazebo/SITL bridge through required sensor streams.
- [x] Run a non-arming MAVLink readiness probe.
- [x] Wait for heartbeat, 3D GPS fix and global position.
- [x] Start sensor consumers.
- [x] Run global readiness checks before publishing the operator session.
- [x] Keep control clients disconnected until readiness succeeds. The world runs
      during estimator initialization instead of using a paused-world design.

### 6.3 Implement failure handling

- [x] Per-process stdout/stderr logs.
- [x] Startup timeout for every dependency.
- [x] Clear error identifying the first failed readiness gate.
- [x] Signal handling for Ctrl+C and process crashes.
- [x] Reverse-order shutdown.
- [x] Release ports and temporary files.
- [x] Final episode status even after failure.

### 6.4 Define the operator commands

Current simulation interface:

```text
./scripts/start-sim --profile <name> [--gui]
./scripts/manual-control [--controller 0]
./scripts/run-mission --mission takeoff_hover_land
Ctrl+C in the simulator terminal to stop
```

### 6.5 Test the launcher

- [x] Launch and stop 20 times without stale processes.
- [x] Recover cleanly when Gazebo fails.
- [x] Recover cleanly when SITL fails.
- [x] Reject occupied ports before launching.
- [x] Reject invalid scenario and unavailable profile configurations.
- [x] Ensure failed readiness never publishes a controllable session.

### Phase 6 exit gate

- [x] A clean simulation can be launched and stopped with one command.
- [x] Startup is deterministic and observable.
- [x] Failures leave the machine ready for the next run.

---

## Phase 7: Scaffold the Shared Codebase and Reproducible Runtime

### 7.1 Create the repository structure

```text
proto/icarus/v1/
cpp/autonomy_core/
cpp/drone_api/
cpp/mavlink_gateway/
cpp/state_engine/
cpp/mission_executor/
cpp/guardrails/
cpp/safety_supervisor/
cpp/local_planner/
python/dcm/
python/model_runtime/
python/evaluation/
python/dataset_tools/
perception/
simulation/
config/
tests/
```

### 7.2 Establish the versioned Protobuf build boundary

- [x] Declare the `icarus.v1` proto3 package in every existing contract file.
- [x] Generate C++ and Python outputs into ignored `build/generated/` paths.
- [x] Keep generated outputs out of source control.
- [x] Defer actual action, state, service, episode and health messages to Phase 8,
      where their semantics and safety contracts are implemented together.

### 7.3 Establish engineering gates

- [x] CMake build for the implemented C++ simulator plugins.
- [x] Protobuf generation for C++ and Python.
- [x] Unit-test commands.
- [x] Formatting and static analysis.
- [x] Configuration and SDF validation.
- [x] No generated code manually edited.

### 7.4 Make a fresh clone reproducible

- [x] Add one bootstrap entry point: `./scripts/bootstrap`.
- [x] Pin direct Python dependencies in purpose-specific requirement sets.
- [x] Record Ubuntu, Python, C++, Gazebo, Protobuf and gRPC compatibility.
- [x] Pin ArduPilot and ArduPilot Gazebo to immutable commits.
- [x] Leave model weights out of bootstrap; model revisions/checksums become
      mandatory when Phase 11 selects a model artifact.
- [x] Generate build metadata containing the Git revision and dirty-state flag.
- [x] Keep secrets, model caches, logs and generated datasets outside Git.
- [x] Add clean-host and development-container CI jobs.

### 7.5 Add containerized workflows

Use containers for the Icarus services and automated headless evaluation. Keep
the Gazebo GUI optional because display and GPU passthrough differ by host.

```text
containers/
|-- Dockerfile.dev
`-- Dockerfile.simulation
```

- [x] Development image with Python, Protobuf, gRPC and test tools.
- [x] Headless simulation image with pinned Gazebo and SITL source revisions.
- [x] GPU support is unnecessary for deterministic non-LLM tests.
- [x] Exclude model weights, secrets, flight logs, generated worlds and local
      environments from the Docker build context.
- [x] Defer slim service-runtime and Jetson/L4T images until those services and
      the target JetPack release exist; creating fake images now would not be
      reproducible.

Target clean-machine workflow:

```text
git clone <icarus-repository>
cd Icarus
./scripts/bootstrap --profile simulation
./scripts/start-sim --profile simulation-empty
```

Target container workflow:

```text
docker build -f containers/Dockerfile.dev -t icarus-dev .
docker run --rm icarus-dev
```

The exact container engine remains replaceable; repository scripts are the
stable user-facing interface.

### Phase 7 exit gate

- [x] C++ and Python protobuf generation succeeds from one `icarus.v1` source.
- [x] Simulation profiles resolve through one versioned profile model; reserved
      hardware profiles fail closed until Phase 13 supplies adapters.
- [x] A fresh Ubuntu 24.04 CI host runs the documented development bootstrap.
- [x] The same development health check runs in a clean container.
- [x] The full simulation container definition uses the same bootstrap and has
      a manually dispatched CI build gate because SITL compilation is expensive.

---

## Phase 8: Implement the Deterministic Autonomy Stack

Implement and test each component without an LLM.

### 8.1 MAVLink gateway

- [x] Own one vehicle connection.
- [x] Maintain heartbeat and connection state.
- [x] Decode required telemetry.
- [x] Send commands and correlate acknowledgements with bounded retry.
- [x] Serialize outbound traffic and publish MAVLink status/events.
- [x] Support the Phase 8 SITL TCP transport. Pixhawk serial/network transport
      is deliberately a Phase 13 adapter, behind the same interface.

### 8.2 State Engine

- [x] Normalize MAVLink flight/health telemetry and units.
- [x] Merge flight, navigation, estimator, sensor and link state. Perception is
      added in Phase 9 through its reserved contract.
- [x] Track observation timestamp, monotonic sequence and source freshness.
- [x] Report missing, stale, degraded, and healthy states explicitly.
- [x] Publish stable `DroneState` snapshots.

### 8.3 Drone API v1

- [x] `connect()`
- [x] `get_state()`
- [x] `arm()` and `disarm()`
- [x] `takeoff()`
- [x] `goto()` and `execute_route()`
- [x] `hold()` and `orbit()`
- [x] `land()`
- [x] `return_home()`
- [x] `cancel_action()`
- [x] `get_action_status()` and action/state event streams

### 8.4 Mission executor

- [x] Unique action IDs and request hashes.
- [x] Accepted, executing, succeeded, rejected, failed, timed-out, cancelled,
      preempted and safety-aborted states.
- [x] Preconditions and physical completion detection.
- [x] Timeouts and bounded retries.
- [x] Cancellation and safe BRAKE recovery.
- [x] Only one conflicting vehicle action at a time.

### 8.5 Guardrails and safety supervisor

- [x] Schema and range validation.
- [x] Geofence checks before commands and during flight.
- [x] State-freshness checks before and throughout execution.
- [x] Battery, estimator, navigation and link-health rules.
- [x] Speed and altitude limits.
- [x] Manual override and lease-priority preemption.
- [x] DCM lease-expiry, MAVLink-loss and stale-state policies. Perception failure
      becomes enforceable when Phase 9 supplies perception health.
- [x] BRAKE, RTL and land escalation policy. Airborne forced-disarm remains an
      ArduPilot emergency policy, not a normal companion-computer action.

### 8.6 Scripted controller

- [x] Implement a deterministic client using the same Drone API as the DCM.
- [x] Complete every initial mission without an LLM.
- [x] Make it the reference baseline for model evaluation.

### Phase 8 exit gate

- [x] All first-version missions work through the Drone API.
- [x] The Phase 8 acceptance client sends no MAVLink directly; legacy manual
      and simulator-layer validation tools remain explicitly simulation-only.
- [x] The scripted controller passes M01–M09 and Orbit; native failure injection
      passes M10 stale-state recovery.

---

## Phase 9: Implement Perception and Local Avoidance

### 9.1 Normalize sensor inputs

- [x] Common timestamped image-frame metadata interface for Gazebo and hardware
      cameras; pixels remain outside the control API.
- [x] Common range/point-cloud interface for Gazebo and hardware lidar.
- [x] Capture/receive timestamps, sequence numbers and bounded freshness.
- [x] Gazebo FLU, physical FRD and local-NED frame transformations.

### 9.2 Build deterministic perception outputs

- [x] Nearest-obstacle distance and relative bearing.
- [x] Expiring local-NED obstacle representation with vehicle self-mask and
      terrain/downward-return separation.
- [x] Path-clear status, source health and confidence fields.
- [x] Landing-zone quality is explicitly unavailable in V1; downward landing
      analysis is not required by the frozen missions and remains an extension.

### 9.3 Local avoidance

- [x] Independent BRAKE response for stale perception or an obstacle inside the
      emergency envelope.
- [x] Bounded, clearance-inflated A* detours with line-of-sight simplification.
- [x] Send planned collision-free position targets through the existing
      ArduPilot gateway; raw proximity injection is not used in this V1 path.
- [x] Keep reactive collision avoidance independent of the DCM.

### Phase 9 exit gate

- [x] A headless scripted mission avoids a wall/building/tree field using live
      Gazebo LiDAR under turbulent gusting wind. Independent Gazebo truth found
      zero collisions and 1.606 m minimum clearance including vehicle radius.
- [x] A live injected LiDAR dropout safety-aborts the active action, commands
      BRAKE, recovers perception, then permits a controlled landing.
- [x] Equivalent Gazebo-FLU and physical-FRD normalized scan fixtures produce
      identical local-NED obstacle input to the same algorithm. Actual physical
      sensor recordings remain a Phase 13 hardware gate.

---

## Phase 10: Build Logging, Replay, and Dataset Capture

Do this before adding Qwen so baseline and failure data are not lost.

Simulation capture and replay are implemented; see
`docs/simulation/PHASE-10-EPISODES.md`. Phase 11 may begin in observe mode.
The physical-flight privacy/retention and complete planner/MAVLink trace gates
below remain open and do not authorize autonomous hardware operation.

### 10.1 Define an immutable episode format

Record:

- Episode ID and timestamps
- Git revisions and build versions
- Simulation seed or physical vehicle ID
- Configuration and parameter hashes
- Human mission
- State snapshots
- Perception outputs
- Proposed and executed actions
- Guardrail decisions
- MAVLink acknowledgements
- Action outcomes and durations
- Safety events and operator overrides
- Final mission score

### 10.2 Separate data views

- [ ] Raw operational log (compact API event log exists; raw sensors are separate).
- [x] Time-aligned replay record.
- [x] Evaluation summary.
- [x] Candidate training example (unapproved by default).
- [ ] Human annotation and approval status.

Never train directly from raw logs.

### 10.3 Build replay

- [x] Replay state and action sequences without Gazebo.
- [x] Re-run guardrails against recorded actions.
- [x] Re-run DCM decisions against frozen state snapshots.
- [x] Compare controller or model versions on identical episodes.

### 10.4 Protect real-flight data

- [x] Remove session/lease and operator-identifying fields from the compact record.
- [ ] Define retention and backup policy.
- [x] Record whether an action was proposed, approved, executed, or overridden.
- [x] Never label a failed or unsafe action as preferred automatically.

### Phase 10 exit gate

- [ ] Every mission produces a complete episode artifact.
- [ ] An episode can be replayed and scored deterministically.
- [ ] Simulation and physical flights use the same episode schema.

---

## Phase 11: Integrate the DCM

The first offline observe-mode wiring slice is implemented in
`python/dcm/observe.py` and `scripts/observe-dcm`. It passed unit tests and
produced five non-executed mock proposals on a sealed stress episode on
2026-09-19. This does **not** complete any model-runtime or DCM exit gate:
the mock always proposes `none`, no Qwen/Llama adapter is connected, and the
current timeout only rejects a response after it returns. See
`docs/architecture/DCM-OBSERVE-V1.md` and `docs/NEXT-STEPS.md`.

### 11.1 Model runtime abstraction

- [x] Define a provider-independent `ModelRuntime` interface.
- [ ] Support small local Qwen and Llama-family models first.
- [x] Keep model loading separate from mission logic.
- [x] Record model name, quantization, prompt version, and sampling settings.
- [ ] Add runtime adapters without changing the DCM controller.
- [ ] Support local Transformers, llama.cpp, or another selected runtime behind
      the same interface.

Each model is selected by configuration rather than source changes:

```yaml
model:
  id: qwen3-4b-instruct
  family: qwen
  runtime: llama_cpp
  artifact_revision: <immutable-revision>
  quantization: q4_k_m
  context_length: 8192
  temperature: 0.0
  prompt_version: dcm-v1
```

### 11.2 DCM controller loop

- [x] Receive mission, state, previous result, and allowed actions.
- [x] Produce exactly one structured action.
- [x] Reject prose or malformed output.
- [x] Apply a decision timeout.
- [x] Prevent shell, direct MAVLink, and unrestricted file/network access.

### 11.3 Operating modes

- [x] Observe: proposals are logged but never executed.
- [x] Approval: valid proposals require operator approval.
- [x] Autonomous simulation: valid proposals execute automatically.
- [x] Hardware modes remain locked until later phase gates pass.

### 11.4 Build the model evaluation harness

Every candidate model receives the same:

- Frozen Drone API schema
- System prompt and prompt version
- State snapshots and previous action results
- Mission set and scenario seeds
- Safety limits and guardrail implementation
- Context budget and decision timeout
- Number of repeated runs

The harness must run both offline decision tests and full closed-loop simulator
missions. Offline tests are fast and isolate tool selection; simulator tests
measure the consequences of a sequence of decisions.

### 11.5 Score models on multiple dimensions

- [ ] Mission success rate.
- [x] Invalid-action rate.
- [ ] Correct API/tool selection rate.
- [x] Argument validity and accuracy.
- [ ] Guardrail rejection rate.
- [ ] Recovery success rate.
- [ ] Action count and completion time.
- [ ] Safety interventions.
- [x] Decision latency and timeout rate.
- [ ] Tokens per second.
- [ ] Peak VRAM and system RAM.
- [x] Model crash or runtime failure rate.
- [x] Consistency across repeated runs and seeds.

Do not choose a model from one combined number alone. Produce a weighted project
score plus the underlying metrics so safety, reliability, speed, and resource
trade-offs remain visible.

### 11.6 Generate comparable reports

Target command:

```text
icarus eval run --suite dcm-v1 --models qwen3-4b,llama-3b --repeats 5
icarus eval compare --run <evaluation-id>
```

Target outputs:

```text
evaluations/<evaluation-id>/
|-- manifest.yaml
|-- per_episode.jsonl
|-- summary.json
|-- comparison.csv
`-- report.html
```

- [x] Store exact model artifact revision and checksum.
- [ ] Store prompt, API schema, code, parameters, and scenario revisions.
- [x] Report confidence intervals or run-to-run variation.
- [x] Preserve failed traces for diagnosis.
- [x] Prevent evaluation episodes from entering training data.
- [ ] Compare fine-tuned models against their own base model and the scripted
      controller.

### Phase 11 exit gate

- [ ] DCM passes observe-mode evaluation on unseen missions.
- [ ] Approval-mode actions remain inside the safety policy.
- [ ] Autonomous simulation never bypasses the deterministic executor.
- [ ] Qwen and Llama candidates can be evaluated without changing simulator or
      autonomy-core code.
- [x] A versioned comparison report identifies the best model for the current
      constraints and shows why it won.

---

## Phase 12: Full Simulation Test Campaign

### 12.1 Golden test run

- [ ] Empty world.
- [ ] Fixed seed.
- [ ] Takeoff, route, return, and land.
- [ ] Complete logs and replay.
- [ ] Scripted and DCM controllers compared on the same scenario.

### 12.2 Scenario matrix

- [ ] Light and strong wind.
- [ ] Gusts.
- [ ] Static obstacles.
- [ ] Narrow route.
- [ ] GPS degradation.
- [ ] Sensor dropout.
- [ ] MAVLink delay and loss.
- [ ] Low battery.
- [ ] Rejected unsafe mission.
- [ ] DCM timeout or crash.

### 12.3 Regression policy

- [ ] Fixed test seeds for regression.
- [ ] Separate randomized seeds for robustness.
- [ ] Pass thresholds established before model tuning.
- [ ] Failed episodes automatically retained.
- [ ] No fine-tuning data appears in held-out evaluation scenarios.

### Phase 12 exit gate

- [ ] Deterministic controller meets the required reliability threshold.
- [ ] DCM meets the defined model threshold on held-out scenarios.
- [ ] Safety supervisor handles every injected critical failure.
- [ ] Results are reproducible from stored episode metadata.

---

## Phase 13: Preserve the Path to Real Hardware

Detailed physical-flight development is deliberately deferred. During
simulation work, enforce only the architectural constraints needed to avoid a
rewrite later.

- [ ] Keep Protobuf contracts, Drone API, State Engine, mission executor,
      guardrails, DCM, logging, and evaluation independent of Gazebo.
- [ ] Put SITL, Gazebo sensors, and simulation time behind interfaces.
- [ ] Select simulation or hardware through a configuration profile.
- [ ] Keep simulation `SIM_*` parameters separate from common vehicle and
      Pixhawk hardware parameters.
- [ ] Provide a Jetson-specific container/build target without forcing desktop
      and Jetson images to use identical NVIDIA system libraries.
- [ ] Use the same episode and evaluation format for simulation and flight.

Future boundary replacements:

```text
SITL transport       -> Pixhawk MAVLink transport
Gazebo sensors       -> Physical sensor drivers
Simulation clock     -> Monotonic hardware clock
Simulation parameters -> Pixhawk hardware overlay and calibrations
```

When simulation milestones are complete, create a separate detailed hardware
plan covering bench tests, calibration, manual tuning, HIL, observe mode,
approval mode, and restricted autonomous flight.

### Phase 13 exit gate

- [ ] No simulation dependency leaks into shared mission or DCM code.
- [ ] A hardware profile can be added by implementing boundary adapters rather
      than rewriting the autonomy stack.

---

## Phase 14: Dataset Curation and Fine-Tuning

Fine-tuning begins only after the API, state schema, safety layer, episode
format, and held-out evaluation suite are stable.

### 14.1 Build datasets from approved episodes

- [ ] Human command to mission intent.
- [ ] State and intent to valid Drone API action.
- [ ] Multi-step mission trajectories.
- [ ] Recovery examples.
- [ ] Preferred versus rejected actions.
- [ ] Simulation-to-real comparison episodes.

### 14.2 Curate rather than copy logs

- [ ] Remove malformed and unsafe examples.
- [ ] Preserve useful failures as rejected examples.
- [ ] Balance ordinary, recovery, and adverse-condition cases.
- [ ] Deduplicate near-identical trajectories.
- [ ] Separate by scenario family before train/validation/test splitting.
- [ ] Freeze the held-out test set before training.

### 14.3 Training progression

- [ ] Prompted base-model baseline.
- [ ] Supervised fine-tuning for schema and tool use.
- [ ] Re-evaluate on the frozen simulator suite.
- [ ] Add recovery/preference training only when metrics justify it.
- [ ] Evaluate real-flight replays before any live use.

### Phase 14 exit gate

- [ ] Fine-tuning improves held-out performance rather than only training
      scenarios.
- [ ] Safety is still enforced by deterministic code.
- [ ] Every deployed model has a versioned evaluation report.

---

## Immediate Next 12 Actions

Complete these in this exact order:

1. [x] Decide the exact Mark4 V2 size and hardware bill of materials.
2. [x] Write the first mission set and measurable success conditions.
3. [x] Write the initial simulation safety limits.
4. [x] Run and document the official Iris takeoff-hover-land smoke test.
5. [x] Verify and document every Gazebo/SITL/MAVLink connection.
6. [x] Create the Icarus-owned `simulation/` directory structure.
7. [ ] Create the primitive rigid-body Mark4 model.
8. [ ] Add and independently validate all four motors.
9. [ ] Connect the Mark4 model to ArduPilot SITL.
10. [ ] Create the Mark4 base `.parm` file.
11. [ ] Pass repeated no-wind takeoff-hover-land tests.
12. [ ] Add the first sensor only after the basic vehicle is stable.

Do not build the DCM integration yet. The first major milestone is a stable,
repeatable Mark4 simulation that can be launched, controlled, inspected, and
stopped without manual cleanup.

While completing these actions, record every dependency and external revision
in repository-owned manifests. Do not allow early simulation code to depend on
this laptop's absolute paths. The full images and clean-clone verification land
in Phase 7, before the Drone API and DCM services are built.

## First Major Milestone

The simulation foundation is complete when one command can:

1. Validate configuration.
2. Start an empty Gazebo world.
3. Spawn the Mark4 V2.
4. Start ArduCopter SITL.
5. Verify the physics and MAVLink connections.
6. Run a scripted takeoff-hover-land mission.
7. Record a complete episode.
8. Shut down cleanly.
9. Repeat successfully without manual intervention.

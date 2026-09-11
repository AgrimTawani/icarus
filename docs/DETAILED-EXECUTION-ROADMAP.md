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

### 6.1 Define launch profiles

- [ ] `simulation-empty`
- [ ] `simulation-wind`
- [ ] `simulation-obstacles`
- [ ] `simulation-adverse`
- [ ] `hardware-bench`
- [ ] `hardware-flight`

Profiles select adapters, endpoints, parameter overlays, sensors, safety policy,
logging level, and DCM operating mode.

### 6.2 Implement startup ordering

- [ ] Validate configuration.
- [ ] Allocate an episode ID and log directory.
- [ ] Check ports and stale processes.
- [ ] Start Gazebo paused.
- [ ] Spawn the selected vehicle.
- [ ] Start SITL with the selected parameter file.
- [ ] Wait for the Gazebo/SITL bridge.
- [ ] Start MAVLink gateway or temporary smoke-test client.
- [ ] Wait for heartbeat and required telemetry.
- [ ] Start sensor consumers.
- [ ] Run global readiness checks.
- [ ] Unpause simulation only after readiness succeeds.

### 6.3 Implement failure handling

- [ ] Per-process stdout/stderr logs.
- [ ] Startup timeout for every dependency.
- [ ] Clear error identifying the first failed readiness gate.
- [ ] Signal handling for Ctrl+C and process crashes.
- [ ] Reverse-order shutdown.
- [ ] Release ports and temporary files.
- [ ] Final episode status even after failure.

### 6.4 Define the operator commands

Target interface:

```text
icarus sim validate
icarus sim launch --scenario empty_hover --seed 42
icarus status
icarus mission run missions/takeoff_hover_land.yaml
icarus logs show <episode-id>
icarus stop
```

### 6.5 Test the launcher

- [ ] Launch and stop 20 times without stale processes.
- [ ] Recover cleanly when Gazebo fails.
- [ ] Recover cleanly when SITL fails.
- [ ] Reject occupied ports before launching.
- [ ] Reject invalid parameter or scenario files.
- [ ] Ensure failed readiness never permits arming.

### Phase 6 exit gate

- [ ] A clean simulation can be launched and stopped with one command.
- [ ] Startup is deterministic and observable.
- [ ] Failures leave the machine ready for the next run.

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

### 7.2 Define versioned Protobuf contracts

- [ ] `action.proto`
- [ ] `action_result.proto`
- [ ] `drone_state.proto`
- [ ] `drone_api.proto`
- [ ] `perception.proto`
- [ ] `episode.proto`
- [ ] `health.proto`

### 7.3 Establish engineering gates

- [ ] CMake build for C++ components.
- [ ] Protobuf generation for C++ and Python.
- [ ] Unit-test commands.
- [ ] Formatting and static analysis.
- [ ] Configuration-schema validation.
- [ ] No generated code manually edited.

### 7.4 Make a fresh clone reproducible

- [ ] Add one bootstrap entry point, such as `./scripts/bootstrap.sh`.
- [ ] Pin Python dependencies with a generated lock file.
- [ ] Pin Ubuntu, compiler, Gazebo, Protobuf, and gRPC compatibility.
- [ ] Pin ArduPilot and ArduPilot Gazebo to recorded commits or releases.
- [ ] Record model files by immutable revision and checksum.
- [ ] Generate build metadata containing the Git revision and dirty-state flag.
- [ ] Keep secrets, model caches, logs, and generated datasets outside Git.
- [ ] Add a clean-machine verification job.

### 7.5 Add containerized workflows

Use containers for the Icarus services and automated headless evaluation. Keep
the Gazebo GUI optional because display and GPU passthrough differ by host.

```text
containers/
|-- dev.Dockerfile
|-- runtime.Dockerfile
|-- simulation.Dockerfile
|-- jetson.Dockerfile
`-- compose.yaml
```

- [ ] Development image with C++, Python, Protobuf, gRPC, and test tools.
- [ ] Headless simulation image with pinned Gazebo and SITL versions.
- [ ] Runtime image containing only the services required to run Icarus.
- [ ] Separate Jetson image based on the matching NVIDIA JetPack/L4T release.
- [ ] GPU support is optional for deterministic non-LLM tests.
- [ ] Persist logs, datasets, model cache, and configuration through explicit
      volumes.
- [ ] Add health checks and dependency ordering to Compose.
- [ ] Do not bake model weights, secrets, or flight logs into images.

Target clean-machine workflow:

```text
git clone <icarus-repository>
cd Icarus
./scripts/bootstrap.sh
./scripts/icarus sim launch --scenario empty_hover
```

Target container workflow:

```text
git pull
docker compose build
docker compose --profile simulation up
```

The exact container engine remains replaceable; repository scripts are the
stable user-facing interface.

### Phase 7 exit gate

- [ ] A minimal C++ gRPC server and Python client exchange a health message.
- [ ] Contracts are versioned as `icarus.v1`.
- [ ] Simulation and hardware profiles use the same domain models.
- [ ] A fresh supported machine can build and run the health check from the
      documented bootstrap path.
- [ ] The headless health check also passes in a container.

---

## Phase 8: Implement the Deterministic Autonomy Stack

Implement and test each component without an LLM.

### 8.1 MAVLink gateway

- [ ] Own one vehicle connection.
- [ ] Maintain heartbeat and connection state.
- [ ] Decode required telemetry.
- [ ] Send commands and correlate acknowledgements.
- [ ] Rate-limit and log traffic.
- [ ] Support SITL TCP/UDP and Pixhawk serial/network transports.

### 8.2 State Engine

- [ ] Normalize telemetry units.
- [ ] Merge flight, sensor, perception, action, and mission state.
- [ ] Track timestamp and age for every source.
- [ ] Report missing, stale, degraded, and healthy states explicitly.
- [ ] Publish stable `DroneState` snapshots.

### 8.3 Drone API v1

- [ ] `connect()`
- [ ] `get_state()`
- [ ] `arm()` and `disarm()`
- [ ] `takeoff()`
- [ ] `goto()`
- [ ] `hold()`
- [ ] `land()`
- [ ] `return_home()`
- [ ] `cancel_action()`
- [ ] `get_action_status()`

### 8.4 Mission executor

- [ ] Unique action IDs.
- [ ] Pending, executing, completed, failed, timed-out, and cancelled states.
- [ ] Preconditions and completion detection.
- [ ] Timeouts and bounded retries.
- [ ] Cancellation and safe recovery.
- [ ] Only one conflicting vehicle action at a time.

### 8.5 Guardrails and safety supervisor

- [ ] Schema and range validation.
- [ ] Geofence checks.
- [ ] State-freshness checks.
- [ ] Battery and sensor-health rules.
- [ ] Speed and altitude limits.
- [ ] Manual override.
- [ ] DCM/MAVLink/perception failure policies.
- [ ] Hold, RTL, land, and disarm escalation policy.

### 8.6 Scripted controller

- [ ] Implement a deterministic client using the same Drone API as the DCM.
- [ ] Complete every initial mission without an LLM.
- [ ] Make it the reference baseline for model evaluation.

### Phase 8 exit gate

- [ ] All first-version missions work through the Drone API.
- [ ] No test script sends MAVLink directly.
- [ ] The scripted controller passes the complete mission suite.

---

## Phase 9: Implement Perception and Local Avoidance

### 9.1 Normalize sensor inputs

- [ ] Common image-frame interface for Gazebo and hardware cameras.
- [ ] Common range/point-cloud interface for Gazebo and hardware lidar.
- [ ] Timestamp synchronization.
- [ ] Frame transformations.

### 9.2 Build deterministic perception outputs

- [ ] Nearest-obstacle distance and bearing.
- [ ] Free-space or occupancy representation.
- [ ] Path-clear status and confidence.
- [ ] Landing-zone quality if required.

### 9.3 Local avoidance

- [ ] Immediate stop/hold behavior.
- [ ] Safe local detour generation.
- [ ] Feed appropriate proximity information to ArduPilot when used.
- [ ] Keep reactive collision avoidance independent of the DCM.

### Phase 9 exit gate

- [ ] Scripted missions avoid known obstacles.
- [ ] Loss of perception produces a safe deterministic response.
- [ ] Equivalent recorded simulated and physical sensor inputs use the same
      perception algorithm.

---

## Phase 10: Build Logging, Replay, and Dataset Capture

Do this before adding Qwen so baseline and failure data are not lost.

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

- [ ] Raw operational log.
- [ ] Time-aligned replay record.
- [ ] Evaluation summary.
- [ ] Candidate training example.
- [ ] Human annotation and approval status.

Never train directly from raw logs.

### 10.3 Build replay

- [ ] Replay state and action sequences without Gazebo.
- [ ] Re-run guardrails against recorded actions.
- [ ] Re-run DCM decisions against frozen state snapshots.
- [ ] Compare controller or model versions on identical episodes.

### 10.4 Protect real-flight data

- [ ] Remove secrets and operator-identifying information.
- [ ] Define retention and backup policy.
- [ ] Record whether an action was proposed, approved, executed, or overridden.
- [ ] Never label a failed or unsafe action as preferred automatically.

### Phase 10 exit gate

- [ ] Every mission produces a complete episode artifact.
- [ ] An episode can be replayed and scored deterministically.
- [ ] Simulation and physical flights use the same episode schema.

---

## Phase 11: Integrate the DCM

### 11.1 Model runtime abstraction

- [ ] Define a provider-independent `ModelRuntime` interface.
- [ ] Support small local Qwen and Llama-family models first.
- [ ] Keep model loading separate from mission logic.
- [ ] Record model name, quantization, prompt version, and sampling settings.
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

- [ ] Receive mission, state, previous result, and allowed actions.
- [ ] Produce exactly one structured action.
- [ ] Reject prose or malformed output.
- [ ] Apply a decision timeout.
- [ ] Prevent shell, direct MAVLink, and unrestricted file/network access.

### 11.3 Operating modes

- [ ] Observe: proposals are logged but never executed.
- [ ] Approval: valid proposals require operator approval.
- [ ] Autonomous simulation: valid proposals execute automatically.
- [ ] Hardware modes remain locked until later phase gates pass.

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
- [ ] Invalid-action rate.
- [ ] Correct API/tool selection rate.
- [ ] Argument validity and accuracy.
- [ ] Guardrail rejection rate.
- [ ] Recovery success rate.
- [ ] Action count and completion time.
- [ ] Safety interventions.
- [ ] Decision latency and timeout rate.
- [ ] Tokens per second.
- [ ] Peak VRAM and system RAM.
- [ ] Model crash or runtime failure rate.
- [ ] Consistency across repeated runs and seeds.

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

- [ ] Store exact model artifact revision and checksum.
- [ ] Store prompt, API schema, code, parameters, and scenario revisions.
- [ ] Report confidence intervals or run-to-run variation.
- [ ] Preserve failed traces for diagnosis.
- [ ] Prevent evaluation episodes from entering training data.
- [ ] Compare fine-tuned models against their own base model and the scripted
      controller.

### Phase 11 exit gate

- [ ] DCM passes observe-mode evaluation on unseen missions.
- [ ] Approval-mode actions remain inside the safety policy.
- [ ] Autonomous simulation never bypasses the deterministic executor.
- [ ] Qwen and Llama candidates can be evaluated without changing simulator or
      autonomy-core code.
- [ ] A versioned comparison report identifies the best model for the current
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

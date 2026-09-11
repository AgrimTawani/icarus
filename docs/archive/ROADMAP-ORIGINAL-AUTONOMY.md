# Archived Roadmap: Original Autonomy Plan

> Historical planning document retained for context. The current plan is
> [`../MASTER-PLAN.md`](../MASTER-PLAN.md), with detailed execution history in
> [`../DETAILED-EXECUTION-ROADMAP.md`](../DETAILED-EXECUTION-ROADMAP.md).

## Purpose

This document is the project plan for building the drone autonomy stack in stages.

The project should begin with a reliable, simulator-based autonomy stack and a well-defined Drone API. The language model should remain replaceable. Larger models, fine-tuning, Jetson deployment, and physical-flight integration should happen only after the API, state representation, guardrails, and evaluation system are reliable.

## Project Assumptions

- Development host: native Ubuntu 22.04 LTS
- Simulator: Gazebo with ArduPilot SITL
- Flight-control protocol: MAVLink
- Flight controller target: Pixhawk 6X
- Onboard compute target: Jetson Thor with 128 GB RAM
- DCM: local Qwen-based LLM
- Initial model: small quantized Qwen model for integration testing
- Larger model and fine-tuning: later project stages
- First development environment: simulation only

## Core Architecture

```text
Human Command
      ↓
Drone Cognitive Model (DCM)
      ↓
Structured Action
      ↓
Guardrail and Execution Layer
      ↓
Drone API
      ↓
MAVLink
      ↓
ArduPilot SITL / Pixhawk 6X
      ↓
Drone or Simulated Drone
      ↓
State Engine
      └──────────────→ DCM
```

The primary command path is:

```text
DCM → Drone API → MAVLink → Flight Controller
```

The State Engine is a side input to the DCM. It continuously supplies structured state updates but does not decide which action should be executed.

## Development Principles

1. The DCM must never generate raw motor commands.
2. The DCM must never communicate directly with MAVLink.
3. The DCM must never execute arbitrary Python, shell commands, or operating-system operations.
4. Every model action must pass through the guardrail and execution layer.
5. The simulator and physical Pixhawk must use the same Drone API contract.
6. The model must produce one structured action at a time.
7. All actions, state changes, errors, and outcomes must be logged.
8. Fine-tuning must not begin until the API and evaluation system are stable.
9. The system should be tested against unseen missions, not only training missions.
10. A larger model must not be used to compensate for an unreliable API or unclear state representation.

## Phase 0: Freeze the Initial Scope

### Objective

Define the first version of the autonomy system before implementing individual components.

### Initial DCM responsibilities

- Receive a human command or mission.
- Read the current structured drone state.
- Select one high-level action.
- Call the Drone API through the execution layer.
- Read the result of the action.
- Select the next action or finish the mission.

### Out of scope for the first version

- Raw motor control
- Low-level stabilization
- Complex perception
- Swarm behavior
- Multi-drone coordination
- Long-term memory
- Reinforcement learning
- Physical flight
- Fine-tuning

### Initial simulated missions

Start with these missions:

1. Take off and hover.
2. Fly to a coordinate.
3. Hold position.
4. Return home.
5. Land.
6. Complete a short multi-step route.
7. Cancel an active mission.
8. Recover from a failed API action.

### Phase 0 deliverables

- [ ] Written definition of the first mission set
- [ ] Written definition of the DCM's responsibilities
- [ ] Written definition of excluded functionality
- [ ] Initial architecture diagram
- [ ] Initial project directory structure

### Exit criteria

The first version of the project has a clear scope and no component is being asked to perform another component's responsibilities.

## Phase 1: Define the Drone API Contract

### Objective

Create a stable, model-independent interface between the DCM and the aircraft system.

### Initial API functions

```text
connect()
get_state()
arm()
takeoff(altitude)
goto(position)
hold()
orbit(target, radius)
land()
return_home()
cancel_action()
get_action_status(action_id)
```

Keep the initial API small. Add new functions only when a mission genuinely requires them.

### Define each function

For every function, document:

- Function name
- API version
- Required arguments
- Optional arguments
- Argument types
- Valid ranges
- Return format
- Error format
- Timeout behavior
- Cancellation behavior
- Required state conditions
- Whether the action is synchronous or asynchronous

### Example action format

```json
{
  "action": "takeoff",
  "arguments": {
    "altitude_m": 5
  }
}
```

### Example result format

```json
{
  "success": true,
  "action_id": "action_001",
  "action": "takeoff",
  "status": "completed",
  "message": "Target altitude reached",
  "state": {
    "armed": true,
    "flight_mode": "GUIDED",
    "altitude_m": 5.0
  }
}
```

### API requirements

- [ ] Inputs are strongly typed.
- [ ] Outputs are structured.
- [ ] Errors are structured.
- [ ] API functions do not expose raw MAVLink details.
- [ ] API functions can be called by scripted tests.
- [ ] API functions can be called by the future DCM.
- [ ] API versioning is defined.
- [ ] Long-running actions have action IDs.
- [ ] Long-running actions can be monitored and cancelled.

### Phase 1 deliverables

- [ ] Drone API specification
- [ ] API data models
- [ ] API error model
- [ ] API version identifier
- [ ] Initial API test cases

### Exit criteria

Every planned first-version mission can be represented using the API without exposing MAVLink details to the caller.

## Phase 2: Build the Simulator Adapter

### Objective

Connect the Drone API to ArduPilot SITL and Gazebo without involving an LLM.

### Adapter relationship

```text
Drone API
    ↓
Simulator Adapter
    ↓
ArduPilot SITL / Gazebo
```

The simulator adapter translates high-level API functions into the required MAVLink and SITL operations.

### Required simulator capabilities

- Start a simulated vehicle.
- Reset the vehicle.
- Set initial conditions.
- Receive Drone API actions.
- Execute actions through ArduPilot.
- Return structured action results.
- Return structured telemetry.
- Detect action completion.
- Detect action failure.
- Support repeatable simulation seeds.
- Record simulation time and episode identifiers.

### Scripted tests

Create tests for:

```text
test_connect()
test_get_state()
test_arm()
test_takeoff()
test_goto()
test_hold()
test_orbit()
test_land()
test_return_home()
test_cancel_action()
```

### Phase 2 deliverables

- [ ] Simulator adapter
- [ ] API-to-SITL mapping
- [ ] Simulator reset mechanism
- [ ] Repeatable initial conditions
- [ ] Structured simulator results
- [ ] Scripted mission runner

### Exit criteria

The complete initial mission set can be run through the Drone API without an LLM.

## Phase 3: Build the State Engine

### Objective

Convert raw telemetry into a concise, stable state representation that the DCM can use.

### Initial state format

```json
{
  "connection": "connected",
  "armed": true,
  "flight_mode": "GUIDED",
  "position": {
    "x": 12.5,
    "y": -4.0,
    "altitude_m": 8.0
  },
  "velocity": {
    "x": 0.0,
    "y": 0.0,
    "z": 0.0
  },
  "battery_percent": 86,
  "active_action": "goto",
  "action_status": "running",
  "last_action_result": "takeoff_completed",
  "mission_stage": "transit"
}
```

### State Engine responsibilities

- Read telemetry.
- Normalize units.
- Track current vehicle state.
- Track active actions.
- Track previous action results.
- Track mission progress.
- Emit state snapshots.
- Emit state-change events.
- Handle connection changes.
- Provide replayable state history.

Do not send every raw telemetry field to the LLM. The State Engine should expose only information required for the current decision task.

### Phase 3 deliverables

- [ ] State schema
- [ ] Telemetry-to-state mapping
- [ ] State update mechanism
- [ ] State history logger
- [ ] State replay tool
- [ ] State fixtures for tests

### Exit criteria

The same mission state can be represented consistently across simulator runs and later mapped to real Pixhawk telemetry.

## Phase 4: Build the Guardrail and Execution Layer

### Objective

Ensure that no model output can directly execute an invalid, unknown, or disallowed operation.

### Execution flow

```text
LLM output
    ↓
Parser
    ↓
Schema validation
    ↓
State precondition checks
    ↓
Action limits
    ↓
Drone API execution
    ↓
Structured result
```

### Required protections

- Function allowlisting
- Strict JSON schema validation
- Argument type validation
- Argument range validation
- State precondition checks
- One action per decision cycle
- Action timeouts
- Cancellation
- Duplicate-action detection
- API version validation
- Full decision logging
- Human approval mode
- Dry-run mode
- Action result verification
- Independent fallback behavior

### Operating modes

#### Observe mode

The DCM proposes actions, but nothing is executed.

#### Approval mode

A human approves every proposed action.

#### Assisted mode

Approved classes of actions execute automatically. Other actions require approval.

#### Autonomous simulation mode

Actions execute automatically inside the simulator.

### Phase 4 deliverables

- [ ] Action parser
- [ ] Action schema validator
- [ ] Preconditions engine
- [ ] Action limits
- [ ] Execution modes
- [ ] Decision audit log
- [ ] Rejection and error reporting

### Exit criteria

Malformed, unknown, disallowed, or invalid actions are rejected before they reach MAVLink.

## Phase 5: Build the Deterministic Autonomy Test Suite

### Objective

Prove that the Drone API, simulator adapter, State Engine, and guardrail layer work without an LLM.

### Test categories

#### Unit tests

Test individual API functions, state conversions, validators, and error handlers.

#### Integration tests

Test the path:

```text
Drone API → Simulator Adapter → ArduPilot SITL → State Engine
```

#### Scenario tests

Run complete missions from start to finish.

#### Failure tests

Test timeouts, disconnections, invalid state, failed actions, and simulator resets.

#### Replay tests

Replay recorded state/action sequences and verify deterministic outputs.

### Required scenarios

- Normal takeoff
- Normal navigation
- Normal landing
- Return home
- Repeated action requests
- Invalid arguments
- Action timeout
- Lost simulator connection
- Unexpected flight mode
- Cancelled mission
- Failed movement
- Simulator reset

### Phase 5 deliverables

- [ ] Automated test suite
- [ ] Scenario runner
- [ ] Replay runner
- [ ] Failure-injection tests
- [ ] Test result reports

### Exit criteria

The core stack passes its deterministic test suite without using an LLM.

## Phase 6: Integrate a Small Local LLM

### Objective

Test the DCM loop before fine-tuning or using a larger model.

### Model strategy

Use a small quantized Qwen model for integration testing:

1. Qwen3 1.7B for the first controller loop.
2. Qwen3 4B for a more capable local DCM.
3. Larger models only after the stack is stable.

### DCM input

The DCM should receive:

- Mission description
- Current structured state
- Previous action
- Previous action result
- Available API actions
- Output schema
- Current mission stage

### DCM output

The DCM should produce exactly one action:

```json
{
  "action": "goto",
  "arguments": {
    "x": 20,
    "y": 10,
    "altitude_m": 8
  }
}
```

### Integration modes

Start with:

1. Observe mode
2. Approval mode
3. Assisted mode
4. Autonomous simulation mode

### Phase 6 deliverables

- [ ] Local model runner
- [ ] DCM controller loop
- [ ] Prompt/state formatter
- [ ] Structured output parser
- [ ] Observe-mode integration
- [ ] Approval-mode integration
- [ ] Full decision logs

### Exit criteria

The local model can read the state, propose API actions, receive results, and continue a short simulated mission through the guarded execution layer.

## Phase 7: Build the Evaluation Harness

### Objective

Measure DCM performance consistently before and after dataset creation or fine-tuning.

### Metrics

Track:

- Mission completion rate
- Valid action rate
- Invalid action rate
- Guardrail rejection rate
- Number of actions per mission
- Mission duration
- Unnecessary action count
- Recovery success rate
- Model response latency
- Token usage
- Final vehicle state

### Model comparisons

The harness should compare:

- Scripted controller
- Base local LLM
- Fine-tuned LLM
- Larger future LLM

All models must be tested using the same missions, API version, state schema, and guardrail layer.

### Phase 7 deliverables

- [ ] Evaluation runner
- [ ] Metric calculator
- [ ] Comparison reports
- [ ] Mission result database
- [ ] Failure replay links or identifiers

### Exit criteria

You can run the same mission suite against multiple controllers and objectively compare the results.

## Phase 8: Freeze the API and Collect Data

### Objective

Generate high-quality datasets only after the API, state schema, and logging format are stable.

### Episode record

Every episode should record:

```json
{
  "episode_id": "episode_001",
  "mission": "fly to inspection area and return",
  "initial_state": {},
  "state_before_action": {},
  "previous_action": {},
  "previous_result": {},
  "model_output": {},
  "validated_action": {},
  "state_after_action": {},
  "mission_success": true,
  "simulator_seed": 42,
  "api_version": "v1",
  "model_version": "qwen3-1.7b-base"
}
```

### Dataset types

#### Dataset A: Command and mission intent

Maps natural-language commands to structured mission intent.

```text
"Fly to the inspection area and circle it twice."
        ↓
inspect_area(location="inspection_area", repetitions=2)
```

#### Dataset B: Drone API tool use

Maps an intent to a valid Drone API action.

```text
take off to 5 meters
        ↓
takeoff(altitude_m=5)
```

#### Dataset C: State-to-action trajectories

Maps mission state and previous action result to the next correct API action.

#### Dataset D: Recovery and preference data

Contains preferred and rejected actions for the same state.

### Data sources

Use:

- Scripted controllers
- Human-reviewed demonstrations
- Base-model rollouts
- Corrected failed rollouts
- Randomized simulator conditions
- Replay scenarios

Do not automatically treat every base-model output as correct training data.

### Dataset splitting

Split by complete mission or episode, not by individual decisions.

Recommended starting split:

```text
Training:   70%
Validation: 15%
Testing:    15%
```

Keep related trajectory steps together so that the test set measures genuine generalization.

### Phase 8 deliverables

- [ ] API tool-use dataset
- [ ] Mission-intent dataset
- [ ] State-to-action dataset
- [ ] Recovery/preference dataset
- [ ] Dataset validation scripts
- [ ] Dataset split manifest
- [ ] Dataset version identifier

### Exit criteria

The dataset is versioned, validated, traceable to simulator episodes, and separated into training, validation, and test sets.

## Phase 9: Fine-Tune the First Model

### Objective

Improve the DCM's command interpretation, API use, state-based decisions, and multi-step mission behavior.

### Recommended order

1. Fine-tune valid API output format.
2. Fine-tune state-to-action behavior.
3. Add multi-step mission examples.
4. Add recovery examples.
5. Evaluate on unseen missions.
6. Add preference optimization only if supervised fine-tuning is insufficient.

### Initial training method

Start with supervised fine-tuning using LoRA or QLoRA.

Do not begin with full-model fine-tuning or reinforcement learning.

### Fine-tuning input format

```json
{
  "messages": [
    {
      "role": "system",
      "content": "You are a drone controller. Return one Drone API action as JSON."
    },
    {
      "role": "user",
      "content": "Mission: take off to 5 meters. State: armed, on ground."
    },
    {
      "role": "assistant",
      "content": "{\"action\":\"takeoff\",\"arguments\":{\"altitude_m\":5}}"
    }
  ]
}
```

### Fine-tuning rules

- Keep the API schema in the training examples.
- Train on corrected behavior, not raw failures as desired output.
- Keep the guardrail layer active during evaluation.
- Do not train the model to generate raw MAVLink.
- Do not train hidden chain-of-thought as the control interface.
- Preserve a held-out test set.
- Compare against the original base model.

### Phase 9 deliverables

- [ ] Training configuration
- [ ] LoRA/QLoRA adapter
- [ ] Training logs
- [ ] Validation results
- [ ] Base-versus-fine-tuned comparison
- [ ] Model version identifier

### Exit criteria

The fine-tuned model improves measured performance on held-out missions without bypassing or weakening the guardrail layer.

## Phase 10: Stress-Test the Autonomy Stack

### Objective

Find failures that are not visible in simple demonstrations.

### Stress-test conditions

- New command wording
- New starting positions
- New routes
- New simulator seeds
- Longer missions
- Delayed API results
- API failures
- Lost connections
- Unexpected state changes
- Malformed model output
- Multiple actions in one response
- Unknown API functions
- Repeated actions
- Mission cancellation

### Required behavior

The system should:

- Reject malformed actions.
- Reject unknown functions.
- Refuse actions that violate preconditions.
- Return useful errors to the DCM.
- Recover or terminate cleanly.
- Preserve a complete audit trail.
- Reproduce failures through replay.

### Phase 10 deliverables

- [ ] Stress-test suite
- [ ] Failure-injection suite
- [ ] Replayable failure cases
- [ ] Updated guardrail rules
- [ ] Updated evaluation report

### Exit criteria

Failures are explainable, reproducible, and contained within the simulator.

## Phase 11: Hardware-in-the-Loop and Pixhawk Integration

### Objective

Move from simulated flight-controller behavior to physical Pixhawk communication without changing the DCM or Drone API contract.

### Integration order

1. Connect the Drone API to a Pixhawk on a bench.
2. Verify telemetry reception.
3. Verify connection and heartbeat handling.
4. Verify flight-mode reporting.
5. Verify command acknowledgements.
6. Verify state conversion.
7. Test with motors disabled.
8. Compare physical telemetry with simulator state.
9. Run limited, controlled operations.

The DCM should not know whether it is operating against simulation or physical hardware.

### Phase 11 deliverables

- [ ] Pixhawk adapter
- [ ] Physical telemetry mapping
- [ ] Hardware connection tests
- [ ] Simulator-versus-Pixhawk state comparison
- [ ] Bench-test report

### Exit criteria

The same Drone API and State Engine interfaces work with both the simulator adapter and Pixhawk adapter.

## Phase 12: Deploy to Jetson Thor

### Objective

Move the tested DCM and autonomy stack to the target onboard computer.

### Deployment tasks

- Quantize the selected model.
- Select an inference runtime.
- Deploy the DCM controller.
- Deploy the State Engine.
- Deploy the Drone API.
- Deploy the guardrail layer.
- Configure the MAVLink connection.
- Measure memory usage.
- Measure inference latency.
- Measure action-to-state turnaround time.
- Repeat the simulation evaluation suite on Jetson.

### Deployment constraints to measure

- Model load time
- Peak memory usage
- Inference latency
- State update latency
- Storage usage
- Thermal behavior
- Long-running stability
- Recovery after process restart

### Phase 12 deliverables

- [ ] Quantized model
- [ ] Jetson deployment package
- [ ] Runtime configuration
- [ ] Resource measurements
- [ ] Jetson evaluation report

### Exit criteria

The autonomy stack performs acceptably on Jetson under the memory, latency, and runtime constraints of the onboard system.

## Phase 13: Evaluate Larger Models and Better Hardware

### Objective

Determine whether model capacity or hardware is the actual remaining limitation.

Only move to a larger model after the following are stable:

- Drone API
- State Engine
- Guardrail layer
- Simulator adapter
- Evaluation harness
- Dataset pipeline
- Fine-tuning process
- Jetson deployment process

### Consider a larger model if

- The API is stable.
- The state representation is clear.
- Guardrails work correctly.
- The model consistently fails on reasoning tasks.
- Fine-tuning improves behavior but performance plateaus.
- Evaluation shows model capacity is the bottleneck.

Do not upgrade hardware simply because the software architecture is unfinished.

## Project Completion Criteria

The first complete autonomy-stack milestone is achieved when:

- [ ] The Drone API is versioned and documented.
- [ ] The simulator adapter executes the API correctly.
- [ ] The State Engine provides stable structured state.
- [ ] The guardrail layer rejects invalid actions.
- [ ] The DCM can complete basic simulator missions.
- [ ] All actions and state transitions are logged.
- [ ] The evaluation harness compares controllers consistently.
- [ ] Dataset generation is reproducible.
- [ ] Fine-tuning improves held-out performance.
- [ ] The same interfaces work with simulation and Pixhawk adapters.
- [ ] The stack can be deployed and measured on Jetson Thor.

## Immediate Next Actions

Begin with these tasks in order:

1. Write the first Drone API specification.
2. Define the structured action format.
3. Define the structured result format.
4. Implement the simulator adapter.
5. Implement `connect()` and `get_state()`.
6. Implement `takeoff()` and `land()`.
7. Implement `goto()` and `return_home()`.
8. Create scripted mission tests.
9. Define the State Engine schema.
10. Implement the guardrail and execution layer.
11. Add complete action and state logging.
12. Run the full stack without an LLM.
13. Add a small local Qwen model in observe mode.
14. Enable approval mode.
15. Build the evaluation harness.

Do not begin dataset collection or fine-tuning until these immediate actions are complete.

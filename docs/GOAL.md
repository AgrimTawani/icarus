# Icarus Goal

## Mission

Build a reusable, safety-bounded autonomy platform for a sensor-rich quadrotor
that can be developed in realistic simulation and transferred to physical
hardware without rewriting mission logic, model integration, safety policy,
logging or evaluation.

Icarus should let an operator express an objective in natural language, allow a
local decision-making model to choose only from approved high-level actions,
and execute those actions through deterministic software and ArduPilot. The
system must collect reproducible episodes so different base models—initially
Qwen and Llama—can be compared and later fine-tuned on flight data.

## V1 Product Outcome

A developer can clone the repository onto a supported machine, recreate the
environment, launch a named scenario, connect a selected model runtime, and run
a measured mission against either:

- the Icarus Gazebo vehicle plus ArduPilot SITL; or
- the physical Icarus vehicle plus Pixhawk through the same Drone API.

For the initial mission set, the system can safely arm, take off, hover, fly a
local waypoint or short route, return home, land, cancel an action, reject an
unsafe action and recover from one simulated failure.

## Non-Negotiable Principles

- **Flight safety is deterministic.** The model cannot bypass hard limits,
  arming checks, geofencing, collision constraints or emergency actions.
- **The model is not the flight controller.** It never commands motors or rate
  loops and never communicates directly with MAVLink.
- **Simulation and real flight share contracts.** Environment-specific details
  live behind adapters and profiles, not in mission or model logic.
- **Evidence beats demos.** Claims require repeatable commands, structured logs
  and measurable pass/fail criteria.
- **Scenarios are reproducible.** Seeds, wind, faults, vehicle revision, safety
  policy, prompts and model versions are recorded.
- **Models are replaceable.** Provider-specific code cannot leak into the Drone
  API, mission executor or simulator.
- **Data capture is designed in.** Every run can emit a synchronized episode for
  replay, evaluation and later dataset curation, subject to storage policy.
- **Portability is explicit.** Supported hosts are versioned and ultimately
  containerized; absolute machine paths are forbidden from runtime contracts.

## Vehicle and Deployment Target

The reference aircraft is a reinforced 427 mm, 10-inch X-frame quadrotor derived
from the supplied Mark4-style geometry. The design target is at most 4.5 kg
takeoff mass, Pixhawk 6X flight control, 6S propulsion, forward depth camera,
360-degree lidar, downward rangefinder, GNSS/compass and a flight-suitable
NVIDIA Jetson Thor compute assembly. The current laptop is the simulation and
development host; it is not the final onboard computer.

## Explicit Non-Goals for the Current Milestone

- letting a language model produce raw actuator commands;
- claiming obstacle avoidance before a planner and perception tests exist;
- treating a visually plausible Gazebo mesh as an aerodynamically validated
  airframe;
- full ROS 2 adoption without a demonstrated need;
- physical-flight readiness before simulation gates and hardware-in-the-loop
  checks pass;
- training directly from uncurated logs or using test scenarios as training
  data.

## Success Definition

Icarus succeeds when the same mission request and safety contract can run in
simulation and on the aircraft, failures are contained by deterministic layers,
results are reproducible from repository state, and model quality can be
compared using frozen scenarios and transparent metrics rather than anecdotes.

Phases 6 through 9 have completed the operator surface, reproducible runtime,
typed flight services and deterministic LiDAR obstacle avoidance. The immediate
objective is Phase 10: produce synchronized, immutable and replayable episode
records before any DCM is allowed to direct flight.

# Icarus Master Plan

This is the canonical high-level plan. The detailed checklist and dated phase
evidence live in [`DETAILED-EXECUTION-ROADMAP.md`](DETAILED-EXECUTION-ROADMAP.md).

## Delivery Strategy

Build from physics outward. First prove the simulator and aircraft, then make
launching reliable, then add typed control services, perception, data capture
and model reasoning. Physical integration reuses the same contracts only after
the corresponding simulation gates are measurable and stable.

| Phase | Outcome | State |
| --- | --- | --- |
| 0 | Freeze vehicle, first missions and safety limits | Complete |
| 1 | Prove official Gazebo–ArduPilot SITL foundation | Complete |
| 2 | Establish project-owned simulation workspace and frames | Complete |
| 3 | Engineer and validate the custom vehicle model | Complete |
| 4 | Connect propulsion, SITL and native simulated sensors | Complete |
| 5 | Build deterministic worlds, wind, obstacles and faults | Complete |
| 6 | Unified launch profiles, operator controls and manual test workflow | Complete |
| 7 | Reproducible build/runtime packaging and CI foundation | Complete |
| 8 | Drone API, state engine, MAVLink gateway, missions and guardrails | Planned |
| 9 | Perception, obstacle map and local planner | Planned |
| 10 | Synchronized logging, replay and dataset pipeline | Planned |
| 11 | Pluggable DCM/model runtime with structured tool use | Planned |
| 12 | Frozen scenario evaluation and model comparison | Planned |
| 13 | Hardware-in-loop and staged physical integration | Deferred |
| 14 | Dataset curation, fine-tuning experiments and regression evaluation | Deferred |

## Phase 6 — Reliable Operator Surface

Create one command surface for named profiles: GUI inspection, headless tests,
manual pilot sessions and automated missions. It must validate ports and
dependencies, present actionable startup status, expose MAVLink endpoints,
prevent duplicate launches and always clean up child processes. Add documented
manual control through a compatible GCS or joystick path without changing the
automated test path.

Exit gate: 20 consecutive start/run/stop cycles leave no stale processes or
occupied ports; invalid configurations fail before arming; both GUI manual and
headless automated profiles are documented and verified.

## Phase 7 — Portable Development Runtime

Pin external revisions, introduce a container or equivalent reproducible image,
split lightweight simulation dependencies from optional ML/fine-tuning tools,
and add CI checks for schemas, formatting, unit tests and SDF generation. GPU,
GUI and USB access remain explicit host profiles.

Exit gate: a clean supported Ubuntu host can recreate and validate the project
from repository state using documented commands, without importing hidden local
configuration.

## Phase 8 — Deterministic Flight Services

Generate code from the protobuf contracts and implement the state engine, Drone
API, guardrails, mission executor, safety supervisor and MAVLink gateway. Every
action uses IDs, deadlines, cancellation, explicit terminal states and a
telemetry snapshot. The first missions remain small and measurable.

Exit gate: all v1 missions execute through the Drone API in SITL; unsafe,
stale-state and connection-loss cases resolve to defined safe outcomes.

## Phase 9 — Perception and Local Safety

Calibrate and synchronize camera, lidar and range data; construct a local
obstacle representation; implement deterministic collision checks and a local
planner. Perception health becomes part of arming and action preconditions.

Exit gate: frozen obstacle scenarios pass clearance, latency and false-positive
thresholds, including sensor degradation and recovery cases.

## Phase 10 — Flight Records and Replay

Record synchronized state, observations, requested actions, validated actions,
safety decisions, MAVLink outcomes, model metadata and scenario ground truth.
Define an immutable episode manifest and replay tools. Raw sensor payloads are
optional and governed separately from compact metadata.

Exit gate: a run can be replayed deterministically enough to reproduce action
and safety decisions, and dataset integrity checks detect missing or misaligned
streams.

## Phase 11 — Decision and Control Model

Implement a provider-neutral model runtime for Qwen, Llama and later backends.
The DCM receives bounded state and perception summaries, emits exactly one
schema-valid high-level action, and has no direct MAVLink or shell access.

Exit gate: malformed output, timeout, hallucinated tools and unsafe requests are
rejected deterministically; switching models requires configuration, not changes
to flight services.

## Phase 12 — Evaluation Campaign

Freeze prompts, tool schemas, scenario versions, seeds and safety policy. Score
mission success, safety interventions, invalid actions, completion time,
trajectory efficiency, compute latency and resource consumption. Run repeated
trials and publish confidence intervals rather than a single best run.

Exit gate: the same harness produces an auditable comparison of at least two
base models and a no-LLM deterministic baseline.

## Phases 13–14 — Real Vehicle and Learning Loop

Progress through software-in-loop, hardware-in-loop, props-off bench tests,
tethered tests and conservative outdoor flights. Replace the simulation adapter
with Pixhawk and physical sensor adapters while retaining APIs, mission logic,
safety policy, logs and evaluation. Curate flight episodes with provenance and
train/evaluation separation before any fine-tuning experiment.

## Immediate Work Queue

1. Freeze the verified simulation and portable-runtime boundary; change it only
   for a demonstrated regression or later as-built calibration evidence.
2. Define and implement the Phase 8 Drone API, state and action contracts.
3. Put a deterministic mock client through guardrails before connecting a model.
4. Implement the state engine, MAVLink gateway and safety-supervisor boundaries.
5. Keep all later DCM actions behind the Drone API and guardrails—never MAVLink.

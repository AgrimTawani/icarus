# Direct mission agent

The live entry point is `./scripts/dcm-fly`. It uses a single Qwen 3.5 4B Q4
model selected by `config/dcm/agent-runtime.json`. The console prints the exact
model identity before connecting. An explicit `--manifest` is an operator
override; `--profile edge-vision` no longer silently selects the old 4B model.

The model's target tools use `visual_query` for visible appearance and
image-relative relationships. Compass orientation is established separately;
camera capture poses are not object positions or preconfigured waypoints.
Measured capture heading accompanies VLM queries. Python callers using the
former `description` keyword remain compatible; no mission-text parser is
involved in that argument-name translation.

## Execution

One step at a time, in `python/dcm/fly.py`:

1. Read live state, health and perception (used to bind tool schemas: registered
   target IDs, saved waypoint names, motion limits, detector classes).
2. Ask the model for one native tool call. `ask_user` pauses the conversation;
   `finish` reports the outcome. Neither is a flight command.
3. Execute that call through the typed Drone API (`ToolExecutor`), then append a
   brief result (status, message, key measured facts) and a one-line vehicle
   state to the conversation.
4. Stop after three unsuccessful calls in a row, at the decision limit, on
   `cancel`, or when the operator interrupts.

Nothing runs between steps that the model did not ask for: no automatic camera
or VLM refreshes, no batching, no completion hints, no context packing. The
conversation (system prompt, request, then each call/result) is append-only, so
llama-server reuses its prompt cache and a step only processes the newest
exchange. Model reasoning is off. If the context fills, the oldest call/result
pair is dropped; the request is always kept. Full results stay in the episode.

There is no whole-mission JSON, semantic compiler, admission or review model.
JSON Schema checks tool names, argument types and required fields. The Drone API
still owns physical limits, leases, live clearance, action preconditions,
cancellation and autopilot protections.

## Prompt and tools

`config/dcm/prompts/system.md` is the whole prompt: follow the request in order
with its exact quantities, use only requested motion, measure an object with
`localize_target` before flying to it, and finish with measured results.

The catalogue is fixed (`AGENT_TOOLS` in `agent_tools.py`): flight primitives,
`goto`, `orbit`, `approach_target`, `overfly_target`, `fly_pattern`, `detect`,
`inspect_scene`, `localize_target`, `assess_landing_zone`, `ask_user`, `finish`.
Scan and route workflows remain available to Python callers, not the agent.
Handle-bound schemas enumerate exact registered target IDs and waypoint names.
The pinned llama.cpp cannot compile a grammar for a parameterless object, so
such tools advertise one optional log-only `reason` field, dropped before dispatch.

Visual objects: `inspect_scene` question_type=condition asks the VLM a yes/no
question; its grammar makes it describe the image before giving `found`. A VLM
answer is not a measurement. `localize_target` associates the image region with
depth/LiDAR and returns a `target_id`, which `approach_target` (surface
standoff) and `orbit` (circle at standoff, nose facing it) consume.

Same-view gate (`viewpoint_gate.py`): when localization finds nothing after a
positive visual answer, dispatch refuses `inspect_scene`, `localize_target` and
visible-object flight until a movement succeeds (`VIEWPOINT_REQUIRED`). The model
chooses that movement from the request; the gate only guarantees new evidence
comes from a new view.

Coordinate orbits resolve named home/launch references from measured frames.
Horizontal target-facing yaw and camera visibility have separate contracts.
Fixed-camera calibration and target height determine geometric feasibility;
projection is not image proof or occlusion detection. Simulation acceptance
retains camera frames and capture-pose brackets during the actual orbit.
Object-relative circles enclose measured surfaces and add a reported controller
tracking reserve. The live clearance floor still applies; unseen target extent
remains unknown. This is minimum standoff, not constant distance from every wall.
Object standoff controls the target enclosure radius; it does not enlarge the
global collision-clearance floor around unrelated obstacles.

## Commands

Run each long-running service in its own terminal:

```bash
./scripts/start-sim --profile edge-vision --server-only
./scripts/start-autonomy
./scripts/dcm-fly --profile edge-vision --mode autonomous
```

The direct agent warms its own camera detector before accepting missions.
`start-autonomy --profile edge-vision` also remains valid, but starts no
duplicate continuous YOLO observer. The API launcher exits when its simulator
session ends. GPU prefill batches are bounded by the flight manifest;
CPU VLM workers are bounded separately by the perception artifact settings.
Freshness and clearance limits are unchanged. Home-frame transforms use
ArduPilot's measured `HOME_POSITION` local coordinates and are invalidated on
EKF-origin changes. Transient heartbeat transport failures retry only while
the server-confirmed lease is valid; a preempted client cannot clear another
controller's authority.

If return-home recovery fails, console exit requests landing in place through
the same Drone API. Lateral LiDAR loss cannot indefinitely cancel an active
vertical LAND; vehicle-state, control-authority and autopilot protections
remain active. Interrupted or failed sessions seal as incomplete.

Simulation is headless unless `--gui` is explicitly supplied to the launcher.
`scripts/autonomy/run_operator_acceptance.py --cases <cases.json> --output <new-directory>`
owns a separate headless evaluation stack and its camera recorder. It refuses
an already active stack and cleans up its own process identities after recovery;
it does not globally kill unrelated processes. Evaluation stops between cases
if the monitored source or configuration changes. Agent completion still needs
independent inspection of receipts and camera evidence.
Use `--mode approval` for per-tool/group confirmation. `/prompts on` prints
exact model input and tool definitions. `/hold`, `/stop`, `/land`, `/rtl`, and
`/abort` preempt planning or execution. `/quit` returns an airborne aircraft
home before releasing control; manual override remains authoritative.

## Migration and limits

The retired planner files and their compiler-specific tests were removed.
A snapshot of the pre-migration, uncommitted sources is retained in
`output/direct-agent-20261004/pre-migration-source.tar.gz`. Historical episode
artifacts are unchanged. The old offline v2 action-replay evaluator remains
available for archived data; it cannot fly or select the live architecture.
The v3 ordered-objective context and renderer have also been removed.

The initial tools support sequential flight and observation, conditions at
observation boundaries, and bounded grouping. This does not implement continuous
LLM/VLM monitoring during an uninterrupted long action or arbitrary autonomous
following. Such requests require short motion/observation steps or an actual
monitored backend tool. The agent must explain unavailable capabilities rather
than claim to have executed them. VLM descriptions remain uncertain evidence.

Validation evidence and migration measurements are stored in
`output/direct-agent-20261004`. This migration does not certify physical flight
or guarantee every natural-language interpretation by a 4B model.

The migration report is `output/direct-agent-20261004/REPORT.md`; subsequent
broader-test findings and handoff are in `output/direct-agent-broad-20261004/HANDOFF.md`.
Those migration reports contain historical prompt sizes. The current fixed
prefix must be measured with the active model's chat template and native tool
schemas, including telemetry/history separately; document word count is insufficient.
The default context is 16,384 tokens. Prompt text alone is not the full input cost.

The broader tests found that exact repetition counts, retaining every ordered
step, and explanations of unsupported physical capabilities remain unreliable
in the 4B model. It claimed ten observations after executing nine even with
counters provided; a headless conditional mission also omitted its initial
rotation while reporting completion. Do not treat its completion prose as
independent verification.

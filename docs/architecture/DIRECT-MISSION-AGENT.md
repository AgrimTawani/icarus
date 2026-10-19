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

1. Preserve the original mission and clarification answers in a conversation.
2. Fetch state, health and perception concurrently. Compact obstacle geometry
   and telemetry without injecting simulator ground truth.
3. Ask for one native tool call. `ask_user` pauses the conversation; `finish`
   communicates its final outcome. Neither is a flight command.
4. Execute the selected tool using the existing typed Drone API. A takeoff
   arms if necessary. A sequence contains two or three unconditional flight
   actions, each waited to completion; the first non-success stops the group.
5. Return the result and refresh context for the next decision. `mission_complete`
   is only a hint: the same agent sees the results and continues omitted work or
   calls `finish`. Successful actions are not an independent proof of mission intent.

There is no whole-mission JSON, semantic compiler, resolver, admission model,
review model, preflight mission graph, or automatic plan confirmation.
JSON Schema checks only tool names, argument types and required fields.
The Drone API still owns physical limits, leases, live clearance, action
preconditions, cancellation and autopilot protections.

## Prompt and tools

The four agreed files are concatenated in this order:
`system.md`, `soul.md`, `perception.md`, `drone_api.md`. There are no example
missions. Native tool definitions are generated from `agent_tools.py` and
included once in the request; their serialization adds tokens beyond the prose
prefix. New communication tools carry questions/results unambiguously.
`get_context(include_geometry=true)` requests measured obstacle bounds;
`localize_target` exposes the existing camera/LiDAR measurement service.
Neither reconstructs the removed language-grounding gate.

Native target tools use `visual_query` for an image referent; legacy Python
callers may still supply `description`. The operator's wording remains in
target identity and reporting. The vision component identifies the referred
object, then supplies an image box using the pinned model's native format;
LiDAR/depth and measured capture pose supply metric geometry. Unknown visual
answers remain unknown. Navigation metadata stays in the grounding receipt
and pilot context rather than becoming extra visual-presence constraints.

Handle-dependent tools appear after a measured target or saved workflow exists.
Their schemas enumerate exact registered IDs, avoiding transcription of opaque
handles; this does not interpret the mission or guess a missing reference.
Episodes record the actual tool catalog with each decision for reproducibility.

The state/perception context is data. Missing data remains unavailable and
timestamps are retained. Hull vertices are not sent every turn. Detailed
localization remains in perception code. Flight history drops repeated state
snapshots from model input while complete execution records remain in episodes.
An empty geometric-perception detection field is explicitly unknown camera
evidence, not proof of zero objects. The agent's `detect` tool always defaults
to a current forward-camera image; it cannot accidentally invoke the legacy
multi-camera counting window by omitting the camera argument.

Feedback includes factual successful-action counts and indexed observations.
These aid the model but do not independently verify its interpretation of the
mission. A malformed native tool response gets one concise retry before any
action is dispatched; unrelated server failures are not blindly retried.
Failed tool receipts remain unresolved until that tool later succeeds;
all attempts remain in the execution ledger. Corrected arguments may recover
an operation without an exact-argument admission rule. The
final `completed` claim cannot conceal those unresolved failures; the agent
must continue the operation or report incomplete. This is a receipt check,
not an interpretation or upfront validation of the user's mission. It does
not independently prove omitted instructions or the truth of VLM descriptions.
Current target geometry appears once in the registry view. Historical nested
localizations retain identity, timestamp, range, bearing, height and uncertainty,
and refer to that current geometry; raw historical geometry remains in episodes.
Older observation details are paged out of the decision view, not deleted.
`read_action_result` retrieves retained receipts by the execution ledger's
stable index and a JSON-data path. Large results return field/page cursors;
lookup has no flight authority and explicitly labels historical evidence.
This keeps retry chains and long scans from repeatedly injecting diagnostics
into the 16K window, without dropping the operator's request or adding a
mission compiler. Exact model token counts still guard each request.
If counting finds insufficient room, the agent repacks retained receipts once
before inference, keeping the original request and every clarification. This
does not spend an LLM decision or dispatch an action. If even that cannot fit,
the session reports the context limit without truncating the operator's text.

Angular scans own a resumable turn/observe phase. A failed capture leaves the
observation pending; it cannot advance the heading. Without a declared
predicate the workflow returns each observation for the agent's decision.
Only an explicit unconditional sweep or measured detector predicate permits
automatic continuation. Workflow state belongs to one mission; clarification
preserves it, while a new mission starts with fresh workflow state. The console
retains the previous request/outcome as reference data for follow-up requests.

Recent native exchanges preserve action order. Observation evidence appears
once in the indexed feedback, rather than again in every cumulative tool
result. Full telemetry remains in episodes; the model sees the measurements
and validity needed for decisions. Unknown battery endurance is not reported
as zero remaining flight time.

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

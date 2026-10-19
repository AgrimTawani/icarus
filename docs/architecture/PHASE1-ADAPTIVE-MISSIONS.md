> Historical architecture. Superseded by [Direct mission agent](DIRECT-MISSION-AGENT.md) on 2026-10-04. The whole-mission compiler is removed from the live path.

# Phase 1 adaptive missions — implementation and manual test handoff

## Current architecture: single interpretation (2026-10-03, validation in progress)

This section supersedes the admission/review architecture described in earlier
repair notes below. Those notes and their results remain historical evidence.

The live interpreter now emits one typed `mission-v7` plan. Separate model
admission and intent-review calls do not authorize, block, or repair live plans,
including bounded continuations. Deterministic code checks supported operations,
parameter bounds, reference types, flight state, and evidence freshness. An
operator confirms the compiled readback before execution. This confirmation
and typed validation are not proof that every possible sentence is understood.

Typed physical quantities preserve the requested units for deterministic
conversion; heading reference and direction remain explicit. Measured aircraft
and fresh obstacle summaries provide context without hidden scene identities.
Unavailable sensing remains unknown. Execution completion, perception evidence
validity, and independent ground-truth verification are reported separately.
HTTP or token-budget failures are reported as model transport failures, not as
mission capability refusals. Edge-vision V3 defaults to 8192 tokens; explicit
4096 remains available.

The live outcome's `completion_basis` is `approved_plan_execution` and
`semantic_fidelity_independently_verified` is false. The legacy machine field
`request_satisfied` remains compatible, but the console describes completion of
the approved plan; successful execution does not independently establish that
the model interpreted the original request correctly. Historical review records
remain readable without restoring a live admission or review model.

GPU offloading is generic configuration: `gpu_layers` defaults to `"auto"` and
also accepts `"all"` or a nonnegative integer; `gpu_memory_margin_mib` optionally
sets a positive memory margin. Artifact settings override manifest defaults,
and explicit runtime constructor layer overrides remain supported. Native
llama.cpp fitting adjusts offloading while the configured context size stays
explicit. Effective settings are recorded in runtime provenance. This is resource
adaptation support, not measured performance or proof of model suitability.

Mission YOLO inference now uses one pinned detector owned by the client session,
with serialized predictions and separate camera trackers. Declared detector work
is prepared after plan approval and before arming or starting the flight budget;
synthetic kernel warmup is never recorded as observation evidence. Actual camera
frames, capture timestamps and poses are still obtained during execution. Periodic
observations report missed interval opportunities without treating `max_calls`
as a promised sample count. The separate continuous observer still owns its own
model context; shared-service consolidation and measured performance remain open.

Clarification uses a typed single-slot response and preserves the rest of the
mission. Rotation direction is explicit or null (reviewed default), and the
compiled readback discloses automatic prerequisites and execution defaults.
The model's freeform summary is retained diagnostically rather than used as a
second behavior confirmation. An explicit hold is a timed hold operation using
the reviewed default when duration is omitted; it is not a no-action branch.

Observed-target localization uses the current view and bounded passive
agreement captures. It never adds search rotations after a miss. Search
maneuvers must appear explicitly in the approved mission. Existing LiDAR,
freshness, association and uncertainty checks still apply to measured targets.

Relative yaw completion checks signed net rotation, endpoint error and fresh,
settled attitude. Noise and backtracking cannot accumulate false progress.
Navigation uses measured home-relative altitude separately from EKF NED down.
Battery checks use measured simulator voltage/current/state of charge with
independent freshness; unknown power is not replaced with nominal full power.
Broken action streams are reconciled and the original action is settled before
any replacement command. Cleanup attempts all owned stages.

A configured reasoning budget uses exact native chat-token counting, reserves
mission-response space, and shares the request deadline. Oversized context
fails explicitly without silent truncation. Owned inference uses one server
slot because requests are serialized. Model artifacts and runtime controls
can be provisioned from an immutable publisher revision and verified checksum.

The current acceptance run is still in progress. The older passing counts below
must not be used as acceptance evidence for this revision. Current evidence is
indexed in [the improvement report](../../output/phase1-robust-20261003/phase1-improvement-report.md).


## 2026-10-03 follow-up repairs — known failures fixed and tested

This section supersedes the incomplete acceptance status below, which is retained
as history. Work and testing used `/home/agrim/Desktop/Icarus`. The original
episode checksum is unchanged; simulator, autonomy and local-model test processes
were stopped after the aircraft landed. Phase 2/Laya is unchanged.

The planner now uses source-attributed nodes and distinct evidence/object/surface
references; deterministic normalization builds internal requirement paths. Source
choices come from the request and confirmed clarification, preventing fabricated
quotes. Shared numeric bounds and typed references are checked before execution.
An explicit no-action branch preserves the current flight state without arming.
A separate admission step asks for missing choices and refuses unavailable
capabilities. Clarification answers preserve the original request and only accept
a suggested default when the operator confirms it.

Independent structured intent review is mandatory for live ready plans. Missing,
invalid or incomplete review blocks execution. Concrete mismatches enter bounded
repair; one clarification test exercised this by rejecting an incorrect first
proposal before accepting the corrected plan. Positive and altered-threshold
negative controls passed; model review is still not a proof of interpretation.
Count-to-presence conditions now consistently use validated numeric evidence.

LiDAR replay isolated transient facade loss during braking pitch, even after
forward velocity reached zero. Capture now waits for consecutive position,
velocity, heading and tilt settling samples; failed settling stops progress.
Measured target height and configured sensor vertical field of view determine
whether the requested final standoff remains observable. Impossible geometry is
refused before approach, with alternatives, without silently changing altitude.
No scene coordinates, phrase-specific thresholds or range calibration offsets
were added to production behavior.

| Verification | Result |
| --- | --- |
| Python unit suite | 983 tests and 249 subtests passed in 22.23 s. |
| C++ unit suites | 2/2 passed in 6.66 s. |
| Live local-model planning | 6/6: numeric branches, current-view-only approach, unsupported slow descent and two clarification variants. |
| Clarification follow-ups | 2/2: accepted forward default preserves 6 m; explicit left preserves 12 m. |
| Independent review controls | 4/4: correct trees accepted; changed count thresholds blocked. |
| Actual language-to-flight | 84.1 s: camera counted three people before arming, selected the requested branch, took off to 3 m and returned/landed/disarmed. |
| Measured approach fixture | 162.7 s: four measured approach segments, 3.13 m final range for requested 3 m (0.5 m tolerance), normal return and disarm. |
| Sensor limitation fixture | 118.3 s: requested 3 m standoff at 7 m altitude was explicitly refused as unobservable, followed by return and disarm. |
| Hygiene | Modified-code lint and whitespace checks passed; original failure-log SHA-256 preserved. |

Evidence index: `output/phase1-repair-20261003/acceptance-summary.json`.
The language-driven flight is distinct from explicit geometry execution fixtures.
The final source-attribution schema refinement was verified with ground-only
model checks and unit tests after flight. Earlier failures and test-watchdog
interruptions remain in the evidence directory; the final approach run completed
normally after correcting the watchdog allowance for return and landing.

Controlled descent-rate execution remains a separate unsupported feature; the
fix here is a clear refusal rather than a false promise to descend slowly.
Rooftop landing, other hardware profiles, long sessions and general perception
accuracy are not certified by these bounded tests. The known planning,
clarification and tracking failures are resolved within this verification scope;
complex manual Gazebo missions are the next useful acceptance step.

---

## 2026-10-03 bounded headless acceptance — partial, not all ten complete

This section supersedes the historical unit-only reports below. Tests and repairs
ran in `/home/agrim/Desktop/Icarus`, preserving the existing work and original
episode logs. The simulator and autonomy processes started for this pass were
stopped normally after landing. Phase 2/Laya remains unchanged.

Four short flight runs exercised approach, conditional execution (including a
repair retest), and operator return-home interruption. Flight case durations were
161, 57, 49, and 36 seconds, including recovery; the approach watchdog requested
safe return at 115 seconds. Ground-only camera checks took 53 seconds. Language
planning was tested separately, with at most two bounded attempts per prompt.
Explicit flight fixtures isolate executor behavior; they are not successful
natural-language mission acceptance.

| Area | Evidence and current result |
|---|---|
| 1. Baseline | Original episode SHA-256 still matches the retained fixture. All new logs preserved. |
| 2. Shared contracts | Unit regressions pass for invalid evidence, mismatched count types, scope and explicit capability denials. Live API action-list metadata now reaches planning correctly. |
| 3. Expressive intent | Live count-before-arming branch and three concurrent observations completed. Numeric language interpretation still fails requirement-path validation. |
| 4. Semantic fidelity | Final live count/current-view prompts failed closed on invalid requirement paths and references. Ambiguous “Fly 6 meters” still became a takeoff interpretation. This area is not accepted. |
| 5. Whole-plan preflight | Invalid plans and an undersized harness action budget were blocked before flight. Capability-denial regressions pass. Status and observation field combinations are now constrained by transport grammar. |
| 6. Perception | Real simulator boolean and count VLM calls completed with valid typed evidence in 27 and 25 seconds. This validates sampler execution, not general visual accuracy. |
| 7. Geometry | Building found in two views; first 3 m segment completed; cluster then disappeared and reacquisition failed. Aircraft recovered home. Physical cause remains unresolved. Current-range reporting now invalidates stale measurements on loss (unit verified). |
| 8. Surface/motion | Grounded downward-depth suitability probe passed. Slow rooftop descent remains unsupported; language planner asked an incidental clarification instead of a clean capability refusal. No rooftop landing acceptance claimed. |
| 9. Runtime/UX | Live prearm branching, concurrent observation, saved-pose return and `/rtl` interruption passed. Saved local pose no longer adds home twice or loses altitude; strengthened live retest verifies commanded coordinates. Visual percentages use execution steps separately from inference-call limits. |
| 10. Acceptance | **898 Python unit tests + 249 subtests passed in 17.01 seconds**; modified-module Ruff and whitespace checks passed. Live failures above prevent an all-complete claim. |

Generic repairs found in this pass include API capability-shape normalization,
consistent evidence eligibility, live capability denial, disjoint observation
and status schemas, bounded rejected-plan feedback, real clarification-slot
validation, saved-pose frame conversion, stale-range reporting, and visual-budget
accounting. No production scene coordinates, people thresholds, mission phrase
matching, or relaxed geometry thresholds were introduced.

Evidence: `output/phase1-short-20261003/acceptance-summary.json` indexes the final
raw reports; the directory also retains earlier failed runs and console logs.
Final reports are `planning-1790970527.json`, `perception-1790969787.json`,
`conditional-1790970248.json`, `geometry-1790969876.json`, and
`interrupt-1790970159.json`. The roof-speed harness's broad “flight blocked” pass
is only a safety result, not correct interpretation or descent support.

Remaining work is specific: make requirement/target references reliable under
live generation and clarification; retain enough cluster-association geometry
to diagnose physical target loss; implement a real vertical-rate capability
before promising controlled descent. Full roof landing, other hardware profiles,
all interrupt commands, long sessions and perception accuracy remain untested.

---

## 2026-10-03 architecture repair — unit-test acceptance only

This section supersedes the earlier capability/validation claims below. Repairs
were applied to the tested original checkout, preserving its pre-existing
uncommitted work and episode logs. No simulator, live model, flight, or end-to-end
acceptance was run. Phase 2/Laya is unchanged.

### Implementation and evidence

| Repair step | Implemented behavior | Verification boundary |
| --- | --- | --- |
| 1. Baseline | `tests/fixtures/phase1_failure_evidence.json` retains raw plans, failed observation, condition timing and approach readings with the original events SHA-256. | The logs do not establish a complete source snapshot of the previously uncommitted implementation. |
| 2. Shared contracts | `mission_contract.py` defines result/reference types and condition semantics; adaptive schema and dependency validation use it. Existing transport `ACTIONS` remains the numerical safety authority. Invalid hold/move bindings and coordinate goto/orbit bindings are rejected. | A pose, heading, tracked object and measured surface are distinct. |
| 3. Expressive intent | Typed single-frame counts, comparison operators, all/any/not, empty branches, explicit unknown policy. | Unique-person counting across frames and arbitrary numeric VLM measurement are not advertised. |
| 4. Semantic fidelity | Live planner transport requires source-attributed obligations and scalar assertions. Unattributed nodes and contradictory assertions fail validation. Semantic review checks additions and omissions; known mismatches block readiness. Clarification changes only its open field. | Attribution and a model review are not proof of language understanding. Unknown review remains disclosed/unverified. Historical replay adapters can accept explicitly labelled legacy-untraced documents. |
| 5. Whole-plan preflight | Every branch receives static type, capability, terminal/lifecycle, numerical-envelope, guaranteed-duration and observation-budget checks. Live capability metadata reaches the planner without world landmarks. | Unknown geometry and relative positions remain guarded runtime obligations. No invented inference-latency guarantee. |
| 6. Perception | Boolean/count schemas, strict decoding, explicit failure codes, capture provenance, missing-count handling and grammar/schema digests. CLI diagnostics cannot become successful evidence. | The stored grammars are regenerated with the pinned native converter in an offline unit test. |
| 7. Geometry | Compare successive measured hulls instead of treating nearest-point displacement as object movement. Preserve selected hull, cluster/time, latest rejected and last accepted ranges. | The original 6.2 m discontinuity's physical cause is unproven. Generic tests cover rotated/translated geometry and true discontinuities. |
| 8. Surface/motion | Finite measured roof geometry, fresh suitable depth and touchdown location/elevation checks. Unsupported vertical speed and descent-rate requests fail before flight. | The current transport has no descent-rate control; slow rooftop descent is not silently approximated. Assessed-surface landing remains calibrated Gazebo-only. |
| 9. Runtime/UX | Failed/duplicate/out-of-window observations do not count as completed visual work. Explicit unknown branches retain their policy; unhandled failed observations make the mission incomplete. Continuations obey graph deadlines and remaining limits; model requests serialize across abandoned waits. Readback describes branches; recovery-home achievement is recorded separately. | Existing interrupt, command-owner and C++ safety authority are retained. |
| 10. Acceptance | Full Python unit suite, generic boundary/fault tests, fake-aircraft executor tests and retained-plan regression fixtures. | These are unit tests, including mocked production execution paths, not headless simulation or live NLP acceptance. |

### Sampler failure mitigation

The pinned CLI can emit an error and still exit with code zero. Its output-format
grammar path also feeds assistant-template generation-prompt tokens into the JSON
grammar, and a catch/rethrow slices the diagnostic into `std::exception`.
Structured observations now pass the same constrained JSON grammar through the
USER `--grammar` path, for which the pinned sampler deliberately skips template
prefill. `scripts/autonomy/generate_vlm_grammars.py` uses the pinned native
converter to produce `python/perception/vlm_grammars.json`; a schema digest
prevents silent drift. Strict output decoding remains in force. This removes the
identified template-prefill failure mechanism without parsing free-form prose as
conditions. The exact original triggering token and live inference success are
not claimed to be verified.

### Generalization and remaining boundaries

There is no special people-count threshold, mission phrase parser, building
coordinate, North-facing destination, or hidden environment map in these repairs.
The former universal 0.28 m correction, derived from one simulated building, was
removed. Default correction is zero; an independently justified sensor calibration
may explicitly set `lidar.surface_range_bias_m`. No such calibration was invented.

Structural validation cannot establish that model-interpreted obligations capture
the operator's actual meaning. The planner and reviewer still require held-out
live-language acceptance before any stronger claim. Real camera latency, physical
landing dynamics, real interruption timing, different hardware profiles, and
long consecutive live sessions also remain unverified. Unsupported capabilities
are reported rather than substituted. A runtime stop is not mission completion.

Verification for this repair: **861 Python unit tests + 249 subtests passed**
in 10.24 seconds. Ruff checks on modified production modules and `git diff
--check` passed. The retained episode events SHA-256 still matches the regression
fixture. The earlier 722-test report below is historical; neither result is live
model or flight acceptance.

---

Updated 2026-10-02. **Implemented and offline-tested; new flight behavior has
not been simulation-accepted.** This development pass intentionally runs no
Gazebo/AirSim missions and no local inference models. Phase 2/Laya is untouched.

## What changed

The LLM still owns natural-language interpretation. It can now describe a small
bounded mission tree, rather than flattening every decision and destination
before takeoff. The executor does not parse the operator's wording, match a
source phrase to a number, invent a destination, or import a scenario map.

| Layer | Responsibility |
|---|---|
| Semantic planner v7 | Interpret intent, use reviewed defaults, ask about material ambiguities, choose ordered/control-flow objectives. |
| Typed contract | Validate node shapes, capabilities, finite bounds and named data dependencies. No regex/NLP replacement. |
| Operator readback | Show branches, loops, references, camera/tool schedules and bounds; require confirmation. |
| Adaptive runtime | Resolve a leaf from the live entry pose, evaluate explicit evidence conditions, run one branch, retain mission state. |
| Perception workers | Return camera-specific measurements/answers; never issue a movement or cancel command. |
| Command owner | Send guarded Drone API actions; cancel movement on a condition, deadline or operator interruption. |
| Existing control stack | C++ guardrails, local avoidance, safety supervisor and ArduPilot retain flight/stability authority. |

Existing ordinary flat missions retain the established executor. Adaptive nodes
or a relative action following runtime object navigation select the new path.
The new path reuses the same resolver, flight primitives, localization, approach,
orbit and each-side controllers. Relative coordinates are resolved **at objective
entry**, not reused from the preflight origin. Normal completion adds the
configured return home only if still airborne; exceptions enter senior recovery.

## New vocabulary

- `sequence`: ordered children.
- `repeat`: a finite count of children.
- `if`: named evidence, then/else branches, explicit unknown fallback.
- `until`: body then condition, bounded iterations and elapsed time. This is a
  do-until loop; use `if` for a precondition and `during` for concurrent observation.
- `during`: one move/orbit/turn/hold plus a camera task, interval, maximum calls
  and deadline; optional evidence condition stops the movement.
- `observe`: YOLO, VLM or downward landing geometry, a named result, explicit camera.
- `bind`: current pose/heading, or a named observation's capture pose/heading.
- `above`: measured object top and footprint, climb beside it, translate above it,
  then assess the downward surface; may save the surface as a reference.
- `land_on`: use that surface reference, reassess on arrival and monitor descent.
  A suitable named downward `observe` can also supply a surface reference.
- `replan`: one bounded semantic continuation with compact actual mission memory,
  only when a semantic decision was explicitly deferred. Normal steps and explicit
  branches do not call the LLM again.

The planner can use `{ "ref": "entry" }` as a target/center after binding a pose,
or reuse a localized object identity saved by approach/inspection. A saved
observation heading can be used by `face`. This never turns an RGB box into
metric coordinates: object navigation still requires camera/LiDAR geometry.

Example of the **machine plan**, not wording an operator must memorize:

```json
{
  "status": "ready",
  "understood_intent": "Fly north while watching below; stop on seeing a person",
  "objectives": [
    {"kind": "takeoff", "altitude_m": 5},
    {
      "kind": "during",
      "motion": {"kind": "move", "direction": "north", "distance_m": 8},
      "observation": {"kind": "observe", "save_as": "people", "tool": "yolo",
        "query": "person", "camera": "downward_brio"},
      "interval_s": 1, "max_calls": 8, "timeout_s": 30,
      "stop_when": {"evidence": "people", "test": "found", "max_age_s": 3},
      "on_unknown": "stop"
    }
  ]
}
```

If the movement finishes without that stop condition, the report says so; it
does not claim that “stop when found” succeeded. Negative detection is evidence
about a sampled view, not proof that no people exist in the area.

## Evidence and concurrency

Unknown, false and true are distinct. Missing, malformed, stale, duplicate and
out-of-order results cannot silently become a positive condition. VLM conditions
use schema-constrained JSON booleans/null, not keyword searches in prose.
Camera selection is carried to the actual capture adapter: forward D435i or
downward BRIO. Ordinary existing YOLO scan behavior remains unchanged.

Only one adaptive inference is admitted at a time. Sampling intervals are minimum
spacing, **not promised FPS**; slow VLM calls cannot be launched in parallel to
pretend to meet a schedule. A result carries capture/reception time and a pose
bracket, not the aircraft's position at the end of inference. This is not
hardware exposure synchronization. Excessive/unknown pose-bracket motion prevents
spatial binding. A result captured during movement may finish afterward while
the autopilot holds its endpoint; a frame captured only after movement ended
does not count as concurrent coverage.

Adaptive action watchers request 5 Hz status ticks. Their owner thread polls the
evidence and sends cancellation; a cancellation counts as condition completion
only after the API acknowledges establishing hold and cancelling the action.
The report retains `command_outcome: CANCELLED_BY_CONDITION`. Legacy watchers
remain transition-only. `/rtl`, `/land`, `/hold` and `/abort` keep priority.

Resource bounds: 64 tree nodes, depth 5, 128 visits, 24 iterations per loop,
128 observations, 900 seconds, one semantic continuation, plus the existing
CLI action budget and shared runtime-target budgets. These are ceilings, not
instructions to fill a mission. C++ flight/safety limits are not relaxed.

## Prompts, memory and sensors

The compact semantic system prompt describes the available actions and the
adaptive contract. The AerialClaw idea adopted here is **catalog first, load
relevant strategy only**: conditional, concurrent-camera, object-relative and
surface notes are selected from the typed plan. They are supplied only to an
actual continuation, not repeatedly injected before every motor command.
No AerialClaw `WORLD_MAP`, default building, hidden waypoint or accumulated
environment narrative is added.

Current mission memory keeps active/remaining objectives, live pose, readings,
named measured references, recent observations and outcomes. Continuation context
is a compact projection; complete evidence stays in the episode. Visual text is
evidence, not an instruction source. References start empty each mission.

The additive `DroneState.navigation_sensors` field exposes autopilot-reported
downward range, flow velocity/quality and barometric pressure. Each has its own
reception timestamp. Planner summaries mark missing/stale values unavailable.
They explicitly distinguish **height above home** from downward clearance and
report current optical-flow fusion as **unknown**: receiving flow data does not
prove that the EKF is currently fusing it. Stability remains ArduPilot's job.

## Roof landing boundary

`above`/`land_on` are calibrated **Gazebo-only** features. They require agreeing
localization, an observed (not assumed hidden) footprint, unclipped measured roof
height, a central inset column, clear guarded ascent/translation, fresh downward
depth, consistent surface elevation, limited tilt/drift and monitored descent.
Missing geometry stops the objective and invokes recovery. An ordinary `land`
does not masquerade as this assessed-surface maneuver.

The belly depth camera is explicitly a simulation-only proxy; BRIO hardware is
RGB-only. This does not certify physical rooftop landing, moving-target tracking,
AirSim object localization, or universally correct VLM judgments.

## Manual launch — four terminals, in order

Stop an older session normally first. Do not run two stacks or two mission
consoles. These commands do not start a Gazebo GUI; the camera viewer is separate.

Terminal 1 — wait for simulator readiness:

```bash
cd /home/agrim/Desktop/Icarus
./scripts/start-sim --profile edge-vision
```

Terminal 2 — wait for autonomy readiness:

```bash
cd /home/agrim/Desktop/Icarus
./scripts/start-autonomy --profile edge-vision
```

Terminal 3 — always view both camera feeds:

```bash
cd /home/agrim/Desktop/Icarus
./scripts/view-camera
```

Terminal 4 — no mission supplied; enter your own at `mission>`:

```bash
cd /home/agrim/Desktop/Icarus
./scripts/dcm-fly --runtime llama --profile edge-vision --mode autonomous \
  --context-length 8192 --timeout-ms 45000 --show-prompts
```

8192-token context is now an allowed opt-in for the edge profile; 4096 remains
the compatibility default. It uses more KV memory. The longer planner deadline
allows a structured multi-step response, not faster VLM inference. These resource
settings still need your on-machine manual check; no model was loaded in this pass.

Start with ordinary takeoff/yaw/return to verify the baseline. Then test:

1. Take off to 4 m; while moving 6 m north, detect people **below**; return.
2. Inspect the forward view; if a person is visible yaw 30° clockwise, otherwise
   yaw 30° counterclockwise; return. Check both outcomes with actual camera views.
3. Turn in finite steps until a building is found, including a not-found case.
4. Approach a building, then move 3 m left from the reached position, then return.
5. Bind the current/takeoff point and orbit that point after a relative move.
6. Interrupt a concurrent mission with `/hold`, `/rtl`, `/land`, `/abort`, one per run.
7. Try roof inspection/landing last. Missing observed roof geometry should be a
   clear refusal/recovery, never a guessed rooftop landing.
8. Run consecutive missions and a clarification/unsupported-following request.

Read the readback before `y`. Check camera/tool choices, branch outcomes,
capture times, action outcomes and final findings—not just whether it landed.
Do not assume every paraphrase or visual judgment is validated by unit tests.
After landing, `/quit` in terminal 4, then Ctrl+C in camera/autonomy/simulator
terminals closes the owned processes. Do not kill the controller while airborne.

## Verification and remaining acceptance

Offline tests cover production execution with fake aircraft, branch selection,
bounded loops, deferred relative moves, capture-heading bindings, concurrent
condition cancellation, stale/duplicate/late evidence, camera routing, malformed
VLM answers, missing roof geometry, surface consistency, partial results and
recovery. Protobuf generation and C++ builds are required after the API additions.
The existing Python and C++ suites remain regression gates.

Verification in this pass: **722 Python tests + 249 subtests passed**, including
39 new adaptive tests; protobuf generation and the `build/phase8` build passed;
**2/2 C++ suites passed**. Targeted adaptive/runtime tests and lint also passed.
No simulator flight, local model inference or GPU workload was launched.

**Still manual:** real model planning/grammar behavior, real camera inference
latency, wind/clearance performance, roof observability and landing dynamics,
all interruption commands during real movement, long consecutive runs and visual
answers compared with private simulator ground truth. Passing offline tests is
not a claim that these flight acceptance gates have passed.


### Mission console context default (2026-10-03)

The edge-vision V3 mission console now defaults to an 8192-token context to
leave space for structured mission output after the drone instructions and
measured state. Explicit `--context-length 4096` remains available for reduced
memory use, with less headroom for long requests or plans. Other profiles and
legacy V2 retain their existing 4096-token default. This changes the console
selection only, not the model manifest or global profile.

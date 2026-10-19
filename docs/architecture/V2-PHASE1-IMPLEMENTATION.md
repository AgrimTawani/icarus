> Historical architecture. Superseded by [Direct mission agent](DIRECT-MISSION-AGENT.md) on 2026-10-04. The whole-mission compiler is removed from the live path.

# V2 Phase 1 implementation and evidence


> Current status (2026-10-03): the historical “implementation complete” statements
> below do not establish complex-mission acceptance. The architecture repairs,
> unit-only verification boundary, and remaining capability limits are recorded
> in [PHASE1-ADAPTIVE-MISSIONS.md](PHASE1-ADAPTIVE-MISSIONS.md). No new live model or
> simulator acceptance was performed for those repairs.

Date: 2026-09-27

Branch: `feat/version-v2-software-architecture`

Decision: implementation complete. As of 2026-09-24, V3 is the interactive,
observe and evaluation CLI default by explicit operator decision; V2 remains
available through `--context v2` for compatibility and baseline comparisons.

## Plan-once visual scheduling correction (2026-09-30)

The operator-approved architecture no longer asks the LLM to choose every
individual action after it has already produced a valid ordered mission. One
preflight semantic-planning call now returns the intent, objectives, safe
defaults and provenance. Deterministic validation remains narrow: schema,
grounding, capabilities, bounds, lifecycle prerequisites and safety only.

Every ready mission also carries `icarus.dcm.visual-plan.v1`, which declares
visual-objective and visual-execution-step percentages, exact/maximum YOLO and
VLM calls, each trigger, and timed capture duration before takeoff.
`scan_detect` alternates a bounded
yaw with YOLO at every viewpoint; `scan_inspect` does the same with a VLM
frame. The minimal compiled-plan executor then sequences only these validated
objectives. It never calls the LLM during execution, never grows a model
context with detection history, and cannot fail at the historical 8,000-byte
V3 decision-context limit. Fresh state, typed action validation, the Drone API,
C++ guardrails, ArduPilot and the independent safety supervisor remain in the
authority path. Any executor failure stops progress and requests deterministic
RTL from raw live state when still airborne.

The exact current contract is documented in
[`DCM-OBJECTIVE-PIPELINE.md`](DCM-OBJECTIVE-PIPELINE.md). Older prompt-v9/v10
results below remain historical wiring/regression evidence; they do not prove
the plan-once runtime. Fresh live records now do: the 76-step yaw/YOLO episode
`20260930T012601_9e55edb9b9a1`, nine-step yaw/VLM episode
`20260930T012901_a30dbb3fb71d`, and zero-action clarification/unsupported
episode `20260930T013321_41d06228c593`. Both flight episodes passed complete
deterministic replay; details and limitations are in the objective-pipeline
document.

## Ordered-objective correction (2026-09-28)

Transcript review showed that prompt v8 could silently omit an unsupported
clause, then expose the whole action vocabulary after its narrow obligation
list became empty. Because a named scenario-building reference was still
present, Qwen repeatedly selected its fixed waypoints even for a
yaw-only prompt. A mission containing only a 360-degree yaw could then receive
an empty semantic requirement list and be incorrectly marked successful.

The semantic planner (`dcm-semantic-planner-v3-grounded-json`) replaces that
behavior:

- The local Qwen runtime performs semantic NLP for every V3 operator prompt,
  producing ordered typed objectives and flight/YOLO/VLM routing.
- llama.cpp schema-constrains the planner JSON envelope. Deterministic code
  validates value provenance, operator grounding, argument bounds, order,
  sensor capability and lifecycle safety. It does not reinterpret language or
  maintain command regexes for paraphrases.
- The only deterministic plan additions are the two reviewed lifecycle rules:
  arm before takeoff and safe return after an airborne mission. No destination,
  maneuver or perception objective can be synthesized outside the LLM.
- Invalid semantic plans get one feedback retry. Persistent serialization or
  validation failure is an `internal_error`, never a false clarification
  request; no deterministic fallback may substitute a different mission.
- Only the active objective's exact action/arguments may advance the plan;
  the minimal executor opens the next objective only after a fresh successful
  result and does not ask the LLM to select that action again.
- Automatic keyword macro selection is removed. Macros are explicit
  `--macro-id` overlays and cannot become a second hidden intent parser.
- Scenario landmarks and world-object names are never model-visible. Deictic
  phrases such as `the building in front` permit only current-camera
  inspection; navigation needs operator-grounded local geometry.
- `yaw` is now a typed relative action across the DCM, Drone API, C++
  guardrails/executor and ArduPilot. Physical completion is verified by
  accumulated live yaw, including a complete 360-degree turn.
- The final CLI summary lists every objective, YOLO counts, VLM answers and any
  unsupported or incomplete clause.
- `--show-prompts` and interactive `/prompts on` print the literal preflight
  semantic-planning system/user messages. Normal V3 has no later model call.

The exact contract and examples are in
[`DCM-OBJECTIVE-PIPELINE.md`](DCM-OBJECTIVE-PIPELINE.md). Generic commands such
as `go to that building` request operator-grounded local coordinates. Scenario
geometry is never supplied to either LLM stage.

## Phase-1 hardening closeout (2026-09-27)

The compact context was `dcm-prompt-v8-active-vocabulary` for the retained
2026-09-27 evidence below. The live default is now prompt v10. Deterministic
code exposes only the active objective's legal progress action or actions, plus
the bounded `none`, `hold`, `return_home` and `land` safety exits. The model
still chooses each progress action; the validator does not secretly execute a
mission script. Objective order, operator-grounded waypoints, detector classes, hover
duration and terminal completion are independently enforced after inference.

Hardening also added native absolute-heading confirmation, nearest-safe-entry
orbit generation, a three-identical-failure breaker, terminal airborne RTL,
ten-second launcher health debounce, truthfully recorded action results and
schema validation at the live decision-trace boundary. The Gazebo actor path
and count geometry were corrected. `inspect_scene` and `detect` use the mounted
forward view for this mission; YOLO may observe both cameras but the report
attributes the accepted maximum same-frame count to its actual feed. Qwen2-VL
2B Q4 runs CPU-only on this 6 GB host so the Qwen flight model, Gazebo and VLM
cannot contend for CUDA memory.

Retained live evidence from the final code path:

| Episode | Scenario and operator wording | Result |
| --- | --- | --- |
| `20260927T041123_0a62e3f27053` | Building inspection, reviewed orbit, count all visible people, hold and RTL | 9/9 actions succeeded; Qwen2-VL described the building; forward YOLO observed four people; semantic report and complete replay passed |
| `20260927T042122_6ecfa541aa24` | Historical fixed-target detect/people wording | 7/7 actions succeeded under the retired landmark-injection design; retained only as regression evidence |
| `20260927T042555_f1235131da86` | Fly above the building, hover over it for four seconds, RTL | 5/5 actions succeeded; reviewed building-hover and RTL completion passed |
| `20260927T043110_1dad8ed23b68` | Five-second hover and RTL in gusting wind | 4/4 actions succeeded; semantic completion and complete replay passed |
| `20260927T043402_1f1ad161e42a` | Twenty-second hold and RTL under the scheduled MAVLink-loss profile | 4/4 actions succeeded and complete replay passed; this particular run is resilience evidence, not a claim that the transient produced a recorded safety intervention |
| `20260928T013526_1d0dd80ce5c8` | Unseen wording: “get us up”, “swivel right”, “stay put”, “come home” | Semantic planner accepted on its first attempt; arm/takeoff/yaw/hold/RTL all succeeded with zero invalid decisions; complete replay passed |
| `20260928T013628_76cab9a03d1d` | Unseen wording for motor start, ascent, person count and building visibility | Semantic planner routed YOLO and VLM correctly; all five objectives succeeded; CLI answers and complete replay passed |

These results close the requested Phase-1 implementation and operator-flow
hardening. They do not approve unattended or physical flight and do not replace
the larger frozen promotion campaign. Aggregating exactly these five sealed
episodes produced a 1.0 recorded mission-success rate, 29 executed actions and
1.0 recovery rate for the one recoverable event. Tool-selection accuracy is
intentionally `null` because these live records have no human tool annotations;
the report does not manufacture a score from model output.

### Retired prompt-v9 live acceptance (2026-09-28)

These records used the earlier prompt-v9 path. They remain replay evidence but
do not validate the current v10 semantic-authority split. The run used a fresh headless `edge-vision` Gazebo
session, the local Qwen3 flight model, Qwen2-VL and YOLO11n. Every successful
episode below passed the strict episode-integrity and C++ guardrail replay:

| Episode | Natural-language mission | Verified result |
| --- | --- | --- |
| `20260928T213711_376851ae5458` | “Arm and fly till 5.5 meters and hover over the building in front of you.” | Historical hidden-target/grounded-repair behavior; the current pipeline instead requests coordinates and does not fly. |
| `20260928T213942_dca58510b55b` | Full 360° clockwise scan in 10° steps, checking for people after every step, then Land | All 36 physically verified yaw steps and all 36 YOLO windows completed. The forward D435i observed the scenario ground truth maximum of four people; the downward BRIO contributed frames without being misreported as a person source. One malformed model proposal was rejected before execution and the next proposal self-corrected. |
| `20260928T214459_1553b7840d38` | “Please arm, climb slowly to 5 metres, turn slowly to the right by 40 degrees, wait there for two seconds, then come home.” | The semantic planner accepted on its first attempt. “Slowly” selected the reviewed 0.5 m/s climb and 5°/s yaw command floors, both bounded by the connected profile's ArduPilot limits; Hold and RTL completed. |
| `20260928T214614_27a6ca5e86ef` | The earlier ASR-like “tape off till 2 meters” wording, then yaw, detect people and Land | Historical deterministic repair behavior. The current LLM planner must understand the typo itself or ask a clarification; the validator cannot rewrite it. |

Two faults discovered during this live pass are now regression-tested. A Goto
may not report success merely for crossing the edge of its acceptance sphere:
the executor now requires position plus horizontal/vertical velocity to remain
settled for 500 ms before opening a visual objective. The D435i RGB and depth
clips are also distinct: RGB renders to 100 m while depth remains capped at
10 m. The previous shared 10 m clip placed the building facade exactly on the
inspection boundary, so a truthful VLM sometimes saw only the nearer people.

ArduPilot exposes `ATC_RATE_WPY_MAX` as the maximum Guided/Auto yaw-target slew
rate (the parameter's documented range starts at 5°/s); it does not expose a
separate “slow mission” minimum. Icarus therefore owns the explicit 5°/s
minimum command floor and clamps the maximum to the connected profile's
`ATC_RATE_WPY_MAX`. The same profile reads `WP_SPD_UP` as the Guided vertical
speed ceiling and keeps the reviewed 0.5 m/s slow-climb choice below it. See
the [ArduPilot Copter parameter list](https://ardupilot.org/copter/docs/parameters.html)
and [Guided-mode speed documentation](https://ardupilot.org/copter/docs/ac2_guidedmode.html).

Negative evidence fails closed. Episode `20260928T210945_ee6b7d8e9099`
recorded three negative VLM answers; the aircraft never advanced to the target
waypoint and the senior recovery returned it home. A later retry that first
produced two negative answers and then an affirmative answer exposed and fixed
the no-progress breaker: it now consults the newly advanced objective before
deciding that a repeated perception action made no progress.

The current regression gate is 345 Python tests plus 168 subtests, both native
C++ suites, valid generated SDF, the Icarus7 AirSim module validator and the
executable AirLib mass/inertia/propulsion test. This remains simulation
acceptance, not permission for unattended or physical flight.

### Prompt disposition

- Retain prompt v8 evidence as historical evidence only. Prompt v9 is the
  Phase-1 interactive and simulation default and requires a fresh live
  acceptance set before any unattended promotion claim.
- Keep V2 only as an explicit historical/baseline switch; do not blend its
  repeated action/freshness prose back into V3.
- Keep SOUL, BODY and the selected soft-skill macro as deterministic context
  inputs, but render only three principles, semantic live embodiment facts and
  one compiled `MissionContract`. Full prose and provenance remain audit-only.
- Use the LLM for mission-language interpretation, but keep source grounding,
  capability/geometry validation, objective order and success scoring in
  deterministic code. The decision pass receives pending evidence and chooses
  one next action; it never decides that its own mission succeeded.
- Do not start Laya/Phase 2 from this branch. Phase 2 must compare against this
  frozen prompt and the same sealed scenario evidence.

## Delivered architecture

![Icarus V2 Phase 1 prompt-to-motors architecture](../assets/icarus-v2-dcm-to-motors-architecture.png)

The diagram follows the complete prompt path: the LLM transforms the exact
operator mission into one provenance-validated `MissionContract`; three compact
SOUL principles, semantic live BODY facts and one canonical evidence-health
summary; the resulting system/user messages reach the local DCM before its
single typed proposal crosses schema, macro,
freshness, approval and C++ authority boundaries. It also shows the separate
semantic-vision loop, independent safety override, offline feedback lane and
the distinct Gazebo, AirSim and future physical device branches. V3 is the CLI
default. The current AirSim multirotor is still not physically equivalent to
the Gazebo Akshu airframe.

- `dcm-contract-v3-compact` builds a deterministic, byte-bounded context from
  the existing curated allow-list. It adds source freshness, sectorized
  obstacles, bounded history/failure counters and a context hash without
  exposing raw sensors, truth or authority tokens.
- Model-visible context is deliberately smaller than the audit record. BODY
  exposes only semantic deployment facts; action lists, freshness limits,
  profile paths, revisions and hashes stay in sealed evidence. SOUL renders as
  three reviewed principles. Empty world-facts text is not sent to the model.
- Five YAML-front-matter mission macros have explicit action families,
  preconditions, evidence links and nominal/adverse/recovery fixtures.
- Explicit `--macro-id` remains an override; otherwise distinctive mission
  intent no longer auto-selects a macro. Explicit `--macro-id` selection keeps
  its action subset enforced.
- The validated semantic plan and completion evidence render as one
  `icarus.dcm.mission-contract.v2-objectives` object. Scenario references and
  environment facts are absent; explicit macro subsets remain enforced.
- V3 output has action, arguments and one closed `reason_code`; free-form
  rationale is no longer generated. Only validated action/arguments cross the
  control boundary.
- Decision traces are schema-validated before live recording. Episode sealing
  automatically creates hash-bound reliability and human-review artifacts;
  model hypotheses remain untrusted and cannot mutate policy.
- Runtime health has healthy/late/invalid/unavailable/stale/circuit-open states.
  Three consecutive failures open the circuit; typed senior fallback requests
  hold and then RTL through the unchanged Drone API. Normal inference is
  suppressed while open, with a bounded recovery probe.
- The final model-visible context is budgeted and hashed only after the
  validated plan and live BODY are attached. Post-inference execution additionally
  requires that the aircraft has not materially moved or changed state.
- Control-lease renewal is the independent DCM-process heartbeat. If it stops
  while airborne, the C++ safety supervisor commands BRAKE and then RTL after
  the configured delay, even if the Python process is gone.

## Compact prompt revision (2026-09-24)

The first Phase-1 prompt represented mission meaning as raw text, derived
obligations, environment data, macro prose and special-case rules simultaneously. It
also repeated action vocabulary and freshness/provenance fields. The compact
revision preserves all deterministic safety layers but removes that semantic
duplication from model input.

For the same representative AirSim building mission and live BODY fixture, the
rendered prompt changed from 8,482 to 5,026 characters. The existing
conservative byte-to-token upper estimate changed from approximately 2,828 to
1,676 tokens (41% lower). Actual tokenizer counts depend on the pinned model.
The model-visible payload itself is 2,427 bytes in that fixture, below the new
8,000-byte fail-closed ceiling. Together with the approximately 3,042-character
fixed system message, the conservative upper estimate fits the default 4,096
token runtime while leaving bounded response headroom. This measurement is a prompt-size regression
check, not an autonomy-quality claim.

The compact prompt has not inherited the earlier V3 promotion evidence. It
must be replayed and flown through the same frozen scorecard before any claim
that it improves mission performance, latency or unattended-flight readiness.

## Frozen Gazebo comparison

The same 11 sealed Gazebo episodes, Qwen3-4B Q4 artifact, history input,
5-second deadline and three repeats produced 135 decision points per contract.
No proposal was executed in this offline comparison.

| Metric | Current V2-history | V3 candidate |
| --- | ---: | ---: |
| Valid / invalid / timeout / stale | 128 / 0 / 1 / 6 | 127 / 0 / 2 / 6 |
| Tool-selection agreement | 71.9% | 71.7% |
| Guardrail rejection | 0.0% | 0.0% |
| Recovery proxy | 57.1% | 62.5% |
| Steady median latency | 676.1 ms | 1,215.9 ms |
| Peak VRAM | 4,069 MiB | 4,052 MiB |

The candidate improved recovery but doubled timeouts and exceeded the frozen
1.25x latency ratio. Three paired closed-loop Gazebo candidate/baseline runs
are also not yet present. `evaluate-dcm-v2-promotion` therefore correctly
returns `retain-v1`. That result still blocks unattended promotion, but it no
longer controls the interactive context default: V3 is selected unless the
operator explicitly requests `--context v2`.

Retained local report hashes:

- V2 evaluation: `c22d2970d42b82088dda9e542de3707f827cb5de48f7ed558c54027c52abab37`
- V3 evaluation: `2004b1d03bd68dc3af748d796bba652e7570b0f0487154ddb847608754ba60ba`
- promotion decision: `4308545b20e420c7ce44654c6501b347eeadc2fe69e260ad41c319eb4a0e8fc9`

## AirSim end-to-end proof

The pinned Blocks v1.8.0 binary now receives reviewed settings from an isolated
runtime HOME, runs `VehicleType: ArduCopter`, and exchanges lock-step dynamics
with pinned ArduPilot SITL. RGB, DepthPlanar and LiDAR are timestamp-checked.
LiDAR enters the common perception engine through a separate bounded UDP
contract and C++ body-FRD adapter. No direct movement or pose RPC exists in
Icarus's client.

A Drone API mission passed through the real path—Arm, Takeoff 2 m, Hold 3 s,
Land, disarm—in 17.96 seconds. Killing the sensor bridge made the combined
health response degraded; the next preflight failed with
`PERCEPTION_UNAVAILABLE`. Report hashes:

- flight: `4c41ec292743c54910a1ac165c87595fd6b7eff8febcedbde4898640d33c09f9`
- stale-perception refusal: `f0ad58e4358382a998362fd1bc0b4193938e3833bb8768089eaf9ad7eada165e`

AirSim remains a supplementary track. Blocks does not supply Icarus's frozen
Gazebo scenarios, seeded atmosphere or equivalent scene determinism, so this
proof does not promote it over Gazebo.

### Realistic city and semantic mission extension

The optional, digest-pinned AerialVLN Simulators v2 archive now supplies 25
external city-scale Unreal environments under CC BY-NC 4.0. Scene 6 is the
tested default because it provides a bright textured residential area and a
safe nearby low-rise target. `start-airsim-realistic` preserves the same
ArduPilot route but attaches no landmark or waypoint reference to the session.

Live AirSim RGB frames now feed the existing edge-vision boundaries. Qwen2-VL
describes the current camera view; YOLO supplies the visible-person count. Neither
has flight authority. Deterministic completion evidence is compiled into the
mission contract without choosing the next action. Reviewed macro constraints and ordinary
argument/guardrail validation refuse actions outside the authorized envelope.

The historical prompt-driven scene-6 episode
`20260923T212718_6bf94baf9b39` executed Arm, Takeoff 3 m, InspectScene, Goto the
then-hidden `(25, -6, 22)` NED/AGL waypoint, Hold 3 s, Detect person, and
ReturnHome/land. That run proved the visual/control wiring but not the current
semantic architecture; the hidden waypoint route is now removed.
Its zero-person result is **inconclusive** because the AirSim scene has no
deterministic visible-person ground truth in the current test harness.

## Reproduction

```bash
./scripts/generate-dcm-body --check
.venv/bin/python -m pytest -q tests/unit
ctest --test-dir build/phase8 --output-on-failure

./scripts/fetch-airsim-blocks
./scripts/start-airsim-stack --binary \
  third_party/airsim/blocks-v1.8.0-linux/Blocks/LinuxNoEditor/Blocks.sh
# second terminal
./scripts/start-autonomy-airsim
./scripts/run-mission --mission takeoff_hover_land --altitude 2 --hover-seconds 3

# One-time optional realistic city installation, then tested scene 6.
./scripts/fetch-airvln-environments
./scripts/start-airsim-realistic
# second terminal
./scripts/start-autonomy-airsim
# third terminal
./scripts/view-airsim-camera
# fourth terminal
./scripts/dcm-fly --runtime llama --profile edge-vision --mode autonomous \
  --context-length 4096 --max-decisions 18 --show-prompts \
  --mission "Arm, take off to 3 metres, yaw right 90 degrees, check the building, count people, then return home."
```

The evaluation and promotion commands are documented in
[`DCM-V2-FEEDBACK.md`](DCM-V2-FEEDBACK.md).

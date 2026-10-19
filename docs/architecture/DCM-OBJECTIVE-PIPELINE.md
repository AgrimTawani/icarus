> Historical architecture. Superseded by [Direct mission agent](DIRECT-MISSION-AGENT.md) on 2026-10-04. The whole-mission compiler is removed from the live path.

# DCM plan-once objective pipeline

Date: 2026-09-30

Status: implemented in the default V3 path. This document describes the code
that is executed now. V2 keeps the historical per-action model loop only for
explicit compatibility comparisons.

## Design rule

The LLM owns mission meaning. Deterministic software owns types, limits,
sequencing and safety. It does not run a second natural-language interpreter
and it does not ask the LLM to rediscover the next obvious action after every
successful step.

```text
exact operator prompt
  -> one local-LLM semantic-planning call
       -> ready
       -> needs_clarification: ask; execute nothing
       -> unsupported/unsafe: explain alternative; execute nothing
  -> typed objective plan with per-value provenance
  -> deterministic schema/grounding/capability/safety validation
  -> compile visual-inference schedule and exact maximum call count
  -> show interpretation, safe defaults, objectives and visual schedule
  -> one approval for the complete plan (approval mode)
  -> minimal compiled-plan executor
       -> read fresh state
       -> select the already-declared active objective
       -> validate exact typed action and state transition
       -> Drone API -> C++ guardrails -> ArduPilot
       -> record result and unlock the next objective only on success
  -> deterministic RTL recovery if execution ends airborne
  -> CLI mission summary, detector counts and VLM answers
```

This removes the former V3 action-selection LLM loop. A 36-view scan therefore
uses one planning inference, not one planning inference followed by 72 action
inferences. Repeated camera results stay in the sealed episode and mission
memory; they are not copied into a growing model context. The historical
8,000-byte decision-context ceiling cannot terminate normal V3 execution.

## Semantic planner

`python/dcm/semantic_planner.py` sends exactly one system/user pair before any
flight action. The system message defines:

- the multirotor mission-planner role and absence of direct flight authority;
- `ready`, `needs_clarification`, and `unsupported` outcomes;
- the typed objective vocabulary and arguments;
- safe reviewed lifecycle defaults and provenance rules;
- YOLO versus VLM routing;
- scan scheduling rules and the ban on unrequested visual calls;
- the ban on invented coordinates, landmarks or environment knowledge.

The user message contains the original operator text, reviewed default policy,
limited live flight state, motion constraints, declared perception capability,
and an explicitly empty environment context. A single repair attempt may also
contain the previous invalid response and validation error. The planner never
receives a Gazebo scenario name, building catalog, target coordinates or
simulator truth.

The LLM may safely insert only reviewed lifecycle/default values. Every value
has provenance such as `operator`, `reviewed_policy:<key>`, `live_state:<key>`
or a bounded mathematical derivation. Missing destination, geometry, target
identity or risky intent causes clarification; unsupported tracking or visual
servoing causes an unsupported result. Neither outcome can arm the aircraft.

Use `--show-prompts` or interactive `/prompts on` to print this literal
planner input and response. Normal V3 has no later action-selection prompt.

## Deterministic validation has deliberately narrow authority

The validator may:

- reject malformed JSON, unsupported objective types or out-of-range values;
- verify that operator numbers/classes occur in the complete request;
- verify reviewed-policy and live-state provenance;
- add `arm` before takeoff and `return_home` after airborne work when the
  reviewed default policy enables those lifecycle prerequisites;
- enforce capability, action count and state-transition limits.

It may not:

- reinterpret the sentence or choose a destination;
- add a maneuver or perception task;
- substitute a regex-generated mission;
- silently map a phrase to scenario geometry;
- change objective order or invent missing mission-defining values.

An operator-requested action with an otherwise valid plan is not rejected just
because the LLM serialized its source field as null; the complete operator
text is retained as the source. Numeric and class provenance are still checked
independently. Invalid plans receive one bounded repair call and then fail
closed as an internal planning error.

## Preflight visual plan

Every ready plan carries `icarus.dcm.visual-plan.v1`, defined by
`config/dcm/schema/visual-plan-v1.schema.json`. The CLI displays visual
dependency, exact and maximum calls, capture duration and each trigger before
approval or takeoff. It reports two explicit, non-subjective weights: visual
objectives as a percentage of requested objectives, and visual calls as a
percentage of compiled execution steps. It does not pretend to know a semantic
“importance” score or exact elapsed-time share before the simulator runs.

| Objective | Tool | Calls and trigger |
| --- | --- | --- |
| `detect` | YOLO | one timed window at objective start |
| `inspect_scene` | VLM | one current-camera frame at objective start |
| `scan_detect` | YOLO | one timed window after every bounded yaw step |
| `scan_inspect` | VLM | one frame after every bounded yaw step |

For a 360-degree person scan in 10-degree steps, the plan declares exactly 36
yaw actions and 36 YOLO windows before flight. For a 180-degree qualitative
inspection in 30-degree steps, it declares six yaw actions and six VLM calls.
The executor cannot add an extra call. Current visual objectives are
`report_only`: their results answer the operator but do not alter the flight
path. Conditional search-until-found, visual servoing, continuous tracking and
arbitrary perimeter surveillance remain unsupported until they receive bounded
geometry, stopping conditions and a reviewed execution primitive.

## Minimal compiled-plan executor

`python/dcm/fly.py:_execute_compiled_plan` does not interpret language and does
not call `runtime.propose`. For each active objective it:

1. reads compact fresh state and perception health;
2. derives the exact already-approved action from plan progress;
3. revalidates action schema, numeric bounds, physical state and objective
   match;
4. sends the typed request through the existing Drone API;
5. waits for the authoritative result and records a `mission_step` event;
6. advances only when matching successful evidence exists.

`scan_detect` and `scan_inspect` deterministically alternate one declared yaw
step with one declared visual checkpoint. This is sequencing, not autonomous
mission choice. Any stale state, invalid transition, failed action or Python
exception stops normal progress. If live raw state says the aircraft remains
airborne, a separate safety-recovery path requests RTL without constructing an
LLM context.

The remaining authority layers are unchanged: the Drone API and C++ guardrails
can reject every request; ArduPilot closes guidance, attitude/rate and motor
loops; its failsafes and the independent Icarus safety supervisor outrank the
DCM. The LLM never emits MAVLink or actuator values.

## Objective and perception examples

| Operator clause | Objective | Perception |
| --- | --- | --- |
| `arm` | `arm` | none |
| `take off to 5 m` | `takeoff(5)` | none |
| `yaw left 45 degrees` | `yaw(-45)` | none |
| `count people` | `detect([person])` | YOLO once |
| `scan 360 in 10 degree steps for people` | `scan_detect(...)` | YOLO 36 times |
| `inspect each view for damage` | `scan_inspect(...)` | VLM at each view |
| `check whether a building is visible` | `inspect_scene(question)` | VLM once |
| `hold for 3 seconds` | `hold(3000)` | none |
| `return home` | `return_home` | none |

`detect` and `inspect_scene` observe the current camera pose and authorize no
movement. A phrase such as `go to the building` requires a unique reviewed
location supplied by the operator/live perception system; Icarus never falls
back to a hidden scenario landmark.

## Operator-visible result and evidence

The final report states success or incompleteness, every action result, each
objective's completion, YOLO counts, VLM answers and the cause of any failure.
VLM answers are qualitative and unverified. YOLO output is detector evidence,
not simulator ground truth. Every planning attempt, validated mission plan,
visual schedule, typed step, action result and recovery request is retained in
the sealed episode.

## Compatibility and safety status

`--context v3` is the default and uses the plan-once executor for every nonempty
ready plan. `--context v2` retains the old per-action model loop for historical
comparison only. Empty V3 plans retain a legacy fixture seam for replay/unit
compatibility and are not produced by a ready live semantic plan.

This architecture reduces model authority and context growth; it does not
approve unattended or physical flight. Full frozen simulation promotion,
adverse/fault scenarios and physical-airframe validation remain required.

## Live acceptance evidence

The implementation was exercised in a fresh headless `edge-vision` Gazebo
session on 2026-09-30 with the pinned local Qwen planner, ArduPilot SITL and the
real YOLO/VLM services:

- `20260930T012601_9e55edb9b9a1`: one planning call compiled a 76-step mission
  containing Arm, Takeoff, 36 physical 10-degree yaw steps, 36 YOLO windows,
  Hold and RTL. All 76 mission steps succeeded, all 36 visual checkpoints ran,
  RTL landed, and strict episode/replay validation passed. The episode contains
  one `mission_planning`, 76 `mission_step`, 36 `semantic_detection`, and zero
  `dcm_decision` events, proving the old per-action LLM path was not used. YOLO
  reported a maximum of five people while scenario truth is four, so this run
  proves scheduling/wiring but also records one detector false positive.
- `20260930T012901_a30dbb3fb71d`: one plan compiled a 90-degree scan into three
  yaw steps and exactly three VLM checkpoints. All nine mission steps succeeded,
  the three live forward-camera questions completed, RTL landed, and strict
  replay passed. VLM latency was approximately 17.3–22.0 seconds per frame and
  all three answers were negative from the tested headings.
- `20260930T013321_41d06228c593`: in one model session, continuous person
  following returned `unsupported`, and ungrounded building navigation returned
  `needs_clarification` asking only for local north/east/altitude. The episode
  contains two planning records and no action request.

These are simulation acceptance records, not physical-flight approval or a
claim that detector/VLM answers equal simulator truth.

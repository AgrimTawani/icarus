# Version 2 research and architecture decision

Status: research complete on 2026-09-23. This is a design and experiment
plan, not permission to relax a safety boundary or enable a new model in
autonomous flight.

## Executive decision

Adopt AerialClaw's *cognitive-loop concepts* in Phase 1, but do not adopt its
control path, mutable memory behaviour, or its AirSim adapter. Icarus already
has the more important foundation: typed actions, deterministic guardrails, an
independent safety supervisor, sealed episodes and promotion gates. The V2
work should make the untrusted DCM substantially better informed and better
at learning from **measured** outcomes without changing its rank in the
authority order.

The local `laya` repository is assumed to be the "Lava" model mentioned in
the request. It is a fast, non-autoregressive typed-decision model, not a
general planner. It is promising as a calibrated, advisory System-1 assessor
alongside a DCM/LLM; it is not a replacement for the DCM and cannot construct
continuous flight arguments. Phase 2 should begin only after Phase 1 creates
the frozen labelled decision corpus required to establish Laya's calibration
on drone data.

AirSim can run without an interactive desktop and retain image APIs, but it
must be a supplementary visual/domain-randomization simulator. It must not
replace Gazebo/ArduPilot as Icarus's promotion simulator until it proves
contract parity and repeatability. `-nullrhi` is specifically unsuitable for
vision tests because it disables image rendering.

## Evidence reviewed

| Evidence | What it establishes | Important limitation |
| --- | --- | --- |
| `docs/concept/CONCEPT-AERIALCLAW-ADOPTION.md` | Ten concrete adoption ideas, file-level provenance, and an already careful comparison with Icarus | Exploratory document, not implementation authority |
| `_reference/AerialClaw` at `e01adaa` (2026-07-08) | A working LLM loop with context construction, skills, reflection and several adapters | Tests cover mock/server/repository paths; there is no end-to-end test of its LLM feedback loop or AirSim flight path |
| `_reference/laya` at `573e5b6` (2026-09-21) | A maintained SDK for `choice`, ordinal `score`, and probability (`noul`) decisions, with batching, routing and benchmark disclosures | Its reported datasets are text-classification/triage tasks, not flight control or drone perception |
| Icarus current source and 217 passing unit tests | A versioned contract, action-only control plane, current DCM live/observe implementations, episode integrity, closed-loop scoring and promotion gates | Passing unit tests do not establish that a model is ready for autonomous flight |
| AirSim and ArduPilot primary documentation | AirSim exposes multirotor, sensor, image, wind and ArduPilot-SITL integration APIs; its `NoDisplay` mode retains API images | Classic AirSim has an older UE4 toolchain and assets; a compatible environment and its licenses must be pinned and tested |

External references used for the AirSim conclusions: [AirSim settings and
NoDisplay semantics](https://microsoft.github.io/AirSim/settings/), [image
API and ComputerVision distinction](https://microsoft.github.io/AirSim/image_apis/),
[AirSim Docker headless entry point](https://microsoft.github.io/AirSim/docker_ubuntu/),
[ArduPilot SITL with AirSim](https://shop.ardupilot.org/dev/docs/sitl-with-airsim.html),
and [current ArduPilot Gazebo SITL support](https://shop.ardupilot.org/dev/docs/sitl-with-gazebo.html).

## Findings from AerialClaw

### What should be adopted

1. **Versioned planner context.** AerialClaw's perception daemon reduces state,
   obstacle direction and optional visual evidence to a small, stable planner
   summary. Icarus already curates data in `python/dcm/contract.py`; V2 should
   make the rendered bundle an explicit `dcm-context-v3` schema with field and
   token budgets. The summary must carry source time, freshness, confidence,
   nearest obstacle by bearing sector, current mission state, a bounded action
   history, deterministic failure/repetition counters and provenance hashes.
2. **Mission macros (soft skills).** AerialClaw uses Markdown strategy
   documents to compose typed hard skills. Icarus should introduce reviewed,
   versioned mission macros such as perimeter sweep, degraded-position return,
   inspection orbit and landing-site survey. A macro is advice to the DCM;
   every resolved action still passes existing validation, guardrails, mission
   execution and safety supervision.
3. **Separate identity, capability and world knowledge.** The useful pattern is
   not personality; it is keeping a hand-authored, safety-oriented `SOUL`, a
   generated factual `BODY`, and a reviewed map/hazard knowledge base separate.
   `BODY` must be generated from the active config/contracts and signed into
   the episode, never inferred by a model. World discoveries must enter a
   proposed-evidence queue, not modify an active mission document.
4. **Bounded active perception.** The existing `detect`, `inspect_scene` and
   `assess_landing_zone` tools are an excellent start. Give each an explicit
   precondition, cost/latency budget, maximum call count, freshness result and
   compact evidence output. They remain semantic-only and cannot create a
   MAVLink/control path.
5. **Measured reliability and reflection.** Capture action proposal, rationale
   class, validator verdict, guardrail verdict, execution result, recovery,
   timing and scenario identity per decision. Aggregate these deterministically
   into an action/macro reliability ledger. An LLM reflection may generate a
   hypothesis, but only deterministic episode metrics and review can alter a
   macro, world fact, threshold or deployment profile.
6. **Explicit degraded operation.** Add a deterministic DCM health state:
   `healthy`, `late`, `invalid`, `unavailable`, `stale-input`, and
   `circuit-open`. Timeout, malformed output, restart and rate-limit actions
   must be logged and lead to a configured hold/RTL/land policy owned by senior
   components, never a model's self-description.
7. **Explanation is non-authoritative.** Store a bounded `rationale` field
   separately from strict action JSON. The action parser must reject prose and
   cannot derive arguments from rationale. Rationale is an audit and training
   artifact, not safety evidence.

### What must not be copied

* Do not permit direct adapter control from Python skills. Icarus's Drone API
  and MAVLink gateway remain the only control route.
* Do not let the DCM append `WORLD_MAP`, `MEMORY`, `SKILLS` or macro documents
  during or after a flight. AerialClaw's reflection engine and soft-skill
  manager write mutable Markdown directly; that is useful for exploration but
  creates prompt drift and unreviewed policy mutation.
* Do not use keyword-triggered full documents as the only retrieval policy.
  Use mission metadata plus deterministic catalog selection, log the selected
  macro IDs and revisions, and retain a hard byte/token ceiling.
* Do not treat an LLM's stated certainty or reflection as a reliability score.
  Reliability comes from a named, held-out corpus stratified by scenario and
  failure mode.
* Do not copy AerialClaw's AirSim adapter. Its own header says normal movement
  APIs were unavailable and its implementation moves the vehicle through
  repeated `simSetVehiclePose` teleports. That invalidates dynamics, control
  latency, collision and safety conclusions.

## Phase 1 target architecture

```text
sealed scenario + current state/perception + previous action result
                              |
                              v
                    Context builder (v3, deterministic)
                              |
        +---------------------+---------------------+
        |                                           |
        v                                           v
 reviewed macro catalog (L1/L2)             BODY / reviewed world facts
        |                                           |
        +---------------------+---------------------+
                              v
              DCM planner (untrusted, deadline-bound)
                              |
                  action JSON | rationale (separate)
                              v
       schema validator -> guardrails -> executor -> safety supervisor
                              |
                              v
      sealed decision/episode evidence -> metrics -> review/promotion queue
```

The sole runtime authority change is *none*: the chain below the DCM is
unchanged. The context builder, macro resolver and evidence writer are
read-only during a flight, except for an append-only episode stream.

### Deliverables and acceptance gates

| Increment | Deliverable | Required proof before the next increment |
| --- | --- | --- |
| P1.1 Context contract | `dcm-context-v3`, renderer, schema tests, bounded sector summary and deterministic counters | Identical sealed input produces byte-identical context; excluded authority/raw-data fields cannot leak |
| P1.2 Profile facts | Generated capability manifest and static safety values; episode records hashes/revisions | Deliberate config/action mismatch is visible to the planner and test fails on stale manifest |
| P1.3 Macro library | 3–5 reviewed macros with YAML front matter (ID, revision, allowed action families, preconditions, evidence links) plus Markdown guidance | Macro cannot name undeclared action; every macro has nominal, adverse and recovery episode tests |
| P1.4 Closed-loop feedback | Decision trace schema, deterministic reliability aggregator and hypothesis/proposal queue | Model text cannot mutate live policy; report is reproducible from sealed episodes |
| P1.5 Runtime behaviour | Failure state machine, circuit breaker and explicit senior fallback | Repeated timeouts/invalid output generate no uncontrolled action and the existing safety fallback still passes |
| P1.6 Promotion experiment | Gazebo campaign compares v2 against the current DCM under identical seeds and contracts | V2 must meet the existing non-relaxable gates and improve predeclared metrics on held-out episodes; otherwise retain V1 |

Predeclare the scorecard before P1.6: mission completion, safety interventions,
guardrail-rejection rate, invalid/late rate, recovery after a failed proposal,
time-to-completion, action count, prompt size, inference latency and compute
peak. Require no safety regression and confidence intervals / repeated seeds;
do not promote based on action agreement alone.

## AirSim: recommended research track

### Answer

Yes, run a GPU-backed Unreal/AirSim binary without a visible desktop. For
visual tests, use AirSim's `ViewMode: "NoDisplay"` plus a GPU/offscreen
launch path (the official Docker wrapper supports a `headless` suffix, and
some packaged builds work with an empty `DISPLAY` or `-RenderOffScreen`).
Verify image RPC responses in the exact binary/container. Do **not** use
`-nullrhi` when camera/depth/segmentation outputs are needed: it disables
rendering, so `simGetImages` cannot provide the evidence Icarus needs.

For pure non-visual throughput tests, `-nullrhi` is potentially useful, but
it measures a different simulator modality and may not share visual timing or
load. `NoDisplay` is the appropriate comparison to current headless Gazebo
vision tests because it suppresses only the main viewport and keeps API image
rendering active.

### Integration rule

Use AirSim only behind a new **simulation adapter**, retaining this route:

```text
Icarus Drone API -> MAVLink gateway -> ArduPilot SITL <-> AirSim physics/sensors
                                                       |
                                               AirSim sensor adapter
                                                       |
                                      normalized Icarus state/perception contracts
```

Never use AirSim's direct Python `move*` API or `SimpleFlight` for Icarus
autonomy experiments. Configure an `ArduCopter` vehicle and use AirSim's
ArduPilot SITL ports; retain Icarus's MAVLink path, failsafes and hardware
semantics. AirSim truth, collision API and object labels are scoring/debug
channels only and must be impossible to include in a DCM observation.

### Time-boxed proof-of-feasibility

1. Pin an AirSim commit/release, UE binary or container digest, GPU driver,
   environment asset license, `settings.json`, ArduPilot revision/params and
   all ports in `config/dependencies`; start with a small public scene, not
   AerialClaw's non-reproducible OpenFly host.
2. Implement `start-airsim` as a separate profile with no GUI. Smoke-test RPC,
   ArduPilot connection, camera RGB/depth/LiDAR timestamps, `NoDisplay`, clean
   shutdown, and explicit failure when rendering is unavailable.
3. Write a contract-conformance suite: NED/frame conversion, units, timestamp
   freshness, sensor metadata, action lifecycle, failsafes, fault injection,
   and proof that truth does not reach curation.
4. Re-run a small, frozen subset of the existing Gazebo corpus in both engines;
   compare safety and mission metrics, not trajectories bit-for-bit. Then add
   photometric/environment tests: light/time-of-day, texture, occlusion,
   rain/fog only if the chosen environment actually supports and pins them,
   and wind through a single documented force authority.
5. Decide go/no-go using reproducibility (same seed), start-up/teardown
   reliability, image availability, GPU cost, sensor-contract fidelity and
   absence of safety regressions. Keep Gazebo as the promotion baseline unless
   all conditions pass.

AirSim may improve visual diversity, high-detail geometry, segmentation and
perception robustness. It does not by itself make dynamics "much more
realistic"; realism depends on the environment, vehicle parameters, sensor
models, controller integration and validation. Icarus's Gazebo atmosphere and
fixed-seed wind testing are currently stronger than AerialClaw's committed
simulation assets, which contain no wind/weather force model.

## Phase 2: Laya System-1 assessor

### Fit and non-fit

Laya's public interface evaluates a state against a caller-provided question
schema in one encoder pass. Its `choice`, ordinal `score`, and calibrated
binary probability outputs map well to bounded assessments; its reported T4
latencies and calibration are encouraging but must be treated as vendor
benchmarks, not Icarus evidence. It is trained around language/triage-style
typed decisions, has finite context (512/1024 depending on checkpoint), and
its own benchmark documents poor high-cardinality choice behaviour without
shortlisting. It cannot generate a safe `goto` coordinate, recover an unseen
flight strategy, or replace control policy.

Use it only as one advisory input to the DCM and senior deterministic systems:

| Candidate Laya question | Output | Allowed use |
| --- | --- | --- |
| Is current context actionable? | calibrated probability | ask DCM, hold, or acquire bounded perception; never override safety |
| Which reviewed macro applies? | small closed choice set | retrieve one macro revision for the DCM |
| Is an observation internally inconsistent/stale-looking? | probability / risk score | log and request deterministic freshness check |
| Should this proposal be escalated for approval? | risk ordinal | raise human-review requirement, never remove it |
| Is a recovery strategy needed after a rejection? | boolean/choice | choose from deterministic recovery templates |
| Which specialist runtime should receive an inspection task? | closed choice | runtime routing only |

Laya must never approve arming, bypass a guardrail, choose numeric flight
arguments, issue a command, or turn a low probability into permission to act.
For `goto`/orbit candidates, a deterministic candidate generator or existing
local planner would first produce a finite, policy-valid set; Laya may only
rank/escalate that set, while normal guardrails remain authoritative.

### Phase 2 experiment sequence

1. Freeze a Phase-1-derived labelled corpus. Labels must be measurable:
   action outcome, guardrail outcome, macro success, recovery necessity and
   human escalation—not LLM self-ratings. Split by scenario family and seed
   to prevent near-duplicate leakage.
2. Build a process-isolated `SystemOneRuntime` that accepts the same curated
   context but returns only a versioned assessment record: model/revision/hash,
   question schema hash, answer distribution, confidence, latency, device and
   fallback reason. It has no Drone API/MAVLink imports.
3. Benchmark three baselines: deterministic rules, the chosen Laya checkpoint
   and a no-Laya DCM. Measure per-question accuracy/F1 where applicable,
   Brier score, ECE/reliability diagrams, selective-risk curve, false-negative
   rate for hazardous/escalation cases, p50/p95 latency, RAM/VRAM and cold
   start. Calibration must be recalibrated on a validation split and verified
   again on held-out scenarios.
4. Shadow deploy beside Phase-1 DCM in Gazebo only. It can add evidence and
   request an already-safe hold/inspection, but not alter a flight action.
   Compare paired episodes and audit every disagreement.
5. Consider limited assistance only if it reduces a predeclared operational
   error with no safety regression, stays within the deadline under worst-case
   load, and the promotion gate accepts the full combined system. Otherwise
   retain it as offline triage/research tooling.

## Recommended implementation order

1. Land P1.1 and its contract tests before adding prompts or macros.
2. Land P1.2/P1.3 as reviewed data artifacts and resolver tests.
3. Land P1.4/P1.5 with episode schema migration and failure tests.
4. Run the P1.6 Gazebo comparison and make a written promote/no-promote
   decision.
5. In parallel but isolated, execute the AirSim feasibility proof; it must not
   block Phase-1 cognition work or replace its Gazebo evidence.
6. Only then create the Laya corpus and run the Phase-2 shadow experiment.

This ordering turns AerialClaw's useful feedback loop into an Icarus feedback
loop: observe -> seal -> measure -> review -> promote, rather than observe ->
model reflects -> production knowledge changes.

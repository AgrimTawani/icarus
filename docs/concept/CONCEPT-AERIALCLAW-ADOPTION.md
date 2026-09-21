# Concept: Capabilities Worth Adopting from AerialClaw

Status: exploratory. Not implementation authority — see
[`docs/README.md`](../README.md) source-of-truth rules. Nothing in this
document describes Icarus's current implementation; it describes concepts to
evaluate for the DCM layer once Phase 11/12 reach the point of granting a
model live authority.

## Introduction

AerialClaw (XDEI-Group) is a public, working LLM-drone-agent framework that
has already closed the loop this project has deliberately not closed yet: it
lets an LLM perceive, decide and act on a real control loop, in simulation,
today. Icarus's own architecture is stricter and more defensible for the
stated goal — a safety-bounded platform meant to eventually fly a real 4.5 kg
aircraft — but strictness alone does not produce a good planner. AerialClaw
earned several concrete lessons about what a live agent loop actually needs
by building one and iterating on it in the open. Those lessons are worth
harvesting deliberately, before Icarus writes its own DCM-to-flight splice,
rather than rediscovering them after a bad decision in the field.

This document extracts ten specific mechanisms from AerialClaw's codebase,
explains the underlying problem each one solves, and describes how the
*concept* — not the code — should be adapted for Icarus. Every adaptation is
written to be additive to the existing authority model
(`docs/architecture/SOFTWARE-ARCHITECTURE.md`): guardrails, the safety
supervisor and ArduPilot's own failsafes remain strictly senior to anything
described here. Nothing below proposes giving the DCM new authority; it
proposes giving the DCM better-designed inputs, better-designed outputs, and
better-designed feedback loops within the authority it will eventually be
granted.

Each section cites the exact AerialClaw file(s) and mechanism it is drawn
from, so the concept can be checked against the source rather than taken on
faith.

## Table of Contents

1. [Frozen Perception & Context Contract for the Planner](#1-frozen-perception--context-contract-for-the-planner)
2. [Mission-Macro / Soft-Skill Composition Layer](#2-mission-macro--soft-skill-composition-layer)
3. [Persistent, Model-Legible World & Capability Documents](#3-persistent-model-legible-world--capability-documents)
4. [Active Perception as a Bounded, Advisory DCM Tool](#4-active-perception-as-a-bounded-advisory-dcm-tool)
5. [Operational Skill Reliability Ledger](#5-operational-skill-reliability-ledger)
6. [Graduated Autonomy as a Runtime Confirmation Dial](#6-graduated-autonomy-as-a-runtime-confirmation-dial)
7. [Graceful Degradation & Failure-Honesty Contract](#7-graceful-degradation--failure-honesty-contract)
8. [Explain/Act Channel Separation and Hallucination Guarding](#8-explainact-channel-separation-and-hallucination-guarding)
9. [Decision Rationale Capture as a First-Class Artifact](#9-decision-rationale-capture-as-a-first-class-artifact)
10. [Reflection-to-Evidence Loop](#10-reflection-to-evidence-loop)
11. [Simulation Environments, Terrain & Weather/Wind Modeling](#11-simulation-environments-terrain--weatherwind-modeling)
12. [Synthesis: How These Interlock](#12-synthesis-how-these-interlock)

---

## 1. Frozen Perception & Context Contract for the Planner

### What AerialClaw does

AerialClaw never hands the model raw telemetry or raw sensor frames. Every
decision point is built from a small number of deliberately-designed, layered
text summaries:

- `perception/daemon.py` (`PerceptionDaemon`) runs a background thread on a
  fixed interval (default 3s) and produces a single combined string capped at
  roughly 100 tokens: a one-line state summary (`"高度5.0m, 电池87%, 悬停中,
  位置NED(...)"`), a LiDAR-derived obstacle line that buckets the full 360°
  scan into 8 compass sectors and reports only the *nearest* obstacle per
  sector (`"障碍物: 前方24.5m[视觉+雷达], 左方18.2m[仅雷达]"`), and an optional
  VLM line injected asynchronously by `vlm_analyzer.py`. Sectors are also
  cross-referenced against camera field-of-view so the model knows whether a
  reported obstacle is corroborated visually or LiDAR-only.
- `brain/planner_agent.py`'s `build_system_prompt()` assembles this into a
  strictly layered prompt: identity (SOUL/BODY) → experience (MEMORY/SKILLS,
  optionally vector-retrieved via `memory_manager.get_context_for_planning()`)
  → live perception → world map → a **compact L1 skill table** (~200 tokens)
  → current state → output-format rules. A second stage
  (`plan()`'s `two_stage=True` path) only loads the *full* `SKILL.md` detail
  for the specific skills the model already chose in stage 1, and asks it to
  revise — full skill docs never enter the prompt speculatively.
- `brain/agent_loop.py`'s `_build_iteration_prompt()` additionally includes
  bounded execution history (not the full trace — only recent steps),
  explicit consecutive-failure/repetition counters computed in code (not left
  for the model to notice), and up to 3 vector-retrieved "similar past
  experiences" filtered by a minimum similarity score.

The throughline: every input the model receives is a **deliberately
engineered, size-capped, code-constructed artifact** — never a raw dump of
whatever data happens to be available.

### Why it matters

An LLM's decision quality is bounded by prompt quality as much as model
quality. A planner fed raw, unbounded telemetry either drowns in irrelevant
detail (increasing latency and cost) or misses the one obstacle sector that
matters. AerialClaw treats "what does the model see at decision time" as a
first-class engineering artifact with its own module, its own token budget,
and its own construction logic — not an afterthought of "just pass the
state."

### What Icarus should add

Icarus already has the right instinct in its curated-state philosophy
(`docs/architecture/SOFTWARE-ARCHITECTURE.md`'s "DCM sees a curated state")
and its evaluation harness depends on that input being reproducible. What's
missing is treating the **shape** of that curated bundle as a versioned,
designed contract with the same rigor AerialClaw applies to its perception
daemon:

- A fixed, documented token/field budget for the state-and-perception summary
  the DCM receives at each decision point, with an explicit sectorized or
  bucketed representation of LiDAR/obstacle data (not a raw point cloud or
  raw range array) — the same "compress to compass sectors, keep only the
  nearest hit per sector" idea, adapted to Icarus's obstacle map.
- Failure/repetition signals computed deterministically in code and injected
  into the prompt as explicit facts (e.g. "3 consecutive rejected actions of
  this type"), rather than relying on the model to infer patterns from a raw
  history list — this is cheap, deterministic, and removes an entire class of
  planner failure.
- A staged/tiered skill-and-action disclosure: a compact catalog of the
  currently legal actions first, full parameter/precondition detail only for
  actions the model is actively considering — keeping the steady-state prompt
  small without ever hiding what's legal.
- Because Icarus's evaluation harness (`python/dcm/evaluate.py`,
  `compare-dcm`) already requires byte-identical inputs across model
  candidates for a fair comparison, this contract should be versioned exactly
  like `DCM-OBSERVE-V1.md` already implies — a schema change is a contract
  version bump, not a silent prompt edit.

---

## 2. Mission-Macro / Soft-Skill Composition Layer

### What AerialClaw does

AerialClaw draws a hard line between two layers, implemented very
differently:

- **Hard skills** are Python classes with typed schemas
  (`skills/motor_skills.py`, `perception_skills.py`, etc.), registered in
  `skills/registry.py`. Each hard skill has a fixed input/output schema and a
  fixed doc.
- **Soft skills** (`skills/soft_skill_manager.py`) are *not code at all* —
  they are Markdown strategy documents living in `skills/soft_docs/`
  (`search_target.md`, `rescue_person.md`, `patrol_area.md`, etc.), each with
  a `## 概述` (overview) section, a recommended strategy, key parameters, and
  a `## 历史经验` (historical experience) section that the system appends to
  over time. `SoftSkillManager` exposes them to the model two ways: a
  one-line summary table always present in every prompt, and, in
  `agent_loop.py`'s `run()`, a **keyword-triggered full injection** — if the
  operator's goal contains "救" or "rescue"-family keywords, the *entire*
  `rescue_person.md` document is injected into the system prompt for that run
  only, giving the model a full recommended procedure without bloating every
  other prompt.
- Crucially, a soft skill is not just a menu entry — it composes the model's
  own reasoning. The model reads the doc and *decides itself* how to sequence
  hard-skill calls to match the recommended strategy; the soft-skill layer
  never bypasses hard-skill validation.
- New soft skills are **synthesized automatically**: `skills/dynamic_skill_gen.py`'s
  `detect_patterns()` scans a rolling window of past skill chains
  (`data/skill_chains.json`) for any exact skill sequence (length ≥ 2) that
  repeats at least `min_count` times, computes its average duration and
  success rate, and — if `generate_soft_skill_doc()` is called with that
  pattern — asks the LLM to write a new Markdown strategy doc for it, which
  `agent_loop.py._update_memory()` then writes to disk via
  `SoftSkillManager.create_skill()`. A parallel retirement mechanism
  (`get_retirement_candidates()` / `retire_skills()`) removes soft skills that
  are too short to be real, unused for `max_age_days`, or rated `poor` three
  times running.

### Why it matters

Without this layer, every "search this area" or "approach and assess a
person" request forces the model to re-derive an entire multi-step procedure
from first principles, every single time, with no guarantee of consistency
between two runs of what should be the same well-understood behavior. The
soft-skill layer gives the model a library of **named, inspectable,
improvable procedures** that sit strictly above the validated action layer —
composition knowledge, not new authority.

### What Icarus should add

This is the single cleanest, lowest-risk concept to adopt, because it adds
nothing to the trust boundary: a soft skill only ever resolves to a sequence
of already-guardrailed, already-validated typed actions. The concept for
Icarus:

- A library of **mission macros** — versioned documents (not code) describing
  a recommended composition of typed Drone API actions for a named,
  recurring mission shape (e.g. "perimeter sweep," "return-with-degraded-GPS,"
  "landing-site survey"). The DCM reads the macro relevant to the current
  mission intent and uses it to structure its sequence of proposed actions;
  guardrails and the safety supervisor validate every individual action
  exactly as they would without a macro.
- Because Icarus's evaluation methodology already treats scenarios as frozen,
  versioned artifacts, a mission macro is a natural sibling artifact type: a
  macro's recommended procedure could itself be scored against the frozen
  scenario corpus the same way a model candidate is, giving you an objective
  answer to "does following this macro actually improve mission success" —
  something AerialClaw has no equivalent mechanism to verify.
- The auto-synthesis idea (promote a repeated, successful action sequence
  into a named macro) is valuable but should be **evidence-gated** in
  Icarus's style rather than LLM-judged in AerialClaw's style: a candidate
  macro synthesized from repeated successful episodes should be treated as a
  proposed regression-corpus addition requiring the same promotion scrutiny
  as a model candidate, not something silently written to disk after one
  reflection pass.
- The retirement concept — pruning macros that are unused or that correlate
  with poor outcomes — maps directly onto Icarus's existing regression/
  promotion discipline and needs no new mechanism, only extending "what gets
  reviewed at a promotion gate" to include macros, not just models.

---

## 3. Persistent, Model-Legible World & Capability Documents

### What AerialClaw does

Three Markdown documents live in `robot_profile/` and are read into *every*
prompt regardless of mode (`agent_loop.py`, `planner_agent.py`,
`chat_mode.py` all read them independently):

- **`SOUL.md`** — static, hand-authored personality/values/communication
  style (e.g. "safety first: proactively suggest retreat on low battery or
  dangerous environment," "honest: say 'I'm not sure' rather than fabricate
  an answer"). Never machine-written.
- **`BODY.md`** — regenerated at every startup by
  `robot_profile/body_generator.py`'s `generate_body_md()`, which
  *introspects the live system* rather than being hand-maintained: it queries
  the connected `SimAdapter` for its name/description/supported vehicle
  types/connection state, queries the live `sensor_bridge` for actual camera
  count/resolution/FOV and actual LiDAR scan range/angular resolution, and
  queries the live `SkillRegistry` for the actual currently-registered hard
  skills with their real parameter schemas. The file explicitly states "本文件由系统启动时自动生成，请勿手动编辑" (auto-generated, do not hand-edit) —
  the model is always reading a **ground-truth reflection of what's actually
  connected**, not a stale spec.
- **`WORLD_MAP.md`** — hand-authored *and* model-extended geographic/
  environmental knowledge for the current operating area: known landmarks
  with NED coordinates, distance/height annotations, hazard notes ("超高层建
  筑最高477m, 保持安全距离"), and a recommended default patrol route. The
  observed example includes explicit safety guidance baked into the
  geography doc itself ("起飞点周围40m内有低层建筑, 起飞后立刻升高到20m以上").
  Below the seeded content there's an `## 探索发现` (exploration findings)
  section the running agent appends discoveries to via the `update_map` hard
  skill, growing the document across missions.

Separately (referenced by `agent_loop.py` but not shown in the files
inspected), a `MEMORY.md` and `SKILLS.md` pair serve as the durable record of
task lessons and measured per-skill performance respectively — `SKILLS.md`
entries are populated with *actual measured* success rates and average
timings from `skill_memory`/`reflection_engine`, not asserted by the model.

### Why it matters

This solves two different problems that are easy to conflate but shouldn't
be: (1) keeping the model's self-description **synchronized with reality**
as hardware/config changes (`BODY.md`'s introspective regeneration), and (2)
letting the vehicle **accumulate genuine operating-environment knowledge**
across missions instead of starting from zero every flight (`WORLD_MAP.md`'s
append-only exploration section). Neither is memory in the "remember what I
said" sense — both are durable, inspectable, versionable documents that
happen to be readable by both humans and the model.

### What Icarus should add

Icarus's safety envelope today is static declared config
(`config/safety/v1.yaml`); nothing in the architecture accumulates *measured*
operating knowledge the way `WORLD_MAP.md`/`SKILLS.md` do. Two distinct
concepts worth separating cleanly, following Icarus's own evidence discipline
rather than AerialClaw's LLM-asserted-then-trusted approach:

- **An introspected, regenerated capability profile** analogous to
  `BODY.md`: at startup (or per deployment target — dev workstation vs.
  aircraft companion computer), a profile documenting what's actually
  connected — which adapter, which sensors are live, which typed actions are
  currently registered with their real parameter schemas — built by querying
  the live services (state engine, drone API, perception) rather than
  hand-maintained. This directly prevents a DCM prompt from claiming
  capabilities that don't exist on the currently-connected vehicle
  configuration, which matters more for Icarus than AerialClaw given the
  explicit sim-vs-physical parity goal (`ADR-0002`).
- **A persistent, versioned world/environment knowledge base** that
  accumulates *only from validated evidence* — episode outcomes, replayed and
  passed through the same guardrail-replay integrity check Phase 10 already
  performs — never from a model's unverified assertion about the world. This
  is the natural extension of the landing-zone assessment work already
  underway: a validated "this depth source, this geometry, this location was
  assessed suitable on this date" fact is exactly the kind of durable,
  evidence-backed world knowledge this concept generalizes to hazards,
  known-good staging areas, or measured-vs-declared performance margins
  (e.g. "under gust conditions matching scenario X, effective safe speed was
  measured lower than the declared envelope"). The critical divergence from
  AerialClaw: entries are written only by the evidence/evaluation pipeline,
  never by the DCM narrating a belief about itself or the world.

---

## 4. Active Perception as a Bounded, Advisory DCM Tool

### What AerialClaw does

Perception is explicitly two-layered, and the layers have different trust
levels and different triggers:

- **Passive** (`perception/daemon.py`): cheap, continuous, always-on,
  produces the ~100-token summary described in Section 1. No VLM call is made
  in this path by default — it's built from LiDAR ranges and adapter state
  only. `passive_perception.py` (a second, higher-cost variant used inside
  the AirSim/agent-loop stack) *does* run a VLM call on a fixed cadence
  (default every 5s, only while flying) but keeps its prompt deliberately
  tiny (`PASSIVE_PROMPT`, target ~100 tokens) and writes results into
  `WorldModel`, deduplicating new obstacles against existing ones by
  direction+type+distance before appending.
- **Active** (`perception/vlm_analyzer.py`'s `VLMAnalyzer`, and
  `passive_perception.py`'s `perceive_active()`): a deliberately
  request-scoped, on-demand deep analysis the model invokes explicitly
  (via the `observe`/`perceive` hard skill) with a *specific, model-authored
  focus question passed as `focus=` — e.g. "windows show cracks?" — rather
  than a generic "describe everything." Three purpose-built prompt templates
  exist (`analyze_environment`, `search_target`, `evaluate_navigation`) with
  fixed output JSON schemas, so results are always structurally predictable
  regardless of what the model asked. `VLMAnalyzer` tracks its own call
  count/average latency for cost/performance visibility and retries on
  502/503 with exponential backoff.
- The result of an active call is explicitly **advisory** — it updates
  `WorldModel`/the prompt context for the *next* decision, but the VLM never
  has a code path to an actuator; only the DCM-equivalent (`agent_loop`)
  chooses what to do with the new information, and that choice still goes
  through the normal skill-execution/safety-policy path.

### Why it matters

This is precisely the shape of problem Icarus is already sitting on with its
own uncommitted `setup-vlm-runtime` work: how do you let a model ask a
semantic vision question ("is that a person," "is this surface clear") when
the whole architecture's premise is that vision-language output is
unreliable, without either (a) granting it flight authority it hasn't earned,
or (b) leaving it permanently unused. AerialClaw's answer is: keep it
narrow, request-scoped, schema-constrained, and advisory-only, and always
feed its output back through the same decision layer as everything else
rather than special-casing it.

### What Icarus should add

Icarus's own `setup-vlm-runtime` header already states the right constraint
("not wired into DCM action contract... no flight authority") — this section
formalizes *how* to wire it in later without violating that constraint,
mirroring AerialClaw's proven split:

- Keep deterministic passive perception (LiDAR obstacle map, calibrated
  depth-based landing assessment) as the always-on, flight-relevant layer —
  this is already stronger than AerialClaw's passive layer and should stay
  that way.
- When the VLM is eventually authorized, expose it only as a narrow,
  on-demand **advisory query tool** the DCM can invoke with a specific,
  bounded question and a fixed output schema (e.g. a classification with a
  confidence field), analogous to `analyze_environment`/`search_target`'s
  fixed-template approach rather than open-ended captioning. Its output
  should be typed and validated the same way any DCM-facing perception
  summary is — never free text injected raw into the next decision prompt.
  This is also directly useful groundwork for the existing
  landing-zone-assessment boundary: a semantic "does this look like a hazard"
  query could someday *complement* (never replace) the geometric coverage/
  slope/roughness check, exactly the same relationship AerialClaw draws
  between deterministic LiDAR ranges and VLM commentary.
  The VLM's output should never itself gate an action — it should only ever
  become one more field in the bounded state/perception summary defined in
  Section 1, subject to the same guardrail evaluation as anything else the
  DCM sees.
- Any future VLM-authorization decision belongs in the same promotion-gate
  discipline already governing DCM models (Section 10) — an explicit,
  evidence-based "this advisory tool is reliable enough to inform decisions"
  gate, not an implicit one earned by simply existing in the codebase.

---

## 5. Operational Skill Reliability Ledger

### What AerialClaw does

Two related but distinct mechanisms track skill performance, at different
timescales:

- **`memory/skill_memory.py`**'s `SkillMemory` is a live, in-process ledger:
  every skill execution reports `{skill, robot, success, cost_time}` via
  `update_skill_statistics()`, which maintains both a **global** and a
  **per-robot** `SkillStats` (`total_executions`, `success_count`,
  `total_cost_time`, with `success_rate`/`average_cost_time` computed as
  properties). `get_skill_reliability()` and `get_all_skill_reliabilities()`
  expose this directly for consumption — `reflection_engine.py`'s
  `build_reflection_prompt()` explicitly includes this table
  (`## 技能历史统计`) in every reflection call, so the LLM doing the
  reflecting sees hard numbers, not just the current episode. There's also a
  `get_best_robot_for_skill()` lookup, forward-looking for the swarm case.
- **`memory/skill_evolution.py`**'s `SkillEvolution` operates on a longer
  timescale over the qualitative feedback the reflection engine produces per
  skill (`good`/`acceptable`/`poor` plus a free-text suggestion and
  recommended parameters), persisted to
  `data/skill_evolution/evolution_history.json`. It computes two specific,
  concrete signals: `get_degraded_skills()` flags any skill whose recent
  window (default last 5 reviews) has a `poor` rate above a threshold
  (default 40%), and `get_param_drift()` detects when a skill's
  LLM-recommended parameters keep changing between reflections (evidence the
  model hasn't converged on a stable understanding of how to call it).
- Both feed forward into the live prompt: `agent_loop.py` reads `SKILLS.md`
  (itself updated by `reflection_engine.update_skills()`, which merges
  `skill_feedback` with the measured `stats_map`) directly into the system
  prompt every run, so a skill's *actual, measured, running* reliability is
  visible to the model *before* it decides to invoke it — not just used
  retrospectively to grade past decisions.

### Why it matters

This is the crucial difference from a purely offline evaluation harness:
AerialClaw's evaluation is not just "did model X do well on the frozen
scenario corpus" — it's a live, continuously-updated record of "how has
*this specific action*, on *this specific vehicle*, actually performed in
real operation," visible to the decision-maker at decision time. A model
that's passed offline promotion can still discover in the field that one
particular action has degraded (worn actuator, changed environment,
firmware regression) — a static promotion gate alone can't catch that.

### What Icarus should add

Icarus's evaluation harness (`evaluate-dcm`, `compare-dcm`) is a **pre-flight
gate**: is this model good enough to be trusted, checked against a frozen
corpus, before granting it any live authority. That is a different question
from **is this specific action, on this specific hardware, currently
trustworthy** — and Icarus has no mechanism for the second question. The
concept to add:

- A live, per-action reliability ledger, analogous to `SkillMemory`, fed by
  real mission outcomes (both simulated and eventually physical), tracking
  success rate and timing per typed action — global and per-vehicle-instance,
  mirroring the per-robot dimension AerialClaw already models for its future
  swarm case.
- Exposed as part of the bounded state/perception contract (Section 1) so a
  DCM decision at any given moment is informed by "how has `land` actually
  performed on this vehicle recently," not just by what the model was told
  about `land` at promotion time.
- A degradation-detection signal (mirroring `get_degraded_skills()`) that can
  independently trigger operator attention or tighten the confirmation dial
  (Section 6) for a specific action *without* revoking a model's overall
  promotion status — this is a much finer-grained safety lever than an
  all-or-nothing model promotion/demotion.
- Because Icarus already captures sealed, replayable episodes (Phase 10),
  this ledger is close to a pure derived-data problem: it can be computed
  from the existing episode record rather than requiring a new capture
  mechanism, which is a meaningfully lower-risk starting point than
  AerialClaw's live in-process accumulation.

---

## 6. Graduated Autonomy as a Runtime Confirmation Dial

### What AerialClaw does

`config/safety/safety_config.yaml`-equivalent (`config/safety_config.yaml`)
defines three named tiers — `strict` / `standard` / `permissive` — each with
its own `auto` (execute without confirmation) / `confirm` (pause for a human
click) / `deny` (hard-blocked regardless of tier) action lists. `strict`
requires confirmation for nearly every motion action including `hover`;
`standard` auto-approves perception/status actions but still confirms
`takeoff`/`land`/`fly_to`/etc.; `permissive` auto-approves everything except
`velocity_control`. The tier is a single config value
(`safety_level: standard`), switchable by restarting the service, and it's
explicitly separate from — layered *underneath* — the numeric `flight_envelope`
block, which is described as hardcoded and not reachable by the model or the
tier setting at all. Separately, the Web UI's live Manual/AI toggle lets an
operator seize direct control at any instant regardless of what tier is
active, described in the README as "the fundamental safety guarantee for
real-world deployment."

### Why it matters

A model that has passed every offline evaluation gate might still reasonably
warrant maximum caution on its *first* real deployment, then earn wider
unattended scope over successive successful flights — promotion status and
moment-to-moment operator trust are different axes, and conflating them into
one binary "promoted or not" forces an operator to either accept full
unattended behavior immediately or get none of the benefit of automation at
all.

### What Icarus should add

Icarus's model-promotion policy (`PHASE-12-REGRESSION-POLICY.md`) answers
"has this model earned more autonomy," which is necessary but not
sufficient — it says nothing about how cautious a *specific mission*, on a
*specific day*, with a *specific vehicle*, should be, independent of model
promotion status. The concept, adapted to sit *below* guardrails rather than
replace them (AerialClaw's tiers sit in the same process as the model, which
Icarus should not replicate):

- A runtime, per-mission **confirmation-cadence setting**, orthogonal to
  model promotion, that an operator sets before a flight — e.g. "confirm
  every proposed action" for a first real-hardware flight of an otherwise
  fully-promoted model, loosening over subsequent successful flights. This
  setting should live and be enforced at the guardrail/mission-executor
  layer (i.e., it changes how many DCM proposals require a human
  acknowledgment before guardrail-approved actions execute), never inside
  the DCM process itself — preserving Icarus's separation of the model from
  its own trust boundary, unlike AerialClaw's config-in-the-same-process
  approach.
- This is a natural, low-risk companion to the sim→physical transition
  already planned: it gives a concrete answer to "how do we fly the first
  real-hardware mission with a sim-promoted model without either refusing to
  use it at all or trusting it exactly as much as in sim."
- An always-available, low-latency human override channel — independent of
  cadence setting — is the one piece of this concept Icarus's authority
  ordering already guarantees structurally (the safety supervisor and pilot
  kill/manual override sit above the DCM unconditionally), so this concept
  is really only about adding the *graduated, mission-scoped* dial on top of
  a guarantee Icarus already has, not about adding the override capability
  itself.

---

## 7. Graceful Degradation & Failure-Honesty Contract

### What AerialClaw does

Several independent mechanisms enforce the same principle — a broken or
unavailable model channel must never be reported as, or silently converted
into, mission success:

- `brain/agent_loop.py`'s main loop tracks `_parse_fail_count` across
  iterations; three consecutive unparseable LLM outputs abort the task
  explicitly with `"任务中止：LLM 连续输出无法解析，未能生成可执行决策"`
  ("mission aborted: LLM output repeatedly unparseable, no executable
  decision produced") rather than silently retrying forever or, worse,
  treating silence as completion.
- The same file's `_safe_return()` is invoked whenever a task ends via
  max-iterations, `stuck`, or abort while still airborne: it first *asks the
  LLM* to plan a safe return given current state, but if that call fails or
  produces no valid action, it falls through to an unconditional, hardcoded
  `return_to_launch` dispatch — a deterministic fallback that does not depend
  on the LLM channel being healthy at all.
- `brain/chat_mode.py`'s `unified_chat()` wraps the LLM call in a
  try/except; on failure it attempts `_fallback_single_action_plan()` — a
  narrow, explicitly-scoped, purely rule-based parser that recognizes only
  unambiguous one-shot commands (`takeoff`, `land`, `hover`, `return_to_launch`,
  or `fly_to` with an explicit coordinate) and converts them directly to a
  plan without touching the LLM at all, tagging the response
  `"（LLM 通道暂不可用，已使用本地单动作兜底。）"` ("LLM channel unavailable,
  used local single-action fallback") so the operator knows a deterministic
  fallback fired rather than the model. Anything not matching this narrow
  set correctly falls through to an honest error message rather than a
  guessed plan.
- `llm_client.py` converts raw HTTP failures into typed, user-facing
  messages via `_friendly_http_error()` (auth failure vs. model-not-found vs.
  rate-limited vs. upstream-down each get distinct, actionable text) while
  keeping the raw diagnostic detail in logs only — a deliberate split between
  what an operator needs to see and what a debugger needs to see.

### Why it matters

The single most dangerous failure mode for any LLM-in-the-loop system is not
"the model made a bad decision" — guardrails exist for that — it's "the
model produced nothing coherent, and the system either hung, crashed, or
quietly recorded success anyway." AerialClaw treats this as a first-class
design requirement at three separate layers (agent loop, chat mode, HTTP
client), not an afterthought try/except.

### What Icarus should add

This is arguably the highest safety-value, lowest-architectural-risk concept
in this document, because it requires no new authority anywhere — it only
requires deciding, in advance and explicitly, what already-guardrailed,
already-safe behavior happens when the DCM stops producing usable output
mid-mission. Concepts to design in before the live loop exists:

- An explicit, bounded retry-then-abort contract for DCM output that fails
  to parse or fails schema validation — a small fixed number of retries,
  then a defined, deterministic transition (hold, RTL, or land, chosen by
  mission phase/policy) that does not itself depend on the DCM being
  available. This is a natural extension of guardrails' existing
  state-freshness checking: an unparseable or missing DCM decision should be
  treated exactly like stale state already is, not as a new, unhandled case.
  This is also a "Phase 12" thing in spirit: it's just another injected-fault
  category (loss of a *coherent decision*, alongside link loss and DCM
  timeout, which Phase 12 already tests) rather than a new subsystem.
- A hard invariant, enforced at the mission-executor or episode-recorder
  level rather than trusted to the DCM's own self-report: a parse failure,
  schema-validation failure, or DCM timeout must never be recorded as mission
  success in the episode log. Given how central episode integrity already is
  to Icarus's evaluation methodology, this is less a new feature than a
  correctness requirement on an existing one.
- A narrow, explicitly-scoped deterministic command path for the small set
  of unambiguous one-shot operator commands (land, RTL, hold) that does not
  route through the DCM at all — useful specifically as the degraded-mode
  answer to "the model channel is down, but the operator still needs to be
  able to say land right now." AerialClaw's scoping discipline here is worth
  preserving exactly: deliberately narrow enough that it can never be
  mistaken for a planning capability.
- User-facing vs. diagnostic error message separation for any model-runtime
  failure (auth, model-not-found, timeout, malformed response), so an
  operator gets an actionable message while full diagnostic detail is
  retained in logs — a small but real operability gap today.

---

## 8. Explain/Act Channel Separation and Hallucination Guarding

### What AerialClaw does

`brain/chat_mode.py` does not use a hardcoded intent classifier to decide
whether an operator message is a question or a command (a legacy
`classify_intent()` function exists but is explicitly noted as unused by the
live path). Instead, `unified_chat()` gives the model one unified prompt with
full context and lets it decide for itself whether to reply conversationally
or emit a `{"plan": [...]}` JSON block — and then **verifies** that choice
after the fact rather than trusting it blindly:

- `parse_response()` scans the raw reply for a valid `plan` JSON block; if
  found, the surrounding natural-language text is extracted separately as the
  conversational component and both are returned together.
- If no plan is found, `_detect_action_hallucination()` checks the reply text
  against a curated list of action-implying phrases in Chinese and English —
  "我正在飞" ("I'm currently flying"), "开始执行" ("beginning execution"),
  "朝东" ("heading east"), etc. — because a model that *describes* taking an
  action without emitting the JSON plan has produced a dangerous lie: the
  vehicle has not moved, but the operator has been told it has.
- On a detected hallucination, the system does **not** silently accept the
  text or silently discard it. It appends the hallucinated reply as an
  assistant turn, appends a fixed correction prompt
  (`HALLUCINATION_CORRECTION_PROMPT`, which states explicitly "光用文字描
  述动作不会让你的身体移动" — "describing an action in text alone will not
  move your body") and re-queries the model at lower temperature. If the
  retry produces a valid plan, that plan is used with the *original*
  descriptive text kept for readability. If the retry still fails to produce
  a plan, the system replaces the reply entirely with a hardcoded, honest
  message telling the operator the current skill set can't do this and
  asking for a more specific instruction — never letting a hallucinated
  narrative reach the operator unlabeled.
- The system prompt itself (`build_unified_prompt()`) states the invariant
  directly to the model in its rules section: "**绝不可以**...假装执行了动
  作！...你的身体只听JSON plan, 不听你的文字描述" ("You must never... pretend
  to have executed an action! ... Your body only obeys the JSON plan, not
  your text description").

### Why it matters

Free-form LLM text and machine-executable intent are fundamentally different
kinds of output, and collapsing them into "whatever the model says" creates a
specific, dangerous failure class: a fluent, confident-sounding narrative
that implies physical action occurred when it did not. This is worse than an
obvious error, because it's designed (by the model's own training) to sound
authoritative. AerialClaw's answer isn't just architectural separation — it's
active, automated **detection and correction** of exactly this failure mode,
with a hard fallback to an honest refusal if correction fails.

### What Icarus should add

Icarus's typed action contract already structurally prevents free text from
being interpreted as an action — the DCM must emit one schema-constrained
typed action, not prose, which is a stronger starting position than
AerialClaw's "hope the JSON block is there." But the underlying risk this
section addresses is broader than the JSON-vs-prose mechanics: it's whether
*any* natural-language surface the DCM produces (status explanations,
operator-facing chat, rationale text — see Section 9) could be misread, by an
operator or downstream tooling, as implying an action occurred. Concepts
worth adding:

- A structurally separate, clearly-labeled channel for any DCM-produced
  natural-language explanation or status narration, so it is never
  positioned adjacent to or confusable with the one schema-constrained typed
  action per decision point Icarus's action contract already enforces —
  operators should never be able to mistake "the model explained what it
  would consider doing" for "the model proposed and guardrails approved an
  action."
- A verification pass — even a simple deterministic keyword/pattern check
  akin to `_detect_action_hallucination()` — over any DCM-produced narration
  field, flagging language that asserts a completed or in-progress physical
  state ("has landed," "is now heading north") when no corresponding
  validated action exists in that decision's record. Given that Icarus
  already captures rationale/state per decision (Section 9), this check is
  nearly free to add and closes a real gap: a confidently-worded but false
  narration could otherwise mislead an operator even though it never touched
  the actual control path.
- This concept is intentionally scoped as an **operator-trust safeguard**,
  not a flight-safety mechanism — Icarus's guardrails already make it
  structurally impossible for hallucinated narration to reach the vehicle.
  The risk being closed here is a human misreading DCM output during
  supervised operation, which matters just as much given how central human
  oversight is to Icarus's authority model.

---

## 9. Decision Rationale Capture as a First-Class Artifact

### What AerialClaw does

The model is never asked only for an action — every decision-point output
schema requires accompanying rationale, at multiple points:

- `agent_loop.py`'s per-iteration output contract (`AGENT_SYSTEM_PROMPT`)
  requires `thinking` (first-person, 2-3 sentences, what was observed and
  considered), `decision` (act/done/stuck), the `action` itself,
  `reflection` (what was learned from the *previous* step — explicitly
  `null` on the first iteration), and `goal_progress`. This is stored
  verbatim per step in `action_history` and is what feeds both the next
  iteration's prompt (so the model's own past reasoning is visible to its
  future self) and the end-of-task reflection pipeline.
- `planner_agent.py`'s planning contract separately requires `reasoning`
  (why this plan) alongside the `plan` array itself, logged and printed at
  each planning stage.
- Distinct from per-step rationale, `reflection_engine.py` performs a
  **second-order** LLM pass after the whole task, explicitly required to
  output `outcome_analysis` ("结果分析: 成功/失败的核心原因" — root-cause
  analysis, not just outcome restatement) and per-skill `performance` ratings
  with a `suggestion` field — i.e., the system captures not just "what did
  the model decide" per step, but "why, in retrospect, did the whole episode
  succeed or fail."

### Why it matters

Icarus's own evaluation work has already demonstrated the value of this
exact capability empirically: the finding that Qwen3-4B never once proposed
`land` across 27 decision points where it should have was only interpretable
*as a decision-quality problem* because the evaluation harness could inspect
what the model was choosing between and why. Rationale capture is what turns
"the model behaved badly" into "here specifically is why," which is the
difference between a fixable finding and an unexplained anomaly.

### What Icarus should add

Icarus's episode format already captures actions, state, and perception per
boundary crossing (Phase 10). The concept to add is explicit: extend that
same per-decision-point capture to include the DCM's own stated
reasoning/confidence for each proposed action, not only the action itself —
symmetric to what `agent_loop.py` already does for `thinking`/`reflection`,
but stored as sealed episode data rather than live prompt state:

- A rationale field in the action-proposal schema itself, required (not
  optional) at the contract level, so every candidate action arrives with the
  DCM's own stated justification — this is nearly free to add to the schema
  now, before any live loop exists, and expensive to reconstruct later from
  black-box episode replay alone.
- A distinction, mirrored from AerialClaw's two-tier structure, between
  **per-decision rationale** (why this action, right now) and **post-episode
  analysis** (why did the whole mission succeed or fail, in retrospect) —
  these answer different questions and Icarus's evaluation harness would
  benefit from both being separately queryable, the same way AerialClaw's
  `evaluate.py`/`compare.py` benefit from having granular per-decision
  reasoning available when a specific decision point looks wrong.
- Because Icarus already treats "evidence beats demos" as a foundational
  principle, this concept is really just an application of that principle
  one layer deeper: the reasoning behind a decision is itself evidence, and
  should be captured with the same discipline as the decision's outcome.

---

## 10. Reflection-to-Evidence Loop

### What AerialClaw does

`memory/reflection_engine.py`'s `ReflectionEngine.reflect()` runs after every
task: it gathers the full skill-execution trace, perception events, replan/
emergency-stop/obstacle counts, current measured skill statistics
(`skill_memory.get_all_skill_reliabilities()`), and the *existing* long-term
memory file (explicitly included so the reflection prompt can instruct the
model to avoid re-recording something already known), then asks the LLM for
a structured reflection with five required fields: `summary`,
`outcome_analysis`, `environment_insights`, `task_lessons`, and
`skill_feedback` (per-skill `performance`/`suggestion`/`recommended_params`).
That structured output is then mechanically fanned out to concrete,
persistent artifacts: `update_memory()` appends new lines to specific
sections of `MEMORY.md`; `update_skills()` merges the feedback into
`SKILLS.md`'s per-skill entries, preserving old notes where no new feedback
exists; `SkillEvolution.record_feedback()` appends to the long-run trend
history used for degradation/drift detection (Section 5); and — the
closed-loop part — `agent_loop.py._update_memory()`'s step 5 feeds the same
task's successful skill chain into `dynamic_skill_gen.detect_patterns()`,
potentially minting a new soft skill (Section 2) directly from this
reflection, with the reflection's own `task_lessons` simultaneously appended
to any existing soft-skill doc whose name or keywords match the lesson text.

The reflection prompt's own system instructions are explicit about
epistemic caution: "环境知识要稳定持久: 不记录偶发事件, 记录规律性发现"
(environment knowledge should be stable and durable — don't record one-off
events, record regular patterns) and "策略更新要保守: 只有充分证据时才建议
改变策略" (strategy updates should be conservative — only suggest changing
strategy when there's sufficient evidence) — i.e., AerialClaw's own design
explicitly warns against over-updating from a single episode, even though its
actual persistence mechanism (a single LLM judgment per task, immediately
written to disk) has no structural enforcement of that caution beyond the
prompt asking nicely.

### Why it matters

Reflection only has value if it changes something durable. AerialClaw's
version closes that loop end-to-end — LLM reflection literally rewrites
memory files, skill stats, and can synthesize new strategy documents,
automatically, per task. The weakness is exactly where its own prompt
worries: a single model-authored judgment gets written directly into
long-term state with no independent verification step between "the model
said this was a lesson" and "the system now treats it as durable knowledge."

### What Icarus should add

This is where Icarus's existing evidence/evaluation discipline is a direct,
structural improvement over AerialClaw's approach, not just a
reimplementation of it — the concept to adopt is the *loop*, not
AerialClaw's trust model for what closes it:

- A reflection pass after task/episode completion that produces the same
  kind of structured output AerialClaw's does (outcome analysis, environment
  insights, per-action feedback) — this part is worth taking directly, it's
  a well-designed schema.
- But where AerialClaw's reflection output is written straight to durable
  memory on the reflecting model's own say-so, Icarus's version should route
  reflection output through the same promotion-style gate already used for
  models (Section 6/Phase 12): a reflected-on finding — especially a
  candidate `environment_insight` or a candidate skill-degradation
  signal — becomes a **proposed addition to the frozen scenario/regression
  corpus**, or a proposed update to the evidence-backed world model
  (Section 3), reviewed the same way any other promotion-relevant evidence
  is, rather than an immediate, unverified write. A failure or near-miss
  becomes a new regression scenario Phase 12 already knows how to treat as a
  first-class artifact — reflection's job is to *propose* that scenario, not
  to unilaterally rewrite what the system considers true.
- The operational skill reliability ledger (Section 5) and skill-evolution-
  style degradation/drift detection (also Section 5) are natural downstream
  consumers of this same reflection pass, exactly as they are in AerialClaw
  — reflection output should feed the live ledger's trend detection, not
  just a one-off memory note.
- AerialClaw's own two caution instructions embedded in its reflection
  prompt ("don't record one-off events," "be conservative about strategy
  changes") are effectively a description of what a promotion gate enforces
  *structurally* — Icarus should implement that caution as an actual gate
  with real evidence thresholds, not as a prompt instruction a reflecting
  model is trusted to honor.

---

## 11. Simulation Environments, Terrain & Weather/Wind Modeling

This section is descriptive rather than adoption-oriented: it documents
exactly what simulation content AerialClaw ships and runs, because the
README's visuals (a dense, high-rise, photorealistic city flythrough) and
the repository's actual committed simulation assets turn out to be two very
different things — a distinction worth being precise about before treating
"get a similarly realistic test environment" as a small task.

### 11.1 What's actually in the git repository: Gazebo Harmonic / PX4 SITL

The only simulation content version-controlled inside AerialClaw is two
Gazebo Harmonic world files (`sim/worlds/urban_rescue.sdf` and
`urban_rescue_full.sdf`) plus one vehicle model
(`sim/models/x500_lidar_2d_cam/model.sdf`) — the same simulator family
(Gazebo Harmonic) and flight stack (PX4 SITL, via MAVSDK) Icarus already
uses conceptually, though Icarus uses ArduPilot rather than PX4.

**`urban_rescue.sdf`** (the lightweight default) is a minimal hand-built
scene: a flat 300m×300m ground plane, two intersecting roads, a helipad, and
a handful of primitive-box structures — two ~4m houses, one ~18m office
building, three rubble piles, four cylinder-and-sphere "victim" figures, one
decorative "fire" building with emissive flame/smoke geometry, and three
simple sphere-and-cylinder trees. Everything is built from raw SDF
`<box>`/`<cylinder>`/`<sphere>` primitives with flat ambient/diffuse color —
there is no imported mesh, no texture, and no architectural detail anywhere
in this file.

**`urban_rescue_full.sdf`** (the richer disaster-response scenario, and the
one referenced by `sim_quickstart.sh`'s default demo) is substantially
larger but built the same way, organized into seven named, colored ground
zones covering roughly the same 300m×300m footprint:

| Zone | Content |
|---|---|
| A — 居民区 (Residential) | Three box houses (4–5m) with inserted window/door panels, two Gazebo-Fuel pedestrian models |
| B — 办公区 (Office) | A 6-floor ~18m office block and a 4-floor ~12m apartment block, both built as a single box shell with per-floor window insets, one Fuel pedestrian |
| C — 灾区 (Disaster/rubble) | Three collapsed-building models (tilted "slab" and "wall" box fragments simulating rubble), four "Rescue Randy" victim-mannequin Fuel models (a standard search-and-rescue-robotics benchmark prop), four marker posts, three loose debris boxes |
| D — 树林区 (Forest) | Fuel-model oak and pine trees, one "hiker" victim figure |
| E — 物资区 (Supply depot) | A Fuel warehouse model, an ambulance, medic and pedestrian figures |
| F — Fire zone | A scorched box building with emissive-material flame and smoke *geometry* (static colored cylinders, not a particle or volumetric effect), a scorched-ground decal, a Fuel fire-truck model |
| G — SOS/access-control | A red cloth marker and sign, Jersey barriers, traffic cones, construction barrels, an overturned dumpster, a "trapped" SUV, additional firefighter and injured-civilian figures |

The people, vehicles, trees, and props throughout are pulled from **Gazebo
Fuel** (`fuel.gazebosim.org`), Open Robotics' free, community-hosted model
library — referenced by `<include><uri>https://fuel.gazebosim.org/...` tags
that Gazebo resolves (and caches locally) at world-load time, meaning a
fresh machine needs network access the first time it loads this world. The
buildings themselves are not Fuel assets — they're hand-authored boxes.

Two facts are worth being explicit about because they cut directly against
the "dense city" impression:

- **Maximum building height in the entire committed world is ~18m** (the
  6-floor office block). There is nothing resembling a skyscraper, a
  high-rise district, or the kind of dense vertical canyon flying the
  README's Shanghai GIF shows. This is a low-rise disaster-response
  training ground, not a city.
- **Lighting is a single static directional sun + fill light** — there is no
  day/night cycle, no dynamic shadows-over-time, and no atmospheric
  scattering/haze system.

The vehicle model, `x500_lidar_2d_cam`, is PX4's stock `x500` quadrotor with
one 2D planar LiDAR (`lidar_2d_v2`, top-mounted — not a 3D/360° unit despite
the README's "360° LiDAR" framing, which applies to the separate AirSim path
below) and four side cameras plus one downward camera, each 640×480, 80°
horizontal FOV (1.396 rad), tilted 15° down, and — notably — updating at only
**5 Hz**, not full video framerate. Physics runs on ODE at a 4ms step /
250Hz update rate with `real_time_factor=1.0`.

### 11.2 Wind and weather: confirmed absent

A full-repository search (across all Python, SDF, YAML and Markdown files)
turns up no wind plugin, no `<wind>` SDF element, no gust model, and no
weather/rain/fog/precipitation system anywhere in AerialClaw — in either the
Gazebo path or the AirSim path. `config/sim_config.yaml`'s only
environment-variability knob is an optional, **disabled-by-default**
`sensor_noise` block (`gps_stddev`, `imu_noise_level`) — that perturbs
*sensor readings*, not aerodynamic force on the airframe; it is not wind
simulation in any sense. AerialClaw's own auto-generated `BODY.md` states
this limitation directly to its own LLM planner: *"天气: 仿真环境无风雨影响,
真实环境需考虑"* — "weather: the simulation environment has no wind/rain
effects; this must be considered for the real environment." AirSim itself
does expose a wind API (`simSetWind`) independent of AerialClaw, but nothing
in `adapters/airsim_adapter.py`, `airsim_physics.py`, or `airsim_rpc.py`
calls it, and the one observed AirSim `settings.json` excerpt
(`docs/AIRSIM_DEPLOYMENT.md`) contains no wind configuration.

This is a genuine, confirmed gap in AerialClaw, not an artifact of
incomplete documentation — and it is a dimension where Icarus's own recent
work (a custom atmosphere established as the sole wind-force authority, with
fixed-seed gust and strong-wind regression scenarios already implemented and
evidenced under Phase 12) is materially ahead of what AerialClaw has built,
despite AerialClaw's overall head start on the live agent loop.

### 11.3 The dense, photorealistic city: AirSim + OpenFly (external, not in the repo)

The visuals that actually match "dense buildings, skyscrapers, city-like
area" — the README's Shanghai flythrough GIF and the geography described in
`robot_profile/WORLD_MAP.md` (nearby 14–24m buildings, a 150m mid-rise
commercial district, and skyscrapers up to 477m matching Shanghai's real
Lujiazui financial district) — come from a **separate, external simulation
path that is not part of the AerialClaw git repository at all**.

Per `docs/AIRSIM_DEPLOYMENT.md`, this environment is a pre-built Unreal
Engine 4 binary from a separate research project referred to as "OpenFly"
(the same project underlying the "AirVLN" scene numbers 16/18/23/26 also
listed there), hosted on a **remote GPU server** the AerialClaw team happens
to have SSH access to (`~/code/openfly/envs/airsim/env_airsim_sh/` on a
named remote host). AerialClaw's own code contributes nothing to this
environment's content — it only supplies a client: `adapters/airsim_rpc.py`
is a deliberately minimal, pure-socket msgpack-RPC implementation written
specifically to avoid the official `airsim` pip package's tornado/asyncio
conflicts (documented at length as eight separate numbered pitfalls in that
same file). Reaching this environment requires: a real GPU-class remote
machine running UE4 headless (`-nullrhi`), an SSH tunnel forwarding port
41451, and a local Python ≤3.12 (the `airsim` package's
`msgpack-rpc-python` dependency does not support 3.13+).

Flight control in this path is AirSim's own built-in **SimpleFlight**
simulated controller — not PX4 or ArduPilot — configured via a
`~/Documents/AirSim/settings.json` on the remote server with simple
kinematic limits (`MoveMaxSpeed`, `LinearAccelMax`). This is a categorically
different, simpler flight-dynamics path from the Gazebo/PX4 SITL stack
described above, and switching between them in AerialClaw means switching
adapters, not just worlds.

The practical implication: **cloning the AerialClaw repository does not get
you the dense-city environment shown in its own README.** It gets you the
RPC client code capable of *talking to* such an environment, plus
documentation of exactly which pitfalls to expect connecting to one — but
the environment asset itself is an external, remote-hosted, GPU-dependent
binary from a separate project, not confirmed to be public or
redistributable, and not something evaluated for licensing or availability
here.

### 11.4 What this means for testing Icarus in a denser environment

Given the above, the honest options for the dense/realistic environment the
user wants, roughly in order of how much they preserve Icarus's existing
sim/real parity investment (ADR-0002) and ArduPilot/Gazebo Harmonic stack:

- **Reuse Gazebo Fuel as an asset source inside the existing ArduPilot/
  Gazebo Harmonic stack.** This is the one concrete, low-risk idea directly
  transferable from AerialClaw's own repo: `fuel.gazebosim.org` is a free,
  community asset library (buildings, vehicles, pedestrians, trees, props)
  usable from any Gazebo Harmonic world via a plain `<include><uri>` tag,
  exactly as `urban_rescue_full.sdf` does — no new simulator, no new flight
  stack, and it stays inside Icarus's already-adopted simulator version.
- **Author taller, denser structures using the same cheap technique
  AerialClaw uses** — stacked box volumes with per-floor window insets —
  scaled up to real high-rise heights (AerialClaw's tallest is only ~18m;
  nothing stops the same technique from producing a 100m+ tower). This reads
  reasonably at drone-camera altitude without needing photorealistic meshes,
  and can be organized using the same zone-based world-authoring pattern
  (named, colored ground regions per district type) AerialClaw uses to keep
  a large world legible.
- **For true photorealistic, UE4-grade high-rise density** matching the
  README GIF specifically, the honest path is outside anything AerialClaw's
  own repository provides: either a UE4/AirSim-based city (which would mean
  adopting AirSim's SimpleFlight or bridging it to ArduPilot separately,
  a real architectural fork from the current stack) or higher-fidelity
  Gazebo/Ignition city content from elsewhere (existing "city" world packs,
  OpenStreetMap-derived procedural building generation, or CARLA-exported
  meshes converted for Gazebo) layered onto the ArduPilot/Gazebo Harmonic
  stack Icarus already has working end-to-end — preserving sim/real parity
  rather than introducing a second, incompatible flight-control simulator
  purely for visual density.
- Whichever direction is chosen, **wind/weather realism should not regress**
  in the process — this is a dimension AerialClaw simply never built, and
  it's already a demonstrated Icarus strength (custom atmosphere as wind
  authority, fixed-seed gust regression evidence) worth preserving through
  any environment-density work rather than something to copy from
  AerialClaw at all.

---

## 12. Synthesis: How These Interlock

These ten concepts are not independent features to pick and choose from —
they form two natural clusters that depend on each other:

**Input-side concepts** (Sections 1, 3, 4) define what the DCM is allowed to
see at a decision point: a bounded, versioned perception/state contract; a
persistent, evidence-backed world/capability model; and a narrow, advisory
active-perception tool. Together these are the answer to "what does the
model know."

**Output-and-feedback-side concepts** (Sections 2, 5, 6, 7, 8, 9, 10) define
what the DCM is allowed to do with that knowledge, how confidently, and what
happens to the outcome afterward: mission macros for composing known-good
procedures; a live reliability ledger and a graduated confirmation dial that
both modulate trust independent of static model promotion; explicit,
deterministic handling of a broken model channel; a hard separation between
explanation and action; captured rationale per decision; and a reflection
pipeline that turns operational experience back into evidence rather than
unverified belief.

The single idea threading through all ten, and the one place Icarus's
approach should diverge most deliberately from AerialClaw's, is this:
AerialClaw treats the model's own output — its stated reasoning, its
reflection, its environment insights — as sufficient justification to update
durable system state directly. Icarus's existing culture (guardrails,
independent safety supervisor, frozen-scenario evaluation, replayable
episodes) already supplies the missing piece AerialClaw doesn't have: a
verification/promotion layer between "the model said so" and "the system now
acts as if it's true." Every concept above should be adopted with that layer
interposed, not removed.

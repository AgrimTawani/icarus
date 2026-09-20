# Icarus Data and Evaluation

## Purpose

Icarus treats every simulation or physical flight as a versioned episode that
can support debugging, replay, model comparison and—after curation—training.
Logging must never block the flight-control path.

## Episode Manifest

The planned manifest identifies:

- repository revision, dirty-state flag and runtime/container versions;
- vehicle, parameter, safety-policy and sensor-profile revisions;
- scenario name, schema version, random seed and generated-world hash;
- mission and operator inputs;
- model provider, immutable model revision, quantization, prompt and tool schema;
- synchronized stream inventory, clock mapping and missing-data indicators;
- final metrics, safety events and termination reason.

Streams include normalized vehicle state, perception summaries, model inputs and
outputs, requested and approved actions, execution feedback, safety decisions,
MAVLink events and simulator ground truth. Raw video/lidar is optional and kept
separate with explicit retention and privacy rules.

## Dataset Pipeline

```text
runtime events -> immutable raw episode -> validation -> annotation/curation
               -> versioned dataset manifest -> train/validation/evaluation split
               -> model experiment -> frozen regression evaluation
```

Raw episodes are never edited in place. Derived datasets record parent episode
IDs and transformation versions. Failed and safety-intervention cases are kept;
they are often more valuable than clean demonstrations.

## Model Comparison

Qwen, Llama and future models run through the same adapter contract. A campaign
freezes scenario revisions, seeds, prompts, action schema, safety policy and
resource limits. It includes a deterministic no-LLM baseline.

Minimum metrics:

- mission success and correct terminal state;
- collisions, geofence violations and safety interventions;
- rejected, malformed or hallucinated actions;
- time to completion and path efficiency;
- model decision latency and timeout rate;
- compute memory, utilization and energy proxy where available;
- recovery success after injected faults.

Report distributions and confidence intervals across repeated seeds. Never rank
models from one visually impressive run.

## Leakage Control

Evaluation scenarios and seeds are versioned and excluded from fine-tuning
exports. Near-duplicate environments are grouped before splitting. Simulator
ground truth may label data but must not be presented to the runtime model as a
sensor observation. Physical-flight evaluation remains a separate final set.

## Offline Decision Evaluation (implemented)

`python/dcm/evaluate.py` scores a runtime across an episode corpus without a
simulator or any path to the aircraft:

```bash
./scripts/fly-episode-corpus                 # build a corpus of flights
./scripts/evaluate-dcm --runtime llama       # score a model across all of it
```

Three scoring rules are load-bearing, and each exists because of a measured
result rather than a preference.

**Repeated runs, reported as a spread.** The same model on the same episode
produced first-decision latencies of 4257, 4563 and 7204 ms, one of which
breached the 5000 ms deadline. Any single run would have been misleading. The
harness defaults to three repeats and reports min, median, p90 and max.

**Vocabulary mismatches are unscoreable, not failures.** Recorded flights use
Drone API actions such as `goto` that are outside the current model
vocabulary. A model that cannot express the baseline's action has not
disagreed with it. Those points are counted and reported separately, never as
model failures, because counting them would penalise every model equally and
mask the differences a comparison exists to find.

**Agreement is not correctness.** The recorded action is one competent choice,
not ground truth. Agreement is therefore reported as a rate over comparable
points, and never as a score out of all points. A model can agree with the
baseline everywhere and still be unfit, and can disagree while being right.

Latency is split between the first decision and steady state, because the
first pays a one-off prompt prefill that the loop does not. Stale decision
points are excluded from latency entirely: they never reach the runtime, so
their zero cost is not a measurement of the model. When a model is never
asked, rates are reported as unavailable rather than as zero, so that a
refused episode cannot read as a clean bill of health.

Determinism is measured rather than assumed: the harness compares the
proposal sequence across repeats and reports how many episodes were stable.

## Current State

Phase 5 writes structured scenario results and sensor-health artifacts under
ignored local `logs/`. The cross-service episode schema, replay engine and
dataset exporter exist as of Phase 10, and the offline decision-evaluation
harness above exists as of Phase 11.

What remains for Phase 12 is the campaign rather than the machinery: a frozen
scenario set with held-out episodes, a deterministic no-LLM baseline to
compare against, at least two models, and the closed-loop simulator runs that
measure the consequences of a sequence of decisions rather than each decision
in isolation. Offline agreement says nothing about that, and no model has yet
been given control of anything.

The frozen-seed, randomized-robustness and promotion rules are defined in
[`PHASE-12-REGRESSION-POLICY.md`](PHASE-12-REGRESSION-POLICY.md). They prevent
model/prompt tuning from moving the pass conditions after results are known.

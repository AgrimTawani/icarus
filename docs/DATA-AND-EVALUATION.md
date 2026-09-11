# Data and Evaluation

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

## Current State

Phase 5 already writes structured scenario results and sensor-health artifacts
under ignored local `logs/`. A stable cross-service episode schema, replay
engine, dataset exporter and model-comparison harness are planned for Phases
10–12 and do not yet exist.

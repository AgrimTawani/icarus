# DCM V2 Feedback Architecture

Status: Phase 1 implementation. The V3 context path is implemented, versioned
and is now the default for `dcm-fly`, `observe-dcm` and `evaluate-dcm`.
`--context v2` preserves historical comparisons. This default change does not
change DCM authority or autonomous-promotion thresholds.

## Components

| Artifact | Location | Runtime property |
| --- | --- | --- |
| Context builder | `python/dcm/v2_context.py` | Pure/read-only transformation of an already curated observation |
| V3 contract entry | `python/dcm/contract.py:curate_v3` | Retains V1 allow-list, adds `dcm-contract-v3-compact` context fields |
| Mission contract compiler | `python/dcm/v2_context.py:compile_mission_contract` | Combines selected macro policy, reviewed target and completion evidence into one model-visible structure without choosing the next action |
| Generated BODY | `python/dcm/v2_profiles.py` + live `MissionClient.capability_profile()` | Full manifest/provenance is audit-only; the model sees connected simulator, firmware, camera, LiDAR and airframe semantics without a duplicate action list |
| Static SOUL/world facts | `config/dcm/profiles/` | Full reviewed documents and hashes are audit evidence; compact V3 renders only three SOUL principles and keeps world-fact prose audit-only |
| Mission macros | `config/dcm/macros/` | Structured model policy plus human Markdown guidance; prose and hashes remain audit-only |
| Reliability aggregation | `python/dcm/v2_feedback.py` | Automatically emits sealed evidence and human-review candidates; cannot write live policy |
| Runtime health | `python/dcm/health.py` | Deterministic late/invalid/unavailable/stale/circuit-open state machine |
| Promotion scorecard | `config/dcm/v2-scorecard.yaml` | Frozen non-relaxable offline + paired-live gates |

## Contract guarantees

1. V3 starts from `curate`, so raw sensors, truth, identifiers and authority
   leases remain excluded by the existing allow-list.
2. Obstacle context is sectorized from normalized perception only. Raw LiDAR
   arrays are neither accepted nor rendered.
3. Failure count, consecutive failures and repeated actions are computed in
   code. The model is not asked to infer them from unbounded history.
4. An explicit `mission_metadata.macro_id` overrides selection. Otherwise a
   small deterministic intent resolver can select one reviewed macro. The
   selected macro's action subset is enforced before Drone API execution;
   hold/RTL/land/none remain universally available safety exits.
5. Macro/profile hashes remain in the audit record and a final context SHA-256
   identifies exactly the compact model-visible semantics. The hash and byte
   budget are recalculated only after the live BODY and compiled mission
   contract have been attached.
   The context builder does not open a vehicle connection or write runtime
   state.
6. V3 requires one value from a closed `reason_code` vocabulary as its third
   response field. Validation strips it before the action reaches guardrails or
   execution; free-form rationale can no longer add text or arguments.
7. Three consecutive invalid/late/unavailable/stale responses open a circuit.
   Normal inference is suppressed for two decision intervals, then one bounded
   recovery probe is allowed if the mission is still active. Airborne recovery
   selects typed `hold`, then terminal `return_home`, through the normal Drone
   API and safety layers while the circuit is open.
8. The system prompt contains the fixed action contract, three compact reviewed
   operating principles and semantic live BODY facts. The user message contains
   the exact mission once, a single `MissionContract`, current state/perception,
   one evidence-health summary and bounded history. Empty world facts, macro
   prose, hashes, paths and duplicate freshness/action fields stay audit-only.
9. A second fresh observation must also be materially consistent with the one
   used for inference (flight phase, arm/land state, position, altitude and
   perception safety state), or the proposal is discarded.

## Feedback loop

```text
episode seal -> live schema validation -> deterministic aggregation -> review queue
                                                  -> macro/prompt proposal
                                                  -> held-out evaluation
                                                  -> explicit promotion only
```

Every newly sealed episode writes hash-bound `feedback.json` and
`review_queue.json`. There is no arrow from model reflection to a live macro, world fact or safety
policy. This is the deliberate Icarus adaptation of AerialClaw's feedback
loop: observations become measured evidence before changing model-visible
knowledge.

## Current macros

The reviewed set is `building-inspection`, `perimeter-sweep`,
`landing-site-survey`, `inspection-orbit` and `degraded-position-return`. Every macro has an ID,
revision, action families, action allow-list, preconditions, evidence link and
nominal/adverse/recovery fixture. They introduce no new actions.

## Measured promotion result

The frozen 11-episode Gazebo corpus was replayed three times (135 decision
points) through the same Qwen3-4B Q4 runtime.

| Metric | Current V2-history | V3 feedback candidate |
| --- | ---: | ---: |
| Invalid | 0.0% | 0.0% |
| Timeout | 0.8% | 1.6% |
| Tool agreement | 71.9% | 71.7% |
| Recovery proxy | 57.1% | 62.5% |
| Steady median | 676 ms | 1,216 ms |
| Peak VRAM | 4,069 MiB | 4,052 MiB |

The original V3 feedback prompt improved only the recovery proxy and fixed the first V3 prompt's missing
takeoff arguments, but missed the frozen timeout and latency gates and lacks
three paired live Gazebo episodes. The machine decision is therefore
`retain-v1`: V3 therefore remains unpromoted for unattended operation, even
though it is now the user-selected CLI context default. This is the required evidence-based answer to whether the
AerialClaw concepts improve the existing system—not an assumption that more
context is automatically better.

The later `dcm-contract-v3-compact` revision intentionally reduces that prompt
and replaces rationale text with a reason code. It requires a new paired
evaluation; the historical table above is retained as baseline evidence and is
not silently attributed to the compact contract.

## Verification

```bash
.venv/bin/python -m pytest -q tests/unit/test_dcm_v2_acceptance.py \
  tests/unit/test_dcm_v2_promotion.py tests/unit/test_dcm_v2_context.py
./scripts/evaluate-dcm-v2-promotion <v2-evaluation.json> <v3-evaluation.json> \
  --baseline-live <v2-live.json> --candidate-live <v3-live.json>
```

The tests also deliberately corrupt BODY, exercise all macro fixture classes,
open/suppress/probe/reset the circuit breaker, enforce macro action subsets,
validate live decision rows, reproduce feedback hashes and prove that missing
live evidence fails promotion closed. The native suite also expires a DCM
control lease while airborne and proves BRAKE-then-RTL escalation.

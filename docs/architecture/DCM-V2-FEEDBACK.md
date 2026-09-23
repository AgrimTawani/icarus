# DCM V2 Feedback Architecture

Status: Phase 1 implementation. The V2 context path is implemented and
versioned, but remains opt-in while it is compared against retained V1/V2
reports. It does not change DCM authority or autonomous-promotion thresholds.

## Components

| Artifact | Location | Runtime property |
| --- | --- | --- |
| Context builder | `python/dcm/v2_context.py` | Pure/read-only transformation of an already curated observation |
| V3 contract entry | `python/dcm/contract.py:curate_v3` | Retains V1 allow-list, adds `dcm-contract-v3-feedback` context fields |
| Generated BODY | `python/dcm/v2_profiles.py` + live `MissionClient.capability_profile()` | Static contract facts plus connected API, simulator, firmware, parameter, camera, LiDAR and airframe facts |
| Static SOUL/world facts | `config/dcm/profiles/` | Reviewed documents with SHA-256 provenance |
| Mission macros | `config/dcm/macros/` | YAML front matter plus Markdown guidance; explicit override or deterministic intent resolution |
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
5. Macro/profile hashes and a final context SHA-256 make a decision
   reproducible. The hash and byte budget are recalculated only after live BODY,
   mission reference and completion obligations have been attached.
   The context builder does not open a vehicle connection or write runtime
   state.
6. V3 requires a bounded rationale as a third response field. Validation strips
   it before the action reaches guardrails or execution; it cannot add an
   argument or authorize anything.
7. Three consecutive invalid/late/unavailable/stale responses open a circuit.
   Normal inference is suppressed for two decision intervals, then one bounded
   recovery probe is allowed if the mission is still active. Airborne recovery
   selects typed `hold`, then terminal `return_home`, through the normal Drone
   API and safety layers while the circuit is open.
8. The system prompt is assembled in explicit reviewed SOUL, generated BODY,
   reviewed world-facts and selected-macro sections. Mission obligations state
   required evidence but never prescribe a next action; planning remains with
   the DCM.
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

V3 improved only the recovery proxy and fixed the first V3 prompt's missing
takeoff arguments, but missed the frozen timeout and latency gates and lacks
three paired live Gazebo episodes. The machine decision is therefore
`retain-v1`: V3 remains implemented and opt-in, while the current contract
stays the default. This is the required evidence-based answer to whether the
AerialClaw concepts improve the existing system—not an assumption that more
context is automatically better.

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

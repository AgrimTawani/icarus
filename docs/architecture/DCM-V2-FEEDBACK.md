# DCM V2 Feedback Architecture

Status: Phase 1 implementation. The V2 context path is implemented and
versioned, but remains opt-in while it is compared against retained V1/V2
reports. It does not change DCM authority or autonomous-promotion thresholds.

## Components

| Artifact | Location | Runtime property |
| --- | --- | --- |
| Context builder | `python/dcm/v2_context.py` | Pure/read-only transformation of an already curated observation |
| V3 contract entry | `python/dcm/contract.py:curate_v3` | Retains V1 allow-list, adds `dcm-contract-v3-feedback` context fields |
| Generated BODY | `python/dcm/v2_profiles.py` | Factual action/freshness/authority manifest; never written during a mission |
| Static SOUL/world facts | `config/dcm/profiles/` | Reviewed documents with SHA-256 provenance |
| Mission macros | `config/dcm/macros/` | Markdown guidance plus JSON metadata; explicit selection only |
| Reliability aggregation | `python/dcm/v2_feedback.py` | Offline report only; cannot write macros, prompts or policy |

## Contract guarantees

1. V3 starts from `curate`, so raw sensors, truth, identifiers and authority
   leases remain excluded by the existing allow-list.
2. Obstacle context is sectorized from normalized perception only. Raw LiDAR
   arrays are neither accepted nor rendered.
3. Failure count, consecutive failures and repeated actions are computed in
   code. The model is not asked to infer them from unbounded history.
4. Macro selection requires `mission_metadata.macro_id`; free-text keyword
   matching is intentionally absent. Every macro is validated against the same
   declared action vocabulary used by the response validator.
5. Macro/profile hashes and a context SHA-256 make a decision reproducible.
   The context builder does not open a vehicle connection or write runtime
   state.
6. Rationale, when a future runtime supplies it, is an audit artifact separate
   from strict action JSON. It cannot add arguments or influence validation.

## Feedback loop

```text
sealed decision rows -> deterministic aggregation -> review queue
                                                  -> macro/prompt proposal
                                                  -> held-out evaluation
                                                  -> explicit promotion only
```

There is no arrow from model reflection to a live macro, world fact or safety
policy. This is the deliberate Icarus adaptation of AerialClaw's feedback
loop: observations become measured evidence before changing model-visible
knowledge.

## Current macros

`perimeter-sweep` and `landing-site-survey` are initial reviewed composition
examples. They introduce no new actions. Each says what it may recommend and
what it cannot authorize. Add a macro only with a revision, allowed-action
allow-list, scenario evidence and nominal/adverse/recovery tests.

## Verification

```bash
.venv/bin/python -m unittest discover -s tests/unit -p 'test_dcm_v2_context.py' -v
```

The test suite checks deterministic context hashes, curation non-leakage,
sectorization, explicit macro selection, profile provenance and evidence-only
aggregation.

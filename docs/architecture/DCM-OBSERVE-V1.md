# DCM Observe-Mode Replay (First Phase 11 Slice)

This first Phase 11 slice consumes a sealed Phase 10 episode offline. It does
not connect to the Drone API, Gazebo, SITL, MAVLink, or a vehicle. It verifies
episode integrity and replays native guardrail decisions before invoking a
model-shaped runtime at each recorded pre-action decision point. The output is
written separately under `logs/dcm/observe/`; the source episode is untouched.

Run the current wiring-test runtime with:

```bash
./scripts/observe-dcm logs/episodes/<episode-id>
```

The current `mock-no-action` runtime always proposes `none`. It proves input
curation, strict response parsing, latency/error accounting, and report
generation; it is **not** an LLM and its proposals are not flight-performance
results. `summary.json` records source hash, runtime, decision counts and the
decisions stream hash. `decisions.jsonl` records each bounded observation,
proposal, validity result, latency and corresponding recorded action. The
recorded action is added *after* the runtime call; it is not shown to the
runtime.

The initial response is exactly one JSON object with `action` and `arguments`.
Only `none`, `arm`, `takeoff`, `hold`, `return_home`, and `land` are accepted.
The vocabulary will expand through versioned contracts; it is not yet the full
Drone API. Unknown keys, prose, unsupported actions and malformed arguments
are rejected. The observer records a late response as a timeout, but this mock
slice does not yet enforce a hard process deadline for a local model. A real
model adapter must enforce that deadline at its process boundary.

## The model contract

`python/dcm/contract.py` holds everything a model may see, say, or be refused,
behind three explicit versions: `dcm-contract-v1`, `dcm-actions-v1` and
`dcm-prompt-v1`. A provider adapter implements `ModelRuntime` and nothing else.
It never decides what is observable, what is allowed, or what counts as fresh.

The action vocabulary is a table, not control flow. Each action declares its
arguments with closed bounds, and the prompt's vocabulary is generated from
that same table, so a model cannot be invited to emit something the validator
would then reject. Widening the vocabulary is a versioned edit to one table.

`curate` cannot receive the recorded next action: it is not a parameter, so it
cannot leak into a prompt. The recorded action is attached to the report only
after the runtime has answered.

`assess_freshness` refuses a stale observation **before** the model is asked. A
model shown stale state answers confidently about a situation that no longer
holds, and that answer would otherwise be recorded as valid. State older than
1000 ms or a local map older than 750 ms is refused; the map limit matches the
ObstacleMap expiry in `perception/obstacle_map/obstacle_map.hpp`. A missing or
unparseable timestamp counts as stale, because absent evidence of freshness is
not evidence of freshness.

Timestamps arrive in two shapes. Protobuf's canonical JSON mapping renders
int64 as a string and int32 as a number, so a recorded episode carries
`observed_at_unix_ms` as `"1789827075223"` while `local_map_age_ms` is `28`.
Both are accepted; anything else is treated as a missing timestamp.

On the sealed 677-record stress episode the contract produced four valid mock
proposals and refused one decision point as stale. That refused point is the
one the recorded episode marks `REASON_CODE_SAFETY_INTERVENTION`, with a local
map 1434 ms old and perception health `HEALTH_LEVEL_UNAVAILABLE`. The native
C++ safety supervisor at flight time and this offline rule independently agree
on the same decision point, which is corroboration rather than proof.

`RuntimeDescriptor` records the model id, family, quantization, artifact path
and SHA-256, runtime name and revision, context length, sampling settings and
deadline. `verify_artifact` re-hashes the weights so a swapped or truncated
file fails loudly instead of quietly changing results.

## The llama.cpp adapter

`python/dcm/llama_runtime.py` implements `ModelRuntime` for a pinned local
GGUF. It owns a `llama-server` subprocess rather than spawning `llama-cli` per
decision, because reloading multi-gigabyte weights per question would dominate
measured latency and say nothing about the model.

```bash
./scripts/observe-dcm logs/episodes/<episode-id> --runtime llama
```

### The deadline

The deadline is wall-clock and enforced by abandoning the request. The first
implementation was wrong in a way worth recording: a timeout passed to
`urllib` applies *per socket operation*, not to total elapsed time, so a real
decision took 7107 ms against a 5000 ms timeout and still returned an answer.
The request now runs on a worker thread against a wall-clock deadline; on
expiry the request is abandoned and the server is terminated and restarted. A
unit test reproduces the case with a server that trickles bytes forever, which
no per-operation timeout can catch.

The verdict is snapshotted before the restart. Tearing the server down can
unblock the worker, and a late answer arriving afterwards must not overturn a
decision the deadline has already made.

Output is deliberately **not** grammar-constrained. llama.cpp can force
schema-valid JSON, which would make the invalid-action rate zero by
construction and hide exactly the differences Phase 12 exists to measure.

### Measured behaviour

Qwen3-4B-Instruct-2507 Q5_K_M, three identical runs of the same episode at
`temperature: 0.0`, on the development host with nothing else on the GPU:

| Decision | Latency |
| --- | --- |
| First | 4257, 4563 and 7204 ms |
| All later | 1127-1259 ms |

Proposals were **identical across all three runs**; only latency varied. The
first decision is 3.5-6x the steady-state cost and straddles the 5000 ms
deadline: it timed out in one run of three. Warming the fixed system prompt at
startup reduces this but does not remove it, and shaping the warmup like a
real observation measured worse rather than better.

This is a deployment constraint, not a model property. A decision loop should
not treat its first decision as representative, and a deadline set from
steady-state numbers alone will fire spuriously. It is also a concrete example
of why Phase 12 requires repeated runs: a single run would have reported
either a clean pass or a timeout, and both would have been misleading.

### What the model actually proposed

Against a freshly captured stress episode, with the recorded action shown for
comparison only:

| Recorded | Proposed | Note |
| --- | --- | --- |
| arm | arm | agrees |
| takeoff | takeoff, 5.0 m | agrees on action; the flight used 3 m |
| goto | hold | `goto` is not in the vocabulary, so no agreement was possible |
| hold | hold | agrees |
| land | **takeoff, 5.0 m** | see below |

Two findings matter more than the agreement count.

**The vocabulary is narrower than the baseline.** The recorded flight used
`goto`, which the model cannot propose. That decision point cannot be scored
as agreement or disagreement, and any future evaluation must exclude such
points rather than count them as model failures.

**A schema-valid proposal can still be the wrong thing to do.** At the final
decision point the vehicle was airborne at 2.77 m, and the previous action had
just terminated `ACTION_STATE_ABORTED_BY_SAFETY` with
`REASON_CODE_SAFETY_INTERVENTION`. The recorded flight landed. The model
proposed climbing to 5 m, deterministically, in every run. The proposal passes
every syntactic and bounds check the contract applies, because the contract
checks shape and limits rather than whether an action makes sense in context.

Nothing was executed; observe mode has no path to the aircraft. But this is
the concrete reason approval mode and independent guardrails must precede any
control authority, and it should be read as evidence about an unprompted 4B
model on a bare contract, not as a verdict on the model.

## First corpus evaluation

Ten episodes across empty, wind, obstacle, adverse and perception-stress
profiles, three runs each, 123 decision points:

| Measure | Result |
| --- | --- |
| Invalid proposals | 0 of 117 asked |
| Runtime errors | 0 |
| Timeouts | 1 (the cold start) |
| Stale refusals | 6 |
| Agreement | 74.8% of 107 comparable points |
| Unscoreable | 9 (`goto`, outside the vocabulary) |
| Deterministic | 9 of 10 episodes |
| Executed actions | 0 |

The model never emitted malformed output. Every proposal across the whole
corpus parsed, named an allowed action and satisfied its argument bounds.

### The headline number is misleading

74.8% agreement hides a systematic failure that only the per-action breakdown
shows:

| Recorded | Proposed | Count |
| --- | --- | --- |
| takeoff | takeoff | 30 |
| arm | arm | 26 |
| hold | hold | 24 |
| land | **takeoff** | 18 |
| land | **hold** | 9 |
| goto | hold | 9 (unscoreable) |

**The model never once proposed `land`.** At all 27 land decision points it
proposed climbing or holding instead, and it did so deterministically. Its
agreement on `arm`, `takeoff` and `hold` is total; its agreement on `land` is
zero. A single aggregate score would have reported a passable 75% for a model
that cannot end a flight.

This is the argument against ranking models by one number, made concrete
inside this project rather than borrowed from a paper. The Phase 12 campaign
must report per-action breakdowns, not a combined score.

The cause is not established. The prompt says to reply `none` when unsure but
gives no guidance on when a flight should end, and the observation carries no
mission-progress or remaining-objective field. A model with no notion that the
mission is over has no reason to land. That is a hypothesis about the contract
and the prompt, not a measured property of Qwen, and it needs a controlled
test rather than a plausible story.

### Latency

| Measure | Result |
| --- | --- |
| Cold start (once per campaign) | 9370 ms |
| Episode first decision | median 343 ms, max 780 ms |
| Steady state | median 540 ms, max 823 ms |

The cold start exceeds the 5000 ms deadline outright, so the first decision
after a server start always times out. Warming the system prompt reduced it
but never removed it, and it measured between 4257 and 9370 ms across
sessions. A deployed loop must make a throwaway decision before the mission
starts rather than set a deadline loose enough to cover it, since a deadline
that tolerates 9 s is no longer protecting anything.

Episode-first decisions are *faster* than steady state, because early-mission
observations are shorter than later ones. That is a property of the prompt,
not of the runtime.

Next: a Phase 12 campaign with frozen scenarios, held-out episodes, a
deterministic no-LLM baseline and at least two models, reported per action.
Investigate the `land` result with a controlled prompt change before reading
anything into it. Only after that should approval mode or closed-loop SITL
control be considered. Hardware control remains out of scope.

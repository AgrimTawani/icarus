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

Next: implement the llama.cpp adapter behind `ModelRuntime`, enforcing a hard
deadline at the process boundary rather than reporting lateness after the fact,
and evaluate proposals on held-out episodes. Only after that should approval
mode or closed-loop SITL control be considered. Hardware control is out of
scope.

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

Next: implement the provider-neutral model adapter, connect a local Qwen
runtime, freeze prompt/model metadata, and evaluate proposals on held-out
episodes. Only after that should approval mode or closed-loop SITL control be
considered. Hardware control is out of scope.

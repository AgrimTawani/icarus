# Next Steps / Agent Handoff

Status as of 2026-09-19. Read this with [`MASTER-PLAN.md`](MASTER-PLAN.md) and
[`reference/PROJECT-STATUS.md`](reference/PROJECT-STATUS.md). This is a handoff,
not a claim that Phase 10 or Phase 11 has passed its full exit gate.

## What is verified now

Re-verified 2026-09-19 after the corpus run: 63 Python unit tests, both C++
suites, the Phase 10 replay gate on every usable episode, and five
consecutive headless flights across scenario profiles.

- Phases 0–9 are reported complete in the project plan.
- The Phase 10 **simulation slice** records sealed episodes and replays the
  recorded actions through native C++ guardrails. Its unit gate and replay of
  `logs/episodes/20260919T194115_6626de1c0e21` passed on 2026-09-19.
- The first Phase 11 **observe-only wiring slice** is in
  `python/dcm/observe.py` and `scripts/observe-dcm`. It consumes a sealed episode,
  calls a mock runtime at pre-action points, strictly parses a narrow action
  schema, and writes a separate report. The mock always proposes `none`; it is
  not Qwen, a trained DCM, or evidence of mission success. On the stress
  episode it produced four valid mock proposals, refused one decision point as
  stale, and executed **zero actions**. The refused point is the one the
  episode records as `REASON_CODE_SAFETY_INTERVENTION`.
- The **model contract** is in `python/dcm/contract.py` behind explicit
  versions, with 36 unit tests. The **model artifacts** are downloaded and
  pinned; llama.cpp is built with CUDA at a pinned revision. No adapter
  connects them yet, so no model has proposed anything.
- Python unit tests and the existing Phase 8/9 C++ tests passed after this
  slice. No GUI or flight was launched for the observe-mode test.

Recheck the local baseline:

```bash
./scripts/test-phase10 logs/episodes/20260919T194115_6626de1c0e21
.venv/bin/python -m unittest discover -s tests/unit -p 'test_*.py' -v
ctest --test-dir build/phase8 --output-on-failure
./scripts/observe-dcm logs/episodes/20260919T194115_6626de1c0e21
```

The example episode is a **local ignored artifact** and will not exist on a
fresh clone. Use another sealed episode under `logs/episodes/` or generate one
with the documented simulation mission workflow. Observe reports are under
`logs/dcm/observe/` and are also local/ignored.

## Immediate implementation order

1. ~~**Review and stabilize the existing worktree.**~~ **Done 2026-09-19.**
   Phase 10 and the observe slice are committed. Phase 10's complete mission
   coverage, full planner/MAVLink trace, physical-flight data governance and
   full exit gate remain open; those hardware and data-policy items do not
   block offline Phase 11 work.
2. ~~**Make a provider-neutral model contract.**~~ **Done 2026-09-19.**
   `python/dcm/contract.py` holds the curated observation, the action table,
   the generated prompt, freshness limits and `RuntimeDescriptor`, behind
   versions `dcm-contract-v1`, `dcm-actions-v1` and `dcm-prompt-v1`. `curate`
   cannot receive the recorded next action. 36 unit tests cover invalid JSON,
   unknown actions, unsafe arguments, stale data and runtime failure. See
   `docs/architecture/DCM-OBSERVE-V1.md`.
3. ~~**Connect the first local model in observe mode.**~~ **Done 2026-09-19.**
   `python/dcm/llama_runtime.py` runs the pinned Qwen3-4B Q5_K_M behind a
   wall-clock deadline that abandons the request and restarts the server.
   `./scripts/observe-dcm <episode> --runtime llama`. Read the measured
   behaviour in `docs/architecture/DCM-OBSERVE-V1.md` before trusting any
   number from it: the first decision is 3.5-6x steady state and timed out in
   one run of three, and at one decision point the model proposed climbing
   immediately after a safety abort where the flight landed. Add a replaceable llama.cpp-compatible
   adapter; pin and record model checksum, quantization, prompt, sampling
   settings and latency. Enforce a real process/network deadline, not merely
   the current post-return timeout check. Do not install OpenClaw or ZeptoClaw
   as a prerequisite.
4. ~~**Build the offline evaluation harness.**~~ **Done 2026-09-19.**
   `python/dcm/evaluate.py` and `./scripts/evaluate-dcm` replay a corpus with
   repeats and report invalid/timeout/error rates, agreement over comparable
   points, stale refusals, latency distributions and determinism.
   `./scripts/fly-episode-corpus` builds the corpus headlessly across scenario
   profiles. Points whose recorded action is outside `contract.ACTIONS` are
   reported as unscoreable rather than as model failures.

   What remains is the **campaign**, not the machinery: a frozen scenario set
   with held-out episodes, a deterministic no-LLM baseline, and at least two
   models compared. See `docs/architecture/DATA-AND-EVALUATION.md`. Compare proposals to the scripted
   baseline without treating that baseline as a perfect label. Report invalid
   actions, timeouts, model errors, safety-rule outcomes and latency. Keep eval
   episodes excluded from training exports.
5. ~~**Investigate the `land` result.**~~ **Done 2026-09-20.** The cause was
   the contract, not the model: the v1 observation never said what had already
   been done. Contract v2 (`--history`) adds the completed-action sequence and
   cut `land -> takeoff` from 18 to 1 while raising `land -> land` from 0 to 9,
   touching no other action.

   Three further variants were then run, each changing one thing. Mission
   elapsed time made it **worse** (3/9 correct landings down to 1/9) and is
   not recommended: the episodes record no target altitude or hover duration,
   so there is nothing for a bare time count to measure against. A directive
   ending-guidance prompt scored best but produced the only invalid output
   seen in this work, twice, and increased climbing after a safety abort. Q4
   scored higher than Q5 on every axis, contradicting the reasoning that
   selected Q5.

   **Do not act on any of those three.** They separate by two or three
   situations out of nine, which is not evidence. Only the history result is
   established, and only because its effect is large: dangerous climbs fell
   from 18 to 1.

6. **Grow the corpus before deciding anything else.** Nine distinct
   mission-ending decision situations cannot separate a 3/9 from a 5/9. This
   is now the binding constraint on every open question — default contract,
   quantization, prompt wording. `./scripts/fly-episode-corpus --repeat N`
   already does the work; it needs more varied missions and more failure
   cases, not new code.

7. **Only then add mission orchestration and controlled SITL modes.** Add
   persistent mission state, bounded reusable sequences and recovery logic.
   Progress from observe mode to explicit operator approval, then autonomous
   simulation behind the same Drone API, guardrails and safety supervisor. The
   model must never receive shell, raw MAVLink, motor or safety-policy access.
8. **Run the Phase 12 scenario campaign.** Freeze seeds, prompts, model and
   policy versions; compare models against the deterministic baseline across
   wind, obstacles, sensor dropouts, link faults and recovery. Hardware work
   remains a separate later gate.

## Needed from the owner

Nothing blocks the current work. The Jetson module is confirmed as **AGX Orin
64GB** (2026-09-20). Its developer kit ships with 64 GB eMMC and no SSD, which
is enough for one deployed model but not for holding several artifacts during a
comparison, so a 512 GB M.2 2280 NVMe Gen4 x4 drive is recommended alongside
the plain kit; see `docs/architecture/DCM-MODEL-SELECTION.md`. Hardware work
remains gated behind Phase 13 and none of it is needed yet.
No Claw framework installation or hardware-flight approval is needed now.

## Boundaries and caveats

`python/dcm/observe.py` is offline only and imports neither the Drone API nor
MAVLink. Its initial action vocabulary is deliberately smaller than the Drone
API (`none`, `arm`, `takeoff`, `hold`, `return_home`, `land`). It checks response
shape, not the complete flight safety policy. A future live DCM must go through
the existing API and independent guardrails. The current timeout is measured
after the mock returns; an actual model adapter needs an enforceable deadline.
Neither Phase 10 nor Phase 11 is complete, and nothing here authorizes physical
autonomous flight.

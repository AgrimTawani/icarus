# Next Steps / Agent Handoff

Status as of 2026-09-19. Read this with [`MASTER-PLAN.md`](MASTER-PLAN.md) and
[`reference/PROJECT-STATUS.md`](reference/PROJECT-STATUS.md). This is a handoff,
not a claim that Phase 10 or Phase 11 has passed its full exit gate.

## What is verified now

- Phases 0–9 are reported complete in the project plan.
- The Phase 10 **simulation slice** records sealed episodes and replays the
  recorded actions through native C++ guardrails. Its unit gate and replay of
  `logs/episodes/20260919T194115_6626de1c0e21` passed on 2026-09-19.
- The first Phase 11 **observe-only wiring slice** is in
  `python/dcm/observe.py` and `scripts/observe-dcm`. It consumes a sealed episode,
  calls a mock runtime at pre-action points, strictly parses a narrow action
  schema, and writes a separate report. The mock always proposes `none`; it is
  not Qwen, a trained DCM, or evidence of mission success. The stress episode
  produced five valid mock proposals and **zero executed actions**.
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

1. **Review and stabilize the existing worktree.** Phase 10 and the observe
   slice are currently uncommitted alongside other modifications. Inspect the
   diff before staging; do not discard or silently commit unrelated changes.
   Re-run the checks above. Phase 10's complete mission coverage, full planner/
   MAVLink trace, physical-flight data governance, and full exit gate remain
   open. Those hardware/data-policy items do not block offline Phase 11 work.
2. **Make a provider-neutral model contract.** Keep the curated observation,
   allowed-action vocabulary, prompt/schema version and runtime metadata
   explicit. Preserve the rule that the recorded next action is comparison
   data, never part of the model input. Add tests for invalid JSON, unknown
   actions, unsafe arguments, stale data and model failure.
3. **Connect the first local model in observe mode.** The user will provide the
   absolute path to a Qwen GGUF artifact. Add a replaceable llama.cpp-compatible
   adapter; pin and record model checksum, quantization, prompt, sampling
   settings and latency. Enforce a real process/network deadline, not merely
   the current post-return timeout check. Do not install OpenClaw or ZeptoClaw
   as a prerequisite.
4. **Evaluate offline before control.** Replay multiple successful and failed
   episodes, including held-out cases. Compare proposals to the scripted
   baseline without treating that baseline as a perfect label. Report invalid
   actions, timeouts, model errors, safety-rule outcomes and latency. Keep eval
   episodes excluded from training exports.
5. **Only then add mission orchestration and controlled SITL modes.** Add
   persistent mission state, bounded reusable sequences and recovery logic.
   Progress from observe mode to explicit operator approval, then autonomous
   simulation behind the same Drone API, guardrails and safety supervisor. The
   model must never receive shell, raw MAVLink, motor or safety-policy access.
6. **Run the Phase 12 scenario campaign.** Freeze seeds, prompts, model and
   policy versions; compare models against the deterministic baseline across
   wind, obstacles, sensor dropouts, link faults and recovery. Hardware work
   remains a separate later gate.

## Needed from the owner

Provide the absolute local path to the first Qwen model artifact once
downloaded. If the target Jetson is selected, provide its model and RAM size.
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

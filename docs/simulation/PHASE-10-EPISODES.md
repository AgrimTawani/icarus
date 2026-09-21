# Phase 10 — Flight Episodes and Offline Replay

Status: simulation episode capture and native guardrail replay implemented and
tested headlessly on 2026-09-19. This is the data foundation for Phase 11
observe-mode model integration, not permission for autonomous hardware flight.

## What is recorded

The Drone API mission clients create one directory in `logs/episodes/` per run.
The write-once `manifest.json` contains the episode ID, mission, source, Git
revision, scenario/seed, actual source-tree/config hashes, outcome, score and privacy/training
status. `events.jsonl` is ordered by a single sequence with Unix and monotonic
timestamps. It stores 10 Hz state and perception summaries, action requests,
server validation results with exact input snapshots, action receipts and
statuses, and vehicle events. Frozen safety/scenario configuration is copied
into `config/`. Identifiers are redacted; raw image/LiDAR payloads are not
captured. Training status defaults to `unreviewed_do_not_train`.
The seal is a local integrity check, not a cryptographic signature or remote
write-once archive. Off-device retention and adversarial tamper protection are
future real-flight data-governance work.

The recorder is attached to `MissionClient`, so `run-mission`, `test-phase8`
and `test-phase9` all produce episodes, including failed runs. New DCM clients
must use this client boundary or implement the same episode schema; a model
must never bypass the Drone API.

## Replay

```bash
./scripts/replay-episode --complete logs/episodes/<episode-id>
```

This checks hashes, sequence/timestamp monotonicity, stream alignment,
request/receipt pairs, terminal action statuses and state/perception presence.
It also loads the frozen safety policy and re-runs every recorded command
through the same compiled C++ `Guardrails::Validate` implementation, comparing
validity, reason and message. Perception-stale safety aborts are checked against
the sensor observation age in the time-aligned record. Other action/safety
outcomes remain recorded terminal evidence, not a full physics or planner
re-simulation. Replay does not need Gazebo or SITL.

Headless acceptance on 2026-09-19:

- Phase 9 stress flight: 664 ordered records, 319 state/perception samples,
  five terminal actions, and recorded LiDAR-staleness evidence at 859 ms.
- Takeoff-hover-land flight: 416 ordered records and four native guardrail
  decisions replayed exactly.
- A later failed stress attempt produced a sealed failed episode, including
  `destination intersects inflated obstacle`, and replayed without dropping
  its failure trace.
- A fresh stress run after full roll/pitch/yaw LiDAR normalization passed:
  zero collisions, 1.929 m clearance, 13.53 m/s peak wind, controlled LiDAR
  dropout and recovery. Its 677-record episode replayed all five native
  guardrail decisions and confirmed perception was 811 ms old at the safety
  abort. Local evidence: `logs/episodes/20260919T194115_6626de1c0e21`.
- Unit tests prove a tampered stream and a missing state stream fail replay.

`./scripts/test-phase10` runs the unit gate. Pass an episode directory to it
to require the current complete-artifact contract and run native guardrail
replay twice deterministically, e.g.
`./scripts/test-phase10 logs/episodes/<episode-id>`.

For a closed-loop DCM flight, score the outcome after it is sealed; this is
read-only and never starts Gazebo or contacts the drone:

```bash
./scripts/evaluate-live-episodes logs/episodes/<episode-id>
./scripts/phase12-campaign logs/episodes/<episode-id>
```

The live score reports mission outcome only when the mission wrapper recorded
one. It will report `null`, not pretend success, for a semantic-only episode.

To make a separate candidate-training view, run
`./scripts/export-candidates logs/episodes/<episode-id> --output logs/datasets/candidates/<episode-id>.jsonl`.
Every row is unapproved and prohibited from training by default; successful
flight is not automatically a preferred action label, and acceptance/evaluation
episodes are marked as such.

The Phase 10 contract supports adding a local Qwen/Llama DCM in **observe mode**.
Before hardware use or training, complete real-flight privacy/retention review,
operator approval workflow, and physical sensor adapter validation. Raw logs
must never be promoted directly into training data.

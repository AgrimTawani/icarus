# Icarus Documentation

This directory is the project handbook. It separates current decisions from
implementation evidence and historical ideas so a new engineer or agent can
quickly establish the truth of the repository.

## Recommended Reading Order

1. [`GOAL.md`](GOAL.md) — what Icarus is building and what success means.
2. [`MASTER-PLAN.md`](MASTER-PLAN.md) — phases, gates and immediate work.
3. [`NEXT-STEPS.md`](NEXT-STEPS.md) — current agent handoff and next actions.
4. [`reference/PROJECT-STATUS.md`](reference/PROJECT-STATUS.md) — what works now.
5. [`architecture/SOFTWARE-ARCHITECTURE.md`](architecture/SOFTWARE-ARCHITECTURE.md)
   — component boundaries, authority and deployment design.
6. [`architecture/SIMULATION-CONNECTIONS.md`](architecture/SIMULATION-CONNECTIONS.md)
   — Gazebo, SITL, MAVLink and sensor connections.
7. [`architecture/DRONE-API-V1.md`](architecture/DRONE-API-V1.md) — Phase 8
   command, authority, state and extension contract.
8. [`simulation/PHASE-8-ACCEPTANCE.md`](simulation/PHASE-8-ACCEPTANCE.md) —
   Drone API flight and injected-failure exit-gate evidence.
9. [`simulation/PHASE-9-ACCEPTANCE.md`](simulation/PHASE-9-ACCEPTANCE.md) —
   headless live-LiDAR obstacle-avoidance evidence.
10. [`guides/GETTING-STARTED.md`](guides/GETTING-STARTED.md) — recreate the local
   environment.
11. [`guides/SIMULATION-OPERATIONS.md`](guides/SIMULATION-OPERATIONS.md) — run,
   test and stop scenarios.

For the current data and DCM work, continue with
[`simulation/PHASE-10-EPISODES.md`](simulation/PHASE-10-EPISODES.md) and
[`architecture/DCM-OBSERVE-V1.md`](architecture/DCM-OBSERVE-V1.md).

## Documentation Map

The top level contains project direction, the live handoff, and this index. Six focused
folders keep the supporting material navigable:

| Folder | Contents |
| --- | --- |
| `architecture/` | Software design, simulation data flow, evaluation design and ADRs |
| `guides/` | Setup, development and simulation operation procedures |
| `reference/` | Current status, dependencies, missions, repository and upstream versions |
| `vehicle/` | Airframe, STL, packaging and BOM engineering |
| `simulation/` | Bring-up details and dated acceptance evidence |
| `concept/` | Historical and exploratory plans; not implementation authority |

## Source-of-Truth Rules

When documents disagree, use this precedence:

1. Executable configuration, schema and tests define machine behavior.
2. `reference/PROJECT-STATUS.md` defines the verified implementation state.
3. `architecture/SOFTWARE-ARCHITECTURE.md` defines component boundaries.
4. `MASTER-PLAN.md` defines sequencing and exit gates.
5. Simulation acceptance reports provide dated evidence, not future promises.
6. Material under `concept/` is exploratory and not authoritative.

Every material change should update the relevant canonical document and add or
update an automated test. Do not report a phase complete without a reproducible
command and retained machine-readable evidence.

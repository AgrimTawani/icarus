# Icarus Documentation

This directory is the project handbook. It separates current decisions from
implementation evidence and historical ideas so a new engineer or agent can
quickly establish the truth of the repository.

## Recommended Reading Order

1. [`GOAL.md`](GOAL.md) — what Icarus is building and what success means.
2. [`MASTER-PLAN.md`](MASTER-PLAN.md) — phases, gates and immediate work.
3. [`reference/PROJECT-STATUS.md`](reference/PROJECT-STATUS.md) — what works now.
4. [`architecture/SOFTWARE-ARCHITECTURE.md`](architecture/SOFTWARE-ARCHITECTURE.md)
   — component boundaries, authority and deployment design.
5. [`architecture/SIMULATION-CONNECTIONS.md`](architecture/SIMULATION-CONNECTIONS.md)
   — Gazebo, SITL, MAVLink and sensor connections.
6. [`guides/GETTING-STARTED.md`](guides/GETTING-STARTED.md) — recreate the local
   environment.
7. [`guides/SIMULATION-OPERATIONS.md`](guides/SIMULATION-OPERATIONS.md) — run,
   test and stop scenarios.

## Documentation Map

The top level contains only the project direction and index. Six focused
folders keep the supporting material navigable:

| Folder | Contents |
| --- | --- |
| `architecture/` | Software design, simulation data flow, evaluation design and ADRs |
| `guides/` | Setup, development and simulation operation procedures |
| `reference/` | Current status, dependencies, missions, repository and upstream versions |
| `vehicle/` | Airframe, STL, packaging and BOM engineering |
| `simulation/` | Bring-up details and dated acceptance evidence |
| `archive/` | Superseded plans kept only for historical context |

## Source-of-Truth Rules

When documents disagree, use this precedence:

1. Executable configuration, schema and tests define machine behavior.
2. `reference/PROJECT-STATUS.md` defines the verified implementation state.
3. `architecture/SOFTWARE-ARCHITECTURE.md` defines component boundaries.
4. `MASTER-PLAN.md` defines sequencing and exit gates.
5. Simulation acceptance reports provide dated evidence, not future promises.
6. Material under `archive/` is never authoritative.

Every material change should update the relevant canonical document and add or
update an automated test. Do not report a phase complete without a reproducible
command and retained machine-readable evidence.

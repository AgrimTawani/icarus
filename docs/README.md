# Icarus Documentation

This directory is the project handbook. It separates current decisions from
implementation evidence and historical ideas so a new engineer or agent can
quickly establish the truth of the repository.

## Recommended Reading Order

1. [`GOAL.md`](GOAL.md) — what Icarus is building and what success means.
2. [`MASTER-PLAN.md`](MASTER-PLAN.md) — phases, gates and immediate work.
3. [`PROJECT-STATUS.md`](PROJECT-STATUS.md) — what works now.
4. [`SOFTWARE-ARCHITECTURE.md`](SOFTWARE-ARCHITECTURE.md)
   — component boundaries, authority and deployment design.
5. [`SIMULATION-CONNECTIONS.md`](SIMULATION-CONNECTIONS.md)
   — Gazebo, SITL, MAVLink and sensor connections.
6. [`GETTING-STARTED.md`](GETTING-STARTED.md) — recreate the local
   environment.
7. [`SIMULATION-OPERATIONS.md`](SIMULATION-OPERATIONS.md) — run,
   test and stop scenarios.

## Simple Naming Map

All project documentation is kept directly in this directory. Canonical files
use plain names such as `GOAL.md`, `MASTER-PLAN.md` and
`SOFTWARE-ARCHITECTURE.md`. Supporting records use readable prefixes:

- `SIM-*` for simulation bring-up and acceptance records;
- `VEHICLE-*` for airframe and component engineering;
- `ADR-*` for architecture decisions;
- `ARCHIVE-*` for superseded historical material.

## Source-of-Truth Rules

When documents disagree, use this precedence:

1. Executable configuration, schema and tests define machine behavior.
2. `PROJECT-STATUS.md` defines the verified implementation state.
3. `SOFTWARE-ARCHITECTURE.md` defines intended component boundaries.
4. `MASTER-PLAN.md` defines sequencing and exit gates.
5. Simulation acceptance reports provide dated evidence, not future promises.
6. Files prefixed `ARCHIVE-` are never authoritative.

Every material change should update the relevant canonical document and add or
update an automated test. Do not report a phase complete without a reproducible
command and retained machine-readable evidence.

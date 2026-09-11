# Development and Contribution Guide

## Working Rules

- Start from `GOAL.md`, `MASTER-PLAN.md` and `PROJECT-STATUS.md`.
- Preserve the authority boundary in the software architecture.
- Add measurable pass/fail criteria before implementation.
- Keep machine-specific paths and secrets out of code and configuration.
- Do not claim a scaffold, visual demo or single run as a completed capability.

## Before Committing

Run the checks relevant to the change. For simulator declarations and builders:

```bash
./scripts/simulation/validate_sdf.sh
/usr/bin/python3 scripts/simulation/test_phase5_scenarios.py
```

For Python changes, use the project environment when available:

```bash
.venv/bin/ruff check .
.venv/bin/pytest
```

Do not commit logs, generated worlds/models, virtual environments, external
checkouts or build products. Update `PROJECT-STATUS.md` only after a
gate has repeatable evidence; update `MASTER-PLAN.md` when sequencing changes;
add an ADR when authority or a major boundary changes.

## Commit Scope

Keep changes reviewable. A feature commit should include its contract,
implementation, configuration, test and documentation when applicable. Never
mix generated acceptance artifacts with source unless a small, curated report
is needed to preserve a conclusion.

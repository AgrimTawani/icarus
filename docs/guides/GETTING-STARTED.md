# Icarus Getting Started

## Supported Development Host

The current bootstrap path supports Ubuntu 24.04 x86_64. The verified laptop
has 16 GiB RAM and an RTX 4050 6 GiB GPU. At least 30 GiB free disk space is
recommended because ArduPilot, Gazebo, Python ML packages and build products are
not small.

## Clone and Inspect

```bash
git clone <repository-url> icarus
cd icarus
```

Read [`../reference/DEPENDENCIES.md`](../reference/DEPENDENCIES.md) before
running the installer. The script uses `sudo`, configures the OSRF package
repository, installs system packages, clones official ArduPilot projects,
builds SITL and the Gazebo plugin, creates two Python environments, installs
Ollama and downloads `qwen3:4b`. It is therefore a deliberate machine setup
operation, not a lightweight project install.

## Full Bootstrap

```bash
./scripts/install_dependencies.sh
```

External repositories are created under `third_party/`, project Python packages
under `.venv/`, and ArduPilot Python packages under
`third_party/ardupilot/.venv/`. All are local and ignored by Git. The installer
currently follows upstream branches when rerun; the known-good revisions are
recorded in [`../reference/THIRD-PARTY.md`](../reference/THIRD-PARTY.md), and an
executable pinning mechanism is a Phase 7 task.

## Validate the Simulation Assets

```bash
./scripts/simulation/validate_sdf.sh
/usr/bin/python3 scripts/simulation/test_phase5_scenarios.py
```

Then run the lightweight integrated scenario:

```bash
./scripts/sim --scenario wind_light
```

Use `--gui` only when a desktop display is available. The current launcher runs
an automated takeoff-hover-land mission; interactive/manual flight is a Phase 6
deliverable and is not yet a supported switch.

## Expected Local Artifacts

- `third_party/`: official external source checkouts and builds;
- `.venv/`: project Python environment;
- `simulation/worlds/generated/`: deterministic generated worlds;
- `simulation/models/phase5_*`: scenario-specific generated model copies;
- `logs/`: run logs, telemetry and acceptance results.

These paths are intentionally ignored. A fresh clone recreates them from the
committed scripts and declarations.

## Common Failure Checks

- A GUI launch requires `DISPLAY` or Wayland/X11 access.
- TCP 5760 and UDP 9002 must be free before launch.
- A rejected scenario intentionally exits before starting Gazebo or arming.
- Do not start a second launcher while one owns the ports.
- If a run is interrupted, allow the launcher to clean up before retrying.

For scenario behavior and pass criteria, continue with
[`SIMULATION-OPERATIONS.md`](SIMULATION-OPERATIONS.md).

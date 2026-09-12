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
running the bootstrap. System installation uses `sudo`, so it is a deliberate
machine setup operation. Model and fine-tuning packages are separated from the
normal development and simulation paths.

## Select a Bootstrap Profile

```bash
# Lint, unit tests and protobuf tools only
./scripts/bootstrap --profile dev

# Development tools plus pinned ArduPilot SITL and Gazebo
./scripts/bootstrap --profile simulation

# Optional model/evaluation Python environment, without simulation
./scripts/bootstrap --profile ml

# Everything (the legacy installer delegates here)
./scripts/bootstrap --profile all
```

External repositories are created under `third_party/`, project Python packages
under `.venv/`, and ArduPilot Python packages under
`third_party/ardupilot/.venv/`. All are local and ignored by Git. Exact upstream
commits are enforced by `config/dependencies/third_party.lock.json`; direct
Python dependencies are divided and version-pinned under `requirements/`.

Validate an existing installation without changing it:

```bash
./scripts/check-workspace --scope dev
./scripts/check-workspace --scope simulation
```

Docker definitions are available at `containers/Dockerfile.dev` and
`containers/Dockerfile.simulation`. The development image is suitable for CI.
The simulation image needs the usual host display/GPU forwarding for a GUI;
headless operation needs neither. Xbox/USB devices are host resources and must
be passed explicitly by the container operator.

## Validate the Simulation Assets

```bash
./scripts/simulation/validate_sdf.sh
/usr/bin/python3 scripts/simulation/test_phase5_scenarios.py
```

Then run the lightweight integrated scenario:

```bash
./scripts/sim --profile simulation-wind
```

Use `--gui` only when a desktop display is available. For manual operation,
replace `scripts/sim` with `scripts/start-sim`, then connect
`scripts/manual-control` and optionally `scripts/view-camera` from independent
terminals. Run `./scripts/start-sim --list-profiles` to inspect named profiles.

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

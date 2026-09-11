# Icarus Simulation Workspace

All project-owned Gazebo assets live here. Nothing in `third_party/` may be
edited to implement an Icarus vehicle, world, parameter, scenario, or test.

## Directories

- `models/`: SDF models, primitive collision geometry, and project-owned meshes.
- `worlds/`: Gazebo worlds that include project-owned models.
- `parameters/`: ArduPilot SITL parameter baselines and scenario overlays.
- `scenarios/`: Versioned mission, environment, fault, and seed definitions.
- `launch/`: Declarative launch profiles consumed by repository scripts.
- `tests/`: Simulation validation and regression-test inputs.

## Asset Policy

Each imported asset must include its source URL, exact revision or version,
license, and any local modification. Original Icarus files use the repository's
project license once one is selected. Until then, do not redistribute imported
meshes or textures.

Validate every SDF file with:

```bash
./scripts/simulation/validate_sdf.sh
```

Phase 5 scenarios are described in [scenarios/README.md](scenarios/README.md).
Generate a deterministic world with `scripts/simulation/build_phase5_world.py`
or run it through `./scripts/sim --scenario <name>`.

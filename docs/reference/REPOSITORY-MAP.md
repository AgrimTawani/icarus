# Repository Map

```text
Icarus/
├── proto/icarus/v1/       Versioned API message contracts
├── cpp/                   Deterministic autonomy services (currently scaffolds)
├── python/                DCM, model, evaluation and dataset code (scaffolds)
├── perception/            Sensor/perception modules (scaffolds)
├── simulation/            Vehicle models, worlds, scenarios, parameters, plugins
├── config/                Safety and simulation configuration
├── tests/                 Cross-component test layout (currently scaffolds)
├── scripts/               Stable operator/bootstrap entry points and builders
├── docs/                  Canonical handbook, evidence and history
├── logs/                  Local generated run artifacts; ignored
└── third_party/           Local external source checkouts/builds; ignored
```

## Source Versus Generated Content

Committed sources include scenario JSON, environment/sensor configuration,
custom/model SDF and mesh assets, builders, launch/test scripts, parameters,
protobuf definitions and documentation.

The following are recreated locally and must not be committed:

- `.venv/` and tool caches;
- `third_party/` clones and all upstream build products;
- `logs/` and flight recordings;
- `simulation/worlds/generated/`;
- `simulation/models/phase5_*` scenario copies;
- CMake and plugin build directories.

## Ownership Rules

- Never modify `third_party/` to implement Icarus behavior.
- Edit scenario declarations or builders, not generated scenario assets.
- Keep simulator/hardware details behind adapters; do not import Gazebo concepts
  into mission or model contracts.
- Component-local `README.md` files may stay beside code for immediate operating
  details. Canonical cross-project documentation belongs under `docs/`.
- Large raw recordings belong in external artifact storage once that system is
  selected, referenced by immutable manifest rather than committed to Git.

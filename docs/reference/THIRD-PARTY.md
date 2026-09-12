# Icarus Third-Party Source Revisions

External source trees are deliberately ignored and recreated beneath
`third_party/` by `scripts/install_dependencies.sh`. Icarus-owned changes must
not be made inside those trees.

## Known-Good Local Revisions

Recorded on 2026-09-12:

| Project | Official source | Commit |
| --- | --- | --- |
| ArduPilot | `https://github.com/ArduPilot/ardupilot.git` | `14c871f2732ccc5f5f558d003eb46fb2fe29d832` |
| ArduPilot Gazebo | `https://github.com/ArduPilot/ardupilot_gazebo.git` | `082a0fe231f6e63bc8d1598f1cba461d9e2ea7f5` |

These commits are enforced by
`config/dependencies/third_party.lock.json`. The bootstrap fetches and checks
out those immutable revisions instead of following upstream branches. Existing
dirty third-party checkouts are rejected rather than overwritten. To inspect
the active revisions:

```bash
./scripts/check-workspace --scope simulation
```

Review and comply with each upstream project's license. The supplied/custom
mesh provenance is documented separately in the `VEHICLE-*` documents and
`simulation/models/` metadata.

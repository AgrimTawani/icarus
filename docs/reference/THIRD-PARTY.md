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

These are the commits used by the verified simulation environment, but the
current installer clones or fast-forwards the upstream default branches. Until
Phase 7 implements an executable lock, reproduce the known state manually after
bootstrap when exact results matter:

```bash
git -C third_party/ardupilot checkout 14c871f2732ccc5f5f558d003eb46fb2fe29d832
git -C third_party/ardupilot submodule update --init --recursive

git -C third_party/ardupilot_gazebo checkout 082a0fe231f6e63bc8d1598f1cba461d9e2ea7f5
```

Review and comply with each upstream project's license. The supplied/custom
mesh provenance is documented separately in the `VEHICLE-*` documents and
`simulation/models/` metadata.

# Icarus7 AirSim vehicle module

This directory converts the reviewed `Icarus7_FlightControl_Review` UE4.27
content project into an AirSim 1.8 vehicle integration. The delivered project
is a visual asset source, not a runnable simulator: it has no AirSim plugin,
pawn, level, runtime sensors, physics, complete-vehicle collision or packaged
Linux binary.

## What the module supplies

- A native `AIcarus7AirSimPawn` derived from AirSim's `AFlyingPawn`.
- The five exact camera component names required by AirSim 1.8.
- Four actuator-driven rotor pivots in ArduPilot Quad-X order: FR, RL, FL, RR.
- Conservative compound body/arm/landing collision; propeller discs remain
  visual and non-colliding.
- Reviewed forward and downward camera positions, LiDAR, GPS/RTK and downward
  range positions in body FRD.
- A pinned AirSim 1.8 physics overlay selected only by `"Model": "Icarus7"`.
- Component-derived 4.4745 kg mass and FRD inertia, canonical rotor positions,
  10-inch propeller geometry, the frozen 15,606 RPM thrust point and the
  existing 0.016 m moment/thrust ratio.
- Separate headless and GUI settings, an idempotent installer, structural
  validator, AirLib executable parameter test, packaging command and guarded
  launcher.

The module never changes the DCM or control authority. The custom pawn is still
an `ArduCopter`; Icarus commands continue through the Drone API, ArduPilot SITL
and AirSim's actuator/sensor boundary.

Current runtime status: the source integration and AirLib physics test are
complete, but this pawn has not yet been compiled by Unreal Engine or flown.
The successful realistic AirVLN mission used that packaged executable's
generic pawn. See the
[end-to-end readiness report](../../../../docs/simulation/AIRSIM-ICARUS7-READINESS.md)
for the precise remaining acceptance gates.

## Source and build boundaries

The reviewed assets are user-supplied and intentionally remain outside Git.
Their three identifying hashes are pinned in `manifest.json`. The source
checkout is pinned to Microsoft AirSim `v1.8.0-linux` commit
`caa13182328e866564c09fdeebca89ba024bc323`.

An existing packaged AirVLN executable cannot accept a new native plugin or
uncooked pawn. The module must be installed into an AirSim-enabled **source**
Unreal project, then compiled and cooked with UE4.27. A source-licensed city
map can be used in that project; the public AirVLN v2 download currently used
by Icarus contains packaged binaries only.

## Install into an AirSim environment source project

```bash
cd /home/agrim/Desktop/Icarus

./scripts/install-airsim-icarus7-module \
  --asset-project '/home/agrim/Downloads/Icarus7_FlightControl_Review (2)' \
  --environment-project /path/to/AirSimEnvironment/Environment.uproject \
  --airsim-source /path/to/pinned/AirSim-v1.8.0

./scripts/validate-airsim-icarus7-module \
  --asset-project '/home/agrim/Downloads/Icarus7_FlightControl_Review (2)' \
  --environment-project /path/to/AirSimEnvironment/Environment.uproject \
  --airsim-source /path/to/pinned/AirSim-v1.8.0
```

For an environment that does not already contain the AirSim plugin, add
`--link-airsim-plugin` during a local source build. Do not use that option to
replace an existing project-owned plugin.

The installer verifies the asset hashes, applies the physics overlay only to
the pinned AirSim revision, copies the assets and native plugin, enables both
plugins, adds the asset root to `DirectoriesToAlwaysCook`, and writes an
`.icarus7-module.json` receipt.

## Build and package

AirSim's C++ library can be built and checked independently:

```bash
./scripts/test-airsim-icarus7-airlib --airsim-source /path/to/pinned/AirSim-v1.8.0
```

The complete Unreal build requires a source-built Unreal Engine 4.27:

```bash
./scripts/package-airsim-icarus7-environment \
  --ue-root /path/to/UnrealEngine-4.27 \
  --environment-project /path/to/AirSimEnvironment/Environment.uproject \
  --airsim-source /path/to/pinned/AirSim-v1.8.0
```

Packaging writes a hash-bound `icarus7-build.json`. The runtime launcher
refuses a binary without this receipt or one whose entrypoint/settings digest
has changed:

```bash
./scripts/start-airsim-icarus7 --gui
# or
./scripts/start-airsim-icarus7
```

## Known engineering limits

- The asset's motor centres are visually symmetric at approximately ±176.8 mm
  in X/Y. The canonical Icarus physics baseline is stretched-X at approximately
  ±138.5/±173.4 mm. This module does not silently distort the visual asset or
  rewrite the accepted Gazebo model. A mechanical review must select one
  geometry before cross-simulator physical-equivalence claims.
- Mass and inertia are component-derived design targets, not measurements from
  an assembled aircraft.
- The power coefficient preserves the Gazebo torque ratio; it is not a measured
  propeller torque curve.
- The package contains no license/redistribution document. Keep the assets out
  of Git until ownership and redistribution terms are recorded.
- Sensor-origin conflicts in the source-validation JSON were resolved in favor
  of `UnrealFinalReopen.json`, which represents the post-import/reopen state.
- A source Unreal environment and UE4.27 are mandatory for the final native
  plugin compile and cooked Linux binary.
- No custom AirSim wind controller or recorded wind campaign exists yet. The
  AirVLN prompt-driven flight therefore does not establish the windy custom
  drone result already available in Gazebo.

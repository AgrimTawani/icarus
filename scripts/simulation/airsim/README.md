# AirSim ArduCopter simulation track

AirSim is a second, isolated simulator track for high-detail visual/perception
experiments. It is **not** a replacement for the Gazebo Harmonic promotion
path, and it is not an alternate control path. Icarus actions must continue
through the Drone API, MAVLink gateway, ArduPilot SITL, guardrails and safety
supervisor.

## What is different from Gazebo

| | Gazebo Harmonic | AirSim |
| --- | --- | --- |
| Status | Current acceptance/promotion baseline | Experimental feasibility track |
| Dynamics/controller path | Gazebo bridge + ArduPilot SITL | AirSim `ArduCopter` + matching ArduPilot SITL ports |
| Visual goal | Deterministic project-owned scenarios | High-detail Unreal visual diversity |
| Full start command | `./scripts/start-sim` | `./scripts/start-airsim-stack --binary /path/to/Environment.sh` |
| Drone API | `./scripts/start-autonomy` | `./scripts/start-autonomy-airsim` |
| Session file | `logs/simulation/active_session.json` | `logs/simulation/airsim_active_session.json` |
| Truth data | Scoring/debug only | Scoring/debug only |

The launcher refuses to overlap the two session types. This is deliberate:
their transport, sensors and timing must not be accidentally mixed.

## Prerequisites

1. Use the digest-pinned Blocks v1.8.0 asset for a small compatibility smoke,
   or install the external AirVLN cities for high-detail visual testing.
   AerialClaw's private OpenFly/Shanghai host is not an Icarus dependency.
2. Use an AirSim version that supports `ArduCopter`; do not use `SimpleFlight`
   and do not use direct AirSim Python movement APIs for Icarus tests.
3. Build/provide the pinned ArduPilot checkout. The full stack launcher starts
   its SITL binary and records the exact revision, command and parameter files.
4. Copy/review the settings template. Pin binary digest, environment license,
   AirSim revision, ArduPilot revision and parameters in the experiment log.

The launcher supplies classic AirSim's settings through an isolated runtime
HOME. Passing a nonexistent `-settings` flag is not supported and previously
caused the binary to fall back to SimpleFlight.

## Commands

```bash
# Structural configuration check; no simulator starts.
./scripts/airsim-doctor --binary /path/to/Environment.sh

# Fetch the digest-verified environment into ignored third_party/.
./scripts/fetch-airsim-blocks

# Optional: install the ~35 GB AirVLN archive (~40 GB extracted).
# The asset is CC BY-NC 4.0 and is for non-commercial research only.
./scripts/fetch-airvln-environments

# Low-level RPC-only diagnostic (not flight-ready).
./scripts/start-airsim --binary /path/to/Environment.sh

# Full ArduPilot stack, then Drone API in a second terminal.
./scripts/start-airsim-stack --binary third_party/airsim/blocks-v1.8.0-linux/Blocks/LinuxNoEditor/Blocks.sh
./scripts/airsim-smoke
./scripts/view-airsim-camera
./scripts/start-autonomy-airsim
./scripts/run-mission --mission takeoff_hover_land --altitude 2 --hover-seconds 3

# High-detail city track (same ArduPilot/Drone API control path).
./scripts/start-airsim-realistic

# High-detail city with the native Unreal/AirSim viewport visible.
./scripts/start-airsim-realistic --gui

# Stop only the recorded AirSim process.
./scripts/stop-airsim
```

`ViewMode: "NoDisplay"` keeps AirSim API rendering available while suppressing
the main viewport. Do not pass `--nullrhi` for RGB/depth/segmentation tests:
it disables rendering and image APIs. It is accepted only with `--no-vision`
for explicitly non-visual throughput experiments.

`./scripts/start-airsim-realistic --gui` is the operator-view alternative. It
uses the reviewed `FlyWithMe` profile, opens the packaged Unreal viewport at
1280x720, and still starts the same ArduPilot SITL stack. This is different
from `view-airsim-camera`: the former is the simulator's native rendered
viewport, while the latter is a diagnostic composite of the vehicle RGB and
false-color depth API streams.

`./scripts/view-airsim-camera` opens a passive live grid with forward RGB and
colorized planar depth from the off-screen AirSim API. Run it in its own
terminal while the stack is active; press `q`, Escape, or Ctrl+C to close it.
The headless autonomy profile intentionally captures RGB at 640x360 and depth
at 320x180. Its diagnostic viewer is evidence-oriented, not a measure of the
native viewport's visual fidelity.
For a non-GUI capture test use
`./scripts/view-airsim-camera --no-window --frames 3 --output /tmp/airsim-camera.png`.

The adapter/conformance suite covers frames, units, timestamps, metadata,
malformed packets, stale-sensor fail-closed behavior and proof that truth/raw
sensor payloads never reach DCM curation. See the architecture document for
the remaining cross-simulator promotion boundary.

## Prompt-driven city mission

The realistic launcher records the scene ID, asset source, license, settings
digest and reviewed landmark reference in the active session. With the Drone
API running, `dcm-fly --profile edge-vision` can use those committed local-NED
waypoints, inspect one live RGB frame with SmolVLM and count visible people
with YOLO. The image models remain evidence-only: every arm, takeoff, goto,
hold and land still passes through Icarus guardrails, MAVLink and ArduPilot.

AirVLN cities are visually rich but are not deterministic people-count test
fixtures. A valid result may therefore be zero visible people. Use the Gazebo
`edge-vision` scenario when a fixed expected count is required for regression.

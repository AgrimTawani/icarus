# AirSim ArduCopter simulation track (experimental)

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

1. Fetch the digest-pinned Blocks v1.8.0 asset or supply another licensed,
   version-pinned Linux environment. AerialClaw's OpenFly host is not an
   Icarus dependency and is not reproduced here.
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

# Low-level RPC-only diagnostic (not flight-ready).
./scripts/start-airsim --binary /path/to/Environment.sh

# Full ArduPilot stack, then Drone API in a second terminal.
./scripts/start-airsim-stack --binary third_party/airsim/blocks-v1.8.0-linux/Blocks/LinuxNoEditor/Blocks.sh
./scripts/airsim-smoke
./scripts/start-autonomy-airsim
./scripts/run-mission --mission takeoff_hover_land --altitude 2 --hover-seconds 3

# Stop only the recorded AirSim process.
./scripts/stop-airsim
```

`ViewMode: "NoDisplay"` keeps AirSim API rendering available while suppressing
the main viewport. Do not pass `--nullrhi` for RGB/depth/segmentation tests:
it disables rendering and image APIs. It is accepted only with `--no-vision`
for explicitly non-visual throughput experiments.

The adapter/conformance suite covers frames, units, timestamps, metadata,
malformed packets, stale-sensor fail-closed behavior and proof that truth/raw
sensor payloads never reach DCM curation. See the architecture document for
the remaining cross-simulator promotion boundary.

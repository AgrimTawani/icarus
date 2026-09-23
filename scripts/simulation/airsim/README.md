# AirSim visual-simulation track (experimental)

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
| Start command | `./scripts/start-sim` | `./scripts/start-airsim --binary /path/to/Environment.sh` |
| Session file | `logs/simulation/active_session.json` | `logs/simulation/airsim_active_session.json` |
| Truth data | Scoring/debug only | Scoring/debug only |

The launcher refuses to overlap the two session types. This is deliberate:
their transport, sensors and timing must not be accidentally mixed.

## Prerequisites

1. Obtain a licensed, version-pinned Linux AirSim/Unreal environment and keep
   it outside this repository. AerialClaw's OpenFly host is not an Icarus
   dependency and is not reproduced here.
2. Use an AirSim version that supports `ArduCopter`; do not use `SimpleFlight`
   and do not use direct AirSim Python movement APIs for Icarus tests.
3. Configure and start a matching ArduPilot AirSim SITL process using the
   ports in `config/simulation/airsim/settings.arducopter.json`. Verify the
   actual AirSim/ArduPilot release pair before flight tests.
4. Copy/review the settings template. Pin binary digest, environment license,
   AirSim revision, ArduPilot revision and parameters in the experiment log.

## Commands

```bash
# Structural configuration check; no simulator starts.
./scripts/airsim-doctor --binary /path/to/Environment.sh

# Start the Unreal process with NoDisplay and offscreen rendering. This opens
# an RPC port but does not claim that SITL/sensors are flight-ready.
./scripts/start-airsim --binary /path/to/Environment.sh

# Stop only the recorded AirSim process.
./scripts/stop-airsim
```

`ViewMode: "NoDisplay"` keeps AirSim API rendering available while suppressing
the main viewport. Do not pass `--nullrhi` for RGB/depth/segmentation tests:
it disables rendering and image APIs. It is accepted only with `--no-vision`
for explicitly non-visual throughput experiments.

Before any autonomous experiment, add and pass a simulator-adapter contract
suite covering NED frames, units, timestamps, sensor metadata, action
lifecycle, loss/fault handling and proof that AirSim truth never reaches DCM
curation. See `docs/architecture/AIRSIM-SIMULATION.md`.

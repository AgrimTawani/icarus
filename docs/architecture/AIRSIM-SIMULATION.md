# AirSim Simulation Track

Status: V2 Phase-1 feasibility implementation complete; experimental and
simulation-only. Gazebo Harmonic remains the promotion baseline.

## Purpose and boundary

AirSim adds a separate Unreal-based visual-simulation track for studying
perception under richer geometry, lighting and texture variation. It does not
replace the existing Gazebo/ArduPilot integration, and it must not create a
second vehicle-control route.

```text
                         control path (unchanged)
DCM -> Guardrails -> Mission Executor -> Drone API -> MAVLink -> ArduPilot SITL
                                                                   ^
                                                                   |
                                          AirSim physics/sensors ---+
                                                                   |
                                                    normalized adapter only
                                                                   v
                                                      Icarus perception/state
```

Direct AirSim `move*` Python calls, `SimpleFlight`, simulator truth, collision
truth and object labels are prohibited from the Icarus DCM/control input path.
Truth is valid only for post-hoc scoring/debugging.

## Two separate simulator tracks

| Property | Gazebo Harmonic | AirSim |
| --- | --- | --- |
| Role | Acceptance and regression baseline | Visual/perception feasibility research |
| Full start command | `./scripts/start-sim` | `./scripts/start-airsim-stack --binary /path/to/Environment.sh` |
| Drone API command | `./scripts/start-autonomy` | `./scripts/start-autonomy-airsim` |
| Controller integration | Existing project Gazebo bridge + ArduPilot SITL | AirSim `ArduCopter` vehicle + matching ArduPilot SITL ports |
| Scene assets | Project-owned, deterministic scenarios | External Unreal environment; digest/license must be recorded |
| Session manifest | `logs/simulation/active_session.json` | `logs/simulation/airsim_active_session.json` |
| Concurrent use | N/A | Refused while Gazebo session is active |
| Promotion status | Existing Phase 8–12 policy | None until adapter conformance and repeated campaign evidence exist |

## Headless visual mode

Use `ViewMode: "NoDisplay"` and the launcher's offscreen-rendering request for
visual experiments. AirSim documents that NoDisplay freezes the main viewport
while API image rendering continues. The launcher requires an explicit
`--no-vision` acknowledgement before it permits `-nullrhi`; that mode disables
rendering and cannot be used for RGB/depth/segmentation validation.

The exact UE binary/container determines the compatible offscreen flags. Each
experiment must retain the binary digest, GPU driver, settings file, AirSim
revision, scene asset/license, ArduPilot revision/parameters and command line.

### Pinned environment and verified boundary

`./scripts/fetch-airsim-blocks` downloads the digest-pinned public AirSim
Blocks v1.8.0 binary into ignored `third_party/airsim/`. Classic AirSim ignores
a guessed `-settings` flag and reads `HOME/Documents/AirSim/settings.json`.
The launcher therefore creates a private runtime HOME and copies the reviewed
settings there. With that boundary the pinned binary reports the `Icarus`
ArduCopter vehicle—not SimpleFlight—and supplies timestamped 256x144 RGB,
256x144 depth and LiDAR samples under `NoDisplay`/`-RenderOffScreen`.

The full stack starts pinned ArduPilot SITL in a named transient user service,
retains binary/settings/parameter/revision hashes, and exposes MAVLink TCP
5760. This local launcher requires a working `systemd --user` manager so SITL
survives the short-lived setup command and can be stopped as an exact unit.
The read-only msgpack client derives from AerialClaw's MIT adapter but
contains no movement/pose calls. Its UDP sensor bridge carries only bounded
finite LiDAR points into a distinct C++ `AirSimLidarAdapter` in body FRD.

## Bring-up sequence

1. Run `./scripts/fetch-airsim-blocks` or supply another reviewed environment.
2. Review `config/simulation/airsim/settings.arducopter.json`; retain the
   `ArduCopter` vehicle and do not substitute `SimpleFlight`.
3. Start the full dynamics/controller/sensor stack:
   `./scripts/start-airsim-stack --binary <Environment.sh>`.
4. Optionally open the passive RGB/depth ground view in another terminal with
   `./scripts/view-airsim-camera`.
5. In another terminal run `./scripts/start-autonomy-airsim`, then use the
   unchanged `./scripts/run-mission` client. `start-airsim` alone is a low-level
   Unreal/RPC diagnostic and is not flight-ready.
6. Stop the exact recorded bridge, SITL service and Unreal group with
   `./scripts/stop-airsim` after the Drone API exits.
7. Compare frozen scenarios against Gazebo; only a non-regressing result can
   justify expanding AirSim use. Gazebo remains the promotion baseline.

## Verified 2026-09-23

- RPC, ArduCopter, RGB, depth, LiDAR and timestamps passed together.
- A real Drone API mission armed, took off to 2 m, held three seconds, landed
  and disarmed in 17.96 s through MAVLink/ArduPilot; no AirSim movement API was
  used.
- Killing the sensor bridge made `GetHealth` degraded and the next preflight
  failed closed with `PERCEPTION_UNAVAILABLE`.
- Native packet tests cover FRD conversion and malformed/truncated rejection;
  Python tests cover finite-point filtering and truth/raw-image non-leakage.

This proves the integration, not cross-simulator equivalence. Blocks has no
project-owned deterministic weather/scene seed, so AirSim is not promoted as
an acceptance replacement for Gazebo.

Official reference material: [AirSim settings](https://microsoft.github.io/AirSim/settings/),
[AirSim image APIs](https://microsoft.github.io/AirSim/image_apis/), and
[ArduPilot SITL with AirSim](https://shop.ardupilot.org/dev/docs/sitl-with-airsim.html).

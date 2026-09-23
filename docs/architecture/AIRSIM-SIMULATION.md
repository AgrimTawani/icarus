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

### High-detail AirVLN city environments

Blocks proves compatibility quickly but is intentionally a primitive test
map. The high-detail track uses the public AerialVLN Simulators v2 dataset:
25 packaged Unreal/AirSim city-scale scenarios with textured buildings,
roads, vegetation and urban props. This is the closest public source to the
AirVLN/OpenFly worlds shown by AerialClaw; the exact private Shanghai binary
shown in AerialClaw's GIF is not present in that repository.

The city archive is an optional external dependency because it is large and
licensed CC BY-NC 4.0. `fetch-airvln-environments` downloads it into ignored
`third_party/`, validates the pinned archive digest, tests the ZIP before
extracting it, and fixes only the packaged Linux entrypoint permissions.
Icarus does not redistribute the asset. `start-airsim-realistic` resolves a
scene by ID and records its dataset version, source and license in the session
manifest. The default scene and complete supported scene list are versioned in
`config/simulation/airsim/airvln-simulators.json`.

The packaged environment is only rendering and collision geometry. Icarus
overrides upstream ComputerVision/SimpleFlight examples with the reviewed
`settings.realistic-city.json`: the vehicle is `ArduCopter`, the camera and
LiDAR are timestamped API sensors, and ArduPilot SITL remains the only flight
controller. No AirSim movement or pose RPC is available to the DCM.

Each promoted city scene has a separate reviewed mission-reference JSON. It
names a primary building and an explicit local-NED hover waypoint; it does not
contain a hidden person count. A prompt such as “check that building, hover
over it, and count no. of people” must produce successful `inspect_scene`,
`goto`, `hold` and `detect(["person"])` evidence in the requested order before
landing. SmolVLM supplies the qualitative building description and YOLO11n
supplies the visible-person count. Both are read-only perception tools.

AirVLN scenes are visually rich research worlds, not deterministic crowd
fixtures. Zero is a valid observed person count when the camera contains no
person. The project-owned Gazebo `edge_people_building` scenario remains the
correct regression when a known nonzero ground-truth count is required.

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

For the high-detail track, replace steps 1 and 3 with:

```bash
./scripts/fetch-airvln-environments   # one-time, large external download
./scripts/start-airsim-realistic      # tested default scene
./scripts/start-airsim-realistic --gui # native 1280x720 Unreal viewport
```

The `--gui` mode is for an operator who wants to inspect the rendered city. It
uses a separate reviewed `FlyWithMe` settings profile and retains the full
ArduPilot/RPC/sensor stack. The default command remains off-screen and
`NoDisplay` for repeatable automated runs. `view-airsim-camera` is not the
Unreal viewport: it displays the vehicle's RGB capture beside a false-color
depth product, at the sensor resolutions selected for autonomy throughput.

`--scene <id>` selects another installed AirVLN scene, but it does not make
that scene a calibrated mission environment. Prompt-driven building flight
requires a committed and reviewed `airvln-env-<id>-reference.json`.

## Verified 2026-09-23

- RPC, ArduCopter, RGB, depth, LiDAR and timestamps passed together.
- A real Drone API mission armed, took off to 2 m, held three seconds, landed
  and disarmed in 17.96 s through MAVLink/ArduPilot; no AirSim movement API was
  used.
- Killing the sensor bridge made `GetHealth` degraded and the next preflight
  failed closed with `PERCEPTION_UNAVAILABLE`.
- Native packet tests cover FRD conversion and malformed/truncated rejection;
  Python tests cover finite-point filtering and truth/raw-image non-leakage.
- The digest-pinned 34,809,808,002-byte AirVLN archive passed a complete ZIP
  integrity test and installed all 25 scenes. Scene 6 passed ArduCopter RPC,
  640x360 RGB, 320x180 depth, 16-channel LiDAR and timestamp smoke together.
- The scene-6 gray-house route was calibrated with live forward/belly frames;
  `(north=25 m, east=-6 m, altitude=22 m AGL)` was reached through the Drone
  API and ArduPilot, held and returned from successfully.
- Qwen completed the prompt-driven mission arm, takeoff 3 m, building
  inspection, reviewed goto, hold 3 s, person detection and RTL/land. One
  out-of-order proposal was refused by deterministic mission guidance and the
  next decision recovered. The final semantic result satisfied all four
  requirements with zero failed flight actions and a camera-observed count of
  zero people. Evidence episode: `20260923T212718_6bf94baf9b39`.
- That run peaked at 5,311 MiB VRAM and 657.8 MiB process RAM on the RTX 4050
  laptop profile. Scene content and detector output are not crowd ground truth.

This proves the integration, not cross-simulator equivalence. Blocks has no
project-owned deterministic weather/scene seed, so AirSim is not promoted as
an acceptance replacement for Gazebo.

Official reference material: [AirSim settings](https://microsoft.github.io/AirSim/settings/),
[AirSim image APIs](https://microsoft.github.io/AirSim/image_apis/), and
[ArduPilot SITL with AirSim](https://shop.ardupilot.org/dev/docs/sitl-with-airsim.html).

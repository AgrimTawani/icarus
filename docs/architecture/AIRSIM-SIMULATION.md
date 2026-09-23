# AirSim Simulation Track

Status: experimental V2 Phase 1. AirSim has no acceptance or autonomous
promotion authority. Gazebo Harmonic remains the verified simulator.

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
| Public start command | `./scripts/start-sim` | `./scripts/start-airsim --binary /path/to/Environment.sh` |
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

### Reproducible smoke asset, and its hard limit

`./scripts/fetch-airsim-blocks` downloads the digest-pinned public AirSim
Blocks v1.8.0 binary into ignored `third_party/airsim/`. A live launch was
verified to open RPC 41451 with `NoDisplay` and `-RenderOffScreen`; the
read-only `./scripts/airsim-smoke` checks RPC ping, vehicles and one scene
image without arming or moving anything. The same binary logs `SimpleFlight`.
It therefore has exactly one role: visual/RPC smoke evidence. The launcher
terminates it by default and requires `--allow-simpleflight-smoke` to record a
`visual-rpc-smoke-only` session. It cannot be used as evidence for ArduCopter,
SITL, DCM decisions, safety evaluation, or promotion.

## Bring-up sequence

1. Acquire and license a compatible AirSim/UE environment outside Git.
2. Review `config/simulation/airsim/settings.arducopter.json`; retain the
   `ArduCopter` vehicle and do not substitute `SimpleFlight`.
3. Start matching AirSim ArduPilot SITL with its documented sensor/control
   ports. A launched Unreal RPC process alone is not a flight-ready stack.
4. Run `./scripts/airsim-doctor --binary ...`, then start the isolated AirSim
   process. Confirm camera, depth and LiDAR timestamps from the actual binary;
   `./scripts/airsim-smoke` is only a read-only RPC/image prerequisite.
5. Implement/pass adapter conformance before any DCM experiment: frames/units,
   freshness, sensor metadata, action lifecycle, failsafe/link faults,
   deterministic seed behaviour and a truth-nonleak test.
6. Compare frozen scenarios against Gazebo; only a non-regressing result can
   justify expanding AirSim use. Gazebo remains the promotion baseline.

Official reference material: [AirSim settings](https://microsoft.github.io/AirSim/settings/),
[AirSim image APIs](https://microsoft.github.io/AirSim/image_apis/), and
[ArduPilot SITL with AirSim](https://shop.ardupilot.org/dev/docs/sitl-with-airsim.html).

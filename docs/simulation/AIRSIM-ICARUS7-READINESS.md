# AirSim Icarus7 End-to-End Readiness

Last updated: 2026-10-24

## Bottom line

Icarus can already run its complete prompt-to-flight and visual-perception
pipeline in a packaged AirVLN city using that executable's **generic AirSim
drone**. As of 2026-10-24 the custom Icarus7 pawn has also been **compiled by a
source-built Unreal Engine 4.27, cooked into the Blocks environment, spawned,
and flown** (SimpleFlight takeoff/hover, ground contact, camera imagery; see
`docs/simulation/images/icarus7-blocks-*.png`), and later the same day it
**held altitude under ArduCopter SITL** (GUIDED takeoff to 5 m: peak 5.5 m,
steady 5.00 m at 20 % throttle, land, disarm; see
`icarus7-arducopter-hover.png`). What is still missing for the
custom drone is a *realistic* world: the downloaded AirVLN cities are packaged
executables, and a compiled executable cannot take a new native pawn, so the
Icarus7 currently flies only in Blocks.

Therefore the following combined claim is **not yet true**:

> The Icarus7 custom drone completed the Gazebo-equivalent, windy,
> prompt-driven DCM/VLM mission in a realistic AirSim world.

## Keep the two AirSim results separate

| Track | What has actually run | What it proves | What it does not prove |
| --- | --- | --- | --- |
| Packaged AirVLN city | Generic compiled AirSim pawn, ArduPilot SITL, Drone API, DCM, RGB/depth/LiDAR, Qwen2-VL and YOLO | The Icarus control and visual-perception architecture works against AirSim; scene 6 completed the prompt-driven building-inspection/hover/detection/RTL flow | Custom Icarus7 rendering, collision, dynamics or sensors; deterministic wind; verified person-count accuracy |
| Custom Icarus7 source module | Reviewed meshes, native pawn source, collision, rotor animation, sensor transforms, settings and custom AirLib physics | The source contract is internally consistent; the patched AirLib compiles and instantiates the intended mass, inertia, propulsion and rotor layout | Unreal compilation, pawn spawning, camera imagery, collision behavior, custom-drone flight or an end-to-end mission |
| Gazebo baseline | Project-owned custom vehicle, seeded turbulent atmosphere, sensors, ArduPilot, Drone API and regression scenarios | Current deterministic dynamics/safety acceptance baseline | Unreal visual realism |

The packaged AirVLN executable cannot be upgraded by changing
`settings.json`. `PawnPaths` can select only classes and assets that were
already compiled and cooked into that executable. Its binary and 2.18 GB
content package contain neither `Icarus7AirSimPawn` nor the Icarus7 meshes.

## Completed custom-drone work

Six of the ten implementation and acceptance gates are complete:

1. **Asset audit — complete.** The supplied UE4.27 project, individual drone
   meshes, propellers and sensor metadata were inventoried and identifying
   hashes were frozen.
2. **Native vehicle pawn — complete as source.** `AIcarus7AirSimPawn` derives
   from AirSim's `AFlyingPawn` and provides the five required camera component
   names, four ArduPilot-ordered actuator components and individual visual
   meshes.
3. **Collision and visual actuation — complete as source.** Conservative body,
   arm and landing-gear collision boxes are provided, and the four propellers
   have AirSim-driven rotating pivots.
4. **Physics overlay — compiled and tested.** The pinned AirSim 1.8 source
   recognizes `Model=Icarus7` and instantiates the 4.4745 kg design mass,
   component-derived inertia, rotor positions, 10-inch propeller, thrust and
   torque baseline. The executable AirLib parameter test passes.
5. **Sensors and profiles — complete as configuration.** Separate GUI and
   headless profiles select the native pawn and declare forward/downward
   cameras, depth, LiDAR, IMU, GPS and downward distance sensing.
6. **Reproducible tooling — complete.** The installer validates source-asset
   hashes, applies the pinned AirSim patch, installs the module, and writes a
   receipt. The validator, Unreal packager and guarded runtime launcher are
   implemented. The launcher refuses unverified or modified packages.

Verification currently recorded:

- Custom module structural and settings contract: passed.
- Physics patch against the pinned clean AirSim revision: passed.
- Patched AirLib build and executable parameter test: passed.
- Project unit suite after integration: 273 tests and 161 subtests passed.
- Custom pawn compiled by Unreal Engine 4.27 (Blocks, Linux Development): **passed 2026-10-24**.
- Custom pawn spawn, ground contact, SimpleFlight takeoff, camera `0`/`3`
  imagery in the cooked Blocks executable: **passed 2026-10-24**.
- Custom pawn under ArduPilot SITL via `start-airsim-icarus7`: pending.

## Verified build procedure (2026-10-24)

Host: Ubuntu, UE 4.27 source build at `third_party/UnrealEngine`, AirSim
v1.8.0 at `third_party/airsim/source-v1.8.0`, Blocks as the environment.

1. `scripts/simulation/airsim/build_airlib_libcxx.sh third_party/UnrealEngine third_party/airsim/source-v1.8.0 third_party/airsim/source-v1.8.0/Unreal/Environments/Blocks`
   — AirLib/MavLinkCom/rpc must be built with Unreal's bundled clang + libc++.
   Libraries built by AirSim's `build.sh` with GCC compile but fail at the
   plugin link (`std::__cxx11` vs `std::__1`). The packager now refuses them
   up front.
2. Apply `simulation/airsim/vehicles/icarus7/patches/ue4.27-chaos-stdint-linux-game.patch`
   inside `third_party/UnrealEngine` (one `#include <stdint.h>`; the monolithic
   Linux game build of Chaos otherwise fails with `unknown type name 'uint64_t'`).
   Note: this header is included by the Engine module, so the first editor
   build after applying it recompiles ~1 600 actions.
3. `./scripts/install-airsim-icarus7-module --asset-project <Icarus7_FlightControl_Review> --environment-project .../Blocks/Blocks.uproject --airsim-source third_party/airsim/source-v1.8.0`
4. `./scripts/package-airsim-icarus7-environment --ue-root third_party/UnrealEngine --environment-project .../Blocks/Blocks.uproject --airsim-source third_party/airsim/source-v1.8.0`
   (~2 h cold; ~8 min incremental after a pawn-only change). Run it outside
   any sandbox that blocks writes to `~/Library/Logs`.
5. `./scripts/start-airsim-icarus7 [--gui]` (ArduCopter + SITL on port 41451;
   refuses to start while another AirSim session is recorded).

Pawn rules enforced by the validator since this build: the root component
must be a blocking `UBoxComponent` (AirSim sweeps the root on every move; a
scene-component root falls through the world) and camera child actors must be
`BP_PIPCamera` (bare `APIPCamera` aborts at spawn). Icarus7 LiDAR profiles
were reduced from 40 ch / 200 k pts/s to 16 ch / 50 k pts/s, matching the
proven realistic-city profile.

### ArduCopter acceptance (2026-10-24, evening)

Setup: packaged Icarus7 Blocks + `arducopter --model airsim-copter` on
isolated ports (41460 / `-I1`) while the AirVLN session kept port 41451;
same binary, settings content and parameter chain as `start-airsim-icarus7`.

| Configuration | 5 m GUIDED takeoff | Hover throttle |
|---|---|---|
| Generic AirSim quadrotor, stock params | peak 5.9 m, holds 5.00 m | 54 % |
| Icarus7, stock params | **peak 57 m with throttle 0**, descends 3 m/s | n/a |
| Icarus7 + `icarus7-sitl.parm` | peak 5.5 m, holds 5.00 m, lands, disarms | 20 % |

Cause: the Icarus7 thrust model (4 × 45.6 N on 4.47 kg, T/W 4.15) hovers at
a PWM fraction of 0.24. ArduPilot maps throttle to
`MOT_SPIN_MIN + t·(1 − MOT_SPIN_MIN)` and clamps `MOT_THST_HOVER` at 0.125;
under the default spin floor 0.15 the hover throttle is 0.107, which the
altitude loop can never settle on. `config/simulation/airsim/icarus7-sitl.parm`
lowers `MOT_SPIN_MIN` to 0.05 (hover 0.20) and sets `PSC_ACCZ_P/I` to
hover / 2 × hover per the ArduPilot tuning guide. The launcher passes the file
through `start_stack.py --sitl-param-file`; the validator recomputes the hover
throttle from the thrust model and rejects a mismatch. The physics overlay
itself is unchanged.

Two findings disproved along the way: the LiDAR payload streamed to ArduPilot
did not cause the overshoot, and the apparent ~6 s EKF lag was a MAVLink
reader that did not drain its queue. One real constraint remains: ArduPilot's
JSON parser aborts at an empty `lidar` point cloud and drops the `rc`/`rng`
keys after it, so a light LiDAR (`Lidar1`, 8 ch / 10 k pts/s) must stay
`ExternalController: true` while the perception `IcarusLidar` stays local.

## Work remaining before Gazebo-equivalent end-to-end testing

7. **Build a realistic custom environment — partially complete.** The
   toolchain, module install, compile and cook now work (Blocks). A realistic
   world still needs editable source: the AirVLN `env_6` scene is the
   Marketplace *Modular Neighborhood Pack* demo map, which can be added to
   Blocks from an Epic Games Launcher/Fab download and re-cooked. The packaged
   AirVLN binary itself cannot be modified in place.
8. **Perform custom-pawn runtime acceptance.** Verify that the Icarus7 spawns,
   rotors map FR/RL/FL/RR correctly, collision and centre-of-gravity behavior
   are sensible, ArduPilot can arm/take off/hold/land, and every camera,
   depth, LiDAR, IMU, GPS and distance stream has correct timestamps, frames
   and freshness. This must be observed from the cooked custom executable.
9. **Implement and validate the AirSim wind campaign.** No Icarus AirSim code
   currently calls `simSetWind`; the successful AirVLN mission was not a windy
   acceptance test. Add a simulation-only wind authority that applies a
   versioned vector/time profile, records every applied sample and seed, and
   never exposes simulator truth to the DCM. AirSim wind is not automatically
   equivalent to the Gazebo shear/turbulence/wake model, so tolerances and
   evidence must be defined separately.
10. **Run and score the complete architecture.** Execute a prompt such as
   “arm, take off, inspect the gray house driveway, hover over it, count the visible
   people, then return home” through the compact V3 context, DCM, typed Drone
   API, guardrails, ArduPilot SITL and the custom pawn. RGB/depth/LiDAR must
   come from the custom vehicle. Seal and replay the decision episode, verify
   flight/safety outcomes, and compare against the Gazebo baseline.

## Additional decisions required for a parity claim

- The visual asset places its motors symmetrically at approximately ±176.8 mm
  in X/Y, while the accepted Gazebo physics baseline uses approximately
  ±138.5/±173.4 mm. The custom module currently preserves the visual geometry
  and uses the canonical physics geometry. A mechanical review must select the
  authoritative geometry before claiming cross-simulator physical parity.
- The current mass and inertia are component-derived design targets, not
  measurements from the assembled aircraft.
- A realistic city is not automatically a deterministic perception fixture.
  Person-count acceptance requires reviewed visible-person ground truth for a
  fixed camera, pose and scene state. Without that truth, a VLM/detector answer
  is an observation, not an accuracy pass.
- AirSim and Gazebo use different rendering and aerodynamic implementations.
  Passing both establishes behavior within reviewed tolerances; it does not
  make the simulators physically identical.

## Definition of done

AirSim reaches the same end-to-end testing level as the current Gazebo track
only when one retained run demonstrates all of the following together:

- The cooked executable visibly contains and spawns the Icarus7 pawn.
- ArduPilot SITL, not SimpleFlight or direct AirSim movement RPC, controls it.
- The prompt is planned by the DCM and every flight action crosses the existing
  Drone API, guardrails, executor and safety supervisor.
- The custom pawn supplies the RGB/depth/LiDAR evidence consumed by the
  VLM/detector and the normalized perception layer.
- A recorded wind profile is active and its application is independently
  verifiable.
- The mission arms, takes off, flies only to explicit operator-provided local
  coordinates, inspects the current camera view, holds, performs detection,
  returns/lands and disarms.
- Ground-truth-backed perception scoring, safety results, timestamps,
  configuration hashes and a sealed/replayable decision episode are retained.
- The run meets explicit tolerances without regressing the Gazebo baseline.

Until those conditions pass, the accurate project status is: **AirSim generic
vehicle end-to-end integration proven; custom Icarus7 source integration
complete; custom Icarus7 AirSim runtime acceptance pending.**

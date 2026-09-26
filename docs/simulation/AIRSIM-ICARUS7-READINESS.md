# AirSim Icarus7 End-to-End Readiness

Last updated: 2026-09-27

## Bottom line

Icarus can already run its complete prompt-to-flight and visual-perception
pipeline in a packaged AirVLN city using that executable's **generic AirSim
drone**. The custom Icarus7 vehicle is implemented and its AirLib physics have
been compiled and tested, but the custom Unreal pawn has not yet been compiled,
cooked or flown. The current host has no Unreal Engine 4.27 toolchain, and the
downloaded AirVLN cities are packaged executables rather than editable Unreal
projects.

Therefore the following combined claim is **not yet true**:

> The Icarus7 custom drone completed the Gazebo-equivalent, windy,
> prompt-driven DCM/VLM mission in a realistic AirSim world.

## Keep the two AirSim results separate

| Track | What has actually run | What it proves | What it does not prove |
| --- | --- | --- | --- |
| Packaged AirVLN city | Generic compiled AirSim pawn, ArduPilot SITL, Drone API, DCM, RGB/depth/LiDAR, SmolVLM and YOLO | The Icarus control and visual-perception architecture works against AirSim; scene 6 completed the prompt-driven building-inspection/hover/detection/RTL flow | Custom Icarus7 rendering, collision, dynamics or sensors; deterministic wind; verified person-count accuracy |
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
   recognizes `Model=Icarus7` and instantiates the 4.343 kg design mass,
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
- Custom pawn compiled by Unreal Engine: **not run**.
- Custom pawn flight in AirSim: **not run**.

## Work remaining before Gazebo-equivalent end-to-end testing

The final four gates remain open:

7. **Build a realistic custom environment.** Obtain an editable UE4.27 AirSim
   environment, not only a packaged `.pak`, and establish its license. Install
   a source-built Unreal Engine 4.27 toolchain, install the Icarus7 module into
   that project, compile it and cook a Linux executable. The exact public
   AirVLN binary already used cannot be modified in place.
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
   “arm, take off, inspect that building, hover over it, count the visible
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
- The mission arms, takes off, inspects the reviewed building, reaches and
  holds the reviewed waypoint, performs detection, returns/lands and disarms.
- Ground-truth-backed perception scoring, safety results, timestamps,
  configuration hashes and a sealed/replayable decision episode are retained.
- The run meets explicit tolerances without regressing the Gazebo baseline.

Until those conditions pass, the accurate project status is: **AirSim generic
vehicle end-to-end integration proven; custom Icarus7 source integration
complete; custom Icarus7 AirSim runtime acceptance pending.**

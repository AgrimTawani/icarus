# Phase 5 Simulation Acceptance — 2026-09-12

> Historical acceptance record. The smooth native Gazebo wind implementation
> described below was replaced later on 2026-09-12 by the canonical seeded
> Icarus turbulent atmosphere documented in `guides/SIMULATION-OPERATIONS.md`.

Status: **passed**. Phase 5 now provides strict, deterministic environment and
scenario layers. This is not obstacle-avoidance autonomy; route execution and
planning remain later phases. No screenshots, image pixels or videos were saved,
and no dependencies were installed.

## Implemented scenarios

Eight versioned JSON definitions live in `simulation/scenarios`:

| Scenario | Purpose |
| --- | --- |
| `empty_validation` | Flat, known home/origin, no wind or obstacles |
| `wind_light` | Populated village + constant 1.5 m/s wind |
| `wind_strong` | Populated village + constant 5 m/s wind |
| `wind_gusting` | Populated village + 3 m/s wind with 60% sinusoidal gusts |
| `wind_direction_change` | Populated village + 2.5 m/s gusts and ±90° direction swing |
| `wind_limit_reject` | 8 m/s exceeds the 6 m/s operating limit; pre-arm rejection |
| `obstacle_course` | Measured walls/boxes, simplified buildings, trees, two routes |
| `adverse_combined` | Obstacles, wind, noise, GPS degradation, link faults, low battery |

Every definition contains its world/vehicle profile, initial pose and home,
seed, wind and obstacle configuration, sensor fault schedule, mission, maximum
duration and explicit pass/reject criteria. The loader rejects missing/extra
top-level fields, invalid dimensions, unsupported mission types, inconsistent
wind decisions and buildings above the project's two-storey / 6 m limit.

`build_phase5_world.py` creates a scenario-specific SDF, vehicle model and
ground-truth sidecar. Rebuilding all eight twice produced identical hashes.
All generated SDF files pass Gazebo validation. Scenario-specific vehicle copies
prevent later scenario builds from silently changing an earlier world's sensor,
GPS or battery configuration.

## Environment behavior

Wind uses the model-level, seeded `IcarusTurbulentAtmosphere` system. It is the
sole aerodynamic-force authority: the base and rotor links deliberately do not
opt into Gazebo's native `WindEffects`, preventing force double-counting if that
system is ever added to a world. Constant, gust-magnitude and direction-swing
profiles are encoded in generated SDF. The 8 m/s case is rejected before simulation or
arming; the rejection record confirms no processes were started.

All flyable wind profiles now use the shared `mixed_village` environment instead
of the empty validation field. It surrounds a five-metre clear launch zone with
three detailed two-storey buildings, a wall, three trees, roads, footpath and
markings. Decorative detail is visual-only; scoring collisions stay primitive.

The obstacle map uses primitive collision geometry:

- Walls and boxes have exact axis-aligned dimensions.
- Buildings use one collision box each and are at most 6 m tall.
- Trees have cylinder trunks and explicit spherical canopy collision policy.
- Ground truth contains exact obstacles and ENU waypoints for open and narrow
  routes. The scorer inflates obstacles by a configurable 0.35 m vehicle radius
  and checks swept trajectory segments, duration and final-goal error.

A synthetic open-route trajectory passed. A trajectory crossing the calibration
wall was correctly rejected and named that wall, proving both scoring outcomes.

## Combined adverse validation

The combined scenario uses 2 m/s wind with 50% gusts and ±45° direction changes,
native noisy sensors, public-GPS position/velocity standard deviations of
0.8 m / 0.15 m/s, 35% initial battery state, 850 W model load, a 200 ms public
consumer delay and an 800 ms lidar dropout. Faults are scheduled from simulation
time and recovered automatically. They operate at the public sensor-consumer
boundary; they do not delay MAVLink or inject faults into ArduPilot's EKF.

The final integrated ArduPilot flight passed takeoff, ten-second hover, landing
and disarm. Maximum drift was 0.664 m, maximum tilt 17.90°, and altitude stayed
2.979–3.008 m during hover, within the declared adverse envelope. Effective
real-time factor was 0.978, above the 0.8 requirement. The older calm-air hover
gate remains the default; scenario-specific limits are explicit inputs and the
global emergency envelope is unchanged.

Evidence:

- `logs/simulation/phase5_acceptance_f1978c88/results.json`
- `logs/simulation/scenario_adverse_combined_20260912T025448_ac99b8/`
- `logs/simulation/scenario_wind_light_20260912T031113_0bae76/` — populated
  mixed-village flight passed at 0.975× real time, with 0.148 m maximum drift
  and 8.96° maximum tilt.
- `logs/simulation/scenario_rejection_20260912T025149_19fe8c.json`

The 5 m/s `wind_strong` world is intentionally a more severe stress case. The
current vehicle held altitude but exceeded its declared hover envelope (1.40 m
drift and 20.41° tilt), so the supervisor stopped that flight. This is a vehicle
wind-model/control calibration task, not a world-generation failure.

An intentionally disconnected Gazebo-only performance experiment measured about
0.70× because the ArduPilot lockstep bridge had no controller. It is retained as
failed diagnostic evidence, not used to claim the exit gate. The representative
integrated scenario is the valid performance measurement.

## Commands

```bash
# Generate any scenario and its ground truth
/usr/bin/python3 scripts/simulation/build_phase5_world.py obstacle_course

# Run an integrated scenario; add --gui only for operator inspection
./scripts/sim --scenario adverse_combined

# Validate definitions, deterministic builds, SDF, scorer and integrated evidence
/usr/bin/python3 scripts/simulation/test_phase5_scenarios.py \
  --integrated-run-directory logs/simulation/scenario_adverse_combined_20260912T025448_ac99b8

# Score an external sampled ENU trajectory
/usr/bin/python3 scripts/simulation/score_phase5_trajectory.py \
  obstacle_course trajectory.json --route open
```

The next roadmap phase is Phase 6: turn the launcher into named launch profiles
and the complete operator command surface, then run its 20-cycle reliability and
failure-injection gate.

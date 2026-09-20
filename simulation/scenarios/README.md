# Simulation Scenarios

Phase 5 scenarios are strict JSON files. Each selects the world and vehicle,
home and initial pose, deterministic seed, wind, obstacles, sensor profile,
fault schedule, degraded GPS, public-link disturbances, battery state, mission,
duration and machine-checkable success limits.

Available scenarios:

- `empty_validation`: uncluttered, still-air baseline.
- `wind_light`, `wind_strong`, `wind_gusting`, `wind_direction_change`: each
  combines wind with the populated `mixed_village` environment rather than an
  empty test field.
- `wind_limit_reject`: exceeds the declared 6 m/s operating limit and must be
  rejected before Gazebo starts or motors can arm.
- `obstacle_course`: primitive wall/box, two-storey buildings, trees with
  explicit canopy collisions, and open/narrow ground-truth routes.
- `adverse_combined`: obstacles, gusting/directional wind, native sensor noise,
  degraded public GPS, public-sensor link delay/dropout and low battery.
- `mavlink_loss`, `mavlink_delay`: simulator-only gateway control-link fault
  cases. They are distinct from public sensor delay/dropout.

Generate an SDF plus its ground-truth sidecar:

```bash
/usr/bin/python3 scripts/simulation/build_phase5_world.py obstacle_course
```

Run a defined flight, optionally with the GUI:

```bash
./scripts/sim --scenario empty_validation
./scripts/sim --scenario wind_gusting --gui
```

Scenario seed and sensor profile are authoritative; conflicting CLI overrides
are rejected. Current executable mission type is `takeoff_hover_land`. Obstacle
route following belongs to the later mission/planning phases, but trajectories
can already be scored against ground truth:

```bash
/usr/bin/python3 scripts/simulation/score_phase5_trajectory.py \
  obstacle_course trajectory.json --route open
```

Fault schedules use simulation-relative time. Delay/dropout acts at the public
sensor-consumer boundary, not the MAVLink control link or ArduPilot EKF. Generated
worlds/models are deterministic build artifacts under `simulation/worlds/generated`
and `simulation/models/phase5_*`; edit scenario JSON, not generated files.

`mavlink_fault_schedule`, when present, is passed by `scripts/start-autonomy`
only to the local simulator gateway. A `loss` event drops both telemetry and
outbound commands for its interval; a `delay` event adds the declared latency
to both directions. It exists solely to exercise recovery behavior and is never
used in a physical-vehicle launch.

The mixed-village preset is stored in `config/simulation/environment_presets.json`.
It keeps a five-metre clear launch area surrounded by three two-storey buildings,
a wall and three trees. Buildings use simple collision boxes but add roofs,
doors and windows visually; trees retain explicit trunk/canopy collisions. Roads,
footpath and markings are visual-only and do not create false obstacles.

# Simulation Connections and Data Flow

This is the canonical overview of the current simulator data flow. Detailed
bring-up evidence is retained in the `SIM-*` documents.

## Implemented Flight Path

```text
./scripts/sim
    -> scripts/simulation/launch_compact.py
       -> validates scenario and generates world/model
       -> starts Gazebo server (+ GUI when requested)
       -> starts ArduCopter SITL
       -> starts deterministic pymavlink mission controller
       -> starts sensor/health/fault recorders

Gazebo physics --JSON/UDP :9002--> ArduPilot SITL
Gazebo motors <--normalized servo outputs-- ArduPilot SITL
Controller <---------MAVLink TCP :5760--------> ArduPilot SITL
Public sensor streams --> recorder/fault boundary (future perception consumers)
```

Gazebo owns rigid-body dynamics, collisions, wind and native sensor simulation.
ArduPilot owns estimation, stabilization, flight modes and motor mixing. The
pymavlink controller performs the present automated takeoff-hover-land test; it
is test infrastructure, not the future Drone API.

## Sensor Paths

The flight bridge supplies the simulator state needed by ArduPilot SITL and
returns four motor outputs to the Gazebo model. A dedicated FRD IMU and physical
model state support that lockstep interface. ArduPilot SITL currently generates
its GPS, compass, barometer and battery behavior within the SITL integration.

Gazebo-native camera, 360-degree lidar and downward range sensors are exposed as
public simulation streams and checked by the health recorder. Phase 5 delays or
drops data at this public consumer boundary. Those faults do **not** corrupt the
MAVLink connection or ArduPilot EKF. Connecting these streams to Icarus
perception and the normalized state engine belongs to Phases 8–9.

## Frames and Units

- Gazebo world and scenario waypoints use ENU: +X east, +Y north, +Z up.
- ArduPilot navigation uses NED: +X north, +Y east, +Z down.
- Vehicle/body contracts name FRD or FLU explicitly; no unnamed body frame is
  accepted.
- Distances are metres, angles radians internally, speeds m/s and time seconds.

See [`SIM-COORDINATE-FRAMES.md`](SIM-COORDINATE-FRAMES.md)
for the validated conversion details.

## Scenario Build

Each JSON file in `simulation/scenarios/` declares seed, environment preset,
vehicle, pose, wind, obstacles, sensor profile, faults, battery, mission and
success limits. `build_phase5_world.py` deterministically creates a generated
world, a scenario-specific vehicle model and a ground-truth sidecar. Generated
outputs are ignored by Git; scenario definitions and builders are the source.

All flyable wind cases use the `mixed_village` preset: a five-metre clear launch
zone surrounded by three two-storey buildings, three trees, a wall, roads and a
footpath. Primitive collision geometry is used for scoring and reliable physics.

## Launch and Shutdown

The launcher owns every child process and records a run directory beneath
`logs/simulation/`. Startup stops before arming when validation or connection
checks fail. Normal completion and interruption terminate children and release
ports. Phase 6 will expose formal GUI, headless, manual and automated profiles
and prove 20-cycle cleanup reliability.

## Simulation-to-Real Replacement

In real operation, Gazebo and SITL disappear. The MAVLink gateway points to the
Pixhawk, and perception adapters point to physical drivers. The Drone API,
mission executor, guardrails, safety supervisor, DCM contracts, event format and
evaluation logic remain unchanged. Gazebo ground truth has no real-flight
equivalent and is used only by simulation scoring.

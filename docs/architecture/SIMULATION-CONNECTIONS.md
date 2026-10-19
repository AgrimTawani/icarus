# Icarus Simulation Connections and Data Flow

This is the canonical overview of the **Gazebo baseline** simulator data flow.
Detailed bring-up evidence is retained in the `SIM-*` documents. The isolated
experimental AirSim track is documented separately in
[`AIRSIM-SIMULATION.md`](AIRSIM-SIMULATION.md); it has no promotion authority.

The two tracks have different launchers and session manifests and cannot run
concurrently. AirSim's full route is:

```text
start-airsim-stack -> Blocks/Unreal (NoDisplay) <-> ArduPilot airsim-copter
start-autonomy-airsim -> Drone API -> MAVLink TCP 5760 -> ArduPilot
AirSim read-only RPC -> bounded LiDAR UDP 41452 -> AirSimLidarAdapter
```

`start-airsim` alone starts only Unreal/RPC. It is deliberately not equivalent
to `start-sim` or `start-airsim-stack`.

## Implemented Flight Path

```text
./scripts/start-sim --profile <name>
    -> scripts/simulation/launch_compact.py --server-only
       -> validates scenario and generates world/model
       -> starts Gazebo server (+ GUI when requested)
       -> starts ArduCopter SITL
       -> starts calibrated image-flow and Gazebo-to-MAVLink sensor bridges
       -> verifies sensors, MAVLink navigation and forward video
       -> publishes the active session only after global readiness

Exactly one independent control client:
    ./scripts/manual-control -> keyboard/Xbox RC override client
    ./scripts/run-mission / ./scripts/dcm-fly
                            -> Drone API -> safety-bounded MAVLink gateway

Gazebo physics --JSON/UDP :9002--> ArduPilot SITL
Gazebo motors <--normalized servo outputs-- ArduPilot SITL
Controller <---------MAVLink TCP :5760--------> ArduPilot SITL
Public sensor streams --> companion perception, recorder and fault boundary
MID-360 / downward range / image-derived flow
    --> Gazebo-only sensor bridge --MAVLink2 UDP :14562--> ArduPilot SERIAL2
```

Gazebo owns rigid-body dynamics, collisions, wind and native sensor simulation.
ArduPilot owns estimation, stabilization, flight modes and motor mixing. The
`run-mission` client exercises the Drone API. Dedicated simulator acceptance
controllers can also test SITL directly and must not run alongside a mission
client. The manual client supplies
simulated RC input while ArduPilot still owns stabilization and motor mixing.

## Sensor Paths

The flight bridge supplies the simulator state needed by ArduPilot SITL and
returns four motor outputs to the Gazebo model. A dedicated FRD IMU and physical
model state support that lockstep interface. ArduPilot SITL currently generates
its GPS, compass, barometer and battery behavior within the SITL integration.

Gazebo-native camera, 360-degree lidar and downward range sensors are exposed as
public simulation streams and checked by the health recorder. The companion
uses the cameras and 3-D LiDAR for perception. The Gazebo-only autopilot bridge
additionally forwards a horizontal LiDAR proximity slice (`OBSTACLE_DISTANCE`),
the real downward range rays (`DISTANCE_SENSOR`), and calibrated image-derived
flow (`OPTICAL_FLOW`). GPS/barometric altitude stay primary; valid flow is
configured as supplemental velocity aiding and range supports terrain/landing.

Sensor `drop` faults on `lidar`, `range` and `optical_flow` now also stop their
autopilot input streams, allowing ArduPilot's native timeout behaviour to be
tested. This does not corrupt the flight-command connection. Direct GPS/IMU
hardware-channel fusion remains outside this change. See
[`Gazebo hardware fidelity`](../simulation/GAZEBO-HARDWARE-FIDELITY.md#ardupilot-sensor-integration-gazebo-only)
for frame conventions, parameter defaults, control-mode restrictions and tests.

## Frames and Units

- Gazebo world and scenario waypoints use ENU: +X east, +Y north, +Z up.
- ArduPilot navigation uses NED: +X north, +Y east, +Z down.
- Vehicle/body contracts name FRD or FLU explicitly; no unnamed body frame is
  accepted.
- Distances are metres, angles radians internally, speeds m/s and time seconds.

See [`../simulation/COORDINATE-FRAMES.md`](../simulation/COORDINATE-FRAMES.md)
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

The launcher owns every simulator child process and records a run directory
beneath `logs/simulation/`. It publishes a local active-session manifest only
after sensor, camera, heartbeat, GPS and global-position readiness. Startup failures occur before a client can arm. Control
clients own no simulator processes and can disconnect independently. Normal
interruption terminates children and releases ports. The Phase 6 gate passed 20
consecutive lifecycle cycles plus occupied-port, duplicate-launch, malformed
scenario, unavailable-hardware-profile, stale-sensor and child-crash injection.

## Simulation-to-Real Replacement

In real operation, Gazebo and SITL disappear. The MAVLink gateway points to the
Pixhawk, and perception adapters point to physical drivers. The Drone API,
mission executor, guardrails, safety supervisor, DCM contracts, event format and
evaluation logic remain unchanged. Gazebo ground truth has no real-flight
equivalent and is used only by simulation scoring.

# Simulation: Coordinate and Unit Conventions

## Units

Use SI units everywhere: metres, seconds, kilograms, radians, metres per second,
newtons, newton-metres, volts, amperes, and joules. Degrees may appear only in
human-facing configuration fields explicitly suffixed `_deg`.

## Gazebo World Frame

The Gazebo world uses ENU:

- `+X`: east
- `+Y`: north
- `+Z`: up
- right-handed rotation

The visible debug model uses red for +X/east, green for +Y/north, and blue for
+Z/up. It is included by `simulation/worlds/coordinate_debug.sdf`.

## Vehicle Body Frames

The Gazebo `base_link` uses FLU:

- `+X`: forward
- `+Y`: left
- `+Z`: up

ArduPilot body quantities use FRD:

- `+X`: forward
- `+Y`: right
- `+Z`: down

Convert a Gazebo body vector to ArduPilot body coordinates with:

```text
(x_frd, y_frd, z_frd) = (x_flu, -y_flu, -z_flu)
```

## World Conversion

Convert Gazebo ENU positions or vectors to ArduPilot NED with:

```text
(north, east, down) = (y_enu, x_enu, -z_enu)
```

The ArduPilot Gazebo plugin expresses the equivalent transform with its
`gazeboXYZToNED` setting. Icarus code must use a named transform utility rather
than reproducing sign swaps at call sites.

## Mark4 Motor Numbering

Use ArduCopter `FRAME_CLASS=1` and `FRAME_TYPE=1` (QUAD/X). Viewed from above
with the nose forward:

```text
                 +X forward

        M3 CW                    M1 CCW
     front-left               front-right


        M2 CCW                   M4 CW
      rear-left                rear-right

                 -X rear
```

The model link and joint names use zero-padded physical numbers:
`motor_01` through `motor_04` and `rotor_01_joint` through `rotor_04_joint`.
The ArduPilot plugin control channels remain zero-based, so channel 0 drives
`motor_01`, channel 1 drives `motor_02`, and so on. Never infer motor order from
XML element order.

## Sensor Frames

- `imu_link`: FLU and rigidly aligned with `base_link` in the first model.
- `gnss_link`: FLU; antenna phase-center pose is explicit.
- `rangefinder_down_link`: sensor +X is the optical/range axis and points down.
- `camera_link`: physical housing frame, FLU when level and facing forward.
- `camera_optical`: +Z forward, +X right, +Y down.
- `lidar_link`: +X forward, +Y left, +Z up.
- `lidar_optical`: preserve the vendor scan convention inside the driver, then
  transform outputs into `base_link` before fusion.

Every sensor pose is defined once relative to `base_link`. State Engine and
perception outputs declare their frame and monotonic timestamp; unstamped or
anonymous-frame data is invalid.

## Naming

- SDF model names: lower snake case prefixed with `icarus_` when globally
  resolved, for example `icarus_mark4_v2_10`.
- Links and joints: lower snake case with functional names.
- Gazebo topics: `/icarus/<vehicle_id>/<subsystem>/<stream>`.
- Protobuf frames: canonical lowercase names matching this document.
- Vehicle IDs: `vehicle_01`, `vehicle_02`, and so forth.

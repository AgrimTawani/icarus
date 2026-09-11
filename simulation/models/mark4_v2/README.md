# Mark4 V2 Model

This directory contains the Icarus 427 mm Mark4-derived digital twin. Its
authoritative dimensions, mass budget, propulsion choice, and packaging gates
are defined in `docs/vehicle/mark4_v2_bom.md`.

The model starts with project-authored primitive geometry. No third-party mesh
has been imported.

## Phase 3.1 rigid body

`model.sdf` preserves these whole-vehicle reference properties:

- the frozen 4.343 kg takeoff mass;
- a centre of mass 40 mm below the propeller plane;
- a provisional diagonal inertia tensor of `(0.0303, 0.0365, 0.0424) kg m^2`;
- 427 mm crossed arms, the custom centre structure, representative payload,
  battery, and landing skids; and
- primitive collision geometry suitable for the first physics tests.

These are rounded provisional engineering estimates from Phase 3.1. The original
component calculation was not saved, and the complete payload placement is not
yet represented by geometry. Zero products of inertia are a simplifying
assumption (left/right symmetry alone does not guarantee zero XZ product).
These values need a reproducible component/CAD model and ultimately measurements
before controller tuning can be treated as representative of hardware.

Validate the model and inspect its aggregate inertial properties with:

```bash
./scripts/simulation/validate_sdf.sh
gz sdf --inertial-stats simulation/models/mark4_v2/model.sdf
```

The repeatable gravity test world is
`simulation/worlds/mark4_rigid_body_drop.sdf`. Motors default to zero command;
the test checks passive landing-gear contact with gravity.

## Phase 3.2 rotors and actuator bench

Four 20 g rotor links represent the propellers. Motor mass remains in the body.
Subtracting the propeller mass/inertia with the parallel-axis theorem preserves
the whole-vehicle values above. Each rotor uses the inertia of a uniform 254 mm
disk: Ixx=Iyy=m*r^2/4, Izz=m*r^2/2. The three blades are visual primitives;
blade collision and motor-bell rotational inertia are not modeled yet.

| Array index | Link / joint | Position FLU, metres | Spin viewed from above |
| --- | --- | --- | --- |
| 0 | motor_01 / rotor_01_joint | +0.15097, -0.15097, 0 | CCW |
| 1 | motor_02 / rotor_02_joint | -0.15097, +0.15097, 0 | CCW |
| 2 | motor_03 / rotor_03_joint | +0.15097, +0.15097, 0 | CW |
| 3 | motor_04 / rotor_04_joint | -0.15097, -0.15097, 0 | CW |

The installed Gazebo MulticopterMotorModel consumes a four-element
`gz.msgs.Actuators.velocity` vector on
`/icarus/vehicle_01/actuators/motor_speed`. Values are nonnegative physical
angular speeds in rad/s, not normalized throttle. Joint animation runs at 1/10
speed while thrust uses physical speed. Rotation signs come from the plugin.

Initial coefficients:

- Physical ceiling: 15,606 RPM = 1634.2565 rad/s, from the existing BOM reference.
- Thrust: F=k*omega^2, k=1.7066574223409543e-5 N/(rad/s)^2, derived from the
  BOM's 4.648 kgf maximum point. This is a single-point bring-up approximation;
  **the complete manufacturer thrust/RPM/current curve is not imported yet**.
- Reaction torque: Q=-spin*0.016*F; 0.016 m is a provisional Gazebo default,
  not a measured XRotor torque coefficient.
- Speed rise/fall time constants: 40/80 ms, provisional assumptions.
- Rotor lateral drag and rolling moment: zero for the isolated bench tests.

The BOM's 70% operational throttle ceiling is not 70% of angular speed. A
calibrated throttle-to-speed mapping and ceiling belong in the controller
integration. Do not interpret this motor bench as a flight-ready controller.
The ArduPilot plugin currently publishes per-channel Double commands, whereas
this model consumes an Actuators vector; Phase 3.3 must supply the conversion
and ensure only one system owns joint velocity commands.

Run the motor bench using the system interpreter with installed Gazebo bindings:

```bash
/usr/bin/python3 scripts/simulation/test_mark4_motors.py
./scripts/simulation/run_mark4_drop_test.sh
```

The bench generates zero-gravity worlds from this model in `logs/simulation/`,
uses a fresh Gazebo partition/process group per case, and advances fixed 1 ms
steps. It checks each rotor's spin, inactive channels, first-order rise/fall,
upward thrust, roll/pitch moment signs, opposite yaw reaction, and balanced
collective thrust magnitude, maximum speed saturation and return to zero. It
saves the world, Gazebo log and result JSON.
This is actuator verification, not SITL flight verification.

Latest validation: all five cases passed. At a 300 rad/s physical command,
the active joint reached 18.9636 rad/s after 40 ms and 29.7979 rad/s after
200 ms (10x simulation slowdown); inactive joints stayed near zero. Collective
rise after 200 ms was 0.015248 m with negligible attitude change. A command at
twice maximum speed saturated at 163.4250 rad/s joint speed. Passive drop
settled at 0.155999 m with 0.000 degree tilt. The whole-vehicle mass/CG/inertia
were preserved. Logs: `logs/simulation/mark4_motors_20260912T005611_845d2b/`.

References: [Gazebo motor model implementation](https://github.com/gazebosim/gz-sim/blob/gz-sim8/src/systems/multicopter_motor_model/MulticopterMotorModel.cc),
[Hobbywing product page](https://www.hobbywing.com/en/products/xrotor3115), and the
local ArduPilot `AP_MotorsMatrix.cpp` Quad X table.

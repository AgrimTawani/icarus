# Vehicle: Icarus Mark4 V2 Reference Build

## Status

This document freezes the Phase 0.1 simulation reference vehicle. It is a
physics-model baseline, not an instruction to purchase hardware. Every assumed
mass, thrust curve, inertia, and mounting position must be measured again before
physical construction.

## Frame Decision

Use a **Mark4 V2-derived 10-inch chassis with a 427 mm diagonal wheelbase**.
The motor geometry and replaceable-arm concept remain Mark4 V2; the center
structure is custom because the stock plates are not designed for this payload.

The preferred 7-inch frame is rejected for Icarus because the intended payload
includes a Jetson Thor, Pixhawk 6X, cooling and power electronics, GNSS, lidar,
camera, and a battery large enough to power both propulsion and compute. The
Jetson AGX Thor developer kit is approximately 243.19 x 112.40 x 56.88 mm and
1.94 kg. It is a bench-development device and must not be modeled as the final
airborne computer. The production T5000 module is 100 x 87 x 15.29 mm and 350 g
before its carrier, cooling, and power system. This makes a 7-inch packaging
layout unnecessarily restrictive.

The 10-inch choice is the minimum credible size for this payload, not an
unqualified final-airframe recommendation. Phase 0 assumes a future custom or
flight-suitable T5000 carrier and sets strict packaging, mass, thermal, and
power gates. If the complete aircraft exceeds any of those gates, the correct
change is a larger airframe or smaller compute system, not optimistic
simulation parameters.

## Engineering Verdict

The revised vehicle is credible as a simulation reference and a possible
prototype, with these qualifications:

- Propulsion has enough control margin at the 4.5 kg design limit.
- Hover falls below 50% throttle using the published static test curve.
- The stock Mark4 center plates are rejected; a reinforced two-level center
  chassis is required.
- The T5000 developer kit cannot fly on this aircraft. Only a compact production
  module assembly can pass the packaging and mass gates.
- Estimated endurance is short. This is an autonomy research aircraft, not an
  endurance-optimized mapping platform.
- Stability is not certified by thrust-to-weight ratio. It still requires a
  measured inertia tensor, vibration analysis, ArduPilot tuning, and wind tests.

## Simulation Geometry

Use an X quadrotor with the body origin at the geometric center and the forward
axis between the two front arms.

| Property | Frozen value |
| --- | ---: |
| Diagonal motor-to-motor wheelbase | 427 mm |
| Motor coordinate magnitude on X and Y | 150.97 mm |
| Adjacent motor spacing | 301.93 mm |
| Propeller diameter | 254 mm |
| Approximate adjacent propeller clearance | 47.93 mm |

Initial motor centers relative to `base_link`, before final frame measurement:

```text
front-left:   (+0.15097, +0.15097, 0.000) m
front-right:  (+0.15097, -0.15097, 0.000) m
rear-right:   (-0.15097, -0.15097, 0.000) m
rear-left:    (-0.15097, +0.15097, 0.000) m
```

The primitive model may use these points immediately. Visual meshes and inertial
properties must later come from a measured, assembled frame.

## Custom Center Chassis

Build the center as a replaceable avionics bay rather than stretching the stock
top plate:

- Reinforced carbon arm thickness: 7 mm target, subject to laminate analysis.
- Lower structural plate: maximum 200 x 110 mm, chamfered around rotor disks and
  positioned at least 45 mm below the propeller plane.
- Upper isolated avionics shelf: maximum 150 x 105 mm and at least 20 mm below
  the blade-swept volume.
- Thor module and carrier envelope: maximum 140 x 100 mm, including connectors
  but excluding a duct or heat spreader that stays inside the center bay.
- Underslung battery tray: approximately 180 x 75 mm with two independent
  retention straps and a positive end stop.
- Landing skids must protect the battery, rangefinder, and compute bay.
- Maintain at least 15 mm measured blade-to-structure clearance under worst-case
  arm and propeller flex.

This geometry must be checked in CAD before a plate is cut. Components may
overlap a rotor disk in top view only when they are vertically separated from
the blade and do not materially block the rotor inflow or outflow.

## Frozen Propulsion Baseline

| Component | Selection | Simulation value |
| --- | --- | --- |
| Motors | 4 x Hobbywing XRotor 3115 900 KV | 125 g each; 4-6S |
| Propellers | HQ 10 x 4.5 x 3 | Two CW and two CCW |
| ESC | Hobbywing XRotor FPV G2 65 A 4-in-1 | Bidirectional DShot600; 6S; 65 A continuous per channel |
| Main battery | 6S 10,000 mAh LiPo, at least 30C | 22.2 V nominal; 222 Wh; assume 1.35 kg until measured |

The manufacturer test for the selected motor and propeller reports, per rotor,
761 g thrust at 40%, 1,336 g at 50%, and 4,648 g maximum at sea level. The
selected-propeller test reaches 15,606 RPM at its maximum point. This is a short
test value and must not be treated as continuous usable thrust. Import the
complete manufacturer curve into Gazebo rather than fitting the vehicle from
the maximum value alone.

Start with DShot600 in the hardware configuration. The Gazebo actuator model
does not emulate DShot electrical timing; it consumes normalized motor output
through the ArduPilot Gazebo interface.

The 65 A ESC is acceptable because the chosen propeller's published maximum
test current is 61.6 A per motor and the initial operational throttle ceiling is
70%, where the test current is 25.7 A. It must receive direct cooling airflow,
and the initial aircraft must not use full-throttle output as a sustained
operating condition.

## Flight Controller and Compute

| Component | Selection | Placement rule |
| --- | --- | --- |
| Flight controller | Pixhawk 6X module with standard baseboard | At the center of gravity, arrow forward, on vibration isolation |
| Onboard compute | Jetson T5000 128 GB production module on a flight-suitable carrier | Centered and slightly aft on an upper structural deck |
| Thor operating envelope | Use the T5000 70 W power mode initially | Do not design around the 120/130 W modes |
| Compute power | Dedicated regulated supply designed for the selected carrier | Electrically isolated from noisy motor power as required by the final design |

The Thor developer kit remains a desk-development target only. The simulated
compute assembly, including module, carrier, cooling, regulator, and enclosure,
has a **900 g maximum mass allocation**. Its provisional simulation mass is
900 g until real components are selected and weighed. It must also sustain the
70 W mode at the worst expected ambient temperature without thermal throttling.

## Initial Sensor Suite and Mounting

Flight-critical sensors already represented by the Pixhawk are IMUs,
barometers, and magnetometer. Add the following external sensors to the digital
twin in this order:

| Sensor | Purpose | Initial mounting position |
| --- | --- | --- |
| Holybro H-RTK ZED-F9P Ultralight-class GNSS/compass | Global position, home, heading backup | Rear mast, above the power wiring and propeller plane |
| LightWare LW20/C-class rangefinder | Low-altitude and landing height | Bottom center, unobstructed view downward |
| Forward stereo/depth camera | Visual perception and forward depth | Nose center, level, outside propeller arcs |
| Livox Mid-360-class lidar | 360-degree obstacle geometry | Top center, above the upper deck with an unobstructed field of view |

The Mid-360 reference is 65 x 65 x 60 mm, 265 g, and averages 6.5 W. The LW20/C
reference is 30 x 20 x 43 mm and 19 g with cable. The camera is initially
modeled as an OAK-D-Lite-class stereo RGB/depth device. Exact camera mass,
enclosure, and optics remain parameters until the physical camera variant is
chosen.

## Mass and Center-of-Gravity Budget

| Group | Initial simulation mass |
| --- | ---: |
| Reinforced arms and custom center chassis | 0.350 kg provisional |
| Four motors | 0.500 kg |
| Propellers | 0.080 kg |
| ESC, distribution, and motor wiring | 0.150 kg |
| Pixhawk 6X and baseboard | 0.074 kg |
| GNSS | 0.064 kg provisional |
| Downward rangefinder | 0.019 kg |
| Thor compute assembly | 0.900 kg maximum allocation |
| 360-degree lidar | 0.265 kg |
| Stereo/depth camera | 0.061 kg provisional |
| Regulators and data harnesses | 0.250 kg |
| Battery | 1.350 kg provisional |
| Mounts, guards, and landing structure | 0.250 kg |
| Fasteners, adhesive, and mass-reconciliation allowance | 0.030 kg |
| **Estimated takeoff mass** | **4.343 kg** |

Freeze **4.5 kg as the design and simulated takeoff-mass limit**. At the nominal
4.343 kg estimate, hover requires approximately 1,086 g per rotor. Linear
interpolation of the published static curve puts hover near 46% throttle.

| Throttle test point | Total thrust | Ratio at 4.343 kg | Ratio at 4.5 kg |
| ---: | ---: | ---: | ---: |
| 40% | 3.044 kgf | 0.70:1 | 0.68:1 |
| 50% | 5.344 kgf | 1.23:1 | 1.19:1 |
| 60% | 7.484 kgf | 1.72:1 | 1.66:1 |
| 70% operational ceiling | 9.804 kgf | 2.26:1 | 2.18:1 |
| 100% short test point | 18.592 kgf | 4.28:1 | 4.13:1 |

Use 70% as the initial software motor-output ceiling. This preserves a little
over 2:1 thrust-to-weight at the design mass without treating the motor's short
full-load test as a continuous rating. Increase that ceiling only after thermal
and structural evidence supports it.

Interpolating the same bench curve gives roughly 0.8 kW propulsion power near
nominal hover. With 70 W compute, sensors, conversion losses, and an 80% usable
fraction of a 222 Wh pack, expect roughly **8-11 minutes**, not the idealized
energy-only result. Gazebo must model voltage sag and reserve policy rather than
assuming constant battery voltage.

Keep the assembled center of gravity within 5 mm horizontally of the geometric
center and below the propeller plane. Place the battery below the center plate
and move it fore/aft to balance the slightly aft compute assembly. Do not hide a
bad center of gravity with controller tuning.

The expected hover point, 427 mm wheelbase, and greater than 2:1 capped thrust
ratio are compatible with a stable ArduCopter tune. The high central mass will
increase roll and pitch inertia, so the Iris gains must not be copied. Begin
with conservative autotune bounds after validating motor order, vibration,
inertia, and actuator response. Strong-wind stability remains a test result,
not a BOM property.

## Gazebo Integration Decision

Model sensors by their physical capability and normalize their output at the
Icarus adapter boundary. Do not require a vendor-specific Gazebo plugin for the
first model.

| Real component | Gazebo Harmonic representation | Icarus path |
| --- | --- | --- |
| Pixhawk IMUs | SDF `imu` sensor, following the official Iris example | ArduPilot Gazebo JSON bridge into SITL |
| GNSS/compass | SDF `navsat` plus magnetometer/noise model | SITL flight state; normalized State Engine fields |
| LW20/C | One-beam SDF lidar/ray sensor | Gazebo Transport adapter; optionally MAVLink `DISTANCE_SENSOR` |
| OAK-D Lite-class camera | SDF RGB camera plus depth camera | Gazebo Transport adapter into the perception service |
| Mid-360-class lidar | Configurable SDF GPU lidar with 360 x 59 degree field of view | Gazebo Transport point cloud into the perception service |
| Motors/propellers | Published thrust/RPM/current lookup data | ArduPilot motor outputs through the official Gazebo plugin |

Gazebo Harmonic already provides IMU, lidar, camera, depth-camera, and NavSat
sensor primitives, while the installed official ArduPilot Gazebo repository
provides the SITL vehicle bridge and working Iris IMU/NavSat patterns. Consume
Gazebo Transport directly for the first implementation; ROS 2 is an optional
adapter, not a prerequisite for flying the simulated vehicle.

Use two lidar profiles:

- `fast`: reduced rays and update rate for routine development and LLM tests.
- `sensor_fidelity`: Mid-360-like field of view, rate, noise, latency, and
  dropout for perception evaluation.

This keeps the everyday simulation fast while preserving a repeatable
high-fidelity evaluation mode. The physical adapters later use DepthAI,
Livox SDK2, serial/I2C, or DroneCAN and publish the same normalized sensor
contracts.

## Physical Go/No-Go Gates

Do not purchase or fly this configuration unless all are true:

- The complete Thor assembly fits within its envelope and weighs at most 900 g.
- The aircraft weighs no more than 4.5 kg ready to fly.
- CAD proves blade, wiring, antenna, and cooling clearances under deformation.
- The center chassis survives a proof load derived from at least 2.5 times the
  aircraft weight and expected maneuver loads.
- A single-motor thrust stand reproduces the chosen propeller curve and passes
  a sustained thermal test at the intended operating points.
- The regulator powers Thor at 70 W without resets during battery sag and motor
  transients.
- Measured center of gravity and inertia replace all provisional values.

Failure of a gate forces an airframe/compute redesign. Controller tuning is not
an acceptable workaround for structural, thermal, or power failure.

## Parameters That Must Remain Configurable

- Total mass and full 3x3 inertia tensor
- Center-of-gravity offset
- Motor thrust and torque curves
- Motor time constants and maximum RPM
- Propeller direction and actuator order
- Battery voltage, capacity, internal resistance, and discharge curve
- Compute power draw and thermal throttling state
- Sensor pose, rate, latency, noise, dropout, and field of view
- Wind drag coefficients and projected areas

## Acceptance Gate

Phase 0.1 is accepted for simulation when:

- The SDF uses the geometry and component positions in this document.
- The mass table totals automatically from model configuration.
- The center of gravity and inertia are visible in a model-inspection command.
- Motor order and rotation directions match ArduCopter's selected frame class.
- The published motor test points are represented by a versioned data file.
- No developer-kit dimensions or mass are silently substituted for the intended
  flight compute assembly.

## Sources

- [NVIDIA Jetson Thor overview and developer-kit dimensions](https://developer.nvidia.com/blog/introducing-nvidia-jetson-thor-the-ultimate-platform-for-physical-ai/)
- [NVIDIA Jetson Thor power modes](https://docs.nvidia.com/jetson/archives/r39.2.1/DeveloperGuide/SD/PlatformPowerAndPerformance/JetsonThor.html)
- [NVIDIA Jetson Thor module datasheet](https://developer.nvidia.com/embedded/downloads)
- [Hobbywing XRotor 3115 specifications and thrust data](https://www.hobbywing.com/en/products/xrotor3115)
- [Holybro Pixhawk 6X specifications](https://holybro.com/products/pixhawk-6x-rev3)
- [Livox Mid-360 specifications](https://www.livoxtech.com/mid-360/specs)
- [Luxonis OAK-D Lite specifications](https://docs.luxonis.com/hardware/products/OAK-D%20Lite)
- [LightWare LW20/C specifications](https://lightwarelidar.com/shop/lw20-c-100-m/)
- [Gazebo Harmonic sensor documentation](https://gazebosim.org/docs/harmonic/sensors/)
- [Official ArduPilot Gazebo plugin](https://github.com/ArduPilot/ardupilot_gazebo)

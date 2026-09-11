# Simulation: Compact Two-Level Drone Baseline

Flight integration now exists as a separate dynamic model. See
[recorded flight and one-command launch](SIM-COMPACT-FLIGHT.md). The static sensor
fixture described below remains available for isolated sensor tests.

Supersedes the tall `akshu_icarus_v1` packaging candidate. One shallow electronics
bay and one underslung battery level replace the compute tower. Chamfered centre
plates, vented side ribs, a flush GNSS patch, integrated forward camera and small
lidar housing keep the body low. Overall height is approximately 217 mm versus
353 mm for the previous candidate; source arms, prop visuals and extended legs
are retained. The original supplied STL remains untouched.

This is deliberately a **simulation-first chassis**, not a claim that the
previous Thor/carrier/cooler hardware allocation fits inside it. Sensor housings
are generic visualization geometry, not vendor CAD. The earlier physical BOM
and mass/inertia assumptions have not been revalidated for this design.

## Working Gazebo sensors

Native Harmonic sensor definitions live in
`simulation/models/akshu_compact/model.sdf`. The validation world loads Sensors
(Ogre2), Imu and NavSat systems. These are actual sensor streams, not placeholder
meshes or ROS-specific plugins. Definitions follow the installed Gazebo examples
and [official sensor documentation](https://gazebosim.org/docs/harmonic/sensors/).

| Topic | Sensor | Configuration |
| --- | --- | --- |
| `/icarus/sensors/rgbd/image` | RGBD RGB image | 640 × 480, 15 Hz |
| `/icarus/sensors/rgbd/depth_image` | RGBD depth image | 640 × 480, 15 Hz, 0.05–30 m |
| `/icarus/sensors/lidar` | GPU lidar | 360 × 16 rays, 10 Hz, 360° azimuth, −15°…30° elevation, 0.1–40 m |
| `/icarus/sensors/range_down` | Single-ray GPU lidar | Downward, 20 Hz, 0.02–40 m |
| `/icarus/sensors/imu` | IMU | Body FLU, 200 Hz |
| `/icarus/sensors/navsat` | NavSat | 5 Hz |

These are generic idealized simulation sensors, not calibrated Livox/OAK/LW20
emulations. Rates are configured simulation-time rates, not benchmarked wall
clock throughput. Noise, calibration, timestamps across bridges, flight-frame
conversions and environmental robustness are future integration work.

## Verified

`scripts/simulation/test_akshu_sensors.py` launches an isolated headless Gazebo
world, subscribes to all six streams, requires repeated messages, checks image
sizes, verifies the lidar sees a known obstacle, checks the downward ground
distance, stationary IMU gravity and GPS location, then stops its process group.
It uses existing system Python/Gazebo bindings; no new dependencies.

Initial live result: PASS; 5,760 lidar rays, downward distance 0.884 m with the
body origin fixed 1 m above the test pad, IMU vertical acceleration 9.80665 m/s²,
GPS latitude −35.363262°. Evidence:
`logs/simulation/compact_sensors_0bb9725d/results.json`.

```bash
/usr/bin/python3 scripts/simulation/build_akshu_compact.py
/usr/bin/python3 scripts/simulation/test_akshu_sensors.py
/usr/bin/python3 scripts/simulation/render_mark4.py \
  --model model://akshu_compact --vehicle-height 0.191
```

The model remains static for this sensor check. It is not yet connected to
ArduPilot SITL and does not replace the existing flight-tested primitive model.
Next: integrate this visual/sensor baseline with dynamic collision geometry,
revised mass/inertia and the motor/SITL pipeline, then repeat hover/landing tests.

## Files

- `simulation/models/akshu_compact/icarus_compact_mm.stl`: revised assembly, mm.
- `simulation/models/akshu_compact/meshes/`: individual Gazebo meshes, metres.
- `simulation/worlds/akshu_sensor_validation.sdf`: isolated sensor test scene.
- `logs/simulation/akshu_compact_render/`: inspected Gazebo detail/top/front/side renders.

STL carries geometry only; working sensor definitions are in SDF. This combined
assembly is not a single printable solid or a fabrication drawing.

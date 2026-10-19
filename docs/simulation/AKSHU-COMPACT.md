# Compact Two-Level Drone Simulation Baseline

Flight integration now exists as a separate dynamic model. See
[recorded flight and one-command launch](COMPACT-FLIGHT.md). The static sensor
fixture described below remains available for isolated sensor tests.

Supersedes the tall `akshu_icarus_v1` packaging candidate. One shallow electronics
bay and one underslung battery level replace the compute tower. Chamfered centre
plates, vented side ribs, a flush GNSS patch, integrated forward camera and small
lidar housing keep the body low. Overall height is approximately 217 mm versus
353 mm for the previous candidate; source arms, prop visuals and extended legs
are retained. The original supplied STL remains untouched.

This is deliberately a **simulation-first chassis**, not a fabrication claim.
The current visuals and SDF mounts are hardware-specific proxies for the
Icarus7 D435i, BRIO, MID-360, ARK Flow, H-RTK and Pixhawk layout; they are not
vendor CAD. The component-derived flight model is now 4.4745 kg. It still
requires as-built mass, inertia, clearance, vibration and thermal validation.

## Working Gazebo sensors

Native Harmonic sensor definitions live in
`simulation/models/akshu_compact/model.sdf`. The validation world loads Sensors
(Ogre2), Imu and NavSat systems. These are actual sensor streams, not placeholder
meshes or ROS-specific plugins. Definitions follow the installed Gazebo examples
and [official sensor documentation](https://gazebosim.org/docs/harmonic/sensors/).

| Topic | Hardware role | Configuration |
| --- | --- | --- |
| `/icarus/sensors/d435i/{image,depth_image,points}` | Forward Intel RealSense D435i | 848×480 RGB-D at 30 Hz; 0.3–100 m RGB rendering and 0.3–10 m depth, separate 200 Hz IMU |
| `/icarus/sensors/brio_down` | Downward Logitech BRIO | RGB-only 1280×720 at 30 Hz |
| `/icarus/sensors/landing_depth_proxy/*` | Simulation-only landing geometry | 320×180 RGB-D at 15 Hz; not BRIO hardware |
| `/icarus/sensors/mid360` | Livox MID-360 envelope | 500×40 rays at 10 Hz, 360° × −7…52°, 0.1–40 m; 200 Hz IMU |
| `/icarus/sensors/ark_flow_range` | ARK Flow ToF array | 8×4 rays at 20 Hz, 12.4°×6.2°, 0.08–30 m; 200 Hz IMU |
| `/icarus/sensors/ark_flow/optical_flow` | ARK Flow simulation proxy | BRIO phase-correlation estimate; explicitly non-EKF and no flight authority |
| `/icarus/sensors/h_rtk_gnss`, `/h_rtk_compass` | H-RTK ZED-F9P + RM3100 | NavSat 8 Hz, magnetometer 50 Hz |
| `/icarus/sensors/pixhawk_imu_*` | Pixhawk 6X Pro triple IMU | 500, 500 and 200 Hz independent noisy streams |
| `/icarus/sensors/pixhawk_{magnetometer,barometer,barometer_secondary}` | Pixhawk BMM150 + dual barometer topology | 50 Hz magnetometer; two 30 Hz pressure streams |

The complete vendor-spec mapping and every deliberate approximation are in
[Gazebo hardware fidelity](GAZEBO-HARDWARE-FIDELITY.md). Publication rates are
simulation-time rates; the integrated run reports actual wall throughput.

## Verified

`scripts/simulation/test_akshu_sensors.py` launches an isolated headless Gazebo
world, subscribes to all native hardware-facing streams, requires repeated messages, checks image
sizes, verifies the lidar sees a known obstacle, checks the downward ground
distance, triple and auxiliary IMUs, dual compasses, dual barometers and GPS,
then stops its process group.
It uses existing system Python/Gazebo bindings; no new dependencies.

The MID-360 now produces 20,000 rays per 10 Hz frame and the ARK proxy 32 rays
per 20 Hz frame. Each run writes a fresh result beneath
`logs/simulation/compact_sensors_*`.

```bash
/usr/bin/python3 scripts/simulation/build_akshu_compact.py
/usr/bin/python3 scripts/simulation/test_akshu_sensors.py
/usr/bin/python3 scripts/simulation/render_mark4.py \
  --model model://akshu_compact --vehicle-height 0.191
```

The isolated model remains static for this particular sensor check. The sibling
`akshu_compact_sitl` model is the dynamic ArduPilot vehicle used by
`start-sim`; it is no longer a separate primitive vehicle.

## Files

- `simulation/models/akshu_compact/icarus_compact_mm.stl`: revised assembly, mm.
- `simulation/models/akshu_compact/meshes/`: individual Gazebo meshes, metres.
- `simulation/worlds/akshu_sensor_validation.sdf`: isolated sensor test scene.
- `logs/simulation/akshu_compact_render/`: inspected Gazebo detail/top/front/side renders.

STL carries geometry only; working sensor definitions are in SDF. This combined
assembly is not a single printable solid or a fabrication drawing.

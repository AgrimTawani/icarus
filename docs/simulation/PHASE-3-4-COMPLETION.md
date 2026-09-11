# Phase 3–4 Simulation Acceptance — 2026-09-12

Status: **passed**, using the user's explicit approval: “Use the automated
simulation gate.” Five consecutive automated flights replace the original five
manual flights. No screenshots, image pixels or videos were saved during this
acceptance work. No additional dependencies were installed.

## Phase 3: compact digital twin

The approved two-level compact chassis is retained. The imported source STL is
unchanged. The dynamic model has 4.343 kg total mass, four 20 g rotor allocations,
explicit component-based mass/inertia and the imported stretched-X motor layout.
These are provisional engineering allocations, not measured hardware properties.

Acceptance results:

- Five consecutive takeoff/hover/directional-motion/yaw/land/disarm missions
  passed with all nine public sensor streams. Four used nominal sensor settings;
  the fifth used noise, with seeds 100–104.
- Hover altitude: 2.984–3.003 m; maximum horizontal drift: 0.0336 m;
  maximum roll/pitch: 0.3523°. Motor mean PWM: approximately 1456.4.
- Published reference-curve interpolation predicts 4.341 kgf at that PWM,
  within 0.051% of the 4.343 kg weight-equivalent. This checks model consistency,
  not independent proof of the real motor/propeller curve.
- Four individual motor tests plus collective thrust passed: channel order,
  spin and torque directions, force/vertical-motion response, configured
  40/80 ms response constants and speed ceiling. Current dynamics passed the
  acceptance tolerances without further gain changes.
- Passive drops from body z=0.8 m passed with and without visual geometry.
  Rest height was 0.19009961 m in both cases; removing visuals changed neither
  physics nor the observed trajectory. The solver briefly allowed 2.829 mm
  skid penetration, then settled; no rebound above the contact plane occurred.
  The test bounds penetration below 8 mm, rebound below 25 mm and final
  height/variation within 1 mm. This is not a zero-penetration contact claim.
- 65 unique visual meshes contain 30,170 triangles, versus 266,716 in the
  supplied source (88.7% reduction); eight primitive collision shapes remain.
- Effective flight real-time factor was 0.967–0.979 on this laptop, above the
  0.8 gate. Performance on other devices remains unverified.
- Three structural tests passed for mass/inertia, rotor transforms and frames.

Evidence (paths relative to repository root):

- `logs/simulation/phase34_acceptance_913c080e/results.json`
- `logs/simulation/mark4_motors_20260912T021826_20e9a5/`
- `logs/simulation/compact_landing_2de9d115/results.json`

## Phase 4: native sensors and public health

Poses below are XYZ metres relative to base_link, whose axes are forward/left/up
(FLU). Rotations are zero unless specified. Gazebo world axes are ENU; MAVLink
local position uses NED. Timestamp units are simulation seconds; freshness uses
monotonic wall time.

| Channel | Simulated mount XYZ | Values/frame | Measured publication rate |
| --- | --- | --- | --- |
| RGB | .135, 0, −.028 | 640×480 image, camera forward +X | 15.15 Hz |
| Depth | same RGBD mount | metres along camera viewing direction | 15.15 Hz |
| Lidar | 0, 0, .028 | range metres, scan angles radians in sensor frame | 10 Hz |
| Down range | .110, 0, −.108; pitch +π/2 | metres, points along body −Z | 20 Hz |
| Public IMU | 0, 0, −.026 | m/s² and rad/s, FLU | 200 Hz |
| GPS/NavSat | −.077, 0, .003 | geodetic degrees, elevation metres, ENU velocity m/s | 5 Hz |
| Compass | −.077, 0, .003 | tesla, sensor/body frame | 50 Hz |
| Barometer | 0, 0, −.026 | pressure Pa, scalar | 30.30 Hz |
| Battery | model-level electrical state | V, A, Ah; charge/capacity gives SOC 0–1 | approximately 49 Hz |

The dedicated ArduPilot IMU is separate, at the public IMU mount but with roll
π (FRD), configured at 1,000 Hz. It and physical state feed the JSON bridge.
ArduPilot's GPS, compass, barometer and battery remain SITL-generated. Public
Gazebo sensor topics are available to perception/health consumers but are **not
all fused into ArduPilot navigation**. Obstacle avoidance is not implemented.

The Gazebo magnetometer is explicitly configured to output tesla and not NED;
its defaults otherwise produce gauss despite the protobuf field name. Observed
field magnitude at the configured location was approximately 55.08 µT. Pressure
is approximately 101313 Pa in the static fixture: its local atmospheric datum
must not be confused with GPS's 584 m world-elevation datum. Gazebo's battery
plugin with `fix_issue_225=true` publishes raw `percentage` on a **0–100** scale;
consumers should use charge/capacity for normalized SOC.

### Noise, latency and failure checks

`config/simulation/sensor_profiles.json` defines nominal/noisy profiles, Gaussian
means and standard deviations. The noisy configuration includes accelerometer
0.03 m/s², gyro 0.002 rad/s, GPS position 0.3 m, magnetometer 0.2 µT, pressure
2 Pa, lidar 0.02 m and downward range 0.005 m standard deviations. RGBD uses
native camera noise. These are configurable experimental assumptions, not
vendor-calibrated error distributions. Battery uses configurable capacity,
load, voltage and resistance, not Gaussian ADC noise or a validated propulsion
power model.

Each of the nine channels passed an independent `--only` run. All-channel
nominal and noisy runs passed rate, finite-value, geometry/electrical and
statistical checks. Noise magnitude checks cover sampled accelerometer, GPS
altitude, magnetic-field norm, pressure, lidar, downward range and depth;
RGB's aggregate intensity varies as expected. This does not characterize every
axis, pixel or correlation. Battery charge declines under its configured load.

Production freshness logic is shared with the tests. Every channel passed
drop/frozen-timestamp detection and recovery. Eighteen additional live fault
injections exercised the real recorder, with stale detection approximately
1.97–2.28 seconds after injection, and all channels recovered. The configured
stale deadline is 2 seconds. A simulated 200 ms consumer delay was also checked
for every channel using deterministic-clock tests; this is not a measured
hardware/end-to-end latency distribution.

Faults operate at the **consumer observation/health boundary**; they do not
unplug native Gazebo sensors or inject EKF failures into ArduPilot. Health is
published as JSON in Gazebo StringMsg on `/icarus/health/sensors` and saved in
`sensors/health.json`. In isolated test partitions, `/icarus/test/sensor_fault`
accepts `{"channel":"imu","mode":"freeze"}`; modes are normal, drop, freeze,
and delay. The launcher treats unhealthy streams as a failed simulation run and
tears it down; this behavior is not a real-flight safety supervisor.

Optical flow is not required for the initial GPS-guided missions and is deferred.
RGB transport is prepared for future scene-perception work; camera pixel capture
and dataset collection require a separate approved workflow.

Evidence:

- Nominal: `logs/simulation/phase4_nominal_e553461d/results.json`
- Final noisy regression: `logs/simulation/phase4_noisy_e91b5537/results.json`
- Live per-channel faults: `logs/simulation/phase4_nominal_8b7eb6c6/results.json`
- Independent RGB/depth/IMU/GPS/compass/barometer/lidar/range/battery runs:
  `phase4_nominal_9d3a9339`, `ce4ae8e8`, `b490250e`, `02d32ee3`,
  `9c9e87d4`, `b0feef4f`, `66f79a97`, `2d384de8`, `f5d20df9`
  (each suffix uses the same `logs/simulation/phase4_nominal_` prefix).
- Final launcher regression: `logs/simulation/launcher_faults_135122c7/results.json`;
  occupied port, duplicate launcher, interruption, stale streams and recorder
  exit all passed, with owned processes stopped and ports released.

## Reproduce without media capture

```bash
/usr/bin/python3 scripts/simulation/test_compact_model.py
/usr/bin/python3 scripts/simulation/test_compact_landing.py
/usr/bin/python3 scripts/simulation/test_phase4_sensors.py --profile nominal --live-faults
/usr/bin/python3 scripts/simulation/test_phase4_sensors.py --profile noisy
/usr/bin/python3 scripts/simulation/test_phase4_sensors.py --only imu
/usr/bin/python3 scripts/simulation/run_phase34_acceptance.py
/usr/bin/python3 scripts/simulation/test_compact_launcher.py
```

Run flight/launcher suites sequentially, with their ports free. The launcher
builds the selected sensor profile and uses an isolated Gazebo partition. Logs
retain numeric sensor protobufs; RGB/depth messages retain metadata only, with
the data payload removed. The verifier checks this and creates no media.

Next: original Phase 5 world/scenario layers. Wind/obstacle mission acceptance,
full sensor fusion, API/safety/autonomy layers, dataset capture, portability and
real-aircraft validation remain separate milestones.

Primary implementation references: [Gazebo sensor guide](https://gazebosim.org/docs/harmonic/sensors/),
[Magnetometer system](https://github.com/gazebosim/gz-sim/blob/gz-sim8/src/systems/magnetometer/Magnetometer.cc),
[LinearBatteryPlugin](https://github.com/gazebosim/gz-sim/blob/gz-sim8/src/systems/battery_plugin/LinearBatteryPlugin.cc).

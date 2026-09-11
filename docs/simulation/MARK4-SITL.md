# Mark4 SITL Bring-Up

## Commands

Run from the repository root with Gazebo Harmonic and the existing dependency
environment. The launcher builds only the small project-owned motor adapter;
it does not install packages or rebuild ArduPilot.

```bash
# Fresh simulator, fresh SITL EEPROM, parameter readback, 3 m hover, land.
/usr/bin/python3 scripts/simulation/launch_mark4.py

# Also check north/east motion and yaw against Gazebo ground truth.
/usr/bin/python3 scripts/simulation/launch_mark4.py --direction-check

# Validate a hover, pause physics, then open a close-up GUI for inspection.
/usr/bin/python3 scripts/simulation/launch_mark4.py --review
```

Close the review GUI or press Ctrl+C in the launcher to stop the owned processes.
The review deliberately pauses at hover. Pressing play resumes physics; because
the test client has disconnected, the configured GCS-loss policy will request
landing after the timeout. The review is intended for orbiting/zooming around
the paused vehicle, not interactive flight control.

## Connections

1. ArduCopter `--model JSON` and the official ArduPilot Gazebo plugin exchange
   state and outputs over localhost UDP port 9002 in lockstep at a 1 ms physics
   step. Gazebo owns the vehicle physics.
2. A Gazebo IMU on `base_link`, oriented FRD, provides the JSON bridge's inertial
   measurements. The base link itself remains FLU. GPS/barometer/compass are
   currently SITL's synthetic baseline sensors; the full Phase 4 sensor suite
   has not been added.
3. ArduPilot output channels 0–3 publish normalized `gz.msgs.Double` commands
   on `/icarus/vehicle_01/sitl/motor/{0,1,2,3}`.
4. `IcarusMotorBridge` maps these commands to physical rad/s, writes the model's
   `gz.msgs.Actuators` component, and zeros an input after 250 ms without an
   update in advancing simulation time. A paused simulator does not age inputs.
5. Gazebo's four motor-model systems are the only joint-velocity owners. Direct
   bench topic commands do not override the SITL adapter's Actuators component.
6. The scripted test client connects to localhost TCP 5760, sends a GCS
   heartbeat, and uses MAVLink GUIDED/LAND commands.

Each run uses a unique Gazebo transport partition. UDP/TCP ports are checked
before startup; the launcher refuses occupied ports rather than stopping other
processes. Every run owns a separate SITL state directory and process groups.
Shutdown uses bounded INT/TERM/KILL escalation for those groups only.

## Parameters and provisional physics

The profile is `simulation/parameters/mark4_v2_base.parm`, layered on the
checked-out ArduPilot `copter.parm`. Every explicit setting is read back over
MAVLink and compared before arming. This checkout uses `ARMING_SKIPCHK=0`
(no checks skipped), `ATC_ANGLE_MAX` in degrees, `WP_SPD*` in m/s,
`LAND_SPD_MS`, `RTL_ALT_M` and `MAV_GCS_SYSID`; older names must not be assumed
to work. Physical RC-loss checking is disabled on this simulated GCS bench;
GCS-loss LAND and geofencing are enabled.

The rotor adapter linearly interpolates the BOM's available throttle/thrust
samples: 0/0, 40%/0.761 kgf, 50%/1.336 kgf, 60%/1.871 kgf, 70%/2.451 kgf per
rotor. It clamps commands to 70%, converts thrust to speed using F=k*omega²,
and retains the existing 40/80 ms speed response. The segment below 40% and
the motor torque/response coefficients remain provisional. This is not a
complete imported manufacturer curve, calibrated battery model, or hardware
performance prediction. Full wind/drag, sensor fidelity, and CAD-derived
inertia remain later work.

## Acceptance checks and artifacts

The basic check requires a normal arm, takeoff to 3 m, and a ten-second hover
after settling. Every fresh hover sample must remain within ±0.5 m vertically,
1 m horizontally of the launch origin, and 10 degrees roll/pitch. At least
40 fresh samples are required. LAND must complete with disarming and a reported
relative altitude within 0.3 m of ground.

The directional check adds one metre north, one metre east, return to the
origin, and an absolute 135-degree heading. It compares Gazebo ENU positions
with NED targets (0.3 m tolerance), and checks the heading against Gazebo yaw
(expected -45 degrees, 5-degree tolerance).

Each `logs/simulation/mark4_flight_*` directory contains:

- launch commands, partition, third-party revisions and input hashes;
- build, simulator, SITL and controller logs;
- the verified parameter values;
- timestamped JSONL telemetry and the SITL flight log;
- result JSON with hover and optional directional measurements.

The GUI uses primitive geometry to expose the frame, battery, avionics bay,
skids and rotors clearly. Vendor meshes, full payload housings and detailed
sensor mounts are not represented yet. All visible geometry is project-authored.

## Visual-review checkpoint results

Five consecutive scripted flight checks passed on 2026-09-12 using the
readback-verified profile. The accepted run directories under `logs/simulation/`
are `mark4_flight_20260912T010827_168ff2`,
`mark4_flight_20260912T011029_9174b6`,
`mark4_flight_20260912T011144_04dbb3`,
`mark4_flight_20260912T011258_dec9d2`, and
`mark4_flight_20260912T011413_fb2080`.

All runs recorded 200–201 fresh hover samples. Across the runs, hover altitude
was 2.985–3.001 m, horizontal drift at most 0.030 m, and roll/pitch at most
0.289 degrees. Every run landed and disarmed. The first accepted run also
checked translation and heading against Gazebo: commanded north mapped to +Y,
east to +X, and heading 135 degrees produced Gazebo yaw -43.67 degrees.

All six project SDF files passed validation, including runtime world loading.
The current primitive model can also be rendered without the desktop:

```bash
/usr/bin/python3 scripts/simulation/render_mark4.py
```

This creates static Gazebo inspection images on the pad, not flight evidence.
The first reviewed images are in `logs/simulation/mark4_render_5c29dd02/`.
The renderer uses actual model geometry and two camera sensors; no AI image
generation or desktop unlock is involved. It selects the installed NVIDIA EGL
vendor for its own process when available, without changing system drivers.

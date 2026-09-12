# Compact Flight and One-Command Launch

Original roadmap Phases 3 and 4 now pass their simulation gates, with the user's
approved automated-flight substitution. See [acceptance evidence and scope](PHASE-3-4-COMPLETION.md).
No hardware calibration or unrestricted autonomy is claimed.

## Run

From the repository root:

```bash
./scripts/sim --gui
```

This is an **automated flight**, not just opening Gazebo. It verifies parameters
and navigation, arms, takes off to 3 m, measures a ten-second hover, moves 1 m
north, then 1 m east, returns, commands yaw, lands and disarms. After a short
ground-rest check, it verifies numeric telemetry and image metadata,
and closes its simulator, GUI, SITL and recorder processes.

Other modes:

```bash
./scripts/sim                   # Same automated test, headless
./scripts/sim --preflight-only  # Sensor/navigation checks, never arms
./scripts/sim --seed 104
```

Ctrl+C stops the owned simulation processes. Closing the GUI during the test is
treated as an interrupted run, not a successful flight. This launcher is strictly
for local simulation; it does not accept a real aircraft endpoint.

## Startup and shutdown contract

1. Check existing Gazebo, build tools and ArduPilot/plugin executables. Refuse
   occupied MAVLink TCP 5760 / JSON UDP 9002 ports or a duplicate launcher.
2. Rebuild the deterministic flight SDF and local motor adapter. No packages
   are installed and no unrelated process is terminated.
3. Create an isolated Gazebo partition and fresh per-run SITL EEPROM directory.
4. Start Gazebo with rendering, SITL and the sensor recorder. Require repeated
   messages on all nine physical sensor channels plus the atmosphere stream
   before running the controller.
5. Read back every configured ArduPilot parameter and await GPS/navigation
   readiness before arming. Parameter responses have bounded retries.
6. Monitor child processes, recorder heartbeat and stale streams during flight.
   A child exit, two-second stale sensor timestamp or stalled recorder causes failure
   and simulator teardown; it never silently yields a passing result.
7. On success, stop and flush recording, decode all saved protobuf records,
   check message counts/timestamps, and verify that image pixels were not saved.
8. On success, failure or interruption, stop owned process groups with bounded
   INT/TERM/KILL escalation and write `launch.json` with exit codes.

These are tested operational safeguards, not a guarantee against every possible
OS/GPU failure. A forced kill of the launcher itself cannot run Python cleanup.

## Model and sensor connections

`simulation/models/akshu_compact_sitl/model.sdf` retains the approved compact
visuals and mounts. Its four propeller meshes move with their own revolute links;
their body-frame offsets are preserved. Motor geometry uses the imported
approximately 443.8 mm stretched-X diagonal, not the old square-X coordinates.

`config/simulation/vehicle_components.json` is the component-level source of
truth. The builder automatically derives total/base mass, centre of gravity and
the full inertia tensors using each component's geometry, mass and mounting
transform, including separate propeller link inertias. It writes the inputs,
provenance and source hash to `mass_properties.json`; no aggregate mass, CG or
inertia tensor is manually copied into the generated SDF. Total mass is currently
4.343 kg. Inputs marked as design targets still require as-built measurement.

Motor rise/decay, thrust mapping, torque and output limits reuse the tested
provisional model. Landing contacts use simple skids; detailed mesh contact and
propeller strikes are not simulated.

ArduPilot's JSON bridge consumes a dedicated 1,000 Hz **FRD** IMU with the same
single physical-noise configuration used by the public sensor model. The
separately recorded 200 Hz public IMU uses **FLU**. ArduPilot's GPS,
compass, barometer and battery remain SITL-generated; the public Gazebo streams
are independently tested but are **not yet fused into navigation or
obstacle avoidance**. Gazebo world poses are ENU; MAVLink local positions are NED.
Automated movement/yaw tests compare both representations.

## Per-run artifacts

Each invocation prints a unique `logs/simulation/compact_flight_<timestamp>_<id>`
directory containing:

- `result.json`: flight gates, hover measurements, direction checks and landing.
- `launch.json`: final outcome, inputs' hashes, dependency Git revisions and
  child exit codes. `runtime.json` identifies owned process groups/partition.
- `telemetry.jsonl`, `parameters_verified.json`, SITL native logs and process logs.
- `sensors/*.pbstream`: length-prefixed, zlib-compressed Gazebo protobuf
  messages, with RGB/depth pixel payloads removed; `schema.json` records types/topics and format; `index.jsonl` records
  simulation timestamps, receipt times and byte offsets.
- `sensors/health.json`: received/recorded counts, freshness and byte budget.
- `sensors/wind.pbstream`: seeded three-axis local wind samples used by the
  aerodynamic model.
- `recording_verified.json`: numeric integrity checks only; no preview files.

Recording is intentionally downsampled: RGB/depth ≤1 Hz, lidar ≤5 Hz, range
≤10 Hz, public IMU/compass ≤20 Hz, GPS/battery ≤5 Hz and barometer ≤10 Hz. Sensor publication
rates remain unchanged. Actual saved rates are measured in the verification
report; scheduling can yield lower rates. Original message timestamps are
preserved. Files have a 512 MiB compressed-data cap per run.

Camera sensors run to validate transport, but this workflow saves no pictures,
image pixels or videos. Earlier historical runs may contain previews; the current
launcher and verifier do not create them. Training-dataset capture needs a later
explicitly approved recording workflow.

## Tests

```bash
# Four individual motors plus collective thrust/response/limit checks
ICARUS_MOTOR_TEST_MODEL="$PWD/simulation/models/akshu_compact_sitl/model.sdf" \
  /usr/bin/python3 scripts/simulation/test_mark4_motors.py

# Port conflict, duplicate launch, interrupt, stale sensors and recorder exit
# Run only when no flight launcher is active; tests own their child processes.
/usr/bin/python3 scripts/simulation/test_compact_launcher.py

# Recheck numeric recordings without generating any media
/usr/bin/python3 scripts/simulation/inspect_flight_recording.py \
  logs/simulation/<run-directory>
```

Historical full-flight evidence (before the current no-media policy):

| Run | Hover altitude | Maximum drift | Maximum tilt | Outcome |
| --- | --- | --- | --- | --- |
| `20260912T015130_65ec42` | 2.987–3.001 m | 0.0342 m | 0.339° | Full mission passed |
| `20260912T015345_3776ed` | 2.985–3.001 m | 0.0334 m | 0.366° | Full mission + post-landing rest passed |
| `20260912T015807_bdd4bf` | 2.986–3.002 m | 0.0332 m | 0.310° | GUI + full mission + recording verification passed |
| `20260912T020117_b83854` | 2.986–3.002 m | 0.0317 m | 0.346° | Final launcher, framing and recording checks passed |

The second run's three post-landing ground-truth body heights were all
0.19010 m. Individual-motor/collective tests passed in
`logs/simulation/mark4_motors_20260912T015504_ecc124`.

An early GUI trial (`20260912T015613_03cea5`) timed out on parameter readback
before arming and cleaned up. The parameter transport deadline was subsequently
increased, retaining mandatory value verification.

All five launcher failure checks passed in
`logs/simulation/launcher_faults_5571f89f/results.json`: occupied port, duplicate
launcher, interrupt, frozen sensor publication (paused simulation), and recorder
exit. The injected-failure runs never armed; every owned process group exited
and the MAVLink port was released. Three structural regression tests also pass
with `scripts/simulation/test_compact_model.py`.

## Next boundary

Phase 5 world/scenario layers are complete; see
[Phase 5 acceptance](PHASE-5-COMPLETION.md). Phases 6 and 7 subsequently closed
the operator-lifecycle and portable-runtime gates. The typed Drone API/MAVLink
gateway, safety supervisor, DCM control, obstacle avoidance and real-hardware
validation remain later work.

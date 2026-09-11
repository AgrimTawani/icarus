# Simulation: Official Iris Connection Baseline

## Result

Phase 1 passed on 2026-09-12 (Asia/Kolkata). The official Iris completed three
consecutive automated takeoff-hover-land runs using Gazebo Harmonic, the
official ArduPilot Gazebo plugin, ArduCopter SITL, and a deterministic pymavlink
test client.

Run command:

```bash
./scripts/simulation/run_iris_smoke_test.sh
```

The launcher derives all paths from its repository location, creates an
isolated SITL state directory for every run, loads the official Iris parameter
file, captures artifacts, and terminates its complete process groups on success
or failure.

## Proven Connection Path

```text
Gazebo Iris IMU and model state
    -> official ArduPilot Gazebo plugin
    -> JSON/UDP input to ArduCopter SITL
    -> EKF and vehicle state
    -> MAVLink TCP 5760
    -> deterministic pymavlink controller
    -> GUIDED arm/takeoff/LAND commands
    -> SITL motor outputs
    -> JSON/UDP output to Gazebo rotor joints
```

The default instance uses UDP 9003 toward ArduPilot and UDP 9002 from
ArduPilot. Successful physical motion, altitude feedback, controlled landing,
and disarm prove the state and actuator directions together; a heartbeat alone
would not be sufficient evidence.

MAVProxy remains available for interactive debugging, but it is deliberately
not in the automated test path. The test connects directly with pymavlink so
the launch is deterministic and suitable for later CI execution.

## Three-Run Gate

| Run ID | Result | Peak altitude | Hover range | Duration |
| --- | --- | ---: | ---: | ---: |
| `20260911T183807Z` | Pass | 5.06 m | 4.64-5.06 m | 46.823 s |
| `20260911T183919Z` | Pass | 5.06 m | 4.64-5.06 m | 46.847 s |
| `20260911T184011Z` | Pass | 5.06 m | 4.64-5.06 m | 46.801 s |

Each run produced 51 hover altitude samples, landed, disarmed, and left no
ArduCopter, `sim_vehicle.py`, or Gazebo server process running.

Artifacts are under:

```text
logs/simulation/iris_smoke/<run-id>/
|-- controller.log
|-- gazebo.log
|-- result.json
|-- sitl.log
`-- sitl_state/
```

## Problems Found and Preserved

The failed development runs are intentionally retained because they verify that
the launcher captures diagnosable failures:

- The installed pymavlink version does not accept a timeout argument on its
  convenience arm-wait function. The controller now uses an explicit bounded
  heartbeat loop.
- A stale `third_party/ardupilot/eeprom.bin` caused `Check frame class and type`
  arming rejection. Every run now wipes EEPROM in its own state directory and
  explicitly loads `gazebo-iris.parm`.
- Direct SITL TCP connections initially emitted only heartbeat messages. The
  client now requests the required telemetry streams before checking navigation
  readiness.
- The controller now waits for a 3D GPS fix and global position before entering
  GUIDED mode and arming.

Gazebo emitted non-fatal rendering and mesh-collision warnings from the official
gimbal example in server-only mode. They did not prevent IMU initialization,
lockstep flight, or camera initialization. The Icarus model will use primitive
collision geometry rather than mesh collision geometry.

## Baseline Versions

- Gazebo Sim: 8.15.0
- ArduPilot revision: `14c871f2732c`
- ArduPilot Gazebo revision: `082a0fe231f6`
- World: official `iris_runway.sdf`
- Vehicle defaults: official `gazebo-iris.parm`

These revisions must be recorded in the future reproducible build manifest.

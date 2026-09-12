# Icarus Simulation Operator Guide

## Standard Commands

Run from the repository root:

```bash
# Terminal 1: world, vehicle, ArduPilot and sensors only
./scripts/start-sim --scenario wind_light --gui

# Terminal 2: select exactly one control client
./scripts/manual-control
./scripts/run-mission --mission takeoff_hover_land

# Optional independent ground-station camera viewer
./scripts/view-camera

# Existing one-shot automated acceptance mode remains available
./scripts/sim --scenario empty_validation
./scripts/sim --scenario wind_strong --gui

# Validate the declared operating-limit rejection without starting Gazebo
./scripts/sim --scenario wind_limit_reject
```

Stop the simulator with `Ctrl+C` in its terminal. The launcher handles the signal,
stops owned children and releases TCP 5760 and UDP 9002. Do not kill individual
children first unless diagnosing a cleanup failure.

## Separated Runtime Model

`start-sim` owns only the shared simulation foundation: scenario generation,
Gazebo, the vehicle, ArduPilot SITL, simulated sensors, faults, logs and cleanup.
It publishes the local endpoint `tcp:127.0.0.1:5760` in an ignored active-session
file. It does not arm or command the drone.

Control runs independently in another terminal. `manual-control` sends bounded
RC overrides; `run-mission` runs the deterministic acceptance mission; future
Drone API and DCM processes will connect through the same session contract.
Do not run two control clients simultaneously.

## Keyboard and Xbox Manual Flight

Start the simulator with `--gui`, then run `./scripts/manual-control`. With an
Xbox controller attached, the client runs without a window and remains active
while Gazebo or the camera viewer has focus. Use `./scripts/manual-control --hud`
to open the keyboard/HUD window; keyboard input requires that window to have
focus, but Xbox input does not.

| Function | Keyboard | Standard Xbox mapping |
| --- | --- | --- |
| Roll / pitch | `A/D` and `W/S` | Right stick |
| Yaw / throttle | `Q/E` and `Up`/`Down` | Left stick |
| Arm | `Enter` | A |
| Take off to 3 m | `T` | Y |
| Hold in LOITER | `H` | Start |
| Land | `L` | B |
| Return to launch | `R` | X |
| Ground-only disarm | `Backspace` | Back |
| Stabilize mode | `1` | Hold LB + D-pad left |
| Alt Hold mode | `2` | Hold LB + D-pad down |
| Loiter mode | `3` or `H` | Hold LB + D-pad up |
| Acro mode | `4` | Hold LB + D-pad right |
| Exit | `Esc` or close window | — |

The Xbox throttle is deliberately zero-based rather than centered like a normal
altitude-control stick: center or downward travel commands 0%, and upward travel
maps linearly to 0–100%. Because the Xbox stick springs to center, releasing it
returns throttle to 0%; it must be held at the required position. Exiting while
armed requests `LAND` and waits for disarm before releasing RC overrides. The
client is hard-restricted to the localhost SITL endpoint and cannot connect to a
physical aircraft.

An arm request may arrive while the simulated GPS/EKF is still establishing its
position. One arm press remains pending for up to 45 seconds and retries every
three seconds; the pilot window shows GPS fix and local-position readiness plus
the latest ArduPilot pre-arm reason. Disarm, LAND or RTL cancels a pending arm.

Mode switching is guarded by the Xbox left bumper to prevent an accidental
D-pad press from selecting ACRO. `LOITER` holds position and altitude;
`ALT_HOLD` and `LOITER` still interpret the RC throttle channel through
ArduPilot's climb/descent logic, while `STABILIZE` and `ACRO` use it as manual
throttle. `STABILIZE` self-levels; `ACRO` provides rate control without
self-leveling. Use ACRO only with enough clearance to recover.

List SDL-detected controllers with:

```bash
./scripts/manual-control --list-controllers
```

## Forward Camera Feed

`start-sim` owns the drone-side camera adapter and publishes the forward RGB
camera as low-latency H.264 over RTP/UDP port 5600. The ground-station viewer is
an independent process:

```bash
./scripts/view-camera
```

The viewer may start before or after the simulator and reports live FPS and
frame freshness in its terminal. Closing it does not stop or otherwise control
the aircraft. The stream is not recorded. A physical camera adapter will retain
the same H.264/RTP boundary, with its destination configured to the ground
station address; the viewer therefore remains source-agnostic. Override the
local receive port with `./scripts/view-camera --port PORT` when required.
To send the simulated onboard stream to another machine, launch with
`--video-destination GROUND_STATION_IPV4 --video-port PORT` and open the same
UDP port in the ground-station firewall.

## Scenario Catalog

| Name | Environment and purpose | Current expectation |
| --- | --- | --- |
| `empty_validation` | Empty, still-air baseline | Pass |
| `wind_light` | Mixed village, constant 1.5 m/s wind | Pass |
| `wind_strong` | Mixed village, constant 5 m/s wind | Known failing stress case |
| `wind_gusting` | Mixed village, 3 m/s with 60% gust amplitude | Pass target |
| `wind_direction_change` | Mixed village, 2.5 m/s and ±90° swing | Pass target |
| `wind_limit_reject` | 8 m/s exceeds 6 m/s declared limit | Reject before launch |
| `obstacle_course` | Buildings, trees, walls and scored routes | Hover pass; route autonomy not built |
| `adverse_combined` | Wind, obstacles, noise, link faults and low battery | Integrated Phase 5 pass |

The strong-wind vehicle held altitude in the latest run but reached 1.40 m
drift and 20.41° tilt, beyond its 0.75 m/18° scenario envelope. The supervisor
correctly stopped the flight. Do not loosen the gate to hide this result; tune
wind-force/aerodynamic representation and controller parameters with evidence.

## What the Launcher Does

1. Loads and strictly validates the scenario JSON.
2. Rejects contradictory overrides and out-of-policy wind before arming.
3. Checks required ports.
4. Builds deterministic generated world/model artifacts.
5. Starts Gazebo, ArduPilot SITL and sensor/fault recorders.
6. Publishes the active local session for an independent control client.
7. Monitors child processes and sensor health until `Ctrl+C`.
8. Writes a structured run directory and shuts everything down.

## Manual Testing Status

The keyboard client and SDL Xbox mapping are implemented. Background polling
was verified with the attached Xbox 360 Controller while no pilot window
existed. Keyboard/controller coexistence, controller-disconnection neutral
inputs and automatic land-on-exit are part of the client design.

## Build and Score Without Flying

```bash
/usr/bin/python3 scripts/simulation/build_phase5_world.py obstacle_course

/usr/bin/python3 scripts/simulation/score_phase5_trajectory.py \
  obstacle_course trajectory.json --route open
```

Generated world/model files are disposable. Edit the JSON scenario,
environment preset, vehicle source model or builder—not a generated copy.

## Evidence

Run artifacts are stored under `logs/simulation/` and ignored by Git. A result
should identify the scenario, input hashes, process logs, health record,
trajectory/dynamics metrics and terminal state. Copy only concise, dated
acceptance conclusions into a dated `SIM-*` report; do not commit bulk recordings.

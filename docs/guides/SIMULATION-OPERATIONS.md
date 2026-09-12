# Icarus Simulation Operator Guide

## The Three Normal Operator Commands

Run each command from the repository root. They deliberately have separate
lifetimes: the aircraft runtime can remain active while a pilot or camera viewer
connects, disconnects, or restarts.

### 1. Start the aircraft runtime: `start-sim`

```bash
./scripts/start-sim --scenario wind_light --gui
```

Treat `start-sim` as the simulated onboard aircraft. It starts and supervises
Gazebo dynamics and the selected world, the vehicle, ArduPilot SITL, simulated
sensors, sensor health/fault services, and the onboard H.264 camera encoder. It
publishes connection details for ground-station clients. It does **not** arm,
take off, or fly the aircraft.

Important options:

| Option | Meaning |
| --- | --- |
| `--scenario NAME` | Selects the versioned world, wind, obstacles, sensor profile and limits. Example: `wind_light`. |
| `--gui` | Opens the Gazebo visual client. Without it, the same simulation runs headlessly. |
| `--video-destination IPV4` | Address of the ground-station computer that should receive video. This is the receiver's address, not the drone's address. Default: `127.0.0.1`. |
| `--video-port PORT` | UDP destination port for H.264/RTP video. Default: `5600`. The viewer must listen on the same port. |

For one laptop, omit the video options because their defaults are already
correct:

```bash
./scripts/start-sim --scenario wind_light --gui
```

The equivalent explicit form is:

```bash
./scripts/start-sim \
  --scenario wind_light \
  --video-destination 127.0.0.1 \
  --video-port 5600 \
  --gui
```

For a separate ground-station laptop at `192.168.1.50`, run on the simulated
aircraft computer:

```bash
./scripts/start-sim \
  --scenario wind_light \
  --video-destination 192.168.1.50 \
  --video-port 5600 \
  --gui
```

`GROUND_STATION_IPV4` in examples is a placeholder and must be replaced with
the receiving computer's actual IPv4 address. UDP 5600 must be permitted by its
firewall. Do not start control clients until `SIMULATOR READY` appears. Stop the
aircraft runtime with `Ctrl+C` in this terminal; it then stops only the processes
it owns and removes the active-session record.

### 2. Connect the manual transmitter: `manual-control`

```bash
./scripts/manual-control
```

This is a ground-station control client. It connects to the active ArduPilot
endpoint and converts Xbox inputs into bounded MAVLink RC overrides and flight
commands. With an Xbox controller attached it has no window, so Gazebo or the
camera viewer may remain focused. Its terminal displays aircraft state every two
seconds. It does not start or stop Gazebo, ArduPilot, or the camera stream.

Only one control client may be active: use `manual-control` **or** `run-mission`,
never both. Stop manual control with `Ctrl+C`; if the aircraft is armed, the
client requests LAND before releasing control. Use `--hud` for the keyboard/HUD
window and `--list-controllers` to enumerate detected controllers.

The current manual-control connector is intentionally restricted to the local
SITL session. Its input and command boundary is structured like a ground-station
transmitter, but a physical MAVLink network/serial profile has not yet passed a
real-hardware safety test.

### 3. Connect the camera viewer: `view-camera`

```bash
./scripts/view-camera
```

This is a passive ground-station client. It listens for the forward H.264/RTP
video on UDP 5600, opens the video window, and prints LIVE/STALE status, decoded
FPS and last-frame age. It sends no vehicle commands. It may start before or
after `start-sim`, and closing it never stops the aircraft or encoder. Nothing is
recorded unless a separate recording feature is explicitly added and enabled.

For a non-default receive port:

```bash
./scripts/view-camera --port 5601
```

The port must match `start-sim --video-port 5601`. The viewer does not need a
vehicle IP because RTP/UDP is pushed by the vehicle-side streamer to the ground
station's address.

### Automated alternatives

These are testing commands, not part of the normal three-process manual launch:

```bash
# Deterministic mission client connected to an already-running aircraft
./scripts/run-mission --mission takeoff_hover_land

# Coupled, one-shot acceptance launch
./scripts/sim --scenario empty_validation
./scripts/sim --scenario wind_strong --gui

# Validate the declared operating-limit rejection without starting Gazebo
./scripts/sim --scenario wind_limit_reject
```

The launcher handles termination, stops owned children and releases TCP 5760
and UDP 9002. Do not kill individual children first unless diagnosing cleanup.

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

## ArduPilot Stability and Current Fidelity Limits

The vehicle is not flying with an untuned, neutral ArduPilot configuration.
Every run wipes SITL state and loads the upstream Copter SITL defaults followed
by `simulation/parameters/mark4_v2_base.parm`. The project overlay explicitly
sets:

| Parameter group | Current values and effect |
| --- | --- |
| Roll/pitch rate PID | `ATC_RAT_RLL/PIT_P=0.10`, `I=0.10`, `D=0.003`; active body-rate stabilization. |
| Roll/pitch angle P | `ATC_ANG_RLL/PIT_P=3.5`; active self-level response in assisted modes. |
| Yaw rate PI | `ATC_RAT_YAW_P=0.15`, `I=0.015`. |
| Angle limit | `ATC_ANGLE_MAX=30` degrees. |
| Thrust model | `MOT_THST_EXPO=0.65`, `MOT_THST_HOVER=0.40`, hover learning enabled. |
| Output range | 1000–2000 microseconds, with the modeled operational motor ceiling at 70%. |

Position and velocity controller parameters not listed in the overlay retain
ArduPilot firmware defaults. `manual-control` requests LOITER when connecting
and before arming; assisted takeoff uses GUIDED and returns to LOITER. LOITER is
a closed-loop position and altitude controller, so it is expected to oppose
wind and pilot disturbances. STABILIZE still self-levels, while ACRO removes
self-leveling but retains the inner rate controller.

The current simulation nevertheless makes control substantially easier than a
physical prototype:

- the ArduPilot-facing IMU model has zero configured measurement noise;
- the upstream SITL baseline sets `SIM_BARO_RND=0`;
- `wind_light` is a spatially simple, constant 1.5 m/s flow with a smooth
  three-second rise and no gusts or direction changes;
- all motors and propellers have identical thrust response and fixed time
  constants, with no imbalance, damage, ESC variance or battery-dependent
  thrust loss;
- rotor drag and rolling-moment coefficients are currently zero;
- the airframe is perfectly rigid and omits structural flex, vibration,
  propwash, detailed ground effect and many aerodynamic cross-couplings;
- mass, inertia and centre of gravity are provisional calculated values rather
  than measurements from a built aircraft.

Therefore the current stability demonstrates that the control/simulation
connection works; it does **not** validate real-world handling fidelity or prove
that these gains are correct for the eventual aircraft. Realism should be
improved by measuring and calibrating propulsion, inertia, centre of gravity,
sensor noise/latency, motor mismatch, drag, gust spectra and battery sag, then
tuning ArduPilot against those data. Arbitrarily reducing gains just to make the
vehicle look less stable would produce a less defensible simulation.

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

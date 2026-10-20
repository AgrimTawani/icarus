# Gazebo Icarus7 Hardware-Fidelity Profile

The adaptive Phase 1 API now also exposes autopilot-reported range, flow and
barometer readings with individual timestamps through `DroneState.navigation_sensors`.
Planner context distinguishes above-home altitude from ground clearance and
does not claim current flow fusion. See the [adaptive mission handoff](../architecture/PHASE1-ADAPTIVE-MISSIONS.md)
for the exact boundary and manual tests; this telemetry addition was offline-tested.

**Status:** hardware profile tested on 2026-09-28; Gazebo-to-ArduPilot sensor
integration added on 2026-10-02. This is a
hardware-specific digital-twin profile, not a claim that Gazebo reproduces each
vendor device's firmware, optics, electronics, transport protocol or failure
physics.

This document is the source of truth for the hardware represented by
`akshu_compact_sitl`. It supersedes older descriptions of a generic OAK-D,
LW20, single IMU, single compass or single barometer. The physical source
layout is
`/home/agrim/Downloads/Icarus7_FlightControl_Review (2)/Icarus7_FlightControl_Review/FinalIntegrationLayout.json`.
Gazebo uses FLU (`+X` forward, `+Y` left, `+Z` up), so the source package's
right-positive Y coordinates are negated.

## Vehicle mass and sensor mounts

`config/simulation/vehicle_components.json` is the mass source. The generated
vehicle is **4.4745 kg**, with CG
`[0.005173, -0.001490, -0.051985] m` in the body frame. The builder derives
mass, CG and inertia rather than copying aggregate values into SDF. These are
selected-component and design-target inputs; the real assembled aircraft must
still be weighed and measured.

| Physical item | Gazebo origin, metres | Native Gazebo representation | Published data |
| --- | --- | --- | --- |
| Intel RealSense D435i, forward | `[0.1199, 0, -0.008]` | RGB-D camera, 848×480 at 30 Hz; RGB render 0.3–100 m, depth 0.3–10 m; separate 200 Hz IMU | `/icarus/sensors/d435i/image`, `/depth_image`, `/points`, `/icarus/sensors/d435i_imu` |
| Logitech BRIO 4K, downward | `[0.101, 0, -0.0799]` | RGB camera, 1280×720 at 30 Hz, 90° diagonal-FOV approximation | `/icarus/sensors/brio_down` |
| Landing-depth proxy, downward | BRIO origin | 320×180 RGB-D camera at 15 Hz | `/icarus/sensors/landing_depth_proxy/*` |
| Livox MID-360, forward/top | `[0.070, 0, 0.0795]` | 500×40 GPU-lidar lattice at 10 Hz; 360° × −7…52°; 0.1–40 m; separate 200 Hz IMU | `/icarus/sensors/mid360`, `/icarus/sensors/mid360_imu` |
| ARK Flow MR, downward | range `[0.055, 0, -0.1125]`; IMU Z `-0.105` | 8×4 ray grid at 20 Hz, 12.4°×6.2°, 0.08–30 m; 200 Hz IMU; derived image-flow estimate | `/icarus/sensors/ark_flow_range`, `/ark_flow_imu`, `/ark_flow/optical_flow` |
| Holybro H-RTK ZED-F9P/RM3100 | `[-0.091, -0.068, 0.044]` | NavSat at 8 Hz and independent magnetometer at 50 Hz | `/icarus/sensors/h_rtk_gnss`, `/h_rtk_compass` |
| Pixhawk 6X Pro + Mini Baseboard | `[0, 0, 0.03345]` | Three independent noisy IMUs at 500/500/200 Hz, BMM150-role magnetometer at 50 Hz and two independent air-pressure sensors at 30 Hz | `/icarus/sensors/pixhawk_imu_{primary,secondary,tertiary}`, `/pixhawk_magnetometer`, `/pixhawk_barometer`, `/pixhawk_barometer_secondary` |
| ArduPilot bridge IMU | `[0, 0, -0.026]`, rotated FLU→FRD | Separate 1000 Hz IMU consumed by the JSON SITL bridge | internal bridge only |

The D435i vendor specification is active stereo with approximately 0.3–3 m
ideal range, 1280×720 depth at 30 FPS, 87°×58° depth FOV and a built-in IMU;
the older D400 mechanical table used by the mass manifest specifies 90×25×25
mm and 72 g. The model deliberately uses 848×480 to keep two live feeds,
rendering, YOLO and the DCM within the development laptop's budget. Gazebo's
single RGB-D FOV is used for both color and depth, whereas the real D435i RGB
and stereo depth optics have different FOVs. RGB uses a 100 m rendering clip
so distant scene objects remain visible; the nested depth-camera clip remains
10 m. Keeping the shared camera clip at 10 m had incorrectly made distant
fixture buildings disappear from RGB while nearer people
remained visible. Sources:
[Intel product specification](https://www.intel.com/content/www/us/en/products/sku/190004/intel-realsense-depth-camera-d435i/specifications.html) and
[Intel D400 datasheet](https://cdrdv2-public.intel.com/841984/Intel-RealSense-D400-Series-Datasheet.pdf).

The BRIO supports 4K/30, 1080p/60 and 720p/90 with 65°, 78° or 90° diagonal
FOV. Gazebo uses 720p/30 and a plain RGB camera. It does **not** create physical
depth data, which is why landing depth has a separately named simulation-only
proxy. Source: [Logitech BRIO datasheet](https://www.logitech.com/content/dam/logitech/en_us/video-collaboration/pdf/brio-datasheet.pdf).

The MID-360 model matches its 200,000 points/s budget, 10 Hz frame rate,
360° horizontal and −7°…52° vertical envelope, 0.1 m blind zone, 40 m range at
10% reflectivity, 265 g mass and onboard-IMU topology. Gazebo produces a
uniform lattice; it does not reproduce Livox's non-repetitive scan pattern,
reflectivity response, Ethernet/PTP stack, fog or sunlight behavior. Source:
[Livox MID-360 specifications](https://www.livoxtech.com/mid-360/specs).

ARK Flow contains a PAW3902 optical-flow sensor, AFBR-S50 time-of-flight array
and IMU. Gazebo has no vendor-native PAW3902 plugin. `optical_flow_bridge.py`
therefore uses phase correlation over the downward BRIO image (320-pixel-wide
processing), pinhole focal-length scaling and body-FRD angular rates.
`flight_authority=false` means this publisher cannot command a flight; its
valid measurements now feed Gazebo SITL through the separate autopilot sensor
bridge below. Textureless/noise-only frames have zero quality. This is not a
physical ARK calibration or driver. The 32-pixel ToF array is approximated by
an 8×4 ray grid. Source:
[ARK Flow hardware specification](https://docs.arkelectron.com/products/sensor/ark-flow).

The H-RTK publishes at its maximum 8 Hz RTK rate and uses a 1 cm position-noise
approximation. Gazebo does not simulate a base station, RTCM corrections,
survey-in, constellation visibility, multipath, jam/spoof behavior or RTK
fix-state transitions; therefore an 8 Hz stream is not proof of an RTK fix.
The independent compass represents the RM3100 topology. The profile remains
SI-valued, but the SDF builder converts horizontal metres to degrees to work
around [Gazebo Sensors issue #325](https://github.com/gazebosim/gz-sensors/issues/325),
whose NavSat implementation otherwise treats the nominally metre-valued field
as degrees and creates kilometre-scale error. Source:
[Holybro H-RTK specification](https://docs.holybro.com/gps-and-rtk-system/zed-f9p-h-rtk-series/specification).

Pixhawk redundancy matches the 6X Pro topology: ADIS16470, IIM-42652 and
ICM-45686 IMUs; ICP20100 and BMP388 barometers; and BMM150 magnetometer.
Gazebo's IMU, air-pressure and magnetometer primitives provide independent
noisy channels, but not those chips' transfer functions, vibration isolation,
thermal compensation, bus timing or voting firmware. Source:
[Holybro Pixhawk 6X Pro technical specification](https://docs.holybro.com/autopilot/pixhawk-6x-pro/technical-specification).

## Camera stream and perception boundary

The onboard streamer accepts each sensor's native resolution and normalizes
both feeds to 640×480 H.264/RTP at **30 FPS** by default. Port 5600 is
`forward_d435i`; port 5601 is `downward_brio`. `view-camera` opens a normal
two-column 1280×540-class window, limited to the current display work area.
YOLO may inspect both RGB feeds. Qwen2-VL uses an ephemeral forward D435i
capture. The landing assessor reads the named depth proxy, never invents BRIO
depth.

No image pixels are retained by the routine sensor recorder. Its protobuf
streams keep only image metadata. Vision evidence records hashes, counts and
answers, not frames.

## ArduPilot sensor integration (Gazebo only)

`start-sim` automatically starts `scripts/simulation/autopilot_sensor_bridge.py`
using the ArduPilot venv. A dedicated MAVLink2 SERIAL2 link uses UDP loopback
14562; operator/Drone API TCP 5760 is unchanged. The bridge never arms, moves,
sets parameters, sends a GCS heartbeat, or accepts a physical-aircraft endpoint.
`simulation/parameters/gazebo_sensors.parm` is loaded **only by the Gazebo
launcher**, after the common aircraft defaults. AirSim is unchanged.

| Source | ArduPilot input/use | Important boundary |
| --- | --- | --- |
| MID-360 horizontal slice | `OBSTACLE_DISTANCE`, `PRX1_TYPE=2`; 72 clockwise body-FRD sectors | ±5° slice, 1m self-mask; unknown is not free space. Full companion 3-D map remains unchanged. |
| ARK downward ray grid | `DISTANCE_SENSOR`, ID 11, downward orientation 25, `RNGFND1_TYPE=10` | Actual range rays, not the artificial landing-depth camera. Median axial distance, 0.08–30m; too-close/no-return/invalid are distinct. |
| Image-derived flow | MAVLink2 `OPTICAL_FLOW.flow_rate_x/y`, `FLOW_TYPE=5` | This pinned backend accepts `OPTICAL_FLOW`, not `OPTICAL_FLOW_RAD`; it obtains rotation compensation from AHRS. Mount offset describes the BRIO source, not the ARK housing. |

GPS position/velocity and barometric altitude remain the primary references.
`EK3_SRC2_VELXY=5`, `EK3_SRC_OPTIONS=1` and `EK3_FLOW_USE=1` allow valid optical
flow to supplement velocity estimates. Range feeds terrain-height estimation
and the autopilot's applicable landing/surface logic; it does not replace the
barometric altitude reference or authorize landing on an arbitrary roof.
No GPS-denied flight capability is claimed by this integration.

`AVOID_ENABLE=3`, `AVOID_MARGIN=2`, `AVOID_BEHAVE=1` enable fence/proximity
stopping in supported control modes, with automatic reverse backup disabled.
**Simple avoidance is not a blanket shield for Guided position targets.** The
pinned firmware invokes it in Guided velocity control; Icarus position-based
missions still require the existing companion clearance guardrails. No hidden
BendyRuler rerouting or waypoint-controller switch is introduced here.

Startup reads back every sensor parameter and checks reflected autopilot
range/flow telemetry. `autopilot_sensors.json` records sensor ages, counts,
parameter mismatches and ArduPilot telemetry. `flow_fusion_verified` records
whether an explicit “fusing optical flow” status was observed **at least once
in that session**, not continuous or guaranteed fusion. Quality, tilt, range
and innovation checks remain with ArduPilot. Missing, duplicate, corrupt and
stale samples are never replayed as fresh readings. The normal sensor-fault
bus supports `lidar`, `range` and `optical_flow` drop/normal events.

Reproduce the bounded headless integration test (it actually flies):

```bash
# Terminal 1
./scripts/start-sim --profile edge-vision
# Terminal 2 — camera only; does not launch the Gazebo GUI
./scripts/view-camera
# Terminal 3 — do not run dcm-fly/manual-control simultaneously
third_party/ardupilot/.venv/bin/python scripts/simulation/test_autopilot_sensors.py \
  --fly --faults --proximity-stop
```

The test compares motion and range with independent Gazebo truth, exercises
sensor timeout/recovery and saves `sensor_acceptance_*/result.json` and bounded
telemetry in the current simulator run. Ground truth is test evidence only;
it is not sent into optical flow or the mission planner. Stop Terminal 1 with
Ctrl+C after the test has landed. For ordinary missions, use the usual
`start-autonomy` / `dcm-fly` commands instead of the acceptance controller.

## Network boundary

All local Gazebo launch, build, render and test entry points set
`GZ_IP=127.0.0.1`; the active session also records this value. Gazebo Transport
is therefore forced to loopback rather than selecting a Wi-Fi address. Camera
RTP and the JSON physics bridge also default to `127.0.0.1`. An operator can
explicitly send video to another ground station, but this is optional and does
not change simulation physics or sensor transport. Gazebo documents `GZ_IP` as
the local address used when a machine has multiple interfaces:
[Gazebo Transport environment variables](https://gazebosim.org/api/transport/15/envvars.html).

ArduPilot SITL's TCP server may appear as a wildcard listener in `ss`; Icarus
still connects to it only through the session's `tcp:127.0.0.1:5760` endpoint.
That listener is not the Gazebo sensor transport and does not make the
simulation dependent on an external network.

## Deliberate limitations

- The landing-depth proxy is simulated geometry evidence, not a BRIO feature.
- Image-derived flow now aids Gazebo SITL, but does not reproduce ARK firmware,
  its native camera or real-world calibration. Physical flight still requires
  real sensor calibration, frame/delay validation and dedicated fault tests.
- Public Gazebo redundant IMU/barometer/compass channels are recorded and
  fault-testable. SITL navigation still uses its dedicated bridge IMU and
  ArduPilot's simulated state; the public channels are not a claim of full
  hardware-in-the-loop sensor fusion.
- Camera autofocus, exposure, rolling shutter, stereo matching artifacts,
  USB bandwidth and illumination are not modeled.
- CAN, DroneCAN, Ethernet, USB, wiring faults, RF links, power-rail noise,
  compute load, Jetson thermal throttling and RP3 receiver behavior are not
  native sensor plugins and remain outside this profile.
- Mass, geometry and inertia are detailed enough for regression simulation,
  but require as-built measurement before digital-twin validation.

## Reproducible verification

Run with no other Icarus simulator active:

```bash
./.venv/bin/python scripts/simulation/build_akshu_compact.py
./.venv/bin/python scripts/simulation/build_compact_flight.py
./.venv/bin/python scripts/simulation/test_compact_model.py
./scripts/simulation/validate_sdf.sh
/usr/bin/python3 scripts/simulation/test_akshu_sensors.py
/usr/bin/python3 scripts/simulation/test_phase4_sensors.py --duration 10
```

For the integrated stack:

```bash
./scripts/start-sim --profile edge-vision
# separate terminals:
./scripts/view-camera
./scripts/start-autonomy --profile edge-vision
./scripts/dcm-fly --runtime llama --role primary --mode autonomous \
  --profile edge-vision --context-length 4096 --show-prompts
```

The launcher's `sensors/health.json`, `camera_stream.json` and
`optical_flow.json` and `autopilot_sensors.json` must all report `ready` at startup;
a non-stale active session must
record `"gz_ip": "127.0.0.1"` and `"nominal_fps": 30`.

The 2026-09-28 live sensor acceptance passed every recorded channel in runs
`compact_sensors_341bb023` and `phase4_physical_cc952a45`. A fresh integrated
session then passed a VLM-gated building mission and a 36-view dual-camera
YOLO scan; see episodes `20260928T213711_376851ae5458` and
`20260928T213942_dca58510b55b`. The scan's maximum simultaneous count was four,
matching the committed scenario truth. Routine episodes retain structured
evidence and hashes, not camera pixels.

### ArduPilot integration acceptance — 2026-10-02

- `scenario_edge_people_building_20261002T215117_376402/`
  `sensor_acceptance_20261002T215425`: takeoff, independent-truth XY checks,
  90° yaw, separate LiDAR/range/flow drop and recovery, velocity-command
  proximity braking, and landing all passed. Minimum braking clearance was
  **2.08m**. Hover range was **3.00m**, versus **3.009m** from Gazebo geometry.
- `scenario_wind_light_20261002T215820_4b5ee2/`
  `sensor_acceptance_20261002T215859`: the same movement/yaw/fault checks
  (without the scene-specific braking run) passed in 1.5m/s nominal wind with
  stochastic turbulence. ArduPilot explicitly reported optical-flow fusion.
  Range was **3.00m**, versus **3.006m** from geometry.
- On that same wind session, ordinary Drone API preflight
  `mission_20261002T220219_aa8171` and takeoff/hold/land
  `mission_20261002T220302_2579f0` passed. No mission-language changes were made.
- Python regression: **683 tests + 249 subtests passed**; rebuilt C++:
  **2/2 suites passed**. New bridge/flow conversion and freshness tests: 13.
- Launcher fault suite `launcher_faults_7f2e8e70`: **11 checks passed**, including
  crashed autopilot-sensor bridge, stale sensors, duplicate launch and complete
  owned-process/port cleanup. No aircraft was armed in those lifecycle tests.

These are bounded simulation acceptance cases, not hardware certification,
GPS-denied qualification, or proof that every wind/terrain condition is safe.


### Local detour tracking

The local planner verifies straight segments against measured obstacle returns.
Intermediate detour corners must therefore be reached closely and at settled
velocity before the next segment is commanded. Merely entering the normal
destination acceptance radius at transit speed can cut across the protected
corner. This does not change the streamed position/velocity orbit controller.

An experimental Guided waypoint/BendyRuler configuration was rejected after
a headless controller test still breached clearance. It is not enabled in the
simulator profile. Parameter readback alone did not prove motion performance.

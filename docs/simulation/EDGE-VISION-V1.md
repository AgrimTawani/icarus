# Edge Autonomous Vision V1

**Status:** implementation complete; simulation-only and experimental.  This
document records the Version 1 boundary, operator workflow, verification, and
known failures. It is not a real-aircraft readiness claim.

## What V1 provides

`edge-vision` is a dedicated Gazebo Harmonic + ArduPilot SITL profile for the
RTX 4050 Laptop GPU (6 GB VRAM) and 16 GB system RAM. It combines:

- the existing typed Drone API, guardrails, planner, safety supervisor,
  executor, MAVLink gateway, ArduPilot SITL, and headless Gazebo session;
- the existing Qwen3 4B Q4 GGUF through llama.cpp at a fixed 4096-token
  context for simulator-only DCM proposals;
- pinned YOLO11n detection and per-camera IoU tracking;
- on-demand SmolVLM-256M-Instruct qualitative inspection;
- a passive, local forward/downward RGB-D camera grid; and
- sealed episode evidence, deterministic replay, and an edge evidence report.

No visual component has flight authority. Models cannot access shell commands,
the filesystem, network, MAVLink, or the safety policy. Flight requests still
enter only through the existing typed Drone API path.

## Provisioning and artifact integrity

Run once, or again after an interrupted model/asset setup:

```bash
./scripts/setup-edge-vision-runtime
```

It keeps all non-Git artifacts beneath `~/models/edge/` and writes
`~/models/edge/MANIFEST.json`. The manifest pins the Qwen Q4 reuse, YOLO11n,
the complete SmolVLM snapshot, and the local Gazebo actor asset by SHA-256.
The actor is Fuel model `Mingfei/actor`, version 1, CC-BY-4.0; runtime worlds
reference the pinned local copy and never download a Fuel resource.

YOLO11n is an Ultralytics AGPL-3.0 dependency and is research-only pending a
commercial-license or replacement decision. SmolVLM is Apache-2.0. See
[`../reference/THIRD-PARTY.md`](../reference/THIRD-PARTY.md).

## Deterministic scenario and coordinate contract

`simulation/scenarios/edge_people_building.json` creates a fixed-seed world
with `north_building`, four human actor targets, obstacles, a safe route, and
an expected unique-person count of four.

Gazebo uses ENU: `+X` is east and `+Y` is north. The Drone API uses local NED.
Accordingly, the building is placed in Gazebo at `(east=0, north=18, up=3)` and
the model receives the committed local-NED landmark `(north=18, east=0,
down=-3)`. This mapping is intentionally scenario data, not a YOLO inference.
It fixes the former axis mismatch where the visual building was east while the
model was instructed to fly north.

Stock YOLO11n has no dependable `building`, `roof`, or safe-landing class. The
DCM must use the known landmark to navigate; it must not pretend to visually
identify arbitrary buildings. Landing safety remains the deterministic
downward-depth/LiDAR/geometry path.

## Cameras and vision scheduling

The headless simulator publishes both mounted RGB-D image feeds:

| Feed | Gazebo topic | RTP port | V1 role |
| --- | --- | --- | --- |
| Forward RGB-D | `/icarus/sensors/rgbd/image` | base port (default `5600`) | operator grid, supplemental YOLO coverage |
| Downward RGB-D | `/icarus/sensors/rgbd_down/image` | base port + 1 (default `5601`) | operator grid, YOLO person-count source, landing geometry |

`./scripts/view-camera` composes both RTP/H.264 feeds into a labeled passive
grid. It sends no vehicle command and retains no recording.

The continuous observer round-robins both feeds at a **combined** maximum of
5 FPS (2.5 FPS per feed with two cameras). Each feed has its own tracker. The
downward feed is the sole unique-count source: V1 deliberately does not claim
cross-camera identity re-identification, so it does not add counts from both
views. Observer evidence includes camera identity and whether a frame is
count-eligible; pixels are never written into episode logs.

SmolVLM accepts one bounded qualitative question, one image at a time, at most
once every five seconds. It loads, infers, and releases CUDA memory by default.
It has no routing, landing, or other flight authority.

## Operator workflow

Use four terminals after `SIMULATOR READY` is printed:

```bash
# Terminal 1
./scripts/start-sim --profile edge-vision

# Terminal 2
./scripts/view-camera

# Terminal 3
./scripts/start-autonomy --profile edge-vision

# Terminal 4
./scripts/dcm-fly --runtime llama --role primary --mode autonomous \
  --profile edge-vision --context-length 4096 \
  --mission "Arm, take off to 5 metres, fly to the known north building, orbit it at 7 metres, count unique people visible below for 20 seconds, return home, and land." \
  --json
```

After `dcm-fly` prints an episode directory, pass that exact path to:

```bash
./scripts/report-edge-vision logs/episodes/PRINTED_EPISODE_DIRECTORY
./scripts/test-phase10 logs/episodes/PRINTED_EPISODE_DIRECTORY
```

Do not request “count the buildings you see.” That is outside YOLO11n's
supported semantics. The `north_building` ID is an explicit scenario landmark.

## Mission scoring and retained evidence

The edge report distinguishes flight execution from semantic mission success.
A safe landing alone is not success when a requested person count has no
successful `detect(["person"])` evidence or does not match the scenario's
expected count. The DCM prompt reminds the model to request that evidence
before returning home; it never prevents a safety-driven return or landing.

The earlier retained episode `20260923T005945_3b38847cc6de` demonstrates why
this distinction matters. Its flight actions completed and Phase 10 replay
passed, but it had no `detect` action, observed zero people against four
targets, and also requested an unsupported visual building count. It is a
sealed integrity/safety artifact, **not** edge-mission acceptance evidence.

## Verification record

The following commands were run during V1 closeout:

```bash
# Unit and generated-scenario checks
PYTHONPATH="$PWD" ./.venv/bin/python -m pytest -q tests/unit
# Result: 225 passed, 161 subtests passed

# Edge-focused checks, including model-result normalization, tracking, rate
# limits, checksum validation, report semantics, and headless world generation
PYTHONPATH="$PWD" ./.venv/bin/python -m pytest -q \
  tests/unit/test_edge_vision.py tests/unit/test_edge_report.py \
  tests/unit/test_edge_scenario_integration.py tests/unit/test_scenario_config.py
# Result: 13 passed

# Generated scenario inspection
PYTHONPATH="$PWD" ./.venv/bin/python scripts/simulation/build_phase5_world.py edge_people_building

# Parse dual H.264 encoder and composed camera-grid pipelines with GStreamer
/usr/bin/python3 -m py_compile scripts/simulation/camera_stream.py \
  scripts/simulation/view_camera.py scripts/simulation/launch_compact.py

# Episode integrity/replay of the retained historical episode
./scripts/test-phase10 logs/episodes/20260923T005945_3b38847cc6de
# Result: passed; this verifies integrity/replay only, not semantic success.
```

A clean headless `edge-vision` session was also observed at V1 closeout. Its
active-session record named both RTP streams, and its `camera_stream.json`
reported `ready`, zero invalid/push-failure frames, and approximately 14.2 FPS
for each 640×480 feed. Gazebo, ArduPilot SITL, sensor recorder, and the
dual-camera bridge were all supervised by the launcher. This is camera-stream
readiness evidence; it is not a post-fix autonomous person-count result.

A preflight on the actual GPU found an RTX 4050 Laptop GPU with 6,141 MiB
VRAM and 5,008 MiB free. The prior combined Qwen Q4 + YOLO profile recorded
4,413 MiB peak VRAM and 1,667.1 MiB peak RAM in the retained episode. Those
numbers are measured historical evidence, not a guarantee for every driver or
desktop workload. No post-coordinate-fix end-to-end autonomous count result is
claimed until the operator reruns the acceptance workflow above.

## V1 limits and next evidence

V1 is intentionally simulation-only. It does not authorize unattended flight,
real-hardware control, visual building selection, visual rooftop selection,
or VLM-derived landing decisions. Before any future promotion, rerun the
acceptance mission after each material model, prompt, driver, or scenario
change; retain the report, Phase 10 result, resource evidence, and any safety
intervention rather than replacing failures with a success claim.

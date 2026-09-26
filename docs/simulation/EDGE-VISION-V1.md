# Edge Autonomous Vision V1

**Status:** Phase-1 hardening complete; simulation-only and experimental. This
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
- on-demand Qwen2-VL-2B-Instruct Q4 qualitative inspection, forced to CPU on
  the 6 GB development host;
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
`~/models/edge/MANIFEST.json`. The manifest pins the Qwen flight-model Q4,
YOLO11n, Qwen2-VL model/projector and local Gazebo actor asset by SHA-256.
The actor is Fuel model `Mingfei/actor`, version 1, CC-BY-4.0; runtime worlds
reference the pinned local copy and never download a Fuel resource.

YOLO11n is an Ultralytics AGPL-3.0 dependency and is research-only pending a
commercial-license or replacement decision. Qwen2-VL is Apache-2.0. See
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
| Forward RGB-D | `/icarus/sensors/rgbd/image` | base port (default `5600`) | operator grid, VLM inspection and primary YOLO count view |
| Downward RGB-D | `/icarus/sensors/rgbd_down/image` | base port + 1 (default `5601`) | operator grid, supplemental YOLO coverage and landing geometry |

`./scripts/view-camera` composes both RTP/H.264 feeds into a labeled passive
grid. It sends no vehicle command and retains no recording.

The continuous observer round-robins both feeds at a **combined** maximum of
5 FPS (2.5 FPS per feed with two cameras). Each feed has its own tracker. The
forward camera contributes detection coverage but never claims a cross-frame
unique count because vehicle motion defeats simple image-IoU identity. A DCM
person-count action collects the declared 20-second observer window and uses
the largest same-frame person observation from one camera. It never sums feeds
or claims cross-camera re-identification. Observer evidence includes camera
identity; pixels are never written into episode logs.

Qwen2-VL accepts one bounded qualitative question, one image at a time, at most
once every five seconds. It loads, infers and releases the process per request.
The edge profile explicitly disables CUDA and projector offload so it cannot
evict or starve the flight DCM on this host. It has no routing, landing, or
other flight authority.

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
  --max-decisions 24 \
  --mission "Arm, take off to 5 metres, fly north to the building, visually inspect it, orbit it, count all visible people, hover for 5 seconds, then return home and land." \
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

The final acceptance episode `20260927T041123_0a62e3f27053` completed all nine
actions. Qwen2-VL described the building, the reviewed orbit completed from the
nearest safe entry point, and YOLO observed the scenario's four people from the
forward feed across 44 frames. The requested hold and RTL completed, the edge
report marked flight and semantic success, and strict complete replay covered
3,367 records. Alternative prompt wording, building-hover-only and gusting-wind
missions also passed in episodes `20260927T042122_6ecfa541aa24`,
`20260927T042555_f1235131da86` and `20260927T043110_1dad8ed23b68`.

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

# Post-run regression checks for task ordering, count-window aggregation, and
# non-overlapping scenario targets
PYTHONPATH="$PWD" ./.venv/bin/python -m pytest -q \
  tests/unit/test_edge_vision.py tests/unit/test_edge_report.py \
  tests/unit/test_edge_scenario_integration.py tests/unit/test_dcm_contract.py \
  tests/unit/test_dcm_fly.py
# Result: 84 passed, 137 subtests passed

# Generated scenario inspection
PYTHONPATH="$PWD" ./.venv/bin/python scripts/simulation/build_phase5_world.py edge_people_building

# Parse dual H.264 encoder and composed camera-grid pipelines with GStreamer
/usr/bin/python3 -m py_compile scripts/simulation/camera_stream.py \
  scripts/simulation/view_camera.py scripts/simulation/launch_compact.py

# Episode integrity/replay of the retained historical episode
./scripts/test-phase10 logs/episodes/20260923T005945_3b38847cc6de
# Result: passed; this verifies integrity/replay only, not semantic success.

# Final Phase-1 hardening regression (2026-09-27)
PYTHONPATH="$PWD" ./.venv/bin/python -m pytest -q tests/unit
# Result: 294 passed, 161 subtests passed
ctest --test-dir build/phase8 --output-on-failure
# Result: 2/2 C++ suites passed
./scripts/test-phase10 logs/episodes/20260927T041123_0a62e3f27053
# Result: complete deterministic replay passed; 3,367 records
```

A clean headless `edge-vision` session was also observed at V1 closeout. Its
active-session record named both RTP streams, and its `camera_stream.json`
reported `ready`, zero invalid/push-failure frames, and approximately 14.2 FPS
for each 640×480 feed. Gazebo, ArduPilot SITL, sensor recorder, and the
dual-camera bridge were all supervised by the launcher. This is camera-stream
readiness evidence. The retained 2026-09-27 acceptance episode above supplies
the separate post-fix autonomous person-count result.

A preflight on the actual GPU found an RTX 4050 Laptop GPU with 6,141 MiB
VRAM and 5,008 MiB free. The prior combined Qwen Q4 + YOLO profile recorded
4,413 MiB peak VRAM and 1,667.1 MiB peak RAM in the retained episode. Those
numbers are measured historical evidence, not a guarantee for every driver or
desktop workload. VLM CPU-only scheduling now avoids the earlier combined
CUDA out-of-memory failure at the cost of longer qualitative-inspection latency.

## V1 limits and next evidence

V1 is intentionally simulation-only. It does not authorize unattended flight,
real-hardware control, visual building selection, visual rooftop selection,
or VLM-derived landing decisions. Before any future promotion, rerun the
acceptance mission after each material model, prompt, driver, or scenario
change; retain the report, Phase 10 result, resource evidence, and any safety
intervention rather than replacing failures with a success claim.

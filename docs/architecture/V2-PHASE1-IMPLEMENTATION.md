# V2 Phase 1 implementation and evidence

Date: 2026-09-23

Branch: `feat/version-v2-software-architecture`

Decision: implementation complete; retain the current DCM default.

## Delivered architecture

- `dcm-contract-v3-feedback` builds a deterministic, byte-bounded context from
  the existing curated allow-list. It adds source freshness, sectorized
  obstacles, bounded history/failure counters, profile/macro provenance and a
  context hash without exposing raw sensors, truth or authority tokens.
- BODY is generated from the active action/freshness contract and checked in
  as a manifest. A stale manifest fails closed. SOUL and world facts are static
  reviewed inputs and cannot be changed by a model or active mission.
- Four YAML-front-matter mission macros have explicit action families,
  preconditions, evidence links and nominal/adverse/recovery fixtures.
- V3 output has action, arguments and a separate bounded rationale. Only the
  validated action/arguments cross the control boundary.
- Decision traces validate against a versioned schema. The feedback builder is
  deterministic; model hypotheses enter an explicitly untrusted human-review
  queue and cannot mutate policy.
- Runtime health has healthy/late/invalid/unavailable/stale/circuit-open states.
  Three consecutive failures open the circuit; typed senior fallback requests
  hold and then RTL through the unchanged Drone API.

## Frozen Gazebo comparison

The same 11 sealed Gazebo episodes, Qwen3-4B Q4 artifact, history input,
5-second deadline and three repeats produced 135 decision points per contract.
No proposal was executed in this offline comparison.

| Metric | Current V2-history | V3 candidate |
| --- | ---: | ---: |
| Valid / invalid / timeout / stale | 128 / 0 / 1 / 6 | 127 / 0 / 2 / 6 |
| Tool-selection agreement | 71.9% | 71.7% |
| Guardrail rejection | 0.0% | 0.0% |
| Recovery proxy | 57.1% | 62.5% |
| Steady median latency | 676.1 ms | 1,215.9 ms |
| Peak VRAM | 4,069 MiB | 4,052 MiB |

The candidate improved recovery but doubled timeouts and exceeded the frozen
1.25x latency ratio. Three paired closed-loop Gazebo candidate/baseline runs
are also not yet present. `evaluate-dcm-v2-promotion` therefore correctly
returns `retain-v1`. V3 is available for further experiments via
`--context v3`; it is not the runtime default.

Retained local report hashes:

- V2 evaluation: `c22d2970d42b82088dda9e542de3707f827cb5de48f7ed558c54027c52abab37`
- V3 evaluation: `2004b1d03bd68dc3af748d796bba652e7570b0f0487154ddb847608754ba60ba`
- promotion decision: `4308545b20e420c7ce44654c6501b347eeadc2fe69e260ad41c319eb4a0e8fc9`

## AirSim end-to-end proof

The pinned Blocks v1.8.0 binary now receives reviewed settings from an isolated
runtime HOME, runs `VehicleType: ArduCopter`, and exchanges lock-step dynamics
with pinned ArduPilot SITL. RGB, DepthPlanar and LiDAR are timestamp-checked.
LiDAR enters the common perception engine through a separate bounded UDP
contract and C++ body-FRD adapter. No direct movement or pose RPC exists in
Icarus's client.

A Drone API mission passed through the real path—Arm, Takeoff 2 m, Hold 3 s,
Land, disarm—in 17.96 seconds. Killing the sensor bridge made the combined
health response degraded; the next preflight failed with
`PERCEPTION_UNAVAILABLE`. Report hashes:

- flight: `4c41ec292743c54910a1ac165c87595fd6b7eff8febcedbde4898640d33c09f9`
- stale-perception refusal: `f0ad58e4358382a998362fd1bc0b4193938e3833bb8768089eaf9ad7eada165e`

AirSim remains a supplementary track. Blocks does not supply Icarus's frozen
Gazebo scenarios, seeded atmosphere or equivalent scene determinism, so this
proof does not promote it over Gazebo.

### Realistic city and semantic mission extension

The optional, digest-pinned AerialVLN Simulators v2 archive now supplies 25
external city-scale Unreal environments under CC BY-NC 4.0. Scene 6 is the
tested default because it provides a bright textured residential area and a
safe nearby low-rise target. `start-airsim-realistic` preserves the same
ArduPilot route and attaches a reviewed landmark reference to the session.

Live AirSim RGB frames now feed the existing edge-vision boundaries. SmolVLM
describes the primary building; YOLO supplies the visible-person count. Neither
has flight authority. A deterministic mission-guidance feedback field exposes
pending evidence obligations, and proposals that contradict it are recorded
and refused before reaching the Drone API. This prevents a model from claiming
“hover over that building” at an invented coordinate or landing before required
visual evidence is collected.

The final prompt-driven scene-6 episode
`20260923T212718_6bf94baf9b39` executed Arm, Takeoff 3 m, InspectScene, Goto the
reviewed `(25, -6, 22)` NED/AGL waypoint, Hold 3 s, Detect person, and
ReturnHome/land. One premature goto was refused, the model recovered on its
next decision, every executed action succeeded, and semantic completion passed.

## Reproduction

```bash
./scripts/generate-dcm-body --check
.venv/bin/python -m pytest -q tests/unit
ctest --test-dir build/phase8 --output-on-failure

./scripts/fetch-airsim-blocks
./scripts/start-airsim-stack --binary \
  third_party/airsim/blocks-v1.8.0-linux/Blocks/LinuxNoEditor/Blocks.sh
# second terminal
./scripts/start-autonomy-airsim
./scripts/run-mission --mission takeoff_hover_land --altitude 2 --hover-seconds 3

# One-time optional realistic city installation, then tested scene 6.
./scripts/fetch-airvln-environments
./scripts/start-airsim-realistic
# second terminal
./scripts/start-autonomy-airsim
# third terminal
./scripts/view-airsim-camera
# fourth terminal
./scripts/dcm-fly --runtime llama --profile edge-vision --mode autonomous \
  --context-length 4096 --max-decisions 18 \
  --mission "Arm, take off to 3 metres, check that building, hover over it for 3 seconds, count no. of people, then return home and land."
```

The evaluation and promotion commands are documented in
[`DCM-V2-FEEDBACK.md`](DCM-V2-FEEDBACK.md).

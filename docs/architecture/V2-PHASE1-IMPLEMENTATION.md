# V2 Phase 1 implementation and evidence

Date: 2026-09-23

Branch: `feat/version-v2-software-architecture`

Decision: implementation complete. As of 2026-09-24, V3 is the interactive,
observe and evaluation CLI default by explicit operator decision; V2 remains
available through `--context v2` for compatibility and baseline comparisons.

## Delivered architecture

![Icarus V2 Phase 1 prompt-to-motors architecture](../assets/icarus-v2-dcm-to-motors-architecture.png)

The diagram follows the complete prompt path: the exact operator mission is
combined with one deterministically compiled `MissionContract`, three compact
SOUL principles, semantic live BODY facts and one canonical evidence-health
summary; the resulting system/user messages reach the local DCM before its
single typed proposal crosses schema, macro,
freshness, approval and C++ authority boundaries. It also shows the separate
semantic-vision loop, independent safety override, offline feedback lane and
the distinct Gazebo, AirSim and future physical device branches. V3 is the CLI
default. The current AirSim multirotor is still not physically equivalent to
the Gazebo Akshu airframe.

- `dcm-contract-v3-compact` builds a deterministic, byte-bounded context from
  the existing curated allow-list. It adds source freshness, sectorized
  obstacles, bounded history/failure counters and a context hash without
  exposing raw sensors, truth or authority tokens.
- Model-visible context is deliberately smaller than the audit record. BODY
  exposes only semantic deployment facts; action lists, freshness limits,
  profile paths, revisions and hashes stay in sealed evidence. SOUL renders as
  three reviewed principles. Empty world-facts text is not sent to the model.
- Five YAML-front-matter mission macros have explicit action families,
  preconditions, evidence links and nominal/adverse/recovery fixtures.
- Explicit `--macro-id` remains an override; otherwise distinctive mission
  intent deterministically selects a macro. Its action subset is enforced.
- Reviewed macro policy, mission references and completion evidence compile to
  one `icarus.dcm.mission-contract.v1` object. Macro prose and provenance stay
  audit-only, while the action subset remains enforced in code.
- V3 output has action, arguments and one closed `reason_code`; free-form
  rationale is no longer generated. Only validated action/arguments cross the
  control boundary.
- Decision traces are schema-validated before live recording. Episode sealing
  automatically creates hash-bound reliability and human-review artifacts;
  model hypotheses remain untrusted and cannot mutate policy.
- Runtime health has healthy/late/invalid/unavailable/stale/circuit-open states.
  Three consecutive failures open the circuit; typed senior fallback requests
  hold and then RTL through the unchanged Drone API. Normal inference is
  suppressed while open, with a bounded recovery probe.
- The final model-visible context is budgeted and hashed only after reference,
  obligations and live BODY are attached. Post-inference execution additionally
  requires that the aircraft has not materially moved or changed state.
- Control-lease renewal is the independent DCM-process heartbeat. If it stops
  while airborne, the C++ safety supervisor commands BRAKE and then RTL after
  the configured delay, even if the Python process is gone.

## Compact prompt revision (2026-09-24)

The first Phase-1 prompt represented mission meaning as raw text, obligations,
reference data, macro prose and special-case system rules simultaneously. It
also repeated action vocabulary and freshness/provenance fields. The compact
revision preserves all deterministic safety layers but removes that semantic
duplication from model input.

For the same representative AirSim building mission and live BODY fixture, the
rendered prompt changed from 8,482 to 5,026 characters. The existing
conservative byte-to-token upper estimate changed from approximately 2,828 to
1,676 tokens (41% lower). Actual tokenizer counts depend on the pinned model.
The model-visible payload itself is 2,427 bytes in that fixture, below the new
8,000-byte fail-closed ceiling. Together with the approximately 3,042-character
fixed system message, the conservative upper estimate fits the default 4,096
token runtime while leaving bounded response headroom. This measurement is a prompt-size regression
check, not an autonomy-quality claim.

The compact prompt has not inherited the earlier V3 promotion evidence. It
must be replayed and flown through the same frozen scorecard before any claim
that it improves mission performance, latency or unattended-flight readiness.

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
returns `retain-v1`. That result still blocks unattended promotion, but it no
longer controls the interactive context default: V3 is selected unless the
operator explicitly requests `--context v2`.

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
has flight authority. Deterministic completion evidence is compiled into the
mission contract without choosing the next action. Reviewed macro constraints and ordinary
argument/guardrail validation refuse actions outside the authorized envelope.

The final prompt-driven scene-6 episode
`20260923T212718_6bf94baf9b39` executed Arm, Takeoff 3 m, InspectScene, Goto the
reviewed `(25, -6, 22)` NED/AGL waypoint, Hold 3 s, Detect person, and
ReturnHome/land. That historical run proved the complete visual/control path,
but its zero-person result is now correctly classified as **inconclusive**:
the scene reference has no reviewed visible-person ground truth.

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

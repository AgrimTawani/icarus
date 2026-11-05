# Results: decision speed on the v2 mission agent (2026-11-06)

Branch `feat/laya-decision-speed`. Goal and baseline: [LAYA-DECISION-SPEED-GOAL.md](LAYA-DECISION-SPEED-GOAL.md).
Research notes: [LAYA-FINDINGS.md](LAYA-FINDINGS.md) (Laya), [TOOL-CALLING-FRAMEWORKS.md](TOOL-CALLING-FRAMEWORKS.md)
(DSPy, ZeroClaw, techniques). Every number below comes from an episode under `logs/episodes/` or a replay of one;
`scripts/autonomy/episode_latency.py <episode>` reprints them.

## What was slow, and what changed

The loop was not slow because the 4B model thinks slowly. It was slow because it re-read its whole prompt.

| Cause (measured) | Fix | Effect |
|---|---|---|
| The tool catalogue (~6.8k of ~8-9k prompt tokens, rendered first by the chat template) changed when the vehicle took off (climb-rate field removed) and whenever a target was measured (target_id enum). Replaying a recorded orbit mission: 5 of 7 decisions had `cache_n=0`, ~10 s each. | Catalogue is static per session; unknown target ids are rejected by `validate_call` from the live targets; airborne climb rate is still refused by the executor. (`3874a33`) | Same recorded requests: median decision 9.9 s -> 2.1 s; every step a cache hit. |
| Every camera question stopped llama-server so Qwen2.5-VL-3B could have the GPU; the next decision reloaded weights and re-prefilled ~9k tokens. | Qwen3.5-4B is multimodal: its projector is declared in `config/dcm/agent-runtime.json` and the same resident server answers inspect/ground questions. llama-server's RAM prompt cache restores the mission conversation after each image. (`ca99803`) | Decision after a camera question: ~1.7 s, cache hit, no reload. |
| The startup warm-up sent no tools, so it cached nothing the first decision could use. | Warm-up sends the session's exact catalogue. (`ca99803`) | First decision ~8 s -> ~1-3 s (8,221 tokens cached). |
| Beside Unreal (AirSim), reserving GPU memory for the projector made `--fit` leave 5 of 34 layers on the CPU: decisions at 15-20 tok/s instead of 42. | Runtime reads llama.cpp's own "offloaded N/M layers" report and, only when N < M, restarts with the projector on the CPU. In Gazebo both fit on the GPU. (`9a3115e`) | AirSim decisions back to 41-43 tok/s. |
| With the projector on the CPU, a survey (dozens of frames) kept 8 threads busy and the sim's sensor bridges went stale: 2 of 2 AirSim house surveys aborted (route measurements stale / perception stale). | A CPU-projector llama-server runs at nice 10, so the flight stack wins CPU contention. | Next 2 of 2 surveys: route completed, 10/10 waypoints, 8 houses each. |

## Camera questions: resident Qwen3.5-4B vs the one-shot Qwen2.5-VL-3B

24 hand-labelled frames (AirSim + Gazebo), same prompts and grammar (`docs/research/vision-bench/`):

| | building | person | car | median latency |
|---|---|---|---|---|
| Qwen2.5-VL-3B, llama-cli per question (before) | 23/24 | 22/24 | 21/24 (3 false) | 3.9 s |
| Qwen3.5-4B resident, projector on GPU | 22/24 (1 false, "possible structure") | 23/24 | 23/24 (1 false) | 0.8-1.4 s |
| Qwen3.5-4B resident, projector on CPU (AirSim) | not re-scored | | | 2.2-2.5 s |

Qwen3.5 boxes are 0-1000 normalized whatever the prompt says, so the resident artifact declares `json_norm1000_xyxy`.

## Live headless missions (unchanged stock `dcm-fly`, autonomous mode)

Gazebo `edge-vision`, projector on GPU. No same-wording baseline exists for most; the 2026-10-23/24 median decision was 14.9 s.

| Mission (operator words) | Outcome | Decisions | Model time | Median decision |
|---|---|---|---|---|
| Go up ~5 m, look around until you find a building, fly around it once with the camera on it, come back and land | completed; orbit 360.3 deg, radial error 0.13 m | 6 | 21.5 s | 3.4 s |
| Codex's exact rotate / detect / approach 5 m / orbit facing mission (failed and looped on 2026-10-20/23) | completed; approach + orbit 360.4 deg, radial error 0.12 m | 6 | 21.1 s | 3.8 s |
| Climb to 4 m, see if there are people; if so say how many, else come down | completed; 4 people (detector) | 6 | 19.7 s | 3.0 s |
| 3 m, look north; building? colour? grey -> 2x1 m rectangle, else wait; no building -> wait 1 s; home | completed; grey branch, route flown | 7 | 27.3 s | 3.8 s |
| Go up 3 m, face east, wait 5 s, land | completed | 5 | 12.8 s | 2.0 s |
| Hop up and take a look around, tell me what you see, then land | completed but shallow: one forward view, no turn | 4 | 15.6 s | 2.7 s |

AirSim city scene 6, projector on CPU. Same words as the 2026-10-23/24 runs:

| Mission | Outcome | Decisions | Model time (before -> now) |
|---|---|---|---|
| Take off to 10 m, find the nearest house, orbit it at 6 m facing it, return and land | completed; orbit 360.9 deg, radial error 0.20 m | 8 | 81.3 s -> 15.7 s |
| Search the area around here and tell me how many cars you can find, then land | completed, but chose a 360 deg turn at 3 m (0 cars in view) instead of the area survey that found 3 cars on 2026-10-24 | 4 | 44.9 s -> 6.7 s |
| Go up and do a surveillance loop around the closest house, camera on it, come back and land | completed; first orbit safety-aborted, climbed, orbit 360.6 deg | 9 | 143.0 s -> 18.1 s |
| Fly up to 20 m and count the houses in the 60 x 60 m area in front of you, land back home | completed 2/2 after the nice-10 fix (0/2 before): route 10/10 waypoints, 8 houses (7 on 2026-10-24) | 4 | 53.3 s -> 16.9 / 16.2 s |
| Take off to 12 m, circular perimeter of 15 m radius around the launch point at 2 m/s, land where you started | completed; 360.3 deg, radial error 0.30 m | 4 | 23.5 s -> 7.4 s |
| Codex's exact rotate / detect / approach / orbit mission | completed after a safety climb | 9 | n/a -> 17.7 s |

Same-wording AirSim missions: **346 s -> 65 s of model time (-81 %)**, median decision ~2 s.

## Still wrong (not speed; left for decision-quality work, no prompt patches were added)

- **Success over-claimed.** Twice the survey route FAILED and the model still called `finish(outcome=completed)`
  with the partial count. `summarize()` prints the failed call to the operator, but the outcome is the model's word.
- **Invented recovery motion.** After `VIEWPOINT_REQUIRED` on an orbit the model flew `move_relative forward 10 m`,
  which the operator never asked for. The result text tells it to use motion "the user's request calls for".
- **Interpretation variance at temperature 0.** "Search the area around here ... cars" picked a 360 deg turn instead
  of `survey_count`; "take a look around" inspected one view. Both are word-sense ambiguities, not hardcoding targets.
- **Hedged VLM evidence counts as found.** "possible structure in background" became `found=true` on one frame; the
  LiDAR localization gate is what stops an approach on it.

## Laya, DSPy, ZeroClaw: verdicts

- **Laya**: not in the loop. Its value proposition (one forward pass instead of generation) no longer has a slow step
  to replace: decisions are ~2 s of mostly argument generation and camera questions ~1-2.5 s. Measured here: the
  vegaml 0.8B image reader answered "yes" to building / person / car on almost every frame zero-shot and its
  abstain/route flags caught none of the errors; it needs the CPU (~7 GB RAM, 4.4 s per image) because the image path
  does not fit beside the mission model in 6 GB; the ModernBERT text checkpoints were 1/8 on tool choice and 3/6 on
  branch evaluation zero-shot. The video path tiles 4 frames into one image, no temporal model. The one open lead is
  a fitted (`fit()`, >= 35 labelled rows) 2-way image `choice` question as a calibrated second opinion on visual
  conditions; it read 7/8 frames correctly in a small CPU test.
- **DSPy**: no as a runtime (no latency mechanism, would duplicate the loop's deadline/interrupt/approval handling).
  Possibly later as an offline prompt/description optimizer against a step-replay dataset from episode logs.
- **ZeroClaw** (zeroclaw-labs/zeroclaw, Rust personal-assistant runtime): no; a plain ReAct loop with nothing for
  cache reuse, hybrid models or latency.

## Not done

- Speculative decoding with the model's MTP head: this llama.cpp build loads the GGUF but ignores the `nextn` tensors.
- No repeat runs per mission except the house survey (4 flights); every other row is one flight.

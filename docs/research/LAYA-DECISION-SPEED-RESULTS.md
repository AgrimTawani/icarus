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
| With the projector on the CPU, a survey (dozens of frames) kept 8 threads busy and the sim's sensor bridges went stale: 2 of 2 AirSim house surveys aborted (route measurements stale / perception stale). | A CPU-projector llama-server runs at nice 10, so the flight stack wins CPU contention, and encodes at most 256 image tokens (3.2 s instead of 5.1 s per Gazebo frame, same 10/10 answers, boxes within ~15 px). | Next 2 of 2 surveys: route completed, 10/10 waypoints, 8 houses each. |

## Camera questions: resident Qwen3.5-4B vs the one-shot Qwen2.5-VL-3B

24 hand-labelled frames (AirSim + Gazebo), same prompts and grammar (`docs/research/vision-bench/`):

| | building | person | car | median latency |
|---|---|---|---|---|
| Qwen2.5-VL-3B, llama-cli per question (before) | 23/24 | 22/24 | 21/24 (3 false) | 3.9 s |
| Qwen3.5-4B resident, projector on GPU | 22/24 (1 false, "possible structure") | 23/24 | 23/24 (1 false) | 0.8-1.4 s |
| Qwen3.5-4B resident, projector on CPU (AirSim 640x360 / Gazebo 848x480, 256-token cap) | 10/10 clear Gazebo frames | | | 2.2-2.5 s / 3.2 s |

Qwen3.5 boxes are 0-1000 normalized whatever the prompt says, so the resident artifact declares `json_norm1000_xyxy`.

## Live headless missions (unchanged stock `dcm-fly`, autonomous mode)

Gazebo `edge-vision`. These six ran before the placement fix, with the projector on the GPU displacing model layers (decisions at ~24 tok/s); a later regression flight of Codex's mission with every layer on the GPU and the CPU projector took 12.9 s of model time over 6 decisions (median 2.3 s), orbit radial error 0.08 m. No same-wording baseline exists for most; the 2026-10-23/24 median decision was 14.9 s.

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

## Decision quality (second pass, same day)

What the first pass left wrong, and what changed. No mission-specific prompt text was added.

| Problem | Change | Live result (AirSim scene 6) |
|---|---|---|
| **Success over-claimed**: twice `finish(completed)` after a FAILED survey, once quoting the partial count as the answer. | The loop checks a `completed` claim once against the receipts: if the latest call of any tool did not succeed, the finish is returned to the model with the unresolved steps (`unresolved_failures` in `fly.py`). A second identical claim is delivered, tagged `unresolved_failures`. A refused repeat observation (`VIEWPOINT_REQUIRED` on inspect/localize) does not count. Failed results now also carry `path_completed`, so a partial survey says it is partial. | 4 missions with failed surveys: the model reported `incomplete` itself twice; once the loop stopped it after 3 failures; once it claimed `completed`, the check fired and it ended `incomplete` (that run, before the wording fix, also re-flew the survey from home; the check message no longer invites a redo). Missions whose failed steps were later redone (orbit retried after a climb) passed without a check. |
| **Invented recovery motion** after `VIEWPOINT_REQUIRED` (forward 10 m). | The result now says a turn gives a new view (`search_for_object`), that flying to a new position needs a distance or place the operator gave, else `ask_user` or `finish(incomplete)`. | The contradicted-view case did not recur in these flights; covered by unit tests only. |
| **"Search the area ... how many cars"** flown as a turn in place. | `search_for_object` now states it stops at the first sighting and cannot count; the counting rule covers "beyond the current view (an area, the surroundings ...)". | 2 of 2 runs chose `survey_count`. |
| AirSim survey routes went stale | Local position (LOCAL_POSITION_NED) requested at 20 Hz instead of 5 Hz (`mavlink_gateway.cpp`): at 5 Hz a pose was routinely ~200 ms old against the 500 ms limit. | Pose-only staleness gone (median 45 ms). Surveys still aborted because the whole sim stalled 0.5-2 s: **the laptop was on battery** (AC offline), frames took ~5 s instead of 2.8 s. Re-test on AC. |

Second-pass flights: house orbit completed; surveillance loop completed (orbit retried after climb); Codex's mission
completed once and ended `incomplete` once (planner found no collision-free climb); surveys see above.

Still open:
- **"Take a look around"** inspects one view. The live catalogue has no turn-and-describe tool (`scan_heading` is not
  exposed to decisions); not changed.
- **Hedged VLM evidence counts as found** ("possible structure"); the LiDAR gate still stops an approach on it.
- When the survey route fails, the model sometimes goes home instead of retrying once as the prompt says.

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

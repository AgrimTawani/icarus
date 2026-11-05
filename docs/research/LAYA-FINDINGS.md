# Laya for Icarus decision speed and determinism: findings

> The scratchpad paths below were session-local. The labelled frame set, labels and the two VLM benchmark
> scripts were kept in `docs/research/vision-bench/` (frames under `output/laya-speed-20261106/vision-bench/`).
> Outcome of this work: [LAYA-DECISION-SPEED-RESULTS.md](LAYA-DECISION-SPEED-RESULTS.md).

Date: 2026-11-06. Branch `feat/laya-decision-speed`. Research only; nothing under `python/` was changed.
Laya reference: `_reference/laya` at v0.4.24 (upstream main). Isolated venv: `/home/agrim/venvs/laya`
(`python3 -m venv`, pip without sudo; torch 2.14.0+cu130, laya 0.4.24, vegaml 0.15.0, **torchvision 0.29.0 added by
hand: `pip install laya` does not pull it and the image path fails without it**, plus `imageio-ffmpeg` for the video test).
Machine: Ryzen 7 7435HS (16 threads), 15.8 GB RAM, RTX 4050 Laptop 6 GB, driver 595.99 / CUDA 13.2.

Every number below was measured on this machine in this session unless marked "doc".

## 0. What the Icarus loop actually spends time on (baseline on this machine)

Measured with the real `decision_tools()` + `system_prompt()` against the 4B `llama-server` started with the manifest's
arguments (`-c 16384 -ngl 99 --fit on --fit-target 512 -b 256 -ub 32 --reasoning-budget 384 --reasoning on`):

| Item | Measured |
|---|---|
| llama-server VRAM, 4B Q4_K_M, 16k context | **3.4 GB** (615 -> 4038 MiB used) |
| Server start to `/health` ok | 3.0 s |
| Decision, prompt NOT in cache (8,145 prompt tokens) | **11.7 s** = 7.4 s prefill (~1,100 tok/s) + 4.3 s generation (186 tokens, ~43 tok/s) |
| Same decision, prompt cached (`cache_prompt`) | **4.4 s** = 0.07 s prefill + 4.3 s generation |
| Tool schemas in the prompt | 23 tools, 26.4k chars ≈ 6.6k tokens of the ~8.1k; `orbit` alone 5.7k chars |
| VLM condition check (Qwen2.5-VL-3B, llama-cli, load-infer-release, `gpu_layers=99`) | 2.7-5.5 s in episode logs; **3.9 s median** on the 24 frames here |

Two structural costs that are easy to miss in "15 s median":

1. **Every VLM call stops the flight LLM.** `LlamaCppRuntime.visual_inference()` (`python/dcm/llama_runtime.py:292`)
   calls `self.stop()` when `gpu_layers != 0`, and `run_mission.py` takes that lease around every `inspect_image` /
   `ground_all` / `localize_target`. The next decision then pays a server restart (3 s) and a full re-prefill (7.4 s)
   because the prompt cache died with the process: **~10 s per decision that follows a camera question, before the
   model has generated a single token.** `search_for_object` makes up to 13 such calls per sweep.
2. Generation is 4.3 s at ~43 tok/s because the reasoning budget (384) plus the tool call is ~190 tokens. Nothing
   Laya does shortens that; only forcing the tool (not recommended, see 1c) or a smaller reasoning budget would.

## 1. What Laya is, in the terms that matter here

- `laya.Agent("convaiinnovations/laya")`: a 150M ModernBERT encoder + decision heads. One forward pass answers a dict
  of typed questions (`choice` over labelled criteria, `score` over ordered levels, `noul` = P(yes)) about a text/JSON
  state. Text only. Probabilities are temperature-scaled, nothing else. The shipped checkpoint warns on load that its
  temperature for `choice` with 11+ options is invalid and is clamped to 0.5 ("treat confidence as uncalibrated").
- `laya.Agent(vegaml=True)`: the `vegaml` engine under laya's names. A frozen Qwen3.5-0.8B reader; features are pooled
  from intermediate layers and a small "physics engine" settles a typed answer, so there is no token generation. Adds
  a conformal `prediction_set` (alpha 0.1), `abstain`, `unbound` (out-of-distribution flag) and `route` =
  `auto|review`. `fit(rows)` fits a logistic head per question on the frozen text features in seconds (a head is
  served only if it beats the zero-shot engine on held-out rows); `calibrate(rows)` refits temperature, conformal
  threshold and a review band. `predict(image=...)` runs the same questions through the reader's vision tower
  (`vegaml/_vision.py`); **heads and calibration do not apply to pixels** (doc and code), so on images only the engine's
  own abstain/unbound/set decide `route`.
- `predict(video=...)`: `laya/video.py`. ffmpeg extracts the sound track; if it carries speech the decision is made on
  the transcript (text path). A silent clip has 4 evenly spaced frames tiled into a 2x2 contact sheet (512 px cells)
  and that **one image** goes through the image path. There is no temporal model: "video" is a contact sheet.
- `predict_batch(states, questions)`: on the ModernBERT checkpoints a real batched forward; on vegaml it is a Python
  loop over states ("vegaml packs a state's questions into one pass, not many states"). `system_one` is an alias of
  `predict`. `laya.shortlist` is an embedding pre-filter for high-cardinality `choice` questions (needs your own
  `embed_fn`). `laya.integrations.tools.LayaToolSelector` wraps "which tool" as one `choice` question. Hooks
  (`on_predict_start/end`) are observation/override callbacks; useful only to log into the episode.

## 2. Where Laya could sit in the Icarus loop (and what the measurements say)

Ranked by expected latency saved per mission, after measuring. Letters refer to the brief's candidates.

### 2.1 (a) Visual condition checks instead of the VLM — the only place that short-circuits a whole LLM step

Where: `ToolExecutor.execute`, branch `name == "inspect_scene"` with `question_type == "condition"`
(`python/dcm/agent_tools.py:716-737`), and `skills.search_for_object` (`python/dcm/skills.py:28-58`) which calls it once
per 30-degree step. Today this runs `python/perception/vlm.py:inspect_image` (Qwen2.5-VL-3B through a fresh `llama-cli`
process per question, grammar-constrained `{"seen":..., "found": true|false|null}`) inside the `visual_inference()`
lease that stops the 4B server.

The Laya shape would be one `noul` (or 2-way `choice`) question on the frame:

```python
agent.predict(image=frame_path, questions={
    "found": {"type": "noul", "instructions": f"Is a {visual_query} visible in this drone camera image?"}})
# -> answers["found"]["noul"] (P(yes)), ["prediction_set"], ["abstain"], ["unbound"], ["route"]
```

and the adapter would map `route == "review"` (or abstain/unbound) to `found=None` so the existing `usable` test in
`agent_tools.py:731-737` turns it into `OBSERVATION_UNAVAILABLE` and `search_for_object` keeps turning instead of
treating "uncertain" as "yes". The LLM still decides *to* search, with what query, and what to do after `found`.

What the measurement says (section 3): **zero-shot, vegaml's image path does not read our frames.** It answers
"true" with p 0.96-1.00 for building, person and car on every one of 24 frames, including road/grass/sky, the
image-withheld prior is already 0.87/0.99/0.91, and `route` is `auto` with a singleton `prediction_set` on all of them:
the conformal machinery certifies the wrong answers, because its calibration was fitted on text features, not pixels.
The 3B VLM we already run gets 23/24, 22/24 and 21/24 on the same frames. So (a) is **not usable as a drop-in**, and the
"never treat uncertain as yes" property we wanted is exactly what the image path cannot give (heads and calibration
do not apply to pixels, so `fit()` on our labelled frames cannot fix it either). Latency numbers for it are in
section 3/4; they are also worse than the VLM on CPU.

### 2.2 Keeping the 4B server alive across camera questions — bigger than any model swap

This is an Icarus property the Laya study exposed rather than a Laya feature. Each camera question costs the *next*
decision ~10 s (restart 3 s + re-prefill 7.4 s, measured) on top of its own 3-4 s. Any visual checker that does not
need the GPU, or that fits beside the 4B, removes that. Laya on CPU would qualify if it were accurate; the current
VLM with `gpu_layers=0` (CPU, 4 threads, as the manifest allows) is the other candidate and should be timed the same
way. See the recommendation in section 6.

### 2.3 (c) Shortlisting tool schemas — measured net loss with the prompt cache, small gain without it

Where: `fly.py:188 tools = decision_tools(context)`. 23 schemas are ~6.6k of the ~8.1k prompt tokens.
Measured: with `cache_prompt` the per-step prefill is 0.07 s, so shrinking the schema set only helps the *first*
decision and decisions after a server restart (7.4 s -> ~2.5 s for ~2.5k tokens at ~1,100 tok/s). But a shortlist that
changes **per step** invalidates the cached prefix every step and would cost ~2-3 s of prefill *each* decision: a
net loss against 0.07 s. A shortlist fixed **per mission** (computed once from the request) keeps the cache and saves
~5 s once per mission plus ~5 s per restart; it also shrinks KV use.
Accuracy: the English ModernBERT checkpoint zero-shot on a 22-way `choice` over the Icarus tool descriptions picked
`takeoff` with p=1.0 for 8/8 mission states (1/8 correct), i.e. the 11+-option temperature clamp makes the
distribution degenerate; `laya.shortlist` needs an external embedder we do not have. A usable shortlist would have to
be `fit()` on vegaml with labelled rows (the episode logs under `logs/episodes/*/events.jsonl` already hold
`agent_decision` records = (messages, chosen tool), so labels are free), and any shortlist must always keep
`ask_user, finish, hold, cancel, land, return_home, takeoff` to stay safe. Expected saving: ~5 s per mission, not
per step. Low priority.

### 2.4 (b) Evaluating conditional / nested branches on the latest tool result + state

Where this would go: between `fly.py:272` (result appended) and the next `runtime.decide` — a Laya pass over
`{"mission": ..., "last_call": name, "result": brief(result), "state": vehicle_state(context)}` asking
`step_done` (noul), `object_found` (noul), `next` (choice: continue / retry_same / change_view / stop_report /
ask_operator). Measured zero-shot on 6 hand-built cases with the English checkpoint: 3/6 on `next`, `step_done`
0.95 on a success and 0.39 on a second safety abort but 0.70 on the first abort; ~1.8 s on CPU for the three questions.
Two things make this a poor fit regardless of accuracy: (1) the typed parts of this branch are already deterministic
in Icarus (`status`, `requires_user_choice`, `VIEWPOINT_REQUIRED`, the repeated-failure counters in
`fly.py:293-302`), so Laya would re-derive what `result["status"]` states; (2) the non-typed part (does the operator's
"if you see a house, orbit it, otherwise land" hold?) is where a 33 ms encoder with 0.36 zero-shot accuracy on the
typed-decisions benchmark (doc) would be deciding mission flow, which is the hardcoding the brief rules out. Where it
*could* earn its place without hardcoding: as a **guard**, not a decider, e.g. a `noul` "does the chosen tool call
contradict the latest result?" that only triggers a concise re-ask of the LLM. Not recommended for the first
integration.

### 2.5 (d) Other API surfaces

- `predict` with several questions per frame: on CPU three `noul` questions on one image took 20.4 s against 4.4 s
  for one, so questions are **not** free on the vegaml image path (GPU numbers in section 4). On the ModernBERT text
  path they are (one encoder pass).
- `fit()`/`calibrate()`: real and fast for **text** (seconds, no gradient), and the episode logs are a labelled corpus.
  This is the one place Laya could become deterministic for Icarus: a fitted head on vegaml for "which skill does this
  operator sentence ask for" (search / orbit / survey_count / approach / plain motion) with `route` gating. But it
  would be a classifier over operator text, not a replacement for a decision step, so its only use is 2.3's
  per-mission shortlist.
- `system_one` is the same call as `predict`. Hooks are logging only.
- `Router(fast=laya.Agent(), vega=agent)`: sends images to vega, text to the encoder; nothing Icarus needs.

## 3. Accuracy sanity check on our own camera frames

24 frames, hand-labelled by looking at them (`labels.json` in the session scratchpad; 12 AirSim neighbourhood frames
from `logs/review/20261024T0720*_house_orbit_regression`, `*_search_count_cars`, `*_surveillance_loop`, and 12 Gazebo
frames from `output/phase1-hardening-20261019/{v3-acceptance,acceptance-r6,v20-...}/camera`). Ground truth:
building 21/24 yes, person 10/24 yes, car 1/24 yes (the car class is positive-poor; read its row as a false-positive
test). The three negatives for "building" are f00 (road+grass), f05 (dark grass/bushes), f13 (Gazebo road, grass, sky
and an orange pole: the kind of frame that produced the stuck loop).

### 3.1 The VLM we run today, same frames, same questions

Qwen2.5-VL-3B Q4_K_M via `inspect_image(..., structured=True, result_type="boolean", gpu_layers=99)`, one fresh
`llama-cli` per question as in production:

| question | correct | fp | fn | `found=None` | median ms (incl. model load) |
|---|---|---|---|---|---|
| building visible | **23/24** | 0 | 1 (f23, roof seen from straight above) | 0 | 3,958 |
| person visible | **22/24** | 0 | 2 (f01 small blue figure, f19 heads at frame edge) | 0 | 3,866 |
| car visible | **21/24** | 3 (f03, f04, f11: suburban streets, no car) | 0 | 0 | 3,849 |

No false "building: true" on the road/grass/sky frames in this sample, so the stuck-loop answer in the brief was not
reproduced here; it is a known failure mode of this model, not a systematic one on these 24 frames.

### 3.2 vegaml 0.8B image path, zero-shot, `noul` questions (CPU, the shape in the brief)

| question | acc | tp | fp | tn | fn | abstain | route=review | wrong answers caught by route |
|---|---|---|---|---|---|---|---|---|
| building | 0.875 | 21 | **3** | 0 | 0 | 0 | 0 | **0 of 3** |
| person | 0.375 | 9 | **15** | 0 | 0 | 0 | 0 | 0 of 15 |
| car | 0.042 | 1 | **23** | 0 | 0 | 0 | 0 | 0 of 23 |
| people count (score none/one/two/3+) | exact 6/24 | | | | | 0 | 24 | sets of 2-3 levels, always "two" |

Every `noul` answer is "true" with p 0.96-1.00 and a singleton `prediction_set`, so `route=auto`. The image-withheld
parity call (`predict_image_text`) gives p(true) = 0.87 / 0.985 / 0.91 for the three questions with **no image**: the
answer is the wording's prior. Asking the negation ("is this image free of any building?") also returns p 0.97-1.00
on the same frames. Conclusion: the `noul` head on the image path does not read these pictures, and the conformal
set certifies that non-answer. Count as `score` is likewise prior-driven ("two" for everything).

### 3.3 Question form matters: a 2-way `choice` does read the frame (CPU, 8 frames: 3 negatives + 5 positives)

| form | acc | fp | fn | review | wrong caught by review | p(building) on the 3 negatives |
|---|---|---|---|---|---|---|
| `noul` "Is a building or house visible?" | 5/8 | 3 | 0 | 0 | 0/3 | 0.99, 1.00, 0.99 |
| `noul` with described `criteria` | 5/8 | 3 | 0 | 8 | 3/3 | 0.94, 0.98, 0.93 |
| **`choice` {building_visible, no_building} with descriptions** | **7/8** | **0** | 1 | 6 | **1/1** | 0.31, 0.31, 0.39 |
| `choice` 5-way scene (building / road_or_ground / trees / sky / people) | 6/8 | 0 | 2 | 4 | 2/2 | 0.13, 0.24, 0.12 |
| `noul` negated | 3/8 | 0 | 5 | 0 | 0/5 | (p(empty) 0.998, 0.995, 0.998) |

So the brief's instinct is right in shape but wrong in type: the picture only moves the answer when the two outcomes
are both named and described as `choice` criteria, and then `route=review` does catch the error, at the price of a
high review rate (6/8). The 24-frame GPU run of the `choice` forms for building, person and car is in 3.4.

### 3.4 The 2-way `choice` form on all 24 frames (CPU)

Not completed: the 24-frame run of the `choice` forms for building, person and car was stopped when the coordinator
asked for CPU and GPU to be released for the live sim (the GPU attempts had OOM'd, see 4.2, and the CPU run was
stalling the sim's sensor bridges). The evidence for the `choice` form is therefore the 8-frame table in 3.3 only
(3 negatives, 5 positives, building question). Before any integration, rerun
`scratchpad/bench_wording.py cpu` (session scratchpad, `OMP_NUM_THREADS=4`, `CUDA_VISIBLE_DEVICES=""`) on the full 24
frames and on a second batch of negatives; the person and car `choice` forms are already in the script.

## 4. Load time, memory and latency on this machine

GPU use was cut short: the mission model was resident on the 6 GB GPU for the user's live sim runs during this
session, and the two attempts to load vegaml's image path on CUDA ended in OOM (see 4.2), so every latency below
is **CPU** unless the row says otherwise. CPU latencies were taken with the user's sim and `llama-server` running
concurrently part of the time; the uncontended figures are marked.

### 4.1 Measured

| What | Device | Load (warm cache) | RSS / VRAM | Per call |
|---|---|---|---|---|
| `Agent("convaiinnovations/laya")` English ModernBERT, 150M | CPU, 16 threads | 7.9 s (import 0.2 s) | 2.3 GB RSS | 1 noul: **640 ms** median; 1 choice(22 opts)+: 1,011 ms; 1 choice+2 noul: 1,787 ms; `predict_batch` of 8 states x 22-way choice: 7,647 ms (~960 ms/state) |
| `Agent(vegaml=True)` 0.8B, text engine only | CPU | 15.8 s | 5.2 GB RSS after load | (not timed on text) |
| `Agent(vegaml=True)` image path, first image call | CPU | + vision copy load on first call | **~7 GB RSS** (two fp32 copies of the 0.8B; machine went to 0.4 GB free with the sim running) | 1 noul on one 848x480 frame: **4.4 s** median (uncontended, 16 threads); 2.3-2.9 s with `OMP_NUM_THREADS=6` under contention; 3 noul + 1 score on one frame: **20.4 s** (questions are not free on the image path); 24 frames x 4 questions: median 18.7 s/frame |
| `Agent(vegaml=True)` on CUDA, text engine only | GPU (before the ban) | 11.4 s | **1.52 GB** VRAM allocated (bf16) | - |
| `Agent(vegaml=True)` image path on CUDA, as shipped | GPU | - | **OOM at 4.72 GiB** with nothing else on the GPU (fp32 vision copy on top of the bf16 text copy) | - |
| Qwen2.5-VL-3B via `llama-cli` (current VLM), per question | GPU, load-infer-release | 0 (loads per call) | ~2.5 GB VRAM transient | **3.9 s** median incl. load |
| 4B `llama-server` decision | GPU | 3.0 s start | 3.4 GB VRAM | 4.4 s cached prompt / 11.7 s cold prompt |

Download sizes, for the record: `convaiinnovations/laya` 804 MB (English only), `nandakishorm/vega-U` engine 75 MB
(+1.4 GB `align_targets.npz` it did not fetch), `Qwen/Qwen3.5-0.8B` 1.4 GB. Install: 5.6 GB venv (torch cu13).

### 4.2 VRAM co-residency (question 4)

Not possible as shipped, and not advisable even with a fix:
- As shipped, `laya.Agent(vegaml=True)` loads the 0.8B **twice** for image work: the text engine
  (`vegaml/_common.py:Perception`, bf16 on CUDA = 1.52 GB measured) and `VegaVision` (`vegaml/_vision.py`,
  `dtype=torch.float32` default, ~3.2 GB). Alone on the empty 5.63 GiB device it reached 4.72 GiB and OOM'd.
- With a bf16 vision copy (a one-line default change, not exposed through laya) it would be ~3.1 GB; the 4B server
  needs 3.4 GB plus the desktop's ~0.6-0.9 GB: **6.5-7 GB on a 6 GB card.** Dropping the text copy as well
  (~1.6 GB for vision only) gives ~5.5-5.9 GB: marginal, and the `--fit on` server would start shedding layers.
  Also the sim renderer and the VLM already contend for the same card.
- So on this hardware Laya's image path would have to run on **CPU**: 4.4 s per yes/no question uncontended (about
  the same as the VLM's 3.9 s, which includes a model load the server-resident VLM would not pay), ~7 GB RSS, and it
  would keep the 4B server alive (no restart + re-prefill), which is where the real ~10 s/decision saving is.
  The text-only ModernBERT checkpoint is cheap enough anywhere (2.3 GB RSS, 0.6-1.8 s on CPU, 33 ms class on a GPU
  per doc) but has no use in the loop that survived measurement (sections 2.3, 2.4).

## 5. The "video" model (question 5)

What it is: `laya/video.py`. ffmpeg extracts audio at 16 kHz; `has_speech` (RMS > 1e-3 and >= 3 words after a
whisper-family transcription) sends the transcript through the **text** path. Otherwise `sample_frames` takes `n=4`
frames at the centres of 4 equal slices, `contact_sheet` tiles them into a 2x2 sheet of 512 px cells (1024x1024,
black padding), and that single image goes through `predict(image=...)`. The answer carries `calibrated: False`.
There is no motion model, no frame order beyond the tile layout, and each frame is downscaled to 512 px on its
long side, then the whole sheet to the processor's `max_pixels` (1024x32x32 ≈ 1 MP), so a 640x360 frame ends up at
roughly 512x288: small objects (a person at 20 m) are gone.

Usefulness for Icarus: the drone never has a clip, it has frames, so the helper to use would be
`laya.video.contact_sheet(last_4_frames)` directly (no ffmpeg; ffmpeg is also not installed on this machine, a static
`imageio-ffmpeg` binary was added to the venv for the test). A question like "across the four frames does the
building stay in view / enter / leave" is a 4-way `choice` over one image: zero-shot, with the `choice` behaviour
seen in 3.3, it is at best a coarse temporal sanity check. Measured latency and answers on 4-frame sheets from the
house-orbit run: **not measured** (the script `scratchpad/bench_video.py` is ready, including the real
`predict(video=)` path over a 12-frame 2 fps mp4 built from the orbit frames, but it was not run before the GPU/CPU
stop). The cost is bounded by what was measured: one contact sheet is one image call, so expect ~4.4 s per
question on CPU (uncontended, 16 threads), and the same prior-driven behaviour for `noul`. Verdict without the number:
not useful for a drone decision; the ~4 s/frame VLM already sees the object at full resolution, and the temporal
questions a turn needs ("did it leave the frame") are answered more reliably by the LiDAR target tracker and the
`views` list `search_for_object` already returns.

## 6. Ranked integration points, risks, and the recommended minimal design

### 6.1 Ranking (expected saving per mission, on this machine)

| # | Change | Where | Expected saving | Measured basis | Status |
|---|---|---|---|---|---|
| 1 | Stop restarting the 4B server around camera questions (any checker that does not take the GPU lease) | `LlamaCppRuntime.visual_inference` (`llama_runtime.py:292`), `run_mission.py:779-830` | **~10 s on every decision that follows a camera question** (3 s restart + 7.4 s re-prefill); a 13-view search saves ~10 s once (it is one tool call) but every `inspect_scene` step saves it | 11.7 s cold vs 4.4 s cached decision | Icarus-side; independent of Laya |
| 2 | Laya vegaml 2-way `choice` as the condition checker, CPU, route-gated | `agent_tools.py:716-737` + `skills.search_for_object` | enables #1; per view ~4.4 s CPU vs 3.9 s VLM (no saving on the view itself) | 7/8 with 0 fp on 8 frames; `noul` form useless | **Needs the 24-frame rerun first**; RSS ~7 GB is a problem beside the sim |
| 3 | Per-mission tool shortlist (not per-step) | `fly.py:188` | ~5 s on the first decision and after each restart; 0 per step | 6.6k of 8.1k prompt tokens are schemas; prefill 1,100 tok/s | Needs `fit()` on episode logs; zero-shot 1/8 |
| 4 | Branch evaluation on tool results | between `fly.py:272` and the next `decide` | 0 (would add 1.8 s CPU) | 3/6 zero-shot; typed statuses already deterministic | Not recommended |
| 5 | Video / contact sheet | - | 0 | section 5 | Not recommended |

What the LLM still has to do in every variant: pick the tool and its arguments, carry the operator's quantities,
decide when to `finish`. Laya never chooses a flight action in this design; it only answers a yes/no about a frame
or trims the schema list, and in both cases its `route=review` falls back to today's behaviour.

### 6.2 Risks

- **Image path accuracy is wording-dependent and uncalibrated.** `noul` and `score` returned the prompt's prior
  with `route=auto` on 24/24 frames; only a described 2-way `choice` read the frames, on 8 of them. The conformal
  set / abstain / unbound signals are fitted on text features, so on images they can certify a wrong answer (they did,
  72 times out of 72 `noul` answers). `fit()`/`calibrate()` cannot repair this: heads and calibration do not apply to
  pixels (doc and `vegaml/_vision.py`).
- **Memory.** Two fp32 copies of the reader on CPU (~7 GB RSS) alongside the sim, the 4B server (3.5 GB RSS) and
  the VLM is more than 16 GB on this laptop under load; it swapped during the CPU run. On the GPU the shipped image
  path OOMs alone. A production adapter would have to construct `VegaVision` directly in bf16/int8 and skip the text
  engine, which is not a public laya API.
- **CPU contention.** 16-thread torch on CPU stalled the sim's sensor bridges in this session; any CPU checker must
  run at `torch.set_num_threads(4)` and be measured again at that setting (4.4 s was at 16 threads).
- **Packaging.** `pip install laya` lacks `torchvision` (image path raises) and `ffmpeg` (video path raises); the
  venv needed both added by hand. Checkpoints: 2.2 GB download on this link took ~25 min.
- **Hardcoding by the back door.** A fitted "which tool" head used to *force* `tool_choice` would move mission
  behaviour into a classifier trained on our own past missions. Use it only to shrink the schema list, never to pick.
- **Review rate.** On the 8-frame `choice` test 6/8 answers routed to `review`; in `search_for_object` that means more
  turns per sweep, which is safe but slower than today's VLM on views the VLM answers correctly.

### 6.3 Recommended minimal design (if #2 survives the 24-frame rerun)

One adapter module, no new framework, no change to prompts or tool schemas:

```
python/perception/laya_condition.py        # new, ~80 lines
    class LayaCondition:
        def __init__(self, threads=4, device="cpu"): lazy-load laya.Agent(vegaml=True, device="cpu"); torch.set_num_threads(threads)
        def check(self, image_path, visual_query) -> dict:
            q = {"view": {"type": "choice",
                 "instructions": "Which best describes this drone camera image?",
                 "criteria": {"visible": f"at least one {visual_query} is in the picture",
                              "absent": f"no {visual_query}; only other scenery"}}}
            a = agent.predict(image=image_path, questions=q)["answers"]["view"]
            found = None if a["route"] == "review" or a["abstain"] or a["unbound"] else a["choice"] == "visible"
            return {"found": found, "validity": "valid" if found is not None else "uncertain",
                    "answer": f'p_visible={a["probabilities"]["visible"]:.2f} set={a["prediction_set"]} route={a["route"]}',
                    "latency_ms": ..., "measurement_kind": "laya_choice", "flight_authority": False}
```

Call sites (two edits, both behind a manifest flag `condition_checker: "laya" | "vlm"`):

1. `scripts/autonomy/run_mission.py:inspect_scene` — when `structured and result_type == "boolean"` and the flag is
   `laya`, call `LayaCondition.check` **without** entering the `visual_inference()` lease (the 4B server stays up);
   everything else (description questions, `ground_all`, `localize_target`) keeps the VLM and the lease.
2. Nothing in `agent_tools.py` or `skills.py` has to change: `found=None` already fails the `usable` test
   (`agent_tools.py:731-737`) and becomes `OBSERVATION_UNAVAILABLE`; `search_for_object` already treats anything but
   `found is True` as "keep turning". That is the "uncertain is never yes" guarantee, and it costs no new logic.

Episode record: the existing `semantic_visual_inspection` event carries the answer string, so the route/set are
logged for free; add the frame's sha as today.

Order of work: (1) first do the server-restart fix (ranking #1) on its own and measure — it is the whole latency win
and needs no Laya; (2) rerun the 24-frame `choice` test at 4 threads, plus 20 more negative frames; (3) only if fp=0
and review rate < 50 %, wire the adapter behind the flag and A/B a search mission with `start-sim`.

## 7. Artefacts (session scratchpad, `/tmp/claude-1000/-home-agrim-Desktop-Icarus/fdc741d6-aeb2-412b-94a7-d54e608d8f8f/scratchpad/`)

`labels.json` (24 hand labels + notes), `frames/` (the 24 frames + contact sheets), `bench_text.py` +
`bench_text_cpu.log`, `bench_image.py` + `image_results_cpu.json` + `bench_image_cpu.log`, `bench_wording.py` +
`wording_cpu8.log`, `bench_vlm.py` + `vlm_results.json`, `llama_vram.py` (server VRAM + cold/cached decision),
`bench_video.py` and `bench_coresident.py` (written, not run), `bf16patch.py` (measurement aid for the fp32 vision copy).
The venv is `/home/agrim/venvs/laya`; the Icarus `.venv` was not touched.

The snippet that worked for an image yes/no (CPU):

```python
# /home/agrim/venvs/laya/bin/python ; CUDA_VISIBLE_DEVICES="" ; needs torchvision installed by hand
import torch, laya
torch.set_num_threads(4)
agent = laya.Agent(vegaml=True, device="cpu")           # 15.8 s warm load; ~5 GB RSS, ~7 GB after the first image
q = {"view": {"type": "choice", "instructions": "Which best describes this drone camera image?",
              "criteria": {"building_visible": "a house or building with walls or a roof is in the picture",
                           "no_building": "only grass, road, trees, sky, open ground; no building"}}}
out = agent.predict(image="frame.png", questions=q)      # ~2.3-4.4 s on CPU per question
a = out["answers"]["view"]
found = None if a["route"] == "review" else a["choice"] == "building_visible"
# a["probabilities"], a["prediction_set"], a["abstain"], a["unbound"], a["route"], a["route_reason"]
```

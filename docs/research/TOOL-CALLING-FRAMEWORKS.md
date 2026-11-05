# Tool-calling frameworks and techniques for the Icarus mission loop

Branch `feat/laya-decision-speed`. Companion to `LAYA-DECISION-SPEED-GOAL.md`.
Research only; nothing here has been run against a simulator or llama-server.

## 0. What the code and the logs say (the ground truth every verdict is tied to)

Measured from the 480 `agent_decision` records in the newest 60 `logs/episodes/*/events.jsonl`
(2026-10-24 runs, pinned llama.cpp `1af554f` 2026-09-19, Qwen3.5-4B Q4_K_M, `--parallel 1`, `-c 16384`,
reasoning off):

| Quantity | Value |
| --- | --- |
| Median decision latency / p90 | **14.9 s / 26.4 s** |
| Median prompt tokens per decision | **8,526** (first decision ~7,990) |
| Median prefill rate on full re-prefill | ~945 tok/s, i.e. ~8–9 s of every slow step is prefill |
| Median generated tokens / rate | 77 tokens at ~43 tok/s, i.e. ~1.8 s |
| Steps that hit the prompt cache (e.g. the `hold`/`approach_target` loop in `20261024T065811`) | **0.9–1.9 s** each |
| Consecutive decision pairs whose `tools` JSON differed | **198 of 426 (46 %)** |

Code facts (from `python/dcm/agent_tools.py`, `fly.py`, `llama_runtime.py`, the GGUF header, the pinned llama.cpp):

- `decision_tools()` exposes **23 tools** per step. Compact JSON of the catalogue is **25.4 KB ≈ 5.4k Qwen
  tokens**; the chat template renders `tool | tojson` plus a ~250-token format preamble, and the logs put
  the first-step prompt at ~8.0k tokens with `system.md` at ~840 tokens, so **≈6.8k of every prompt is
  the tool catalogue**. Largest: `orbit` 5.5 KB (destination `oneOf`), `goto` 2.8 KB, `detect` 1.9 KB
  (80-class enum), `overfly_target` 1.8 KB, `approach_target` 1.8 KB.
- The Qwen3.5 chat template (read from `tokenizer.chat_template` in the GGUF) puts the tools
  **first** in the system turn: `<|im_start|>system\n# Tools ... <tools>{json}</tools> ... \n\n{system.md}`.
  So any change in the catalogue invalidates the prompt cache from token ~10, and `_warm()` (which sends
  `system.md` with no tools) shares almost no prefix with a real decision: it warms nothing useful.
- `decision_tools()` mutates the catalogue per step: `target_id` enums appear/disappear as targets are
  registered, `waypoint_name` enums and the `named_waypoint`/`registered_target` branches come and go,
  `maximum_climb_rate_mps` is popped from `takeoff` once airborne, `standoff_m`/`clearance_above_m`
  defaults are injected, `classes` enum follows `detector_classes`. That is the 46 % churn above, and it
  explains "prefix cache not effective".
- Camera tools (`inspect_scene`, `localize_target`, `search_for_object`, `survey_count`, and any
  object destination that goes through `skills.acquire`) call `runtime.visual_inference()`, which
  **stops llama-server** (`llama_runtime.py:292-307`) so that a separate Qwen2.5-VL-3B llama-server
  (`perception/vlm.py`, `-ngl 99`, ~2 GB + 0.9 GB mmproj) can use the 6 GB GPU. The next decision reloads
  2.9 GB of weights and re-prefills 8–9k tokens: the 15–17 s `localize_target` runs in the log.
- Qwen3.5-4B is a **hybrid** model: `qwen35.full_attention_interval = 4` (3 Gated DeltaNet : 1 attention),
  32 layers plus `qwen35.nextn_predict_layers = 1` (an MTP head is present in this GGUF, block_count 33).
  Recurrent state cannot be rolled back to an arbitrary token; llama-server can only resume from a
  context checkpoint (default `--ctx-checkpoints 32`, taken at the end of each prompt). Append-only
  conversations reuse the cache (the 0.9–1.9 s steps prove it on this build); a change anywhere before
  the last checkpoint forces "full prompt re-processing".
- The loop itself is append-only: `fly.py` appends `vehicle_state()` to the last message and then sends
  `system + mission + all exchanges`. That part is right.
- The pinned llama.cpp: `tool_choice` accepts only `auto|none|required` (`common/chat.cpp:345`), and
  `grammar` together with `tools` throws "Cannot specify grammar with tools" (`chat.cpp:1295`). So the
  decision cannot be narrowed per step by grammar or by naming one tool without changing the catalogue.
  Constrained decoding is already in use: with tools and `tool_choice=required` llama.cpp builds the
  grammar from the schemas (that is why the `reason` padding field for parameterless tools exists).
- `llama_memory_hybrid::get_can_shift()` delegates to the attention cache, so `--cache-reuse` is not
  refused, but KV shifting cannot move recurrent state; for a prefix that changes at the front it has
  nothing to reuse anyway. `--swa-full` is irrelevant (Qwen3.5 has no SWA; the server disables it).

Conclusion before any framework: **~85 % of a slow step is re-prefilling a tool catalogue that changed or
a cache that was destroyed with the process.** No orchestration library fixes that; two local changes do.

## 1. DSPy (stanfordnlp/dspy)

### What it is for this case

- `dspy.ReAct` keeps a structured `dspy.History` with one event per turn and re-sends the whole history
  every step; it supports native function calling through the adapter (`JSONAdapter` on by default,
  `ChatAdapter(use_native_function_calling=False)` by default), reserves a `submit` tool, and does a
  "forced submit" with `tool_choice` on max-iters/parse error/context overflow; it does **not** truncate
  history. The docs say its examples "are not accuracy or latency benchmarks"; the only cost claim is
  "up to 50 % on some tasks" **from provider prompt caching**, i.e. exactly the mechanism we must get from
  llama-server ourselves. ([dspy ReAct deep dive](https://dspy.ai/current/diving-deeper/react/),
  [react.py](https://github.com/stanfordnlp/dspy/blob/main/dspy/predict/react.py),
  [chat_adapter.py](https://raw.githubusercontent.com/stanfordnlp/dspy/main/dspy/adapters/chat_adapter.py))
- Optimizers: `BootstrapFewShot` (~10 examples), `BootstrapFewShotWithRandomSearch` (50+),
  `MIPROv2` (instructions + demos, "200 examples or more to prevent overfitting"; has a 0-shot
  instruction-only mode), `GEPA` (reflective instruction evolution from textual feedback,
  `ScoreWithFeedback`, ICLR 2026; claims >10 % over MIPROv2 and 35x fewer rollouts than GRPO), `SIMBA`,
  `BootstrapFinetune` (weights). ([optimizers](https://raw.githubusercontent.com/stanfordnlp/dspy/main/docs/docs/learn/optimization/optimizers.md),
  [GEPA API](https://dspy.ai/current/api/optimizers/GEPA/overview/), [GEPA paper](https://arxiv.org/abs/2507.19457))
- In `dspy.ReAct` the tool descriptions live inside the react predictor's instructions, so an instruction
  optimizer rewrites tool descriptions too; a third-party walkthrough reports the optimizers learned
  argument conventions from rejected calls in traces. ([LangWatch guide](https://langwatch.ai/docs/improve-your-agent/optimize-with-dspy))
- Works with llama-server via LiteLLM (`dspy.LM("openai/<name>", api_base="http://127.0.0.1:<port>/v1")`);
  native tool calling only if LiteLLM believes the model supports it, otherwise it silently falls back to
  text parsing.

### Verdict: runtime DSPy — reject. Offline DSPy — borrow the method, not necessarily the library.

Runtime (DSPy as the loop):
- Same per-step prompt growth as `fly.py` (full history resent); no latency mechanism we do not already
  have; adds LiteLLM + its own field markers (`[[ ## next_thought ## ]]`) or JSON adapter, which is longer
  than our native request and bypasses llama.cpp's schema grammar unless native calling is on.
- Would have to be re-plumbed under `interrupted_call`, the hard deadline/restart in `_chat_owned`, the
  `approval` mode, `ContextBudgetExceeded`/`drop_oldest_exchange`, `IncompleteModelResponse` concise retry,
  the viewpoint gate and the repeated-failure stops. None of that exists in `dspy.ReAct` (it raises on
  overflow instead of trimming). Net: more code, zero speed-up, less determinism.

Offline (optimize `system.md` + tool `DESCRIPTIONS` against episode logs):
- Plausible and cheap to try, but note what it can and cannot fix. The two worst runs in the logs are
  loops (`localize_target` x8 at 15 s each; `hold`/`approach_target` alternating x30). Those are feedback
  and catalogue design problems (fixed partly in `91d55ef`) that no instruction wording removes. Prompt
  text can help with "wrong tool" and "skipped step" on vague missions.
- With a stable cached prefix (section 3) the **length** of `system.md` stops mattering for latency
  (paid once per mission); optimizers tend to lengthen instructions, and GEPA will happily write
  mission-specific rules learned from the trainset into the prompt, which the branch rules forbid. Guard
  with held-out mission families and a length penalty in the metric.
- The optimized artefact is text. Paste it into `system.md`/`DESCRIPTIONS`; nothing from DSPy ships.

Concrete recipe if you run it (time-box: one day):
- Dataset: one example per decision from `events.jsonl`: inputs = (mission, prior exchanges, vehicle
  state line, tool catalogue); label = the correct next call (tool name + the argument keys and
  operator-given quantities). Successful episodes label themselves; for failed ones hand-label the
  first wrong decision only. Aim for 150–300 step examples over 8–10 mission families; keep 2 families
  held out.
- Metric (step level, 0..1): 0.6 tool name match + 0.3 required-argument match (keys present, operator
  quantities exact, `visual_query` class match) + 0.1 "not a repeat of the previous identical failed
  call"; minus 0.001 per instruction token over the current length. Episode-level success stays in sim
  (headless, camera-checked), outside the optimizer, as the acceptance gate.
- Program: a single `dspy.Predict` whose signature instructions are `system.md` and whose inputs are the
  rendered conversation; run `MIPROv2(auto="light", zero-shot)` first, `GEPA` second with a strong
  reflection LM (API model, offline only). Compare on held-out families; adopt only if step accuracy
  rises and sim success does not fall.
- Honest note: that program is a search over instruction strings scored by a metric. A 60-line
  proposer loop with the same metric gives the same thing without the dependency. The durable value is
  the dataset + metric + replay harness, which the branch needs anyway for "every claim backed by a
  measurement".

## 2. ZeroClaw

### What it is (verified)

- `github.com/zeroclaw-labs/zeroclaw`: "Fast, small, and fully autonomous AI personal assistant
  infrastructure" — a **single Rust binary agent runtime** for a personal assistant (channels: Discord,
  Telegram, Matrix, email, webhooks; gateway + web dashboard; cron; SOP engine; GPIO/I2C/SPI peripherals;
  SQLite + embeddings memory; OS sandboxes; "supervised" autonomy by default). Created 2026-02-13, Rust,
  Apache-2.0/MIT, **33.1k stars** at 2026-11-05, topics include `openclaw`; it has an import command for
  `~/.openclaw/` data, and community tables position it as the small-footprint sibling of OpenClaw (TS)
  and NanoBot (Python) with self-reported "<5 MB RAM, <10 ms startup" (unverified, edge-normalised).
  ([repo](https://github.com/zeroclaw-labs/zeroclaw), [search summary of mirrors](https://github.com/rustystack/zeroclaw))
- Tool calling (from `docs/book/src/architecture/tool-execution-lifecycle.md` and
  `request-lifecycle.md` in the repo): `Tool` trait = name, description, JSON schema, async `execute`;
  a scoped registry with "per-run narrowing"; "native-tool providers receive structured specs;
  text-protocol providers receive prompt instructions"; `agent::turn::parse_response` normalises native
  and text calls; `run_tool_call_loop` repeats "until it returns final text, hits cancellation, or
  reaches the iteration cap"; `call_prep` runs approval and "ordinary duplicate-call guards" before
  dispatch; results are "bounded, and appended back to provider history". The text parser crate
  `zeroclaw-tool-call-parser/src/lib.rs` is ~209 KB of heuristics for `<tool_call>`, JSON, markdown and
  GLM-style envelopes.
- Local models: Ollama and "any OpenAI-compatible endpoint" with fallback chains and routing; no
  llama.cpp-specific code, no prompt-cache, slot, grammar or checkpoint handling, no hybrid-model
  awareness. Its loop is a plain ReAct loop over a streaming provider.

### Verdict: reject (nothing to adopt); two small ideas already present or trivial

- It solves a different problem (a messaging-facing assistant with many channels and a sandbox) and
  its agent loop is the same shape as `fly.py` minus our deadline/interrupt/approval machinery. Running
  Icarus inside it would mean re-implementing the Drone API executor as its `Tool`s and losing
  `interrupted_call`, the hard deadline restart, the viewpoint gate and episode recording.
- Ideas worth a glance: (a) "per-run narrowing" of the tool registry = our `decision_tools(names=)`;
  (b) duplicate-call guards *before* dispatch rather than after N failures — `fly.py` now stops after 3
  identical failures (`91d55ef`); moving the check in front of dispatch (reject the call with a one-line
  reason as the tool result) saves the flight attempt, not the model call.

## 3. Techniques, ranked for Icarus (latency win x simplicity)

| # | Technique | Expected win on the measured loop | Effort | Applies? |
| --- | --- | --- | --- | --- |
| 1 | **Freeze the per-episode prompt prefix** (constant tool catalogue; dynamic facts in the state line; validate post-hoc) | 46 % of steps go from ~15 s to ~2 s (cache-hit steps measured at 0.9–1.9 s) | small, `agent_tools.decision_tools` | yes |
| 2 | **Never kill llama-server for camera work** (one server, one multimodal model, two slots; or CPU VLM) | removes the 2.9 GB reload + 8 s re-prefill after every camera call (all the 15–17 s `localize_target` steps) | medium, `llama_runtime.visual_inference`, `perception/vlm.py` | yes |
| 3 | Warm with the real catalogue; log `timings.prompt_n`/cache hits per decision | first decision ~8 s → ~2–3 s; gives the measurement the branch rules require | tiny | yes |
| 4 | Shrink the catalogue (orbit/goto/overfly/approach `oneOf` destinations, 80-class `detect` enum) | only the first step after 1–3 (6.8k → ~3k tokens ≈ 4 s once per mission); better 4B tool choice (fewer/shorter tools → higher selection accuracy, RAG-MCP evidence) | medium | yes, second |
| 5 | Speculative decoding: MTP head already in this GGUF (`--spec-type draft-mtp --spec-draft-n-max 2`) or Qwen3.5-0.8B draft (`-md`, `--spec-type draft`, ~0.6 GB) | generation 1.8 s → ~1.0–1.3 s if acceptance is high on XML tool calls; reports range from 1.7x to no gain; MTP reported to slow prefill | small to try, must measure | after 1–2 |
| 6 | Plan-then-execute (ReWOO / LLMCompiler) | N model calls → ~2 per mission; plan is reviewable in approval mode | medium–large; one-shot plans from a 4B are the risk | experiment only, after 1–4 |
| 7 | Phase gating by post-hoc validation (not by catalogue) | determinism, no latency | small | yes |
| 8 | Tool retrieval / shortlisting | none at 23 tools; a per-step shortlist changes the prefix and kills the cache | — | no |
| 9 | Constrained decoding (GBNF) | already on; nothing to add | — | already |
| 10 | Small function-calling models (xLAM-2, Hammer 2.1, FunctionGemma, Qwen3-4B) | none on prefill; single-turn API-mapping benchmarks do not measure our state-conditioned multi-turn choice; restrictive licences | large | not now |

### 3.1 Prompt/KV cache reuse on a hybrid model (the real lever)

- Mechanism on this build: `cache_prompt=true` → common-prefix match → for hybrid/recurrent memory the
  server restores the newest context checkpoint whose position is at or before the divergence and
  reprocesses from there; without one it logs "forcing full prompt re-processing due to lack of cache
  data (likely due to SWA or hybrid/recurrent memory)" (`server-context.cpp:3349-3400`). Checkpoints are
  created when a prompt finishes processing (`create_checkpoint`), default 32 per slot, min spacing
  8192 tokens only enforced when the list is full. The design comes from
  [PR #15293 (SWA checkpoints)](https://github.com/ggml-org/llama.cpp/pull/15293) and
  [PR #16382 (recurrent/hybrid checkpoints)](https://github.com/ggml-org/llama.cpp/pull/16382); the last
  response is always re-evaluated. [Issue #24714](https://github.com/ggml-org/llama.cpp/issues/24714)
  (Qwen3.5/3.6, closed as "expected behaviour") confirms that editing anything but the tail forces full
  reprocessing on these models, while "adding new messages to a multi-message conversation did not
  trigger it".
- Therefore the rule for Icarus is simple: **the rendered prompt may only ever grow at the end.** The
  catalogue must be byte-identical for the whole episode. Move what changes into the per-step
  `vehicle_state()` line (it already lists "measured targets" and "saved waypoints"): keep
  `registered_target`/`named_waypoint` branches always present with plain `string` `target_id`/
  `waypoint_name`, and reject unknown ids in `validate_call` with a message naming the known ones.
  Keep `maximum_climb_rate_mps` in `takeoff` and have the executor return the existing
  NEEDS_CLARIFICATION when airborne. Freeze `standoff_m`/`clearance_above_m` defaults at episode start
  (they come from `operating_defaults()`, which does not change mid-flight) and the `classes` enum from
  the detector at episode start.
- `--cache-reuse N`: not useful here (nothing to shift; recurrent state cannot be shifted).
  `--swa-full`: not applicable (no SWA). `--ctx-checkpoints`: keep the default; `--checkpoint-min-step 0`
  did not help the reporters in #24714 and is unnecessary once the prefix is frozen.
- Slot save/restore (`--slot-save-path`, `POST /slots/0?action=save|restore`): the fallback if the server
  must still be stopped; it saves the recurrent state too (`llama_state_seq_*`), so a restore skips the
  8-9 s prefill but not the weight reload.
- Measure it: `timings.prompt_n` in the response is the number of tokens actually processed; record it
  next to `tokens_prompt` in `last_stats`, and grep the stderr tail for "restored context checkpoint" vs
  "forcing full prompt re-processing".

### 3.2 One GPU, one server (why the camera path is the second lever)

- Current: Qwen3.5-4B (2.9 GB) is unloaded so Qwen2.5-VL-3B (2.0 GB + 0.9 GB mmproj) can load; both
  together exceed the 6 GB card. Each camera call therefore costs a weight reload plus full prefill on
  the next decision, and the log's "30–150 s after camera calls".
- Qwen3.5-4B is itself a vision-language model (the HF card: "causal language model with a vision
  encoder"; `mmproj-Qwen_Qwen3.5-4B-f16.gguf`, 54 MB, is already in `~/models/qwen`). Run one
  llama-server with `--mmproj`, `--parallel 2` (size `-c` for two slots or use `--kv-unified-per-slot`):
  slot 0 holds the mission conversation (text only), slot 1 answers the yes/no and grounding questions
  with the image. Multimodal only disables `cache_reuse`/`ctx_shift`, not checkpoints. VRAM ≈ 3.2 GB +
  image buffers, so no swap, no reload, no re-prefill. The 2.7 s VLM answers become a request on the
  same process. Caveat: the yes/no calibration of Qwen3.5-4B on "is there a building" must be measured
  against the Qwen2.5-VL-3B baseline (which the goal doc already flags as uncalibrated).
- If a separate VLM stays: run it with `gpu_layers=0` (the `_cpu_only_arguments` path exists) or a
  smaller VLM (SmolVLM-500M is in `~/models/edge`), and never stop the decision server.

### 3.3 Catalogue size and tool choice

- Evidence that fewer/shorter tools help small pickers: RAG-MCP reports tool-selection accuracy 13.6 % →
  43.1 % and >50 % fewer prompt tokens when only relevant tools are shown
  ([arXiv 2505.03275](https://arxiv.org/abs/2505.03275)); a cloud-scale study reports a naive "all tools
  inlined" baseline falling from 92.4 % (15 tools) to 78.2 % (640 tools)
  ([arXiv 2607.15593](https://arxiv.org/pdf/2607.15593)). At 23 tools we are at the small end, so the
  gain is from *schema bulk*, not count: `orbit` alone is 5.5 KB because its destination `oneOf`
  enumerates coordinate/home/launch/visible_object/registered_target/named_waypoint variants.
- Cuts that keep behaviour: one `destination` object with a `type` enum and optional fields (validate
  the combination post-hoc, as `validate_destination` already does); replace the 80-class `detect`
  enum by the detector's live class list rendered once in the state line (validation already filters
  by `detector_classes`); drop `fly_pattern`, `overfly_target`, `assess_landing_zone`, `move_relative`
  from the per-step catalogue if the mission families never need them (they remain Python-callable).
  Target: ≤ 3k tokens of tools. Do this *after* 3.1, otherwise its effect is masked.
- Tool retrieval/shortlisting per step is the wrong tool here: it changes the prefix every step.
  Deterministic gating belongs in `validate_call`/`ToolExecutor` (reject `arm` when airborne, reject
  motion when landed, with a one-line reason), which costs no tokens and keeps the cache.

### 3.4 Plan-then-execute vs step-wise ReAct

- ReWOO reports "5x token efficiency" on HotpotQA by planning without observations
  ([arXiv 2305.18323](https://arxiv.org/abs/2305.18323)); LLMCompiler reports up to 3.7x latency and 6.7x
  cost reduction vs ReAct by planning a DAG of calls and replanning when needed
  ([arXiv 2312.04511](https://arxiv.org/abs/2312.04511)). Both gains come from parallelisable,
  observation-independent sub-tasks; our missions are sequential, and the interesting ones are
  observation-dependent ("if found, go close and orbit").
- For Icarus the honest version is "plan once, step with the plan in context": the first decision is a
  constrained JSON plan (`steps[{tool, args, if: found|not_found, then[...]}]`, recursion is expressible
  in llama.cpp's JSON-schema→GBNF) stored as the first assistant message (part of the cached prefix);
  later steps pick tools as now and the loop only consults the model at branches, failures and
  `finish`. For a 4-step mission that is 2–3 model calls instead of 6–8, and the plan is reviewable in
  approval mode. Risk: one-shot plan quality from a 4B model, and it is a reversal of the v1 planner →
  v2 direct-agent decision (`config/dcm/vague_terms.json` still describes the old planner grammar).
  Run it as an experiment after 3.1–3.3, on the vague-mission suite, not before.

### 3.5 Generation: speculative decoding and MTP

- Generation is only ~1.8 s of a step (77 tokens at 43 tok/s), so this is a second-order win after the
  prefill fixes, but it is cheap to try.
- llama.cpp merged MTP support on 2026-05-16 ([PR #22673](https://github.com/ggml-org/llama.cpp/pull/22673));
  the pinned build (2026-09-19) includes it (`--spec-type draft-mtp`, `--spec-draft-n-max N`), and this
  GGUF carries the MTP head (`qwen35.nextn_predict_layers = 1`). Reported gains 1.4–2x on 27B targets
  with ~75 % acceptance; a 6 GB laptop run showed a smaller gain; prompt processing slowed. Tool-call
  XML is highly predictable, so acceptance should be good; measure `tokens_predicted`/s before and after.
- Draft-model alternative: Qwen3.5-0.8B is the vocabulary-matched draft; speculative checkpointing for
  recurrent targets exists ([PR #19493](https://github.com/ggml-org/llama.cpp/pull/19493)); one
  Qwen3.6-35B-A3B write-up found no net gain in llama.cpp ([hackmd](https://hackmd.io/@thc1006/SJly6IE6Wx.md)),
  so expect anything from 0 to 1.5x. The 0.8B draft also costs ~0.6 GB VRAM that 3.2 needs.

### 3.6 Constrained decoding

- Already in place: llama-server with `--jinja`-style native tool templates builds a grammar from the
  schemas when `tools` + `tool_choice=required` are sent; Qwen3.5 uses an XML `<function=..><parameter=..>`
  format, which is why unparsable output surfaces as the "peg-native" HTTP 500 that `_chat_owned` maps
  to `IncompleteModelResponse`. ([function-calling.md](https://github.com/ggml-org/llama.cpp/blob/master/docs/function-calling.md))
- Limits that shape the design above: `tool_choice` cannot name a function, and `grammar` cannot be
  combined with `tools`; `-ctk q4_0`-style KV quantisation degrades tool calling (docs) — keep KV f16.

### 3.7 Small specialised function-calling models

- xLAM-2-3b-fc-r (Qwen2.5-based, APIGen-MT, GGUF available, **CC-BY-NC-4.0**)
  ([card](https://huggingface.co/Salesforce/xLAM-2-3b-fc-r)); Hammer2.1-3b (Qwen2.5-Coder-based, function
  masking, **qwen-research** licence) ([card](https://huggingface.co/MadeAgents/Hammer2.1-3b),
  [paper](https://arxiv.org/abs/2410.04587)); FunctionGemma 270M (Gemma 3, "intended to be fine-tuned",
  BFCL simple 61.6 / multiple 63.5 zero-shot, Mobile Actions 58 % → 85 % after fine-tune)
  ([card](https://huggingface.co/google/functiongemma-270m-it)); Qwen3-4B-Instruct-2507 (Apache, BFCL-V4
  ~35 % prompt mode per aggregators) vs Qwen3.5-4B (~50 % BFCL-V4 per one comparison site; sources
  disagree) ([BFCL](https://gorilla.cs.berkeley.edu/leaderboard.html), V4 as of 2026-04-12).
- Why not now: BFCL measures single-turn request→API mapping; our failure mode is choosing the next
  step given state, rules and history, which these models were not trained for; the latency problem is
  prefill, which they do not change; and the two strongest small ones have non-commercial or research
  licences. Their one structural advantage, non-hybrid attention (trivial prompt caching), disappears
  once the prefix is frozen. If, after 3.1–3.4, step accuracy is still the blocker, the right move is a
  LoRA/`BootstrapFinetune` of Qwen3.5-4B on the replay dataset from section 1, not a model swap.

## 4. Recommended order of work

1. Freeze the catalogue (3.1) and warm with it; add `prompt_n`/cache logging. Expect median ≈ 2–3 s on
   non-camera steps immediately; verify "restored context checkpoint" in the stderr tail.
2. Stop unloading the decision server for camera work (3.2); try the single multimodal server first.
   Expect the post-camera steps to drop from 15–17 s to the same 2–3 s.
3. Trim the catalogue to ≤ 3k tokens (3.3) and move dynamic ids to the state line; re-run the vague
   suite to check tool-choice accuracy did not fall.
4. Build the step-replay dataset + metric (section 1) from the episode logs; use it for every prompt or
   catalogue change. Optionally run the DSPy/MIPROv2 experiment on top of it.
5. Try MTP (`--spec-type draft-mtp`) for the generation second; keep it only if measured faster.
6. Only then consider the plan-once variant (3.4).

## Sources

- Icarus: `python/dcm/fly.py`, `python/dcm/agent_tools.py`, `python/dcm/skills.py`,
  `python/dcm/llama_runtime.py`, `python/perception/vlm.py`, `config/dcm/prompts/system.md`,
  `config/dcm/agent-runtime.json`, `logs/episodes/2026102*/events.jsonl`,
  `third_party/llama.cpp` @ `1af554f` (`tools/server/server-context.cpp`, `common/chat.cpp`,
  `common/arg.cpp`, `common/speculative.cpp`), GGUF header of `Qwen_Qwen3.5-4B-Q4_K_M.gguf`.
- DSPy: https://dspy.ai/current/diving-deeper/react/ ; https://github.com/stanfordnlp/dspy/blob/main/dspy/predict/react.py ;
  https://raw.githubusercontent.com/stanfordnlp/dspy/main/dspy/adapters/chat_adapter.py ;
  https://raw.githubusercontent.com/stanfordnlp/dspy/main/docs/docs/learn/optimization/optimizers.md ;
  https://dspy.ai/current/api/optimizers/GEPA/overview/ ; https://arxiv.org/abs/2507.19457 ;
  https://langwatch.ai/docs/improve-your-agent/optimize-with-dspy
- ZeroClaw: https://github.com/zeroclaw-labs/zeroclaw (README, `docs/book/src/architecture/tool-execution-lifecycle.md`,
  `docs/book/src/architecture/request-lifecycle.md`, `crates/zeroclaw-tool-call-parser/src/lib.rs`; repo metadata via GitHub API) ;
  mirror/positioning table: https://github.com/rustystack/zeroclaw
- llama.cpp: https://github.com/ggml-org/llama.cpp/pull/15293 ; https://github.com/ggml-org/llama.cpp/pull/16382 ;
  https://github.com/ggml-org/llama.cpp/issues/24714 ; https://github.com/ggml-org/llama.cpp/pull/19493 ;
  https://github.com/ggml-org/llama.cpp/pull/22673 ; https://github.com/ggml-org/llama.cpp/blob/master/docs/function-calling.md ;
  https://github.com/ggml-org/llama.cpp/blob/master/tools/server/README.md
- Qwen3.5: https://huggingface.co/Qwen/Qwen3.5-4B ; small-model series coverage
  https://insiderllm.com/guides/qwen-3-5-small-models-9b-beats-30b/ ; draft/spec reports
  https://hackmd.io/@thc1006/SJly6IE6Wx.md
- Plan-then-execute: https://arxiv.org/abs/2305.18323 (ReWOO) ; https://arxiv.org/abs/2312.04511 (LLMCompiler)
- Tool selection/retrieval: https://arxiv.org/abs/2505.03275 (RAG-MCP) ; https://arxiv.org/pdf/2607.15593 ;
  https://arxiv.org/pdf/2510.04550 (TRAJECT-Bench) ; https://arxiv.org/abs/2503.01763 (ToolRet)
- Small FC models: https://huggingface.co/Salesforce/xLAM-2-3b-fc-r ; https://huggingface.co/MadeAgents/Hammer2.1-3b ;
  https://arxiv.org/abs/2410.04587 ; https://huggingface.co/google/functiongemma-270m-it ;
  https://gorilla.cs.berkeley.edu/leaderboard.html ; https://llm-stats.com/models/compare/qwen3.5-397b-a17b-vs-qwen3.5-4b ;
  https://benchmarklist.com/models/qwen3-4b-instruct-2507/

# UAV / robot LLM-agent architectures: what transfers to the Icarus mission loop

Branch `feat/laya-decision-speed`, 2026-11-06. Web research plus a read of today's episode logs; no code
was changed and nothing here was run against a simulator. Companion to `TOOL-CALLING-FRAMEWORKS.md`
(which already covers prompt caching, catalogue size, MTP, small function-calling models, DSPy, ZeroClaw)
and `LAYA-DECISION-SPEED-RESULTS.md`. Laya, DSPy, ZeroClaw and MTP speculative decoding are not revisited.

How to read the numbers: paper figures were read from the arXiv HTML pages through an automated page
reader, not typed from the PDFs by hand. Spot-check any figure before quoting it outside this repo.
Section 6 lists what could not be verified.

## 0. Bottom line

- No surveyed project runs a zero-shot 3-4B model as a multi-step UAV mission agent and reports good
  results. The ones that work with small models either fine-tune on their own tool set (Octopus v2,
  TinyAgent, BTGenBot-2, the distilled SPINE planner, DroneCATS' 2B fine-tune) or restrict the model to
  parsing (NeLV). Zero-shot small models score 30-40 % (Taking Flight with Dialogue), 6 % (Qwen3.5-4B as a
  low-level visual pilot in DroneCATS) or fail multi-step missions outright (the MAVLink/MCP harness).
  Icarus's composite skills, static catalogue, executor guardrails and receipt check are what the
  stronger designs converge on; the literature has no missing piece that would change the picture.
- Decision latency is no longer where mission time goes. In today's logs the model accounts for a
  median 16.5 s of a median 274 s episode (about 6 %). Every latency mechanism below is bounded by that.
- What is left to gain is on the output side: 80-90 % of a 2 s decision is generating tool-call tokens,
  and the Qwen3.5 native call format costs about 14-18 tokens of wrapper plus 15-20 tokens per
  parameter. The two transferable latency ideas are (1) emit fewer tokens per call and (2) generate
  while flying instead of while hovering (TypeFly's two ideas). The rest of the popular mechanisms
  (tool RAG, functional tokens, plan caching, single-shot planning) address prompt cost or network
  round trips that a cached local prompt does not pay.

## 1. Our numbers (the yardstick for every verdict)

Given: ~8-9k prompt tokens, ~6.8k of it a static cached catalogue; per decision ~100-300 new prompt
tokens at ~1000 tok/s (0.1-0.3 s) plus 40-80 generated tokens at ~42 tok/s (1.0-1.9 s); median ~2 s;
4-9 decisions per mission; camera questions 0.8-2.5 s on the same resident model.

Measured for this note from the 28 episodes / 170 `agent_decision` records logged on 2026-11-06
(`logs/episodes/20261106T*/events.jsonl`; several ran beside a live sim, so rates are a little lower
than the clean figures above):

| Quantity | Value |
| --- | --- |
| Decisions per episode | median 6 (4-9) |
| Generated tokens per decision | median 69.5 (p10 29, p90 118), at a median 35 tok/s under sim load |
| Generated tokens per mission | median 385, i.e. ~9 s at 42 tok/s |
| Model time vs episode wall time (last 14 episodes) | median 16.5 s of 274 s, 4-16 % |
| Fit of generated tokens (n=170, residual sd 18) | ~17.6 + 12.3 per parameter + 0.35 per parameter-name char + 0.24 per value char |
| Fixed wrapper, measured | `land {}` = 14 tokens; `hold {"duration_s":10}` = 28; `takeoff {"target_altitude_agl_m":5}` = 31 |
| Biggest calls | `search_for_object` 114, `survey_count` 114, `approach_target` 96, `orbit` 80, `finish` 79 |
| Optional parameters emitted at exactly their schema default | 13 % of all output tokens, median ~57 tokens per mission |
| `finish` message | 19 % of all output tokens (after the flying is done) |
| Decisions that followed a SUCCEEDED non-observational step | 88 of 170 (52 %); 63 excluding `finish`; median 2 per mission |
| Non-SUCCEEDED tool results | 28 of 145 (15 ABORTED_BY_SAFETY, 7 FAILED, 5 REJECTED, 1 VIEWPOINT_REQUIRED) |
| Responses with more than one tool call | 0 (the request already sends `parallel_tool_calls: true`; `fly.py` executes only the first) |

Two details matter later. `search_for_object` always carried `clockwise=true, step_deg=30, rate_deg_s=30`
(18 of 18 calls) and usually `max_sweep_deg=360` and `camera`, although only `visual_query` is required:
about 70 of its 114 tokens restate defaults. And TypeFly's cloud latency model had an output token
costing ~2800 input tokens; for us the ratio is ~24 for uncached tokens and effectively infinite for
cached ones, so the conclusion "only output tokens matter" holds even more strongly here.

## 2. Survey

Columns: what the model emits; how often it is called; how conditionals and replanning work; what is
done for latency; reported numbers; model. "n.r." = not reported in the paper.

### 2.1 UAV agents

| Project | Output | Calls per mission | Conditionals / replanning | Latency mechanism | Reported numbers | Model |
| --- | --- | --- | --- | --- | --- | --- |
| **TypeFly / MiniSpec** (Yale; [arXiv 2312.14950](https://arxiv.org/abs/2312.14950), [site](https://typefly.github.io)) | A whole program in MiniSpec, a tiny DSL: skill calls with <=2-char abbreviations, `N{...}` bounded loops, `?cond{...}` conditionals, variables, `->` return | 1 plan, plus a new plan on `replan`, plus a 1-2 token answer per `probe` | Conditionals are inside the program and run by the interpreter; `probe(question)` asks the LLM a small question mid-plan; replan is triggered by the plan, by errors, or by executor budgets (rotation >180 deg, forward >10 m) | Output-token minimisation (DSL + abbreviations) and stream interpreting: the drone executes statement 1 while the LLM generates statement 2 | MiniSpec -34 % output tokens on average (up to -42 %) vs Python; response time -32 %, -62 % with streaming; plans of 6-47 tokens; time to first action 1.1-1.6 s; 10/10 on 10 of 11 tasks, 7/10 on one (0/10 without replan); latency model with output-token cost ~2800x input-token cost | GPT-4 (cloud); YOLOv8 on an RTX 4090 edge server; Tello |
| **ChatGPT for Robotics / PromptCraft** (Microsoft; [arXiv 2306.17582](https://arxiv.org/abs/2306.17582)) | Python against a small, descriptively named function library; XML tags for parsable parts | Dialogue; human "on the loop" reviews code before it runs | Loops and sensor checks live in the generated code; corrections by follow-up chat | none | Qualitative only (AirSim inspection, obstacle avoidance) | ChatGPT |
| **GSCE** ([arXiv 2502.12531](https://arxiv.org/abs/2502.12531)) | Python against skill APIs; prompt = Guidelines + Skill APIs + Constraints + Examples | 1 | In code | none | ">90 % success" on 44 tasks per a secondary summary (not verified) | cloud LLMs |
| **AeroAgent / ROSchain** ([arXiv 2311.15033](https://arxiv.org/abs/2311.15033)) | An `operation` + `config` pair looked up in a command table; separate "active observation" requests for extra sensors | One per observe-act step | Step-wise; multimodal memory retrieved from a vector store | none; paper only says iterative prompting had "significant latency" | Normalised rewards vs a DRL agent; no latency | GPT-4V |
| **REAL** (MIT; [arXiv 2311.01403](https://arxiv.org/abs/2311.01403)) | A list of controller-function names plus a reason | 0.1-1 Hz inside planning/control loops | Step-wise; high-rate data is pre-digested (FFT) into text first | Low-rate LLM decoupled from the fast controller | Altitude error ~30 cm -> ~10 cm with tuning calls; no latency table | GPT-4 API from an onboard NUC |
| **SPINE** (Penn; [arXiv 2410.03035](https://arxiv.org/abs/2410.03035)) | JSON `{primary_goal, relevant_graph, reasoning, plan[2-4 actions]}`; only the first action is executed | 6.6 (sim) / 8.3 (real) LLM queries per mission | Receding horizon: replan on map updates; a validator checks syntax/reachability/explorability and returns natural-language feedback, bounded retries, then the user | none | 94.3 % sim / 100 % real; about half the time and distance of a plan-once baseline that used 1.4-1.7 queries (536 s vs 1245 s sim); the one sim failure was declaring completion early | GPT-4 |
| **SPINE distilled, onboard** ([arXiv 2505.09477](https://arxiv.org/abs/2505.09477)) | Same | Same | Same | Small model on a Jetson Orin NX | GPT-4o 100 %, distilled Llama-3.2-3B (LoRA, GPT-4o teacher) 72.7 %, stock Llama-3.2-3B 9.2 % on 11 short-horizon specs; the distilled model "struggled with multi-iteration planning"; latency n.r. | Llama-3.2-3B |
| **Taking Flight with Dialogue** (Southampton; [arXiv 2506.07509](https://arxiv.org/abs/2506.07509)) | One primitive per step: `Turn(theta)` or `Move(d)` | <=20 steps | Step-wise; a separate VLM answers yes/no visibility | none; history capped at 5 commands | Mission success 30-40 % for Gemma3-4B, Qwen2.5-3B, Llama-3.2-3B (100 % syntactically valid commands); DeepSeek-7B 38 % valid, 0-5 % success; latency n.r. | 3-7B via Ollama on a workstation, PX4 |
| **PEACE** ([arXiv 2606.00104](https://arxiv.org/abs/2606.00104)) | One typed plan: reasoning + ordered tool-call list | 1, plus bounded replans on tool failure | No conditionals; geometric safety layer clamps altitude and aborts geofence breaches, failure goes back to the planner | Fewer calls than ReAct (claimed, not measured) | Qualitative, 108 prompts in PX4 SITL; paper itself notes open-loop plans are brittle in dynamic scenes | unnamed "off-the-shelf LLM" |
| **SRDrone** ([arXiv 2508.15501](https://arxiv.org/abs/2508.15501)) | XML behaviour tree, then structured edits to it | Plan + up to 5 refinements | BT control nodes; outcome judged from the whole state sequence, not the final state; kNN experience base | none | 96.25 % real-world success; +44.87 % over baselines; latency n.r. | cloud LLM, Orin NX for the rest |
| **MAVLink/MCP harness** (Ramos Silva & Burke; [arXiv 2601.15486](https://arxiv.org/abs/2601.15486)) | Native function calls, 98 tools, ~23k-token schema every turn | Step-wise | Model treated as an "untrusted commander": server-side tiers, confirmation tokens, independent geofence, audit log | Provider prompt caching (58-99 % hit) | ArduPilot/PX4 SITL and real aircraft; cloud models 76-100 % on flying missions, decision medians 0.65-3.05 s; local models <=8B: 14.5-35.3 s per turn, passed the 2 simple missions and failed the 6 multi-step ones ("a measured capability floor at the 4-8 B scale"); a 37.8 min mission owned by the server completed, the same mission supervised by a model passed 3 of 9; recovery after a rejected call 53-69 %, model-dependent | 11 cloud models, 4 local |
| **DroneCATS** ([arXiv 2609.01404](https://arxiv.org/abs/2609.01404)) | One JSON action per frame: `go` (image point + depth), `rotate`, `think`, `finished` | Every control step | Step-wise | none | Mean success: best cloud model 64.4 %, Qwen3.5-27B 39.4 %, 9B 21.2 %, **4B 6.2 %**, 2B 0 %; a fine-tuned 2B 31.2 % (55 % on real Tellos); small models reach the target but fail to declare arrival; 0.6-3.5 s per step | cloud + Qwen3.5 + Cosmos 2B |
| **AERIS** ([arXiv 2606.30151](https://arxiv.org/abs/2606.30151)) | Schema-constrained decision (mode, waypoint, yaw, speed, confidence) | Every 2 s | Step-wise; if a decision is invalid or late the hub reuses the last valid one | Semantic loop (0.5 Hz) decoupled from a 50 Hz controller; three sizes per perception model | Cycle 1.31-1.63 s mean; AerialVLN success 20.5-25.8 % | several 0.5-3B SLMs on a ground station |
| **CognitiveDrone / RaceVLA** ([arXiv 2503.01378](https://arxiv.org/abs/2503.01378)) | 4D velocity commands (VLA) | 10 Hz, plus a 2 Hz VLM that rewrites the instruction | End to end | n/a | 59.6 % (77.2 % with the reasoner) vs RaceVLA 31.3 % on their gate benchmark | 7B OpenVLA |
| **UAV-VLA** ([arXiv 2501.05014](https://arxiv.org/abs/2501.05014)) / **UAV-CodeAgents** ([arXiv 2505.07236](https://arxiv.org/abs/2505.07236)) | Offline waypoint plan from satellite imagery; the latter a multi-agent ReAct loop on smolagents | Pre-flight only | n/a | n/a | UAV-VLA: 30 plans in 5 min 24 s, 6.5x faster than a human; CodeAgents: 96.96 s per mission, 93 % | GPT + Molmo; Qwen2.5-72B + fine-tuned Qwen2.5-VL-7B |
| **NeLV** ([arXiv 2510.21739](https://arxiv.org/abs/2510.21739)) | Start, end, POIs, constraints; classical planners do the rest | Once per instruction | n/a | LLM is only a parser | Qualitative | Phi-4-mini |
| **Swarm-GPT** ([arXiv 2312.01059](https://arxiv.org/abs/2312.01059)) / **SwarmGPT** ([arXiv 2412.08428](https://arxiv.org/abs/2412.08428)) / **FlockGPT** | Waypoints or motion primitives, then a safety filter / planner | Pre-flight, re-prompt to refine | n/a | n/a | Choreography; not mission agents | cloud |
| **NavGPT** ([arXiv 2305.16986](https://arxiv.org/abs/2305.16986)) | One navigation decision per step from text descriptions of views | Every step | Step-wise | none | Zero-shot R2R "still falling short of trained models" | GPT family |

### 2.2 Robot frameworks and behaviour trees

| Project | Output | Calls | Conditionals / replanning | Latency | Reported numbers | Model |
| --- | --- | --- | --- | --- | --- | --- |
| **Code as Policies** ([arXiv 2209.07753](https://arxiv.org/abs/2209.07753)) | Python policy code, recursively defining missing functions | 1 + recursion | In code | none | 39.8 % HumanEval | Codex-class |
| **ROSA** (NASA JPL; [arXiv 2410.06472](https://arxiv.org/abs/2410.06472)) | LangChain ReAct tool calls over ROS tools | Step-wise | Step-wise; blacklist decorator, parameter validation; no low-level safety by default | none | "a few seconds" per multi-tool query; no accuracy numbers; needs a tool-calling model with >=8192 context | GPT-4o, Claude 3.5, Llama 3.x |
| **ROS-LLM** ([arXiv 2406.19741](https://arxiv.org/abs/2406.19741)) | JSON action sequence, Python, or XML behaviour tree | Prompt updated each environment step; human feedback after each run | Reflection on human/environment feedback | none | Experiments use action sequences only; no latency | DeepSeek-Coder 7B |
| **BTGenBot** ([arXiv 2403.12761](https://arxiv.org/abs/2403.12761)) / **BTGenBot-2** ([arXiv 2602.01870](https://arxiv.org/abs/2602.01870)) | XML behaviour tree (BehaviorTree.CPP) | 1, plus subtree regeneration when local Fallback/Retry fails | BT Fallback / Condition nodes run without the model; an inference-time validator constrains output to a closed node vocabulary, a runtime validator requests a new subtree | none | BTGenBot-2 (Llama-3.2-1B, QLoRA on 5,204 pairs): 84.6 % zero-shot, 90.4 % with validators, vs 7B BTGenBot 57.7 %, GPT-5 Thinking 71.2 %; **5.1-11 s per tree** on a GTX 1080 | fine-tuned 1B |
| **LLM-BT** ([arXiv 2404.05134](https://arxiv.org/abs/2404.05134)) | ChatGPT task steps -> BERT parser -> behaviour tree; subtrees inserted on node failure | 1 + updates | In the tree | none | Simulation only | ChatGPT + BERT |

### 2.3 Small-model function calling

| Project | Mechanism | Numbers | What it needs |
| --- | --- | --- | --- |
| **Octopus v2** ([arXiv 2404.01744](https://arxiv.org/abs/2404.01744)) | One new vocabulary token per function (`<nexa_i>`), so picking a function is a single-token classification; no tool descriptions in the prompt; arguments follow in parentheses | 99.5 % on 20 Android APIs (98.1 % with 100 samples per API); 0.38 s per call vs 13.46 s for Llama-7B + RAG; "95 %" less context | Fine-tuning Gemma-2B with 100-1000 samples per function; new tokens |
| **TinyAgent / LLMCompiler** (Berkeley; [arXiv 2409.00608](https://arxiv.org/abs/2409.00608)) | Fine-tuned planner emits a whole plan with `$1` placeholders; Tool RAG (DeBERTa-v3-small multi-label classifier) picks the tools shown | Tool RAG: prompt 2762 -> 1397 tokens, success 78.9 -> 80.1 % (1.1B) and 83.1 -> 85.0 % (7B); latency 2.9 s (1.1B 4-bit), 13.1 s (7B 4-bit) vs GPT-4-Turbo 3.9 s at 79.1 % | 80k synthetic training examples for 16 functions |
| **Gorilla OpenFunctions v2** ([model card](https://huggingface.co/gorilla-llm/gorilla-openfunctions-v2)) | Emits a compact call string `f(a=1)` that is parsed into JSON afterwards | 83.67 % on BFCL per the card | 7B fine-tune |
| **Agentic Plan Caching** (Stanford, NeurIPS 2025; [arXiv 2506.14852](https://arxiv.org/abs/2506.14852)) | Store plan templates from finished runs, match new requests by keyword, let a small model fill the template | -50.31 % cost, -27.28 % latency, 96.61 % of baseline accuracy | A large planner to avoid and a small adapter model |
| **llama.cpp tool calling** ([function-calling.md](https://github.com/ggml-org/llama.cpp/blob/master/docs/function-calling.md), [PR #9639](https://github.com/ggml-org/llama.cpp/pull/9639), [PR #12034](https://github.com/ggml-org/llama.cpp/pull/12034), [PR #12379](https://github.com/ggml-org/llama.cpp/pull/12379)) | Native per-template formats; lazy grammar (unconstrained until a trigger, then grammar) for `tool_choice=auto`, full grammar from token 0 for `required`; `parallel_tool_calls` per request; streamed tool-call deltas with partial-JSON healing | Docs: generic format "may consume more tokens"; aggressive KV quantisation degrades tool calling | Our pinned build `1af554f` has all of it; `grammar` cannot be combined with `tools` (see the companion doc) |

### 2.4 Industry

Nothing public and technical was found. Primordial Labs' Anura (voice/natural-language tasking, shown on
Teal 2 and Skydio drones) and Scout AI's Fury (a "VLA" orchestrator that builds a mission plan from a
prompt, submits it for approval, then tasks each asset) are described only in press coverage, with no
architecture, model size or latency
([FlightGlobal](https://www.flightglobal.com/military-uavs/primordial-labs-demos-voice-command-technology-for-small-uavs/153490.article),
[Marine Corps Times](https://www.marinecorpstimes.com/news/your-marine-corps/2025/03/04/voice-controlled-drones-a-military-game-changer-primordial-labs-says/),
[GovConWire](https://www.govconwire.com/articles/scout-ai-100m-series-a-fury-ai)).
The Anduril-OpenAI partnership (announced 2024-12-04) is about counter-UAS data and models paired with
Lattice, not LLM tasking of an aircraft
([Inside Unmanned Systems](https://insideunmannedsystems.com/anduril-partners-with-openai-to-advance-cuas-ai/)).
For Shield AI Hivemind, Skydio and Auterion the searches returned no statement about LLM tasking at all.
The one pattern visible in the press material (plan, human approval, then execution) is what Icarus's
approval mode already does.

## 3. What the survey agrees on

1. **Output tokens are the latency.** TypeFly measured it and designed the whole system around it. On
   our box it is 80-90 % of a decision.
2. **Generate while moving.** Physical actions take longer than generating the next statement
   (TypeFly's stream interpreter; AERIS, REAL and CognitiveDrone decouple a slow semantic loop from a
   fast controller). Icarus generates while hovering.
3. **Loops and long sequences belong to the executor.** TypeFly's bounded loops, behaviour-tree
   Fallback nodes, and the MCP harness's server-owned mission (completed 37.8 min; model-supervised
   3 of 9). Icarus's composite skills are this idea. TypeFly's caveat applies too: too many high-level
   skills made GPT-4 write plans from high-level skills only, "often incorrect".
4. **Closed loop beats plan-once on quality.** SPINE's plan-once baseline used 1.4-1.7 queries and took
   over twice the time and distance; PEACE concedes open-loop brittleness; SPINE still asks for a
   multi-step plan each time but executes only its first action.
5. **Validator feedback is the cheapest robustness lever.** SPINE ("even minimal feedback significantly
   improved performance"), BTGenBot-2 (+6 points from validators), the MCP harness (53-69 % recovery
   after a rejected call).
6. **Premature or missing completion claims are a recognised small-model failure.** SPINE's only sim
   failure, DroneCATS' declaration gap (declaring arrival tracks instruction-following, Spearman 0.95),
   and an embodied study noting models "hallucinate success". Commit `c8ec8ec` (claims checked against
   receipts) is the right structural answer.
7. **Small models are reliable on a fixed tool set only after fine-tuning on it.** Stock vs tuned:
   9.2 -> 72.7 % (Llama-3.2-3B, SPINE), 0 -> 31.2 % (Cosmos 2B, DroneCATS), 41 -> 80 % (TinyAgent 1.1B vs
   an untuned 7B baseline), BTGenBot-2 1B above GPT-5. Nobody reports a zero-shot 4B model doing well.
8. **A 4B model should not be the low-level pilot.** Qwen3.5-4B scored 6.2 % choosing image points and
   yaw per frame (DroneCATS); 3-4B models managed 30-40 % with Turn/Move primitives. Skill-level tools
   are the right altitude for this model.

## 4. Ranked shortlist

Ranked by expected gain for the effort and risk on this hardware. Gains use section 1's numbers at
42 tok/s. Items 1-2 are speed, 3-4 mostly robustness, 5-6 are larger bets.

### 1. Executor-owned defaults: the model emits only what the operator specified

- Mechanism: design the call interface for output-token economy; unspecified parameters are omitted and
  filled by the executor, so a call is its name plus the operator's quantities.
- Demonstrated by: TypeFly (skills take 0-1 arguments, 2-char names, 6-47 token plans); Octopus v2 (a
  call is one token plus arguments).
- Gain for us: default-valued optional parameters are 13 % of output, a median ~57 tokens = **~1.4 s per
  mission**; concentrated in `search_for_object` (114 -> about 45 tokens, **-1.6 s on that decision**) and
  `survey_count`'s `shape`. Optional extension: have the executor compose the `finish` text from receipts and let
  the model give only the outcome plus any choice it made; that is another ~60 tokens (~1.4 s), though
  after the flying is over. Together 15-30 % of generation time, i.e. 2 s -> roughly 1.5-1.7 s on an
  average decision. Honest ceiling: under 1 % of mission wall time.
- Why the defaults are emitted today: every property is visible with its default and the system prompt
  says to "use the listed defaults for anything the operator did not specify", which the model satisfies
  by writing them out. The change is one generic sentence plus making sure the executor applies
  `operating_defaults()` for absent keys. Not mission-specific.
- Risk with a 4B model: low. If it still writes a default nothing breaks. Check that operator-given
  values ("counter-clockwise, 45 degree steps") are still passed.
- Size: ~20-50 lines and a replay of the suite.

### 2. Stream the response and start the first call while the next one is being generated

- Mechanism: when the next steps do not depend on what the first one returns, the model issues them in
  one response; the executor starts call 1 as soon as it is complete in the stream, queues the rest, and
  cuts the queue at the first non-SUCCEEDED result or the first observational tool
  (`search_for_object`, `survey_count`, `inspect_scene`, `detect`, `localize_target`, `ask_user`).
- Demonstrated by: TypeFly's stream interpreter (-62 % time to first action; -32 % from the DSL alone).
  PEACE and SPINE also emit several calls per response, but without streaming.
- Gain for us: 63 of 170 decisions (37 %) followed a succeeded non-observational step and were not
  `finish`: a median 2 per mission, each ~2-3 s of hover that would overlap with flight instead:
  **~4-6 s less hovering per mission**. Model time is unchanged apart from ~0.1-0.3 s of prefill per
  merged call. **Without streaming the gain is roughly zero**: the same tokens are generated, only
  earlier, and the drone waits longer before its first action.
- What exists: the request already sends `parallel_tool_calls: true`; `fly.py` takes `calls[0]` and
  discards the rest; the pinned llama.cpp streams tool-call deltas (PR #12379). Today's logs contain no
  multi-call response, because the system prompt says "one native tool call at a time", so its first
  line has to become a generic rule about independent follow-on steps.
- Risk with a 4B model: medium. It may chain past a branch point; the observational-tool cut-off handles
  that structurally, not by prompt. The history must record exactly the calls that ran; on this hybrid
  model the last assistant turn is re-evaluated anyway, so a truncated batch costs ~0.1 s. Approval mode
  needs a decision on whether a batch is approved as a whole.
- Size: ~150-250 lines (stream parsing in `llama_runtime.py`, a queue in `fly.py`, tests).

### 3. Failure results that state the permitted recoveries, enforced by the executor

- Mechanism: every non-SUCCEEDED result carries a machine-generated list of what may follow (retry the
  same call once, the specific fix, `ask_user`, `finish` incomplete) and `validate_call` rejects anything
  else, so recovery is a choice among stated options, not an invention.
- Demonstrated by: SPINE's validator (feedback names the offending item and suggests the alternative);
  BTGenBot-2 (84.6 -> 90.4 %); the MCP harness's server-side checks.
- Gain for us: not speed. 28 of 145 results today were non-success (19 %), and the open issues in the
  handover (invented motion after VIEWPOINT_REQUIRED, over-claiming after FAILED) are both wrong moves
  made right after such a result. Each avoided wrong recovery saves at least one decision (~2 s) and
  the flight it would have caused.
- Risk: low. The list comes from executor state and the reason code, not from mission wording; a
  rejected call costs one extra decision. `tool_choice` cannot name a function on this build, so the
  mask has to be post-hoc.
- Size: ~50-150 lines.

### 4. Bounded repetition inside skills

- Mechanism: "keep doing X until Y, at most N times" is a parameter of the skill and runs in the
  executor; the model is consulted when the bound is hit or the condition is met.
- Demonstrated by: TypeFly (`N{...}` with early return, replan when a motion budget is exceeded);
  behaviour-tree Retry/Fallback nodes; the MCP harness's server-owned mission.
- Gain for us: each repeated `search_for_object` or `survey_count` decision avoided is ~114 tokens =
  **~2.7 s**, and one fewer point where a 4B model can drop the loop. It applies to a minority of
  missions: 5 of 28 episodes today repeated one of those calls back to back. The same idea for a simple branch
  ("if seen do A else B" as one composite call) would save one ~2 s decision on conditional missions,
  but only 2 of 28 episodes used `inspect_scene` at all and it needs nested calls in the schema, so not yet.
- Risk: medium-low. Each new knob adds schema and output tokens (works against item 1), and TypeFly
  found that piling up high-level skills degraded plans. Add a bound only where logs show repeats.
- Size: small per skill.

### 5. Compact call syntax under a grammar (MiniSpec-style), replacing the native tool-call format

- Mechanism: the model writes `search_for_object("building")` under a GBNF grammar built from the
  catalogue instead of the Qwen3.5 XML call, removing most wrapper and per-parameter tokens.
- Demonstrated by: TypeFly/MiniSpec; Gorilla OpenFunctions v2 and Octopus v2 emit call strings.
- Gain for us (an estimate from the token fit, not a tokenizer measurement): about 14 tokens per call
  and ~10 per parameter, i.e. 40-55 % of output, **~0.9 s per decision, ~5 s per mission**; a median
  decision would go from ~2 s to ~1.2 s. TypeFly's own figure against Python was 34 %. This is the
  largest pure-latency lever left, and it is still ~2 % of mission wall time.
- Risk with a 4B model: high. It leaves the format the model was trained on; the catalogue becomes
  prose signatures; llama.cpp's tool parser, the `tools` grammar and the native message history are all
  replaced by our own (`grammar` cannot be sent with `tools`). TypeFly had GPT-4; no surveyed project
  shows a zero-shot small model writing a custom DSL reliably.
- Size: medium-large (grammar generation, parser, history rendering, full re-validation). Only worth
  trying behind the replay suite, and more sensibly after item 6.

### 6. LoRA-distil Qwen3.5-4B on its own tool set from replayed episodes

- Mechanism: fine-tune the resident model on (state, history) -> next call pairs from logged episodes,
  corrected by a stronger teacher, so tool choice and arguments are in-distribution.
- Demonstrated by: SPINE distilled (9.2 -> 72.7 %), TinyAgent, Octopus v2, BTGenBot-2, DroneCATS.
- Gain for us: robustness, the only lever the literature shows closing most of the gap between a small
  and a large model on a fixed API. No latency gain by itself; it is what would make item 5 safe and
  would let catalogue descriptions shrink.
- Risk: the result is tied to the catalogue (every tool change means re-tuning); SPINE's distilled
  model still failed multi-iteration planning because its data lacked long horizons; a LoRA ->
  GGUF path for the hybrid Qwen3.5 was not verified here; it needs a teacher and curated traces (394
  episodes exist, many with known-bad decisions).
- Size: large (days), and against "keep it simple". Do it only if decision quality is still the
  blocker after items 1-4.

## 5. Attractive, but would not help here

| Mechanism | Where it comes from | Why not for us |
| --- | --- | --- |
| Tool RAG / per-step tool pruning | TinyAgent, RAG-MCP | It shortens the prompt, which we already do not pay for (cached), and changing the catalogue per step destroys the cache on this hybrid model (the 15 s decisions of October). TinyAgent's own accuracy gain was 1-2 points. |
| Functional tokens | Octopus v2 | The "95 % context" saving is prompt we have cached; on the output side it replaces a 3-6 token function name (~0.1 s). Needs fine-tuning and new vocabulary. |
| Shrinking the catalogue for speed | general | Same reason: it affects only the one-off warm-up and KV memory. It may still help accuracy, which is a different argument. |
| Single-shot plan-then-execute | PEACE, ReWOO, LLMCompiler | The savings are network round trips, re-reading an uncached prompt, and parallel tool execution. We have none of those costs and one vehicle. The plan has the same tokens as the calls it replaces; without streaming (item 2) the drone only waits longer before moving, and open-loop plans were the slower, more brittle option in SPINE and PEACE. |
| Plan caching | Agentic Plan Caching | It avoids a large planner by letting a small model adapt a template; our planner is the small model. Matching vague mission text to stored plans is mission-specific behaviour by another route, and it would save at most the first decision. |
| Behaviour-tree generation | BTGenBot-2, SRDrone, LLM-BT | An XML tree is far more output than a tool call (5-11 s per tree for a tuned 1B model), needs fine-tuning, and its benefit (reactive execution without the model) is what composite skills already give. |
| Code generation | Code as Policies, ChatGPT for Robotics, GSCE | More tokens than a call, needs a sandbox and a human reviewing code, and a stronger coder than a 4B model. |
| End-to-end VLA / per-frame LLM piloting | CognitiveDrone, RaceVLA, DroneCATS | 7B at 10 Hz does not fit 6 GB beside the sim, and Qwen3.5-4B scored 6.2 % as a per-frame pilot. |
| Multi-agent or commander/sub-agent splits | UAV-CodeAgents, AERIS, DroneCATS commander mode | More model calls and more VRAM (97 s to create a mission in UAV-CodeAgents). |
| Speculating the next decision during a tool run | (own idea, checked against the cache rules) | The next prompt contains the real result and vehicle state, which cannot be predicted exactly; a wrong guess forces reprocessing. Item 2 gets the same overlap without guessing. |
| Lazy grammar | llama.cpp | It is for `tool_choice=auto`, where text may precede a call. We use `required`, so the grammar is active from the first token already. |

## 6. Not verified

- Latency was not reported by: SPINE (both papers), Taking Flight with Dialogue, PEACE, SRDrone, ROSA,
  ROS-LLM, AeroAgent, REAL, NeLV. Statements about their speed above are the authors' qualitative ones.
- PEACE names no model and reports no success rates; its "fewer LLM calls than ReAct" is a claim.
- MAVLink/MCP harness: about 200k of 255k characters were read. The identities of the local models were
  inconsistent between the two parts read (one named a 4B Gemma, the other did not), so only the
  aggregate local-arm findings are cited.
- DroneCATS, AERIS, BTGenBot-2 and PEACE are 2026 preprints; no peer-reviewed versions were checked.
- UAV-VLA, RaceVLA, FlockGPT, LLM-BT, NavGPT, GSCE and Gorilla OpenFunctions were read from abstracts,
  model cards or search summaries only. The GSCE ">90 %" figure is from a secondary summary. An IROS
  2025 paper "LLM-CBT" (closed-loop behaviour trees for UAV-UGV swarms) was found by title only.
- ROS-LLM: no Nature Machine Intelligence version was confirmed; only the arXiv preprint.
- TypeFly: the related-work section says "30 % fewer tokens" and the evaluation says 34 %; the
  introduction says "<1.5 s" while the table has one task at 1.60 s. No onboard or small-model follow-up
  to TypeFly was found.
- No controlled ReAct vs plan-then-execute comparison on robots with small models was found; the
  evidence used is SPINE's plan-once baseline and PEACE's own caveat.
- Industry: no vendor architecture, model size or latency figure was found for any product.
- The compact-syntax saving in item 5 comes from a linear fit over 170 logged calls (residual sd 18
  tokens), not from tokenising candidate syntaxes. The wall-time share uses `episode_start` to
  `episode_end`, which may include time that is not flight.
- Nothing in section 4 has been tried. Each needs the replay suite or a sim run before it is believed.

## Sources

- Icarus: `logs/episodes/20261106T*/events.jsonl` (28 episodes, 170 decisions), `python/dcm/fly.py`
  (`calls[0]`, "extra proposals are discarded"), `python/dcm/llama_runtime.py`
  (`tool_choice="required", parallel_tool_calls=True, "stream": False`), `third_party/llama.cpp` @ `1af554f`
  (`tools/server/server-common.cpp`, `common/chat-peg-parser.cpp`).
- UAV agents: https://arxiv.org/abs/2312.14950 ; https://typefly.github.io ; https://arxiv.org/abs/2306.17582 ;
  https://arxiv.org/abs/2502.12531 ; https://arxiv.org/abs/2311.15033 ; https://arxiv.org/abs/2311.01403 ;
  https://arxiv.org/abs/2410.03035 ; https://arxiv.org/abs/2505.09477 ; https://arxiv.org/abs/2506.07509 ;
  https://arxiv.org/abs/2606.00104 ; https://arxiv.org/abs/2508.15501 ; https://arxiv.org/abs/2601.15486 ;
  https://arxiv.org/abs/2609.01404 ; https://arxiv.org/abs/2606.30151 ; https://arxiv.org/abs/2503.01378 ;
  https://arxiv.org/abs/2501.05014 ; https://arxiv.org/abs/2505.07236 ; https://arxiv.org/abs/2510.21739 ;
  https://arxiv.org/abs/2312.01059 ; https://arxiv.org/abs/2412.08428 ; https://arxiv.org/abs/2305.16986
- Robot frameworks: https://arxiv.org/abs/2209.07753 ; https://arxiv.org/abs/2410.06472 ;
  https://arxiv.org/abs/2406.19741 ; https://arxiv.org/abs/2403.12761 ; https://arxiv.org/abs/2602.01870 ;
  https://arxiv.org/abs/2404.05134 ; https://arxiv.org/abs/2603.03148 (embodied agents: 30-150 s responses,
  hallucinated success)
- Small-model function calling: https://arxiv.org/abs/2404.01744 ; https://arxiv.org/abs/2409.00608 ;
  https://huggingface.co/squeeze-ai-lab/TinyAgent-1.1B ; https://huggingface.co/gorilla-llm/gorilla-openfunctions-v2 ;
  https://arxiv.org/abs/2506.14852
- llama.cpp: https://github.com/ggml-org/llama.cpp/blob/master/docs/function-calling.md ;
  https://github.com/ggml-org/llama.cpp/pull/9639 ; https://github.com/ggml-org/llama.cpp/pull/12034 ;
  https://github.com/ggml-org/llama.cpp/pull/12379
- Industry: https://www.flightglobal.com/military-uavs/primordial-labs-demos-voice-command-technology-for-small-uavs/153490.article ;
  https://www.marinecorpstimes.com/news/your-marine-corps/2025/03/04/voice-controlled-drones-a-military-game-changer-primordial-labs-says/ ;
  https://www.govconwire.com/articles/scout-ai-100m-series-a-fury-ai ;
  https://insideunmannedsystems.com/anduril-partners-with-openai-to-advance-cuas-ai/ ; https://shield.ai/hivemind/

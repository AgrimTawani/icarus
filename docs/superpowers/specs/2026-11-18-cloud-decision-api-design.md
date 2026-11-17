# Design: cloud decision API (and a Jev test), then a physical drone

Date: 2026-11-18. Branch `feat/cloud-decision-api`, cut from `feat/laya-decision-speed` at `38c742e`
(that branch is not merged into `main` yet; this work builds on its resident-vision and slot code).

## Intent

Today every model call runs on the laptop's GPU: Qwen3.5-4B in `llama-server` picks one tool per step, and the
same weights with the projector answer camera questions. The operator wants model calls to go to a hosted API
instead, chosen by environment variables (base URL, key, model), so that:

- the drone does not need GPU compute, which matters for Phase 2 on a commercial drone;
- larger hosted models can be used than a 6 GB laptop GPU holds;
- the operator's Sarvam credits (and free tiers such as Groq) pay for inference.

Second, measure whether Jev (TypeSafe AI's hosted typed-decision model) makes decisions faster or better.

Third (Phase 2), fly the same architecture on a commercial drone, with Icarus on a ground laptop.

## Goals

1. One config switch selects `cloud` or `local` for the mission model and the camera model. Cloud is the default.
2. Any OpenAI-compatible provider works by setting `ICARUS_LLM_BASE_URL`, `ICARUS_LLM_API_KEY` and
   `ICARUS_LLM_MODEL` (and the `ICARUS_VLM_*` equivalents, which default to the LLM values).
3. Cloud missions are no less correct than local ones on the existing Gazebo and AirSim mission suites, and
   their decision latency is measured and recorded the same way.
4. A Jev experiment answers, with numbers: does Jev answer our visual-condition and branch questions faster and
   at least as accurately as the cloud LLM/VLM?
5. Phase 2 has a written outline with its decision point (the drone) and the work each answer implies.

## Non-goals

- No new agent framework. The mission loop (`python/dcm/fly.py`) stays as it is.
- No change to flight authority: the C++ guardrails, safety supervisor and ArduPilot keep final say.
- YOLO (`detect`) stays local. It is a small detector, not a language model.
- No prompt edits that hardcode mission behaviour.

## Anti-goals (failure even if it "works")

- A network blip, rate limit or slow reply leaves the drone doing anything other than holding safely.
- An API key lands in a log, an episode file, a commit or a URL.
- The local path is deleted, so a flight without internet becomes impossible or old episodes can no longer be
  compared.
- Jev is wired in to choose flight actions. It may only answer typed questions whose "unsure" maps to "keep
  looking" or "ask the LLM".

## Constraints found in the code and the providers

- **Mission model**: `LlamaCppRuntime.decide` already POSTs OpenAI-style `/v1/chat/completions` with `tools`,
  `tool_choice: "required"` and `parallel_tool_calls`. The loop needs `decide(messages, tools, concise=False)`,
  `last_stats`, `stop()`, optional `warm(tools)` and `vision()`, and two exception types
  (`ContextBudgetExceeded`, `IncompleteModelResponse`). llama-only fields (`cache_prompt`, `id_slot`,
  `chat_template_kwargs`, `reasoning_budget_tokens`, `top_k`, `min_p`, `repeat_penalty`) must be dropped. The
  pre-flight token count endpoint is llama-only.
- **Tool schemas** use `oneOf` and `const` (`python/dcm/agent_tools.py`). Some providers reject these; the
  provider check must test the real catalogue.
- **Camera model**: every camera question uses a llama.cpp GBNF grammar (`python/perception/vlm_grammars.json`)
  through llama's native `/completion`. The grammars are derived from JSON schemas in `python/perception/vlm.py`
  (`CONDITION_SCHEMA`, `DESCRIPTION_SCHEMA`, `PIXEL_BOX_SCHEMA`, `PIXEL_BOXES_SCHEMA`), so the cloud path can send
  the schema as `response_format: json_schema` instead. Decoders already treat malformed answers as unknown.
- **Images**: Gazebo frames are raw PPM (848x480, about 1.2 MB). Cloud APIs take JPEG/PNG/WebP as a data URL.
- **Box coordinates**: the local path declares `grounding_format: json_norm1000_xyxy`. Each cloud vision model's
  convention must be declared and checked on labelled frames before `localize_target` and `survey_count` use it.
- **Failure handling**: any model error ends the mission with a Hold (`fly.py`, `hold_on_error`). There is no
  retry, so a 429 rate limit ends a mission today. The 60 s decision deadline (`deadline_ms`) stays.
- **Provider limits**: each decision sends about 8,400 prompt tokens (the fixed tool catalogue is about 6,800).
  Groq's free tier allows on the order of 8k-70k tokens per minute per model, which is one to eight decisions a
  minute, and its free models may not accept images. Sarvam Token Factory (`https://api.sarvam.ai/v2`) is
  OpenAI-compatible, supports tool calling and structured output on every model, and lists image input on
  Gemma 4 31B and GLM-5.3 Flash. So Sarvam is the planned flight provider; Groq is a text-only test target.
- **Jev**: hosted only, `POST https://api.typesafe.ai/v1/systemone`, questions of type choice / score / noul
  (and an experimental box), image input experimental behind a header, claimed about 100 ms from US West.
  Signups were paused on 2026-09-22. It is the same question model Laya implements; Laya's zero-shot image
  answers were unusable on our frames (`docs/research/LAYA-FINDINGS.md`).
- **Physical**: the MAVLink gateway connects over TCP to ArduPilot only; camera tools capture only from Gazebo or
  AirSim; `hardware-bench` / `hardware-flight` launch profiles are reserved and unavailable. A telemetry radio
  cannot carry camera video.

## Approaches considered

1. **Cloud default with the local model kept as an offline fallback (chosen).** One `provider` switch in the
   runtime manifest and `dcm-fly --runtime cloud|local`. The local runtime stays untouched. Costs: two runtimes
   to keep working. Gains: flights without internet, and a direct comparison against the 2 s local baseline.
2. **Cloud only, delete the local path.** Less code, but no offline flight, no baseline and a large deletion
   that would have to be undone if a provider fails.
3. **A gateway library (LiteLLM or similar) in front of every provider.** Adds a dependency and a layer for
   something the OpenAI wire format already gives us; no feature here needs it.

The operator stated the provider choice is just environment variables; approach 1 was my pick, taken because the
operator did not answer the fallback question and the plan had to move. Reversing it later means deleting the
local runtime, which stays easy.

## Design

### Runtime selection

- `config/dcm/agent-runtime.json` gains a `cloud` block: `provider_label`, `context_length`, `deadline_ms`,
  `max_retries`, `vision` (with `grounding_format`, `image_max_side_px`, `jpeg_quality`,
  `minimum_request_interval_s`). Secrets never go in the manifest.
- `RuntimeDescriptor` gets a cloud variant that records provider label, base URL host and model id (no key) in
  the episode's provenance record.
- `fly_cli.build_runtime` picks `CloudRuntime` or `LlamaCppRuntime` from `--runtime` (default `cloud`) and the
  environment; it refuses to start cloud mode without a key and prints which provider/model it will use.
  The same switch is added where the other CLIs construct `LlamaCppRuntime` (`evaluate_cli.py`,
  `observe_cli.py`, `evaluate_operator_missions.py`, `replay_decision_latency.py`).

### `CloudRuntime` (`python/model_runtime/cloud.py`)

- `decide(messages, tools, concise=False)`: one OpenAI chat-completions call with `tools`, `tool_choice:
  "required"`, `temperature: 0`, `max_tokens`. `parallel_tool_calls` is not sent: the loop executes only the
  first call, and some providers reject the field.
  Runs on a worker thread joined at the deadline, as today.
- Error mapping: context-length errors become `ContextBudgetExceeded` (so the loop drops the oldest exchange);
  `finish_reason == "length"` and tool-format failures (e.g. Groq `tool_use_failed`) become
  `IncompleteModelResponse`; HTTP 429/5xx and network errors retry with backoff (honouring `Retry-After`) up to
  `max_retries`, but never past the deadline; anything else raises, and the loop holds the drone.
- `last_stats`: `tokens_prefilled`, `tokens_cached` (from `usage.prompt_tokens_details.cached_tokens` when
  present), `tokens_predicted`, `latency_ms`, `retries`, provider/model. Fields the latency scripts read keep
  their names.
- `start()`, `warm()`, `stop()` are cheap: `stop()` abandons an in-flight request.
- The exceptions move to `python/dcm/runtime_config.py` so both runtimes raise the same classes.

### Cloud camera model (`CloudVision`, same module)

- Implements the existing resident-vision interface: `artifact` dict and
  `complete(image_b64, question, prefill, n_predict, *, grammar=None, timeout_s=45)`.
- Translates the grammar back to its JSON schema by its entry in `vlm_grammars.json` and sends
  `response_format: {"type": "json_schema", ...}`; falls back to `json_object` plus the schema in the prompt if
  the provider rejects strict schemas.
- Converts PPM to JPEG (Pillow, already in the venv), downscales to `image_max_side_px`, and sends it as an
  `image_url` data URL.
- `fly.attach_vision` and `run_mission._vision_lease` already prefer a resident vision object, so no change is
  needed there beyond passing `CloudVision` from `runtime.vision()`.

### Measurement

- `scripts/autonomy/episode_latency.py` keeps working from `last_stats`.
- `replay_decision_latency.py` gains `--runtime cloud` to replay recorded decisions against a provider and report
  agreement with the recorded tool and latency, as it does today for prompt edits.
- `scripts/autonomy/check_cloud_provider.py` (new): one command that verifies a provider before flying: accepts
  the real tool catalogue, returns a valid tool call, answers a condition and a box question on two stored
  frames, and prints latency and token use.

### Jev experiment (only if a key exists)

- Offline first, no flight code: a script runs Jev and the cloud VLM over the 24 labelled frames in
  `docs/research/vision-bench/labels.json`, asking the same "is X visible" questions (as a two-way `choice`,
  which was the only form Laya read correctly), and over recorded decision states, asking "which tool next" as a
  `choice` over the catalogue. Report accuracy, false positives, latency from this laptop and cost.
- Only if Jev beats the cloud VLM on accuracy with zero false positives: a flag `condition_checker: jev` routes
  `inspect_scene` condition questions to Jev, with "unsure" (probability between 0.3 and 0.7) mapped to
  `found: null`, which `search_for_object` already treats as "keep turning". Then fly the find-and-orbit
  missions with and without it.
- If no key is available, the experiment is recorded as blocked; Cloudflare's open-weight Clef (same request
  format) through a hosted endpoint is the substitute to evaluate.

### Phase 2 outline (physical drone, ground laptop)

Decision point: the drone. What each answer implies:

- **ArduPilot or PX4 with MAVLink exposed** (most open commercial frames): add serial/UDP transport to the
  MAVLink gateway (or put mavlink-router in front of it), add a `hardware-flight` profile with real limits and
  geofence, and add PX4 mode mapping if it runs PX4.
- **Closed vendor SDK**: a new gateway that speaks the SDK, behind the same Drone API, is a project of its own.
- **Either way**: a video downlink the laptop can read (digital FPV/HDMI capture or the vendor's stream), a
  camera adapter that replaces the Gazebo/AirSim capture in `run_mission.py`, camera calibration for that
  camera, and an independent 4G/Wi-Fi link to the cloud on the laptop. Bench tests (props off), tethered hover,
  then the simplest mission. Physical flight needs the operator's local regulatory approval; nothing in this
  repo authorises it.

## Left to the builder

- Exact provider model ids, by running `check_cloud_provider.py` on Sarvam and Groq.
- Retry counts and backoff within the deadline.
- JPEG size and quality, chosen from the accuracy check on labelled frames.

## Calls I made (operator to check)

- Cloud is the default; local stays as the offline fallback.
- Sarvam is the planned flight provider, Groq a text-only test target, because of free-tier token limits.
- Jev is tested offline first and never chooses flight actions.
- The branch builds on the unmerged `feat/laya-decision-speed`.

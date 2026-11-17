# Cloud Decision API Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Mission decisions and camera questions go to any OpenAI-compatible hosted API chosen by environment variables, with the local llama.cpp model kept as an offline fallback, and a measured verdict on Jev.

**Architecture:** A new `CloudRuntime` and `CloudVision` in `python/model_runtime/cloud.py` implement the same duck-typed interface the mission loop already uses for `LlamaCppRuntime` and `ResidentVision`. `dcm-fly --runtime cloud|local` picks one (default `cloud`). Nothing in `python/dcm/fly.py` or the C++ stack changes behaviour.

**Tech Stack:** Python 3 stdlib `urllib` (as `llama_runtime.py` uses), Pillow 12.3 (already in `.venv`), pytest, OpenAI chat-completions wire format.

**Spec:** `docs/superpowers/specs/2026-11-18-cloud-decision-api-design.md`

## Global Constraints

- Environment variables, exactly: `ICARUS_LLM_BASE_URL`, `ICARUS_LLM_API_KEY`, `ICARUS_LLM_MODEL`; `ICARUS_VLM_BASE_URL`, `ICARUS_VLM_API_KEY`, `ICARUS_VLM_MODEL` each default to the LLM value; optional `ICARUS_VLM_GROUNDING_FORMAT` overrides the manifest's.
- No API key in the manifest, logs, episode events, `last_stats`, `as_record()`, exception messages, `repr()` or commits.
- `dcm-fly` defaults to `--runtime cloud`; `--runtime local` runs today's `LlamaCppRuntime` unchanged.
- Decision deadline stays `deadline_ms` = 60000 from the manifest; `--timeout-ms` still overrides it.
- `temperature` 0, `tool_choice` `"required"`; `parallel_tool_calls` is not sent (the loop executes only the first call).
- Any model failure that is not a context overflow or truncation still raises out of `decide`, so `fly.py` holds the drone (`hold_on_error`).
- YOLO `detect` stays local and untouched.
- No edits to `config/dcm/prompts/system.md` or tool descriptions.
- Unit tests: `.venv/bin/python -m pytest -q tests/unit` (1163 passed on `38c742e`); every task leaves it green.
- Commit messages end with `Co-Authored-By: Claude Opus 5.5 <noreply@anthropic.com>`.

## Review Focus

1. **Key leakage through an error path:** a provider 401/400 whose body or our exception message includes the `Authorization` header or the key must not leak it. Test in Task 1 (`repr`, `as_record`) and Task 2 (error messages, `last_stats`).
2. **Rate limit longer than the deadline:** a 429 with `Retry-After: 120` and 60 s left must raise `DeadlineExceeded` without sleeping past the deadline. Test in Task 2.
3. **Operator interrupt during a slow cloud call:** `stop()` must return at once and a reply arriving after it must be discarded, never executed. Test in Task 2.
4. **Downscaled image with pixel-coordinate boxes:** boxes in `json_pixel_xyxy` would refer to the shrunken image. `CloudVision` must not resize when the format is `json_pixel_xyxy`. Test in Task 3.
5. **No internet at start:** `dcm-fly` in cloud mode with an unreachable host must fail before arming, naming the host and suggesting `--runtime local`. Test in Task 2 (`start`) and Task 4 (CLI).

---

## File Structure

- Create `python/model_runtime/__init__.py` (empty) and `python/model_runtime/cloud_config.py`: settings from manifest + environment, no network.
- Create `python/model_runtime/cloud.py`: `CloudRuntime` (decisions) and `CloudVision` (camera questions), one HTTP helper shared by both.
- Modify `python/dcm/runtime_config.py`: home of `IncompleteModelResponse` (moved from `llama_runtime.py`, re-exported there).
- Modify `config/dcm/agent-runtime.json`: add the `cloud` block.
- Modify `python/dcm/fly_cli.py`: `--runtime cloud|local`, `build_runtime`.
- Modify `python/dcm/evaluate_cli.py`, `python/dcm/observe_cli.py`, `scripts/autonomy/evaluate_operator_missions.py`, `scripts/autonomy/replay_decision_latency.py`: same switch via one helper.
- Create `scripts/autonomy/check_cloud_provider.py`, `scripts/autonomy/bench_jev.py`.
- Tests: `tests/unit/test_cloud_config.py`, `tests/unit/test_cloud_runtime.py`, `tests/unit/test_cloud_vision.py`, `tests/unit/test_fly_cli_runtime.py`, `tests/unit/test_bench_jev.py`.

---

### Task 1: Cloud settings from the manifest and environment

**Files:**
- Create: `python/model_runtime/__init__.py`, `python/model_runtime/cloud_config.py`
- Modify: `config/dcm/agent-runtime.json`
- Test: `tests/unit/test_cloud_config.py`

**Interfaces:**
- Produces: `CloudSettings` (frozen dataclass) with fields `base_url: str`, `api_key: str`, `model: str`, `vision_base_url: str`, `vision_api_key: str`, `vision_model: str`, `context_length: int`, `deadline_ms: int`, `max_retries: int`, `vision: dict` (keys `grounding_format`, `image_max_side_px`, `jpeg_quality`, `minimum_request_interval_s`); `CloudSettings.from_manifest(path: Path, environ: Mapping[str, str] = os.environ, **overrides) -> CloudSettings`; `as_record() -> dict` (provider host, model ids, limits; no keys); `MissingCloudSetting(ValueError)`.

- [ ] **Step 1: Write the failing tests**

```python
def test_reads_env_and_defaults_vision_to_llm(tmp_path):
    s = CloudSettings.from_manifest(manifest(tmp_path), environ={
        "ICARUS_LLM_BASE_URL": "https://api.sarvam.ai/v2", "ICARUS_LLM_API_KEY": "sk-secret",
        "ICARUS_LLM_MODEL": "gemma4"})
    assert (s.vision_base_url, s.vision_api_key, s.vision_model) == ("https://api.sarvam.ai/v2", "sk-secret", "gemma4")
    assert s.deadline_ms == 60000 and s.context_length == 16384 and s.max_retries == 3
    assert s.vision["grounding_format"] == "json_norm1000_xyxy"

def test_vision_env_overrides(tmp_path): ...  # ICARUS_VLM_MODEL / ICARUS_VLM_GROUNDING_FORMAT win

def test_missing_key_names_the_variable(tmp_path):
    with pytest.raises(MissingCloudSetting, match="ICARUS_LLM_API_KEY"):
        CloudSettings.from_manifest(manifest(tmp_path), environ={"ICARUS_LLM_BASE_URL": "https://x/v1", "ICARUS_LLM_MODEL": "m"})

def test_key_never_appears_in_repr_or_record(tmp_path):
    s = settings_with_key("sk-secret")
    assert "sk-secret" not in repr(s) and "sk-secret" not in json.dumps(s.as_record())
    assert s.as_record()["provider_host"] == "api.sarvam.ai"

def test_deadline_override(tmp_path):
    assert settings(tmp_path, deadline_ms=5000).deadline_ms == 5000
```

- [ ] **Step 2: Run to verify they fail**

Run: `.venv/bin/python -m pytest -q tests/unit/test_cloud_config.py`
Expected: FAIL, `ModuleNotFoundError: python.model_runtime.cloud_config`

- [ ] **Step 3: Add the manifest block** to `config/dcm/agent-runtime.json` (top level, beside `artifacts`):

```json
"cloud": {"context_length": 16384, "deadline_ms": 60000, "max_retries": 3,
          "vision": {"grounding_format": "json_norm1000_xyxy", "image_max_side_px": 1024,
                     "jpeg_quality": 85, "minimum_request_interval_s": 0.5}}
```

- [ ] **Step 4: Implement `CloudSettings`** in `python/model_runtime/cloud_config.py`; mark `api_key` and `vision_api_key` with `field(repr=False)`; `base_url` has any trailing `/` stripped.

- [ ] **Step 5: Run tests, then the whole unit suite**

Run: `.venv/bin/python -m pytest -q tests/unit/test_cloud_config.py && .venv/bin/python -m pytest -q tests/unit`
Expected: PASS; suite count rises by the new tests, no failures (the manifest test `test_runtime_manifest_settings.py` must still pass with the extra block).

- [ ] **Step 6: Commit**

```bash
git add python/model_runtime config/dcm/agent-runtime.json tests/unit/test_cloud_config.py
git commit -m "Read cloud model settings from the manifest and environment, never recording the key"
```

---

### Task 2: `CloudRuntime` for mission decisions

**Files:**
- Create: `python/model_runtime/cloud.py`
- Modify: `python/dcm/runtime_config.py` (add `IncompleteModelResponse`), `python/dcm/llama_runtime.py` (import it from `runtime_config` instead of defining it), `python/dcm/fly.py:227` (import from `runtime_config`)
- Test: `tests/unit/test_cloud_runtime.py`

**Interfaces:**
- Consumes: `CloudSettings` (Task 1); `DeadlineExceeded`, `ContextBudgetExceeded`, `IncompleteModelResponse` from `python/dcm/runtime_config.py`.
- Produces: `CloudRuntime(settings: CloudSettings)` with `start() -> None` (GET `{base_url}/models`, 5 s timeout; on failure `RuntimeError` naming the host and saying "use --runtime local to fly offline"), `warm(tools) -> None` (calls `start()`), `decide(messages: list[dict], tools: list[dict], *, concise: bool = False) -> dict` (the assistant message: `content` and/or `tool_calls`), `stop() -> None`, `vision() -> CloudVision` (Task 3), attribute `last_stats: dict | None`, attribute `descriptor_record: dict` (= `settings.as_record()`).
- Produces: private `_post_json(url, key, body, deadline_at, max_retries) -> dict` reused by `CloudVision`.

- [ ] **Step 1: Write the failing tests** (patch `python.model_runtime.cloud.urllib.request.urlopen`):

```python
def test_request_body_is_plain_openai(fake_http):
    rt = CloudRuntime(settings()); rt.decide([{"role": "user", "content": "go"}], TOOLS)
    body = fake_http.last_json
    assert body["model"] == "gemma4" and body["tool_choice"] == "required" and body["temperature"] == 0
    for llama_only in ("cache_prompt", "id_slot", "chat_template_kwargs", "reasoning_budget_tokens",
                       "top_k", "min_p", "repeat_penalty", "parallel_tool_calls"):
        assert llama_only not in body
    assert fake_http.last_headers["Authorization"] == "Bearer sk-secret"

def test_returns_message_and_stats(fake_http):  # usage.prompt_tokens 8400, cached 8000, completion 90, completion_time 0.3
    msg = rt.decide(...)
    assert msg["tool_calls"][0]["function"]["name"] == "takeoff"
    assert rt.last_stats == {"tokens_prompt": 8400, "tokens_cached": 8000, "tokens_prefilled": 400,
                             "tokens_predicted": 90, "predicted_per_second": 300.0, "retries": 0,
                             "provider_host": "api.sarvam.ai", "model": "gemma4"}

def test_context_overflow_becomes_context_budget_exceeded(fake_http):  # 400 body contains "context_length_exceeded"
def test_length_finish_and_tool_use_failed_become_incomplete(fake_http):  # finish_reason "length"; 400 "tool_use_failed"
def test_429_retries_then_succeeds(fake_http):  # 429 Retry-After 1, then 200 -> last_stats["retries"] == 1
def test_retry_after_past_deadline_raises_deadline_without_sleeping(fake_http, fake_clock):
    # deadline_ms 2000, 429 Retry-After 120 -> DeadlineExceeded, total sleep < 2 s
def test_auth_error_raises_runtimeerror_without_key(fake_http):  # 401 -> RuntimeError; "sk-secret" not in str(error)
def test_stop_abandons_inflight_and_discards_late_reply(slow_http):
    # decide on a thread; stop() returns in < 0.2 s; decide raises DeadlineExceeded or InterruptedError; no message returned
def test_start_unreachable_names_host_and_local_fallback(fake_http):  # URLError -> "api.sarvam.ai" and "--runtime local" in message
def test_content_only_reply_is_returned_without_tool_calls(fake_http)  # fly.py turns it into awaiting_user
```

- [ ] **Step 2: Run to verify they fail**

Run: `.venv/bin/python -m pytest -q tests/unit/test_cloud_runtime.py`
Expected: FAIL, `ModuleNotFoundError`

- [ ] **Step 3: Move `IncompleteModelResponse`** to `python/dcm/runtime_config.py`; in `llama_runtime.py` replace the class with `from python.dcm.runtime_config import IncompleteModelResponse`; in `fly.py:227` import it from `runtime_config`.

- [ ] **Step 4: Implement `CloudRuntime`** in `python/model_runtime/cloud.py`. The HTTP call runs on a daemon worker thread joined until the deadline, as `LlamaCppRuntime._chat_owned` does; `stop()` sets an abandoned flag so a late reply is dropped. Retries: on 429/500/502/503/504 or `URLError`, sleep `min(Retry-After or 2**attempt, time left)`; if the wait would pass the deadline raise `DeadlineExceeded`. Error-body classification for `ContextBudgetExceeded`: any of `context_length_exceeded`, `maximum context length`, `too many tokens`, `context window` (case-insensitive). For `IncompleteModelResponse`: `tool_use_failed`, `failed to call a function`, or `finish_reason == "length"`. `concise=True` adds the same short "reply with one tool call only" system nudge `LlamaCppRuntime` uses (copy its text from `llama_runtime.py`). Exception messages include status code and provider host only, never headers or the request body.

- [ ] **Step 5: Run tests, then the suite**

Run: `.venv/bin/python -m pytest -q tests/unit/test_cloud_runtime.py && .venv/bin/python -m pytest -q tests/unit`
Expected: PASS, no regressions (`test_llama_runtime.py` and `test_mission_loop.py` still import `IncompleteModelResponse`).

- [ ] **Step 6: Commit**

```bash
git add python/model_runtime/cloud.py python/dcm/runtime_config.py python/dcm/llama_runtime.py python/dcm/fly.py tests/unit/test_cloud_runtime.py
git commit -m "Add a cloud runtime for mission decisions with deadline-bounded retries"
```

---

### Task 3: `CloudVision` for camera questions

**Files:**
- Modify: `python/model_runtime/cloud.py`
- Test: `tests/unit/test_cloud_vision.py`

**Interfaces:**
- Consumes: `_post_json` and `CloudRuntime` (Task 2); `measurement_grammar(result_type)` and the schema constants in `python/perception/vlm.py`.
- Produces: `CloudVision(runtime: CloudRuntime)` with `artifact: dict` (`{"family": "cloud", "model": ..., "provider_host": ..., "resident": True, "grounding_format": ..., "minimum_request_interval_s": ...}`) and `complete(image_b64: str, question: str, prefill: str, n_predict: int, *, grammar: str | None = None, timeout_s: float = 45.0) -> str` (the model's text answer). `CloudRuntime.vision()` returns one shared instance.

- [ ] **Step 1: Write the failing tests**

```python
def test_grammar_maps_back_to_its_schema(fake_http):
    vision.complete(PPM_B64, "Is a house visible?", "", 160, grammar=measurement_grammar("boolean"))
    rf = fake_http.last_json["response_format"]
    assert rf["type"] == "json_schema" and rf["json_schema"]["schema"] == CONDITION_SCHEMA

def test_unknown_grammar_raises_valueerror()  # a grammar not produced by measurement_grammar
def test_ppm_is_sent_as_jpeg_data_url_and_downscaled(fake_http):
    # 848x480 PPM, image_max_side_px 512 -> image_url starts "data:image/jpeg;base64,", decoded size (512, 290)
def test_pixel_format_is_never_resized(fake_http):
    # grounding_format json_pixel_xyxy, image_max_side_px 512 -> decoded size stays (848, 480)
def test_strict_schema_rejected_falls_back_to_json_object(fake_http):
    # first 400 mentions "response_format"/"json_schema" -> second request has {"type": "json_object"} and the schema text in the prompt
def test_prefill_is_prepended_to_question_not_sent_as_assistant(fake_http)  # resident callers pass "" today; non-empty prefill goes at the end of the user text
def test_resident_callers_work_unchanged(fake_http):
    # python.perception.vlm.ground_all(path, "house", manifest, server=vision) returns boxes from a canned {"bboxes": [[100,100,200,200]]}
def test_timeout_raises_timeouterror(slow_http)
```

- [ ] **Step 2: Run to verify they fail**

Run: `.venv/bin/python -m pytest -q tests/unit/test_cloud_vision.py`
Expected: FAIL, `ImportError: cannot import name 'CloudVision'`

- [ ] **Step 3: Implement `CloudVision`.** Build `{measurement_grammar(t): schema}` once for the five result types (`boolean`, `count`, `description`, `pixel_box`, `pixel_boxes`). Decode the image with Pillow from the base64 bytes, convert to RGB, `thumbnail((side, side))` only when `grounding_format != "json_pixel_xyxy"`, save as JPEG at `jpeg_quality`. Send one user message with a text part and an `image_url` part, `max_tokens = n_predict`, `temperature` 0. Reuse the vision endpoint, key and model from settings. Return `choices[0].message.content` (empty string if missing).

- [ ] **Step 4: Run tests, then the suite**

Run: `.venv/bin/python -m pytest -q tests/unit/test_cloud_vision.py && .venv/bin/python -m pytest -q tests/unit`
Expected: PASS

- [ ] **Step 5: Commit**

```bash
git add python/model_runtime/cloud.py tests/unit/test_cloud_vision.py
git commit -m "Answer camera questions through the cloud model with the same answer schemas"
```

---

### Task 4: Pick the runtime in the CLIs

**Files:**
- Modify: `python/dcm/fly_cli.py:44-67`, `python/dcm/evaluate_cli.py:73`, `python/dcm/observe_cli.py:21`, `scripts/autonomy/evaluate_operator_missions.py:25`
- Test: `tests/unit/test_fly_cli_runtime.py`

**Interfaces:**
- Consumes: `CloudSettings.from_manifest` (Task 1), `CloudRuntime` (Task 2).
- Produces: `fly_cli.build_runtime(args) -> tuple[runtime, record]` where `args.runtime in ("cloud", "local")`; for `local` it returns today's `(LlamaCppRuntime(descriptor), descriptor)` unchanged; for `cloud` it returns `(CloudRuntime(settings), settings.as_record())`. Shared helper `python/model_runtime/select.py: make_runtime(kind: str, manifest: Path, **overrides)` used by all four CLIs.

- [ ] **Step 1: Write the failing tests**

```python
def test_default_runtime_is_cloud(monkeypatch):
    args = fly_cli.parse_args([])  # factor argument parsing into parse_args(argv) if it is not already
    assert args.runtime == "cloud"

def test_local_runtime_is_unchanged(monkeypatch):
    runtime, descriptor = fly_cli.build_runtime(fly_cli.parse_args(["--runtime", "local"]))
    assert type(runtime).__name__ == "LlamaCppRuntime"

def test_cloud_without_key_exits_naming_variable(monkeypatch, capsys):
    monkeypatch.delenv("ICARUS_LLM_API_KEY", raising=False)
    with pytest.raises(SystemExit): fly_cli.build_runtime(fly_cli.parse_args([]))
    assert "ICARUS_LLM_API_KEY" in capsys.readouterr().err

def test_cloud_banner_shows_host_and_model_not_key(monkeypatch, capsys)  # "api.sarvam.ai" and "gemma4" printed, "sk-secret" absent
```

- [ ] **Step 2: Run to verify they fail**

Run: `.venv/bin/python -m pytest -q tests/unit/test_fly_cli_runtime.py`
Expected: FAIL

- [ ] **Step 3: Implement** `make_runtime` and the `--runtime {cloud,local}` argument (visible in `--help`, default `cloud`); keep `llama` as an accepted alias for `local`. The startup banner that prints the model identity prints provider host and model for cloud. `--timeout-ms` and `--context-length` map onto `CloudSettings` overrides.

- [ ] **Step 4: Run tests, then the suite**

Run: `.venv/bin/python -m pytest -q tests/unit/test_fly_cli_runtime.py && .venv/bin/python -m pytest -q tests/unit`
Expected: PASS

- [ ] **Step 5: Commit**

```bash
git add python/model_runtime/select.py python/dcm/fly_cli.py python/dcm/evaluate_cli.py python/dcm/observe_cli.py scripts/autonomy/evaluate_operator_missions.py tests/unit/test_fly_cli_runtime.py
git commit -m "Default dcm-fly to the cloud runtime and keep --runtime local for offline flights"
```

---

### Task 5: Provider check and cloud replay

**Files:**
- Create: `scripts/autonomy/check_cloud_provider.py`
- Modify: `scripts/autonomy/replay_decision_latency.py`
- Test: `tests/unit/test_cloud_runtime.py` (add the checker's pure functions)

**Interfaces:**
- Consumes: `make_runtime` (Task 4), `decision_tools` and `system_prompt` from `python/dcm/agent_tools.py`, two stored frames from `output/laya-speed-20261106/vision-bench/` named in `docs/research/vision-bench/labels.json`.
- Produces: `check_cloud_provider.py [--manifest PATH]`, exit 0 only when all four checks pass, printing one line per check with latency and tokens: (1) the real catalogue is accepted and a takeoff request returns a valid `takeoff` call; (2) `validate_call` accepts it; (3) a condition question on a building frame and on a road frame returns `found` true and false; (4) a `pixel_box` question returns a box inside the image. `replay_decision_latency.py --runtime cloud` replays recorded `agent_decision` requests through `CloudRuntime.decide` and reports the same agreement fields as today (`same_tool_as_recorded`, `same_call_as_recorded`, `gen_tokens`).

- [ ] **Step 1: Write the failing test** for the check summariser:

```python
def test_check_summary_fails_when_any_check_fails():
    assert check_cloud_provider.summarise([("catalogue", True, 900), ("box", False, 1500)]) == (1, "FAIL box")
```

- [ ] **Step 2: Run to verify it fails**, then **implement** both scripts (the replay keeps its llama path for `--runtime local`, its default stays `local` so old commands still work).

- [ ] **Step 3: Run tests, then the suite**

Run: `.venv/bin/python -m pytest -q tests/unit`
Expected: PASS

- [ ] **Step 4: Commit**

```bash
git add scripts/autonomy/check_cloud_provider.py scripts/autonomy/replay_decision_latency.py tests/unit/test_cloud_runtime.py
git commit -m "Add a provider check and cloud replay of recorded decisions"
```

---

### Task 6: Provider acceptance and sim comparison (needs keys)

No code unless a check fails. Needs `ICARUS_LLM_*` set for Sarvam (and a Groq key for the text-only check). Laptop on AC power.

- [ ] **Step 1: Run the provider check on Sarvam with a vision model**

Run: `ICARUS_LLM_BASE_URL=https://api.sarvam.ai/v2 ICARUS_LLM_MODEL=<gemma4 id from the Sarvam models page> .venv/bin/python scripts/autonomy/check_cloud_provider.py`
Expected: four `PASS` lines. If the catalogue is rejected for `oneOf`/`const`, stop and report the provider error before changing schemas.

- [ ] **Step 2: Check box convention.** Run `scripts/autonomy/check_cloud_provider.py` with `ICARUS_VLM_GROUNDING_FORMAT=json_pixel_xyxy` as well; keep whichever format puts the box on the labelled object, and set it in the manifest.

- [ ] **Step 3: Replay 170 recorded decisions** with `replay_decision_latency.py --runtime cloud --limit 0` over the 2026-11-06 episodes; record agreement and median latency in `docs/research/CLOUD-API-RESULTS.md`.

- [ ] **Step 4: Fly the Gazebo suite** from `docs/research/HANDOVER-2026-11-06.md` section 2 with `./scripts/dcm-fly --runtime cloud ...` (five missions), then the AirSim find-and-orbit and car-count missions. Compare outcome, decisions and model time against the local results table in `docs/research/AUDIT-2026-11-06.md`.
Expected: no mission the local runtime completes ends `incomplete` for a model reason; median decision latency recorded.

- [ ] **Step 5: Commit the results doc and any manifest change**

```bash
git add docs/research/CLOUD-API-RESULTS.md config/dcm/agent-runtime.json
git commit -m "Record cloud provider checks, replay agreement and sim flights"
```

---

### Task 7: Jev offline benchmark (only with `TYPESAFE_API_KEY`)

**Files:**
- Create: `scripts/autonomy/bench_jev.py`
- Test: `tests/unit/test_bench_jev.py`

**Interfaces:**
- Consumes: `docs/research/vision-bench/labels.json` (24 labelled frames), recorded `agent_decision` events in `logs/episodes/`, `CloudVision` (Task 3) for the comparison column.
- Produces: `bench_jev.py --frames --decisions --out output/jev-bench-YYYYMMDD/` writing `results.json` and printing a table: per question (building, person, car) accuracy, false positives, `review` count (noul/choice probability between 0.3 and 0.7), median latency from this laptop, for Jev and for the cloud VLM; for decisions, Jev `choice` over tool names vs the recorded tool. Exits 2 with "TYPESAFE_API_KEY not set; Jev benchmark blocked" when no key.

- [ ] **Step 1: Write the failing tests** for the pure parts:

```python
def test_jev_request_is_two_way_choice():
    body = bench_jev.visual_request("frame-b64", "house")
    q = body["questions"]["view"]
    assert q["type"] == "choice" and set(q["criteria"]) == {"visible", "absent"}
    assert body["model"] == "jev-latest"

def test_uncertain_band_is_review_not_yes():
    assert bench_jev.verdict({"choice": "visible", "probabilities": {"visible": 0.62, "absent": 0.38}}) is None
    assert bench_jev.verdict({"choice": "visible", "probabilities": {"visible": 0.91, "absent": 0.09}}) is True

def test_missing_key_exits_blocked(monkeypatch)
```

- [ ] **Step 2: Run to verify they fail**, **implement** (POST `https://api.typesafe.ai/v1/systemone`, `Authorization: Bearer`, header `TypeSafe-Danger-Zone-Images: true` for image requests), **run tests and the suite**.

- [ ] **Step 3: Run the benchmark** and add a "Jev" section to `docs/research/CLOUD-API-RESULTS.md` with the table and a verdict. Gate for Task 8: Jev visual accuracy at least the cloud VLM's, zero false positives, `review` under 50 %.

- [ ] **Step 4: Commit**

```bash
git add scripts/autonomy/bench_jev.py tests/unit/test_bench_jev.py docs/research/CLOUD-API-RESULTS.md
git commit -m "Benchmark Jev against the cloud VLM on labelled frames and recorded decisions"
```

---

### Task 8: Jev as a visual-condition checker (only if Task 7's gate passes)

**Files:**
- Modify: `config/dcm/agent-runtime.json` (`"condition_checker": "vlm"`, allowed `"jev"`), `scripts/autonomy/run_mission.py:inspect_scene` (condition questions only)
- Test: `tests/unit/test_cloud_vision.py` (or a new `test_condition_checker.py`)

**Interfaces:**
- Consumes: `bench_jev.visual_request` and `bench_jev.verdict` (Task 7), moved into `python/perception/jev_condition.py` as `check(image_path: Path, visual_query: str) -> dict` returning the same dict shape `inspect_image` returns for `result_type="boolean"` (`found: bool | None`, `answer`, `latency_ms`, `measurement_kind: "jev_choice"`, `flight_authority: False`).

- [ ] **Step 1: Failing tests:** with the flag `jev`, a condition question calls `jev_condition.check` and not the VLM; a `None` verdict yields `found: None`, which `search_for_object` already treats as keep turning; description, box and count questions still use the VLM.
- [ ] **Step 2: Implement, run tests and the suite.**
- [ ] **Step 3: A/B flight:** AirSim and Gazebo find-and-orbit, three runs each with `vlm` and with `jev`; record searches, false sightings and mission time in `CLOUD-API-RESULTS.md`.
- [ ] **Step 4: Commit**

```bash
git add python/perception/jev_condition.py scripts/autonomy/run_mission.py config/dcm/agent-runtime.json tests/unit
git commit -m "Optionally answer visual conditions with Jev, never treating unsure as found"
```

---

## Phase 2 (outline, not tasks yet)

Blocked on choosing the drone. When it is chosen, write a new spec and plan covering, per the design doc: MAVLink transport over serial/UDP (or mavlink-router in front of the TCP gateway) and PX4 mode mapping if needed, a `hardware-flight` launch profile with real limits and geofence, a camera adapter and video downlink the laptop can read, camera calibration, an independent internet link on the laptop, and the bench → tethered hover → first mission test ladder. Physical flight needs the operator's regulatory approval.

## Timeline and process (Phase 1)

Reference design: Aerial Claw (`_reference/AerialClaw`). Its live model path (`llm_client.py`) is an
OpenAI-compatible client chosen by `LLM_BASE_URL` / `LLM_API_KEY` / `LLM_MODEL`, with `VLM_*` falling back to the
`LLM_*` values. Its `AnthropicClient` in `llm/base_client.py` is defined but not used. This plan follows the same
pattern with `ICARUS_*` names; Claude can be reached the same way through Anthropic's OpenAI-compatible endpoint,
checked with `check_cloud_provider.py` like any other provider.

| Days (working) | Work | Done when |
|---|---|---|
| 0.5 | Branch clean-up and merge of `feat/laya-decision-speed` into `main` | `main` at `38c742e` or later, dead branches gone |
| 0.5 | Task 1: settings | unit suite green |
| 1 | Task 2: cloud decisions | unit suite green |
| 1 | Task 3: cloud camera answers | unit suite green |
| 0.5 | Task 4: runtime switch in the CLIs | unit suite green |
| 0.5 | Task 5: provider check and cloud replay | unit suite green; PR opened for Tasks 1-5 |
| 2-3 | Task 6: provider acceptance, replay, Gazebo and AirSim flights | results doc committed; cloud no worse than local |
| 0.5-1 | Task 7: Jev benchmark (needs a key) | verdict recorded |
| 1-2 | Task 8: Jev checker and A/B flights (only if Task 7's gate passes) | A/B recorded |
| 2 | Buffer: provider quirks (schema rejections, box conventions, rate limits) | |

Total: about 8 working days without Jev, 10-12 with it. Starting 2026-11-19, code (Tasks 1-5) lands by
2026-11-25, sim acceptance by about 2026-11-28, and Jev, if a key exists, by about 2026-12-03.

Process per task: failing test, code, unit suite, commit (TDD as written above). One PR for Tasks 1-5 into `main`;
Tasks 6-8 follow as their own commits and PR. Sims run headless on AC power with the stock scripts.

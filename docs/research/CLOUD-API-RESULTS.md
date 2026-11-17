# Cloud model API: results

Plan: `docs/superpowers/plans/2026-11-18-cloud-decision-api.md`. Task 6 (real provider) is still waiting for API
keys. This page records what was verified without them.

## Stand-in provider check (2026-11-18)

The cloud runtime was run end to end against an OpenAI-compatible server on this laptop: `llama-server` serving
Qwen3.5-4B Q4 with its projector at `http://127.0.0.1:8090/v1`, with a dummy key. This exercises every cloud code
path: the HTTP client, tool calling, camera questions as JSON schema, image conversion, stats and provenance.
It does not measure a hosted provider's latency, limits or schema rules.

### `check_cloud_provider.py`

```
PASS provider_reachable  2 ms
PASS catalogue_takeoff  3383 ms  tokens prompt=8345 cached=8341 out=74  arm {} (23 tools)
PASS validate_call  0 ms
PASS condition_true  1350 ms  f02: found=True expected=True
PASS condition_false  769 ms  f00: found=False expected=False
PASS pixel_box  1152 ms  f02: present=True box=[69.12, 28.08, 470.4, 359.64] image=640x360
RESULT PASS
```

The first run failed `catalogue_takeoff` because the model answered "take off" with `arm`, which is correct for a
disarmed drone. The check now accepts `arm` or `takeoff` as the first step and validates whichever it gets.

### Cloud replay

`replay_decision_latency.py --runtime cloud` was run over 4 recorded episodes from 2026-11-06, 22 decisions in all:

| Measure | Result |
|---|---|
| Errors | 0 |
| Same tool as recorded | 20 of 22 |
| Same call as recorded | 10 of 17 non-finish steps |
| Median decision after the first | 2.2 s |

One survey mission chose `survey_count` again at step 3, where the recording had `return_home`.

### Gazebo flight on the cloud runtime

`./scripts/dcm-fly --runtime cloud`, edge-vision profile, headless, on AC power. Mission: "Go up to about 5 meters,
look around until you find a building, then fly around it once keeping the camera pointed at it, and come back and
land."

- **Outcome:** completed in 5 decisions (takeoff, search, orbit, return home, finish).
- **Orbit:** 360.3 deg sweep, 0.12 m maximum radial error, 1.9 deg heading error.
- **Timing:** median decision 3.8 s; 9 camera questions with a median of 3.4 s each.
- **Prompt cache:** after the first decision, 97-99 % of prompt tokens were cached.
- **Episode `20261118T042512_11db69c9e184`:** a `runtime_provenance` event records the provider host, model ids
  and limits, and the key appears nowhere in the episode.

## Still to do (Task 6, needs keys)

- Run `check_cloud_provider.py` on Sarvam with a vision model, and check the box convention.
- Replay the 170 decisions from 2026-11-06.
- Fly the Gazebo and AirSim suites from `docs/research/HANDOVER-2026-11-06.md`.

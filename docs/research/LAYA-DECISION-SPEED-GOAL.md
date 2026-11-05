# Goal: faster, more deterministic mission decisions (Laya branch)

Branch: `feat/laya-decision-speed` (from `main` = v2 at `91d55ef`).
Window: 2026-11-06 00:49 IST, six hours of work, or stop earlier on a significant improvement.

## Baseline (measured from 184 decisions in 30 episodes, 2026-10-23/24, AirSim + Gazebo)

- Median model decision: **14.9 s** (first decision ~8 s, growing with the conversation; 30-150 s after camera calls).
- Prompt per decision: ~9,600 tokens, prefill ~1,070 tok/s, i.e. ~9 s of prefill every step.
- Reply: ~80 tokens at ~43 tok/s, i.e. ~2 s.
- A camera call (`inspect_scene`, `search_for_object`, `localize_target`) stops llama-server to free the GPU, so the
  next decision reloads weights and refills the prompt.
- VLM questions (yes/no "is there a building") ~2.7 s each, uncalibrated: one said "building" on road/grass/sky frames.

## Targets

1. Median decision latency <= 3 s in steady state; no weight reload after camera work.
2. Mission "thinking time" (sum of decision latency) down >= 60 % on the vague-mission suite.
3. Mission correctness no worse than baseline on the same suite, measured in headless sim, camera-checked.
4. Visual and conditional checks (yes/no, which branch) answered by a typed decision engine where it is
   faster and abstains instead of guessing; an abstain must never be treated as "yes".
5. A written verdict on DSPy and ZeroClaw for this system, with evidence.

## Rules

- Architectural changes only. No mission-specific behaviour in prompts or code. Prompts may be shortened.
- Keep it simple: remove or replace before adding. No new orchestration framework unless it clearly earns its place.
- Every claim backed by a measurement in an episode log.

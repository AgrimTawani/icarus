#!/usr/bin/env python3
"""Per-decision and per-camera-question latency of recorded episodes.

  .venv/bin/python scripts/autonomy/episode_latency.py logs/episodes/<id> [...]
"""
import json
import statistics
import sys
from pathlib import Path


def summarize(episode):
    events = [json.loads(line) for line in (Path(episode) / "events.jsonl").open()]
    start = events[0]["unix_ms"]
    decisions, vision = [], []
    for event in events:
        payload = event.get("payload") or {}
        if event["kind"] == "agent_decision":
            stats = payload.get("stats") or {}
            call = ((payload.get("response") or {}).get("tool_calls") or [{}])[0].get("function", {}).get("name")
            decisions.append(payload["latency_ms"])
            print(f"{(event['unix_ms'] - start) / 1000:7.1f}s decision {str(call):18s} {payload['latency_ms']:6d} ms"
                  f"  prefilled={stats.get('tokens_prefilled')} cached={stats.get('tokens_cached')}"
                  f" gen={stats.get('tokens_predicted')} @ {stats.get('predicted_per_second') or 0:.0f} tok/s")
        elif event["kind"] in ("semantic_visual_inspection", "semantic_visual_location", "target_grounding"):
            if isinstance(payload.get("latency_ms"), (int, float)):
                vision.append(payload["latency_ms"])
    mission = next((e["payload"].get("mission") for e in events if e["kind"] == "episode_start"), "")
    total = (events[-1]["unix_ms"] - start) / 1000
    print(json.dumps({"episode": Path(episode).name, "mission": mission[:80], "wall_s": round(total),
                      "decisions": len(decisions), "thinking_s": round(sum(decisions) / 1000, 1),
                      "median_decision_ms": statistics.median(decisions) if decisions else None,
                      "camera_questions": len(vision),
                      "median_camera_ms": round(statistics.median(vision)) if vision else None}))


for path in sys.argv[1:]:
    summarize(path)

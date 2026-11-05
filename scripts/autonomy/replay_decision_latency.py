#!/usr/bin/env python3
"""Replay the recorded decision requests of an episode against a llama-server and time them.

Measures what the mission loop pays per decision: tokens actually prefilled (prompt_n),
tokens served from cache (cache_n), prefill and generation time. No simulator needed:
each request is the exact messages + tools the agent sent during the recorded mission.

  .venv/bin/python scripts/autonomy/replay_decision_latency.py logs/episodes/<id> \
      --server-args "--checkpoint-min-step 0"
"""

import argparse
import json
import shlex
import statistics
import sys
import time
import urllib.request
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT))

from python.dcm.llama_runtime import LlamaCppRuntime  # noqa: E402
from python.dcm.runtime_config import RuntimeDescriptor  # noqa: E402


def decisions(episode):
    for line in (Path(episode) / "events.jsonl").open():
        event = json.loads(line)
        if event.get("kind") == "agent_decision":
            yield event["payload"]


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("episode")
    parser.add_argument("--manifest", default=str(ROOT / "config/dcm/agent-runtime.json"))
    parser.add_argument("--server-args", default="")
    parser.add_argument("--limit", type=int, default=12)
    parser.add_argument("--static-tools", action="store_true",
                        help="send the first decision's catalogue every step (the loop's static catalogue)")
    options = parser.parse_args()
    descriptor = RuntimeDescriptor.from_manifest(options.manifest, role="primary", contract_version="direct-agent-v1",
                                               prompt_version="replay", vocabulary_version="replay")
    runtime = LlamaCppRuntime(descriptor, extra_args=shlex.split(options.server_args))
    rows = []
    static_tools = None
    with runtime:
        for index, payload in enumerate(decisions(options.episode)):
            if index >= options.limit:
                break
            static_tools = static_tools or payload["tools"]
            tools = static_tools if options.static_tools else payload["tools"]
            body = {"messages": payload["messages"], "tools": tools, "temperature": 0.0,
                    "stream": False, "cache_prompt": True, "max_tokens": 768, "tool_choice": "required",
                    "parallel_tool_calls": True, "chat_template_kwargs": {"enable_thinking": False}}
            started = time.monotonic()
            request = urllib.request.Request(runtime.endpoint + "/v1/chat/completions",
                                             data=json.dumps(body).encode(), headers={"Content-Type": "application/json"})
            with urllib.request.urlopen(request, timeout=300) as response:
                reply = json.loads(response.read())
            wall = (time.monotonic() - started) * 1000
            timings = reply.get("timings", {})
            call = ((reply["choices"][0]["message"].get("tool_calls") or [{}])[0].get("function") or {}).get("name")
            recorded = ((payload.get("response") or {}).get("tool_calls") or [{}])[0].get("function", {}).get("name")
            rows.append({"step": index, "wall_ms": round(wall), "prompt_total": reply.get("usage", {}).get("prompt_tokens"),
                         "prefilled": timings.get("prompt_n"), "cached": timings.get("cache_n"),
                         "prefill_ms": round(timings.get("prompt_ms", 0)), "gen_tokens": timings.get("predicted_n"),
                         "gen_ms": round(timings.get("predicted_ms", 0)), "tool": call, "recorded_tool": recorded,
                         "recorded_ms": payload.get("latency_ms")})
            print(json.dumps(rows[-1]), flush=True)
    walls = [row["wall_ms"] for row in rows[1:]] or [rows[0]["wall_ms"]]
    print(json.dumps({"server_args": options.server_args, "static_tools": options.static_tools, "median_wall_ms_after_first": statistics.median(walls),
                      "first_wall_ms": rows[0]["wall_ms"],
                      "same_tool_as_recorded": sum(r["tool"] == r["recorded_tool"] for r in rows), "steps": len(rows)}))


if __name__ == "__main__":
    main()

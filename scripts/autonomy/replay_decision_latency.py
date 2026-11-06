#!/usr/bin/env python3
"""Replay the recorded decision requests of an episode against a llama-server and time them.

Measures what the mission loop pays per decision: tokens actually prefilled (prompt_n),
tokens served from cache (cache_n), prefill and generation time. No simulator needed:
each request is the exact messages + tools the agent sent during the recorded mission.

  .venv/bin/python scripts/autonomy/replay_decision_latency.py logs/episodes/<id> \
      --server-args "--checkpoint-min-step 0"

With --current the recorded conversation is replayed under today's system prompt and
tool catalogue, and each choice is compared with the recorded one: a regression check
for prompt or catalogue edits (--system-file tries a prompt without installing it).

  .venv/bin/python scripts/autonomy/replay_decision_latency.py logs/episodes/2026* --current --limit 0
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


def arguments(function):
    value = function.get("arguments") or {}
    return json.loads(value) if isinstance(value, str) else value


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("episode", nargs="+")
    parser.add_argument("--manifest", default=str(ROOT / "config/dcm/agent-runtime.json"))
    parser.add_argument("--server-args", default="")
    parser.add_argument("--limit", type=int, default=12, help="decisions per episode; 0 = all")
    parser.add_argument("--current", action="store_true",
                        help="replay under the current system prompt and static catalogue")
    parser.add_argument("--system-file", help="system prompt to use instead (implies --current)")
    parser.add_argument("--static-tools", action="store_true",
                        help="send the first decision's catalogue every step (the loop's static catalogue)")
    options = parser.parse_args()
    descriptor = RuntimeDescriptor.from_manifest(options.manifest, role="primary", contract_version="direct-agent-v1",
                                               prompt_version="replay", vocabulary_version="replay")
    runtime = LlamaCppRuntime(descriptor, extra_args=shlex.split(options.server_args))
    rows = []
    current = options.current or options.system_file
    if current:
        from python.dcm.agent_context import system_prompt
        from python.dcm.fly import session_tools
        system = Path(options.system_file).read_text() if options.system_file else system_prompt()
        current_tools = session_tools({})
    with runtime:
      for episode in options.episode:
        static_tools = None
        for index, payload in enumerate(decisions(episode)):
            if options.limit and index >= options.limit:
                break
            static_tools = static_tools or payload["tools"]
            tools = static_tools if options.static_tools else payload["tools"]
            messages = payload["messages"]
            if current:
                tools = current_tools
                messages = [{"role": "system", "content": system}] + messages[1:]
            body = {"messages": messages, "tools": tools, "temperature": 0.0,
                    "stream": False, "cache_prompt": True, "max_tokens": 768, "tool_choice": "required",
                    "parallel_tool_calls": True, "chat_template_kwargs": {"enable_thinking": False},
                    **runtime._mission_slot()}
            started = time.monotonic()
            request = urllib.request.Request(runtime.endpoint + "/v1/chat/completions",
                                             data=json.dumps(body).encode(), headers={"Content-Type": "application/json"})
            with urllib.request.urlopen(request, timeout=300) as response:
                reply = json.loads(response.read())
            wall = (time.monotonic() - started) * 1000
            timings = reply.get("timings", {})
            message = reply["choices"][0]["message"]
            function = (message.get("tool_calls") or [{}])[0].get("function") or {}
            was = ((payload.get("response") or {}).get("tool_calls") or [{}])[0].get("function", {})
            call, recorded = function.get("name"), was.get("name")
            rows.append({"episode": Path(episode).name, "step": index, "wall_ms": round(wall),
                         "arguments": arguments(function), "recorded_arguments": arguments(was),
                         "text_before_call": len(message.get("content") or ""), "prompt_total": reply.get("usage", {}).get("prompt_tokens"),
                         "prefilled": timings.get("prompt_n"), "cached": timings.get("cache_n"),
                         "prefill_ms": round(timings.get("prompt_ms", 0)), "gen_tokens": timings.get("predicted_n"),
                         "gen_ms": round(timings.get("predicted_ms", 0)), "tool": call, "recorded_tool": recorded,
                         "recorded_ms": payload.get("latency_ms")})
            print(json.dumps(rows[-1]), flush=True)
    walls = [row["wall_ms"] for row in rows[1:]] or [rows[0]["wall_ms"]]
    for row in rows:
        if row["tool"] != row["recorded_tool"]:
            print(f"DIFFERENT TOOL {row['episode']} step {row['step']}: recorded {row['recorded_tool']} "
                  f"{json.dumps(row['recorded_arguments'])} -> now {row['tool']} {json.dumps(row['arguments'])}")
    print(json.dumps({"server_args": options.server_args, "static_tools": options.static_tools, "median_wall_ms_after_first": statistics.median(walls),
                      "first_wall_ms": rows[0]["wall_ms"],
                      "same_tool_as_recorded": sum(r["tool"] == r["recorded_tool"] for r in rows),
                      "same_call_as_recorded": sum(r["tool"] == r["recorded_tool"] and r["arguments"] == r["recorded_arguments"]
                                                   for r in rows if r["tool"] != "finish"),
                      "non_finish_steps": sum(r["tool"] != "finish" for r in rows),
                      "gen_tokens": sum(r["gen_tokens"] or 0 for r in rows),
                      "steps_with_text_before_call": sum(r["text_before_call"] > 0 for r in rows),
                      "steps": len(rows)}))


if __name__ == "__main__":
    main()

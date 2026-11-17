#!/usr/bin/env python3
"""Replay the recorded decision requests of an episode against a llama-server and time them.

Measures what the mission loop pays per decision: tokens actually prefilled (prompt_n),
tokens served from cache (cache_n), prefill and generation time. No simulator needed:
each request is the exact messages + tools the agent sent during the recorded mission.

  .venv/bin/python scripts/autonomy/replay_decision_latency.py logs/episodes/<id> \
      --server-args "--checkpoint-min-step 0"

With --current the recorded conversation is replayed under today's system prompt (the
recorded tool catalogue is kept: it carries that vehicle's limits) and each choice is
compared with the recorded one: a regression check for prompt edits. --system-file
tries a prompt without installing it.

  .venv/bin/python scripts/autonomy/replay_decision_latency.py logs/episodes/2026* --current --limit 0

With --runtime cloud the same requests go to the hosted model from ICARUS_LLM_* through
CloudRuntime.decide instead (--server-args is ignored). Rows report the tokens the provider
counted; prefill and generation times are not reported by providers and show as null. A decision
the provider fails (refusal, timeout, outage) prints a row with an "error" field and the replay
continues; the summary counts those as "errors". Settings problems exit with status 2.

  .venv/bin/python scripts/autonomy/replay_decision_latency.py logs/episodes/<id> --runtime cloud
"""

import argparse
import contextlib
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
from python.model_runtime.select import ALIASES, KINDS, make_runtime  # noqa: E402


def decisions(episode):
    for line in (Path(episode) / "events.jsonl").open():
        event = json.loads(line)
        if event.get("kind") == "agent_decision":
            yield event["payload"]


def arguments(function):
    value = function.get("arguments") or {}
    return json.loads(value) if isinstance(value, str) else value


@contextlib.contextmanager
def stopping(runtime):
    """Always stop a hosted-model runtime that the replay started."""
    try:
        yield runtime
    finally:
        runtime.stop()


def fail(message):
    """A one-line, key-free message on stderr and exit status 2: no traceback for a settings problem."""
    print(f"replay_decision_latency: {' '.join(str(message).split())[:300]}", file=sys.stderr)
    sys.exit(2)


def build_parser():
    parser = argparse.ArgumentParser()
    parser.add_argument("episode", nargs="+")
    parser.add_argument("--runtime", type=lambda name: ALIASES.get(name, name), choices=KINDS, default="local",
                        help="local: a llama-server started here (default); cloud: the hosted model from ICARUS_LLM_*")
    parser.add_argument("--manifest", default=str(ROOT / "config/dcm/agent-runtime.json"))
    parser.add_argument("--server-args", default="")
    parser.add_argument("--limit", type=int, default=12, help="decisions per episode; 0 = all")
    parser.add_argument("--current", action="store_true",
                        help="replay under the current system prompt")
    parser.add_argument("--system-file", help="system prompt to use instead (implies --current)")
    parser.add_argument("--static-tools", action="store_true",
                        help="send the first decision's catalogue every step (the loop's static catalogue)")
    return parser


def main():
    options = build_parser().parse_args()
    cloud = options.runtime == "cloud"
    if cloud:
        from python.model_runtime.cloud_config import MissingCloudSetting
        try:
            runtime, record = make_runtime("cloud", options.manifest)
        except MissingCloudSetting as failure:
            fail(failure)
        except (OSError, KeyError, ValueError) as failure:
            fail(f"cannot read the cloud settings from {options.manifest}: {type(failure).__name__} {failure}")
        try:
            runtime.start()
        except RuntimeError as failure:
            fail(failure)
    else:
        descriptor = RuntimeDescriptor.from_manifest(options.manifest, role="primary", contract_version="direct-agent-v1",
                                                   prompt_version="replay", vocabulary_version="replay")
        runtime = LlamaCppRuntime(descriptor, extra_args=shlex.split(options.server_args))
    rows = []
    current = options.current or options.system_file
    if current:
        from python.dcm.agent_context import system_prompt
        system = Path(options.system_file).read_text() if options.system_file else system_prompt()
    with stopping(runtime) if cloud else runtime:
      for episode in options.episode:
        static_tools = None
        for index, payload in enumerate(decisions(episode)):
            if options.limit and index >= options.limit:
                break
            static_tools = static_tools or payload["tools"]
            tools = static_tools if options.static_tools else payload["tools"]
            messages = payload["messages"]
            if current:
                messages = [{"role": "system", "content": system}] + messages[1:]
            started_at = time.monotonic()
            error = None
            if cloud:
                runtime.last_stats = None
                try:
                    message = runtime.decide(messages, tools)
                except Exception as failure:  # noqa: BLE001 - one failed decision must not end the replay
                    message, error = {}, f"{type(failure).__name__}: {failure}"
                stats = runtime.last_stats or {}
                counts = {"prompt_total": stats.get("tokens_prompt"), "prefilled": stats.get("tokens_prefilled"),
                          "cached": stats.get("tokens_cached"), "prefill_ms": None,
                          "gen_tokens": stats.get("tokens_predicted"), "gen_ms": None}
            else:
                body = {"messages": messages, "tools": tools, "temperature": 0.0,
                        "stream": False, "cache_prompt": True, "max_tokens": 768, "tool_choice": "required",
                        "parallel_tool_calls": True, "chat_template_kwargs": {"enable_thinking": False},
                        **runtime._mission_slot()}
                request = urllib.request.Request(runtime.endpoint + "/v1/chat/completions",
                                                 data=json.dumps(body).encode(), headers={"Content-Type": "application/json"})
                with urllib.request.urlopen(request, timeout=300) as response:
                    reply = json.loads(response.read())
                timings = reply.get("timings", {})
                message = reply["choices"][0]["message"]
                counts = {"prompt_total": reply.get("usage", {}).get("prompt_tokens"),
                          "prefilled": timings.get("prompt_n"), "cached": timings.get("cache_n"),
                          "prefill_ms": round(timings.get("prompt_ms", 0)), "gen_tokens": timings.get("predicted_n"),
                          "gen_ms": round(timings.get("predicted_ms", 0))}
            wall = (time.monotonic() - started_at) * 1000
            function = (message.get("tool_calls") or [{}])[0].get("function") or {}
            was = ((payload.get("response") or {}).get("tool_calls") or [{}])[0].get("function", {})
            call, recorded = function.get("name"), was.get("name")
            rows.append({"episode": Path(episode).name, "step": index, "wall_ms": round(wall),
                         "arguments": arguments(function), "recorded_arguments": arguments(was),
                         "text_before_call": len(message.get("content") or ""), **counts,
                         "tool": call, "recorded_tool": recorded,
                         "recorded_ms": payload.get("latency_ms"),
                         **({"error": " ".join(error.split())[:300]} if error else {})})
            print(json.dumps(rows[-1]), flush=True)
    timed = [row for row in rows if "error" not in row]  # a failed decision has no latency worth timing
    walls = [row["wall_ms"] for row in timed[1:]] or [row["wall_ms"] for row in timed[:1]]
    for row in rows:
        if "error" not in row and row["tool"] != row["recorded_tool"]:
            print(f"DIFFERENT TOOL {row['episode']} step {row['step']}: recorded {row['recorded_tool']} "
                  f"{json.dumps(row['recorded_arguments'])} -> now {row['tool']} {json.dumps(row['arguments'])}")
    provenance = ({"provider_host": record["provider_host"], "model": record["model"],
                   "errors": len(rows) - len(timed)} if cloud else {})
    print(json.dumps({**provenance, "server_args": options.server_args, "static_tools": options.static_tools,
                      "median_wall_ms_after_first": statistics.median(walls) if walls else None,
                      "first_wall_ms": timed[0]["wall_ms"] if timed else None,
                      "same_tool_as_recorded": sum(r["tool"] == r["recorded_tool"] for r in rows),
                      "same_call_as_recorded": sum(r["tool"] == r["recorded_tool"] and r["arguments"] == r["recorded_arguments"]
                                                   for r in rows if r["tool"] != "finish"),
                      "non_finish_steps": sum(r["tool"] != "finish" for r in rows),
                      "gen_tokens": sum(r["gen_tokens"] or 0 for r in rows),
                      "steps_with_text_before_call": sum(r["text_before_call"] > 0 for r in rows),
                      "steps": len(rows)}))


if __name__ == "__main__":
    main()

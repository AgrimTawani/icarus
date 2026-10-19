"""Icarus direct mission agent. No precompiled mission or model review gate."""

import argparse
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT))
sys.path.insert(0, str(ROOT / "build/generated/python"))
from python.dcm.console import OperatorConsole
from python.dcm.fly import (
    AgentSession,
    dumps,
    fly_mission,
    safe_exit_recovery,
    session_episode_summary,
    summarize,
)
from python.dcm.llama_runtime import LlamaCppRuntime
from python.dcm.operator_control import OperatorInterrupt, operator_safety_command
from python.dcm.runtime_config import RuntimeDescriptor, DIRECT_PROMPT_VERSION, DIRECT_TOOL_VERSION

DEFAULT_MANIFEST = ROOT / "config/dcm/agent-runtime.json"
BANNER = """Icarus drone pilot — direct feedback agent
Enter a mission. Clarification answers continue the same mission.
/state | /prompts on|off | /mode approval|autonomous | /quit
Immediate operator commands: /hold /stop /land /rtl /abort
Other input during execution is discarded. No whole-mission approval is required.
"""


def _load_mission_client():
    import importlib.util

    spec = importlib.util.spec_from_file_location(
        "icarus_run_mission", ROOT / "scripts/autonomy/run_mission.py"
    )
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module.MissionClient


def build_runtime(args):
    overrides = {
        "contract_version": "direct-agent-v1",
        "prompt_version": DIRECT_PROMPT_VERSION,
        "vocabulary_version": DIRECT_TOOL_VERSION,
    }
    if args.timeout_ms is not None:
        overrides["deadline_ms"] = args.timeout_ms
    if args.context_length is not None:
        overrides["context_length"] = args.context_length
    descriptor = RuntimeDescriptor.from_manifest(
        args.manifest, role="primary", **overrides
    )
    return LlamaCppRuntime(descriptor), descriptor


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--endpoint", default="127.0.0.1:50051")
    parser.add_argument("--manifest", type=Path, default=DEFAULT_MANIFEST)
    # Keep the user's familiar command valid, with only one live implementation.
    parser.add_argument(
        "--runtime", choices=["llama"], default="llama", help=argparse.SUPPRESS
    )
    parser.add_argument(
        "--profile", choices=["default", "edge-vision"], default="edge-vision"
    )
    parser.add_argument(
        "--mode", choices=["approval", "autonomous"], default="autonomous"
    )
    parser.add_argument("--timeout-ms", type=int)
    parser.add_argument("--context-length", type=int)
    parser.add_argument("--max-decisions", type=int, default=64)
    parser.add_argument("--mission")
    parser.add_argument("--show-prompts", action="store_true")
    parser.add_argument("--json", action="store_true")
    args = parser.parse_args()
    if args.max_decisions <= 0:
        parser.error("--max-decisions must be positive")
    runtime, descriptor = build_runtime(args)
    print(
        f"Model: {descriptor.model_id}; architecture: direct-agent-v1; mode: {args.mode}",
        flush=True,
    )
    client = _load_mission_client()(
        args.endpoint, args.mission or "direct_agent_session", args.profile
    )
    console = OperatorConsole()
    results = []
    connected = False
    exit_code = 0
    try:
        client.connect()
        client.acquire()
        connected = True
        print("Preparing camera tools…", flush=True)
        client.prepare_camera_detector()
        print("Loading mission model…", flush=True)
        runtime.start()
        print(BANNER)
        session = AgentSession(args.mission) if args.mission else None
        pending = args.mission
        while True:
            if pending is None:
                console.discard_pending()
                print(
                    "clarification> "
                    if session and session.awaiting_user
                    else "mission> ",
                    end="",
                    flush=True,
                )
                line = console.readline()
                if not line or line.strip() == "/quit":
                    break
                text = line.strip()
                if not text:
                    continue
                if text == "/state":
                    print(client.state())
                    continue
                if text.startswith("/prompts "):
                    args.show_prompts = text.split()[-1] == "on"
                    continue
                if text.startswith("/mode "):
                    mode = text.split()[-1]
                    if mode in ("approval", "autonomous"):
                        args.mode = mode
                    else:
                        print("Use /mode approval or /mode autonomous")
                    continue
                if (
                    session
                    and session.awaiting_user
                    and not operator_safety_command(text)
                ):
                    if text.lower() in ("cancel", "/cancel"):
                        session = None
                        print("Mission cancelled.")
                        continue
                    session.answer(text)
                else:
                    session = AgentSession(text, previous_mission=(
                        {'request': results[-1]['mission'], 'outcome': results[-1]['status']}
                        if results else None))
            else:
                text, pending = pending, None
            interrupt = OperatorInterrupt()
            interrupt.enable_safety(client)
            if operator_safety_command(text):
                interrupt.submit(text)
            console.attach(interrupt)
            try:
                result = fly_mission(
                    client,
                    runtime,
                    session.mission,
                    mode=args.mode,
                    max_decisions=args.max_decisions,
                    show_prompts=args.show_prompts,
                    stream=console,
                    interrupt=interrupt,
                    session=session,
                )
            finally:
                console.attach(None)
            print(summarize(result), flush=True)
            if args.json:
                print(dumps(result))
            if result["status"] != "awaiting_user":
                results.append(result)
                session = None
            client.episode_outcome, client.episode_score = session_episode_summary(
                results
            )
            if args.mission:
                exit_code = 0 if result["status"] == "completed" else 2
                break
    except KeyboardInterrupt:
        print("Interrupted. Ending the session.")
        client.episode_outcome, client.episode_score = session_episode_summary(
            results, 'Operator interrupted the session before completion')
        exit_code = 130
    except BaseException as error:
        client.episode_outcome, client.episode_score = session_episode_summary(results, error)
        raise
    finally:
        try:
            if connected:
                try:
                    recovery = safe_exit_recovery(client)
                except BaseException as error:
                    client.episode_outcome, client.episode_score = session_episode_summary(results, error)
                    raise
                if recovery["status"] not in ("SUCCEEDED", "manual_override"):
                    print("Exit recovery did not complete:", recovery)
                    client.episode_outcome, client.episode_score = session_episode_summary(
                        results, 'Exit recovery did not complete: ' + str(recovery))
                    exit_code = 2
        finally:
            try:
                runtime.stop()
            finally:
                client.close()
    return exit_code


if __name__ == "__main__":
    raise SystemExit(main())

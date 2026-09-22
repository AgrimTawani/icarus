"""Chat a mission to the DCM and watch it fly in simulation.

Approval mode is the default and must be opted out of. Nothing here bypasses
the Drone API, the guardrails or the safety supervisor: a proposal becomes an
action only by going through the same typed RPCs any other client uses.
"""

import argparse
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT))
sys.path.insert(0, str(ROOT / "build/generated/python"))

from python.dcm.contract import (
    PROMPT_VERSION,
    PROMPT_VERSION_ENDING,
    RuntimeDescriptor,
)
from python.dcm.fly import dumps, fly_mission, session_episode_summary, summarize
from python.dcm.resource_sampler import ResourceSampler

DEFAULT_MANIFEST = Path.home() / "models/qwen/MANIFEST.json"
EDGE_MANIFEST = Path.home() / "models/edge/MANIFEST.json"

BANNER = """\
Icarus DCM flight console
  Type a mission in plain language, or:
    /mode approval|autonomous   change execution mode
    /state                      print current vehicle state
    /quit                       land nothing, release control and exit
"""


def _load_mission_client():
    """Load MissionClient from scripts/autonomy without packaging scripts/.

    scripts/ holds executables rather than an importable package, and adding
    __init__.py files there to satisfy one import would change what the
    directory is for.
    """
    import importlib.util
    path = ROOT / "scripts/autonomy/run_mission.py"
    spec = importlib.util.spec_from_file_location("icarus_run_mission", path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module.MissionClient


def build_runtime(args):
    if args.runtime == "mock":
        from python.dcm.observe import MockRuntime
        return MockRuntime(), None
    if args.runtime == "scripted":
        from python.dcm.baseline import ScriptedRuntime
        return ScriptedRuntime(), None
    from python.dcm.llama_runtime import LlamaCppRuntime
    descriptor = RuntimeDescriptor.from_manifest(
        args.manifest, role=args.role, deadline_ms=args.timeout_ms,
        context_length=args.context_length,
        prompt_version=(PROMPT_VERSION_ENDING if args.ending_guidance
                        else PROMPT_VERSION))
    runtime = LlamaCppRuntime(descriptor,
                              ending_guidance=args.ending_guidance)
    return runtime, descriptor


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--endpoint", default="127.0.0.1:50051")
    parser.add_argument("--runtime", choices=("llama", "mock", "scripted"), default="llama")
    parser.add_argument("--manifest", type=Path, default=DEFAULT_MANIFEST)
    parser.add_argument("--profile", choices=("default", "edge-vision"), default="default")
    parser.add_argument("--role", default="primary",
                        choices=("primary", "secondary", "second_family", "qwen_edge_primary"))
    parser.add_argument("--mode", choices=("approval", "autonomous"),
                        default="approval",
                        help="approval asks before every action; autonomous "
                             "does not and is simulation-only")
    parser.add_argument("--timeout-ms", type=int, default=15000,
                        help="per-decision deadline; the first decision of a "
                             "session is far slower than the rest")
    parser.add_argument("--context-length", type=int, default=4096,
                        help="llama.cpp context for interactive flight; 4096 "
                             "fits alongside Gazebo on the 6 GB GPU")
    parser.add_argument("--ending-guidance", action="store_true",
                        help="prompt v2: state that a flight ends on the ground")
    parser.add_argument("--max-decisions", type=int, default=40)
    parser.add_argument("--mission",
                        help="run one mission and exit instead of chatting")
    parser.add_argument("--json", action="store_true",
                        help="print the machine-readable result too")
    args = parser.parse_args()
    if args.profile == "edge-vision":
        if args.manifest == DEFAULT_MANIFEST:
            args.manifest = EDGE_MANIFEST
        if args.mode != "autonomous":
            parser.error("edge-vision is simulation-only and requires --mode autonomous")
        if args.context_length != 4096:
            parser.error("edge-vision requires --context-length 4096")
        if args.role == "primary":
            args.role = "qwen_edge_primary"

    MissionClient = _load_mission_client()

    runtime, descriptor = build_runtime(args)
    client = MissionClient(args.endpoint, args.mission or "dcm_chat_session")
    mode = args.mode
    session_results = []
    sampler = None
    try:
        client.connect()
        client.acquire()
        print(f"connected to {args.endpoint} in {mode} mode")
        if args.runtime == "llama":
            print("loading model, the first decision is slow...")
            runtime.start()
        sampler = ResourceSampler(
            pid_provider=lambda: (runtime.process.pid if getattr(runtime, "process", None) else None))
        sampler.__enter__()
        if args.mission:
            missions = [args.mission]
        else:
            print(BANNER)
            missions = None

        while True:
            if missions is not None:
                if not missions:
                    break
                mission = missions.pop(0)
            else:
                try:
                    mission = input("mission> ").strip()
                except EOFError:
                    break
                if not mission:
                    continue
                if mission == "/quit":
                    break
                if mission == "/state":
                    print(client.state())
                    continue
                if mission.startswith("/mode"):
                    _, _, requested = mission.partition(" ")
                    if requested.strip() in ("approval", "autonomous"):
                        mode = requested.strip()
                        print(f"mode is now {mode}")
                    else:
                        print("usage: /mode approval|autonomous")
                    continue

            result = fly_mission(
                client, runtime, mission, mode=mode, descriptor=descriptor,
                max_decisions=args.max_decisions)
            if sampler:
                result["resources"] = sampler.peaks()
            session_results.append(result)
            # MissionClient.close() seals unconditionally.  Update these after
            # every mission so an operator exit still retains all previous
            # results and any terminal action failure is marked truthfully.
            client.episode_outcome, client.episode_score = session_episode_summary(
                session_results)
            print()
            print(summarize(result))
            if args.json:
                print(dumps(result))
            print()
    except BaseException as error:
        # This covers model/server failures and interruption after connect.  A
        # sealed failed episode is valuable regression evidence; never let a
        # chat-session exception turn into the default "completed" manifest.
        client.episode_outcome, client.episode_score = session_episode_summary(
            session_results, error)
        raise
    finally:
        if sampler:
            sampler.__exit__(None, None, None)
            if session_results:
                session_results[-1]["resources"] = sampler.peaks()
        stop = getattr(runtime, "stop", None)
        if stop:
            stop()
        client.close()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

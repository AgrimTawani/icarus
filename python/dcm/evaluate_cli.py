"""Evaluate a DCM runtime across an episode corpus, offline and observe-only.

Nothing here imports the Drone API, MAVLink or a simulator, and no proposal is
ever executed.
"""

import argparse
import sys
from pathlib import Path

from python.dcm.contract import (
    PROMPT_VERSION,
    PROMPT_VERSION_ENDING,
    RuntimeDescriptor,
)
from python.dcm.evaluate import evaluate, format_report
from python.dcm.observe import MockRuntime

DEFAULT_MANIFEST = Path.home() / "models/qwen/MANIFEST.json"


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("episodes", type=Path, nargs="*",
                        help="episode directories; defaults to all of logs/episodes")
    parser.add_argument("--runtime", choices=("mock", "llama"), default="llama")
    parser.add_argument("--manifest", type=Path, default=DEFAULT_MANIFEST)
    parser.add_argument("--role", default="primary",
                        choices=("primary", "secondary"),
                        help="which pinned artifact to evaluate")
    parser.add_argument("--repeats", type=int, default=3,
                        help="runs per episode; one run is not a measurement")
    parser.add_argument("--timeout-ms", type=int, default=5000)
    parser.add_argument("--history", action="store_true",
                        help="show the model the actions already completed"
                             " (contract v2) instead of only the last result")
    parser.add_argument("--elapsed", action="store_true",
                        help="show mission elapsed time in the observation")
    parser.add_argument("--ending-guidance", action="store_true",
                        help="prompt v2: state that a flight ends on the ground")
    parser.add_argument("--limit", type=int, default=0,
                        help="evaluate at most this many episodes")
    args = parser.parse_args()

    root = Path(__file__).resolve().parents[2]
    episodes = args.episodes or sorted(
        path for path in (root / "logs/episodes").iterdir()
        if (path / "manifest.json").is_file())
    if args.limit:
        episodes = episodes[:args.limit]
    if not episodes:
        parser.error("no episodes found; fly some with ./scripts/fly-episode-corpus")

    descriptor = None
    if args.runtime == "mock":
        runtime = MockRuntime()
    else:
        from python.dcm.llama_runtime import LlamaCppRuntime
        descriptor = RuntimeDescriptor.from_manifest(
            args.manifest, role=args.role, deadline_ms=args.timeout_ms,
            prompt_version=(PROMPT_VERSION_ENDING if args.ending_guidance
                            else PROMPT_VERSION))
        runtime = LlamaCppRuntime(descriptor,
                                  ending_guidance=args.ending_guidance)

    def progress(episode, attempt, total):
        print(f"  {episode}  run {attempt}/{total}", file=sys.stderr, flush=True)

    try:
        output, report = evaluate(
            episodes, runtime, root / "logs/dcm/evaluation",
            repeats=args.repeats, timeout_ms=args.timeout_ms,
            descriptor=descriptor, progress=progress,
            include_history=args.history,
            include_elapsed=args.elapsed)
    finally:
        stop = getattr(runtime, "stop", None)
        if stop:
            stop()

    print()
    print(format_report(report))
    print()
    print("Evaluation report:", output / "evaluation.json")
    if report["totals"]["counts"]["invalid"] or report["totals"]["counts"]["error"]:
        return 1
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

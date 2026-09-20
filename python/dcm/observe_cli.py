"""Run a DCM runtime against a sealed episode without any flight access.

Observe mode only: proposals are recorded and never executed. Nothing here
imports the Drone API, MAVLink or a simulator.
"""

import argparse
import json
from pathlib import Path

from python.dcm.contract import RuntimeDescriptor
from python.dcm.observe import MockRuntime, observe_episode

DEFAULT_MANIFEST = Path.home() / "models/qwen/MANIFEST.json"


def build_runtime(args):
    """Return (runtime, descriptor). The mock needs no artifact."""
    if args.runtime == "mock":
        return MockRuntime(), None
    from python.dcm.llama_runtime import LlamaCppRuntime
    descriptor = RuntimeDescriptor.from_manifest(
        args.manifest, role=args.role, deadline_ms=args.timeout_ms)
    return LlamaCppRuntime(descriptor, verify_artifact=not args.skip_verify), descriptor


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("episode", type=Path)
    parser.add_argument("--runtime", choices=("mock", "llama"), default="mock",
                        help="mock proposes nothing; llama runs the pinned GGUF")
    parser.add_argument("--manifest", type=Path, default=DEFAULT_MANIFEST,
                        help="model manifest written by setup-model-runtime")
    parser.add_argument("--role", default="primary",
                        choices=("primary", "secondary"),
                        help="which pinned artifact to load")
    parser.add_argument("--timeout-ms", type=int, default=5000,
                        help="hard per-decision deadline for the llama runtime")
    parser.add_argument("--history", action="store_true",
                        help="show the model the actions already completed")
    parser.add_argument("--skip-verify", action="store_true",
                        help="skip the artifact checksum check (not for evaluation)")
    args = parser.parse_args()

    root = Path(__file__).resolve().parents[2]
    runtime, descriptor = build_runtime(args)
    try:
        output, summary = observe_episode(
            args.episode, runtime, root / "logs/dcm/observe",
            timeout_ms=args.timeout_ms, descriptor=descriptor,
            include_history=args.history)
    finally:
        stop = getattr(runtime, "stop", None)
        if stop:
            stop()
    print(json.dumps(summary, indent=2))
    print("Observe report:", output)


if __name__ == "__main__":
    main()

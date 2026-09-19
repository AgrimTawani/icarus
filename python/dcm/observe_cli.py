"""Run the initial mock DCM against a sealed episode without flight access."""

import argparse
import json
from pathlib import Path

from python.dcm.observe import MockRuntime, observe_episode


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("episode", type=Path)
    parser.add_argument("--timeout-ms", type=int, default=5000)
    args = parser.parse_args()
    root = Path(__file__).resolve().parents[2]
    output, summary = observe_episode(
        args.episode, MockRuntime(), root / "logs/dcm/observe",
        timeout_ms=args.timeout_ms)
    print(json.dumps(summary, indent=2))
    print("Observe report:", output)


if __name__ == "__main__":
    main()

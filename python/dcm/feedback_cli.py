"""Build a deterministic DCM reliability report and review queue offline."""

import argparse
import json
from pathlib import Path

from python.dcm.v2_feedback import aggregate, review_proposals


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("decisions", nargs="+", type=Path,
                        help="observe decisions.jsonl files")
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--hypothesis",
                        help="optional untrusted model/human hypothesis retained for review")
    args = parser.parse_args()
    rows = []
    for path in args.decisions:
        rows.extend(json.loads(line) for line in path.read_text().splitlines()
                    if line.strip())
    report = aggregate(rows)
    queue = review_proposals(report, hypothesis=args.hypothesis)
    args.output.mkdir(parents=True, exist_ok=False)
    (args.output / "reliability.json").write_text(
        json.dumps(report, indent=2, sort_keys=True) + "\n")
    (args.output / "review_queue.json").write_text(
        json.dumps(queue, indent=2, sort_keys=True) + "\n")
    print("Reliability report:", args.output / "reliability.json")
    print("Review queue:", args.output / "review_queue.json")


if __name__ == "__main__":
    main()

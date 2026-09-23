import argparse
import json
from pathlib import Path

from python.dcm.v2_promotion import DEFAULT_SCORECARD, decide


def main():
    parser = argparse.ArgumentParser(
        description="Apply the frozen V2 scorecard to paired sealed reports.")
    parser.add_argument("baseline", type=Path)
    parser.add_argument("candidate", type=Path)
    parser.add_argument("--baseline-live", type=Path)
    parser.add_argument("--candidate-live", type=Path)
    parser.add_argument("--scorecard", type=Path, default=DEFAULT_SCORECARD)
    parser.add_argument("--output", type=Path,
                        default=Path("logs/dcm/v2-promotion"))
    args = parser.parse_args()
    output, report = decide(
        args.baseline, args.candidate, args.output, args.scorecard,
        args.baseline_live, args.candidate_live)
    print(json.dumps({"decision": report["decision"],
                      "all_required_gates_passed": report["all_required_gates_passed"],
                      "failed_gates": [gate["name"] for gate in report["gates"]
                                       if not gate["passed"]]}, indent=2))
    print("Decision report:", output / "promotion-decision.json")
    return 0 if report["all_required_gates_passed"] else 1


if __name__ == "__main__":
    raise SystemExit(main())

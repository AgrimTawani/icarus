"""Evaluate Phase 12 promotion thresholds without changing those thresholds.

This is a verdict generator, not a tuning tool: its only inputs are sealed
evaluation reports.  A failed check remains visible in the output and exits
non-zero; there is no flag that turns a failure into a pass.
"""

import argparse
import json
from pathlib import Path


def _check(name, actual, expected, passed):
    return {"name": name, "actual": actual, "expected": expected,
            "passed": bool(passed)}


def check_observe(report):
    """Apply the fixed Phase 12 DCM-observe policy to an evaluation report."""
    totals = report.get("totals", {})
    counts = totals.get("counts", {})
    coverage = report.get("per_action", {}).get("coverage", {}).get("land", {})
    checks = [
        _check("held_out_scoreable_points", totals.get("comparable_points", 0),
               ">= 100", totals.get("comparable_points", 0) >= 100),
        _check("malformed_outputs", counts.get("invalid", 0), "= 0",
               counts.get("invalid", 0) == 0),
        _check("runtime_errors", counts.get("error", 0), "= 0",
               counts.get("error", 0) == 0),
        # Requiring zero total timeouts is conservative: it includes warm-up
        # and therefore cannot accidentally waive a post-warm-up timeout.
        _check("timeouts", counts.get("timeout", 0), "= 0",
               counts.get("timeout", 0) == 0),
        _check("unchecked_proposals", totals.get("guardrail_unchecked", 0), "= 0",
               totals.get("guardrail_unchecked", 0) == 0),
        _check("guardrail_rejected_proposals", totals.get("guardrail_rejected", 0), "= 0",
               totals.get("guardrail_rejected", 0) == 0),
        _check("terminal_land_coverage", coverage,
               "at least one land situation and all agree",
               coverage.get("situations", 0) > 0 and
               coverage.get("always_agreed", 0) == coverage.get("situations", 0)),
    ]
    return {"tier": "dcm_observe", "passed": all(c["passed"] for c in checks),
            "checks": checks}


def check_autonomous(report):
    """Apply the mission-level autonomous-SITL policy to a live report."""
    episodes = report.get("closed_loop_episodes", 0)
    success = report.get("mission_success_rate")
    checks = [
        _check("complete_missions", episodes, ">= 45", episodes >= 45),
        _check("mission_success_rate", success, ">= 0.95",
               success is not None and success >= 0.95),
        _check("safety_interventions", report.get("safety_interventions", 0),
               "= 0 unhandled", report.get("safety_interventions", 0) == 0),
        _check("replayable_episodes", report.get("episodes", 0),
               f"= {episodes}", report.get("episodes", 0) == episodes),
    ]
    return {"tier": "autonomous_sitl", "passed": all(c["passed"] for c in checks),
            "checks": checks}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("tier", choices=("observe", "autonomous"))
    parser.add_argument("report", type=Path)
    args = parser.parse_args()
    report = json.loads(args.report.read_text())
    verdict = check_observe(report) if args.tier == "observe" else check_autonomous(report)
    print(json.dumps(verdict, indent=2))
    raise SystemExit(0 if verdict["passed"] else 1)


if __name__ == "__main__":
    main()

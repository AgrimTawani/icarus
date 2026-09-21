"""Score sealed *closed-loop* DCM episodes without rerunning a simulator.

Offline evaluation answers whether a model emits valid, comparable proposals.
This module measures what happened after those proposals reached the real
simulator execution path: recorded mission outcome, recovery after a failed
action, completion time, and safety interventions.  It never guesses a
mission result from an LLM response, and it never starts Gazebo or sends a
Drone API request.
"""

import argparse
import json
import time
from collections import Counter
from pathlib import Path

from python.dataset_tools.replay import ReplayError, verify_complete

SCHEMA = "icarus.dcm.live-evaluation.v1"
BAD_DECISION_STATUSES = {"invalid", "timeout", "error", "stale", "stale_after"}


def _events(directory):
    with (Path(directory) / "events.jsonl").open(encoding="utf-8") as source:
        return [json.loads(line) for line in source if line.strip()]


def _score_status(manifest):
    """Return a real mission score only when the flight wrapper wrote one."""
    score = manifest.get("score")
    if isinstance(score, dict) and score.get("status") in ("passed", "failed", "completed"):
        return score["status"] in ("passed", "completed")
    # Non-DCM acceptance episodes write a direct ``passed``/``failed`` outcome.
    if manifest.get("outcome") in ("passed", "failed"):
        return manifest["outcome"] == "passed"
    return None


def _safety_event(item):
    payload = item.get("payload", {})
    text = json.dumps(payload, sort_keys=True).lower()
    return (item.get("kind") == "action_status" and
            payload.get("state") == "ACTION_STATE_ABORTED_BY_SAFETY") or \
        (item.get("kind") == "guardrail_validation" and
         not payload.get("result", {}).get("authorized", True)) or \
        (item.get("kind") == "vehicle_event" and "safety" in text)


def score_episode(directory, check_guardrails=True):
    """Return transparent closed-loop metrics for one sealed episode.

    ``correct_tool_selection_rate`` is intentionally ``None`` unless a
    campaign supplies an explicit ``expected_action`` annotation.  Successful
    execution only proves admissibility, not that an API choice fulfilled the
    human mission.  Offline reports provide the separately named
    ``tool_selection_agreement_rate`` against recorded baseline actions.
    """
    directory = Path(directory)
    integrity = verify_complete(directory, check_guardrails=check_guardrails)
    manifest = json.loads((directory / "manifest.json").read_text())
    events = _events(directory)
    decisions = [item["payload"] for item in events
                 if item.get("kind") == "dcm_decision"]
    statuses = Counter(decision.get("status") for decision in decisions)
    executed = [d for d in decisions if d.get("executed")]
    failed = [d for d in decisions if d.get("status") in BAD_DECISION_STATUSES
              or d.get("outcome") not in (None, "SUCCEEDED")]
    recovered = 0
    for index, decision in enumerate(decisions[:-1]):
        failed_here = decision in failed
        next_decision = decisions[index + 1]
        if failed_here and next_decision.get("executed") and \
                next_decision.get("outcome") == "SUCCEEDED":
            recovered += 1
    annotated = [d for d in decisions if d.get("expected_action") is not None]
    correct = sum(1 for d in annotated
                  if (d.get("proposal") or {}).get("action") == d["expected_action"])
    interventions = sum(1 for item in events if _safety_event(item))
    duration_ms = manifest["finished_unix_ms"] - manifest["started_unix_ms"]
    return {
        "episode_id": manifest["episode_id"],
        "source": manifest["source"],
        "scenario": manifest.get("scenario"),
        "seed": manifest.get("seed"),
        "model": manifest.get("model"),
        "integrity": {"complete": integrity["complete"],
                      "replay_deterministic": integrity["replay_deterministic"],
                      "replay_result_sha256": integrity["replay_result_sha256"]},
        "closed_loop": bool(decisions),
        "mission_success": _score_status(manifest) if decisions else None,
        "decision_statuses": dict(statuses),
        "action_count": len(executed),
        "completion_time_ms": duration_ms,
        "recoverable_events": len(failed),
        "recovered_events": recovered,
        "recovery_success_rate": recovered / len(failed) if failed else None,
        "tool_selection_annotations": len(annotated),
        "correct_tool_selection_rate": correct / len(annotated) if annotated else None,
        "safety_interventions": interventions,
    }


def _rate(values):
    return sum(values) / len(values) if values else None


def score_campaign(episodes, output_root, check_guardrails=True):
    """Score a homogeneous list of sealed live episodes and retain the report."""
    per_episode = [score_episode(path, check_guardrails=check_guardrails)
                   for path in episodes]
    closed_loop = [row for row in per_episode if row["closed_loop"]]
    success = [row["mission_success"] for row in closed_loop
               if row["mission_success"] is not None]
    tools = [row["correct_tool_selection_rate"] for row in closed_loop
             if row["correct_tool_selection_rate"] is not None]
    recoverable = sum(row["recoverable_events"] for row in closed_loop)
    recovered = sum(row["recovered_events"] for row in closed_loop)
    report = {
        "schema": SCHEMA,
        "generated_at": time.strftime("%Y-%m-%dT%H:%M:%S%z"),
        "episodes": len(per_episode),
        "closed_loop_episodes": len(closed_loop),
        "mission_success_rate": _rate(success),
        "correct_tool_selection_rate": _rate(tools),
        "tool_selection_annotations": sum(row["tool_selection_annotations"] for row in closed_loop),
        "recovery_success_rate": recovered / recoverable if recoverable else None,
        "recoverable_events": recoverable,
        "recovered_events": recovered,
        "action_count": sum(row["action_count"] for row in closed_loop),
        "completion_time_ms": [row["completion_time_ms"] for row in closed_loop],
        "safety_interventions": sum(row["safety_interventions"] for row in closed_loop),
        "per_episode": per_episode,
    }
    output = Path(output_root) / time.strftime("%Y%m%dT%H%M%S")
    output.mkdir(parents=True, exist_ok=False)
    (output / "live_evaluation.json").write_text(json.dumps(report, indent=2) + "\n")
    return output, report


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("episodes", nargs="+", type=Path)
    parser.add_argument("--output", type=Path,
                        default=Path("logs/dcm/live-evaluation"))
    parser.add_argument("--skip-guardrails", action="store_true")
    args = parser.parse_args()
    try:
        output, report = score_campaign(args.episodes, args.output,
                                        check_guardrails=not args.skip_guardrails)
    except (OSError, ValueError, KeyError, json.JSONDecodeError, ReplayError) as error:
        parser.exit(1, f"Live evaluation FAILED: {error}\n")
    print(json.dumps({key: value for key, value in report.items()
                      if key != "per_episode"}, indent=2))
    print("Live evaluation:", output / "live_evaluation.json")


if __name__ == "__main__":
    main()

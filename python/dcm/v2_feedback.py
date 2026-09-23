"""Deterministic feedback aggregation for V2 DCM evidence.

This is intentionally an offline report builder. It cannot write a macro,
modify a prompt, or change any safety policy.
"""

import hashlib
import json
import statistics
from collections import defaultdict

FEEDBACK_SCHEMA = "icarus.dcm.feedback.v2"
PROPOSAL_SCHEMA = "icarus.dcm.review-proposals.v1"


def _canonical(value):
    return json.dumps(value, sort_keys=True, separators=(",", ":"))


def _finish(item):
    executed = item.get("executed", 0)
    proposed = item.get("proposed", item.get("decisions", 0))
    item["execution_rate"] = executed / proposed if proposed else None
    item["success_rate"] = item.get("succeeded", 0) / executed if executed else None
    latencies = item.pop("_latencies", [])
    item["latency_median_ms"] = round(statistics.median(latencies), 3) if latencies else None
    return item


def aggregate(decisions):
    """Summarize sealed decision rows by action and macro revision."""
    decisions = list(decisions)
    by_action = defaultdict(lambda: {"proposed": 0, "executed": 0,
                                     "succeeded": 0, "failed": 0,
                                     "guardrail_rejected": 0,
                                     "invalid": 0, "late": 0,
                                     "_latencies": []})
    by_macro = defaultdict(lambda: {"decisions": 0, "executed": 0,
                                    "succeeded": 0, "failed": 0,
                                    "_latencies": []})
    total = 0
    for row in decisions:
        if not isinstance(row, dict):
            raise TypeError("decision rows must be mappings")
        total += 1
        proposal = row.get("proposal") or {}
        action = proposal.get("action")
        status = row.get("status")
        if isinstance(action, str):
            item = by_action[action]
            item["proposed"] += 1
            if row.get("executed"):
                item["executed"] += 1
                if row.get("outcome") == "SUCCEEDED":
                    item["succeeded"] += 1
                else:
                    item["failed"] += 1
            if status == "invalid":
                item["invalid"] += 1
            if status == "timeout":
                item["late"] += 1
            if type(row.get("latency_ms")) in (int, float):
                item["_latencies"].append(row["latency_ms"])
            guardrail = row.get("guardrail") or {}
            if guardrail.get("checked") and not guardrail.get("would_execute"):
                item["guardrail_rejected"] += 1
        macro = row.get("mission_macro") or {}
        if isinstance(macro.get("id"), str):
            key = f"{macro['id']}@{macro.get('revision', '?')}"
            item = by_macro[key]
            item["decisions"] += 1
            if row.get("executed"):
                item["executed"] += 1
                if row.get("outcome") == "SUCCEEDED":
                    item["succeeded"] += 1
                else:
                    item["failed"] += 1
            if type(row.get("latency_ms")) in (int, float):
                item["_latencies"].append(row["latency_ms"])
    report = {"schema": FEEDBACK_SCHEMA, "decisions": total,
              "by_action": {key: _finish(value) for key, value in sorted(by_action.items())},
              "by_macro": {key: _finish(value) for key, value in sorted(by_macro.items())}}
    report["source_sha256"] = hashlib.sha256(
        _canonical(decisions).encode()).hexdigest()
    report["report_sha256"] = hashlib.sha256(_canonical(report).encode()).hexdigest()
    return report


def review_proposals(report, minimum_samples=3, maximum_failure_rate=0.25,
                     hypothesis=None):
    """Build an offline review queue; never write or activate policy.

    Any model-authored hypothesis is retained as explicitly untrusted text.
    Queue entries identify evidence and a requested human review, not a patch.
    """
    proposals = []
    for scope, rows in (("action", report.get("by_action", {})),
                        ("macro", report.get("by_macro", {}))):
        for identifier, metrics in sorted(rows.items()):
            executed = metrics.get("executed", 0)
            failed = metrics.get("failed", 0)
            failure_rate = failed / executed if executed else None
            if executed >= minimum_samples and failure_rate is not None and \
                    failure_rate > maximum_failure_rate:
                proposals.append({
                    "scope": scope, "id": identifier,
                    "reason": "measured_failure_rate_exceeded",
                    "failure_rate": failure_rate,
                    "executed_samples": executed,
                    "evidence_report_sha256": report["report_sha256"],
                    "requested_action": "human_review",
                })
    result = {"schema": PROPOSAL_SCHEMA,
              "source_report_sha256": report.get("report_sha256"),
              "proposals": proposals,
              "model_hypothesis": hypothesis,
              "model_hypothesis_trusted": False}
    result["queue_sha256"] = hashlib.sha256(_canonical(result).encode()).hexdigest()
    return result

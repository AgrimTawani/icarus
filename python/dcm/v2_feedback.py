"""Deterministic feedback aggregation for V2 DCM evidence.

This is intentionally an offline report builder. It cannot write a macro,
modify a prompt, or change any safety policy.
"""

from collections import defaultdict

FEEDBACK_SCHEMA = "icarus.dcm.feedback.v1"


def aggregate(decisions):
    """Summarize sealed decision rows by action and macro revision."""
    by_action = defaultdict(lambda: {"proposed": 0, "executed": 0,
                                     "succeeded": 0, "failed": 0,
                                     "guardrail_rejected": 0})
    by_macro = defaultdict(lambda: {"decisions": 0, "executed": 0,
                                    "succeeded": 0})
    total = 0
    for row in decisions:
        if not isinstance(row, dict):
            raise TypeError("decision rows must be mappings")
        total += 1
        proposal = row.get("proposal") or {}
        action = proposal.get("action")
        if isinstance(action, str):
            item = by_action[action]
            item["proposed"] += 1
            if row.get("executed"):
                item["executed"] += 1
                if row.get("outcome") == "SUCCEEDED":
                    item["succeeded"] += 1
                else:
                    item["failed"] += 1
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
    return {"schema": FEEDBACK_SCHEMA, "decisions": total,
            "by_action": dict(sorted(by_action.items())),
            "by_macro": dict(sorted(by_macro.items()))}

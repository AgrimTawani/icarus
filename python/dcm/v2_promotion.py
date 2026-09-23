"""Fail-closed, predeclared Phase-1 V2 promotion decision.

The candidate is never promoted by this module. It writes an auditable
``promote``/``retain-v1`` recommendation from sealed offline and live Gazebo
reports; changing the runtime default remains a separate reviewed commit.
"""

import hashlib
import json
import statistics
import time
from pathlib import Path

import yaml

ROOT = Path(__file__).resolve().parents[2]
DEFAULT_SCORECARD = ROOT / "config/dcm/v2-scorecard.yaml"
SCHEMA = "icarus.dcm.v2-promotion-decision.v1"


def _canonical(value):
    return json.dumps(value, sort_keys=True, separators=(",", ":"))


def _hash_bytes(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def _metric(report, *keys):
    value = report
    for key in keys:
        if not isinstance(value, dict):
            return None
        value = value.get(key)
    return value


def _gate(gates, name, passed, baseline=None, candidate=None, detail=None):
    gates.append({"name": name, "passed": bool(passed),
                  "baseline": baseline, "candidate": candidate,
                  "detail": detail})


def _less_or_equal(gates, name, baseline, candidate, ceiling=None):
    passed = baseline is not None and candidate is not None and candidate <= baseline
    if ceiling is not None:
        passed = passed and candidate <= ceiling
    _gate(gates, name, passed, baseline, candidate,
          f"candidate <= baseline{f' and <= {ceiling}' if ceiling is not None else ''}")


def decide(baseline_path, candidate_path, output_root, scorecard_path=DEFAULT_SCORECARD,
           baseline_live_path=None, candidate_live_path=None):
    """Evaluate the frozen scorecard and retain every input hash."""
    baseline_path, candidate_path = Path(baseline_path), Path(candidate_path)
    scorecard_path = Path(scorecard_path)
    baseline = json.loads(baseline_path.read_text())
    candidate = json.loads(candidate_path.read_text())
    scorecard = yaml.safe_load(scorecard_path.read_text())
    if scorecard.get("schema") != "icarus.dcm.v2-promotion-scorecard.v1":
        raise ValueError("unsupported V2 scorecard")
    offline = scorecard["offline"]
    gates = []
    baseline_episodes = [row.get("episode") for row in baseline.get("per_episode", [])]
    candidate_episodes = [row.get("episode") for row in candidate.get("per_episode", [])]
    _gate(gates, "identical_offline_corpus",
          baseline_episodes == candidate_episodes and bool(baseline_episodes),
          baseline_episodes, candidate_episodes)
    _gate(gates, "identical_repeats",
          baseline.get("repeats") == candidate.get("repeats"),
          baseline.get("repeats"), candidate.get("repeats"))
    points = _metric(candidate, "totals", "decision_points")
    _gate(gates, "minimum_decision_points",
          points is not None and points >= offline["minimum_decision_points"],
          offline["minimum_decision_points"], points)
    for name, key, ceiling in (
        ("invalid_rate", "invalid_rate", offline["maximum_invalid_rate"]),
        ("timeout_rate", "timeout_rate", offline["maximum_timeout_rate"]),
        ("guardrail_rejection_rate", "guardrail_rejection_rate",
         offline["maximum_guardrail_rejection_rate"]),
        ("error_rate", "error_rate", 0.0),
    ):
        _less_or_equal(gates, name, _metric(baseline, "totals", key),
                       _metric(candidate, "totals", key), ceiling)
    base_latency = _metric(baseline, "latency", "steady_state_medians", "median_ms")
    cand_latency = _metric(candidate, "latency", "steady_state_medians", "median_ms")
    latency_ok = (base_latency is not None and cand_latency is not None and
                  cand_latency <= offline["maximum_steady_latency_ms"] and
                  cand_latency <= base_latency * offline["maximum_latency_ratio"])
    _gate(gates, "steady_inference_latency", latency_ok, base_latency, cand_latency)
    base_vram = _metric(baseline, "resource_peaks", "peak_vram_mib")
    cand_vram = _metric(candidate, "resource_peaks", "peak_vram_mib")
    _gate(gates, "compute_peak", base_vram is not None and cand_vram is not None and
          cand_vram <= base_vram + offline["maximum_vram_increase_mib"],
          base_vram, cand_vram)
    improvements = []
    for key, higher_is_better in (("tool_selection_agreement_rate", True),
                                  ("recovery_rate", True),
                                  ("invalid_rate", False),
                                  ("timeout_rate", False)):
        old = _metric(baseline, "totals", key)
        new = _metric(candidate, "totals", key)
        if old is not None and new is not None and (
                (new > old) if higher_is_better else (new < old)):
            improvements.append(key)
    _gate(gates, "predeclared_metric_improvement", bool(improvements),
          None, improvements)

    live_inputs = (baseline_live_path, candidate_live_path)
    if all(live_inputs):
        base_live = json.loads(Path(baseline_live_path).read_text())
        cand_live = json.loads(Path(candidate_live_path).read_text())
        live = scorecard["live_gazebo"]
        base_pairs = [(row.get("scenario"), row.get("seed"))
                      for row in base_live.get("per_episode", [])]
        cand_pairs = [(row.get("scenario"), row.get("seed"))
                      for row in cand_live.get("per_episode", [])]
        _gate(gates, "paired_live_gazebo_episodes",
              base_pairs == cand_pairs and len(base_pairs) >= live["minimum_paired_episodes"],
              base_pairs, cand_pairs)
        _gate(gates, "mission_success_no_regression",
              base_live.get("mission_success_rate") is not None and
              cand_live.get("mission_success_rate") is not None and
              cand_live["mission_success_rate"] >= base_live["mission_success_rate"],
              base_live.get("mission_success_rate"), cand_live.get("mission_success_rate"))
        _gate(gates, "safety_interventions_no_regression",
              cand_live.get("safety_interventions", 0) <=
              base_live.get("safety_interventions", 0) +
              live["maximum_safety_intervention_increase"],
              base_live.get("safety_interventions"), cand_live.get("safety_interventions"))
        base_times = base_live.get("completion_time_ms") or []
        cand_times = cand_live.get("completion_time_ms") or []
        base_time = statistics.median(base_times) if base_times else None
        cand_time = statistics.median(cand_times) if cand_times else None
        _gate(gates, "completion_time", base_time is not None and cand_time is not None and
              cand_time <= base_time * live["maximum_completion_time_ratio"],
              base_time, cand_time)
        base_count = base_live.get("action_count")
        cand_count = cand_live.get("action_count")
        _gate(gates, "action_count", base_count is not None and cand_count is not None and
              cand_count <= base_count * live["maximum_action_count_ratio"],
              base_count, cand_count)
        base_recovery = base_live.get("recovery_success_rate")
        cand_recovery = cand_live.get("recovery_success_rate")
        recovery_ok = (base_recovery is None and cand_recovery is None) or (
            base_recovery is not None and cand_recovery is not None and
            cand_recovery >= base_recovery)
        _gate(gates, "live_recovery_no_regression", recovery_ok,
              base_recovery, cand_recovery)
    else:
        _gate(gates, "paired_live_gazebo_episodes", False, None, None,
              "both sealed live reports are required")

    passed = all(gate["passed"] for gate in gates)
    report = {
        "schema": SCHEMA,
        "generated_at": time.strftime("%Y-%m-%dT%H:%M:%S%z"),
        "decision": "promote-v2" if passed else "retain-v1",
        "all_required_gates_passed": passed,
        "scorecard": {"path": str(scorecard_path),
                      "sha256": _hash_bytes(scorecard_path)},
        "inputs": {
            "baseline": {"path": str(baseline_path), "sha256": _hash_bytes(baseline_path)},
            "candidate": {"path": str(candidate_path), "sha256": _hash_bytes(candidate_path)},
            "baseline_live": ({"path": str(baseline_live_path),
                               "sha256": _hash_bytes(baseline_live_path)}
                              if baseline_live_path else None),
            "candidate_live": ({"path": str(candidate_live_path),
                                "sha256": _hash_bytes(candidate_live_path)}
                               if candidate_live_path else None),
        },
        "improvements": improvements,
        "gates": gates,
    }
    report["report_sha256"] = hashlib.sha256(_canonical(report).encode()).hexdigest()
    output = Path(output_root) / time.strftime("%Y%m%dT%H%M%S")
    output.mkdir(parents=True, exist_ok=False)
    (output / "promotion-decision.json").write_text(json.dumps(report, indent=2) + "\n")
    return output, report

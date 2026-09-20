"""Offline evaluation of a DCM runtime across an episode corpus.

This scores how a model behaves on recorded decision points. It never touches
the Drone API, MAVLink or a simulator, and nothing it measures is executed.

Three rules shape the scoring, each learned from a real result rather than
chosen for tidiness:

* **Repeated runs, reported as a spread.** The same model on the same episode
  produced first-decision latencies of 4257, 4563 and 7204 ms, one of which
  breached the deadline. A single figure would have been misleading whichever
  run it came from.
* **Vocabulary mismatches are unscoreable, not failures.** A recorded flight
  may use actions outside `contract.ACTIONS`, such as `goto`. A model that
  cannot express the baseline's action has not disagreed with it, and counting
  that as a miss would understate every model equally and hide real
  differences.
* **Agreement is not correctness.** The recorded action is one competent
  choice, not ground truth, so agreement is reported as a rate against
  comparable points and never as a score out of all points.
"""

import json
import statistics
import time
from pathlib import Path

from python.dataset_tools.replay import ReplayError
from python.dcm.contract import (
    ALLOWED_ACTIONS,
    CONTRACT_VERSION,
    CONTRACT_VERSION_HISTORY,
)
from python.dcm.observe import observe_episode
from python.dcm.resource_sampler import ResourceSampler

SCHEMA = "icarus.dcm.evaluation.v1"
STATUSES = ("valid", "invalid", "timeout", "error", "stale")


def _percentile(values, fraction):
    if not values:
        return None
    ordered = sorted(values)
    index = min(len(ordered) - 1, round(fraction * (len(ordered) - 1)))
    return ordered[index]


def _summarize(values, suffix):
    if not values:
        return None
    return {
        "count": len(values),
        f"min{suffix}": round(min(values), 1),
        f"median{suffix}": round(statistics.median(values), 1),
        f"p90{suffix}": round(_percentile(values, 0.90), 1),
        f"max{suffix}": round(max(values), 1),
        f"mean{suffix}": round(statistics.fmean(values), 1),
    }


def summarize_latency(values):
    """Describe a latency sample, or return None when there is nothing to say."""
    return _summarize(values, "_ms")


def summarize_rate(values):
    """Describe a tokens-per-second sample; same shape, a different unit."""
    return _summarize(values, "_tps")


def score_decisions(decisions, allowed=ALLOWED_ACTIONS):
    """Score one run's decision stream.

    Latency is split between the first decision and the rest, because the
    first pays a one-off prefill cost that is not representative of the loop.
    """
    counts = dict.fromkeys(STATUSES, 0)
    comparable = 0
    agreed = 0
    unscoreable = []
    first_latency = None
    steady_latencies = []
    asked = 0
    guardrail_checked = 0
    guardrail_rejected = 0
    guardrail_unchecked = 0
    rejection_reasons = []
    tokens_per_second = []
    for decision in decisions:
        status = decision["status"]
        counts[status] = counts.get(status, 0) + 1
        guardrail = decision.get("guardrail")
        if guardrail:
            if guardrail.get("checked"):
                guardrail_checked += 1
                if not guardrail.get("would_execute"):
                    guardrail_rejected += 1
                    rejection_reasons.append(guardrail.get("reason_code"))
            else:
                guardrail_unchecked += 1
        stats = decision.get("runtime_stats")
        if stats and stats.get("predicted_per_second") is not None:
            tokens_per_second.append(stats["predicted_per_second"])
        if status != "stale":
            # Stale points never reach the runtime, so their zero latency is
            # not a measurement of the model.
            if asked == 0:
                first_latency = decision["latency_ms"]
            else:
                steady_latencies.append(decision["latency_ms"])
            asked += 1
        recorded = decision.get("recorded_action")
        if recorded not in allowed:
            unscoreable.append(recorded)
            continue
        if status != "valid":
            continue
        comparable += 1
        if decision["proposal"]["action"] == recorded:
            agreed += 1
    decided = counts["valid"] + counts["invalid"] + counts["timeout"] + counts["error"]
    return {
        "decision_points": len(decisions),
        "counts": counts,
        "asked": asked,
        "invalid_rate": (counts["invalid"] / decided) if decided else None,
        "timeout_rate": (counts["timeout"] / decided) if decided else None,
        "error_rate": (counts["error"] / decided) if decided else None,
        "comparable_points": comparable,
        "agreed_points": agreed,
        "agreement_rate": (agreed / comparable) if comparable else None,
        "unscoreable_points": len(unscoreable),
        "unscoreable_actions": sorted(set(unscoreable)),
        "first_decision_latency_ms": first_latency,
        "steady_latency": summarize_latency(steady_latencies),
        # A valid proposal that the flight safety policy would reject is a
        # different failure mode than a malformed one, so it is counted
        # separately rather than folded into invalid_rate.
        "guardrail_checked": guardrail_checked,
        "guardrail_rejected": guardrail_rejected,
        "guardrail_unchecked": guardrail_unchecked,
        "guardrail_rejection_rate": (
            guardrail_rejected / guardrail_checked if guardrail_checked
            else None),
        "guardrail_rejection_reasons": sorted(
            r for r in set(rejection_reasons) if r),
        "tokens_per_second": summarize_rate(tokens_per_second),
    }


def per_action_breakdown(per_episode, allowed=ALLOWED_ACTIONS):
    """Count recorded-vs-proposed pairs, and distinct situations per action.

    The aggregate agreement rate hid a model that never proposed `land`, so
    this breakdown is produced alongside it rather than left to be derived by
    hand. `situations` counts distinct decision points rather than runs,
    because three repeats of one situation are not three pieces of evidence.
    """
    pairs = {}
    situations = {}
    for entry in per_episode:
        for run in entry["runs"]:
            for index, proposal in enumerate(run["proposals"]):
                recorded = proposal["recorded"]
                if recorded not in allowed or proposal["status"] != "valid":
                    continue
                key = (recorded, proposal["proposed"])
                pairs[key] = pairs.get(key, 0) + 1
                situations.setdefault(recorded, {}).setdefault(
                    (entry["episode"], index), set()).add(proposal["proposed"])
    rows = [{"recorded": recorded, "proposed": proposed, "count": count,
             "agrees": recorded == proposed}
            for (recorded, proposed), count in
            sorted(pairs.items(), key=lambda item: (item[0][0], -item[1]))]
    coverage = {
        recorded: {
            "situations": len(points),
            "always_agreed": sum(1 for answers in points.values()
                                 if answers == {recorded}),
        }
        for recorded, points in sorted(situations.items())
    }
    return {"pairs": rows, "coverage": coverage}


def _aggregate(runs, key):
    values = [run[key] for run in runs if run.get(key) is not None]
    return summarize_latency(values) if values else None


def evaluate(episodes, runtime, output_root, repeats=3, timeout_ms=5000,
             descriptor=None, check_guardrails=True, progress=None,
             include_history=False, include_elapsed=False,
             check_proposal_guardrails=True):
    """Replay every episode `repeats` times and score the result.

    Returns (output_directory, report). Proposals are recorded, never executed.
    """
    episodes = [Path(e) for e in episodes]
    if not episodes:
        raise ValueError("no episodes to evaluate")
    if repeats < 1:
        raise ValueError("repeats must be at least 1")
    output_root = Path(output_root)
    output_root.mkdir(parents=True, exist_ok=True)
    output = output_root / time.strftime("%Y%m%dT%H%M%S")
    output.mkdir(exist_ok=True)

    per_episode = []
    skipped = []
    started = time.monotonic()
    # A 14B model and a 4B model can score identically and cost completely
    # different amounts of memory; that trade-off is invisible unless it is
    # measured across the whole campaign rather than assumed from the
    # quantization alone.
    sampler = ResourceSampler(
        pid_provider=lambda: getattr(getattr(runtime, "process", None),
                                     "pid", None))
    with sampler:
        for episode in episodes:
            runs = []
            for attempt in range(1, repeats + 1):
                if progress:
                    progress(episode.name, attempt, repeats)
                try:
                    report_dir, _ = observe_episode(
                        episode, runtime, output / "observe",
                        timeout_ms=timeout_ms,
                        check_guardrails=check_guardrails, descriptor=descriptor,
                        include_history=include_history,
                        include_elapsed=include_elapsed,
                        check_proposal_guardrails=check_proposal_guardrails)
                except (ReplayError, OSError, KeyError) as unusable:
                    # A corpus accumulates episodes that predate a schema
                    # change or were sealed mid-failure. Losing the whole
                    # campaign to one of them would be worse than reporting
                    # it and moving on.
                    skipped.append({
                        "episode": episode.name,
                        "reason": f"{type(unusable).__name__}: {unusable}"})
                    runs = []
                    break
                decisions = [json.loads(line) for line in
                            (report_dir / "decisions.jsonl").read_text().splitlines()]
                scored = score_decisions(decisions)
                scored["run"] = attempt
                scored["report"] = report_dir.name
                scored["proposals"] = [
                    {"recorded": d.get("recorded_action"),
                     "proposed": d["proposal"]["action"] if d["proposal"] else None,
                     "status": d["status"]}
                    for d in decisions
                ]
                runs.append(scored)
            if not runs:
                continue
            # Determinism is worth measuring directly: a model that answers
            # differently run to run at temperature 0 is a finding in itself.
            signatures = {json.dumps(run["proposals"], sort_keys=True)
                         for run in runs}
            per_episode.append({
                "episode": episode.name,
                "runs": runs,
                "deterministic": len(signatures) == 1,
                "first_decision_latency": _aggregate(
                    runs, "first_decision_latency_ms"),
            })
    resource_peaks = sampler.peaks()

    all_runs = [run for entry in per_episode for run in entry["runs"]]
    totals = dict.fromkeys(STATUSES, 0)
    comparable = agreed = unscoreable = 0
    guardrail_checked = guardrail_rejected = guardrail_unchecked = 0
    rejection_reasons = []
    steady = []
    firsts = []
    tokens_per_second = []
    # The runtime stays up for the whole campaign, so only the very first
    # decision pays to prefill a cold server. Pooling it with every episode's
    # first decision would hide a 7218 ms cold start inside a 387 ms median.
    cold_start_ms = (all_runs[0]["first_decision_latency_ms"]
                     if all_runs else None)
    for run in all_runs:
        for status, value in run["counts"].items():
            totals[status] = totals.get(status, 0) + value
        comparable += run["comparable_points"]
        agreed += run["agreed_points"]
        unscoreable += run["unscoreable_points"]
        guardrail_checked += run["guardrail_checked"]
        guardrail_rejected += run["guardrail_rejected"]
        guardrail_unchecked += run["guardrail_unchecked"]
        rejection_reasons.extend(run["guardrail_rejection_reasons"])
        if run["first_decision_latency_ms"] is not None and run is not all_runs[0]:
            firsts.append(run["first_decision_latency_ms"])
        if run["steady_latency"]:
            steady.append(run["steady_latency"]["median_ms"])
        if run["tokens_per_second"]:
            tokens_per_second.append(run["tokens_per_second"]["median_tps"])
    decided = totals["valid"] + totals["invalid"] + totals["timeout"] + totals["error"]

    report = {
        "schema": SCHEMA,
        "mode": "observe",
        "executed_actions": 0,
        "runtime": runtime.name,
        "contract_version": (CONTRACT_VERSION_HISTORY if include_history
                             else CONTRACT_VERSION),
        "mission_elapsed_shown": include_elapsed,
        "prompt_version": (descriptor.prompt_version if descriptor
                           else None),
        "model": descriptor.as_record() if descriptor else None,
        "episodes": len(per_episode),
        "episodes_requested": len(episodes),
        "skipped_episodes": skipped,
        "repeats": repeats,
        "timeout_ms": timeout_ms,
        "wall_clock_s": round(time.monotonic() - started, 1),
        "totals": {
            "counts": totals,
            "decision_points": sum(run["decision_points"] for run in all_runs),
            "invalid_rate": (totals["invalid"] / decided) if decided else None,
            "timeout_rate": (totals["timeout"] / decided) if decided else None,
            "error_rate": (totals["error"] / decided) if decided else None,
            "stale_refusals": totals["stale"],
            "comparable_points": comparable,
            "agreement_rate": (agreed / comparable) if comparable else None,
            "unscoreable_points": unscoreable,
            "guardrail_checked": guardrail_checked,
            "guardrail_rejected": guardrail_rejected,
            "guardrail_unchecked": guardrail_unchecked,
            "guardrail_rejection_rate": (
                guardrail_rejected / guardrail_checked if guardrail_checked
                else None),
            "guardrail_rejection_reasons": sorted(set(rejection_reasons)),
        },
        "latency": {
            "cold_start_ms": cold_start_ms,
            "episode_first_decision": summarize_latency(firsts),
            "steady_state_medians": summarize_latency(steady),
        },
        "tokens_per_second": summarize_rate(tokens_per_second),
        "resource_peaks": resource_peaks,
        "deterministic_episodes": sum(1 for e in per_episode if e["deterministic"]),
        "per_action": per_action_breakdown(per_episode),
        "per_episode": per_episode,
    }
    (output / "evaluation.json").write_text(json.dumps(report, indent=2) + "\n")
    return output, report


def format_report(report):
    """A short human summary. The JSON remains the authoritative record."""
    totals = report["totals"]
    lines = [
        f"runtime            {report['runtime']}",
        f"contract           {report.get('contract_version', '?')}"
        + ("  +elapsed" if report.get("mission_elapsed_shown") else ""),
        f"prompt             {report.get('prompt_version') or '?'}",
        (f"episodes x repeats {report['episodes']} x {report['repeats']}"
         f"  ({totals['decision_points']} decision points)"),
        "",
        "counts             " + "  ".join(
            f"{name}={value}" for name, value in totals["counts"].items()),
    ]
    for label, key in (("invalid rate", "invalid_rate"),
                       ("timeout rate", "timeout_rate"),
                       ("error rate", "error_rate")):
        value = totals[key]
        lines.append(f"{label:<19}{value:.1%}" if value is not None
                     else f"{label:<19}n/a")
    agreement = totals["agreement_rate"]
    lines.append(
        f"{'agreement':<19}"
        + (f"{agreement:.1%} of {totals['comparable_points']} comparable points"
           if agreement is not None else "n/a"))
    lines.append(f"{'unscoreable':<19}{totals['unscoreable_points']}"
                 " (recorded action outside the vocabulary)")
    lines.append(f"{'stale refusals':<19}{totals['stale_refusals']}")
    rejection_rate = totals["guardrail_rejection_rate"]
    lines.append(
        f"{'guardrail reject':<19}"
        + (f"{rejection_rate:.1%} of {totals['guardrail_checked']} checked"
           f" against the real safety policy"
           if rejection_rate is not None else "n/a")
        + (f" ({totals['guardrail_unchecked']} unchecked, binary not built)"
           if totals["guardrail_unchecked"] else ""))
    if totals["guardrail_rejection_reasons"]:
        lines.append(f"{'':<19}reasons: "
                     + ", ".join(totals["guardrail_rejection_reasons"]))
    lines.append("")
    cold = report["latency"].get("cold_start_ms")
    if cold is not None:
        lines.append(f"{'cold start':<19}{cold:.0f} ms (one per campaign)")
    for label, key in (("episode first", "episode_first_decision"),
                       ("steady state", "steady_state_medians")):
        stats = report["latency"].get(key)
        if stats:
            lines.append(
                f"{label:<19}median {stats['median_ms']:.0f} ms, "
                f"min {stats['min_ms']:.0f}, max {stats['max_ms']:.0f}")
    tps = report.get("tokens_per_second")
    if tps:
        lines.append(f"{'tokens/sec':<19}median {tps['median_tps']:.1f}, "
                     f"min {tps['min_tps']:.1f}, max {tps['max_tps']:.1f}")
    peaks = report.get("resource_peaks") or {}
    if peaks.get("peak_vram_mib") is not None:
        lines.append(f"{'peak VRAM':<19}{peaks['peak_vram_mib']:.0f} MiB"
                     " (whole GPU, not process-isolated)")
    if peaks.get("peak_ram_mib") is not None:
        lines.append(f"{'peak RAM':<19}{peaks['peak_ram_mib']:.0f} MiB"
                     " (model runtime process)")
    breakdown = report.get("per_action")
    if breakdown:
        lines.append("")
        lines.append("per action         recorded -> proposed")
        for row in breakdown["pairs"]:
            lines.append(
                f"{'':<19}{row['recorded']:<12} -> {row['proposed']:<12}"
                f"{row['count']:>4}{'  OK' if row['agrees'] else ''}")
        lines.append("")
        lines.append("coverage           distinct situations (always agreed)")
        for action, stats in breakdown["coverage"].items():
            lines.append(f"{'':<19}{action:<12} {stats['situations']:>3}"
                         f"  ({stats['always_agreed']})")
    lines.append("")
    if report.get("skipped_episodes"):
        lines.append(f"{'skipped episodes':<19}{len(report['skipped_episodes'])}"
                     f" of {report.get('episodes_requested', '?')} (unusable)")
        for entry in report["skipped_episodes"]:
            lines.append(f"{'':<19}  {entry['episode']}: {entry['reason']}")
    lines.append(f"{'deterministic':<19}"
                 f"{report['deterministic_episodes']}/{report['episodes']} episodes")
    lines.append(f"{'executed actions':<19}{report['executed_actions']}")
    return "\n".join(lines)

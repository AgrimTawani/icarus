"""Compare evaluation reports across models, without picking one winner.

Roadmap 11.6 asks for "a versioned comparison report [that] identifies the
best model for the current constraints and shows why it won." The second
half of that sentence is the load-bearing one. Every result this project has
produced so far argues against a single combined score: the model that led
on aggregate agreement (Qwen v1, 74.8%) turned out to never propose `land`,
and the deterministic baseline that later beat every model on landing has no
notion of navigation at all. A comparison that flattens those trade-offs into
one number would repeat that mistake by construction.

This produces a table, not a ranking. Each dimension is reported on its own,
each report's provenance (model, quantization, checksum, prompt/contract
version) is carried through, and the write-up is left to the reader who
knows what the "current constraints" actually are for their deployment.
"""

import csv
import json
import time
from pathlib import Path

DIMENSIONS = (
    ("agreement_rate", "agreement", "pct"),
    ("invalid_rate", "invalid", "pct"),
    ("timeout_rate", "timeout", "pct"),
    ("guardrail_rejection_rate", "guardrail reject", "pct"),
    ("recovery_rate", "recovery (proxy)", "pct"),
)


def _get(report, path, default=None):
    node = report
    for key in path:
        if not isinstance(node, dict) or key not in node:
            return default
        node = node[key]
    return node


def _row(report_path):
    report = json.loads(Path(report_path).read_text())
    model = report.get("model") or {}
    totals = report.get("totals") or {}
    latency = report.get("latency") or {}
    tps = report.get("tokens_per_second") or {}
    peaks = report.get("resource_peaks") or {}
    return {
        "report": str(report_path),
        "runtime": report.get("runtime"),
        "model_id": model.get("model_id"),
        "family": model.get("family"),
        "quantization": model.get("quantization"),
        "artifact_sha256": model.get("artifact_sha256"),
        "contract_version": report.get("contract_version"),
        "prompt_version": report.get("prompt_version"),
        "episodes": report.get("episodes"),
        "repeats": report.get("repeats"),
        "decision_points": totals.get("decision_points"),
        "agreement_rate": totals.get("agreement_rate"),
        "invalid_rate": totals.get("invalid_rate"),
        "timeout_rate": totals.get("timeout_rate"),
        "guardrail_rejection_rate": totals.get("guardrail_rejection_rate"),
        "guardrail_checked": totals.get("guardrail_checked"),
        "recovery_rate": totals.get("recovery_rate"),
        "action_count": totals.get("action_count"),
        "cold_start_ms": latency.get("cold_start_ms"),
        "steady_state_median_ms": _get(
            report, ["latency", "steady_state_medians", "median_ms"]),
        "tokens_per_second_median": tps.get("median_tps"),
        "peak_vram_mib": peaks.get("peak_vram_mib"),
        "peak_ram_mib": peaks.get("peak_ram_mib"),
        "deterministic_episodes": report.get("deterministic_episodes"),
    }


def compare(report_paths, output_dir):
    """Build a side-by-side comparison from several evaluation.json reports.

    Every report must come from the same episode corpus for the comparison to
    mean anything; this does not verify that, because it would need to trust
    the per_episode episode-id list rather than anything stronger, and a
    caller comparing genuinely different corpora is a mistake this function
    cannot rule out for them.
    """
    report_paths = [Path(p) for p in report_paths]
    if len(report_paths) < 2:
        raise ValueError("compare needs at least two reports")
    rows = [_row(p) for p in report_paths]

    corpora = {row["episodes"] for row in rows}
    warning = None
    if len(corpora) > 1:
        warning = (f"reports evaluated different numbers of episodes "
                   f"({sorted(corpora)}); this comparison may not be "
                   f"apples to apples")

    output_dir = Path(output_dir)
    output_dir.mkdir(parents=True, exist_ok=True)
    stamp = time.strftime("%Y%m%dT%H%M%S")
    run_dir = output_dir / stamp
    run_dir.mkdir()

    comparison = {
        "schema": "icarus.dcm.comparison.v1",
        "generated_at": time.strftime("%Y-%m-%dT%H:%M:%S%z"),
        "warning": warning,
        "reports": rows,
    }
    (run_dir / "comparison.json").write_text(
        json.dumps(comparison, indent=2) + "\n")

    fieldnames = list(rows[0].keys())
    with (run_dir / "comparison.csv").open("w", newline="") as handle:
        writer = csv.DictWriter(handle, fieldnames=fieldnames)
        writer.writeheader()
        writer.writerows(rows)

    (run_dir / "comparison.txt").write_text(format_comparison(comparison))
    return run_dir, comparison


def format_comparison(comparison):
    """A human-readable table. Deliberately does not compute a winner."""
    rows = comparison["reports"]
    lines = []
    if comparison.get("warning"):
        lines.append(f"WARNING: {comparison['warning']}")
        lines.append("")
    label_width = max(len(row.get("model_id") or row["runtime"] or "?")
                      for row in rows)
    header = f"{'model':<{label_width}}  " + "  ".join(
        f"{label:>16}" for _, label, _ in DIMENSIONS)
    lines.append(header)
    for row in rows:
        label = row.get("model_id") or row["runtime"] or "?"
        cells = []
        for key, _, kind in DIMENSIONS:
            value = row.get(key)
            if value is None:
                cells.append(f"{'n/a':>16}")
            elif kind == "pct":
                cells.append(f"{value:>15.1%} ")
            else:
                cells.append(f"{value:>16}")
        lines.append(f"{label:<{label_width}}  " + "  ".join(cells))
    lines.append("")
    lines.append("latency and cost (not comparable across dimensions above)")
    for row in rows:
        label = row.get("model_id") or row["runtime"] or "?"
        tps = row.get("tokens_per_second_median")
        vram = row.get("peak_vram_mib")
        ram = row.get("peak_ram_mib")
        lines.append(
            f"  {label:<{label_width}}  "
            f"steady {row.get('steady_state_median_ms') or 0:.0f} ms  "
            f"cold {row.get('cold_start_ms') or 0:.0f} ms  "
            + (f"{tps:.1f} tok/s  " if tps is not None else "")
            + (f"{vram:.0f} MiB VRAM  " if vram is not None else "")
            + (f"{ram:.0f} MiB RAM" if ram is not None else ""))
    lines.append("")
    lines.append("provenance")
    for row in rows:
        label = row.get("model_id") or row["runtime"] or "?"
        lines.append(
            f"  {label}: {row.get('quantization')}  "
            f"contract={row.get('contract_version')}  "
            f"prompt={row.get('prompt_version')}  "
            f"sha256={(row.get('artifact_sha256') or 'n/a')[:16]}")
    lines.append("")
    lines.append(
        "No single number here names a winner. Every dimension trades "
        "against another (see docs/architecture/DCM-OBSERVE-V1.md): the "
        "aggregate agreement rate that looked highest earlier hid a model "
        "that never landed, and a deterministic baseline with zero "
        "latency and zero cost cannot navigate at all.")
    return "\n".join(lines)

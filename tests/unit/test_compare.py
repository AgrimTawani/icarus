"""compare() tests.

The point of this module is refusing to collapse a comparison into a single
winner, so the tests check that every dimension survives into the output and
that a corpus mismatch is surfaced rather than silently averaged over.
"""

import csv
import json
import tempfile
import unittest
from pathlib import Path

from python.dcm.compare import compare, format_comparison


def write_report(root, name, **overrides):
    report = {
        "runtime": name, "episodes": 10, "repeats": 3,
        "model": {"model_id": name, "family": "test", "quantization": "q5",
                  "artifact_sha256": "a" * 64},
        "contract_version": "dcm-contract-v2-history",
        "prompt_version": "dcm-prompt-v1",
        "deterministic_episodes": 9,
        "totals": {
            "decision_points": 120, "agreement_rate": 0.75,
            "invalid_rate": 0.0, "timeout_rate": 0.01,
            "guardrail_rejection_rate": 0.0, "guardrail_checked": 100,
            "recovery_rate": 0.5, "action_count": 100,
        },
        "latency": {"cold_start_ms": 5000.0,
                    "steady_state_medians": {"median_ms": 500.0}},
        "tokens_per_second": {"median_tps": 50.0},
        "resource_peaks": {"peak_vram_mib": 4000.0, "peak_ram_mib": 3000.0},
    }
    report["totals"].update(overrides.pop("totals", {}))
    report.update(overrides)
    path = Path(root) / f"{name}.json"
    path.write_text(json.dumps(report))
    return path


class ComparisonContentTests(unittest.TestCase):
    def test_every_reports_dimensions_are_carried_through(self):
        with tempfile.TemporaryDirectory() as root:
            qwen = write_report(root, "qwen")
            llama = write_report(root, "llama",
                                 totals={"agreement_rate": 0.6})
            output, comparison = compare([qwen, llama], root)
            labels = {row["model_id"] for row in comparison["reports"]}
            self.assertEqual(labels, {"qwen", "llama"})
            rates = {row["model_id"]: row["agreement_rate"]
                     for row in comparison["reports"]}
            self.assertEqual(rates["qwen"], 0.75)
            self.assertEqual(rates["llama"], 0.6)
            self.assertTrue((output / "comparison.json").is_file())
            self.assertTrue((output / "comparison.csv").is_file())
            self.assertTrue((output / "comparison.txt").is_file())

    def test_at_least_two_reports_are_required(self):
        with tempfile.TemporaryDirectory() as root:
            only = write_report(root, "qwen")
            with self.assertRaises(ValueError):
                compare([only], root)

    def test_mismatched_corpus_sizes_produce_a_warning_not_silence(self):
        with tempfile.TemporaryDirectory() as root:
            a = write_report(root, "a", episodes=10)
            b = write_report(root, "b", episodes=7)
            _, comparison = compare([a, b], root)
            self.assertIsNotNone(comparison["warning"])
            self.assertIn("10", comparison["warning"])
            self.assertIn("7", comparison["warning"])

    def test_matched_corpus_sizes_produce_no_warning(self):
        with tempfile.TemporaryDirectory() as root:
            a = write_report(root, "a", episodes=10)
            b = write_report(root, "b", episodes=10)
            _, comparison = compare([a, b], root)
            self.assertIsNone(comparison["warning"])

    def test_csv_rows_match_the_reports(self):
        with tempfile.TemporaryDirectory() as root:
            a = write_report(root, "a")
            b = write_report(root, "b")
            output, _ = compare([a, b], root)
            with (output / "comparison.csv").open() as handle:
                rows = list(csv.DictReader(handle))
            self.assertEqual({row["model_id"] for row in rows}, {"a", "b"})

    def test_missing_fields_render_as_not_available_not_a_crash(self):
        with tempfile.TemporaryDirectory() as root:
            a = write_report(root, "a")
            b = write_report(root, "b", totals={"guardrail_rejection_rate": None})
            _, comparison = compare([a, b], root)
            text = format_comparison(comparison)
            self.assertIn("n/a", text)

    def test_the_report_does_not_declare_a_winner(self):
        # The module's own caveat legitimately says "no winner"; what must
        # never appear is an actual declaration ranking the models.
        with tempfile.TemporaryDirectory() as root:
            a = write_report(root, "a")
            b = write_report(root, "b")
            _, comparison = compare([a, b], root)
            text = format_comparison(comparison)
            for banned in ("the winner is", "wins overall", "best model is",
                          "ranked #1", "recommended model"):
                self.assertNotIn(banned, text.lower())


if __name__ == "__main__":
    unittest.main()

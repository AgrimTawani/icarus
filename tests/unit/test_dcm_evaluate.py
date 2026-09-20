"""Scoring tests.

The scoring rules exist because of specific observed results, so the tests
pin those rules rather than the arithmetic: vocabulary mismatches must not be
counted as model failures, stale refusals must not be counted as model
latency, and agreement must be reported against comparable points only.
"""

import tempfile
import unittest
from pathlib import Path

from python.dcm.evaluate import (
    evaluate,
    format_report,
    per_action_breakdown,
    score_decisions,
    summarize_latency,
)
from python.dcm.observe import MockRuntime


def decision(recorded, proposed=None, status="valid", latency_ms=1000.0):
    return {
        "recorded_action": recorded,
        "proposal": {"action": proposed, "arguments": {}} if proposed else None,
        "status": status,
        "latency_ms": latency_ms,
    }


class ScoringTests(unittest.TestCase):
    def test_agreement_counts_only_comparable_points(self):
        scored = score_decisions([
            decision("arm", "arm"),
            decision("land", "takeoff"),
            # execute_route is a Drone API action the contract does not
            # expose, so the model could not have agreed and this point
            # is unscoreable rather than a miss.
            decision("execute_route", "hold"),
        ])
        self.assertEqual(scored["comparable_points"], 2)
        self.assertEqual(scored["agreed_points"], 1)
        self.assertEqual(scored["agreement_rate"], 0.5)
        self.assertEqual(scored["unscoreable_points"], 1)
        self.assertEqual(scored["unscoreable_actions"], ["execute_route"])

    def test_unscoreable_points_do_not_reduce_agreement(self):
        agreeing = [decision("arm", "arm"), decision("land", "land")]
        with_mismatch = [*agreeing, decision("execute_route", "hold")]
        self.assertEqual(score_decisions(agreeing)["agreement_rate"], 1.0)
        self.assertEqual(score_decisions(with_mismatch)["agreement_rate"], 1.0)

    def test_invalid_proposals_are_not_agreement(self):
        scored = score_decisions([
            decision("arm", None, status="invalid"),
            decision("arm", "arm"),
        ])
        self.assertEqual(scored["comparable_points"], 1)
        self.assertEqual(scored["agreement_rate"], 1.0)
        self.assertEqual(scored["invalid_rate"], 0.5)

    def test_stale_refusals_are_excluded_from_rates_and_latency(self):
        scored = score_decisions([
            decision("arm", "arm", latency_ms=4000.0),
            decision("land", None, status="stale", latency_ms=0.0),
            decision("land", "land", latency_ms=1000.0),
        ])
        # A stale point never reaches the runtime, so its zero latency is not
        # a measurement of the model and must not flatter the median.
        self.assertEqual(scored["asked"], 2)
        self.assertEqual(scored["first_decision_latency_ms"], 4000.0)
        self.assertEqual(scored["steady_latency"]["count"], 1)
        self.assertEqual(scored["steady_latency"]["median_ms"], 1000.0)
        self.assertEqual(scored["counts"]["stale"], 1)
        # Rates are over decisions actually put to the model.
        self.assertEqual(scored["invalid_rate"], 0.0)

    def test_first_decision_is_separated_from_steady_state(self):
        scored = score_decisions([
            decision("arm", "arm", latency_ms=7204.0),
            decision("land", "land", latency_ms=1200.0),
            decision("land", "land", latency_ms=1100.0),
        ])
        self.assertEqual(scored["first_decision_latency_ms"], 7204.0)
        self.assertEqual(scored["steady_latency"]["max_ms"], 1200.0)
        self.assertEqual(scored["steady_latency"]["median_ms"], 1150.0)

    def test_first_asked_decision_is_first_even_after_a_stale_refusal(self):
        scored = score_decisions([
            decision("land", None, status="stale", latency_ms=0.0),
            decision("arm", "arm", latency_ms=5000.0),
            decision("land", "land", latency_ms=1200.0),
        ])
        self.assertEqual(scored["first_decision_latency_ms"], 5000.0)
        self.assertEqual(scored["steady_latency"]["count"], 1)

    def test_timeouts_and_errors_are_counted_separately(self):
        scored = score_decisions([
            decision("arm", None, status="timeout"),
            decision("arm", None, status="error"),
            decision("arm", "arm"),
        ])
        self.assertAlmostEqual(scored["timeout_rate"], 1 / 3)
        self.assertAlmostEqual(scored["error_rate"], 1 / 3)
        self.assertEqual(scored["comparable_points"], 1)

    def test_all_stale_yields_no_rates_rather_than_zero(self):
        scored = score_decisions([decision("arm", None, status="stale",
                                           latency_ms=0.0)])
        # Reporting 0% invalid for a model that was never asked would be a
        # false clean bill of health.
        self.assertIsNone(scored["invalid_rate"])
        self.assertIsNone(scored["agreement_rate"])
        self.assertIsNone(scored["steady_latency"])
        self.assertEqual(scored["asked"], 0)


class LatencySummaryTests(unittest.TestCase):
    def test_empty_sample_has_no_summary(self):
        self.assertIsNone(summarize_latency([]))

    def test_spread_is_reported_not_just_a_single_number(self):
        stats = summarize_latency([4257.0, 4563.0, 7204.0])
        self.assertEqual(stats["count"], 3)
        self.assertEqual(stats["min_ms"], 4257.0)
        self.assertEqual(stats["max_ms"], 7204.0)
        self.assertEqual(stats["median_ms"], 4563.0)

    def test_single_value_still_reports_a_range(self):
        stats = summarize_latency([1200.0])
        self.assertEqual(stats["min_ms"], stats["max_ms"])


class ReportFormattingTests(unittest.TestCase):
    def report(self, **overrides):
        base = {
            "runtime": "llama_cpp:test", "episodes": 2, "repeats": 3,
            "executed_actions": 0, "deterministic_episodes": 2,
            "totals": {
                "counts": {"valid": 10, "invalid": 0, "timeout": 1,
                           "error": 0, "stale": 2},
                "decision_points": 13, "invalid_rate": 0.0,
                "timeout_rate": 1 / 11, "error_rate": 0.0,
                "stale_refusals": 2, "comparable_points": 8,
                "agreement_rate": 0.75, "unscoreable_points": 2,
                "guardrail_checked": 8, "guardrail_rejected": 1,
                "guardrail_unchecked": 0, "guardrail_rejection_rate": 0.125,
                "guardrail_rejection_reasons": ["REASON_CODE_NOT_ARMED"],
            },
            "latency": {
                "cold_start_ms": 7204.0,
                "episode_first_decision": summarize_latency([4257.0]),
                "steady_state_medians": summarize_latency([1200.0]),
            },
        }
        base.update(overrides)
        return base

    def test_report_states_nothing_was_executed(self):
        text = format_report(self.report())
        self.assertIn("executed actions", text)
        self.assertIn("0", text.split("executed actions")[1])

    def test_report_shows_agreement_against_comparable_points(self):
        text = format_report(self.report())
        self.assertIn("75.0% of 8 comparable points", text)
        self.assertIn("outside the vocabulary", text)

    def test_missing_rates_render_as_not_available(self):
        report = self.report()
        report["totals"]["invalid_rate"] = None
        report["latency"]["steady_state_medians"] = None
        text = format_report(report)
        self.assertIn("n/a", text)


class PerActionBreakdownTests(unittest.TestCase):
    def episode(self, name, *runs):
        return {"episode": name,
                "runs": [{"proposals": proposals} for proposals in runs]}

    def proposal(self, recorded, proposed, status="valid"):
        return {"recorded": recorded, "proposed": proposed, "status": status}

    def test_pairs_expose_a_failure_the_aggregate_hides(self):
        # Three land points answered takeoff still yields a high aggregate
        # agreement when arm and takeoff are perfect. The breakdown is the
        # only place that failure is visible.
        runs = [[self.proposal("arm", "arm"),
                 self.proposal("takeoff", "takeoff"),
                 self.proposal("land", "takeoff")]]
        breakdown = per_action_breakdown([self.episode("e1", *runs)])
        pairs = {(r["recorded"], r["proposed"]): r["count"]
                 for r in breakdown["pairs"]}
        self.assertEqual(pairs[("land", "takeoff")], 1)
        self.assertFalse(next(r["agrees"] for r in breakdown["pairs"]
                              if r["recorded"] == "land"))

    def test_situations_count_decision_points_not_repeats(self):
        # Three repeats of one situation are not three pieces of evidence.
        same = [self.proposal("land", "hold")]
        breakdown = per_action_breakdown(
            [self.episode("e1", same, same, same)])
        self.assertEqual(breakdown["coverage"]["land"]["situations"], 1)
        self.assertEqual(breakdown["coverage"]["land"]["always_agreed"], 0)

    def test_always_agreed_requires_every_repeat_to_agree(self):
        agree = [self.proposal("land", "land")]
        flip = [self.proposal("land", "hold")]
        steady = per_action_breakdown([self.episode("e1", agree, agree)])
        wobbly = per_action_breakdown([self.episode("e1", agree, flip)])
        self.assertEqual(steady["coverage"]["land"]["always_agreed"], 1)
        self.assertEqual(wobbly["coverage"]["land"]["always_agreed"], 0)

    def test_unscoreable_and_invalid_points_are_excluded(self):
        runs = [[self.proposal("execute_route", "hold"),
                 self.proposal("arm", None, status="invalid"),
                 self.proposal("arm", "arm")]]
        breakdown = per_action_breakdown([self.episode("e1", *runs)])
        self.assertNotIn("execute_route", breakdown["coverage"])
        self.assertEqual(breakdown["coverage"]["arm"]["situations"], 1)

    def test_distinct_situations_are_keyed_by_episode_and_position(self):
        runs = [[self.proposal("land", "land")]]
        breakdown = per_action_breakdown(
            [self.episode("e1", *runs), self.episode("e2", *runs)])
        self.assertEqual(breakdown["coverage"]["land"]["situations"], 2)


class UnusableEpisodeTests(unittest.TestCase):
    def test_a_broken_episode_is_skipped_rather_than_aborting_the_campaign(self):
        # A corpus accumulates episodes that predate a schema change. One of
        # them must not cost the whole evaluation.
        with tempfile.TemporaryDirectory() as root:
            broken = Path(root) / "broken"
            broken.mkdir()
            (broken / "manifest.json").write_text("{}")
            (broken / "events.jsonl").write_text("")
            output, report = evaluate(
                [broken], MockRuntime(), Path(root) / "out", repeats=2)
            self.assertEqual(report["episodes"], 0)
            self.assertEqual(report["episodes_requested"], 1)
            self.assertEqual(len(report["skipped_episodes"]), 1)
            self.assertEqual(report["skipped_episodes"][0]["episode"], "broken")
            self.assertEqual(report["executed_actions"], 0)
            self.assertTrue((output / "evaluation.json").is_file())

    def test_the_skip_is_visible_in_the_human_summary(self):
        with tempfile.TemporaryDirectory() as root:
            broken = Path(root) / "broken"
            broken.mkdir()
            (broken / "manifest.json").write_text("{}")
            (broken / "events.jsonl").write_text("")
            _, report = evaluate([broken], MockRuntime(), Path(root) / "out",
                                 repeats=1)
            text = format_report(report)
            self.assertIn("skipped episodes", text)
            self.assertIn("broken", text)

    def test_evaluating_nothing_is_an_error_not_an_empty_pass(self):
        with tempfile.TemporaryDirectory() as root, \
                self.assertRaises(ValueError):
            evaluate([], MockRuntime(), Path(root) / "out")

    def test_repeats_must_be_positive(self):
        with tempfile.TemporaryDirectory() as root, \
                self.assertRaises(ValueError):
            evaluate([Path(root)], MockRuntime(), Path(root) / "out", repeats=0)


if __name__ == "__main__":
    unittest.main()

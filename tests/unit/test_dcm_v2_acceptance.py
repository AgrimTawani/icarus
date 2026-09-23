import json
import tempfile
import unittest
from pathlib import Path

import jsonschema

from python.dcm.contract import ACTIONS, curate_v3, validate_decision
from python.dcm.health import DcmHealthMachine
from python.dcm.v2_context import MAX_CONTEXT_BYTES, build_context, load_macros
from python.dcm.v2_feedback import aggregate, review_proposals
from python.dcm.v2_profiles import expected_body_manifest, verify_body_manifest

ROOT = Path(__file__).resolve().parents[2]


class V2AcceptanceTests(unittest.TestCase):
    def test_generated_context_validates_and_is_bounded(self):
        context = curate_v3(
            {"armed": False, "landed": True, "observed_at_unix_ms": 10},
            {"overall": "HEALTH_LEVEL_HEALTHY", "observed_at_unix_ms": 10,
             "local_map_age_ms": 0}, None, "survey", {"seq": 1,
                                                        "unix_ms": 10})
        schema = json.loads((ROOT / "config/dcm/schema/context-v3.schema.json").read_text())
        jsonschema.validate(context, schema)
        self.assertLessEqual(context["context_budget"]["payload_bytes"],
                             MAX_CONTEXT_BYTES)
        self.assertEqual(context["source_evidence"]["perception_age_ms"], 0)

    def test_context_budget_fails_closed(self):
        with self.assertRaisesRegex(ValueError, "limit"):
            build_context({"state": {}, "perception": {},
                           "mission": "x" * MAX_CONTEXT_BYTES})

    def test_stale_body_manifest_is_rejected(self):
        with tempfile.TemporaryDirectory() as temp:
            path = Path(temp) / "BODY.json"
            stale = expected_body_manifest()
            stale["body"]["allowed_actions"] = []
            path.write_text(json.dumps(stale))
            with self.assertRaisesRegex(ValueError, "stale"):
                verify_body_manifest(path)

    def test_macro_library_has_three_way_scenarios_and_valid_actions(self):
        macros = load_macros(ROOT / "config/dcm/macros", ACTIONS)
        self.assertGreaterEqual(len(macros), 3)
        self.assertLessEqual(len(macros), 5)
        for macro in macros:
            self.assertTrue(macro["preconditions"])
            self.assertTrue(macro["evidence_links"])
            fixture = json.loads((ROOT / macro["evidence_links"][0]).read_text())
            self.assertEqual(fixture["macro_id"], macro["id"])
            self.assertEqual({case["kind"] for case in fixture["cases"]},
                             {"nominal", "adverse", "recovery"})
            for case in fixture["cases"]:
                self.assertLessEqual(set(case["expected_actions"]),
                                     set(macro["allowed_actions"]))

    def test_rationale_is_bounded_and_never_part_of_executable_arguments(self):
        proposal, rationale = validate_decision(json.dumps(
            {"action": "hold", "arguments": {"duration_ms": 1000},
             "rationale": "fresh state; bounded hold"}), allow_rationale=True)
        self.assertNotIn("rationale", proposal["arguments"])
        self.assertEqual(rationale, "fresh state; bounded hold")
        with self.assertRaises(ValueError):
            validate_decision(json.dumps(
                {"action": "hold", "arguments": {},
                 "rationale": "x" * 121}), allow_rationale=True)

    def test_health_circuit_opens_and_resets_deterministically(self):
        machine = DcmHealthMachine()
        self.assertEqual(machine.transition("timeout", airborne=True).current, "late")
        self.assertEqual(machine.transition("invalid", airborne=True).current, "invalid")
        opened = machine.transition("error", airborne=True)
        self.assertEqual(opened.current, "circuit-open")
        self.assertTrue(opened.circuit_opened)
        self.assertEqual(opened.fallback_action, "hold")
        self.assertEqual(machine.transition("error", airborne=True).fallback_action,
                         "return_home")
        self.assertEqual(machine.transition("valid", airborne=True).current, "healthy")

    def test_feedback_report_and_untrusted_queue_are_reproducible(self):
        rows = [{"proposal": {"action": "hold"}, "executed": True,
                 "outcome": "FAILED", "latency_ms": 10}] * 3
        first = aggregate(rows)
        second = aggregate(rows)
        self.assertEqual(first, second)
        queue = review_proposals(first, hypothesis="change safety limit")
        self.assertFalse(queue["model_hypothesis_trusted"])
        self.assertEqual(queue["proposals"][0]["requested_action"], "human_review")


if __name__ == "__main__":
    unittest.main()

import tempfile
import unittest
from pathlib import Path

from python.dcm.contract import (
    ACTIONS,
    CONTRACT_VERSION_CONTEXT,
    curate_v3,
    material_state_change,
    render_prompt,
)
from python.dcm.v2_context import (
    CONTEXT_VERSION,
    MACRO_SCHEMA,
    bearing_sector,
    build_context,
    finalize_context,
    load_macros,
    select_macro,
)
from python.dcm.v2_feedback import FEEDBACK_SCHEMA, aggregate


class V2ContextTests(unittest.TestCase):
    def test_bearing_sectors_are_stable(self):
        self.assertEqual(bearing_sector(0), "N")
        self.assertEqual(bearing_sector(45), "NE")
        self.assertEqual(bearing_sector(-90), "W")
        self.assertIsNone(bearing_sector(True))

    def test_context_is_deterministic_and_only_enriches_curated_input(self):
        observation = {
            "mission": "survey", "state": {"armed": False, "landed": True,
                                                  "battery": {"remaining_percent": 90}},
            "perception": {"nearest_obstacle_bearing_deg": 90,
                           "nearest_obstacle_distance_m": 3.12345,
                           "raw_lidar": ["must-not-be-used"]},
            "actions_completed": [{"action": "hold", "outcome": "FAILED"},
                                  {"action": "hold", "outcome": "FAILED"}],
        }
        first = build_context(observation)
        second = build_context(observation)
        self.assertEqual(first["context_version"], CONTEXT_VERSION)
        self.assertEqual(first["context_sha256"], second["context_sha256"])
        self.assertEqual(first["perception_summary"]["obstacle_sectors"]["E"]
                         ["nearest_distance_m"], 3.123)
        self.assertEqual(first["failure_signals"]["consecutive_failures"], 2)
        self.assertEqual(first["failure_signals"]["repeated_action"], "hold")
        self.assertNotIn("raw_lidar", first["perception_summary"])

    def test_macro_selection_requires_explicit_id(self):
        with tempfile.TemporaryDirectory() as temp:
            path = Path(temp) / "macro.md"
            path.write_text(
                "<!-- icarus-macro-meta\n"
                + '{"schema":"' + MACRO_SCHEMA + '","id":"m","revision":"1",'
                + '"tags":["test"],"allowed_actions":["hold"]}\n-->\n# M\n',
                encoding="utf-8")
            catalog = load_macros(temp, ACTIONS)
            self.assertIsNone(select_macro(catalog, {}))
            self.assertEqual(select_macro(catalog, {"macro_id": "m"})["id"], "m")
            with self.assertRaisesRegex(ValueError, "unavailable"):
                select_macro(catalog, {"macro_id": "unknown"})

    def test_reviewed_macro_is_selected_deterministically_from_intent(self):
        root = Path(__file__).resolve().parents[2]
        catalog = load_macros(root / "config/dcm/macros", ACTIONS)
        selected = select_macro(
            catalog, mission="Arm, inspect that building, hover and count people")
        self.assertEqual(selected["id"], "building-inspection")
        self.assertEqual(selected["selection_source"], "deterministic-intent")

    def test_final_hash_covers_late_model_visible_fields(self):
        context = build_context({"mission": "survey", "state": {},
                                 "perception": {}})
        first = finalize_context({**context, "mission_obligations": {
            "pending": ["flight"]}})
        second = finalize_context({**context, "mission_obligations": {
            "pending": ["return_home"]}})
        self.assertNotEqual(first["context_sha256"], second["context_sha256"])

    def test_v3_curation_keeps_v1_allow_list_and_adds_profile_provenance(self):
        context = curate_v3(
            {"armed": False, "landed": True, "forbidden": "no"},
            {"nearest_obstacle_bearing_deg": 1,
             "nearest_obstacle_distance_m": 2, "raw": "no"},
            None, "m", {"seq": 1, "unix_ms": 1000}, history=[])
        self.assertEqual(context["contract_version"], CONTRACT_VERSION_CONTEXT)
        self.assertNotIn("forbidden", context["state"])
        self.assertNotIn("raw", context["perception"])
        self.assertTrue(context["body"]["generated"])
        self.assertIn("sha256", context["soul"])
        self.assertIn("sha256", context["world_facts"])
        self.assertTrue(context["context_sha256"])

    def test_v3_system_prompt_has_explicit_identity_and_capability_sections(self):
        context = curate_v3(
            {"armed": False, "landed": True}, {}, None, "survey",
            {"seq": 1, "unix_ms": 1000}, history=[],
            runtime_body={"simulator": "airsim"})
        prompt = render_prompt(context)
        self.assertIn("REVIEWED SOUL", prompt["system"])
        self.assertIn("GENERATED BODY", prompt["system"])
        self.assertIn("REVIEWED WORLD FACTS", prompt["system"])
        self.assertIn('"simulator": "airsim"', prompt["system"])

    def test_v3_material_recheck_uses_compact_state_summary(self):
        before = {"state": {}, "state_summary": {
            "armed": True, "landed": False, "relative_altitude_m": 3.0}}
        after = {"state": {}, "state_summary": {
            "armed": True, "landed": False, "relative_altitude_m": 4.0}}
        self.assertIn("altitude changed", material_state_change(before, after))


class V2FeedbackTests(unittest.TestCase):
    def test_feedback_is_evidence_only_summary(self):
        report = aggregate([
            {"proposal": {"action": "hold"}, "executed": True,
             "outcome": "SUCCEEDED", "mission_macro": {"id": "m", "revision": "1"}},
            {"proposal": {"action": "hold"}, "guardrail": {"checked": True,
                                                                  "would_execute": False}},
        ])
        self.assertEqual(report["schema"], FEEDBACK_SCHEMA)
        self.assertEqual(report["by_action"]["hold"]["proposed"], 2)
        self.assertEqual(report["by_action"]["hold"]["guardrail_rejected"], 1)
        self.assertEqual(report["by_macro"]["m@1"]["succeeded"], 1)

"""Contract tests: what a model may say, see, and be refused.

The handoff requires coverage of invalid JSON, unknown actions, unsafe
arguments, stale data and model failure. Each has a class below.
"""

import json
import tempfile
import unittest
from pathlib import Path

from python.dcm.contract import (
    ACTIONS,
    ALLOWED_ACTIONS,
    CONTRACT_VERSION,
    CONTRACT_VERSION_HISTORY,
    FRESHNESS_LIMITS,
    HISTORY_LIMIT,
    RuntimeDescriptor,
    assess_freshness,
    curate,
    render_prompt,
    render_vocabulary,
    validate_proposal,
)


class InvalidJsonTests(unittest.TestCase):
    def test_non_object_and_malformed_text_are_rejected(self):
        for raw in ("", "not json", "[]", "null", "42", '"arm"',
                    '{"action":"arm","arguments":{}', '{}',
                    '{"action":"arm"}', '{"arguments":{}}',
                    '{"action":"arm","arguments":{},"extra":1}'):
            with self.subTest(raw=raw), self.assertRaises(ValueError):
                validate_proposal(raw)

    def test_non_string_input_raises_value_error_not_type_error(self):
        # The observer classifies ValueError as an invalid proposal and
        # anything else as a runtime failure. A model returning a dict is a
        # bad proposal, not a crash, so this must not raise TypeError.
        for raw in (None, 42, {"action": "arm"}, b'{"action":"arm"}'):
            with self.subTest(raw=raw), self.assertRaises(ValueError):
                validate_proposal(raw)

    def test_prose_around_json_is_rejected(self):
        for raw in ('Sure! {"action":"none","arguments":{}}',
                    '```json\n{"action":"none","arguments":{}}\n```',
                    '{"action":"none","arguments":{}} Let me know if...'):
            with self.subTest(raw=raw), self.assertRaises(ValueError):
                validate_proposal(raw)


class UnknownActionTests(unittest.TestCase):
    def test_actions_outside_the_vocabulary_are_rejected(self):
        for action in ("launch_missile", "goto", "Arm", "ARM", "",
                       "takeoff ", "disarm", "orbit"):
            raw = json.dumps({"action": action, "arguments": {}})
            with self.subTest(action=action), self.assertRaises(ValueError):
                validate_proposal(raw)

    def test_non_string_action_is_rejected(self):
        for action in (None, 1, True, ["arm"], {"arm": 1}):
            raw = json.dumps({"action": action, "arguments": {}})
            with self.subTest(action=action), self.assertRaises(ValueError):
                validate_proposal(raw)

    def test_every_declared_action_has_a_valid_minimal_form(self):
        minimal = {
            "takeoff": {"target_altitude_agl_m": 3.0},
            "goto": {"north_m": 1.0, "east_m": 1.0, "altitude_agl_m": 3.0},
            "orbit": {"center_north_m": 1.0, "center_east_m": 1.0,
                      "radius_m": 5.0, "altitude_agl_m": 3.0},
        }
        for action, specification in ACTIONS.items():
            arguments = minimal.get(action, {})
            with self.subTest(action=action):
                proposal = validate_proposal(
                    json.dumps({"action": action, "arguments": arguments}))
                self.assertEqual(proposal["action"], action)
                self.assertTrue(
                    all(not spec["required"] for name, spec
                        in specification.items() if name not in arguments))


class UnsafeArgumentTests(unittest.TestCase):
    def test_altitude_outside_bounds_is_rejected(self):
        for altitude in (0, 0.49, 10.01, 100, -3, 1e9):
            raw = json.dumps({"action": "takeoff",
                              "arguments": {"target_altitude_agl_m": altitude}})
            with self.subTest(altitude=altitude), self.assertRaises(ValueError):
                validate_proposal(raw)

    def test_altitude_at_the_bounds_is_accepted(self):
        for altitude in (0.5, 3, 10, 10.0):
            raw = json.dumps({"action": "takeoff",
                              "arguments": {"target_altitude_agl_m": altitude}})
            with self.subTest(altitude=altitude):
                self.assertEqual(
                    validate_proposal(raw)["arguments"]["target_altitude_agl_m"],
                    altitude)

    def test_non_finite_and_wrongly_typed_numbers_are_rejected(self):
        for raw in ('{"action":"takeoff","arguments":{"target_altitude_agl_m":true}}',
                    '{"action":"takeoff","arguments":{"target_altitude_agl_m":"3"}}',
                    '{"action":"takeoff","arguments":{"target_altitude_agl_m":null}}',
                    '{"action":"takeoff","arguments":{"target_altitude_agl_m":NaN}}',
                    '{"action":"takeoff","arguments":{"target_altitude_agl_m":Infinity}}'):
            with self.subTest(raw=raw), self.assertRaises(ValueError):
                validate_proposal(raw)

    def test_booleans_are_not_accepted_as_integers(self):
        # bool subclasses int, so an isinstance check would let True through
        # as a one-millisecond hold.
        raw = '{"action":"hold","arguments":{"duration_ms":true}}'
        with self.assertRaises(ValueError):
            validate_proposal(raw)

    def test_hold_duration_bounds_and_optionality(self):
        self.assertEqual(
            validate_proposal('{"action":"hold","arguments":{}}')["arguments"], {})
        self.assertEqual(
            validate_proposal(
                '{"action":"hold","arguments":{"duration_ms":60000}}'
            )["arguments"]["duration_ms"], 60000)
        for duration in (0, -1, 60001, 1.5):
            raw = json.dumps({"action": "hold",
                              "arguments": {"duration_ms": duration}})
            with self.subTest(duration=duration), self.assertRaises(ValueError):
                validate_proposal(raw)

    def test_missing_required_and_unexpected_arguments_are_rejected(self):
        for raw in ('{"action":"takeoff","arguments":{}}',
                    '{"action":"arm","arguments":{"target_altitude_agl_m":3}}',
                    '{"action":"land","arguments":{"force":true}}',
                    ('{"action":"takeoff","arguments":'
                     '{"target_altitude_agl_m":3,"speed":9}}')):
            with self.subTest(raw=raw), self.assertRaises(ValueError):
                validate_proposal(raw)

    def test_arguments_must_be_an_object(self):
        for arguments in ("[]", "3", '"x"', "null", "true"):
            raw = '{"action":"arm","arguments":' + arguments + "}"
            with self.subTest(arguments=arguments), self.assertRaises(ValueError):
                validate_proposal(raw)


class NavigationActionTests(unittest.TestCase):
    """goto and orbit widen the argument surface, so bound them explicitly.

    Every bound here is tighter than config/safety/v1.yaml: a 30 m ceiling,
    100 m from home and 2.5 m minimum clearance. The contract should refuse
    before the guardrails have to.
    """

    def goto(self, north=10.0, east=10.0, altitude=5.0):
        return json.dumps({"action": "goto", "arguments": {
            "north_m": north, "east_m": east, "altitude_agl_m": altitude}})

    def orbit(self, **overrides):
        arguments = {"center_north_m": 10.0, "center_east_m": 10.0,
                     "radius_m": 5.0, "altitude_agl_m": 5.0}
        arguments.update(overrides)
        return json.dumps({"action": "orbit", "arguments": arguments})

    def test_a_reasonable_goto_is_accepted(self):
        proposal = validate_proposal(self.goto())
        self.assertEqual(proposal["arguments"]["north_m"], 10.0)

    def test_goto_stays_inside_the_geofence_radius(self):
        # The policy allows 100 m from home; the corner of this range is
        # sqrt(50^2 + 50^2) = 70.7 m, so no in-bounds goto can breach it.
        for north, east in ((50.0, 50.0), (-50.0, -50.0)):
            with self.subTest(north=north, east=east):
                validate_proposal(self.goto(north, east))
        for north in (50.1, -50.1, 500.0):
            with self.subTest(north=north), self.assertRaises(ValueError):
                validate_proposal(self.goto(north=north))

    def test_goto_altitude_stays_under_the_ceiling(self):
        # The policy ceiling is 30 m; the contract stops at 25 m.
        validate_proposal(self.goto(altitude=25.0))
        for altitude in (25.1, 30.0, 50.0, 0.4, -5.0):
            with self.subTest(altitude=altitude), self.assertRaises(ValueError):
                validate_proposal(self.goto(altitude=altitude))

    def test_goto_requires_a_full_destination(self):
        for arguments in ({"north_m": 1.0}, {"north_m": 1.0, "east_m": 1.0},
                          {"east_m": 1.0, "altitude_agl_m": 5.0}, {}):
            raw = json.dumps({"action": "goto", "arguments": arguments})
            with self.subTest(arguments=arguments), self.assertRaises(ValueError):
                validate_proposal(raw)

    def test_orbit_radius_clears_the_minimum_separation(self):
        # minimum_clearance_m is 2.5, so an orbit radius below 3 m is refused.
        validate_proposal(self.orbit(radius_m=3.0))
        for radius in (2.9, 2.5, 0.0, -5.0, 15.1, 100.0):
            with self.subTest(radius=radius), self.assertRaises(ValueError):
                validate_proposal(self.orbit(radius_m=radius))

    def test_orbit_revolutions_are_optional_but_bounded(self):
        self.assertNotIn("revolutions",
                         validate_proposal(self.orbit())["arguments"])
        validate_proposal(self.orbit(revolutions=0.25))
        validate_proposal(self.orbit(revolutions=3.0))
        for revolutions in (0.0, -1.0, 3.1, 100.0):
            with self.subTest(revolutions=revolutions), self.assertRaises(ValueError):
                validate_proposal(self.orbit(revolutions=revolutions))

    def test_orbit_reach_stays_inside_the_geofence(self):
        # Worst case is the corner of the centre range plus the largest
        # radius: sqrt(40^2 + 40^2) + 15 = 71.6 m, inside the 100 m policy.
        import math as _math
        worst = _math.hypot(40.0, 40.0) + 15.0
        self.assertLess(worst, 100.0)
        validate_proposal(self.orbit(center_north_m=40.0, center_east_m=40.0,
                                     radius_m=15.0))
        for centre in (40.1, -40.1):
            with self.subTest(centre=centre), self.assertRaises(ValueError):
                validate_proposal(self.orbit(center_north_m=centre))

    def test_navigation_actions_appear_in_the_generated_vocabulary(self):
        vocabulary = render_vocabulary()
        self.assertIn('"goto"', vocabulary)
        self.assertIn('"orbit"', vocabulary)
        self.assertIn("north_m", vocabulary)
        self.assertIn("radius_m", vocabulary)


class StaleDataTests(unittest.TestCase):
    def observation(self, state_age_ms=0, map_age_ms=0):
        now = 1_789_827_075_269
        return {
            "observed_at_unix_ms": now,
            "state": {"observed_at_unix_ms": now - state_age_ms},
            "perception": {"local_map_age_ms": map_age_ms},
        }

    def test_fresh_observation_is_accepted(self):
        self.assertIsNone(assess_freshness(self.observation(46, 28)))

    def test_stale_state_is_refused(self):
        limit = FRESHNESS_LIMITS["state_age_ms"]
        self.assertIsNone(assess_freshness(self.observation(limit, 0)))
        reason = assess_freshness(self.observation(limit + 1, 0))
        self.assertIn("state is", reason)

    def test_stale_perception_is_refused(self):
        # 811 ms is the perception age recorded by the real safety event in
        # episode 20260919T194115, which must be refused.
        reason = assess_freshness(self.observation(0, 811))
        self.assertIn("perception map is", reason)
        self.assertIsNone(
            assess_freshness(
                self.observation(0, FRESHNESS_LIMITS["perception_age_ms"])))

    def test_protobuf_int64_string_timestamps_are_accepted(self):
        # Protobuf's canonical JSON mapping renders int64 as a string, so real
        # episodes carry observed_at_unix_ms as "1789827075223". Rejecting
        # those silently refused every decision in a real episode.
        now = 1_789_827_075_269
        self.assertIsNone(assess_freshness({
            "observed_at_unix_ms": str(now),
            "state": {"observed_at_unix_ms": str(now - 46)},
            "perception": {"local_map_age_ms": 28},
        }))
        reason = assess_freshness({
            "observed_at_unix_ms": str(now),
            "state": {"observed_at_unix_ms": str(now - 5000)},
            "perception": {"local_map_age_ms": 28},
        })
        self.assertIn("state is", reason)

    def test_non_numeric_strings_are_not_timestamps(self):
        for value in ("", "abc", "12.5", "1e9", None, True, 12.5, []):
            with self.subTest(value=value):
                self.assertIsNotNone(assess_freshness({
                    "observed_at_unix_ms": value,
                    "state": {"observed_at_unix_ms": 1},
                }))

    def test_missing_timestamps_are_treated_as_stale(self):
        for observation in ({}, {"observed_at_unix_ms": 1},
                            {"observed_at_unix_ms": 1, "state": {}},
                            {"observed_at_unix_ms": None,
                             "state": {"observed_at_unix_ms": 1}},
                            {"observed_at_unix_ms": 1,
                             "state": {"observed_at_unix_ms": 1},
                             "perception": {"overall": "ok"}}):
            with self.subTest(observation=observation):
                self.assertIsNotNone(assess_freshness(observation))


class CurationTests(unittest.TestCase):
    def test_authority_and_identifiers_never_reach_the_model(self):
        state = {
            "sequence": "1", "observed_at_unix_ms": 10, "flight_phase": "X",
            "authority": {"lease_id": "secret"}, "vehicle_id": "secret-vehicle",
            "position": {"latitude_deg": 1.0},
        }
        observation = curate(state, {"sequence": "2", "overall": "ok"},
                             None, "unit", {"seq": 3, "unix_ms": 11})
        rendered = json.dumps(observation)
        self.assertNotIn("secret", rendered)
        self.assertNotIn("authority", observation["state"])
        self.assertNotIn("vehicle_id", observation["state"])
        self.assertEqual(observation["contract_version"], CONTRACT_VERSION)
        self.assertEqual(observation["allowed_actions"], list(ALLOWED_ACTIONS))

    def test_curation_does_not_accept_the_recorded_next_action(self):
        # The recorded action is comparison data. If it could be curated in,
        # it could leak into the prompt, and every evaluation would be void.
        self.assertNotIn("recorded_action", curate.__code__.co_varnames)


class ObservationHistoryTests(unittest.TestCase):
    def event(self):
        return {"seq": 1, "unix_ms": 1}

    def test_v1_observation_has_no_history_and_says_so(self):
        observation = curate({}, {}, None, "m", self.event())
        self.assertNotIn("actions_completed", observation)
        self.assertEqual(observation["contract_version"], CONTRACT_VERSION)

    def test_v2_observation_carries_completed_actions(self):
        history = [{"action": "arm", "outcome": "SUCCEEDED"},
                   {"action": "takeoff", "outcome": "SUCCEEDED"}]
        observation = curate({}, {}, None, "m", self.event(), history=history)
        self.assertEqual(observation["contract_version"],
                         CONTRACT_VERSION_HISTORY)
        self.assertEqual([h["action"] for h in observation["actions_completed"]],
                         ["arm", "takeoff"])

    def test_an_empty_history_still_selects_v2(self):
        # An empty list means "nothing done yet", which is information. Only
        # None means "this contract does not show history at all".
        observation = curate({}, {}, None, "m", self.event(), history=[])
        self.assertEqual(observation["contract_version"],
                         CONTRACT_VERSION_HISTORY)
        self.assertEqual(observation["actions_completed"], [])

    def test_history_is_bounded_so_a_long_mission_cannot_crowd_out_state(self):
        history = [{"action": "hold", "outcome": "SUCCEEDED"}
                   for _ in range(HISTORY_LIMIT + 20)]
        observation = curate({}, {}, None, "m", self.event(), history=history)
        self.assertEqual(len(observation["actions_completed"]), HISTORY_LIMIT)

    def test_history_is_copied_not_aliased(self):
        # The observer mutates its running list as the episode replays; a
        # recorded observation must not change underneath the report.
        history = [{"action": "arm", "outcome": "SUCCEEDED"}]
        observation = curate({}, {}, None, "m", self.event(), history=history)
        history.append({"action": "land", "outcome": "SUCCEEDED"})
        self.assertEqual(len(observation["actions_completed"]), 1)


class PromptTests(unittest.TestCase):
    def test_prompt_vocabulary_is_generated_from_the_validator_table(self):
        vocabulary = render_vocabulary()
        for action in ALLOWED_ACTIONS:
            self.assertIn('"' + action + '"', vocabulary)
        self.assertIn("0.5", vocabulary)
        self.assertIn("10.0", vocabulary)

    def test_prompt_carries_its_version_and_the_observation(self):
        observation = {"observed_at_unix_ms": 1, "state": {}}
        prompt = render_prompt(observation)
        self.assertEqual(prompt["prompt_version"], "dcm-prompt-v1")
        self.assertIn("exactly one JSON object", prompt["system"])
        self.assertIn("observed_at_unix_ms", prompt["user"])


class RuntimeDescriptorTests(unittest.TestCase):
    def manifest(self, root, sha):
        path = Path(root) / "MANIFEST.json"
        path.write_text(json.dumps({
            "repository": "test/repo",
            "llama_cpp_revision": "1af554f8fc78ba029665a47b839484d9763e2a75",
            "artifacts": [{
                "role": "primary", "path": str(Path(root) / "model.gguf"),
                "quantization": "q5_k_m", "sha256": sha,
            }],
        }))
        return path

    def test_checksum_mismatch_fails_loudly(self):
        with tempfile.TemporaryDirectory() as root:
            weights = Path(root) / "model.gguf"
            weights.write_bytes(b"real weights")
            descriptor = RuntimeDescriptor.from_manifest(
                self.manifest(root, "0" * 64))
            with self.assertRaises(ValueError) as caught:
                descriptor.verify_artifact()
            self.assertIn("checksum mismatch", str(caught.exception))

    def test_matching_checksum_verifies(self):
        import hashlib
        with tempfile.TemporaryDirectory() as root:
            weights = Path(root) / "model.gguf"
            weights.write_bytes(b"real weights")
            sha = hashlib.sha256(b"real weights").hexdigest()
            descriptor = RuntimeDescriptor.from_manifest(self.manifest(root, sha))
            self.assertEqual(descriptor.verify_artifact(), sha)
            self.assertEqual(descriptor.quantization, "q5_k_m")
            self.assertEqual(descriptor.contract_version, CONTRACT_VERSION)

    def test_missing_artifact_is_reported(self):
        with tempfile.TemporaryDirectory() as root:
            descriptor = RuntimeDescriptor.from_manifest(
                self.manifest(root, "0" * 64))
            with self.assertRaises(FileNotFoundError):
                descriptor.verify_artifact()

    def test_unknown_role_is_rejected(self):
        with tempfile.TemporaryDirectory() as root, \
                self.assertRaises(ValueError):
            RuntimeDescriptor.from_manifest(
                self.manifest(root, "0" * 64), role="tertiary")

    def test_descriptor_records_everything_needed_to_reproduce(self):
        with tempfile.TemporaryDirectory() as root:
            record = RuntimeDescriptor.from_manifest(
                self.manifest(root, "0" * 64)).as_record()
            for key in ("model_id", "quantization", "artifact_sha256",
                        "runtime", "runtime_revision", "temperature",
                        "context_length", "deadline_ms", "prompt_version",
                        "contract_version", "vocabulary_version"):
                self.assertIn(key, record)

    def test_family_defaults_to_qwen_when_unstated(self):
        # Every manifest before setup-vision-runtime holds one family and
        # never named it; this default must not silently change for them.
        with tempfile.TemporaryDirectory() as root:
            descriptor = RuntimeDescriptor.from_manifest(
                self.manifest(root, "0" * 64))
            self.assertEqual(descriptor.family, "qwen")

    def test_a_manifest_can_name_a_different_family_per_artifact(self):
        # setup-vision-runtime's manifest holds a second model family
        # alongside the detector, so family must come from the artifact
        # rather than being assumed for the whole manifest.
        with tempfile.TemporaryDirectory() as root:
            path = Path(root) / "MANIFEST.json"
            path.write_text(json.dumps({
                "llama_cpp_revision": "1af554f8fc78ba029665a47b839484d9763e2a75",
                "artifacts": [
                    {"role": "primary", "family": "qwen",
                     "path": str(Path(root) / "qwen.gguf"),
                     "quantization": "q5_k_m", "sha256": "0" * 64},
                    {"role": "second_family", "family": "llama",
                     "path": str(Path(root) / "llama.gguf"),
                     "quantization": "q4_k_m", "sha256": "1" * 64},
                ],
            }))
            primary = RuntimeDescriptor.from_manifest(path, role="primary")
            second = RuntimeDescriptor.from_manifest(path, role="second_family")
            self.assertEqual(primary.family, "qwen")
            self.assertEqual(second.family, "llama")


if __name__ == "__main__":
    unittest.main()

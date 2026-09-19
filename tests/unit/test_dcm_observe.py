import hashlib
import json
import tempfile
import time
import unittest
from pathlib import Path

from python.dcm.observe import MockRuntime, observe_episode, validate_proposal


class BadRuntime:
    name = "invalid-test"

    def propose(self, observation):
        return '{"action":"launch_missile","arguments":{}}'


class SlowRuntime:
    name = "slow-test"

    def propose(self, observation):
        time.sleep(0.01)
        return '{"action":"arm","arguments":{}}'


class ObserveTests(unittest.TestCase):
    def make_episode(self, root):
        episode = Path(root) / "example"
        episode.mkdir()
        events = [
            ("episode_start", {"mission": "unit"}),
            ("state", {"sequence": "1", "flight_phase": "FLIGHT_PHASE_DISARMED",
                       "authority": {"lease_id": "secret"}}),
            ("perception", {"sequence": "1", "path_ahead_clear": True}),
            ("guardrail_validation", {
                "state": {"sequence": "1", "flight_phase": "FLIGHT_PHASE_DISARMED",
                          "authority": {"lease_id": "secret"}},
                "command": {"arm": {"context": {"request_id": "secret"}}},
            }),
            ("action_request", {"method": "Arm", "request": {}}),
            ("action_receipt", {"method": "Arm", "receipt": {}}),
            ("action_status", {"action_id": "a", "state": "ACTION_STATE_SUCCEEDED",
                               "type": "ACTION_TYPE_ARM"}),
            ("episode_end", {"outcome": "passed"}),
        ]
        counts = {}
        stream = episode / "events.jsonl"
        with stream.open("w") as output:
            for seq, (kind, payload) in enumerate(events, 1):
                counts[kind] = counts.get(kind, 0) + 1
                output.write(json.dumps({
                    "seq": seq, "unix_ms": 1000 + seq,
                    "monotonic_ns": seq * 1_000_000,
                    "kind": kind, "payload": payload,
                }) + "\n")
        (episode / "manifest.json").write_text(json.dumps({
            "schema": "icarus.episode.v1", "episode_id": "example",
            "mission": "unit", "source": "simulation", "outcome": "passed",
            "config_sha256": {}, "stream_errors": [],
            "streams": {"events.jsonl": {
                "sha256": hashlib.sha256(stream.read_bytes()).hexdigest(),
                "records": len(events), "counts": counts,
            }},
        }))
        return episode

    def test_strict_schema(self):
        self.assertEqual(validate_proposal(
            '{"action":"takeoff","arguments":{"target_altitude_agl_m":3}}'
        )["action"], "takeoff")
        for value in (
                "not json", '{"action":"arm","arguments":{},"extra":1}',
                '{"action":"takeoff","arguments":{"target_altitude_agl_m":true}}',
                '{"action":"hold","arguments":{"duration_ms":0}}',
                '{"action":"goto","arguments":{}}',
                '{"action":"arm","arguments":{"x":1}}'):
            with self.subTest(value=value), self.assertRaises(ValueError):
                validate_proposal(value)

    def test_observe_logs_without_execution_or_authority_leak(self):
        with tempfile.TemporaryDirectory() as temporary:
            episode = self.make_episode(temporary)
            original = (episode / "events.jsonl").read_bytes()
            output, summary = observe_episode(
                episode, MockRuntime(), Path(temporary) / "reports",
                check_guardrails=False)
            decision = json.loads((output / "decisions.jsonl").read_text())
            self.assertEqual(summary["counts"]["valid"], 1)
            self.assertEqual(summary["executed_actions"], 0)
            self.assertFalse(decision["executed"])
            self.assertEqual(decision["recorded_action"], "arm")
            self.assertNotIn("authority", decision["observation"]["state"])
            self.assertNotIn("secret", json.dumps(decision["observation"]))
            self.assertEqual((episode / "events.jsonl").read_bytes(), original)

    def test_invalid_and_late_responses_are_not_proposals(self):
        with tempfile.TemporaryDirectory() as temporary:
            episode = self.make_episode(temporary)
            for runtime, expected in ((BadRuntime(), "invalid"),
                                      (SlowRuntime(), "timeout")):
                output, summary = observe_episode(
                    episode, runtime, Path(temporary) / "reports",
                    timeout_ms=1, check_guardrails=False)
                decision = json.loads((output / "decisions.jsonl").read_text())
                self.assertEqual(summary["counts"][expected], 1)
                self.assertIsNone(decision["proposal"])


if __name__ == "__main__":
    unittest.main()

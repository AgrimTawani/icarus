"""Flight-loop tests.

This is the first code path where a proposal can reach an aircraft, so these
tests are about refusal rather than capability: what must *not* execute, and
when the loop must stop asking.

No Drone API, simulator or model is involved. The client is a fake that
records what it was asked to do.
"""

import io
import sys
import unittest
from pathlib import Path
from unittest.mock import patch

# fly_mission builds typed Drone API requests, so it needs the generated
# protobuf bindings. No earlier unit test has, and there is no conftest, so
# the path is added here rather than assumed. Run ./scripts/generate-proto if
# this import fails.
sys.path.insert(0, str(Path(__file__).resolve().parents[2]
                       / "build/generated/python"))

from python.dcm.contract import DeadlineExceeded
from python.dcm.fly import (
    MissionMemory,
    approve,
    fly_mission,
    session_episode_summary,
    summarize,
)

FRESH = 1_789_827_075_000


class FakeTerminal:
    def __init__(self, state="ACTION_STATE_SUCCEEDED", message=""):
        self.state = state
        self.message = message


class FakeActionApi:
    def __init__(self, outcome="ACTION_STATE_SUCCEEDED"):
        self.calls = []
        self.outcome = outcome

    def __getattr__(self, method):
        def call(request, timeout=None):
            self.calls.append(method)
            return request
        return call


class FakeClient:
    """Enough of MissionClient to drive the loop, and nothing more."""

    def __init__(self, outcome="ACTION_STATE_SUCCEEDED", age_ms=40):
        self.action_api = FakeActionApi(outcome)
        self.channel = None
        self.session_id = "s"
        self.episode = FakeEpisode()
        self.outcome = outcome
        self.age_ms = age_ms
        self.contexts = []

    def state(self):
        return self

    def context(self, name):
        # Faithful to MissionClient: a real CommandContext whose idempotency
        # key is derived from the name, so a reused name is a detectable bug.
        from icarus.v1 import action_pb2
        self.contexts.append(name)
        return action_pb2.CommandContext(
            request_id="req", idempotency_key=f"trace:{name}",
            vehicle_id="icarus-01", client_id="test")

    def wait_action(self, receipt, timeout):
        return FakeTerminal(self.outcome)


class FakeEpisode:
    def __init__(self):
        self.records = []

    def record(self, kind, payload):
        self.records.append((kind, payload))


class ScriptedRuntime:
    """Returns each queued response in turn, then proposes `none`."""

    name = "scripted"

    def __init__(self, *responses):
        self.queue = list(responses)
        self.asked = 0

    def propose(self, observation):
        self.asked += 1
        if not self.queue:
            return '{"action":"none","arguments":{}}'
        response = self.queue.pop(0)
        if isinstance(response, Exception):
            raise response
        return response


def patched_observe(monkeypatch_target, age_ms=40):
    """Build an observation the freshness rule accepts or rejects on demand."""
    def _observe(client, memory, index):
        return {
            "observed_at_unix_ms": FRESH,
            "state": {"observed_at_unix_ms": FRESH - age_ms,
                      "flight_phase": "FLIGHT_PHASE_AIRBORNE"},
            "perception": {"local_map_age_ms": 20},
            "mission": memory.mission,
            "actions_completed": memory.as_history(),
        }
    return _observe


class ApprovalTests(unittest.TestCase):
    def test_bare_enter_declines(self):
        # An operator who is not paying attention must not launch an aircraft
        # by pressing return.
        for answer in ("\n", "", "no\n", "maybe\n", "Y E S\n", "1\n"):
            with self.subTest(answer=answer):
                self.assertFalse(
                    approve({"action": "arm", "arguments": {}}, "approval",
                            io.StringIO(answer)))

    def test_explicit_yes_approves(self):
        for answer in ("y\n", "Y\n", "yes\n", " yes \n", "YES\n"):
            with self.subTest(answer=answer):
                self.assertTrue(
                    approve({"action": "arm", "arguments": {}}, "approval",
                            io.StringIO(answer)))

    def test_autonomous_mode_does_not_consult_the_operator(self):
        empty = io.StringIO("")
        self.assertTrue(
            approve({"action": "arm", "arguments": {}}, "autonomous", empty))


class ExecutionGateTests(unittest.TestCase):
    def setUp(self):
        from python.dcm import fly
        self.fly = fly
        self.original = fly.observe
        fly.observe = patched_observe(fly)

    def tearDown(self):
        self.fly.observe = self.original

    def run_mission(self, runtime, mode="approval", answers="", **kwargs):
        client = FakeClient(**kwargs)
        result = fly_mission(client, runtime, "test mission", mode=mode,
                             stream=io.StringIO(answers), echo=lambda *_: None,
                             max_decisions=6)
        return client, result

    def test_a_declined_proposal_never_reaches_the_drone_api(self):
        runtime = ScriptedRuntime('{"action":"arm","arguments":{}}')
        client, result = self.run_mission(runtime, answers="n\n")
        self.assertEqual(client.action_api.calls, [])
        self.assertEqual(result["counts"]["declined"], 1)
        self.assertEqual(result["counts"]["executed"], 0)
        self.assertEqual(result["executed"], [])

    def test_an_approved_proposal_reaches_the_matching_rpc(self):
        runtime = ScriptedRuntime('{"action":"arm","arguments":{}}')
        client, result = self.run_mission(runtime, answers="y\n")
        self.assertEqual(client.action_api.calls, ["Arm"])
        self.assertEqual(result["counts"]["executed"], 1)

    def test_an_invalid_proposal_never_reaches_the_drone_api(self):
        runtime = ScriptedRuntime('{"action":"launch_missile","arguments":{}}')
        client, result = self.run_mission(runtime, answers="y\ny\n")
        self.assertEqual(client.action_api.calls, [])
        self.assertEqual(result["counts"]["invalid"], 1)

    def test_a_timed_out_proposal_never_reaches_the_drone_api(self):
        runtime = ScriptedRuntime(DeadlineExceeded("too slow"))
        client, result = self.run_mission(runtime, answers="y\ny\n")
        self.assertEqual(client.action_api.calls, [])
        self.assertEqual(result["counts"]["timeout"], 1)

    def test_every_vocabulary_action_has_a_dispatch(self):
        cases = {
            "arm": ("Arm", {}),
            "takeoff": ("Takeoff", {"target_altitude_agl_m": 3.0}),
            "hold": ("Hold", {"duration_ms": 1000}),
            "land": ("Land", {}),
            "return_home": ("ReturnHome", {}),
            "goto": ("Goto", {"north_m": 5.0, "east_m": 5.0,
                              "altitude_agl_m": 4.0}),
            "orbit": ("Orbit", {"center_north_m": 5.0, "center_east_m": 5.0,
                                "radius_m": 5.0, "altitude_agl_m": 4.0}),
        }
        import json as _json
        for action, (method, arguments) in cases.items():
            with self.subTest(action=action):
                runtime = ScriptedRuntime(_json.dumps(
                    {"action": action, "arguments": arguments}))
                client, _ = self.run_mission(runtime, answers="y\n" * 4)
                self.assertIn(method, client.action_api.calls)

    def test_each_action_gets_a_distinct_idempotency_name(self):
        # context() derives the idempotency key from this name, so reusing one
        # would silently turn a second identical action into a no-op.
        runtime = ScriptedRuntime('{"action":"hold","arguments":{}}',
                                  '{"action":"hold","arguments":{}}')
        client, _ = self.run_mission(runtime, answers="y\ny\ny\n")
        self.assertEqual(len(client.contexts), len(set(client.contexts)))

    def test_landing_ends_the_mission(self):
        runtime = ScriptedRuntime('{"action":"land","arguments":{}}',
                                  '{"action":"arm","arguments":{}}')
        client, result = self.run_mission(runtime, answers="y\ny\n")
        self.assertEqual(client.action_api.calls, ["Land"])
        self.assertEqual(result["counts"]["executed"], 1)

    def test_proposing_none_ends_the_mission_without_acting(self):
        runtime = ScriptedRuntime('{"action":"none","arguments":{}}')
        client, result = self.run_mission(runtime, answers="")
        self.assertEqual(client.action_api.calls, [])
        self.assertEqual(result["counts"]["executed"], 0)

    def test_a_failed_action_is_recorded_and_does_not_end_the_mission(self):
        runtime = ScriptedRuntime('{"action":"arm","arguments":{}}',
                                  '{"action":"none","arguments":{}}')
        with patch("python.dcm.fly.time.sleep") as sleep:
            _, result = self.run_mission(
                runtime, answers="y\n", outcome="ACTION_STATE_ABORTED_BY_SAFETY")
        self.assertEqual(result["counts"]["failed"], 1)
        self.assertEqual(result["executed"][0]["outcome"],
                         "ABORTED_BY_SAFETY")
        sleep.assert_called_once_with(0.5)


class EpisodeRecordingTests(unittest.TestCase):
    def setUp(self):
        from python.dcm import fly
        self.fly = fly
        self.original = fly.observe
        fly.observe = patched_observe(fly)

    def tearDown(self):
        self.fly.observe = self.original

    def test_prompt_and_response_are_recorded_for_replay(self):
        # Phase 10 requires model prompts and responses to be replayable
        # alongside state and actions.
        runtime = ScriptedRuntime('{"action":"arm","arguments":{}}')
        client = FakeClient()
        fly_mission(client, runtime, "test", mode="approval",
                    stream=io.StringIO("y\n"), echo=lambda *_: None,
                    max_decisions=2)
        kinds = [kind for kind, _ in client.episode.records]
        self.assertIn("dcm_decision", kinds)
        payload = next(p for k, p in client.episode.records
                       if k == "dcm_decision")
        self.assertIn("prompt", payload)
        self.assertIn("system", payload["prompt"])
        self.assertEqual(payload["raw_response"],
                         '{"action":"arm","arguments":{}}')
        self.assertTrue(payload["executed"])

    def test_declined_decisions_are_recorded_as_not_executed(self):
        runtime = ScriptedRuntime('{"action":"arm","arguments":{}}')
        client = FakeClient()
        fly_mission(client, runtime, "test", mode="approval",
                    stream=io.StringIO("n\n"), echo=lambda *_: None,
                    max_decisions=2)
        payload = next(p for k, p in client.episode.records
                       if k == "dcm_decision")
        self.assertEqual(payload["status"], "declined")
        self.assertFalse(payload["executed"])


class MissionMemoryTests(unittest.TestCase):
    def test_memory_carries_completed_actions_in_order(self):
        memory = MissionMemory("fly north")
        memory.remember("arm", "SUCCEEDED")
        memory.remember("takeoff", "SUCCEEDED")
        self.assertEqual([entry["action"] for entry in memory.as_history()],
                         ["arm", "takeoff"])

    def test_history_is_a_copy(self):
        memory = MissionMemory("m")
        memory.remember("arm", "SUCCEEDED")
        snapshot = memory.as_history()
        memory.remember("land", "SUCCEEDED")
        self.assertEqual(len(snapshot), 1)


class EpisodeSealSummaryTests(unittest.TestCase):
    def test_failed_action_marks_the_session_episode_failed(self):
        status, score = session_episode_summary([{
            "mission": "m", "counts": {"failed": 1}, "executed": [],
        }])
        self.assertEqual(status, "failed")
        self.assertEqual(score["missions"][0]["mission"], "m")

    def test_declined_action_is_retained_without_claiming_a_flight_failure(self):
        status, score = session_episode_summary([{
            "mission": "m", "counts": {"declined": 1, "failed": 0},
        }])
        self.assertEqual(status, "completed")
        self.assertNotIn("error", score)

    def test_session_exception_marks_the_episode_failed(self):
        status, score = session_episode_summary([], RuntimeError("server died"))
        self.assertEqual(status, "failed")
        self.assertEqual(score["error"]["type"], "RuntimeError")

    def test_cli_sets_outcome_before_client_close_seals_the_episode(self):
        """The live wrapper, not only the summary helper, owns the seal path."""
        from python.dcm import fly_cli

        instances = []

        class ClosingClient:
            def __init__(self, *_):
                self.episode_outcome = "completed"
                self.episode_score = None
                self.closed = False
                instances.append(self)

            def connect(self):
                pass

            def acquire(self):
                pass

            def close(self):
                # This represents MissionClient.close(), which seals using
                # these two fields.
                self.closed = True

        result = {
            "mission": "m", "mode": "approval",
            "counts": {"failed": 1}, "executed": [], "model": None,
        }
        with patch.object(fly_cli, "_load_mission_client", return_value=ClosingClient), \
             patch.object(fly_cli, "build_runtime", return_value=(object(), None)), \
             patch.object(fly_cli, "fly_mission", return_value=result), \
             patch.object(sys, "argv", ["dcm-fly", "--runtime", "mock", "--mission", "m"]):
            self.assertEqual(fly_cli.main(), 0)

        self.assertEqual(len(instances), 1)
        client = instances[0]
        self.assertTrue(client.closed)
        self.assertEqual(client.episode_outcome, "failed")
        self.assertEqual(client.episode_score["missions"], [result])


class ModeValidationTests(unittest.TestCase):
    def test_an_unknown_mode_is_refused(self):
        with self.assertRaises(ValueError):
            fly_mission(FakeClient(), ScriptedRuntime(), "m", mode="yolo")

    def test_summary_names_what_executed(self):
        text = summarize({"mission": "m", "mode": "approval",
                          "counts": {"executed": 0},
                          "executed": [], "model": None})
        self.assertIn("nothing", text)


if __name__ == "__main__":
    unittest.main()

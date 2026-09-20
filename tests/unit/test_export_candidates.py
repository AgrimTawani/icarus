"""export_candidates refuses to export an evaluation episode by default.

replay() is patched out so these test the flagging and refusal logic on its
own, without needing a synthetic episode that also satisfies the C++
guardrail-replay checks byte for byte.
"""

import json
import tempfile
import unittest
from pathlib import Path
from unittest import mock

from python.dataset_tools.export_candidates import candidates, main


def write_episode(root, name, mission="takeoff_hover_land"):
    episode = Path(root) / name
    episode.mkdir()
    events = [
        ("state", {}),
        ("perception", {}),
        ("action_request", {"method": "Arm", "request": {}}),
        ("action_receipt", {"receipt": {"action_id": "a"}}),
        ("action_status", {"action_id": "a", "state": "ACTION_STATE_SUCCEEDED"}),
    ]
    with (episode / "events.jsonl").open("w") as stream:
        for seq, (kind, payload) in enumerate(events, 1):
            stream.write(json.dumps(
                {"seq": seq, "kind": kind, "payload": payload}) + "\n")
    (episode / "manifest.json").write_text(json.dumps(
        {"episode_id": name, "mission": mission}))
    return episode


class FlaggingTests(unittest.TestCase):
    def test_an_episode_in_the_registry_is_flagged_as_evaluation(self):
        with tempfile.TemporaryDirectory() as root, \
                mock.patch("python.dataset_tools.export_candidates.replay"):
            episode = write_episode(root, "ep-a")
            result = candidates(episode, evaluation_episode_ids={"ep-a"})
            self.assertTrue(all(item["evaluation_episode"] for item in result))

    def test_an_episode_not_in_the_registry_is_not_flagged(self):
        with tempfile.TemporaryDirectory() as root, \
                mock.patch("python.dataset_tools.export_candidates.replay"):
            episode = write_episode(root, "ep-b")
            result = candidates(episode, evaluation_episode_ids=set())
            self.assertFalse(any(item["evaluation_episode"] for item in result))

    def test_the_acceptance_substring_still_flags_independently(self):
        # Kept as a second signal for missions named before the registry
        # existed, or produced outside evaluate().
        with tempfile.TemporaryDirectory() as root, \
                mock.patch("python.dataset_tools.export_candidates.replay"):
            episode = write_episode(root, "ep-c", mission="v1_acceptance_suite")
            result = candidates(episode, evaluation_episode_ids=set())
            self.assertTrue(all(item["evaluation_episode"] for item in result))


class RefusalTests(unittest.TestCase):
    def run_main(self, root, episode_name, extra_args=()):
        output = Path(root) / "out.jsonl"
        argv = [str(Path(root) / episode_name), "--output", str(output),
               *extra_args]
        with mock.patch("sys.argv", ["export_candidates", *argv]):
            try:
                main()
                return 0, output
            except SystemExit as exit_:
                return exit_.code, output

    def test_an_evaluation_episode_is_refused_by_default(self):
        with tempfile.TemporaryDirectory() as root, \
                mock.patch("python.dataset_tools.export_candidates.replay"), \
                mock.patch(
                    "python.dataset_tools.export_candidates."
                    "known_evaluation_episode_ids", return_value={"ep-a"}):
            write_episode(root, "ep-a")
            code, output = self.run_main(root, "ep-a")
            self.assertNotEqual(code, 0)
            self.assertFalse(output.exists())

    def test_the_override_flag_allows_it_through(self):
        with tempfile.TemporaryDirectory() as root, \
                mock.patch("python.dataset_tools.export_candidates.replay"), \
                mock.patch(
                    "python.dataset_tools.export_candidates."
                    "known_evaluation_episode_ids", return_value={"ep-a"}):
            write_episode(root, "ep-a")
            code, output = self.run_main(
                root, "ep-a", extra_args=["--include-evaluation-episodes"])
            self.assertEqual(code, 0)
            self.assertTrue(output.exists())

    def test_a_non_evaluation_episode_exports_without_a_flag(self):
        with tempfile.TemporaryDirectory() as root, \
                mock.patch("python.dataset_tools.export_candidates.replay"), \
                mock.patch(
                    "python.dataset_tools.export_candidates."
                    "known_evaluation_episode_ids", return_value=set()):
            write_episode(root, "ep-z")
            code, output = self.run_main(root, "ep-z")
            self.assertEqual(code, 0)
            self.assertTrue(output.exists())


if __name__ == "__main__":
    unittest.main()

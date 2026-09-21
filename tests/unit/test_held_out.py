"""known_evaluation_episode_ids tests.

The registry exists because the substring heuristic it replaced,
"acceptance" in mission, cannot see any DCM evaluation run: those missions
are named things like takeoff_hover_land or an operator's own typed text.
These tests check the registry actually reads what evaluate() writes, and
degrades safely when reports are missing or malformed.
"""

import json
import tempfile
import unittest
from pathlib import Path

from python.dcm.held_out import (
    frozen_held_out_episode_ids,
    known_evaluation_episode_ids,
    protected_evaluation_episode_ids,
)


def write_report(root, name, episode_ids):
    report_dir = Path(root) / name
    report_dir.mkdir(parents=True)
    (report_dir / "evaluation.json").write_text(json.dumps({
        "per_episode": [{"episode": episode_id} for episode_id in episode_ids],
    }))


class RegistryTests(unittest.TestCase):
    def test_episodes_from_a_real_report_are_found(self):
        with tempfile.TemporaryDirectory() as root:
            write_report(root, "run1", ["ep-a", "ep-b"])
            ids = known_evaluation_episode_ids(root)
            self.assertEqual(ids, {"ep-a", "ep-b"})

    def test_episodes_are_unioned_across_multiple_reports(self):
        with tempfile.TemporaryDirectory() as root:
            write_report(root, "run1", ["ep-a"])
            write_report(root, "run2", ["ep-b", "ep-c"])
            ids = known_evaluation_episode_ids(root)
            self.assertEqual(ids, {"ep-a", "ep-b", "ep-c"})

    def test_a_missing_root_returns_an_empty_set_not_an_error(self):
        self.assertEqual(
            known_evaluation_episode_ids("/nonexistent/path"), set())

    def test_a_malformed_report_is_skipped_not_fatal(self):
        with tempfile.TemporaryDirectory() as root:
            broken = Path(root) / "broken"
            broken.mkdir()
            (broken / "evaluation.json").write_text("not json")
            write_report(root, "good", ["ep-a"])
            # The broken report must not hide the good one, and must not
            # raise: one corrupt report should not block every other export.
            self.assertEqual(known_evaluation_episode_ids(root), {"ep-a"})

    def test_a_report_with_no_per_episode_key_yields_nothing(self):
        with tempfile.TemporaryDirectory() as root:
            report_dir = Path(root) / "run1"
            report_dir.mkdir()
            (report_dir / "evaluation.json").write_text(json.dumps({}))
            self.assertEqual(known_evaluation_episode_ids(root), set())

    def test_frozen_split_is_added_even_before_an_evaluation_runs(self):
        with tempfile.TemporaryDirectory() as root:
            split = Path(root) / "split.json"
            split.write_text(json.dumps({
                "schema": "icarus.dcm.held-out-split.v1",
                "training_episode_ids": ["train-a"],
                "held_out_episode_ids": ["held-a"],
            }))
            self.assertEqual(frozen_held_out_episode_ids(split), {"held-a"})
            self.assertEqual(protected_evaluation_episode_ids(
                Path(root) / "reports", split), {"held-a"})

    def test_overlapping_frozen_split_is_refused(self):
        with tempfile.TemporaryDirectory() as root:
            split = Path(root) / "split.json"
            split.write_text(json.dumps({
                "schema": "icarus.dcm.held-out-split.v1",
                "training_episode_ids": ["same"],
                "held_out_episode_ids": ["same"],
            }))
            with self.assertRaisesRegex(ValueError, "both training"):
                frozen_held_out_episode_ids(split)


if __name__ == "__main__":
    unittest.main()

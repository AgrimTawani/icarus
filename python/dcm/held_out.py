"""Which episodes have been used for model evaluation.

Roadmap 11.6 requires preventing evaluation episodes from entering training
data. The prior mechanism in `export_candidates.py` was a substring check on
the mission name, `"acceptance" in mission`, which would not catch any of the
DCM evaluation corpus: those missions are named things like
`takeoff_hover_land` or the operator's own typed text, neither containing
"acceptance". A heuristic that cannot see its own evaluation runs is worse
than useless, since it reports a clean answer while being wrong.

This instead reads the actual evaluation reports evaluate() already writes:
every episode that has ever appeared in a `per_episode` entry of a report
under `logs/dcm/evaluation/` has been evaluated, and that is definitive
rather than inferred.
"""

import json
from pathlib import Path

DEFAULT_EVALUATION_ROOT = (
    Path(__file__).resolve().parents[2] / "logs/dcm/evaluation")
DEFAULT_SPLIT_PATH = (Path(__file__).resolve().parents[2]
                      / "config/evaluation/dcm-v1-held-out.json")


def frozen_held_out_episode_ids(split_path=DEFAULT_SPLIT_PATH):
    """Read the explicit pre-training split, failing closed when malformed."""
    path = Path(split_path)
    if not path.is_file():
        return set()
    try:
        split = json.loads(path.read_text())
    except (OSError, json.JSONDecodeError) as error:
        raise ValueError("held-out split is unreadable") from error
    if split.get("schema") != "icarus.dcm.held-out-split.v1":
        raise ValueError("unknown held-out split schema")
    held_out = split.get("held_out_episode_ids")
    training = split.get("training_episode_ids")
    if not isinstance(held_out, list) or not isinstance(training, list):
        raise ValueError("held-out split lists are missing")
    if not held_out or any(not isinstance(item, str) or not item for item in held_out):
        raise ValueError("held-out split is empty or invalid")
    if set(held_out) & set(training):
        raise ValueError("episode appears in both training and held-out split")
    return set(held_out)


def known_evaluation_episode_ids(evaluation_root=DEFAULT_EVALUATION_ROOT):
    """Every episode name that appears in any evaluation report on disk.

    Missing or unreadable reports are skipped rather than raising: a
    corrupt one earlier report must not silently make every later episode
    look untouched, but it also must not block export of everything else.
    """
    root = Path(evaluation_root)
    episode_ids = set()
    if not root.is_dir():
        return episode_ids
    for report_path in root.glob("*/evaluation.json"):
        try:
            report = json.loads(report_path.read_text())
        except (OSError, json.JSONDecodeError):
            continue
        for entry in report.get("per_episode", []):
            episode_id = entry.get("episode")
            if episode_id:
                episode_ids.add(episode_id)
    return episode_ids


def protected_evaluation_episode_ids(evaluation_root=DEFAULT_EVALUATION_ROOT,
                                     split_path=DEFAULT_SPLIT_PATH):
    """Union dynamic report provenance with the frozen hold-out policy."""
    return known_evaluation_episode_ids(evaluation_root) | frozen_held_out_episode_ids(split_path)

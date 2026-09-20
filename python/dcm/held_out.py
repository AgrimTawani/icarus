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

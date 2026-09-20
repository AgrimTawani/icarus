"""Extract unapproved decision candidates from a verified flight episode."""

import argparse
import json
from pathlib import Path

from python.dataset_tools.replay import TERMINAL, replay
from python.dcm.held_out import known_evaluation_episode_ids


def candidates(episode, evaluation_episode_ids=None):
    replay(episode)
    manifest = json.loads((episode / "manifest.json").read_text())
    if evaluation_episode_ids is None:
        evaluation_episode_ids = known_evaluation_episode_ids()
    # The substring check is kept as a second, independent signal: a mission
    # explicitly named as an acceptance run is flagged even if it was never
    # actually passed through evaluate(), such as before this registry
    # existed. The registry is authoritative where it has an answer.
    is_evaluation = (episode.name in evaluation_episode_ids
                     or "acceptance" in manifest["mission"])
    state = None
    perception = None
    pending = []
    terminal = {}
    for line in (episode / "events.jsonl").read_text().splitlines():
        event = json.loads(line)
        kind = event["kind"]
        data = event["payload"]
        if kind == "state":
            state = data
        elif kind == "perception":
            perception = data
        elif kind == "action_request":
            pending.append({
                "episode_id": manifest["episode_id"],
                "event_seq": event["seq"],
                "mission": manifest["mission"],
                "observation": {"state": state, "perception": perception},
                "proposed_action": data,
                "training_status": "unreviewed_do_not_train",
                "human_approved": False,
                "evaluation_episode": is_evaluation,
            })
        elif kind == "action_receipt":
            pending[-1]["action_id"] = data["receipt"].get("action_id")
        elif kind == "action_status" and data["state"] in TERMINAL:
            terminal[data["action_id"]] = data
    for item in pending:
        item["outcome"] = terminal.get(item.get("action_id"))
    return pending


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("episode", type=Path)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument(
        "--include-evaluation-episodes", action="store_true",
        help="export candidates from an episode used for model evaluation "
             "too; refused by default so held-out scenarios cannot leak "
             "into a training export by accident")
    args = parser.parse_args()
    result = candidates(args.episode)
    if result and result[0]["evaluation_episode"] and not args.include_evaluation_episodes:
        parser.error(
            f"{args.episode.name} was used for model evaluation "
            "(logs/dcm/evaluation/); refusing to export it as training "
            "data. Pass --include-evaluation-episodes to override.")
    if args.output.exists():
        parser.error("output already exists; candidate exports are write-once")
    args.output.parent.mkdir(parents=True, exist_ok=True)
    with args.output.open("x") as output:
        for item in result:
            output.write(json.dumps(item, sort_keys=True) + "\n")
    print(f"Exported {len(result)} unapproved candidates to {args.output}")


if __name__ == "__main__":
    main()

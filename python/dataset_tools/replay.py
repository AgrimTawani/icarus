"""Offline integrity and action/safety replay for sealed Icarus episodes."""

import argparse
import hashlib
import json
import subprocess
from collections import Counter, defaultdict
from pathlib import Path

TERMINAL = {
    "ACTION_STATE_REJECTED", "ACTION_STATE_SUCCEEDED",
    "ACTION_STATE_CANCELLED", "ACTION_STATE_TIMED_OUT",
    "ACTION_STATE_FAILED", "ACTION_STATE_PREEMPTED",
    "ACTION_STATE_ABORTED_BY_SAFETY",
}


class ReplayError(ValueError):
    pass


def digest(path):
    value = hashlib.sha256()
    with path.open("rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            value.update(chunk)
    return value.hexdigest()


def replay(directory, check_guardrails=True):
    directory = Path(directory)
    manifest = json.loads((directory / "manifest.json").read_text())
    if manifest.get("schema") != "icarus.episode.v1":
        raise ReplayError("unknown episode schema")
    if manifest.get("episode_id") != directory.name:
        raise ReplayError("episode ID does not match directory")
    for name, expected in manifest.get("config_sha256", {}).items():
        if name.startswith("/") or ".." in Path(name).parts:
            raise ReplayError("unsafe config path")
        if digest(directory / name) != expected:
            raise ReplayError("configuration snapshot hash mismatch: " + name)
    stream = manifest["streams"]["events.jsonl"]
    path = directory / "events.jsonl"
    if digest(path) != stream["sha256"]:
        raise ReplayError("episode stream hash mismatch")

    counts = Counter()
    requests = []
    receipts = []
    states = defaultdict(list)
    latest_perception = None
    previous_ms = 0
    previous_mono = 0
    last_seq = 0
    safety_matches = []
    with path.open(encoding="utf-8") as source:
        for line in source:
            item = json.loads(line)
            if item["seq"] != last_seq + 1:
                raise ReplayError("missing or reordered event sequence")
            if item["unix_ms"] < previous_ms or item["monotonic_ns"] < previous_mono:
                raise ReplayError("event timestamps went backward")
            last_seq = item["seq"]
            previous_ms = item["unix_ms"]
            previous_mono = item["monotonic_ns"]
            kind = item["kind"]
            payload = item["payload"]
            counts[kind] += 1
            if kind == "action_request":
                requests.append(payload["method"])
            elif kind == "action_receipt":
                receipts.append(payload["method"])
            elif kind == "perception":
                latest_perception = (item["unix_ms"], payload)
            elif kind == "action_status":
                action_id = payload["action_id"]
                states[action_id].append(payload["state"])
                if payload["state"] == "ACTION_STATE_ABORTED_BY_SAFETY":
                    reason = payload.get("message", "")
                    if "perception stale" in reason:
                        age = None
                        if latest_perception:
                            observed = int(latest_perception[1].get(
                                "observed_at_unix_ms", 0))
                            age = item["unix_ms"] - observed
                        safety_matches.append({
                            "action_id": action_id,
                            "recorded_reason": "perception_stale",
                            "observed_perception_age_ms": age,
                            "evidence": age is not None and age >= 750,
                        })
                    else:
                        safety_matches.append({
                            "action_id": action_id,
                            "recorded_reason": reason,
                            "evidence": "recorded_terminal_only",
                        })

    if last_seq != stream["records"] or dict(counts) != stream["counts"]:
        raise ReplayError("manifest record counts do not match stream")
    if counts["episode_start"] != 1 or counts["episode_end"] != 1:
        raise ReplayError("episode is incomplete")
    if counts["state"] == 0 or counts["perception"] == 0:
        raise ReplayError("state/perception stream missing")
    if requests != receipts:
        raise ReplayError("action requests and receipts are missing or reordered")
    if check_guardrails and requests:
        expected_validations = sum(name != "CancelAction" for name in requests)
        if counts["guardrail_validation"] != expected_validations:
            raise ReplayError("missing guardrail input/output records")
    if not all(history[-1] in TERMINAL for history in states.values()):
        raise ReplayError("action status stream has unterminated action")
    if any(match["evidence"] is False for match in safety_matches):
        raise ReplayError("recorded perception-stale abort lacks sensor-age evidence")
    if manifest.get("stream_errors"):
        raise ReplayError("source stream errors: " + str(manifest["stream_errors"]))

    if check_guardrails and counts["guardrail_validation"]:
        root = Path(__file__).resolve().parents[2]
        binary = root / "build/phase8/icarus-replay-guardrails"
        if not binary.is_file():
            raise ReplayError("native guardrail replay binary is not built")
        native = subprocess.run(
            [str(binary), str(directory / "config/v1.yaml"), str(path)],
            text=True, capture_output=True, check=False)
        if native.returncode:
            raise ReplayError(native.stderr.strip() or "native guardrail replay failed")

    return {
        "status": "passed", "episode_id": manifest["episode_id"],
        "mission": manifest["mission"], "source": manifest["source"],
        "outcome": manifest["outcome"], "records": last_seq,
        "counts": dict(counts), "actions": len(requests),
        "terminal_actions": len(states), "safety_replay": safety_matches,
        "guardrail_replay": counts["guardrail_validation"] if check_guardrails else None,
        "event_sequence_sha256": stream["sha256"],
    }


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("episode", type=Path)
    args = parser.parse_args()
    try:
        print(json.dumps(replay(args.episode), indent=2))
    except (OSError, KeyError, ValueError, json.JSONDecodeError) as error:
        parser.exit(1, f"Replay FAILED: {error}\n")


if __name__ == "__main__":
    main()

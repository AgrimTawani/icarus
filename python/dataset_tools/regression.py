"""Create a reproducible Phase 12 campaign manifest from sealed episodes.

The campaign does not declare a controller or model safe.  It records exactly
which immutable episodes were replayed, their replay digests, scenario/config
hashes, code fingerprints and pinned model descriptors.  A later machine can
run the same command and compare the resulting manifest before interpreting
any threshold result.
"""

import argparse
import hashlib
import json
import time
from pathlib import Path

from python.dataset_tools.replay import ReplayError, verify_complete

SCHEMA = "icarus.phase12.regression-campaign.v1"


def _sha256(value):
    return hashlib.sha256(value.encode()).hexdigest()


def _file_sha256(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def build_campaign(episodes, output_root, check_guardrails=True):
    entries = []
    for path in sorted((Path(item) for item in episodes), key=lambda item: item.name):
        replayed = verify_complete(path, check_guardrails=check_guardrails)
        manifest = json.loads((path / "manifest.json").read_text())
        entries.append({
            "episode_id": manifest["episode_id"],
            "path": str(path.resolve()),
            "source": manifest["source"],
            "scenario": manifest.get("scenario"),
            "seed": manifest.get("seed"),
            "outcome": manifest["outcome"],
            "code_revision": manifest["code_revision"],
            "source_tree_sha256": manifest["source_tree_sha256"],
            "config_sha256": manifest["config_sha256"],
            "model": manifest.get("model"),
            "event_sequence_sha256": replayed["event_sequence_sha256"],
            "replay_result_sha256": replayed["replay_result_sha256"],
        })
    canonical = json.dumps(entries, sort_keys=True, separators=(",", ":"))
    report = {
        "schema": SCHEMA,
        "generated_at": time.strftime("%Y-%m-%dT%H:%M:%S%z"),
        "episodes": entries,
        # Stable identifier intentionally excludes generated_at and paths;
        # this is what a repeated run compares.
        "campaign_fingerprint_sha256": _sha256(canonical),
        "guardrail_replay": check_guardrails,
    }
    root = Path(__file__).resolve().parents[2]
    policy = root / "docs/architecture/PHASE-12-REGRESSION-POLICY.md"
    if policy.is_file():
        report["regression_policy_sha256"] = _file_sha256(policy)
    output = Path(output_root) / time.strftime("%Y%m%dT%H%M%S")
    output.mkdir(parents=True, exist_ok=False)
    (output / "campaign.json").write_text(json.dumps(report, indent=2) + "\n")
    return output, report


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("episodes", nargs="+", type=Path)
    parser.add_argument("--output", type=Path,
                        default=Path("logs/phase12/campaigns"))
    parser.add_argument("--skip-guardrails", action="store_true")
    args = parser.parse_args()
    try:
        output, report = build_campaign(
            args.episodes, args.output, check_guardrails=not args.skip_guardrails)
    except (OSError, ValueError, KeyError, json.JSONDecodeError, ReplayError) as error:
        parser.exit(1, f"Campaign FAILED: {error}\n")
    print(json.dumps({key: value for key, value in report.items()
                      if key != "episodes"}, indent=2))
    print("Campaign manifest:", output / "campaign.json")


if __name__ == "__main__":
    main()

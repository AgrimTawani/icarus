"""Read-only V2 DCM profile artifacts.

``BODY`` is generated from the live contract rather than maintained by hand.
The function returns data for a prompt/episode; it never updates a profile on
disk during a mission.
"""

import hashlib
import json
from pathlib import Path

from python.dcm.contract import ACTIONS, FRESHNESS_LIMITS, VOCABULARY_VERSION

ROOT = Path(__file__).resolve().parents[2]
PROFILE_DIR = ROOT / "config" / "dcm" / "profiles"
BODY_MANIFEST = PROFILE_DIR / "BODY.generated.json"
BODY_SCHEMA = "icarus.dcm.body.v1"


def _file_record(name):
    path = PROFILE_DIR / name
    text = path.read_text(encoding="utf-8")
    return {"path": str(path.relative_to(ROOT)),
            "sha256": hashlib.sha256(text.encode("utf-8")).hexdigest(),
            "text": text}


def _canonical(value):
    return json.dumps(value, sort_keys=True, separators=(",", ":"))


def generated_body():
    """Return the capability facts derived from the active code contract."""
    return {
        "generated": True,
        "vocabulary_version": VOCABULARY_VERSION,
        "allowed_actions": sorted(ACTIONS),
        "freshness_limits_ms": dict(FRESHNESS_LIMITS),
        "authority": "DCM proposals remain subordinate to guardrails, safety supervisor, and ArduPilot failsafes.",
        "direct_mavlink": False,
        "raw_sensor_access": False,
    }


def expected_body_manifest():
    facts = generated_body()
    return {
        "schema": BODY_SCHEMA,
        "source_sha256": hashlib.sha256(_canonical(facts).encode()).hexdigest(),
        "body": facts,
    }


def verify_body_manifest(path=BODY_MANIFEST):
    """Fail closed when the checked-in BODY no longer matches the contract."""
    actual = json.loads(Path(path).read_text(encoding="utf-8"))
    expected = expected_body_manifest()
    if actual != expected:
        raise ValueError(
            "generated BODY manifest is stale; run ./scripts/generate-dcm-body and review the diff")
    return actual


def body():
    """Return verified generated facts plus committed-manifest provenance."""
    manifest = verify_body_manifest()
    facts = dict(manifest["body"])
    facts["manifest_path"] = str(BODY_MANIFEST.relative_to(ROOT))
    facts["manifest_sha256"] = hashlib.sha256(BODY_MANIFEST.read_bytes()).hexdigest()
    facts["source_sha256"] = manifest["source_sha256"]
    return facts


def profiles():
    """Return static human-reviewed profile documents with provenance."""
    return {"soul": _file_record("SOUL.md"),
            "world_facts": _file_record("WORLD_FACTS.md")}

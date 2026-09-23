"""Read-only V2 DCM profile artifacts.

``BODY`` is generated from the live contract rather than maintained by hand.
The function returns data for a prompt/episode; it never updates a profile on
disk during a mission.
"""

import hashlib
from pathlib import Path

from python.dcm.contract import ACTIONS, FRESHNESS_LIMITS, VOCABULARY_VERSION

ROOT = Path(__file__).resolve().parents[2]
PROFILE_DIR = ROOT / "config" / "dcm" / "profiles"


def _file_record(name):
    path = PROFILE_DIR / name
    text = path.read_text(encoding="utf-8")
    return {"path": str(path.relative_to(ROOT)),
            "sha256": hashlib.sha256(text.encode("utf-8")).hexdigest(),
            "text": text}


def body():
    """Return a factual, generated model-visible capability manifest."""
    return {
        "generated": True,
        "vocabulary_version": VOCABULARY_VERSION,
        "allowed_actions": sorted(ACTIONS),
        "freshness_limits_ms": dict(FRESHNESS_LIMITS),
        "authority": "DCM proposals remain subordinate to guardrails, safety supervisor, and ArduPilot failsafes.",
        "direct_mavlink": False,
        "raw_sensor_access": False,
    }


def profiles():
    """Return static human-reviewed profile documents with provenance."""
    return {"soul": _file_record("SOUL.md"),
            "world_facts": _file_record("WORLD_FACTS.md")}

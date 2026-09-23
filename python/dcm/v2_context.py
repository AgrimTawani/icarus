"""Deterministic V2 planner context and reviewed mission-macro support.

This module deliberately has no client, simulator, MAVLink, filesystem write,
or model-runtime dependency.  It transforms the already-curated DCM
observation into a compact, versioned context.  That makes the model's input
more useful without giving it another capability or another path to control.
"""

import hashlib
import json
import re
from pathlib import Path

CONTEXT_VERSION = "dcm-context-v3-feedback"
MACRO_SCHEMA = "icarus.dcm.macro.v1"
SECTORS = ("N", "NE", "E", "SE", "S", "SW", "W", "NW")
_META = re.compile(r"<!-- icarus-macro-meta\n(.*?)\n-->", re.DOTALL)


def _canonical(value):
    return json.dumps(value, sort_keys=True, separators=(",", ":"),
                      ensure_ascii=True)


def digest(value):
    """Return the stable SHA-256 used to identify context and macro inputs."""
    return hashlib.sha256(_canonical(value).encode("utf-8")).hexdigest()


def bearing_sector(bearing_degrees):
    """Return an eight-way compass sector, or ``None`` for invalid input."""
    if isinstance(bearing_degrees, bool):
        return None
    try:
        bearing = float(bearing_degrees) % 360.0
    except (TypeError, ValueError):
        return None
    return SECTORS[int((bearing + 22.5) // 45) % len(SECTORS)]


def obstacle_sectors(perception):
    """Compress normalized obstacle evidence without accepting raw point clouds.

    Current V1 perception publishes only its nearest obstacle and bearing.  A
    future adapter may provide a *normalized* ``obstacle_sectors`` mapping,
    but arbitrary lidar arrays are explicitly ignored here.
    """
    source = perception or {}
    supplied = source.get("obstacle_sectors")
    if isinstance(supplied, dict):
        compact = {}
        for sector in SECTORS:
            entry = supplied.get(sector)
            if not isinstance(entry, dict):
                continue
            distance = entry.get("nearest_distance_m")
            if isinstance(distance, bool):
                continue
            try:
                distance = float(distance)
            except (TypeError, ValueError):
                continue
            if distance >= 0:
                compact[sector] = {"nearest_distance_m": round(distance, 3),
                                   "source": "normalized_obstacle_map"}
        return compact
    sector = bearing_sector(source.get("nearest_obstacle_bearing_deg"))
    distance = source.get("nearest_obstacle_distance_m")
    if sector is None or isinstance(distance, bool):
        return {}
    try:
        distance = float(distance)
    except (TypeError, ValueError):
        return {}
    if distance < 0:
        return {}
    return {sector: {"nearest_distance_m": round(distance, 3),
                     "source": "normalized_obstacle_map"}}


def failure_signals(history, previous_result=None):
    """Compute failure and repetition facts in code, never ask the model to infer them."""
    rows = [item for item in (history or []) if isinstance(item, dict)]
    outcomes = [str(item.get("outcome", "")) for item in rows]
    failures = sum(outcome not in ("", "SUCCEEDED") for outcome in outcomes)
    consecutive = 0
    for outcome in reversed(outcomes):
        if outcome == "SUCCEEDED":
            break
        consecutive += 1
    actions = [item.get("action") for item in rows[-3:]]
    repeated = actions[-1] if len(actions) >= 2 and len(set(actions)) == 1 else None
    result = previous_result or {}
    return {
        "completed_actions": len(rows),
        "failed_actions": failures,
        "consecutive_failures": consecutive,
        "repeated_action": repeated,
        "previous_result": result if isinstance(result, dict) else None,
    }


def read_macro(path):
    """Read one reviewed macro document without executing or mutating it."""
    source = Path(path)
    text = source.read_text(encoding="utf-8")
    match = _META.search(text)
    if not match:
        raise ValueError(f"macro {source} has no icarus metadata")
    try:
        metadata = json.loads(match.group(1))
    except json.JSONDecodeError as error:
        raise ValueError(f"macro {source} metadata is not JSON") from error
    required = {"schema", "id", "revision", "allowed_actions", "tags"}
    if not isinstance(metadata, dict) or set(metadata) < required:
        raise ValueError(f"macro {source} has incomplete metadata")
    if metadata["schema"] != MACRO_SCHEMA:
        raise ValueError(f"macro {source} has unsupported schema")
    if not all(isinstance(item, str) for item in metadata["allowed_actions"]):
        raise ValueError(f"macro {source} has invalid allowed_actions")
    if not all(isinstance(item, str) for item in metadata["tags"]):
        raise ValueError(f"macro {source} has invalid tags")
    return {"id": metadata["id"], "revision": metadata["revision"],
            "tags": list(metadata["tags"]),
            "allowed_actions": list(metadata["allowed_actions"]),
            "guidance": text[match.end():].strip(),
            "sha256": hashlib.sha256(text.encode("utf-8")).hexdigest(),
            "path": str(source)}


def load_macros(directory, actions):
    """Load a stable catalog and reject a macro that describes an illegal action."""
    catalog = []
    for path in sorted(Path(directory).glob("*.md")):
        macro = read_macro(path)
        unknown = sorted(set(macro["allowed_actions"]) - set(actions))
        if unknown:
            raise ValueError(f"macro {macro['id']} names unknown actions: {unknown}")
        catalog.append(macro)
    return catalog


def select_macro(catalog, mission_metadata):
    """Select only an explicit macro id; free-form keyword matching is forbidden."""
    requested = (mission_metadata or {}).get("macro_id")
    if requested is None:
        return None
    matches = [macro for macro in catalog if macro["id"] == requested]
    if len(matches) != 1:
        raise ValueError(f"requested macro is unavailable: {requested}")
    macro = matches[0]
    return {key: macro[key] for key in ("id", "revision", "allowed_actions",
                                         "guidance", "sha256")}


def build_context(observation, catalog=(), mission_metadata=None, body=None,
                  soul=None, world_facts=None):
    """Build an auditable V3 context from a pre-curated observation.

    The original curated state is retained for compatibility, while the V2
    fields are additive and compact. ``context_sha256`` is calculated before
    it is inserted so it identifies exactly the model-visible payload.
    """
    if not isinstance(observation, dict):
        raise TypeError("observation must be a mapping")
    state = observation.get("state") or {}
    perception = observation.get("perception") or {}
    history = observation.get("actions_completed") or []
    selected = select_macro(catalog, mission_metadata)
    context = dict(observation)
    context.update({
        "context_version": CONTEXT_VERSION,
        "state_summary": {
            "flight_phase": state.get("flight_phase"),
            "armed": state.get("armed"),
            "landed": state.get("landed"),
            "relative_altitude_m": state.get("relative_altitude_m"),
            "ground_speed_mps": state.get("ground_speed_mps"),
            "battery": state.get("battery"),
            "estimator": state.get("estimator"),
        },
        "perception_summary": {
            "overall": perception.get("overall"),
            "path_ahead_clear": perception.get("path_ahead_clear"),
            "local_map_age_ms": perception.get("local_map_age_ms"),
            "obstacle_sectors": obstacle_sectors(perception),
        },
        "failure_signals": failure_signals(history,
                                            observation.get("previous_result")),
        "mission_macro": selected,
        "body": body or {},
        "soul": soul or {},
        "world_facts": world_facts or {},
        "macro_catalog": [{key: macro[key] for key in ("id", "revision", "tags")}
                          for macro in catalog],
    })
    context["context_sha256"] = digest(context)
    return context

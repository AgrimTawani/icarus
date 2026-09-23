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

import yaml

CONTEXT_VERSION = "dcm-context-v3-compact"
MACRO_SCHEMA = "icarus.dcm.macro.v1"
SECTORS = ("N", "NE", "E", "SE", "S", "SW", "W", "NW")
_META = re.compile(r"<!-- icarus-macro-meta\n(.*?)\n-->", re.DOTALL)
_FRONT_MATTER = re.compile(r"\A---\s*\n(.*?)\n---\s*\n", re.DOTALL)
MAX_CONTEXT_BYTES = 8_000
MAX_MACRO_GUIDANCE_CHARS = 4_000
MAX_PROFILE_TEXT_CHARS = 3_000


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
    match = _FRONT_MATTER.search(text)
    legacy = False
    if not match:
        match = _META.search(text)
        legacy = True
    if not match:
        raise ValueError(f"macro {source} has no YAML front matter")
    try:
        metadata = (json.loads(match.group(1)) if legacy
                    else yaml.safe_load(match.group(1)))
    except (json.JSONDecodeError, yaml.YAMLError) as error:
        raise ValueError(f"macro {source} metadata is invalid") from error
    required = {"schema", "id", "revision", "allowed_actions", "tags",
                "preconditions", "evidence_links"}
    if legacy:
        required -= {"preconditions", "evidence_links"}
    if not isinstance(metadata, dict) or set(metadata) < required:
        raise ValueError(f"macro {source} has incomplete metadata")
    if metadata["schema"] != MACRO_SCHEMA:
        raise ValueError(f"macro {source} has unsupported schema")
    for field in ("allowed_actions", "tags", "allowed_action_families",
                  "preconditions", "evidence_links"):
        value = metadata.get(field, [])
        if not isinstance(value, list) or not value or not all(
                isinstance(item, str) and item.strip() for item in value):
            if legacy and field in ("allowed_action_families", "preconditions",
                                     "evidence_links"):
                continue
            raise ValueError(f"macro {source} has invalid {field}")
    guidance = text[match.end():].strip()
    if len(guidance) > MAX_MACRO_GUIDANCE_CHARS:
        raise ValueError(f"macro {source} guidance exceeds character budget")
    policy = metadata.get("model_policy") or {}
    if policy:
        if not isinstance(policy, dict):
            raise ValueError(f"macro {source} has invalid model_policy")
        for field in ("phases", "recovery_actions", "evidence_requirements"):
            values = policy.get(field)
            if not isinstance(values, list) or not values or not all(
                    isinstance(item, str) and item.strip() for item in values):
                raise ValueError(
                    f"macro {source} has invalid model_policy.{field}")
    return {"id": metadata["id"], "revision": metadata["revision"],
            "tags": list(metadata["tags"]),
            "allowed_actions": list(metadata["allowed_actions"]),
            "allowed_action_families": list(metadata.get("allowed_action_families", [])),
            "preconditions": list(metadata.get("preconditions", [])),
            "evidence_links": list(metadata.get("evidence_links", [])),
            "model_policy": policy,
            "guidance": guidance,
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
        unknown_recovery = sorted(
            set(macro["model_policy"].get("recovery_actions", ())) - set(actions))
        if unknown_recovery:
            raise ValueError(
                f"macro {macro['id']} names unknown recovery actions: {unknown_recovery}")
        if any(item["id"] == macro["id"] for item in catalog):
            raise ValueError(f"duplicate macro id: {macro['id']}")
        catalog.append(macro)
    return catalog


def select_macro(catalog, mission_metadata=None, mission=""):
    """Resolve an explicit macro or deterministically match reviewed intent tags.

    This is deliberately code-owned retrieval, not LLM keyword injection.  An
    explicit id always wins.  Automatic selection requires a distinctive
    mission phrase, is deterministic, and returns at most one reviewed macro.
    """
    requested = (mission_metadata or {}).get("macro_id")
    if requested is not None:
        matches = [macro for macro in catalog if macro["id"] == requested]
        if len(matches) != 1:
            raise ValueError(f"requested macro is unavailable: {requested}")
        macro = matches[0]
        source = "explicit"
    else:
        text = " ".join(str(mission).lower().replace("-", " ").split())
        rules = (
            ("building-inspection", ("building", "inspect"), ("building", "check"),
             ("building", "hover")),
            ("inspection-orbit", ("orbit", "inspect"), ("orbit", "survey")),
            ("landing-site-survey", ("landing", "survey"), ("landing", "site")),
            ("perimeter-sweep", ("perimeter",), ("boundary", "survey"),
             ("perimeter", "sweep")),
            ("degraded-position-return", ("degraded", "return"),
             ("estimator", "recovery"), ("lost", "position")),
        )
        selected_id = next((identifier for identifier, *patterns in rules
                            if any(all(word in text for word in pattern)
                                   for pattern in patterns)), None)
        matches = [macro for macro in catalog if macro["id"] == selected_id]
        if not selected_id or len(matches) != 1:
            return None
        macro = matches[0]
        source = "deterministic-intent"
    return {key: macro[key] for key in ("id", "revision", "allowed_actions",
                                         "model_policy", "guidance", "sha256")} | {
                                             "selection_source": source}


def compile_mission_contract(mission, macro=None, reference=None, obligations=None):
    """Combine mission semantics into one compact model-visible contract.

    Macro prose, profile provenance and simulator truth remain audit material.
    The model receives only reviewed action constraints, structured phases,
    completion evidence and an optional reviewed target.  This function never
    chooses the next action.
    """
    macro = macro or {}
    reference = reference or {}
    obligations = obligations or {}
    policy = macro.get("model_policy") or {}
    contract = {
        "schema": "icarus.dcm.mission-contract.v1",
        "mission_type": macro.get("id", "unclassified"),
        "objectives": [],
        "pending": [],
        "completed": [],
    }
    if macro:
        contract["permitted_actions"] = list(macro.get("allowed_actions", []))
    for key in ("phases", "recovery_actions", "evidence_requirements"):
        if policy.get(key):
            contract[key] = list(policy[key])

    primary_id = reference.get("primary_target_id")
    landmark = next((item for item in reference.get("known_landmarks", [])
                     if item.get("id") == primary_id), None)
    if primary_id:
        target = {"id": primary_id}
        if landmark and landmark.get("hover_waypoint"):
            target["hover_waypoint"] = dict(landmark["hover_waypoint"])
        contract["target"] = target

    completion = {
        "flight": "successful_takeoff",
        "building_visual_inspection": "successful_inspect_scene",
        "building_hover": "hold_at_reviewed_waypoint",
        "person_count": "successful_detect_person",
        "return_home": "landed_at_home",
    }
    for requirement in obligations.get("requirements", []):
        if not isinstance(requirement, dict) or not requirement.get("id"):
            continue
        objective = {
            "id": requirement["id"],
            "completion": completion.get(requirement["id"], "reviewed_evidence"),
        }
        for key in ("target_altitude_agl_m", "reviewed_waypoint",
                    "duration_ms", "detector_classes", "target_id"):
            if requirement.get(key) is not None:
                if key == "target_id" and requirement[key] == primary_id:
                    continue
                if key == "reviewed_waypoint" and contract.get(
                        "target", {}).get("hover_waypoint") == requirement[key]:
                    continue
                objective[key] = requirement[key]
        contract["objectives"].append(objective)
        status_key = ("completed" if requirement.get("status") == "satisfied"
                      else "pending")
        contract[status_key].append(requirement["id"])
    if not contract["objectives"] and not macro:
        contract["operator_text_is_authoritative"] = True
    return contract


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
    selected = select_macro(catalog, mission_metadata,
                            mission=observation.get("mission", ""))
    # Keep the model-visible physical facts once. V1's complete curated state
    # and perception objects overlap heavily with the V3 summaries; retaining
    # both doubled prefill cost without adding evidence.
    context = {key: value for key, value in observation.items()
               if key not in ("state", "perception", "allowed_actions")}
    context["state"] = {key: state.get(key) for key in (
        "sequence", "observed_at_unix_ms", "local_position_ned", "mavlink",
        "manual_override_active", "geofence_breached") if key in state}
    context["perception"] = {key: perception.get(key) for key in (
        "sequence", "observed_at_unix_ms", "local_map_available",
        "local_map_age_ms") if key in perception}
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
        "source_evidence": {
            "state_observed_at_unix_ms": state.get("observed_at_unix_ms"),
            "perception_observed_at_unix_ms": perception.get(
                "observed_at_unix_ms"),
            "perception_age_ms": perception.get("local_map_age_ms"),
            "perception_health": perception.get("overall"),
            "confidence_source": "normalized_deterministic_contract",
        },
        "failure_signals": failure_signals(history,
                                            observation.get("previous_result")),
        "mission_macro": selected,
        "mission_contract": compile_mission_contract(
            observation.get("mission", ""), selected),
        "body": body or {},
        "soul": soul or {},
        "world_facts": world_facts or {},
        "macro_catalog": [{key: macro[key] for key in ("id", "revision", "tags")}
                          for macro in catalog],
    })
    # Profile text remains useful, but no reviewed document can grow without
    # bound and silently crowd the current physical state out of the prompt.
    for profile_name in ("soul", "world_facts"):
        record = context.get(profile_name)
        if isinstance(record, dict) and isinstance(record.get("text"), str):
            record = dict(record)
            record["text"] = record["text"][:MAX_PROFILE_TEXT_CHARS]
            context[profile_name] = record
    return finalize_context(context)


def _model_body(body):
    """Strip manifest and duplicated policy facts from model-visible BODY."""
    deployment = dict((body or {}).get("deployment") or {})
    deployment.pop("capabilities", None)  # the contract vocabulary is authoritative
    cameras = deployment.get("cameras")
    if isinstance(cameras, dict):
        compact = {}
        for camera_id, config in cameras.items():
            captures = []
            for item in (config or {}).get("CaptureSettings", []):
                if not isinstance(item, dict):
                    continue
                captures.append({key: item[key] for key in
                                 ("ImageType", "Width", "Height", "FOV_Degrees")
                                 if key in item})
            compact[camera_id] = {"captures": captures} if captures else {
                "configured": True}
        deployment["cameras"] = compact
    return deployment


def _model_state(context):
    state = dict(context.get("state") or {})
    state.pop("observed_at_unix_ms", None)
    state.pop("sequence", None)
    perception = dict(context.get("perception") or {})
    perception.pop("observed_at_unix_ms", None)
    perception.pop("local_map_age_ms", None)
    perception.pop("sequence", None)
    user = {key: value for key, value in context.items()
            if key not in ("soul", "body", "world_facts", "mission_macro",
                           "macro_catalog", "context_sha256", "context_budget",
                           "source_evidence", "mission_reference",
                           "mission_obligations", "previous_result",
                           "contract_version", "vocabulary_version",
                           "context_version", "source_event_seq",
                           "observed_at_unix_ms")}
    user["state"] = state
    user["perception"] = perception
    user["evidence_health"] = {
        "state": ("fresh" if (context.get("source_evidence") or {}).get(
            "state_observed_at_unix_ms") is not None else "unavailable"),
        "perception": ("fresh" if (context.get("source_evidence") or {}).get(
            "perception_observed_at_unix_ms") is not None and
            (context.get("perception_summary") or {}).get("overall") not in
            (None, "HEALTH_LEVEL_UNAVAILABLE") else "unavailable"),
        "perception_age_ms": (context.get("perception_summary") or {}).get(
            "local_map_age_ms"),
    }
    # Age and health now have one canonical location in the visible payload.
    if isinstance(user.get("perception_summary"), dict):
        user["perception_summary"] = dict(user["perception_summary"])
        user["perception_summary"].pop("local_map_age_ms", None)
    return user


def model_visible_payload(context):
    """Return the semantic payload rendered into V3 system and user sections.

    Hash/budget fields are deliberately omitted to avoid a self-referential
    digest. The macro catalog is retrieval metadata; only the selected reviewed
    macro reaches the model.
    """
    return {
        "profiles": {
            "principles": [
                "Safety outranks mission progress.",
                "Never treat missing or stale evidence as safe.",
                "Propose one bounded action; deterministic systems retain authority.",
            ],
            "body": _model_body(context.get("body")),
        },
        "user": _model_state(context),
    }


def finalize_context(context):
    """Hash and budget the final payload after every model-visible field exists."""
    value = dict(context)
    value.pop("context_sha256", None)
    value.pop("context_budget", None)
    visible = model_visible_payload(value)
    encoded = _canonical(visible).encode("utf-8")
    if len(encoded) > MAX_CONTEXT_BYTES:
        raise ValueError(
            f"V3 context is {len(encoded)} bytes, limit {MAX_CONTEXT_BYTES}")
    value["context_budget"] = {
        "max_bytes": MAX_CONTEXT_BYTES,
        "payload_bytes": len(encoded),
        "estimated_tokens_upper_bound": (len(encoded) + 2) // 3,
    }
    value["context_sha256"] = digest(visible)
    return value

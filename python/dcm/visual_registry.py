"""Deterministic routing of semantic visual queries to a perception tool.

The planner LLM only says *what* to look for ("people", "suspicious damage",
"building").  This registry decides *how*: a query that is one of the pinned
YOLO/COCO detector classes (through a deliberately conservative alias table)
goes to YOLO; everything else becomes a current-camera VLM question.  The LLM
can never choose the tool.
"""

from __future__ import annotations

import re

from python.perception.classes import YOLO_CLASSES

MAX_QUERY_CHARS = 80

# Conservative: only unambiguous plural/synonym forms.  An attribute such as
# "red car" or "injured person" is *not* a detector class and goes to the VLM.
_ALIASES = {
    "people": ("person",), "persons": ("person",), "human": ("person",),
    "humans": ("person",), "pedestrian": ("person",),
    "pedestrians": ("person",), "someone": ("person",), "anyone": ("person",),
    "cars": ("car",), "trucks": ("truck",), "buses": ("bus",),
    "bike": ("bicycle",), "bikes": ("bicycle",), "bicycles": ("bicycle",),
    "motorbike": ("motorcycle",), "motorbikes": ("motorcycle",),
    "motorcycles": ("motorcycle",), "boats": ("boat",), "dogs": ("dog",),
    "cats": ("cat",), "birds": ("bird",), "horses": ("horse",),
    "cows": ("cow",), "trains": ("train",), "aeroplane": ("airplane",),
    "aeroplanes": ("airplane",), "airplanes": ("airplane",),
    "vehicle": ("car", "truck", "bus", "motorcycle"),
    "vehicles": ("car", "truck", "bus", "motorcycle"),
}
_FILLER = re.compile(r"^(?:any|all|the|a|an|some|for|of)\s+")


def normalize_query(query):
    """Return the compact lowercase query text, or raise ValueError."""
    if not isinstance(query, str):
        raise ValueError("visual query must be text")  # noqa: TRY004
    compact = " ".join(re.sub(r"[^\w\s'-]", " ", query.lower()).split())
    if not compact:
        raise ValueError("visual query must not be empty")
    if len(compact) > MAX_QUERY_CHARS:
        raise ValueError(f"visual query exceeds {MAX_QUERY_CHARS} characters")
    return compact


def _core(compact):
    current = compact
    while True:
        stripped = _FILLER.sub("", current)
        if stripped == current:
            return current
        current = stripped


def yolo_classes_for(query):
    """Return the detector classes for ``query`` or None when YOLO cannot answer."""
    core = _core(normalize_query(query))
    if core in YOLO_CLASSES:
        return (core,)
    return _ALIASES.get(core)


def vlm_question(queries, yes_no=False):
    """Build one current-camera question for one or more qualitative queries."""
    parts = []
    for query in queries:
        core = _core(normalize_query(query))
        parts.append("any " + core)
    subject = " or ".join(parts)
    question = f"Is there {subject} visible in the current camera view?"
    if yes_no:
        question += " Answer yes or no first, then describe briefly."
    return question


def route_queries(queries):
    """Split semantic queries into YOLO classes and VLM queries.

    Returns ``{"yolo_classes": [...], "vlm_queries": [...], "routes": [...]}``
    where ``routes`` records the decision for every query for the audit trail.
    """
    if not isinstance(queries, list) or not queries:
        raise ValueError("look needs at least one visual query")
    classes, vlm, routes = [], [], []
    for query in queries:
        compact = normalize_query(query)
        mapped = yolo_classes_for(compact)
        if mapped:
            for name in mapped:
                if name not in classes:
                    classes.append(name)
            routes.append({"query": compact, "tool": "yolo",
                           "classes": list(mapped)})
        else:
            if compact not in vlm:
                vlm.append(compact)
            routes.append({"query": compact, "tool": "vlm"})
    return {"yolo_classes": classes, "vlm_queries": vlm, "routes": routes}


def answer_is_positive(result):
    """Return whether a yes/no VLM answer starts with an affirmative."""
    answer = str((result or {}).get("answer") or "").strip().lower()
    return bool(re.match(r"^(?:yes|yeah|yep)\b", answer))

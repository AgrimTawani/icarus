"""Shared evidence validity, independent of a mission's condition truth value."""

import math


def valid_capture_timestamp(value):
    """Capture provenance must be a finite positive timestamp, never a boolean."""
    return type(value) in (int, float) and math.isfinite(value) and value > 0


def usable_observation(result, observation=None, *, captured_after_ms=None,
                       captured_before_ms=None):
    """Whether a typed measurement was obtained within an optional capture window.

    A valid negative answer (False or zero) is usable. Unknown, model errors,
    missing fields, and a frame outside the action window are not. This does
    not establish factual accuracy, nor authorize any flight action.
    """
    if not isinstance(result, dict):
        return False
    if (result.get("error") or result.get("error_code") or
            result.get("validity") not in (None, "valid")):
        return False
    captured = result.get("captured_at_unix_ms")
    if not valid_capture_timestamp(captured):
        return False
    if captured_after_ms is not None and (
            not valid_capture_timestamp(captured_after_ms) or captured < captured_after_ms):
        return False
    if captured_before_ms is not None and (
            not valid_capture_timestamp(captured_before_ms) or captured > captured_before_ms):
        return False
    observation = observation or {}
    if observation.get("tool", result.get("tool")) == "landing":
        return result.get("assessable") is True and type(result.get("suitable")) is bool
    requested_type = observation.get("result_type", result.get("result_type", "boolean"))
    if result.get("result_type", requested_type) != requested_type:
        return False
    if requested_type == "count":
        if observation.get("count_scope", "visible_frame") != "visible_frame":
            return False
        if result.get("count_scope", "visible_frame") != "visible_frame":
            return False
        return type(result.get("count")) is int and result["count"] >= 0
    if requested_type == "boolean":
        return type(result.get("found")) is bool
    return False

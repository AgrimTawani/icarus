"""Bounded console summaries of recorded sensor evidence; no mission decisions."""
import math
from collections.abc import Mapping

_SCAN_TOOLS = {'scan_heading', 'resume_heading_scan', 'scan_area', 'resume_scan'}
_OBSERVATION_TOOLS = {'detect', 'inspect_scene', 'localize_target'}


def _text(value, limit=300):
    if not isinstance(value, str):
        return ''
    value = ' '.join(value.split())
    return value if len(value) <= limit else value[:limit - 1] + '…'


def _number(value):
    return isinstance(value, (int, float)) and not isinstance(value, bool) and math.isfinite(value)


def _evidence(tool, evidence, camera=None):
    camera = _text(evidence.get('camera') or camera, 60) or 'camera unspecified'
    prefix = f'{camera}: '
    if (evidence.get('validity') in ('unknown', 'error', 'invalid')
            or evidence.get('fresh') is False or evidence.get('available') is False):
        return prefix + 'observation unavailable or stale' + (
            ': ' + _text(evidence['reason']) if _text(evidence.get('reason')) else '')
    if tool == 'detect':
        counts = evidence.get('counts')
        if not isinstance(counts, Mapping) or not counts:
            return prefix + 'accepted detection counts unavailable'
        values = []
        for label, count in list(counts.items())[:16]:
            valid = _number(count) and count >= 0 and int(count) == count
            values.append(f'{_text(label, 50)}={int(count) if valid else "unknown"}')
        return prefix + 'accepted detections ' + ', '.join(values) + ' (not a guaranteed total)'
    if tool == 'inspect_scene':
        answer = _text(evidence.get('answer'))
        return prefix + ('VLM answer (uncertain): ' + answer if answer else 'VLM answer unavailable')
    target = _text(evidence.get('target_id'), 80)
    prefix = 'Target' + (f' {target}' if target else '') + ': '
    if evidence.get('found') is not True:
        state = 'not localized' if evidence.get('found') is False else 'localization unavailable'
        return prefix + state + (': ' + _text(evidence['reason']) if _text(evidence.get('reason')) else '')
    distance = evidence.get('surface_range_m')
    measured = f'measured surface range {distance:.2f} m' if _number(distance) and distance >= 0 else 'surface range unavailable'
    return prefix + 'localized; ' + measured


def observation_feedback(tool, result, arguments=None, *, previous_observation_count=0, max_observations=3):
    """Return factual display lines; callers retain per-scan observation cursors.

    A scan's evidence.observations is cumulative. Pass its previously displayed
    length (keyed by scan_id), then advance to the current length after printing.
    Only the last max_observations NEW stations are expanded, with omissions stated.
    Unknown/failed evidence never becomes a zero count or negative scene answer.
    """
    if tool not in _OBSERVATION_TOOLS | _SCAN_TOOLS or not isinstance(result, Mapping):
        return []
    arguments = arguments if isinstance(arguments, Mapping) else {}
    evidence = result.get('evidence')
    evidence = evidence if isinstance(evidence, Mapping) else {}
    lines = []
    status = result.get('status')
    if status != 'SUCCEEDED':
        lines.append(f'{tool}: {_text(status) or "status unavailable"}' + (
            ': ' + _text(result['message']) if _text(result.get('message')) else ''))
        if tool in _OBSERVATION_TOOLS:
            return lines
    if tool in _OBSERVATION_TOOLS:
        return [_evidence(tool, evidence, arguments.get('camera'))]
    observations = evidence.get('observations')
    if not isinstance(observations, list):
        return lines + [f'{tool}: observation evidence unavailable']
    start = max(0, int(previous_observation_count))
    fresh = observations[start:]
    limit = max(1, min(10, int(max_observations)))
    if len(fresh) > limit:
        lines.append(f'{tool}: {len(fresh)} new observations; showing last {limit}')
    for observation in fresh[-limit:]:
        if not isinstance(observation, Mapping):
            lines.append('Scan observation: evidence unavailable')
            continue
        value = observation.get('evidence')
        value = value if isinstance(value, Mapping) else {}
        kind = 'inspect_scene' if 'answer' in value else 'detect'
        station = observation.get('station')
        prefix = f'Station {station}' if _number(station) else 'Scan observation'
        heading = observation.get('heading_deg')
        if _number(heading):
            prefix += f' at {heading:g}°'
        lines.append(prefix + ' — ' + _evidence(kind, value, arguments.get('camera')))
    event = _text(evidence.get('event'), 80)
    if event:
        lines.append(f'{tool}: {event}')
    return lines

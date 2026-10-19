"""Factual detector-filter receipts; never interpretations of mission conditions."""
import math


def stop_filter_receipt(arguments, evidence, station):
    label = arguments.get('stop_on_detection')
    if (not label or evidence.get('validity') in ('unknown', 'error', 'invalid', 'stale')
            or evidence.get('fresh') is False or evidence.get('available') is False
            or evidence.get('stale') is True):
        return None
    count = evidence.get('counts', {}).get(label)
    if (isinstance(count, bool) or not isinstance(count, (int, float))
            or not math.isfinite(count) or count < 0 or count != int(count)):
        return None
    minimum = arguments.get('minimum_count', 1)
    return {'class': label, 'comparison': '>=', 'minimum_count': minimum,
            'observed_count': count, 'matched': count >= minimum,
            'station': station, 'scope': 'workflow_stop_filter_not_mission_verdict'}

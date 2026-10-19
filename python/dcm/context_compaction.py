"""Presentation-only compaction of redundant workflow receipts."""
import copy


def reference_registered_target_geometry(value, target_ids):
    """Keep historical observations, reference current geometry once per target.

    Object-motion receipts can nest their initial localization. Repeating its
    old view candidates and hull envelope competes with the current registry
    and consumes the small model's context. Keep time, identity, range, bearing,
    height and uncertainty; full historical geometry remains in the episode.
    """
    if isinstance(value, list):
        return [reference_registered_target_geometry(item, target_ids) for item in value]
    if not isinstance(value, dict):
        return value
    if (value.get('schema') == 'icarus.perception.target_estimate.v1'
            and value.get('target_id') in target_ids):
        result = {key: copy.deepcopy(value[key]) for key in (
            'target_id', 'description', 'found', 'reason', 'origin_id',
            'observed_at_unix_ms', 'image_captured_at_unix_ms',
            'surface_range_m', 'surface_range_kind', 'bearing_deg') if key in value}
        for field, keys in (
                ('quality', ('age_ms', 'viewpoints', 'viewpoints_agree', 'position_spread_m')),
                ('tracking', ('status', 'reason', 'geometry_age_ms')),
                ('depth_at_capture', ('fresh', 'age_ms', 'available', 'valid_fraction',
                                      'median_optical_z_m', 'minimum_radial_range_m', 'object_associated'))):
            if isinstance(value.get(field), dict):
                result[field] = {key: copy.deepcopy(value[field][key])
                                 for key in keys if key in value[field]}
        geometry = value.get('geometry', {})
        result['historical_height'] = {key: copy.deepcopy(geometry[key]) for key in (
            'height_m', 'height_status', 'height_evidence_age_ms') if key in geometry}
        result['current_geometry_reference'] = 'targets'
        return result
    return {key: reference_registered_target_geometry(item, target_ids)
            for key, item in value.items()}


def compact_scan_receipts(entries):
    """Group consecutive successful resumes; cumulative station evidence stays separate.

    Original receipt indices are retained as an inclusive range. This function
    never mutates execution history or decides whether a mission is complete.
    """
    result = []

    def eligible(entry):
        return (entry.get('action') in ('resume_heading_scan', 'resume_scan')
                and entry.get('status') == 'SUCCEEDED'
                and isinstance(entry.get('arguments'), dict)
                and isinstance(entry['arguments'].get('scan_id'), str)
                and bool(entry['arguments']['scan_id']))

    def identity(entry):
        return {k: v for k, v in entry.items() if k not in (
            'index', 'message', 'through_index', 'executions', 'evidence_reference')}

    for entry in entries:
        previous = result[-1] if result else None
        if (previous is not None and eligible(previous) and eligible(entry)
                and identity(previous) == identity(entry)
                and entry.get('index') == previous.get('through_index', previous.get('index', -2)) + 1):
            previous['through_index'] = entry['index']
            previous['executions'] = previous.get('executions', 1) + 1
            previous['message'] = entry.get('message', '')
            previous['evidence_reference'] = {
                'scan_id': entry['arguments']['scan_id'],
                'location': 'observations[].evidence: cumulative stations',
            }
        else:
            result.append(copy.deepcopy(entry))
    return result


def compact_scan_evidence(scan):
    """Factor identical station evidence while keeping the latest view explicit."""
    import json
    stations = scan.get('observations', [])
    if len(stations) < 2 or 'shared_station_evidence' in scan:
        return scan
    evidence = [station.get('evidence') for station in stations]
    for station, item in zip(stations, evidence):
        if (not isinstance(item, dict)
                or station.get('status', 'SUCCEEDED') != 'SUCCEEDED'
                or item.get('status', 'SUCCEEDED') != 'SUCCEEDED'
                or item.get('error') or item.get('available') is False
                or item.get('fresh') is False or item.get('stale') is True
                or item.get('validity') in ('unknown', 'invalid', 'error', 'stale')):
            return scan
    common = {key: copy.deepcopy(evidence[0][key])
              for key in ('camera', 'confidence_threshold', 'count_scope')
              if key in evidence[0] and all(key in item and item[key] == evidence[0][key]
                                           for item in evidence)}
    first_counts = evidence[0].get('counts', {})
    zeros = {key: 0 for key, value in first_counts.items()
             if type(value) is int and value == 0 and all(
                 isinstance(item.get('counts'), dict)
                 and type(item['counts'].get(key)) is int and item['counts'][key] == 0
                 for item in evidence)} if isinstance(first_counts, dict) else {}
    if not common and not zeros:
        return scan
    packed = copy.deepcopy(scan)
    packed['shared_station_evidence'] = {**common, **({'counts': zeros} if zeros else {})}
    packed['station_evidence_merge'] = (
        'For each station, merge shared fields with station evidence; merge counts by class. '
        'Station values override shared values. Latest evidence is fully explicit. '
        'Zero counts are detections, not proof of absence.')
    for station in packed['observations'][:-1]:
        item = station['evidence']
        for key in common:
            del item[key]
        for key in zeros:
            del item['counts'][key]
    try:
        size = lambda item: len(json.dumps(item, separators=(',', ':'),
                                         allow_nan=False).encode('utf-8'))
        return packed if size(packed) < size(scan) else scan
    except (TypeError, ValueError):
        return scan

"""Read-only access to retained action receipts; no mission interpretation."""
import copy
import json

MAX_REPLY_BYTES = 2000


def receipt_outcome_facts(result, *, latest=True):
    """Keep small factual outcomes inline even when large evidence is paged.

    Transport success is not object presence or object-relative positioning.
    Scalars and small maps preserve those distinctions without replaying raw
    geometry, telemetry, model metadata or cumulative observation arrays.
    """
    evidence = result.get('evidence', {})
    if not isinstance(evidence, dict):
        return {}
    scalar = lambda value: value is None or type(value) in (bool, int, float, str)
    facts = {}
    for key, value in evidence.items():
        if key in ('schema', 'runtime', 'image', 'provenance', 'latency_ms', 'capture_pose_quality'):
            continue
        if scalar(value) and (not isinstance(value, str) or len(value) <= 300):
            facts[key] = copy.deepcopy(value)
        elif isinstance(value, dict) and all(scalar(v) for v in value.values()) and _size(value) <= 512:
            facts[key] = copy.deepcopy(value)
        elif key == 'counts' and isinstance(value, dict):
            positives = {name: count for name, count in value.items()
                         if type(count) in (int, float) and count > 0}
            facts['nonzero_counts'] = dict(list(positives.items())[:8])
            facts['counts_omitted'] = True
            facts['counts_reference'] = ['result', 'evidence', 'counts']
    observations = evidence.get('observations')
    if latest and isinstance(observations, list) and observations:
        last = observations[-1]
        facts['observed_station_count'] = len(observations)
        facts['latest_observation'] = {
            key: copy.deepcopy(value) for key, value in last.items()
            if scalar(value) and (not isinstance(value, str) or len(value) <= 120)}
        facts['latest_observation']['evidence'] = receipt_outcome_facts(
            {'evidence': last.get('evidence', {})}, latest=False)
    return facts


def _size(value):
    return len(json.dumps(value, ensure_ascii=False, separators=(',', ':')).encode('utf-8'))


def indexed_results(actions):
    index = 0
    for entry in actions:
        items = entry['result'].get('actions', []) if entry['action'] == 'execute_sequence' else [
            {'name': entry['action'], 'arguments': entry.get('arguments', {}), 'result': entry['result']}]
        for item in items:
            index += 1
            yield index, {'action': item['name'], 'arguments': item.get('arguments', {}),
                          'result': item['result']}


def read_action_result(actions, action_index, path=(), offset=0, limit=8):
    if type(action_index) is not int or action_index < 1:
        raise ValueError('action_index must be a positive receipt index')
    if type(offset) is not int or offset < 0 or type(limit) is not int or not 1 <= limit <= 16:
        raise ValueError('offset/limit are invalid')
    value = next((item for index, item in indexed_results(actions) if index == action_index), None)
    if value is None:
        raise ValueError('No retained action result has that index')
    receipt = value
    prefix = []
    for key in path:
        if isinstance(value, dict) and isinstance(key, str) and key in value:
            value = value[key]
        elif isinstance(value, list) and type(key) is int and 0 <= key < len(value):
            value = value[key]
        else:
            # An addressable receipt with a bad subpath is a discovery query,
            # not a failed flight. Return only its actual structure; never
            # infer the missing key or silently substitute another value.
            result = {'action_index': action_index, 'requested_path': list(path),
                'path': prefix, 'path_found': False, 'historical_action': receipt['action'],
                'historical_status': receipt['result'].get('status'), 'current_evidence': False,
                'message': 'Requested path is absent. Follow one of these actual paths or omit path to inspect the receipt.'}
            if isinstance(value, dict):
                result['fields'] = [{'key': k, 'type': type(v).__name__, 'path': [*prefix, k]}
                                    for k, v in list(value.items())[:16]]
            elif isinstance(value, list):
                result.update(total_items=len(value), next_path=[*prefix, 0] if value else None)
            else:
                result['value_type'] = type(value).__name__
            return result
        prefix.append(key)
    result = {'action_index': action_index, 'path': list(path),
              'historical_action': receipt['action'],
              'historical_status': receipt['result'].get('status'), 'current_evidence': False}
    if isinstance(value, str) and _size(value) > MAX_REPLY_BYTES:
        text = value[offset:offset + 400]
        result.update(data=text, total_characters=len(value),
                      next_offset=offset + len(text) if offset + len(text) < len(value) else None)
    elif isinstance(value, list) and (_size(value) > MAX_REPLY_BYTES or offset):
        chunk = value[offset:offset + limit]
        while chunk and _size(chunk) > MAX_REPLY_BYTES:
            chunk.pop()
        if chunk or offset >= len(value):
            result.update(data=copy.deepcopy(chunk), total_items=len(value),
                          next_offset=offset + len(chunk) if offset + len(chunk) < len(value) else None)
        else:
            result.update(total_items=len(value),
                          next_path=[*path, offset],
                          message='This item needs a narrower path; append its index to path.')
    elif isinstance(value, dict) and _size(value) > MAX_REPLY_BYTES:
        keys = list(value)[offset:offset + limit]
        result.update(fields=[{'key': key, 'type': type(value[key]).__name__,
                               'path': [*path, key]} for key in keys],
                      total_fields=len(value),
                      next_offset=offset + len(keys) if offset + len(keys) < len(value) else None)
    else:
        result['data'] = copy.deepcopy(value)
    return result


def archive_older_observations(observations, keep_recent=2):
    """Keep small factual fields; older large details stay addressable by index.

    Counts, short answers, timestamps and failures remain visible. No input,
    receipt or stored workflow is modified. Unfamiliar evidence fields are
    listed with their exact lookup path rather than silently dropped.
    """
    result = copy.deepcopy(observations)
    for entry in result[:-keep_recent] if keep_recent else result:
        evidence = entry.get('evidence')
        if not isinstance(evidence, dict):
            continue
        index = entry['after_action_index']
        details = []
        for key, value in list(evidence.items()):
            threshold = 512 if isinstance(value, str) else 160
            if _size(value) > threshold:
                del evidence[key]
                path = ['result', 'evidence', key]
                details.append({'field': key, 'path': path,
                                **({'items': len(value), 'latest_path': [*path, len(value)-1]}
                                   if isinstance(value, list) and value else {})})
        if details:
            entry['retained_details'] = {'tool': 'read_action_result', 'action_index': index,
                                         'fields': details}
    return result


def observation_memory_view(observations, history_bytes=2000, keep_recent=2):
    projected = archive_older_observations(observations, keep_recent=keep_recent)
    # A single long scan can exceed the window even before it becomes history.
    # Expose exact paging paths, including the latest station, instead of
    # silently cutting a station array or treating missing evidence as false.
    for index, entry in enumerate(projected):
        if _size(entry) > 4000:
            projected[index] = archive_older_observations([entry], keep_recent=0)[0]
    archived = []
    while len(projected) > keep_recent and _size(
            projected[:-keep_recent] if keep_recent else projected) > history_bytes:
        archived.append(projected.pop(0)['after_action_index'])
    view = {'observations': projected}
    if archived:
        view['archived_observations'] = {
            'tool': 'read_action_result', 'action_indices': archived,
            'path': ['result', 'evidence'],
            'reason': 'Older evidence retained outside the decision context; fetch when needed.'}
    return view


def lean_execution_receipt(entry):
    """Preserve action identity/status; large arguments/diagnostics stay readable."""
    result = copy.deepcopy(entry)
    index = result.get('through_index', result['index'])
    for key, path in (('arguments', ['arguments']), ('motion', ['result','motion']),
                      ('message', ['result','message'])):
        if key in result and _size(result[key]) > 256:
            result[key] = {'retained_action_index': index, 'path': path,
                           'tool': 'read_action_result'}
    return result

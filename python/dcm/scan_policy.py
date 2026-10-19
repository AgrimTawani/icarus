"""Explicit lifecycle for reusable observation workflows, not mission planning."""
import copy

EXIT_POLICY = {
    'type': 'string',
    'enum': ['each_observation', 'pause_at_match', 'finish_at_match', 'complete_coverage'],
    'description': ('Choose the scan lifecycle: each_observation yields for arbitrary decisions; '
                    'pause_at_match yields on a detector predicate and permits resuming; '
                    'finish_at_match ends this scan permanently at the first match; '
                    'complete_coverage performs an unconditional bounded scan. '
                    'Match policies require stop_on_detection and detection observation.')}


def scan_arguments(arguments, *, default_observation, coverage_return):
    """Validate lifecycle choices before movement; retain legacy Python callers.

    New agent calls must select a lifecycle in the advertised schema. No user
    text is inspected and no predicate is inferred from class names or counts.
    """
    args = copy.deepcopy(arguments)
    policy = args.get('exit_policy')
    if policy is None:
        return args
    if policy not in EXIT_POLICY['enum']:
        raise ValueError('Unknown scan exit_policy')
    match = policy in ('pause_at_match', 'finish_at_match')
    if match:
        if args.get('observation', default_observation) != 'detect':
            raise ValueError('A detector match policy requires observation=detect')
        if not args.get('stop_on_detection') or args['stop_on_detection'] not in args.get('classes', []):
            raise ValueError('A match policy requires stop_on_detection from the requested classes')
    elif args.get('stop_on_detection') or 'minimum_count' in args:
        raise ValueError('A detection predicate requires pause_at_match or finish_at_match')
    if args.get('repeat_until_match') and policy in ('complete_coverage', 'pause_at_match'):
        raise ValueError('repeat_until_match cannot use this exit_policy')
    expected_return = coverage_return if policy == 'complete_coverage' else 'observation'
    if 'return_on' in args and args['return_on'] != expected_return:
        raise ValueError('return_on conflicts with exit_policy')
    args['return_on'] = expected_return
    return args


def completion_reason(completed, observations):
    if not completed:
        return None
    last = observations[-1] if observations else {}
    return ('detection_match' if last.get('workflow_stop_filter', {}).get('matched')
            else 'coverage_complete')


def failure_receipt(result, workflow_receipt):
    """Keep the original action failure and the workflow's pending obligation."""
    return {**workflow_receipt, **result, 'evidence': workflow_receipt['evidence']}

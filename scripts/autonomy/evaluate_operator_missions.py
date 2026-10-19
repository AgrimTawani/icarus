#!/usr/bin/env python3
"""Run operator-supplied prompts through the live agent and retain raw evidence.

This harness never supplies expected actions, scenario truth or answer labels
to the mission model. Completion is the agent's assessment, not an independent
mission-success score. Human review must inspect each episode and camera proof.
"""
import argparse
import hashlib
import importlib.util
import json
import sys
import subprocess
import time
from contextlib import redirect_stdout
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT))

from python.dcm.fly import AgentSession, fly_mission, safe_exit_recovery, session_episode_summary
from python.dcm.llama_runtime import LlamaCppRuntime
from python.dcm.runtime_config import RuntimeDescriptor


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--cases', type=Path, default=ROOT / 'tests/fixtures/operator_phase1_missions.json')
    parser.add_argument('--case', action='append')
    parser.add_argument('--manifest', type=Path, default=ROOT / 'config/dcm/agent-runtime.json')
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--max-decisions', type=int, default=32)
    parser.add_argument('--clarification-answer', action='append', default=[],
                        help='Explicit simulated operator reply, consumed only after ask_user; repeat for multiple replies')
    args = parser.parse_args()
    cases = json.loads(args.cases.read_text())['cases']
    if args.case:
        unknown = set(args.case) - {case['id'] for case in cases}
        if unknown:
            parser.error('Unknown case IDs: ' + ', '.join(sorted(unknown)))
        cases = [case for case in cases if case['id'] in args.case]
    args.output.mkdir(parents=True, exist_ok=True)
    spec = importlib.util.spec_from_file_location('icarus_eval_client', ROOT / 'scripts/autonomy/run_mission.py')
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    descriptor = RuntimeDescriptor.from_manifest(args.manifest)
    runtime = LlamaCppRuntime(descriptor)
    runtime_started = False
    results = []
    paths = [*sorted((ROOT / 'python/dcm').glob('*.py')),
             *sorted((ROOT / 'python/perception').glob('*.py')),
             ROOT / 'scripts/autonomy/run_mission.py',
             ROOT / 'scripts/autonomy/evaluate_operator_missions.py',
             ROOT / 'scripts/autonomy/run_operator_acceptance.py',
             ROOT / 'scripts/autonomy/record_camera_evidence.py',
             ROOT / 'build/phase8/icarus-drone-api',
             *sorted((ROOT / 'config/dcm/prompts').glob('*.md')),
             ROOT / 'config/dcm/default_policy.json',
             *sorted((ROOT / 'config/perception').glob('*.json')),
             ROOT / 'config/safety/v1.yaml', args.cases, args.manifest]
    provenance = {
        'runtime': descriptor.as_record(),
        'git_commit': subprocess.check_output(['git', '-C', str(ROOT), 'rev-parse', 'HEAD'], text=True).strip(),
        'source_sha256': {str(p.relative_to(ROOT)): hashlib.sha256(p.read_bytes()).hexdigest()
                          for p in paths if p.is_relative_to(ROOT)},
        'results': results}
    def save_results():
        pending = args.output / 'results.json.tmp'
        pending.write_text(json.dumps(provenance, indent=2) + '\n')
        pending.replace(args.output / 'results.json')
    try:
        print('Preparing camera and evaluation runtime…', flush=True)
        for case in cases:
            print('Starting ' + case['id'], flush=True)
            started = time.monotonic()
            result = {'case': case['id'], 'mission': case['mission'],
                      'status': 'running',
                      'independent_mission_verification': False}
            results.append(result)
            save_results()
            client = module.MissionClient('127.0.0.1:50051', case['mission'], 'edge-vision')
            # Evaluation-only camera evidence. The live agent still receives
            # ordinary tool results; no screenshots, labels or scenario truth
            # are injected into its conversation by this harness.
            capture = client._capture_rgb
            camera_directory = args.output / case['id'] / 'camera'
            camera_directory.mkdir(parents=True, exist_ok=True)
            def retain_capture(output, camera='forward_d435i'):
                source = capture(output, camera=camera)
                from PIL import Image
                stamp = time.time_ns()
                destination = camera_directory / f'{stamp}-{camera}.png'
                Image.open(output).convert('RGB').save(destination)
                from python.dcm.agent_tools import message_dict
                destination.with_suffix('.json').write_text(json.dumps({
                    'source': source, 'camera': camera, 'state': message_dict(client.state())}, indent=2))
                return source
            client._capture_rgb = retain_capture
            with (args.output / (case['id'] + '.log')).open('w') as log, redirect_stdout(log):
                try:
                    client.connect()
                    client.acquire()
                    client.prepare_camera_detector()
                    # Match the console: GPU fitting must see the camera
                    # detector's allocation, not claim its reserved memory.
                    if not runtime_started:
                        runtime.start()
                        runtime_started = True
                    session = AgentSession(case['mission'])
                    replies = iter(args.clarification_answer)
                    while True:
                        result.update(fly_mission(client, runtime, case['mission'],
                            max_decisions=args.max_decisions, session=session,
                            echo=lambda *items: print(*items, flush=True)))
                        if result['status'] != 'awaiting_user':
                            break
                        answer = next(replies, None)
                        if answer is None:
                            break
                        exchange = {'question': result['message'], 'answer': answer}
                        result.setdefault('simulated_operator_replies', []).append(exchange)
                        client.episode.record('evaluation_operator_reply', exchange)
                        print('Simulated operator reply: ' + answer, flush=True)
                        session.answer(answer)
                    client.episode_outcome, client.episode_score = session_episode_summary([result])
                except KeyboardInterrupt:
                    result.update(status='cancelled', message='Evaluation interrupted by operator')
                    client.episode_outcome = 'incomplete'
                except Exception as error:
                    result.update(status='error', message=f'{type(error).__name__}: {error}')
                    client.episode_outcome = 'failed'
                finally:
                    try:
                        result['recovery'] = safe_exit_recovery(client)
                    except Exception as error:
                        result['recovery'] = {'status': 'FAILED', 'message': str(error)}
                    finally:
                        if result.get('recovery', {}).get('status') not in ('SUCCEEDED', 'manual_override'):
                            if result.get('status') == 'completed':
                                result['agent_status'] = result['status']
                                result['status'] = 'incomplete'
                            client.episode_outcome, client.episode_score = session_episode_summary(
                                [result], 'Exit recovery did not complete')
                        episode = getattr(client, 'episode', None)
                        result['episode'] = str(episode.directory) if episode is not None else None
                        try:
                            client.close()
                        except Exception as error:
                            result['cleanup_error'] = str(error)
            result['seconds'] = round(time.monotonic() - started, 2)
            changed = [name for name, digest in provenance['source_sha256'].items()
                       if not (ROOT / name).is_file()
                       or hashlib.sha256((ROOT / name).read_bytes()).hexdigest() != digest]
            if changed:
                provenance['source_changed_during_campaign'] = changed
            save_results()
            print(case['id'] + ': ' + result.get('status', 'unknown'), flush=True)
            if changed:
                print('Stopping after recovery: source/config changed; start a fresh campaign.', flush=True)
                break
            if not runtime_started:
                print('Stopping campaign: mission runtime did not start.', flush=True)
                break
            if result.get('status') == 'cancelled':
                break
            if result.get('recovery', {}).get('status') not in ('SUCCEEDED', 'not_needed'):
                print('Stopping campaign: recovery needs review.', flush=True)
                break
    finally:
        runtime.stop()


if __name__ == '__main__':
    main()

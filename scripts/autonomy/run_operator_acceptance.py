#!/usr/bin/env python3
"""Own a headless acceptance stack and clean up only its processes."""
import argparse
import json
import os
import signal
import socket
import subprocess
import sys
import time
from contextlib import ExitStack
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]


def process_identity(pid):
    try:
        fields = Path(f'/proc/{pid}/stat').read_text().rsplit(')', 1)[1].split()
        return int(fields[1]), fields[19], fields[0]
    except (OSError, ValueError, IndexError):
        return None


def descendants(pid):
    records = {}
    for path in Path('/proc').glob('[0-9]*/stat'):
        child = int(path.parent.name)
        identity = process_identity(child)
        if identity:
            records[child] = identity
    parents = {pid}; owned = set()
    while True:
        found = {child for child, record in records.items()
                 if record[0] in parents and (child, record[1]) not in owned}
        if not found:
            break
        owned.update((child, records[child][1]) for child in found)
        parents = found
    return owned


def stop_owned(process, timeout=15, tracked=()):
    if process is None:
        return
    children = descendants(process.pid) | set(tracked)
    try:
        if process.poll() is None:
            try:
                os.killpg(process.pid, signal.SIGINT)
                process.wait(timeout=timeout)
            except (ProcessLookupError, subprocess.TimeoutExpired):
                if process.poll() is None:
                    os.killpg(process.pid, signal.SIGTERM)
                    try:
                        process.wait(timeout=3)
                    except subprocess.TimeoutExpired:
                        os.killpg(process.pid, signal.SIGKILL)
                        process.wait(timeout=3)
    finally:
        # Detached model/sensor children may survive an abnormal parent exit.
        # Match the recorded start time before signalling, guarding PID reuse.
        for signum in (signal.SIGTERM, signal.SIGKILL):
            for pid, start in children:
                current = process_identity(pid)
                if current and current[1] == start and current[2] != 'Z':
                    try:
                        os.kill(pid, signum)
                    except ProcessLookupError:
                        pass
            if signum == signal.SIGTERM:
                time.sleep(0.1)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--cases', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--case', action='append', default=[])
    parser.add_argument('--clarification-answer', action='append', default=[])
    parser.add_argument('--max-decisions', type=int, default=32)
    args = parser.parse_args()
    args.output = args.output.resolve()
    args.cases = args.cases.resolve()
    requested = json.loads(args.cases.read_text())['cases']
    expected_count = sum(1 for case in requested if not args.case or case['id'] in args.case)
    with socket.socket() as probe:
        if probe.connect_ex(('127.0.0.1', 50051)) == 0:
            parser.error('A Drone API is already listening; this runner owns a separate acceptance stack.')
    active = ROOT / 'logs/simulation/active_session.json'
    if active.exists():
        session = json.loads(active.read_text())
        if process_identity(session.get('launcher_pid', -1)):
            parser.error('A simulator is already active; stop it before starting this owned acceptance stack.')
    args.output.mkdir(parents=True, exist_ok=True)
    processes = {}; tracked = {}
    def remember():
        for name, process in processes.items():
            tracked.setdefault(name, set()).update(descendants(process.pid))
    def start(name, command, stack):
        log = stack.enter_context((args.output / (name + '.log')).open('w'))
        child = subprocess.Popen(command, cwd=ROOT, stdout=log, stderr=subprocess.STDOUT,
                                 start_new_session=True)
        processes[name] = child
        (args.output / 'owned-processes.json').write_text(json.dumps(
            {key: value.pid for key, value in processes.items()}, indent=2) + '\n')
        return child
    def wait_until(check, process, seconds):
        deadline = time.monotonic() + seconds
        while time.monotonic() < deadline:
            remember()
            if process.poll() is not None:
                raise RuntimeError('Owned process exited during startup; see its log')
            if check():
                return
            time.sleep(0.25)
        raise TimeoutError('Owned stack readiness deadline exceeded')
    def simulator_ready():
        try:
            data = json.loads(active.read_text())
            return data.get('status') == 'ready' and data.get('launcher_pid') == processes['simulator'].pid
        except (OSError, ValueError):
            return False
    def api_ready():
        with socket.socket() as probe:
            return probe.connect_ex(('127.0.0.1', 50051)) == 0
    code = 1
    with ExitStack() as stack:
        try:
            simulator = start('simulator', [str(ROOT/'scripts/start-sim'), '--profile', 'edge-vision'], stack)
            wait_until(simulator_ready, simulator, 240)
            api = start('autonomy', [str(ROOT/'scripts/start-autonomy'), '--profile', 'edge-vision'], stack)
            wait_until(api_ready, api, 240)
            command = [sys.executable, str(ROOT/'scripts/autonomy/evaluate_operator_missions.py'),
                       '--cases', str(args.cases.resolve()), '--output', str((args.output/'missions').resolve()),
                       '--max-decisions', str(args.max_decisions)]
            for value in args.case:
                command += ['--case', value]
            for value in args.clarification_answer:
                command += ['--clarification-answer', value]
            evaluator = start('evaluation', command, stack)
            start('camera', [sys.executable, str(ROOT/'scripts/autonomy/record_camera_evidence.py'),
                 '--process', str(evaluator.pid), '--directory', str((args.output/'camera').resolve())], stack)
            print('Headless acceptance running; evidence: ' + str(args.output), flush=True)
            while evaluator.poll() is None:
                remember()
                if simulator.poll() is not None or api.poll() is not None:
                    raise RuntimeError('Simulation/API exited during evaluation')
                time.sleep(1)
            code = evaluator.returncode
            if code == 0:
                report = json.loads((args.output/'missions/results.json').read_text())
                results = report['results']
                if len(results) != expected_count or any(row.get('status') != 'completed' for row in results):
                    code = 2
                print('Campaign finished; review receipts and camera evidence independently.', flush=True)
        except KeyboardInterrupt:
            print('Stopping evaluation and recovering before releasing the owned stack.', flush=True)
            code = 130
        finally:
            cleanup_errors = []
            for name, timeout in (('evaluation',120),('camera',5),('autonomy',15),('simulator',20)):
                try:
                    stop_owned(processes.get(name), timeout=timeout, tracked=tracked.get(name, ()))
                except Exception as error:
                    cleanup_errors.append(name + ': ' + str(error))
            if cleanup_errors:
                print('Cleanup needs review: ' + '; '.join(cleanup_errors), flush=True)
                code = 1
            else:
                print('Owned acceptance processes stopped.', flush=True)
    return code


if __name__ == '__main__':
    raise SystemExit(main())

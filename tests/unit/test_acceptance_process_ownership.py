"""Fast process-lifecycle checks; no simulator or models are started."""
import importlib.util
import json
import os
import signal
import subprocess
import sys
from pathlib import Path
from types import SimpleNamespace


def load_runner():
    path=Path(__file__).resolve().parents[2]/'scripts/autonomy/run_operator_acceptance.py'
    spec=importlib.util.spec_from_file_location('owned_acceptance',path)
    module=importlib.util.module_from_spec(spec);spec.loader.exec_module(module)
    return module


def test_forced_parent_cleanup_stops_detached_child_but_preserves_unrelated_process():
    module=load_runner()
    code=('import subprocess,sys,time,signal; signal.signal(signal.SIGINT,signal.SIG_IGN); '
          'p=subprocess.Popen([sys.executable,"-c","import time;time.sleep(30)"],start_new_session=True); '
          'print(p.pid,flush=True); time.sleep(30)')
    parent=subprocess.Popen([sys.executable,'-c',code],stdout=subprocess.PIPE,text=True,start_new_session=True)
    unrelated=subprocess.Popen([sys.executable,'-c','import time;time.sleep(30)'],start_new_session=True)
    try:
        child=int(parent.stdout.readline())
        identity=module.process_identity(child)
        module.stop_owned(parent,timeout=0.05)
        after=module.process_identity(child)
        assert parent.poll() is not None
        assert after is None or after[1]!=identity[1] or after[2]=='Z'
        assert unrelated.poll() is None
    finally:
        module.stop_owned(parent,timeout=0.05)
        unrelated.kill();unrelated.wait(timeout=3)
        parent.stdout.close()


def test_cases_use_separate_owned_stacks_and_keep_partial_results(tmp_path, monkeypatch):
    module=load_runner()
    args=SimpleNamespace(output=tmp_path/'run', cases=tmp_path/'cases.json',
                         max_decisions=20, clarification_answer=['Use a measured viewpoint.'])
    calls=[]; cleaned=[]
    class Finished:
        pid=123456789
        def __init__(self, code): self.returncode=code
        def poll(self): return self.returncode
    def launch(command, **kwargs):
        calls.append(command)
        case=command[command.index('--case')+1]
        directory=Path(command[command.index('--output')+1])/'missions'
        directory.mkdir()
        status='awaiting_user' if case=='missing_location' else 'completed'
        (directory/'results.json').write_text(json.dumps({'source_sha256':{},
            'results':[{'case':case,'status':status}]}))
        return Finished(2 if status=='awaiting_user' else 0)
    monkeypatch.setattr(module.subprocess,'Popen',launch)
    monkeypatch.setattr(module,'stop_owned',lambda process, **kwargs:cleaned.append(process))
    code=module.run_isolated_cases(args,[{'id':'missing_location'},{'id':'flight'}])
    assert code==2
    assert len(calls)==len(cleaned)==2
    directories=[c[c.index('--output')+1] for c in calls]
    assert len(set(directories))==2
    assert all(c.count('--case')==1 and '--clarification-answer' in c for c in calls)
    report=json.loads((args.output/'missions/results.json').read_text())
    assert [r['status'] for r in report['results']]==['awaiting_user','completed']
    assert set(report['case_campaigns'])=={'missing_location','flight'}


def test_interrupt_recovers_current_stack_and_never_starts_next_case(tmp_path, monkeypatch):
    module=load_runner()
    args=SimpleNamespace(output=tmp_path/'run',cases=tmp_path/'cases.json',
                         max_decisions=20,clarification_answer=[])
    calls=[]; cleaned=[]
    class Interrupted:
        pid=123456789
        def poll(self): raise KeyboardInterrupt
    def launch(command, **kwargs):
        calls.append(command);return Interrupted()
    monkeypatch.setattr(module.subprocess,'Popen',launch)
    monkeypatch.setattr(module,'stop_owned',lambda process, **kwargs:cleaned.append(kwargs['timeout']))
    assert module.run_isolated_cases(args,[{'id':'first'},{'id':'second'}])==130
    assert len(calls)==1
    assert cleaned==[180]


def test_sigterm_runs_owned_child_cleanup_instead_of_orphaning_it():
    module=load_runner()
    code=("import importlib.util,signal,subprocess,sys,time; "
          f"s=importlib.util.spec_from_file_location('runner',{str(Path(module.__file__))!r}); "
          "m=importlib.util.module_from_spec(s);s.loader.exec_module(m); "
          "signal.signal(signal.SIGTERM,m.terminate_as_interrupt); "
          "child=subprocess.Popen([sys.executable,'-c','import time;time.sleep(30)'],start_new_session=True); "
          "print(child.pid,flush=True)\n"
          "try:\n time.sleep(30)\n"
          "except KeyboardInterrupt:\n m.stop_owned(child,timeout=1)\n")
    parent=subprocess.Popen([sys.executable,'-c',code],stdout=subprocess.PIPE,
                            text=True,start_new_session=True)
    child=None
    try:
        child=int(parent.stdout.readline())
        before=module.process_identity(child)
        parent.send_signal(signal.SIGTERM)
        parent.wait(timeout=5)
        after=module.process_identity(child)
        assert parent.returncode==0
        assert after is None or after[1]!=before[1] or after[2]=='Z'
    finally:
        module.stop_owned(parent,timeout=0.1)
        after=module.process_identity(child) if child else None
        if after and after[1]==before[1] and after[2]!='Z':
            try: os.kill(child,signal.SIGKILL)
            except ProcessLookupError: pass
        parent.stdout.close()

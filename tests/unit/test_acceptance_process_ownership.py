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
                         manifest=tmp_path/'selected-runtime.json', max_decisions=20, clarification_answer=['Use a measured viewpoint.'], runtime='cloud')
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
    assert all(c[c.index('--manifest')+1]==str(args.manifest) for c in calls)
    report=json.loads((args.output/'missions/results.json').read_text())
    assert [r['status'] for r in report['results']]==['awaiting_user','completed']
    assert set(report['case_campaigns'])=={'missing_location','flight'}


def test_interrupt_recovers_current_stack_and_never_starts_next_case(tmp_path, monkeypatch):
    module=load_runner()
    args=SimpleNamespace(output=tmp_path/'run',cases=tmp_path/'cases.json',
                         manifest=tmp_path/'selected-runtime.json',max_decisions=20,clarification_answer=[],runtime='cloud')
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


def test_primary_manifest_reaches_the_evaluator_without_starting_a_simulator(tmp_path):
    module=load_runner()
    args=SimpleNamespace(cases=tmp_path/'cases.json',output=tmp_path/'run',
        manifest=tmp_path/'candidate-runtime.json',case=['one'],
        max_decisions=20,clarification_answer=['Use another view.'],runtime='local')
    command=module.evaluation_command(args)
    assert command[command.index('--manifest')+1]==str(args.manifest.resolve())
    assert command.count('--manifest')==1 and command[command.index('--case')+1]=='one'
    assert command[command.index('--clarification-answer')+1]=='Use another view.'


def test_nonterminal_report_cannot_be_promoted_when_evaluator_has_exited():
    module=load_runner()
    source={'results':[{'case':'example','status':'running'},
                       {'case':'finished','status':'completed','recovery':{'status':'SUCCEEDED'}}]}
    report=module.terminal_campaign(source,130)
    assert report['evaluation_report_incomplete'] is True
    assert report['results'][0]['status']=='error'
    assert report['results'][0]['saved_status_at_exit']=='running'
    assert report['results'][0]['process_exit_code']==130
    assert 'unverified' in report['results'][0]['message']
    assert report['results'][1]==source['results'][1]
    assert source['results'][0]['status']=='running'


def test_stop_during_recovery_is_deferred_and_signal_handlers_are_restored():
    path=Path(__file__).resolve().parents[2]/'scripts/autonomy/evaluate_operator_missions.py'
    code='''import importlib.util,json,os,signal,sys
spec=importlib.util.spec_from_file_location('evaluation',sys.argv[1]);module=importlib.util.module_from_spec(spec);spec.loader.exec_module(module)
previous=signal.getsignal(signal.SIGINT);termination=signal.getsignal(signal.SIGTERM);result={}
with module.defer_stop_during_recovery(result):
    os.kill(os.getpid(),signal.SIGINT)
    result['recovery']={'status':'SUCCEEDED'}
    assert signal.getsignal(signal.SIGTERM)==termination
assert signal.getsignal(signal.SIGINT)==previous
print(json.dumps(result))
'''
    completed=subprocess.run([sys.executable,'-c',code,str(path)],capture_output=True,text=True,check=True,timeout=10)
    result=json.loads(completed.stdout)
    assert result=={'recovery_interrupts_deferred':1,'recovery':{'status':'SUCCEEDED'}}


def test_parent_stop_still_collects_the_childs_terminal_recovery_report(tmp_path,monkeypatch):
    module=load_runner()
    args=SimpleNamespace(output=tmp_path,cases=tmp_path/'cases.json',manifest=tmp_path/'model.json',
                         max_decisions=4,clarification_answer=[],runtime='cloud')
    case={'id':'case'}
    class Child:
        pid=123
        returncode=130
        def poll(self):raise KeyboardInterrupt()
    def recovered(*args,**kwargs):
        result=tmp_path/'cases/case/missions/results.json';result.parent.mkdir(parents=True,exist_ok=True)
        result.write_text(json.dumps({'results':[{'case':'case','status':'cancelled','recovery':{'status':'SUCCEEDED'}}]}))
    monkeypatch.setattr(module.subprocess,'Popen',lambda *a,**k:Child())
    monkeypatch.setattr(module,'stop_owned',recovered)
    monkeypatch.setattr(module,'descendants',lambda pid:set())
    assert module.run_isolated_cases(args,[case])==130
    report=json.loads((tmp_path/'missions/results.json').read_text())
    assert report['results'][0]['recovery']['status']=='SUCCEEDED'
    assert report['results'][0]['status']=='cancelled'


def test_fixture_answer_does_not_amend_an_unrelated_operator_question():
    path=Path(__file__).resolve().parents[2]/'scripts/autonomy/evaluate_operator_missions.py'
    spec=importlib.util.spec_from_file_location('guarded_eval',path)
    module=importlib.util.module_from_spec(spec);spec.loader.exec_module(module)
    item={'question_pattern':'downward camera','answer':'Use the downward camera.'}
    assert module.fixture_reply(item,'May I use the downward camera?')==item['answer']
    assert module.fixture_reply(item,'What appearance identifies the subject?') is None
    assert module.fixture_reply(item,'Should I change the flight height?') is None


def test_runtime_choice_reaches_both_evaluator_commands(tmp_path):
    module=load_runner()
    for runtime in ('cloud','local'):
        args=SimpleNamespace(cases=tmp_path/'cases.json',output=tmp_path/'run',manifest=tmp_path/'m.json',
                             case=[],max_decisions=4,clarification_answer=[],runtime=runtime)
        for command in (module.evaluation_command(args),module.isolated_case_command(args,'one',tmp_path/'one')):
            assert command[command.index('--runtime')+1]==runtime


def test_acceptance_runtime_defaults_to_cloud_and_accepts_llama_alias(monkeypatch):
    import argparse
    import pytest
    module=load_runner()
    original=argparse.ArgumentParser.parse_args
    seen=[]
    def spy(self,*a,**k):
        seen.append(original(self,*a,**k))
        raise SystemExit(0)
    monkeypatch.setattr(argparse.ArgumentParser,'parse_args',spy)
    for extra,expected in (([],'cloud'),(['--runtime','local'],'local'),(['--runtime','llama'],'local')):
        monkeypatch.setattr(sys,'argv',['acceptance','--cases','c.json','--output','out',*extra])
        with pytest.raises(SystemExit):
            module.main()
        assert seen[-1].runtime==expected


def test_model_runtime_sources_are_pinned_for_operator_evaluation():
    path=Path(__file__).resolve().parents[2]/'scripts/autonomy/evaluate_operator_missions.py'
    spec=importlib.util.spec_from_file_location('pinned_eval',path)
    module=importlib.util.module_from_spec(spec);spec.loader.exec_module(module)
    names={p.relative_to(module.ROOT).as_posix() for p in module.pinned_python_sources()}
    assert 'python/model_runtime/cloud.py' in names
    assert 'python/model_runtime/select.py' in names
    assert 'python/dcm/fly.py' in names


def test_cloud_settings_are_checked_before_anything_is_started(tmp_path,monkeypatch,capsys):
    import pytest
    module=load_runner()
    for name in ('ICARUS_LLM_BASE_URL','ICARUS_LLM_API_KEY','ICARUS_LLM_MODEL'):
        monkeypatch.delenv(name,raising=False)
    cases=tmp_path/'cases.json';cases.write_text(json.dumps({'cases':[{'id':'one','mission':'m'}]}))
    started=[]
    monkeypatch.setattr(module.subprocess,'Popen',lambda *a,**k:started.append(a))
    monkeypatch.setattr(module,'run_isolated_cases',lambda *a,**k:started.append(a))
    output=tmp_path/'out'
    monkeypatch.setattr(sys,'argv',['acceptance','--cases',str(cases),'--output',str(output)])
    with pytest.raises(SystemExit) as stop:
        module.main()
    assert stop.value.code==2
    err=capsys.readouterr().err
    assert 'ICARUS_LLM_BASE_URL is not set' in err and '--runtime local' in err and 'Traceback' not in err
    assert started==[] and not output.exists()


def test_an_unusable_cloud_setting_also_stops_the_runner_before_the_simulator(tmp_path,monkeypatch,capsys):
    import pytest
    module=load_runner()
    monkeypatch.setenv('ICARUS_LLM_BASE_URL','http://api.example.com/v1')
    monkeypatch.setenv('ICARUS_LLM_API_KEY','sk-secret');monkeypatch.setenv('ICARUS_LLM_MODEL','m')
    cases=tmp_path/'cases.json';cases.write_text(json.dumps({'cases':[{'id':'one','mission':'m'}]}))
    started=[]
    monkeypatch.setattr(module.subprocess,'Popen',lambda *a,**k:started.append(a))
    monkeypatch.setattr(sys,'argv',['acceptance','--cases',str(cases),'--output',str(tmp_path/'out')])
    with pytest.raises(SystemExit) as stop:
        module.main()
    assert stop.value.code==2 and started==[]
    err=capsys.readouterr().err
    assert 'ICARUS_LLM_BASE_URL must be an https:// URL' in err and 'sk-secret' not in err


def test_the_local_runtime_does_not_need_cloud_settings_to_start_the_runner(tmp_path,monkeypatch,capsys):
    import pytest
    module=load_runner()
    for name in ('ICARUS_LLM_BASE_URL','ICARUS_LLM_API_KEY','ICARUS_LLM_MODEL'):
        monkeypatch.delenv(name,raising=False)
    cases=tmp_path/'cases.json';cases.write_text(json.dumps({'cases':[{'id':'one','mission':'m'}]}))
    monkeypatch.setattr(module.socket.socket,'connect_ex',lambda self,address:0)  # a Drone API "already listening"
    monkeypatch.setattr(sys,'argv',['acceptance','--cases',str(cases),'--output',str(tmp_path/'out'),'--runtime','local'])
    with pytest.raises(SystemExit) as stop:
        module.main()
    assert stop.value.code==2
    err=capsys.readouterr().err  # stopped by the port guard: it got past the skipped cloud check
    assert 'already listening' in err and 'ICARUS_LLM' not in err


def test_local_provenance_records_the_descriptor_and_pins_the_server_binary_library_and_patches(tmp_path):
    path=Path(__file__).resolve().parents[2]/'scripts/autonomy/evaluate_operator_missions.py'
    spec=importlib.util.spec_from_file_location('provenance_eval',path)
    module=importlib.util.module_from_spec(spec);spec.loader.exec_module(module)
    binary=tmp_path/'bin/llama-server';binary.parent.mkdir();binary.write_text('x')
    (binary.parent/'libllama-common.so.0.0.1').write_text('x');(binary.parent/'libllama-common.so.0.0.2').write_text('x')
    descriptor=SimpleNamespace(as_record=lambda:{'model_id':'local-model','runtime':'llama.cpp'})
    record,paths=module.runtime_provenance('local',SimpleNamespace(binary=binary),descriptor)
    assert record=={'model_id':'local-model','runtime':'llama.cpp'}
    assert paths[0]==binary.resolve()
    assert paths[1]==binary.parent/'libllama-common.so.0.0.1'  # the first library only, as before
    patches=sorted((module.ROOT/'config/dependencies/patches').glob('*.patch'))
    assert patches and paths[2:]==patches


def test_cloud_provenance_is_the_settings_record_and_pins_no_local_server():
    path=Path(__file__).resolve().parents[2]/'scripts/autonomy/evaluate_operator_missions.py'
    spec=importlib.util.spec_from_file_location('provenance_eval_cloud',path)
    module=importlib.util.module_from_spec(spec);spec.loader.exec_module(module)
    record={'provider_host':'api.example.com','model':'m'}
    assert module.runtime_provenance('cloud',object(),record)==(record,[])

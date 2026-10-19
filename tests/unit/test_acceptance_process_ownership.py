"""Fast process-lifecycle checks; no simulator or models are started."""
import importlib.util
import subprocess
import sys
from pathlib import Path


def test_forced_parent_cleanup_stops_detached_child_but_preserves_unrelated_process():
    path=Path(__file__).resolve().parents[2]/'scripts/autonomy/run_operator_acceptance.py'
    spec=importlib.util.spec_from_file_location('owned_acceptance',path)
    module=importlib.util.module_from_spec(spec);spec.loader.exec_module(module)
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

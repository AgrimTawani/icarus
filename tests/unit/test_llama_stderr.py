"""Real child-process diagnostics stress without loading any model/GPU."""
import json
import sys
from unittest.mock import patch

import pytest
from test_llama_runtime import descriptor

from python.dcm.contract import RuntimeDescriptor
from python.dcm.llama_runtime import STDERR_TAIL_BYTES, LlamaCppRuntime


def executable(tmp_path, body):
    path = tmp_path / 'fake-server'
    path.write_text(f'#!{sys.executable}\n' + body)
    path.chmod(0o755)
    return path


def test_startup_flood_cannot_block_and_tail_and_handles_are_bounded(tmp_path):
    binary = executable(tmp_path, "import os\nfor _ in range(512): os.write(2, b'x'*8192)\nos.write(2, b'FINAL_DIAGNOSTIC')\nraise SystemExit(1)\n")
    runtime = LlamaCppRuntime(descriptor(), binary=binary, verify_artifact=False)
    with patch('python.dcm.llama_runtime.STARTUP_TIMEOUT_S', 3), pytest.raises(RuntimeError, match='FINAL_DIAGNOSTIC'):
        runtime.start()
    assert runtime.process is None
    assert runtime._stderr_thread is None
    assert len(runtime._stderr_tail) <= STDERR_TAIL_BYTES
    # Reuse must reset drain lifecycle without retaining old threads or FDs.
    with patch('python.dcm.llama_runtime.STARTUP_TIMEOUT_S', 3), pytest.raises(RuntimeError, match='FINAL_DIAGNOSTIC'):
        runtime.start()
    assert runtime.process is None
    assert runtime._stderr_thread is None


@pytest.mark.parametrize("mode", [None, "off"])
def test_live_flood_drains_until_stop_and_preserves_reasoning_option(tmp_path, mode):
    args_file = tmp_path / 'args.json'
    binary = executable(tmp_path, f"import os,sys,json\nopen({str(args_file)!r}, 'w').write(json.dumps(sys.argv))\nwhile True: os.write(2, b'continuous-diagnostics'*1024)\n")
    config = descriptor().as_record() | {'reasoning_mode': mode}
    runtime = LlamaCppRuntime(RuntimeDescriptor(**config), binary=binary, verify_artifact=False)
    def ready():
        import time
        deadline = time.monotonic() + 3
        while len(runtime._stderr_tail) < STDERR_TAIL_BYTES and time.monotonic() < deadline:
            time.sleep(.01)
        assert len(runtime._stderr_tail) == STDERR_TAIL_BYTES
    with patch.object(runtime, '_await_ready', side_effect=ready), patch.object(runtime, '_warm'):
        try:
            runtime.start()
            process, reader = runtime.process, runtime._stderr_thread
            assert process.poll() is None
            argv = json.loads(args_file.read_text())
            if mode is None:
                assert '--reasoning' not in argv
            else:
                assert argv[-2:] == ['--reasoning', mode]
        finally:
            runtime.stop()
    assert process.poll() is not None
    assert process.stderr.closed
    assert not reader.is_alive()


@pytest.mark.parametrize('mode', [None, 'on', 'off', 'auto'])
def test_generic_manifest_reasoning_mode_provenance(tmp_path, mode):
    manifest = {'llama_cpp_revision': 'test', 'reasoning_mode': mode,
                'artifacts': [{'role': 'primary', 'path': '/fake.gguf', 'quantization': 'q5', 'sha256': '0'*64}]}
    path = tmp_path / 'manifest.json'; path.write_text(json.dumps(manifest))
    value = RuntimeDescriptor.from_manifest(path)
    assert value.reasoning_mode == mode
    assert value.as_record()['reasoning_mode'] == mode
    manifest['artifacts'][0]['reasoning_mode'] = 'off'
    path.write_text(json.dumps(manifest))
    assert RuntimeDescriptor.from_manifest(path).reasoning_mode == 'off'


def test_invalid_reasoning_mode_is_rejected():
    with pytest.raises(ValueError, match='reasoning_mode'):
        RuntimeDescriptor(**(descriptor().as_record() | {'reasoning_mode': 'model-name-special-case'}))

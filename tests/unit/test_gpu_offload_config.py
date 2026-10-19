"""Generic GPU fitting command/config checks; never load a model or GPU."""
import dataclasses
import json
from unittest.mock import Mock, patch

import pytest
from test_llama_runtime import descriptor

from python.dcm.contract import RuntimeDescriptor
from python.dcm.llama_runtime import LlamaCppRuntime


@pytest.mark.parametrize('layers', ['auto', 'all', 0, 99])
@pytest.mark.parametrize('margin', [None, 768])
def test_offload_command_retains_explicit_context(layers, margin):
    config = dataclasses.replace(descriptor(), gpu_layers=layers, gpu_memory_margin_mib=margin)
    runtime = LlamaCppRuntime(config, binary=__file__, verify_artifact=False)
    with patch('python.dcm.llama_runtime.subprocess.Popen', return_value=Mock()) as popen, \
            patch.object(runtime, '_start_stderr_drain'), patch.object(runtime, '_await_ready'), \
            patch.object(runtime, '_warm'):
        runtime.start()
    args = popen.call_args.args[0]
    assert args[args.index('-ngl')+1] == str(layers)
    assert args[args.index('-c')+1] == '8192'
    assert args[args.index('--parallel')+1] == '1'
    assert args[args.index('--fit')+1] == 'on'
    if margin is None:
        assert '--fit-target' not in args
    else:
        assert args[args.index('--fit-target')+1] == str(margin)
    assert runtime.descriptor.as_record()['gpu_layers'] == layers


@pytest.mark.parametrize('override', [0, 99, 'all'])
def test_explicit_constructor_override_remains_authoritative_and_recorded(override):
    runtime = LlamaCppRuntime(descriptor(), gpu_layers=override)
    assert runtime.gpu_layers == override
    assert runtime.descriptor.as_record()['gpu_layers'] == override


def test_configured_prefill_batches_are_sent_and_recorded():
    config = dataclasses.replace(descriptor(), batch_size=256, micro_batch_size=32)
    runtime = LlamaCppRuntime(config, binary=__file__, verify_artifact=False)
    with patch('python.dcm.llama_runtime.subprocess.Popen', return_value=Mock()) as popen, \
            patch.object(runtime, '_start_stderr_drain'), patch.object(runtime, '_await_ready'), \
            patch.object(runtime, '_warm'):
        runtime.start()
    args = popen.call_args.args[0]
    assert args[args.index('--batch-size') + 1] == '256'
    assert args[args.index('--ubatch-size') + 1] == '32'
    assert runtime.descriptor.as_record()['micro_batch_size'] == 32


@pytest.mark.parametrize('field', ['batch_size', 'micro_batch_size'])
@pytest.mark.parametrize('invalid', [0, -1, True, 1.5, '32'])
def test_invalid_prefill_batches_rejected(field, invalid):
    with pytest.raises(ValueError, match=field):
        dataclasses.replace(descriptor(), **{field: invalid})


def test_micro_batch_cannot_exceed_logical_batch():
    with pytest.raises(ValueError, match='micro_batch_size'):
        dataclasses.replace(descriptor(), batch_size=32, micro_batch_size=64)


@pytest.mark.parametrize('invalid', [-1, True, 1.5, None, '99', 'gpu-name', [], {}])
def test_invalid_layer_config_fails_before_launch(invalid):
    with pytest.raises(ValueError, match='gpu_layers'):
        dataclasses.replace(descriptor(), gpu_layers=invalid)


@pytest.mark.parametrize('invalid', [0, -1, True, 1.5, '512'])
def test_invalid_memory_margin_fails_before_launch(invalid):
    with pytest.raises(ValueError, match='gpu_memory_margin'):
        dataclasses.replace(descriptor(), gpu_memory_margin_mib=invalid)


def test_manifest_fallback_artifact_override_and_legacy_defaults(tmp_path):
    manifest = {'llama_cpp_revision': 'test', 'artifacts': [{
        'role': 'primary', 'path': '/fake.gguf', 'quantization': 'q5', 'sha256': '0'*64}]}
    path = tmp_path / 'manifest.json'
    def load():
        path.write_text(json.dumps(manifest))
        return RuntimeDescriptor.from_manifest(path)
    assert load().gpu_layers == 'auto'
    assert load().gpu_memory_margin_mib is None
    manifest.update(gpu_layers=0, gpu_memory_margin_mib=512)
    assert (load().gpu_layers, load().gpu_memory_margin_mib) == (0, 512)
    manifest['artifacts'][0].update(gpu_layers='auto', gpu_memory_margin_mib=1024)
    assert (load().gpu_layers, load().gpu_memory_margin_mib) == ('auto', 1024)

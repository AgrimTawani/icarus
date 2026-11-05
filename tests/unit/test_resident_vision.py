"""One resident multimodal mission model answers camera questions; nothing unloads it."""
import dataclasses
import io
import json
from contextlib import contextmanager
from unittest.mock import Mock, patch

import pytest
from test_llama_runtime import descriptor

from python.dcm.llama_runtime import LlamaCppRuntime, ResidentVision
from python.perception import vlm


def started(config):
    runtime = LlamaCppRuntime(config, binary=__file__, verify_artifact=False)
    with patch('python.dcm.llama_runtime.subprocess.Popen', return_value=Mock()) as popen, \
            patch.object(runtime, '_start_stderr_drain'), patch.object(runtime, '_await_ready'), \
            patch.object(runtime, '_warm'):
        runtime.start()
    return runtime, popen.call_args.args[0]


def test_projector_is_loaded_only_when_declared():
    runtime, args = started(descriptor())
    assert '--mmproj' not in args and runtime.vision() is None
    runtime, args = started(dataclasses.replace(descriptor(), mmproj_path='/m/proj.gguf', mmproj_sha256='ab'))
    assert args[args.index('--mmproj') + 1] == '/m/proj.gguf'
    vision = runtime.vision()
    assert vision.artifact['grounding_format'] == 'json_norm1000_xyxy' and vision.artifact['resident']


def test_projector_needs_its_checksum():
    with pytest.raises(ValueError, match='together'):
        dataclasses.replace(descriptor(), mmproj_path='/m/proj.gguf').verify_artifact()


def test_warm_up_caches_the_session_catalogue():
    runtime = LlamaCppRuntime(descriptor(), binary=__file__, verify_artifact=False)
    runtime.process = Mock()
    sent = []

    def urlopen(request, timeout):
        sent.append(json.loads(request.data))
        return io.BytesIO(b'{}')

    with patch('python.dcm.llama_runtime.urllib.request.urlopen', side_effect=urlopen):
        runtime.warm([{'type': 'function', 'function': {'name': 'hold'}}])
    assert sent[0]['tools'][0]['function']['name'] == 'hold'
    assert sent[0]['messages'][0]['role'] == 'system' and sent[0]['max_tokens'] == 1


def test_resident_question_is_serialized_with_decisions_and_keeps_the_server():
    runtime = LlamaCppRuntime(dataclasses.replace(descriptor(), mmproj_path='/p', mmproj_sha256='a'),
                              binary=__file__, verify_artifact=False)
    runtime.process = Mock()
    vision = ResidentVision(runtime)
    vision._marker = '<__media__>'
    bodies = []

    def urlopen(request, timeout):
        assert runtime._inference_lock.locked()
        bodies.append(json.loads(request.data))
        return io.BytesIO(b'{"content": "{\\"bbox\\": null}"}')

    with patch('python.dcm.llama_runtime.urllib.request.urlopen', side_effect=urlopen):
        assert vision.complete('aW1n', 'Where?', '', 8, grammar='root ::= "x"') == '{"bbox": null}'
    assert not runtime._inference_lock.locked() and runtime.process is not None
    body = bodies[0]
    assert body['prompt']['multimodal_data'] == ['aW1n'] and body['grammar'] == 'root ::= "x"'
    assert '<__media__>Where?' in body['prompt']['prompt_string']
    assert body['prompt']['prompt_string'].endswith('</think>\n\n')  # no thinking for camera answers


class Server:
    artifact = {'family': 'qwen3.5', 'grounding_format': 'json_norm1000_xyxy', 'resident': True}

    def __init__(self, reply):
        self.reply, self.questions = reply, []

    def complete(self, image_b64, question, prefill, n_predict, *, grammar=None):
        self.questions.append(question)
        return self.reply


def frame(tmp_path, size=(1000, 500)):
    from PIL import Image
    path = tmp_path / 'frame.png'
    Image.new('RGB', size).save(path)
    return path


def test_normalized_boxes_become_source_pixels(tmp_path):
    server = Server('{"bbox": [100, 200, 500, 1000]}')
    result = vlm.ground_description(frame(tmp_path), 'building', None, server=server)
    assert 'normalized to 0-1000' in server.questions[0]
    assert result['present'] and result['pixel_box_xyxy'] == [100, 100, 500, 500]
    assert result['runtime']['server'] == 'resident'
    absent = vlm.ground_description(frame(tmp_path), 'building', None, server=Server('{"bbox": null}'))
    assert absent['present'] is False


def test_every_instance_boxes_are_rescaled(tmp_path):
    result = vlm.ground_all(frame(tmp_path), 'person', None,
                            server=Server('{"bboxes": [[0, 0, 500, 500], [500, 500, 1000, 1000]]}'))
    assert result['boxes'] == [[0, 0, 500, 250], [500, 250, 1000, 500]]


def test_inspection_answers_from_the_resident_model(tmp_path):
    server = Server('{"seen": "a grey building", "found": true}')
    result = vlm.inspect_image(frame(tmp_path), 'Is a building visible?', None, server=server,
                               structured=True, result_type='boolean')
    assert result['found'] is True and result['runtime']['server'] == 'resident'


def test_resident_vision_never_takes_the_unload_lease():
    import importlib.util
    from pathlib import Path
    spec = importlib.util.spec_from_file_location(
        'run_mission_for_vision', Path(__file__).resolve().parents[2] / 'scripts/autonomy/run_mission.py')
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    client = module.MissionClient.__new__(module.MissionClient)

    @contextmanager
    def unload():
        raise AssertionError('resident vision must not stop the mission model')
        yield 99

    client.visual_inference = unload
    client.resident_vision = Server('')
    with client._vision_lease() as (layers, server):
        assert server is client.resident_vision and layers == 0
    assert client._vision_limiter().interval_s == 5.0  # Server() declares no interval
    del client.resident_vision
    client.edge_vlm_limiter = None

    @contextmanager
    def leased():
        yield 99

    client.visual_inference = leased
    with client._vision_lease() as (layers, server):
        assert server is None and layers == 99


def test_projector_moves_to_cpu_only_when_it_displaces_model_layers():
    config = dataclasses.replace(descriptor(), mmproj_path='/m/proj.gguf', mmproj_sha256='ab')
    runtime = LlamaCppRuntime(config, binary=__file__, verify_artifact=False)
    reports = iter(['load_tensors: offloaded 29/34 layers to GPU', 'load_tensors: offloaded 34/34 layers to GPU'])
    commands = []

    def popen(command, **kwargs):
        commands.append(command)
        runtime._stderr_tail = bytearray(next(reports).encode())
        return Mock(poll=Mock(return_value=None))

    with patch('python.dcm.llama_runtime.subprocess.Popen', side_effect=popen), \
            patch.object(runtime, '_start_stderr_drain'), patch.object(runtime, '_await_ready'), \
            patch.object(runtime, '_warm'), patch.object(runtime, 'stop', side_effect=lambda: setattr(runtime, 'process', None)):
        runtime.start()
    assert '--no-mmproj-offload' not in commands[0] and '--no-mmproj-offload' in commands[1]
    assert commands[0][commands[0].index('--image-max-tokens') + 1] == '1024'
    assert commands[1][commands[1].index('--image-max-tokens') + 1] == '256'  # cheaper CPU encoding
    assert runtime.projector_on_gpu is False and len(commands) == 2


def test_projector_stays_on_gpu_when_everything_fits():
    config = dataclasses.replace(descriptor(), mmproj_path='/m/proj.gguf', mmproj_sha256='ab')
    runtime = LlamaCppRuntime(config, binary=__file__, verify_artifact=False)

    def popen(command, **kwargs):
        runtime._stderr_tail = bytearray(b'load_tensors: offloaded 34/34 layers to GPU')
        return Mock()

    with patch('python.dcm.llama_runtime.subprocess.Popen', side_effect=popen) as started, \
            patch.object(runtime, '_start_stderr_drain'), patch.object(runtime, '_await_ready'), \
            patch.object(runtime, '_warm'):
        runtime.start()
    assert started.call_count == 1 and runtime.projector_on_gpu is True


def test_cpu_projector_server_yields_cpu_to_the_flight_stack():
    from python.dcm import llama_runtime
    config = dataclasses.replace(descriptor(), mmproj_path='/m/proj.gguf', mmproj_sha256='ab')
    runtime = LlamaCppRuntime(config, binary=__file__, verify_artifact=False)
    runtime.projector_on_gpu = False
    with patch('python.dcm.llama_runtime.subprocess.Popen', return_value=Mock()) as popen, \
            patch.object(runtime, '_start_stderr_drain'), patch.object(runtime, '_await_ready'), \
            patch.object(runtime, '_warm'):
        runtime.start()
    assert popen.call_args.kwargs['preexec_fn'] is llama_runtime._yield_cpu_to_flight_stack
    runtime, _ = started(config)  # projector on the GPU: normal priority
    with patch('python.dcm.llama_runtime.subprocess.Popen', return_value=Mock()) as popen, \
            patch.object(runtime, '_start_stderr_drain'), patch.object(runtime, '_await_ready'), \
            patch.object(runtime, '_warm'):
        runtime.process = None
        runtime.projector_on_gpu = None
        runtime.start()
    assert popen.call_args.kwargs['preexec_fn'] is None

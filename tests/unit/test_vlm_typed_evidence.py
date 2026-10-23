"""Typed measurements and inference failures, without launching model processes."""
import importlib.util
import json
import subprocess
import threading
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import Mock, patch

import pytest

from python.perception.vlm import (
    VlmInferenceError,
    _GroundingServer,
    decode_measurement,
    inspect_image,
)


@pytest.mark.parametrize('count', [0, 1, 7, 103, None])
def test_counts_preserve_zero_and_unknown(count):
    result = decode_measurement(json.dumps({'count': count, 'answer': 'visible instances'}), 'count')
    assert result['count'] == count
    assert result['value'] == count
    assert result['validity'] == ('unknown' if count is None else 'valid')
    assert result['count_scope'] == 'visible_frame'


@pytest.mark.parametrize('payload', [
    {'answer': 'none'}, {'count': True, 'answer': 'one'},
    {'count': -1, 'answer': 'none'}, {'count': 2.5, 'answer': 'some'},
    {'count': '3', 'answer': 'three'}, {'count': 3, 'answer': 'ok', 'found': True},
    {'found': True, 'answer': 'yes'}, '3', None,
])
def test_bad_measurements_never_become_counts(payload):
    result = decode_measurement(json.dumps(payload), 'count')
    assert result['count'] is None
    assert result['validity'] == 'unknown'
    assert result['error_code'] == 'malformed_output'


def run_inspection(tmp_path, completed, **kwargs):
    image = tmp_path / 'frame.ppm'
    binary = tmp_path / 'llama-cli'
    image.write_bytes(b'pixels')
    binary.touch()
    with patch('python.perception.vlm.load_vlm', return_value={
            'path': 'model', 'mmproj_path': 'projector'}), \
            patch('python.perception.vlm.subprocess.run', return_value=completed) as run:
        result = inspect_image(image, 'Count visible objects', tmp_path / 'manifest',
                               binary=binary, **kwargs)
    return result, run.call_args.args[0]


def test_zero_exit_sampler_failure_is_an_error(tmp_path):
    with pytest.raises(VlmInferenceError) as error:
        run_inspection(tmp_path, SimpleNamespace(returncode=0,
            stdout='Error: Failed to initialize samplers: std::exception\n', stderr=''))
    assert error.value.code == 'sampler_initialization'


def test_stderr_is_not_a_visual_answer(tmp_path):
    with pytest.raises(VlmInferenceError) as error:
        run_inspection(tmp_path, SimpleNamespace(returncode=0, stdout='', stderr='runtime initialized'))
    assert error.value.code == 'empty_output'


def test_count_schema_reaches_cli_and_returns_typed_evidence(tmp_path):
    result, command = run_inspection(tmp_path, SimpleNamespace(returncode=0,
        stdout='{"count":7,"answer":"Seven visible objects"}', stderr=''), result_type='count')
    grammar = command[command.index('--grammar') + 1]
    assert 'count-kv' in grammar
    assert '--json-schema' not in command
    assert result['count'] == 7
    assert result['validity'] == 'valid'
    assert result['flight_authority'] is False


def test_grounding_startup_failure_cleans_its_own_process():
    server = _GroundingServer(Path('/unused'), {'path': 'm', 'mmproj_path': 'p'}, 0, 100)
    process = Mock()
    process.poll.return_value = None
    with patch('python.perception.vlm.subprocess.Popen', return_value=process), \
            patch.object(server, '_remaining', side_effect=TimeoutError('deadline')), \
            patch.object(server, '__exit__') as cleanup, pytest.raises(TimeoutError):
        server.__enter__()
    cleanup.assert_called_once()


def client_module():
    path = Path(__file__).resolve().parents[2] / 'scripts/autonomy/run_mission.py'
    spec = importlib.util.spec_from_file_location('typed_evidence_mission', path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def test_failed_observation_retains_capture_provenance(tmp_path):
    module = client_module()
    client = module.MissionClient.__new__(module.MissionClient)
    client.episode = SimpleNamespace(record=Mock())
    client.capture_view = Mock(return_value={'directory': str(tmp_path / 'view'),
        'captured_at_unix_ms': 123, 'camera': 'forward_d435i', 'capture_pose': {'pose': 1}})
    client.inspect_scene = Mock(side_effect=VlmInferenceError('sampler_initialization', 'failure'))
    result = client.observe_mission({'tool': 'vlm', 'query': 'count objects',
                                     'camera': 'forward_d435i', 'result_type': 'count'})
    assert result['count'] is None and result['validity'] == 'error'
    assert result['error_code'] == 'sampler_initialization'
    assert result['captured_at_unix_ms'] == 123
    assert result['capture_pose'] == {'pose': 1}


def test_stationary_observation_settles_without_issuing_a_hold(tmp_path):
    module = client_module()
    client = module.MissionClient.__new__(module.MissionClient)
    client.episode = SimpleNamespace(record=Mock())
    client.ensure_action_settled = Mock()
    client.capture_view = Mock(return_value={
        'directory': str(tmp_path / 'view'), 'captured_at_unix_ms': 123,
        'camera': 'forward_d435i', 'capture_pose': {'pose': 1},
        'capture_pose_quality': {'position_uncertainty_m': 0.01}})
    client.inspect_scene = Mock(return_value={
        'validity': 'valid', 'answer': '{"found":true,"answer":"yes"}',
        'result_type': 'boolean', 'found': True, 'value': True,
        'capture_pose_quality': {'position_uncertainty_m': 0.01}})
    result = client.observe_mission({
        'tool': 'vlm', 'query': 'whether a door is open',
        'camera': 'forward_d435i', 'result_type': 'boolean',
        'capture_motion': 'stationary'})
    client.ensure_action_settled.assert_called_once_with()
    assert result['capture_motion'] == 'stationary'
    assert result['capture_constraint']['issued_flight_command'] is False
    assert result['capture_constraint']['pose_bracket'] == {
        'position_uncertainty_m': 0.01}


def test_session_inference_lock_rejects_overlap_and_releases_after_failure():
    module = client_module()
    entered, release = threading.Event(), threading.Event()
    class Client:
        @module._exclusive_inference
        def inspect(self, wait=False):
            if wait:
                entered.set()
                assert release.wait(2)
            return 'complete'
    client = Client()
    worker = threading.Thread(target=client.inspect, args=(True,))
    worker.start()
    try:
        assert entered.wait(2)
        with pytest.raises(RuntimeError, match='previous visual inference'):
            client.inspect()
    finally:
        release.set()
        worker.join(2)
    assert client.inspect() == 'complete'


@pytest.mark.parametrize('observation', [
    {'tool': 'vlm', 'result_type': 'number'},
    {'tool': 'vlm', 'result_type': 'count', 'count_scope': 'unique_survey'},
    {'tool': 'landing', 'result_type': 'count'},
])
def test_unsupported_measurement_contract_rejected_before_capture(observation):
    module = client_module()
    client = module.MissionClient.__new__(module.MissionClient)
    client.capture_view = Mock(side_effect=AssertionError('must not capture'))
    with pytest.raises(ValueError):
        client.observe_mission(dict(observation, camera='forward_d435i', query='objects'))
    client.capture_view.assert_not_called()


def test_timeout_has_distinct_error_code(tmp_path):
    with patch('python.perception.vlm.subprocess.run', side_effect=subprocess.TimeoutExpired('cli', 1)):
        # run_inspection deliberately owns its mock; exercise the public call here.
        image, binary = tmp_path / 'image', tmp_path / 'cli'
        image.write_bytes(b'pixels')
        binary.touch()
        with patch('python.perception.vlm.load_vlm', return_value={'path': 'm', 'mmproj_path': 'p'}), \
                pytest.raises(VlmInferenceError) as error:
            inspect_image(image, 'Are objects visible?', tmp_path / 'manifest', binary=binary)
    assert error.value.code == 'timeout'


def test_detector_missing_count_is_unknown_not_zero():
    module = client_module()
    client = module.MissionClient.__new__(module.MissionClient)
    client.episode = SimpleNamespace(record=Mock())
    client.state = Mock()
    client.detect = Mock(return_value={'counts': {}})
    with patch.object(module, 'MessageToDict', return_value={}):
        result = client.observe_mission({'tool': 'yolo', 'camera': 'forward_d435i',
                                         'query': 'person', 'result_type': 'count'})
    assert result['count'] is None
    assert result['found'] is None
    assert result['validity'] == 'unknown'


def test_viewing_pose_stays_in_receipt_and_cannot_change_the_pixel_question(tmp_path):
    outputs = SimpleNamespace(returncode=0, stdout='{"answerable":true,"answer":"Visible structure."}',stderr='')
    commands = []
    for heading in (0, 123.456, 359.9):
        context = {'camera':'forward_d435i','aircraft_heading_deg':heading}
        result, command = run_inspection(tmp_path, outputs, result_type='description',view_context=context)
        assert result['view_context'] == context
        commands.append(command[command.index('-p')+1])
    assert commands[0] == commands[1] == commands[2]
    assert 'Measured aircraft compass heading' not in commands[0]
    with pytest.raises(ValueError, match='finite'):
        run_inspection(tmp_path, outputs, result_type='description',
                       view_context={'camera':'forward_d435i','aircraft_heading_deg':float('nan')})


@pytest.mark.parametrize('kind,payload',[('boolean',{'seen':'Visible object is grey.','found':False}),
                                       ('count',{'count':0,'answer':'No visible instances.'})])
def test_structured_answer_is_plain_evidence_not_a_second_encoded_json_document(tmp_path,kind,payload):
    result, _ = run_inspection(tmp_path, SimpleNamespace(returncode=0,
        stdout=json.dumps(payload),stderr=''), result_type=kind)
    assert result['answer'] == payload.get('seen', payload.get('answer'))
    assert result['value'] == payload.get('found',payload.get('count'))

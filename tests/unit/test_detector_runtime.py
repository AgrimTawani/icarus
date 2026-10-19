"""Pinned detector reuse and approved-plan preparation without GPU imports."""
import hashlib
import json
import threading
import time
from types import SimpleNamespace
from unittest.mock import Mock, patch

import pytest

from python.perception.edge_vision import EdgeDetectorRuntime


def manifest(tmp_path):
    weights = tmp_path / 'weights.pt'; weights.write_bytes(b'pinned detector')
    artifact = {'path': str(weights), 'sha256': hashlib.sha256(weights.read_bytes()).hexdigest(),
                'repository': 'pinned', 'revision': 'test', 'runtime': 'test'}
    path = tmp_path / 'manifest.json'
    path.write_text(json.dumps({'profile': 'edge-vision', 'simulation_only': True,
        'llama_cpp_revision': 'test', 'artifacts': [artifact | {'role': role}
        for role in ['yolo', 'vlm', 'qwen_edge_primary']]}))
    return path, weights


def test_load_verify_warm_once_then_reuse_and_close(tmp_path):
    path, weights = manifest(tmp_path)
    model = Mock(); model.predict.return_value = ['actual-result']
    yolo = Mock(return_value=model)
    with patch.dict('sys.modules', {'torch': SimpleNamespace(cuda=SimpleNamespace(is_available=lambda: False)),
                                   'ultralytics': SimpleNamespace(YOLO=yolo)}):
        runtime = EdgeDetectorRuntime(path)
        prepared = runtime.prepare()
        assert prepared['observation'] is False
        assert model.predict.call_count == 1
        # A loaded immutable runtime does not reopen/reload weights per frame.
        weights.unlink()
        runtime.prepare()
        assert runtime.predict('actual-camera-one.png', .35) == 'actual-result'
        assert runtime.predict('actual-camera-two.png', .35) == 'actual-result'
        assert yolo.call_count == 1
        assert model.predict.call_count == 3
        runtime.close()
        assert runtime.model is None
        with pytest.raises(RuntimeError, match='closed'):
            runtime.predict('frame', .35)


def test_predictions_are_serialized(tmp_path):
    path, _ = manifest(tmp_path)
    active = maximum = 0
    def predict(**kwargs):
        nonlocal active, maximum
        active += 1; maximum = max(maximum, active)
        time.sleep(.015)
        active -= 1
        return ['result']
    with patch.dict('sys.modules', {'torch': SimpleNamespace(cuda=SimpleNamespace(is_available=lambda: False)),
                                   'ultralytics': SimpleNamespace(YOLO=lambda _: SimpleNamespace(predict=predict))}):
        runtime = EdgeDetectorRuntime(path); runtime.prepare(warm=False)
        workers = [threading.Thread(target=runtime.predict, args=('frame', .35)) for _ in range(4)]
        for worker in workers: worker.start()
        for worker in workers: worker.join()
        assert maximum == 1
        runtime.close()






def test_actual_camera_captures_keep_separate_trackers_and_shared_model(tmp_path):
    from icarus.v1 import state_pb2
    from test_localize_target import _module
    Client = _module().MissionClient
    client = Client.__new__(Client)
    client.vision_profile = 'edge-vision'; client.simulation_session = {'simulator': 'gazebo'}
    client.episode = SimpleNamespace(session=True, record=Mock())
    client.edge_trackers = {}; client.edge_detection_limiter = Mock()
    client.edge_detector_runtime = Mock()
    client.state = lambda: state_pb2.DroneState()
    captures, trackers, models = [], [], []
    def capture(path, camera):
        captures.append(camera)
        path.write_bytes(b'P6\n1 1\n255\n\xff\xff\xff')
        return camera
    def detect(path, manifest, tracker, **kwargs):
        assert path.is_file()
        trackers.append(tracker); models.append(kwargs['runtime'])
        return {'schema': 'icarus.edge.yolo.v1', 'detections': [], 'classes': [],
                'unique_person_count': 7, 'confidence_threshold': 0.35,
                'observed_at_unix_ms': int(time.time()*1000)}
    client._capture_rgb = capture
    with patch('python.perception.edge_vision.detect_edge_image', side_effect=detect):
        first = client.detect(['person'], camera='forward_d435i')
        second = client.detect(['person'], camera='downward_brio')
        client.detect(['person'], camera='forward_d435i')
    assert captures == ['forward_d435i', 'downward_brio', 'forward_d435i']
    assert trackers[0] is trackers[2] and trackers[0] is not trackers[1]
    assert all(model is client.edge_detector_runtime for model in models)
    assert first['camera'] == 'forward_d435i' and second['camera'] == 'downward_brio'
    assert first['captured_at_unix_ms'] > 0
    from python.dcm.fly import compact_result
    assert first['confidence_threshold'] == 0.35
    assert compact_result(first)['confidence_threshold'] == 0.35



    # Empty current view must not inherit seven historical person tracks.
    assert first['counts'] == {'person': 0}
    assert first['count_scope'] == 'current_camera_frame'
    assert 'unique_person_count' not in first
    assert client.episode.record.call_args.args[1]['unique_person_count'] == 7


@pytest.mark.parametrize('model_id', ['yolo11n', 'yolo11s'])
def test_normalized_result_reports_selected_model_not_hardcoded_nano(tmp_path, model_id):
    from python.perception.edge_vision import detect_edge_image, UniqueTrackCounter
    frame = tmp_path / 'input.ppm'
    frame.write_bytes(b'P6\n1 1\n255\n\xff\xff\xff')
    values = SimpleNamespace(tolist=lambda: [])
    output = SimpleNamespace(names={}, boxes=SimpleNamespace(cls=values, conf=values, xyxy=values), orig_shape=(1,1))
    runtime = SimpleNamespace(prepare=lambda **kw: None, selected='cpu',
        artifact={'path': str(tmp_path / (model_id + '.pt')), 'repository': 'ultralytics/yolo11', 'sha256': 'pinned'},
        predict=lambda *args: output)
    result = detect_edge_image(frame, tmp_path / 'unused.json', UniqueTrackCounter(), runtime=runtime)
    assert result['model'] == model_id


def test_candidate_edge_manifest_override_does_not_change_installed_default(monkeypatch, tmp_path):
    from test_localize_target import _module
    Client = _module().MissionClient
    client = Client.__new__(Client)
    monkeypatch.delenv('EDGE_VISION_MANIFEST', raising=False)
    from pathlib import Path
    assert client._edge_manifest() == Path.home() / 'models/edge/MANIFEST.json'
    candidate = tmp_path / 'candidate.json'
    monkeypatch.setenv('EDGE_VISION_MANIFEST', str(candidate))
    assert client._edge_manifest() == candidate
    with patch('python.perception.edge_vision.EdgeDetectorRuntime') as runtime:
        client._edge_detector()
        runtime.assert_called_once_with(candidate)

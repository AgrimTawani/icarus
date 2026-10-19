"""Exercise installer selection without downloading or loading model weights."""
import hashlib
import importlib.util
import json
import os
from pathlib import Path
import subprocess

import pytest

ROOT = Path(__file__).resolve().parents[2]
pytestmark = pytest.mark.skipif(
    not (ROOT / '.venv/bin/python').exists() or importlib.util.find_spec('ultralytics') is None,
    reason='edge installer needs the provisioned ML environment')


def artifact_file(tmp_path, name):
    path = tmp_path / name
    path.write_bytes(name.encode())
    return str(path), hashlib.sha256(path.read_bytes()).hexdigest()


def provision_inputs(tmp_path, family):
    edge = tmp_path / 'edge'
    actor = edge / 'gazebo_fuel_cache/fuel.gazebosim.org/mingfei/models/actor/1'
    (actor / 'meshes').mkdir(parents=True)
    for name in ('meshes/walk.dae', 'meshes/stand.dae', 'model.sdf'):
        (actor / name).write_text('fixture asset')
    (edge / 'yolo').mkdir()
    (edge / 'yolo/yolo11s.pt').write_bytes(b'fixture detector; never loaded')
    model, sha = artifact_file(tmp_path, 'flight.gguf')
    flight = tmp_path / 'flight.json'
    flight.write_text(json.dumps({'llama_cpp_revision': 'runtime-commit', 'artifacts': [{
        'role': 'primary', 'family': 'qwen3.5', 'repository': 'publisher/flight',
        'revision': 'a' * 40, 'runtime': 'llama_cpp', 'path': model, 'sha256': sha,
        'quantization': 'q4_k_m', 'context_length': 16384, 'deadline_ms': 60000}]}))
    model, sha = artifact_file(tmp_path, 'vision.gguf')
    projector, projector_sha = artifact_file(tmp_path, 'projector.gguf')
    selected = {'role': 'vlm', 'family': family, 'repository': 'publisher/vision',
        'revision': 'b' * 40, 'runtime': 'llama_cpp', 'path': model, 'sha256': sha,
        'mmproj_path': projector, 'mmproj_sha256': projector_sha,
        'quantization': 'q4_k_m', 'grounding_format': 'json_pixel_xyxy',
        'grounding_image': {'alignment': 28, 'minimum_tokens': 1024, 'maximum_tokens': 1536}}
    vision = tmp_path / 'vision.json'
    vision.write_text(json.dumps({'artifacts': [selected]}))
    env = dict(os.environ, EDGE_MODEL_DIR=str(edge), VISION_MANIFEST=str(vision),
        FLIGHT_MANIFEST=str(flight), FLIGHT_ROLE='primary', EDGE_YOLO_MODEL='yolo11s')
    return edge / 'MANIFEST.json', vision, selected, env


def install(env):
    return subprocess.run(['bash', str(ROOT / 'scripts/setup-edge-vision-runtime')],
        cwd=ROOT, env=env, capture_output=True, text=True, timeout=45)


@pytest.mark.parametrize('family', ['qwen2-vl', 'qwen2.5-vl'])
def test_installer_preserves_explicit_model_and_coordinate_policy(tmp_path, family):
    output, _, selected, env = provision_inputs(tmp_path, family)
    for _ in range(2):
        result = install(env)
        assert result.returncode == 0, result.stderr
        installed = json.loads(output.read_text())
        vlm = next(a for a in installed['artifacts'] if a['role'] == 'vlm')
        assert all(vlm[k] == value for k, value in selected.items())
        flight = next(a for a in installed['artifacts'] if a['role'] == 'qwen_edge_primary')
        assert flight['family'] == 'qwen3.5'
        assert flight['context_length'] == 16384


@pytest.mark.parametrize('fault', ['checksum', 'duplicate'])
def test_invalid_existing_selection_is_reported_without_replacing_it(tmp_path, fault):
    output, vision, selected, env = provision_inputs(tmp_path, 'qwen2.5-vl')
    if fault == 'checksum':
        Path(selected['mmproj_path']).write_bytes(b'corrupted')
    else:
        vision.write_text(json.dumps({'artifacts': [selected, selected]}))
    before = vision.read_bytes()
    output.write_text('existing installed selection')
    result = install(env)
    assert result.returncode != 0
    assert ('checksum mismatch' if fault == 'checksum' else 'exactly one') in result.stderr
    assert vision.read_bytes() == before
    assert output.read_text() == 'existing installed selection'

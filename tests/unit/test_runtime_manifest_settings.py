import copy
import hashlib
import json
from types import SimpleNamespace
from unittest.mock import patch

import pytest

from python.dcm.contract import RuntimeDescriptor
from python.dcm.edge_provisioning import flight_artifact
from python.dcm.fly_cli import build_runtime


def manifest(tmp_path):
    model=tmp_path/'candidate.gguf';model.write_bytes(b'unit-test-weights')
    return {'llama_cpp_revision':'runtime-commit','artifacts':[{
        'role':'chosen','family':'any-family','repository':'publisher/model',
        'revision':'publisher-commit','quantization':'iq3_xxs',
        'path':str(model),'sha256':hashlib.sha256(model.read_bytes()).hexdigest()}]}


def save(tmp_path,data):
    path=tmp_path/'manifest.json';path.write_text(json.dumps(data));return path


def test_descriptor_inheritance_and_explicit_override(tmp_path):
    data=manifest(tmp_path);data.update(context_length=4096,deadline_ms=45000)
    path=save(tmp_path,data)
    descriptor=RuntimeDescriptor.from_manifest(path,role='chosen')
    assert (descriptor.context_length,descriptor.deadline_ms)==(4096,45000)
    data['artifacts'][0].update(context_length=8192,deadline_ms=60000)
    path=save(tmp_path,data)
    descriptor=RuntimeDescriptor.from_manifest(path,role='chosen')
    assert (descriptor.context_length,descriptor.deadline_ms)==(8192,60000)
    descriptor=RuntimeDescriptor.from_manifest(path,role='chosen',context_length=2048,deadline_ms=12000)
    assert (descriptor.context_length,descriptor.deadline_ms)==(2048,12000)


@pytest.mark.parametrize('field',['context_length','deadline_ms'])
@pytest.mark.parametrize('bad',[0,-1,True,1.5,'8192',None])
def test_invalid_manifest_settings_fail_without_loading_model(tmp_path,field,bad):
    data=manifest(tmp_path);data[field]=bad
    with pytest.raises(ValueError,match=field):RuntimeDescriptor.from_manifest(save(tmp_path,data),role='chosen')


def test_legacy_deadline_and_configurable_context_fallback(tmp_path):
    path=save(tmp_path,manifest(tmp_path))
    descriptor=RuntimeDescriptor.from_manifest(path,role='chosen',default_context_length=4096)
    assert descriptor.deadline_ms==15000
    assert descriptor.context_length==4096


def test_provisioning_preserves_artifact_identity_and_controls(tmp_path):
    data=manifest(tmp_path);data.update(reasoning_mode='off',gpu_layers='auto',gpu_memory_margin_mib=768,context_length=8192,deadline_ms=90000)
    data['artifacts'][0]['gpu_layers']=0
    before=copy.deepcopy(data)
    result=flight_artifact(data,'chosen')
    assert result['revision']=='publisher-commit' != data['llama_cpp_revision']
    assert result['quantization']=='iq3_xxs'
    assert result['family']=='any-family'
    assert result['repository']=='publisher/model'
    assert result['reasoning_mode']=='off'
    assert result['gpu_layers']==0
    assert result['gpu_memory_margin_mib']==768
    assert result['deadline_ms']==90000
    assert result['context_length']==8192
    assert result['role']=='qwen_edge_primary'
    assert data==before


def test_provisioning_never_uses_runtime_commit_as_publisher_revision(tmp_path):
    data=manifest(tmp_path);data['artifacts'][0].pop('revision')
    with pytest.raises(ValueError,match='revision'):flight_artifact(data,'chosen')


def test_provisioning_rejects_wrong_checksum_and_ambiguous_role(tmp_path):
    data=manifest(tmp_path);data['artifacts'][0]['sha256']='0'*64
    with pytest.raises(ValueError,match='checksum'):flight_artifact(data,'chosen')
    data['artifacts'].append(copy.deepcopy(data['artifacts'][0]))
    with pytest.raises(ValueError,match='exactly one'):flight_artifact(data,'chosen')


@pytest.mark.parametrize('timeout,explicit_context,expected_deadline,expected_context',[
    (None,False,60000,8192),(12000,False,12000,8192),
    (None,True,60000,4096),
])
def test_console_explicit_override_and_edge_v3_minimum(tmp_path,timeout,explicit_context,expected_deadline,expected_context):
    data=manifest(tmp_path);data.update(context_length=4096,deadline_ms=60000)
    data["artifacts"][0]["role"]="primary"
    args=SimpleNamespace(runtime='llama',manifest=save(tmp_path,data),role='chosen',
        timeout_ms=timeout,context_length=4096 if explicit_context else 8192,context_length_explicit=explicit_context,
        profile='edge-vision',context='v3',ending_guidance=False)
    with patch('python.dcm.llama_runtime.LlamaCppRuntime'):
        _,descriptor=build_runtime(args)
    assert descriptor.deadline_ms==expected_deadline
    assert descriptor.context_length==expected_context


def test_setup_model_manifest_producer_is_consumable_without_losing_identity(tmp_path):
    """Exercise the actual installer JSON template, not a test-only manifest copy."""
    import re
    from pathlib import Path
    source=Path('scripts/setup-model-runtime').read_text()
    template=source.split('cat >"${MANIFEST}.pending" <<JSON\n',1)[1].split('\nJSON',1)[0]
    model=tmp_path/'primary.gguf';model.write_bytes(b'producer weights')
    sha=hashlib.sha256(model.read_bytes()).hexdigest()
    values={'HF_REPO':'publisher/any-model','MODEL_REV':'a'*40,'LLAMA_REV':'b'*40,
            'LLAMA_DIR':'/runtime','CUDA_ARCH':'89','WEIGHTS_DIR':str(tmp_path),
            'PRIMARY':'primary.gguf','SECONDARY':'other.gguf','SHA_PRIMARY':sha,'SHA_SECONDARY':'0'*64,
            'MODEL_FAMILY':'qwen','PRIMARY_QUANT':'iq3_xxs','SECONDARY_QUANT':'iq3_xxs',
            'CONTEXT_LENGTH':'8192','DEADLINE_MS':'90000','REASONING_MODE':'on',
            'REASONING_BUDGET':'1024','GPU_LAYERS':'auto','GPU_MEMORY_MARGIN_MIB':'512'}
    rendered=re.sub(r'\$\{([^}]+)\}',lambda m:values[m.group(1)],template).replace('$(date -Is)','test-date')
    artifact=flight_artifact(json.loads(rendered),'primary')
    assert artifact['revision']=='a'*40
    assert artifact['repository']=='publisher/any-model'
    assert artifact['family']=='qwen'
    assert artifact['reasoning_budget']==1024
    assert artifact['reasoning_mode']=='on'
    assert artifact['deadline_ms']==90000
    assert artifact['quantization']=='iq3_xxs'
    assert '--revision "${MODEL_REV}"' in source


def test_legacy_metadata_upgrade_checks_publisher_hash_without_replacing_weights(tmp_path):
    from python.dcm.model_provenance import upgrade_legacy_source
    data=manifest(tmp_path);artifact=data['artifacts'][0]
    for key in ('family','revision'):artifact.pop(key)
    metadata={'sha':'c'*40,'siblings':[{'rfilename':'candidate.gguf','lfs':{'sha256':artifact['sha256']}}]}
    upgraded=upgrade_legacy_source(data,'chosen',metadata)
    result=flight_artifact(upgraded,'chosen')
    assert result['revision']=='c'*40
    assert result['family']=='unknown'
    assert 'revision' not in data['artifacts'][0]
    metadata['siblings'][0]['lfs']['sha256']='0'*64
    with pytest.raises(ValueError,match='does not match publisher'):
        upgrade_legacy_source(data,'chosen',metadata)


def test_cached_publisher_metadata_supports_offline_verified_upgrade(tmp_path):
    from python.dcm.model_provenance import publisher_metadata
    cache=tmp_path/'publisher.json'
    metadata={'sha':'d'*40,'siblings':[]}
    cache.write_text(json.dumps({'repository':'publisher/model','metadata':metadata}))
    with patch('python.dcm.model_provenance.urlopen',side_effect=OSError('offline')):
        assert publisher_metadata('publisher/model',cache)==metadata
        with pytest.raises(ValueError,match='repository differs'):
            publisher_metadata('different/repo',cache)


def test_explicit_publisher_revision_selects_immutable_endpoint(tmp_path):
    from unittest.mock import Mock, patch

    from python.dcm.model_provenance import publisher_metadata
    metadata={'sha':'a'*40,'siblings':[]}
    response=Mock();response.read.return_value=json.dumps(metadata).encode()
    response.__enter__=Mock(return_value=response);response.__exit__=Mock(return_value=False)
    with patch('python.dcm.model_provenance.urlopen',return_value=response) as call:
        assert publisher_metadata('publisher/model',tmp_path/'cache.json','a'*40)==metadata
    assert '/revision/'+'a'*40+'?blobs=true' in call.call_args.args[0]


def test_explicit_revision_rejects_wrong_cached_commit(tmp_path):
    from unittest.mock import patch

    from python.dcm.model_provenance import publisher_metadata
    cache=tmp_path/'cache.json'
    cache.write_text(json.dumps({'repository':'publisher/model','metadata':{'sha':'a'*40}}))
    with patch('python.dcm.model_provenance.urlopen',side_effect=OSError('offline')), pytest.raises(ValueError,match='revision differs'):
        publisher_metadata('publisher/model',cache,'b'*40)
    with pytest.raises(ValueError,match='immutable commit'):
        publisher_metadata('publisher/model',cache,'main')


def test_sampling_inheritance_overrides_and_provenance(tmp_path):
    settings = dict(temperature=.6, top_p=.95, top_k=20, min_p=0,
                    presence_penalty=0, repetition_penalty=1, seed=42)
    data = manifest(tmp_path)
    data.update(settings)
    descriptor = RuntimeDescriptor.from_manifest(save(tmp_path, data), role='chosen')
    assert {key: descriptor.as_record()[key] for key in settings} == settings
    data['artifacts'][0].update(temperature=.8, top_k=10, min_p=None)
    descriptor = RuntimeDescriptor.from_manifest(save(tmp_path, data), role='chosen', top_p=.9)
    assert (descriptor.temperature, descriptor.top_k, descriptor.min_p, descriptor.top_p) == (.8, 10, None, .9)
    assert descriptor.presence_penalty == 0


def test_legacy_sampling_keeps_greedy_and_leaves_other_server_defaults_unchanged(tmp_path):
    descriptor = RuntimeDescriptor.from_manifest(save(tmp_path, manifest(tmp_path)), role='chosen')
    assert descriptor.temperature == 0
    for field in ('top_p', 'top_k', 'min_p', 'presence_penalty', 'repetition_penalty', 'seed'):
        assert getattr(descriptor, field) is None


@pytest.mark.parametrize('field,bad', [
    ('temperature', -1), ('temperature', None), ('temperature', True),
    ('temperature', float('inf')), ('top_p', -0.1), ('top_p', 1.1),
    ('top_p', '0.95'), ('min_p', float('nan')), ('min_p', 1.1),
    ('top_k', -1), ('top_k', 2**31), ('top_k', 1.5), ('top_k', True),
    ('presence_penalty', float('-inf')), ('presence_penalty', '0'),
    ('seed', -1), ('seed', 2**32), ('seed', True), ('seed', 1.5),
    ('repetition_penalty', 0), ('repetition_penalty', -1),
])
def test_invalid_sampling_fails_before_model_loading(tmp_path, field, bad):
    data = manifest(tmp_path)
    data[field] = bad
    with pytest.raises(ValueError, match=field):
        RuntimeDescriptor.from_manifest(save(tmp_path, data), role='chosen')


def test_provisioning_preserves_and_validates_sampling_controls(tmp_path):
    data = manifest(tmp_path)
    settings = dict(temperature=.6, top_p=.95, top_k=20, min_p=0,
                    presence_penalty=0, repetition_penalty=1, seed=42)
    data.update(settings)
    data['artifacts'][0]['top_k'] = 10
    before = copy.deepcopy(data)
    artifact = flight_artifact(data, 'chosen')
    assert {key: artifact[key] for key in settings} == {**settings, 'top_k': 10}
    assert data == before
    data['artifacts'][0]['repetition_penalty'] = -1
    with pytest.raises(ValueError, match='repetition_penalty'):
        flight_artifact(data, 'chosen')


@pytest.mark.parametrize('seed', [0, 2**32 - 1, None])
def test_seed_uint32_boundaries_and_artifact_override(tmp_path, seed):
    data = manifest(tmp_path)
    data['seed'] = 123
    data['artifacts'][0]['seed'] = seed
    result = RuntimeDescriptor.from_manifest(save(tmp_path, data), role='chosen')
    assert result.seed == seed
    assert result.as_record()['seed'] == seed
    assert flight_artifact(data, 'chosen')['seed'] == seed
    overridden = RuntimeDescriptor.from_manifest(save(tmp_path, data), role='chosen', seed=7)
    assert overridden.seed == 7

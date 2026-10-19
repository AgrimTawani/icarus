"""Pure artifact selection for installer; no downloads or installed-state writes."""
import copy
import hashlib
from pathlib import Path

from python.dcm.contract import RuntimeDescriptor

CONTROLS = ('context_length', 'deadline_ms', 'reasoning_mode', 'reasoning_budget', 'gpu_layers',
            'gpu_memory_margin_mib', 'temperature', 'top_p', 'top_k', 'min_p',
            'presence_penalty', 'repetition_penalty', 'seed', 'batch_size', 'micro_batch_size')


def flight_artifact(manifest, role=None, verify=True):
    """Preserve a chosen artifact and inherit generic manifest controls.

    Model publisher revision is separate from the llama.cpp implementation SHA.
    A legacy manifest lacking publisher provenance must be repaired explicitly.
    """
    artifacts = manifest.get('artifacts', [])
    if role is None:
        roles = {a.get('role') for a in artifacts}
        role = 'qwen_edge_primary' if 'qwen_edge_primary' in roles else 'primary'
    selected = [a for a in artifacts if a.get('role') == role]
    if len(selected) != 1:
        raise ValueError(f'flight role {role!r} must identify exactly one artifact')
    artifact = copy.deepcopy(selected[0])
    for field in ('family', 'repository', 'revision', 'runtime'):
        if field not in artifact and field in manifest:
            artifact[field] = manifest[field]
    artifact.setdefault('runtime', 'llama_cpp')
    for field in ('family', 'repository', 'revision', 'path', 'sha256', 'quantization'):
        if not isinstance(artifact.get(field), str) or not artifact[field]:
            raise ValueError(f'flight artifact missing {field}; provide explicit publisher metadata in FLIGHT_MANIFEST')
    for field in CONTROLS:
        if field not in artifact and field in manifest:
            artifact[field] = copy.deepcopy(manifest[field])
    # Validate exactly the same runtime controls before provisioning a manifest.
    RuntimeDescriptor(model_id=Path(artifact['path']).stem, family=artifact['family'],
        quantization=artifact['quantization'], artifact_path=artifact['path'],
        artifact_sha256=artifact['sha256'], runtime=artifact['runtime'],
        runtime_revision=manifest['llama_cpp_revision'], temperature=artifact.get('temperature', 0),
        context_length=artifact.get('context_length', 8192),
        deadline_ms=artifact.get('deadline_ms', 15000),
        **{k:artifact[k] for k in CONTROLS[2:] if k in artifact and k != 'temperature'})
    if verify:
        with Path(artifact['path']).open('rb') as stream:
            actual = hashlib.file_digest(stream, 'sha256').hexdigest()
        if actual != artifact['sha256']:
            raise ValueError('selected flight artifact checksum mismatch')
    artifact.update(role='qwen_edge_primary', model_role='flight_llm')
    return artifact

"""A tracked dependency fix is reproducible; unrelated edits are never erased."""
import subprocess
from pathlib import Path
import pytest
from python.dcm.runtime_patches import apply_runtime_patch


def checkout(tmp_path):
    source = tmp_path / 'source';source.mkdir()
    def git(*args):
        return subprocess.check_output(['git', '-C', str(source), *args])
    git('init', '-q')
    file = source / 'parser.cpp';file.write_text('original parser\n')
    git('add', 'parser.cpp')
    git('-c', 'user.name=Runtime test', '-c', 'user.email=test@example.invalid', 'commit', '-qm', 'base')
    revision = git('rev-parse', 'HEAD').decode().strip()
    file.write_text('corrected parser\n')
    patch = tmp_path / 'runtime.patch';patch.write_bytes(git('diff', '--binary'))
    file.write_text('original parser\n')
    return source, patch, revision, file


def test_runtime_patch_is_idempotent_and_check_only_never_mutates(tmp_path):
    source, patch, revision, file = checkout(tmp_path)
    digest = apply_runtime_patch(source, patch, revision, check_only=True)
    assert file.read_text() == 'original parser\n'
    assert apply_runtime_patch(source, patch, revision) == digest
    assert file.read_text() == 'corrected parser\n'
    assert apply_runtime_patch(source, patch, revision) == digest
    assert apply_runtime_patch(source, patch, revision, check_only=True) == digest


@pytest.mark.parametrize('after_patch', [False, True])
def test_external_changes_are_not_reverted_or_overwritten(tmp_path, after_patch):
    source, patch, revision, file = checkout(tmp_path)
    if after_patch:apply_runtime_patch(source, patch, revision)
    file.write_text(file.read_text() + 'operator edit\n');before=file.read_bytes()
    with pytest.raises(ValueError, match='unrecognized local changes'):
        apply_runtime_patch(source, patch, revision)
    assert file.read_bytes() == before


def test_wrong_revision_and_staged_changes_are_refused(tmp_path):
    source, patch, revision, file = checkout(tmp_path)
    with pytest.raises(ValueError, match='pinned source revision'):
        apply_runtime_patch(source, patch, 'not-the-pinned-revision')
    file.write_text('operator staged edit\n')
    subprocess.run(['git', '-C', str(source), 'add', 'parser.cpp'], check=True)
    with pytest.raises(ValueError, match='staged local changes'):
        apply_runtime_patch(source, patch, revision)
    assert file.read_text() == 'operator staged edit\n'


def test_serving_runtime_must_match_pinned_patch_build_identity(tmp_path):
    from python.dcm.runtime_patches import runtime_build_id, verify_runtime_build
    binary = tmp_path / 'build/bin/llama-server';binary.parent.mkdir(parents=True)
    binary.write_bytes(b'executable')
    patch = tmp_path / 'fix.patch';patch.write_bytes(b'known native patch')
    marker = binary.parent.parent / '.icarus-built-revision'
    for value in (None, 'old-upstream-revision', 'revision+patch-old-digest'):
        if value is not None:marker.write_text(value)
        with pytest.raises(RuntimeError, match='setup-model-runtime'):
            verify_runtime_build(binary, 'revision', patch)
    marker.write_text(runtime_build_id('revision', patch))
    assert verify_runtime_build(binary, 'revision', patch) == marker.read_text()
    patch.write_bytes(b'updated native patch')
    with pytest.raises(RuntimeError, match='setup-model-runtime'):
        verify_runtime_build(binary, 'revision', patch)

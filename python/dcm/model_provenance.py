"""Verify local GGUF bytes against immutable public publisher metadata."""
import argparse
import copy
import hashlib
import json
import re
from pathlib import Path
from urllib.parse import quote
from urllib.request import urlopen


def publisher_revision(metadata):
    revision = metadata.get('sha')
    if not isinstance(revision, str) or not re.fullmatch(r'[0-9a-f]{40}', revision):
        raise ValueError('publisher metadata lacks an immutable commit revision')
    return revision


def publisher_metadata(repository, cache=None, revision=None):
    """Fetch only metadata; cached immutable metadata permits offline upgrades."""
    if revision is not None and not re.fullmatch(r'[0-9a-f]{40}', revision):
        raise ValueError('requested publisher revision must be an immutable commit')
    fetched = False
    try:
        url = 'https://huggingface.co/api/models/' + quote(repository, safe='/') + (('/revision/' + revision) if revision else '') + '?blobs=true'
        with urlopen(url, timeout=20) as response:
            metadata = json.load(response)
        publisher_revision(metadata)
        fetched = True
    except (OSError, ValueError):
        if cache is None or not Path(cache).is_file():
            raise
        stored = json.loads(Path(cache).read_text())
        if stored.get('repository') != repository:
            raise ValueError('cached publisher metadata repository differs')
        metadata = stored['metadata']
        publisher_revision(metadata)
    if revision is not None and publisher_revision(metadata) != revision:
        raise ValueError('publisher metadata revision differs from requested immutable commit')
    if cache and fetched:
        Path(cache).write_text(json.dumps({'repository':repository,'metadata':metadata},indent=2)+'\n')
    return metadata


def verify_publisher_artifact(path, metadata):
    """A known local hash alone is not publisher provenance: compare LFS digest."""
    path = Path(path)
    rows = [row for row in metadata.get('siblings', []) if row.get('rfilename') == path.name]
    if len(rows) != 1:
        raise ValueError(f'publisher metadata does not identify {path.name}')
    expected = rows[0].get('lfs', {}).get('sha256')
    if not isinstance(expected, str) or not re.fullmatch(r'[0-9a-f]{64}',expected):
        raise ValueError('publisher GGUF SHA256 unavailable')
    with path.open('rb') as stream:
        actual = hashlib.file_digest(stream,'sha256').hexdigest()
    if actual != expected:
        raise ValueError(f'{path.name} does not match publisher revision {publisher_revision(metadata)}; select metadata for its exact revision, do not relabel existing bytes')
    return actual


def upgrade_legacy_source(manifest, role, metadata):
    """Return upgraded source without modifying the installed legacy manifest."""
    result = copy.deepcopy(manifest)
    selected = [a for a in result.get('artifacts',[]) if a.get('role') == role]
    if len(selected) != 1:
        raise ValueError('legacy flight role must identify one artifact')
    artifact = selected[0]
    digest = verify_publisher_artifact(artifact['path'], metadata)
    if artifact.get('sha256') != digest:
        raise ValueError('legacy manifest checksum does not match verified publisher bytes')
    artifact.setdefault('repository', manifest.get('repository'))
    artifact.setdefault('revision', manifest.get('revision', publisher_revision(metadata)))
    # Do not guess a missing architecture label from a publisher/repository name.
    artifact.setdefault('family', manifest.get('family', 'unknown'))
    artifact.setdefault('runtime','llama_cpp')
    return result


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--repository',required=True)
    parser.add_argument('--cache',type=Path,required=True)
    parser.add_argument('--revision')
    parser.add_argument('--verify',nargs='*',type=Path)
    args=parser.parse_args()
    if args.verify is not None:
        stored=json.loads(args.cache.read_text())
        if stored['repository'] != args.repository:raise ValueError('repository mismatch')
        metadata=stored['metadata']
        for path in args.verify:verify_publisher_artifact(path,metadata)
    else:
        metadata=publisher_metadata(args.repository,args.cache,args.revision)
    print(publisher_revision(metadata))

if __name__ == '__main__':main()

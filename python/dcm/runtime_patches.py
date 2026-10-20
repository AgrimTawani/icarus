"""Apply the pinned native-runtime patch without overwriting external edits."""
import argparse
import hashlib
import subprocess
from pathlib import Path


def apply_runtime_patch(source, patch, revision, *, check_only=False):
    source, patch = Path(source).resolve(), Path(patch).resolve()
    def git(*arguments, **kwargs):
        return subprocess.run(['git', '-C', str(source), *arguments],
                              check=True, capture_output=True, **kwargs)
    head = git('rev-parse', 'HEAD').stdout.decode().strip()
    if head != revision:
        raise ValueError('Native runtime is not at the pinned source revision')
    if git('diff', '--cached', '--binary').stdout:
        raise ValueError('Native runtime has staged local changes; left unchanged')
    expected = patch.read_bytes()
    current = git('diff', 'HEAD', '--binary').stdout
    if current and current != expected:
        raise ValueError('Native runtime has unrecognized local changes; left unchanged')
    if not current:
        git('apply', '--check', str(patch))
        if not check_only:
            git('apply', str(patch))
            if git('diff', 'HEAD', '--binary').stdout != expected:
                raise ValueError('Applied native patch does not match its recorded source diff')
    return hashlib.sha256(expected).hexdigest()


def runtime_build_id(revision, patch):
    return revision + '+patch-' + hashlib.sha256(Path(patch).read_bytes()).hexdigest()


def verify_runtime_build(binary, revision, patch):
    marker = Path(binary).parent.parent / '.icarus-built-revision'
    expected = runtime_build_id(revision, patch)
    try:
        current = marker.read_text().strip()
    except OSError:
        current = None
    if current != expected:
        raise RuntimeError('Native model runtime does not contain the pinned tool-parser fix. '
                           'Run ./scripts/setup-model-runtime --runtime-only before starting a flight session.')
    return expected


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--source', type=Path, required=True)
    parser.add_argument('--patch', type=Path, required=True)
    parser.add_argument('--revision', required=True)
    parser.add_argument('--check-only', action='store_true')
    args = parser.parse_args()
    try:
        digest = apply_runtime_patch(args.source, args.patch, args.revision,
                                     check_only=args.check_only)
    except (OSError, ValueError, subprocess.CalledProcessError) as error:
        parser.exit(1, str(error) + '\n')
    print(digest)


if __name__ == '__main__':
    main()

"""Make the generated protobuf bindings importable, regardless of caller.

`icarus.v1.*` lives under `build/generated/python`, which is not on
`sys.path` by default. `fly_cli.py` added it itself at the top of the file,
but `guardrail_check.py` and `actions.py` are imported from more places than
one CLI entry point — the evaluation harness, tests, anything that ends up
calling `check_proposal` — and each of those would otherwise need to
remember the same path insertion. Doing it once here, at the point of need
rather than at every entry point, is what actually prevents the next caller
from hitting the same ModuleNotFoundError this one did.
"""

import sys
from pathlib import Path

_GENERATED = Path(__file__).resolve().parents[2] / "build/generated/python"


def ensure_generated_proto_on_path():
    path = str(_GENERATED)
    if path not in sys.path:
        sys.path.insert(0, path)

"""Print, check, or write the canonical generated DCM BODY manifest."""

import argparse
import json
from pathlib import Path

from python.dcm.v2_profiles import BODY_MANIFEST, expected_body_manifest


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    mode = parser.add_mutually_exclusive_group()
    mode.add_argument("--check", action="store_true",
                      help="fail if the checked-in manifest is stale")
    mode.add_argument("--write", action="store_true",
                      help="write the generated manifest for human review")
    args = parser.parse_args()
    rendered = json.dumps(expected_body_manifest(), indent=2, sort_keys=True) + "\n"
    if args.check:
        if not Path(BODY_MANIFEST).is_file() or BODY_MANIFEST.read_text() != rendered:
            raise SystemExit("BODY manifest is stale; run generate-dcm-body --write and review")
        print("BODY manifest is current:", BODY_MANIFEST)
        return
    if args.write:
        BODY_MANIFEST.parent.mkdir(parents=True, exist_ok=True)
        BODY_MANIFEST.write_text(rendered)
        print("Wrote:", BODY_MANIFEST)
        return
    print(rendered, end="")


if __name__ == "__main__":
    main()

"""Command-line wrapper for the pinned open-vocabulary detector."""

import argparse
import json
from pathlib import Path

from python.perception.vision import detect_image, normalize_classes


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("image", type=Path)
    parser.add_argument("--classes", required=True,
                        help="comma-separated classes, e.g. 'person,tree,building'")
    parser.add_argument("--manifest", type=Path,
                        default=Path.home() / "models/vision/MANIFEST.json")
    parser.add_argument("--threshold", type=float, default=0.35)
    parser.add_argument("--text-threshold", type=float, default=0.25)
    parser.add_argument("--device", choices=("auto", "cuda", "cpu"), default="auto")
    parser.add_argument("--output", type=Path)
    args = parser.parse_args()
    classes = normalize_classes(args.classes.split(","))
    result = detect_image(args.image, classes, args.manifest, args.threshold,
                          args.text_threshold, args.device)
    rendered = json.dumps(result, indent=2, sort_keys=True) + "\n"
    if args.output:
        args.output.parent.mkdir(parents=True, exist_ok=True)
        args.output.write_text(rendered)
    print(rendered, end="")


if __name__ == "__main__":
    main()

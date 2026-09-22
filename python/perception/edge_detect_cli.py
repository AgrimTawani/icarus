"""Run one pinned YOLO11n edge-profile observation on an image."""
import argparse
import json
from pathlib import Path

from .edge_vision import UniqueTrackCounter, detect_edge_image


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("image", type=Path)
    parser.add_argument("--manifest", type=Path, default=Path.home() / "models/edge/MANIFEST.json")
    parser.add_argument("--device", choices=("auto", "cuda", "cpu"), default="auto")
    parser.add_argument("--max-fps", type=float, default=5)
    args = parser.parse_args()
    print(json.dumps(detect_edge_image(args.image, args.manifest, UniqueTrackCounter(),
                                        device=args.device, max_fps=args.max_fps), indent=2, sort_keys=True))


if __name__ == "__main__":
    main()

"""Assess one saved calibrated downward depth frame; emits JSON evidence."""

import argparse
import json
from pathlib import Path

import numpy as np

from python.perception.landing_zone import assess_landing_zone


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("depth", type=Path, help="float32 .npy depth image in metres")
    parser.add_argument("--axis-body-frd", required=True, nargs=3, type=float,
                        metavar=("X", "Y", "Z"),
                        help="calibrated camera optical axis in body-FRD")
    parser.add_argument("--horizontal-fov-deg", type=float, default=87.0)
    parser.add_argument("--vertical-fov-deg", type=float, default=58.0)
    parser.add_argument("--output", type=Path)
    args = parser.parse_args()
    result = assess_landing_zone(
        np.load(args.depth, allow_pickle=False),
        optical_axis_body_frd=args.axis_body_frd,
        horizontal_fov_deg=args.horizontal_fov_deg,
        vertical_fov_deg=args.vertical_fov_deg)
    rendered = json.dumps(result, indent=2, sort_keys=True) + "\n"
    if args.output:
        args.output.parent.mkdir(parents=True, exist_ok=True)
        args.output.write_text(rendered)
    print(rendered, end="")


if __name__ == "__main__":
    main()

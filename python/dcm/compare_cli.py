"""Compare several evaluate-dcm reports side by side.

Produces a table, not a ranking; see compare.py for why.
"""

import argparse
from pathlib import Path

from python.dcm.compare import compare, format_comparison


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("reports", type=Path, nargs="+",
                        help="two or more evaluation.json paths to compare")
    parser.add_argument("--output", type=Path,
                        default=Path(__file__).resolve().parents[2]
                        / "logs/dcm/comparison",
                        help="directory to write the comparison run into")
    args = parser.parse_args()

    output, comparison = compare(args.reports, args.output)
    print(format_comparison(comparison))
    print()
    print("Comparison report:", output / "comparison.json")


if __name__ == "__main__":
    main()

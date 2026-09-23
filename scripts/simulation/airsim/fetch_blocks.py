#!/usr/bin/env python3
"""Fetch the pinned public AirSim Blocks asset for visual/RPC smoke checks only.

This does not install an ArduCopter-capable AirSim environment and it never
changes the Gazebo path. The digest is verified before extraction.
"""

import argparse
import hashlib
import json
import urllib.request
import zipfile
from pathlib import Path

ROOT = Path(__file__).resolve().parents[3]
PIN = ROOT / "config/simulation/airsim/blocks-v1.8.0.json"
DEFAULT_DESTINATION = ROOT / "third_party/airsim"


def digest(path):
    hasher = hashlib.sha256()
    with path.open("rb") as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b""):
            hasher.update(block)
    return hasher.hexdigest()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--destination", type=Path, default=DEFAULT_DESTINATION)
    parser.add_argument("--force", action="store_true", help="Redownload the pinned archive.")
    args = parser.parse_args()
    pin = json.loads(PIN.read_text())
    destination = args.destination.resolve()
    archive = destination / pin["archive"]
    extracted = destination / pin["extract_dir"]
    entrypoint = extracted / pin["entrypoint"]
    destination.mkdir(parents=True, exist_ok=True)
    if args.force or not archive.is_file():
        print("Downloading pinned AirSim Blocks smoke asset...")
        urllib.request.urlretrieve(pin["url"], archive)
    actual = digest(archive)
    if actual != pin["sha256"]:
        raise SystemExit(f"checksum mismatch for {archive}: expected {pin['sha256']}, got {actual}")
    if not entrypoint.is_file():
        if extracted.exists():
            raise SystemExit(
                f"expected entrypoint is missing in existing {extracted}; inspect or remove that exact external directory, then retry")
        with zipfile.ZipFile(archive) as bundle:
            bundle.extractall(extracted)
    if not entrypoint.is_file():
        raise SystemExit(f"archive did not contain expected entrypoint: {entrypoint}")
    entrypoint.chmod(entrypoint.stat().st_mode | 0o111)
    print("Ready:", entrypoint)
    print("Capability: visual/RPC smoke only (SimpleFlight); it is refused for ArduCopter tests.")


if __name__ == "__main__":
    main()

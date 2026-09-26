#!/usr/bin/env python3
"""Compile and run the Icarus7 parameter test against a built AirSim checkout."""

import argparse
import subprocess
import tempfile
from pathlib import Path

ROOT = Path(__file__).resolve().parents[3]
SOURCE = ROOT / "simulation/airsim/vehicles/icarus7/tests/airlib_params_test.cpp"


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--airsim-source", type=Path, required=True)
    args = parser.parse_args()
    airsim = args.airsim_source.resolve()
    libraries = airsim / "build_release/output/lib"
    required = tuple(libraries / name for name in (
        "libAirLib.a", "libMavLinkCom.a", "librpc.a"))
    missing = [str(path) for path in required if not path.is_file()]
    if missing:
        raise SystemExit("build AirSim before running this test; missing: " + ", ".join(missing))
    with tempfile.TemporaryDirectory(prefix="icarus7-airlib-") as temp:
        binary = Path(temp) / "icarus7_airlib_params_test"
        command = [
            "g++", "-std=c++14", "-O2", "-pthread",
            "-DMSGPACK_PP_VARIADICS_MSVC=0",
            "-I", str(airsim / "AirLib/include"),
            "-I", str(airsim / "AirLib/deps/eigen3"),
            "-I", str(airsim / "MavLinkCom/include"),
            "-I", str(airsim / "external/rpclib/rpclib-2.3.0/include"),
            str(SOURCE), *(str(path) for path in required),
            "-o", str(binary),
        ]
        subprocess.run(command, check=True)
        subprocess.run([str(binary)], check=True)


if __name__ == "__main__":
    main()

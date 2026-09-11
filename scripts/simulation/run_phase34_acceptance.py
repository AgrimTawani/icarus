#!/usr/bin/python3
"""Five consecutive automated flights; numeric telemetry only."""

import json
import signal
import subprocess
import uuid

from sensor_profiles import ROOT


def main():
    output = ROOT / "logs/simulation" / ("phase34_acceptance_" + uuid.uuid4().hex[:8])
    output.mkdir(parents=True)
    results = []
    print("Acceptance evidence:", output, flush=True)
    for index in range(5):
        profile = "noisy" if index == 4 else "nominal"
        before = set((ROOT / "logs/simulation").glob("compact_flight_*"))
        print(f"Flight {index + 1}/5: {profile}; no media", flush=True)
        with (output / f"flight_{index + 1}.log").open("w") as log:
            child = subprocess.Popen(
                [
                    str(ROOT / "scripts/sim"),
                    "--sensor-profile",
                    profile,
                    "--seed",
                    str(100 + index),
                ],
                stdout=log,
                stderr=subprocess.STDOUT,
                start_new_session=True,
            )
            try:
                child.wait(timeout=480)
                if child.returncode:
                    raise RuntimeError(f"Flight {index + 1} failed; inspect {output}")
            finally:
                if child.poll() is None:
                    child.send_signal(signal.SIGINT)
                    child.wait(timeout=25)
        new = set((ROOT / "logs/simulation").glob("compact_flight_*")) - before
        assert len(new) == 1
        run = new.pop()
        launch = json.loads((run / "launch.json").read_text())
        result = json.loads((run / "result.json").read_text())
        assert launch["status"] == result["status"] == "passed"
        assert (
            not list(run.rglob("*.png"))
            and not list(run.rglob("*.gif"))
            and not list(run.rglob("*.mp4"))
        )
        assert (
            json.loads((run / "sensors/schema.json").read_text())["image_pixels_saved"]
            is False
        )
        verified = json.loads((run / "recording_verified.json").read_text())
        # Compare advancing simulation-time IMU stamps with wall receipt times.
        rows = [
            json.loads(line)
            for line in (run / "sensors/index.jsonl").read_text().splitlines()
            if '"channel": "imu"' in line
        ]
        first, last = rows[0], rows[-1]
        rtf = (last["simulation_s"] - first["simulation_s"]) / (
            last["wall_monotonic_s"] - first["wall_monotonic_s"]
        )
        assert rtf > 0.8, rtf
        results.append(
            {
                "flight": index + 1,
                "profile": profile,
                "run": str(run),
                "hover": result["hover"],
                "real_time_factor": rtf,
                "recorded_channels": list(verified["channels"]),
                "status": "passed",
            }
        )
        (output / "progress.json").write_text(json.dumps(results, indent=2) + "\n")
        print(
            f"PASS {index + 1}/5; RTF={rtf:.3f}; drift={result['hover']['max_drift_m']:.4f} m",
            flush=True,
        )
    (output / "results.json").write_text(
        json.dumps(
            {
                "status": "passed",
                "consecutive_flights": 5,
                "media_saved": False,
                "flights": results,
            },
            indent=2,
        )
        + "\n"
    )
    print("PASS: five consecutive flights:", output, flush=True)


if __name__ == "__main__":
    main()

#!/usr/bin/python3
"""Exercise port conflicts, duplicate starts, interruption and stale sensors."""

import json
import os
import signal
import socket
import subprocess
import time
import uuid

from build_akshu_candidate import ROOT


def main():
    folder = ROOT / "logs/simulation" / ("launcher_faults_" + uuid.uuid4().hex[:8])
    folder.mkdir(parents=True)
    command = [
        "/usr/bin/python3",
        str(ROOT / "scripts/simulation/launch_compact.py"),
        "--preflight-only",
    ]
    results = []
    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as occupied:
        occupied.bind(("127.0.0.1", 5760))
        occupied.listen()
        result = subprocess.run(
            command, capture_output=True, text=True, timeout=10, check=False
        )
        assert result.returncode and "Address already in use" in result.stderr
        assert occupied.getsockname()[1] == 5760
        results.append({"case": "occupied_port", "status": "passed"})
    for case in ("interrupt", "stale_sensors", "recorder_exit"):
        before = set((ROOT / "logs/simulation").glob("compact_flight_*"))
        with (folder / (case + ".log")).open("w") as log:
            parent = subprocess.Popen(
                command, stdout=log, stderr=subprocess.STDOUT, start_new_session=True
            )
            directory = None
            try:
                deadline = time.monotonic() + 60
                while True:
                    new = (
                        set((ROOT / "logs/simulation").glob("compact_flight_*"))
                        - before
                    )
                    if new:
                        assert len(new) == 1
                        directory = next(iter(new))
                        path = directory / "sensors/health.json"
                        if (
                            path.exists()
                            and json.loads(path.read_text()).get("status") == "ready"
                        ):
                            break
                    assert parent.poll() is None, (
                        "Launcher exited before test injection"
                    )
                    assert time.monotonic() < deadline, "Readiness timeout"
                    time.sleep(0.2)
                runtime = json.loads((directory / "runtime.json").read_text())
                if case == "interrupt":
                    duplicate = subprocess.run(
                        command, capture_output=True, text=True, timeout=10, check=False
                    )
                    assert (
                        duplicate.returncode
                        and "Another compact launcher is active" in duplicate.stderr
                    )
                    results.append({"case": "duplicate_launcher", "status": "passed"})
                    parent.send_signal(signal.SIGINT)
                elif case == "recorder_exit":
                    os.kill(runtime["children"]["recorder"], signal.SIGTERM)
                else:
                    env = os.environ.copy()
                    env["GZ_PARTITION"] = runtime["partition"]
                    reply = subprocess.run(
                        [
                            "gz",
                            "service",
                            "-s",
                            "/world/compact_flight/control",
                            "--reqtype",
                            "gz.msgs.WorldControl",
                            "--reptype",
                            "gz.msgs.Boolean",
                            "--timeout",
                            "5000",
                            "--req",
                            "pause: true",
                        ],
                        env=env,
                        capture_output=True,
                        text=True,
                        timeout=8,
                        check=True,
                    )
                    assert "data: true" in reply.stdout
                parent.wait(timeout=30)
                assert parent.returncode != 0
                result = json.loads((directory / "launch.json").read_text())
                assert result["status"] == "failed"
                if case == "stale_sensors":
                    assert "unhealthy" in result["error"]
                if case == "recorder_exit":
                    assert "recorder" in result["error"]
                # Every process group belongs to this test's launcher only.
                runtime = json.loads((directory / "runtime.json").read_text())
                for pid in runtime["children"].values():
                    try:
                        os.killpg(pid, 0)
                    except ProcessLookupError:
                        continue
                    raise AssertionError(f"Process group {pid} was left behind")
                with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as port:
                    port.bind(("127.0.0.1", 5760))
                controller = directory / "controller.log"
                assert (
                    not controller.exists()
                    or "Arming motors" not in controller.read_text()
                )
                results.append(
                    {"case": case, "status": "passed", "run": str(directory)}
                )
            finally:
                if parent.poll() is None:
                    parent.send_signal(signal.SIGINT)
                    parent.wait(timeout=20)
    (folder / "results.json").write_text(json.dumps(results, indent=2) + "\n")
    print(json.dumps(results, indent=2))
    print(folder)


if __name__ == "__main__":
    main()

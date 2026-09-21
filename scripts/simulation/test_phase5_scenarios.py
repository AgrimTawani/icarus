#!/usr/bin/python3
"""Phase 5 deterministic-generation, ground-truth and performance gates."""

import argparse
import hashlib
import json
import os
import signal
import subprocess
import time
import uuid
from pathlib import Path

from build_akshu_candidate import ROOT
from build_phase5_world import build
from gz.msgs10.world_stats_pb2 import WorldStatistics
from gz.transport13 import Node
from scenario_config import SCENARIO_DIR, load_scenario
from score_phase5_trajectory import score


def digest(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def measure_rtf(world_path, world_name, seed):
    env = os.environ.copy()
    partition = "icarus_phase5_test_" + uuid.uuid4().hex
    env["GZ_PARTITION"] = partition
    env["GZ_SIM_RESOURCE_PATH"] = str(ROOT / "simulation/models")
    env["GZ_SIM_SYSTEM_PLUGIN_PATH"] = (
        str(ROOT / "simulation/plugins/build") + ":" + str(ROOT / "third_party/ardupilot_gazebo/build")
    )
    vendor = "/usr/share/glvnd/egl_vendor.d/10_nvidia.json"
    if os.path.exists(vendor):
        env["__EGL_VENDOR_LIBRARY_FILENAMES"] = vendor
    log_dir = ROOT / "logs/simulation" / ("phase5_performance_" + uuid.uuid4().hex[:8])
    log_dir.mkdir(parents=True)
    with (log_dir / "gazebo.log").open("w") as log:
        process = subprocess.Popen(
            ["gz", "sim", "-s", "-r", "--headless-rendering", "--seed", str(seed), "-v", "3", str(world_path)],
            env=env, stdout=log, stderr=subprocess.STDOUT, start_new_session=True,
        )
        values = []
        previous_partition = os.environ.get("GZ_PARTITION")
        os.environ["GZ_PARTITION"] = partition
        node = Node()

        def callback(message):
            values.append((message.sim_time.sec + message.sim_time.nsec * 1e-9, time.monotonic(), message.real_time_factor))

        # Gazebo may expose the canonical world topic or add the root fallback,
        # depending on whether discovery completed before server startup.
        assert node.subscribe(WorldStatistics, f"/world/{world_name}/stats", callback)
        assert node.subscribe(WorldStatistics, "/stats", callback)
        try:
            deadline = time.monotonic() + 45
            while (not values or values[-1][0] - values[0][0] < 20) and time.monotonic() < deadline:
                if process.poll() is not None:
                    raise RuntimeError("Gazebo exited; inspect " + str(log_dir / "gazebo.log"))
                time.sleep(0.05)
            if not values or values[-1][0] - values[0][0] < 20:
                raise TimeoutError("world did not complete warm-up plus measurement")
            target = values[-1][0] - 8
            start_index = min(range(len(values)), key=lambda i: abs(values[i][0] - target))
            rtf = ((values[-1][0] - values[start_index][0])
                   / (values[-1][1] - values[start_index][1]))
        finally:
            if process.poll() is None:
                os.killpg(process.pid, signal.SIGINT)
                try:
                    process.wait(timeout=5)
                except subprocess.TimeoutExpired:
                    os.killpg(process.pid, signal.SIGTERM)
                    process.wait(timeout=5)
            if previous_partition is None:
                os.environ.pop("GZ_PARTITION", None)
            else:
                os.environ["GZ_PARTITION"] = previous_partition
    result = {"world": world_name, "measurement": "steady-state final 8 simulation seconds after warm-up",
              "measured_real_time_factor": rtf, "samples": len(values), "status": "passed" if rtf >= 0.8 else "failed"}
    (log_dir / "results.json").write_text(json.dumps(result, indent=2) + "\n")
    assert result["status"] == "passed", result
    return result, log_dir


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--integrated-run-directory", type=Path)
    args = parser.parse_args()
    scenario_paths = sorted(SCENARIO_DIR.glob("*.json"))
    scenario_names = {path.stem for path in scenario_paths}
    assert {
        "empty_validation", "wind_light", "wind_strong", "wind_gusting",
        "wind_direction_change", "wind_limit_reject", "obstacle_course",
        "adverse_combined",
    } <= scenario_names
    canonical_sources = [
        ROOT / "simulation/models/akshu_compact_sitl/model.sdf",
        ROOT / "simulation/worlds/compact_flight.sdf",
        ROOT / "simulation/launch/compact_gui.config",
    ]
    canonical_hashes = {path: digest(path) for path in canonical_sources}
    results = []
    for path in scenario_paths:
        _, scenario = load_scenario(path)
        world, truth, _, first = build(path)
        hashes = (digest(world), digest(truth), first["vehicle_model_sha256"])
        world2, truth2, _, second = build(path)
        assert hashes == (digest(world2), digest(truth2), second["vehicle_model_sha256"])
        check = subprocess.run(
            ["gz", "sdf", "-k", str(world)], capture_output=True, text=True,
            env={**os.environ, "SDF_PATH": str(ROOT / "simulation/models")}, timeout=15,
            check=False,
        )
        assert check.returncode == 0 and "Valid" in check.stdout, check.stderr
        truth_data = json.loads(truth.read_text())
        if scenario["environment_preset"] == "mixed_village":
            assert len(truth_data["obstacles"]) == 7
            text = world.read_text()
            assert "village_ground_details" in text and "window_front" in text
            assert "canopy_cluster" in text
        if scenario["wind"]["enabled"]:
            model_text = (ROOT / "simulation/models" / ("phase5_" + scenario["name"]) / "model.sdf").read_text()
            assert "IcarusTurbulentAtmosphere" in model_text
            assert "<turbulence_intensity>0</turbulence_intensity>" not in model_text
            # The custom model applies the full aerodynamic wrench.  Native
            # WindEffects opt-in would stack a second, uncalibrated force if a
            # world later enabled that system.
            assert "<enable_wind>true</enable_wind>" not in model_text
        if scenario["world_profile"] in ("obstacles", "adverse"):
            assert scenario["obstacles"] and json.loads(truth.read_text())["obstacles"] == scenario["obstacles"]
            for obstacle in scenario["obstacles"]:
                if obstacle["type"] == "building":
                    assert obstacle["size_m"][2] <= 6
                if obstacle["type"] == "tree":
                    assert obstacle["canopy_collision"] in ("enabled", "disabled")
        if scenario["name"] == "adverse_combined":
            model_text = (ROOT / "simulation/models/phase5_adverse_combined/model.sdf").read_text()
            assert "<initial_charge>3.5</initial_charge>" in model_text
            assert "<power_load>850</power_load>" in model_text
            assert model_text.count("<stddev>0.8</stddev>") >= 2
        results.append({"scenario": scenario["name"], "world_sha256": hashes[0], "ground_truth_sha256": hashes[1], "status": "passed"})

    assert canonical_hashes == {path: digest(path) for path in canonical_sources}, (
        "scenario generation modified a canonical simulator source"
    )

    _, obstacle = load_scenario("obstacle_course")
    safe_route = next(route for route in obstacle["ground_truth"]["routes"] if route["name"] == "open")
    safe_points = [{"t_s": i * 5, "x_m": p[0], "y_m": p[1], "z_m": p[2]} for i, p in enumerate(safe_route["waypoints_enu_m"])]
    safe = score(obstacle, safe_points, "open")
    assert safe["status"] == "passed", safe
    collision = score(obstacle, [
        {"t_s": 0, "x_m": 0, "y_m": 0, "z_m": 2},
        {"t_s": 10, "x_m": 10, "y_m": 0, "z_m": 2},
    ])
    assert collision["status"] == "failed" and "calibration_wall" in collision["collisions"]

    if args.integrated_run_directory:
        integrated = args.integrated_run_directory.resolve()
        launch = json.loads((integrated / "launch.json").read_text())
        performance = json.loads((integrated / "scenario_score.json").read_text())
        assert launch["status"] == "passed" and launch["scenario"] == "adverse_combined"
        assert performance["status"] == "passed" and performance["real_time_factor"] >= 0.8
        performance_dir = integrated
    else:
        completed = subprocess.run(
            [str(ROOT / "scripts/sim"), "--scenario", "adverse_combined"],
            cwd=ROOT, capture_output=True, text=True, timeout=240, check=False,
        )
        assert completed.returncode == 0, completed.stdout + completed.stderr
        line = next(line for line in completed.stdout.splitlines() if line.startswith("Artifacts:"))
        integrated = Path(line.split(":", 1)[1].strip())
        performance = json.loads((integrated / "scenario_score.json").read_text())
        assert performance["status"] == "passed"
        performance_dir = integrated
    report_dir = ROOT / "logs/simulation" / ("phase5_acceptance_" + uuid.uuid4().hex[:8])
    report_dir.mkdir(parents=True)
    report = {"status": "passed", "scenarios": results, "scoring": {"safe": safe, "collision": collision},
              "performance": performance, "performance_evidence": str(performance_dir)}
    (report_dir / "results.json").write_text(json.dumps(report, indent=2) + "\n")
    print(json.dumps(report, indent=2))
    print(report_dir)


if __name__ == "__main__":
    main()

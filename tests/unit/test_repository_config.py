import json
import re
import sys
from pathlib import Path

import pytest

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / "scripts/simulation"))

from launch_profiles import load_profiles, resolve_profile


def test_launch_profiles_resolve_to_versioned_scenarios():
    profiles = load_profiles()
    simulation = {
        name: profile
        for name, profile in profiles.items()
        if profile["adapter"] == "sitl"
    }
    assert set(simulation) == {
        "simulation-empty",
        "simulation-wind",
        "simulation-obstacles",
        "simulation-adverse",
        "simulation-perception-stress",
    }
    for name, profile in simulation.items():
        assert resolve_profile(name) == profile
        assert (ROOT / "simulation/scenarios" / f"{profile['scenario']}.json").is_file()


def test_hardware_profiles_fail_closed_until_authorized():
    for name in ("hardware-bench", "hardware-flight"):
        with pytest.raises(RuntimeError, match="reserved but unavailable"):
            resolve_profile(name)


def test_third_party_lock_uses_immutable_revisions():
    lock = json.loads(
        (ROOT / "config/dependencies/third_party.lock.json").read_text()
    )
    assert lock["version"] == 1
    assert set(lock["repositories"]) == {"ardupilot", "ardupilot_gazebo"}
    for dependency in lock["repositories"].values():
        assert dependency["url"].startswith("https://github.com/ArduPilot/")
        assert re.fullmatch(r"[0-9a-f]{40}", dependency["revision"])


def test_direct_python_requirements_are_exactly_pinned():
    for path in sorted((ROOT / "requirements").glob("*.txt")):
        for line in path.read_text().splitlines():
            value = line.strip()
            if not value or value.startswith(("#", "-r ")):
                continue
            assert "==" in value, f"unbounded dependency in {path}: {value}"

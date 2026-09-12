#!/usr/bin/python3
"""Load the versioned operator launch profiles."""

import json

from build_akshu_candidate import ROOT

PROFILE_PATH = ROOT / "config/simulation/launch_profiles.json"


def load_profiles():
    data = json.loads(PROFILE_PATH.read_text())
    if data.get("version") != 1 or not isinstance(data.get("profiles"), dict):
        raise ValueError("invalid launch profile document")
    for name, profile in data["profiles"].items():
        if not name or profile.get("adapter") not in ("sitl", "hardware"):
            raise ValueError(f"invalid launch profile: {name}")
        if profile["adapter"] == "sitl" and not profile.get("scenario"):
            raise ValueError(f"simulation profile {name} has no scenario")
    return data["profiles"]


def resolve_profile(name):
    profiles = load_profiles()
    try:
        profile = profiles[name]
    except KeyError as error:
        choices = ", ".join(sorted(profiles))
        raise ValueError(f"unknown profile {name!r}; choose one of: {choices}") from error
    if profile.get("available", True) is not True:
        raise RuntimeError(
            f"profile {name} is reserved but unavailable until physical integration"
        )
    return profile

#!/usr/bin/env python3
"""Check a hosted model provider against what a mission needs, before flying with it.

Reads the provider from the environment (never printed): ICARUS_LLM_BASE_URL, ICARUS_LLM_API_KEY,
ICARUS_LLM_MODEL, and optionally the ICARUS_VLM_* equivalents for the camera model. Needs no
simulator. Checks, one line each with latency and tokens:

  provider_reachable  the provider answers and accepts the key (no model call)
  catalogue_takeoff   the real 23-tool catalogue is accepted and a takeoff request returns a takeoff call
  validate_call       the flight code's own validator accepts that call
  condition_true      "is there a building?" on a labelled building frame answers found=true
  condition_false     the same question on a labelled non-building frame answers found=false
  pixel_box           "house" on the building frame returns a box inside the image

A check that cannot run (for example the stored frames are missing, or validate_call has no takeoff
to validate) prints SKIP and is not a pass. A refused request prints the provider's reason on the
FAIL line (the key is never in it). Exit status: 0 only when every check passed; 1 when any failed;
2 when the settings are missing or unusable; 3 when nothing failed but some check was skipped.

  ICARUS_LLM_BASE_URL=https://... ICARUS_LLM_API_KEY=... ICARUS_LLM_MODEL=... \
      .venv/bin/python scripts/autonomy/check_cloud_provider.py
"""

import argparse
import json
import sys
import time
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT))

from python.model_runtime.cloud import RejectedRequest  # noqa: E402
from python.model_runtime.cloud_config import InvalidCloudSetting, MissingCloudSetting  # noqa: E402
from python.model_runtime.select import make_runtime  # noqa: E402

DEFAULT_MANIFEST = ROOT / "config/dcm/agent-runtime.json"
DEFAULT_FRAMES = ROOT / "output/laya-speed-20261106/vision-bench/frames"
DEFAULT_LABELS = ROOT / "docs/research/vision-bench/labels.json"
SAFETY_POLICY = ROOT / "config/safety/v1.yaml"
DEFAULT_POLICY = ROOT / "config/dcm/default_policy.json"
REQUIRED = "ICARUS_LLM_BASE_URL, ICARUS_LLM_API_KEY and ICARUS_LLM_MODEL"
BUILDING_FRAME, ROAD_FRAME = "f02", "f00"
BUILDING_QUESTION = "Is there a building visible in this image?"
TAKEOFF_REQUEST = "Take off to 3 meters."
DETAIL_CHARS = 300


class Skipped(Exception):
    """A check that could not run; the message is the reason."""


def summarise(results):
    """`(exit_status, verdict)` for `(name, ok, latency_ms)` results; `ok` None means skipped.

    Status 0 only when every check passed; any failure makes it 1. Skips never count as
    passes: with no failure but some skip the status is 3 and the verdict names what was
    not verified. (Status 2, missing settings, is `main`'s.)
    """
    failed = [name for name, ok, _ in results if ok is False]
    skipped = [name for name, ok, _ in results if ok is None]
    if failed:
        verdict = "FAIL " + ", ".join(failed)
        return 1, verdict + (f"; skipped (not verified): {', '.join(skipped)}" if skipped else "")
    if skipped:
        return 3, f"NO FAILURES, but skipped (not verified): {', '.join(skipped)}"
    return 0, "PASS"


def format_tokens(stats):
    """Token counts a runtime reported, or `n/a` when it reports none."""
    if not stats or stats.get("tokens_prompt") is None:
        return "tokens n/a"
    return (f"tokens prompt={stats['tokens_prompt']} cached={stats.get('tokens_cached') or 0} "
            f"out={stats.get('tokens_predicted')}")


def format_line(name, ok, latency_ms=None, tokens="", detail=""):
    """One result line: PASS/FAIL/SKIP, name, then latency, tokens and a short reason."""
    status = "SKIP" if ok is None else "PASS" if ok else "FAIL"
    if ok is None:
        return f"{status} {name} ({detail})"
    parts = [f"{status} {name}", f"{latency_ms} ms"]
    if tokens:
        parts.append(tokens)
    if detail:
        parts.append(detail)
    return "  ".join(parts)


def one_line(text, limit=DETAIL_CHARS):
    return " ".join(str(text).split())[:limit]


def find_frame(frames_dir, name):
    """The stored frame `name` (.jpg or .png) in `frames_dir`, or None."""
    for suffix in (".jpg", ".png"):
        path = Path(frames_dir) / f"{name}{suffix}"
        if path.is_file():
            return path
    return None


def building_label(labels_path, name):
    """Whether labelled frame `name` shows a building; raises Skipped when it cannot be read."""
    try:
        return bool(json.loads(Path(labels_path).read_text())[name]["building"])
    except (OSError, ValueError, KeyError, TypeError):
        raise Skipped(f"no building label for {name} in {labels_path}") from None


def box_inside(box, width, height):
    """True for a non-degenerate [x0, y0, x1, y1] that lies within a width x height image."""
    if not isinstance(box, (list, tuple)) or len(box) != 4:
        return False
    if not all(type(value) in (int, float) for value in box):
        return False
    x0, y0, x1, y1 = box
    return 0 <= x0 < x1 <= width and 0 <= y0 < y1 <= height


def first_call(message):
    """`(name, arguments)` of an assistant message's first tool call; ValueError if it has none."""
    calls = (message or {}).get("tool_calls") or []
    if not calls:
        text = one_line((message or {}).get("content") or "", 120)
        raise ValueError(f"no tool call in the reply (text: {text!r})" if text else "no tool call in the reply")
    function = calls[0].get("function") or {}
    arguments = function.get("arguments") or {}
    if isinstance(arguments, str):
        try:
            arguments = json.loads(arguments or "{}")
        except ValueError:
            raise ValueError(f"tool call arguments are not JSON: {one_line(arguments, 120)!r}") from None
    return function.get("name"), arguments


def mission_context():
    """The session constants a real mission's context holds, from the same sources, with no drone.

    `capture_context` takes motion limits from the vehicle's advertised capabilities (the
    guardrail policy in config/safety/v1.yaml) and the detector vocabulary from the client
    (the YOLO classes of the edge-vision profile); defaults come from the default policy.
    """
    import yaml

    from python.perception.classes import YOLO_CLASSES

    policy = yaml.safe_load(SAFETY_POLICY.read_text())
    motion, fence = policy["motion"], policy["geofence"]
    defaults = {key: value for key, value in json.loads(DEFAULT_POLICY.read_text()).items() if key != "schema"}
    return {"state": {"flight_phase": "FLIGHT_PHASE_ON_GROUND", "armed": False},
            "perception": {"fresh": True},
            "defaults": defaults,
            "motion_limits": {"maximum_horizontal_speed_mps": motion["max_horizontal_speed_m_s"],
                              "maximum_climb_speed_mps": motion["max_climb_speed_m_s"],
                              "maximum_descent_speed_mps": motion["max_descent_speed_m_s"],
                              "maximum_distance_from_home_m": fence["max_distance_from_home_m"]},
            "detector_classes": sorted(YOLO_CLASSES)}


def takeoff_request(context=None):
    """`(messages, tools)` exactly as the mission loop builds the first decision of a takeoff mission."""
    from python.dcm.agent_context import system_prompt
    from python.dcm.fly import session_tools, vehicle_state

    context = mission_context() if context is None else context
    tools = session_tools(context)
    user = TAKEOFF_REQUEST + "\n" + vehicle_state(context, with_defaults=True)
    return [{"role": "system", "content": system_prompt()}, {"role": "user", "content": user}], tools


class Checks:
    """The provider checks; each returns `(ok, tokens, detail)` or raises Skipped."""

    def __init__(self, runtime, options):
        self.runtime = runtime
        self.manifest = options.manifest
        self.frames = options.frames_dir
        self.labels = options.labels
        self.context = None
        self.tools = None
        self.call = None

    def provider_reachable(self):
        self.runtime.start()
        return True, "", ""

    def catalogue_takeoff(self):
        self.context = mission_context()
        messages, self.tools = takeoff_request(self.context)
        message = self.runtime.decide(messages, self.tools)
        tokens = format_tokens(self.runtime.last_stats)
        name, arguments = first_call(message)
        self.call = (name, arguments)
        detail = f"{name} {json.dumps(arguments)} ({len(self.tools)} tools)"
        return name == "takeoff", tokens, detail

    def validate_call(self):
        if self.call is None:
            raise Skipped("no call to validate; catalogue_takeoff did not return one")
        if self.call[0] != "takeoff":
            raise Skipped(f"the call was {self.call[0]}, not takeoff; nothing to validate")
        from python.dcm.agent_tools import validate_call

        validate_call(self.call[0], self.call[1], tools=self.tools,
                      detector_classes=self.context.get("detector_classes"))
        return True, "", ""

    def _frame(self, name):
        path = find_frame(self.frames, name)
        if path is None:
            raise Skipped(f"frames not found: {Path(self.frames) / name}.jpg or .png")
        return path

    def _condition(self, frame):
        from python.perception.vlm import inspect_image

        expected = building_label(self.labels, frame)
        path = self._frame(frame)
        result = inspect_image(path, BUILDING_QUESTION, self.manifest, structured=True,
                               result_type="boolean", server=self.runtime.vision())
        found = result.get("found")
        detail = f"{frame}: found={found} expected={expected}"
        if found is None:
            detail += f" ({result.get('error_code') or result.get('uncertainty') or 'no answer'})"
        return found is expected, "tokens n/a", detail

    def condition_true(self):
        return self._condition(BUILDING_FRAME)

    def condition_false(self):
        return self._condition(ROAD_FRAME)

    def pixel_box(self):
        from PIL import Image

        from python.perception.vlm import ground_description

        path = self._frame(BUILDING_FRAME)
        result = ground_description(path, "house", self.manifest, server=self.runtime.vision())
        with Image.open(path) as image:
            width, height = image.size
        box = result.get("pixel_box_xyxy")
        detail = f"{BUILDING_FRAME}: present={result.get('present')} box={box} image={width}x{height}"
        return bool(result.get("present")) and box_inside(box, width, height), "tokens n/a", detail


CHECK_NAMES = ("provider_reachable", "catalogue_takeoff", "validate_call",
               "condition_true", "condition_false", "pixel_box")


def run_check(name, check, echo=print):
    """Run one check and print its line; returns `(name, ok, latency_ms)`. Failures never propagate."""
    started = time.monotonic()
    ok, tokens, detail = None, "", ""
    try:
        ok, tokens, detail = check()
    except Skipped as reason:
        ok, detail = None, str(reason)
    except Exception as failure:  # noqa: BLE001 - any provider failure is this check's failure
        ok, detail = False, one_line(f"{type(failure).__name__}: {failure}")
        if isinstance(failure, RejectedRequest) and failure.detail.strip():
            # The provider's own reason (already key-stripped and size-limited), e.g. which schema keyword it refuses.
            detail += f" | provider said: {one_line(failure.detail)}"
    latency = round((time.monotonic() - started) * 1000)
    echo(format_line(name, ok, latency, tokens, detail))
    return name, ok, latency


def parse_args(argv=None):
    parser = argparse.ArgumentParser(
        description="Check a hosted model provider (settings from ICARUS_LLM_* / ICARUS_VLM_*) before flying with it.")
    parser.add_argument("--manifest", type=Path, default=DEFAULT_MANIFEST,
                        help="runtime manifest holding the cloud limits (default: %(default)s)")
    parser.add_argument("--frames-dir", type=Path, default=DEFAULT_FRAMES,
                        help="stored vision-bench frames, f00.jpg .. (default: %(default)s)")
    parser.add_argument("--labels", type=Path, default=DEFAULT_LABELS,
                        help="labels for those frames (default: %(default)s)")
    return parser.parse_args(argv)


def main(argv=None):
    options = parse_args(argv)
    try:
        runtime, record = make_runtime("cloud", options.manifest)
    except InvalidCloudSetting as failure:
        print(f"check_cloud_provider: {failure}", file=sys.stderr)
        return 2
    except MissingCloudSetting as failure:
        print(f"check_cloud_provider: {failure}; set {REQUIRED} "
              "(ICARUS_VLM_* default to the LLM values)", file=sys.stderr)
        return 2
    except (OSError, KeyError, ValueError) as failure:
        print(f"check_cloud_provider: cannot read the cloud settings from {options.manifest}: "
              f"{one_line(failure)}", file=sys.stderr)
        return 2
    print(f"Chat model {record['model']} via {record['provider_host']}; "
          f"camera model {record['vision_model']} via {record['vision_provider_host']} "
          f"(grounding {record['vision']['grounding_format']})")
    checks = Checks(runtime, options)
    results = []
    try:
        for name in CHECK_NAMES:
            results.append(run_check(name, getattr(checks, name)))
            if name == "provider_reachable" and results[-1][1] is not True:
                break  # nothing else can work; do not pile up the same failure
    finally:
        runtime.stop()
    status, verdict = summarise(results)
    print(f"RESULT {verdict}")
    return status


if __name__ == "__main__":
    sys.exit(main())

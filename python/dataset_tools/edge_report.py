"""Summarize a sealed edge-vision simulation episode without reading pixels."""
import argparse
import json
import re
from pathlib import Path


def _mean(values):
    return round(sum(values) / len(values), 3) if values else None


def _asks_count_of(text, noun):
    normalized = re.sub(r"\bno\.", "number", text.lower())
    direct = (r"\b(?:count|determine|identify|report)\b"
              r"[^.!?;,]{0,80}\b" + re.escape(noun) + r"\b")
    how_many = r"\bhow\s+many\s+" + re.escape(noun) + r"\b"
    return bool(re.search(direct, normalized) or re.search(how_many, normalized))


def _asks_person_detection(text):
    return bool(re.search(
        r"\bdetect\b[^.!?;,]{0,50}\b(?:people|person)\b", text.lower()))


def report(episode):
    episode = Path(episode)
    manifest = json.loads((episode / "manifest.json").read_text())
    scenario = json.loads((episode / "config" / "edge_people_building.json").read_text())
    events = [json.loads(line) for line in (episode / "events.jsonl").read_text().splitlines()]
    observer = []
    snapshot = manifest.get("edge_vision_snapshot")
    if snapshot:
        observer = [json.loads(line) for line in (episode / snapshot["path"]).read_text().splitlines()]
    detections = [item for item in observer
                  if item.get("schema") == "icarus.edge.yolo.v1"]
    count_detections = [item for item in detections
                        if item.get("camera", {}).get("count_eligible", True)
                        and "unique_person_count" in item]
    last = count_detections[-1] if count_detections else {}
    expected = scenario["ground_truth"]["unique_person_count"]
    yolo_latencies = [item["inference_latency_ms"] for item in detections
                      if item.get("inference_latency_ms") is not None]
    timestamps = [item["observed_at_unix_ms"] for item in detections]
    elapsed_s = (timestamps[-1] - timestamps[0]) / 1000 if len(timestamps) > 1 else 0
    vlm = [item["payload"] for item in events if item["kind"] == "semantic_visual_inspection"]
    decisions = [item["payload"] for item in events if item["kind"] == "dcm_decision"]
    safety = [item["payload"] for item in events if item["kind"] == "action_status"
              and "SAFETY" in str(item["payload"].get("state", ""))]
    score = manifest.get("score") or {}
    missions = score.get("missions") or []
    resources = missions[-1].get("resources") if missions else None
    mission = missions[-1] if missions else {}
    mission_text = str(mission.get("mission") or manifest.get("mission", "")).lower()
    actions = mission.get("executed", [])
    if not actions:
        actions = [
            {"action": item["proposal"]["action"],
             "arguments": item["proposal"].get("arguments", {}),
             "outcome": item.get("outcome"), "result": item.get("result")}
            for item in decisions
            if item.get("executed") and isinstance(item.get("proposal"), dict)
        ]
    requested_person_count = (_asks_count_of(mission_text, "people") or
                              _asks_count_of(mission_text, "person") or
                              _asks_person_detection(mission_text))
    requested_building_count = _asks_count_of(mission_text, "building") or _asks_count_of(mission_text, "buildings")
    person_detects = [item for item in actions if item.get("action") == "detect"
                      and item.get("outcome") == "SUCCEEDED"
                      and "person" in item.get("arguments", {}).get("classes", [])]
    # A scan intentionally records one bounded detection result per viewpoint.
    # Its mission measurement is the largest same-frame count observed at any
    # declared viewpoint, not the final viewpoint (which may face empty space).
    # This is the same conservative aggregation used by completion_report and
    # never sums cameras or frames.
    def person_count(item):
        value = (item.get("result") or {}).get("unique_person_count")
        return value if type(value) in (int, float) else -1

    best_person_detect = max(person_detects, key=person_count, default=None)
    action_count = ((best_person_detect.get("result") or {}).get(
        "unique_person_count") if best_person_detect else None)
    count_evidence = (((best_person_detect.get("result") or {}).get(
        "camera_evidence") or {}) if best_person_detect else {})
    count_camera = (max(
        count_evidence,
        key=lambda camera: count_evidence[camera].get(
            "max_simultaneous_persons", -1)) if count_evidence else
        last.get("camera", {}).get("id", "downward_brio"))
    # The bounded count window recorded by the executed detect action is the
    # mission measurement. The observer's cumulative IoU track IDs can split
    # one moving person into several IDs and are diagnostics only.
    observed = action_count if action_count is not None else last.get("unique_person_count")
    recorded_semantic = mission.get("semantic_completion")
    semantic = (dict(recorded_semantic) if isinstance(recorded_semantic, dict)
                else {"required": [], "satisfied": [], "reasons": []})
    if not isinstance(recorded_semantic, dict) and requested_person_count:
        semantic["required"].append("unique_person_count")
        if not person_detects:
            semantic["reasons"].append("requested person count has no successful detect evidence")
        else:
            if action_count == expected:
                semantic["satisfied"].append("unique_person_count")
            else:
                semantic["reasons"].append(f"person count is {action_count!r}; expected {expected}")
            if "orbit" in mission_text and actions.index(person_detects[-1]) < next(
                    (index for index, item in enumerate(actions)
                     if item.get("action") == "orbit" and item.get("outcome") == "SUCCEEDED"), len(actions)):
                semantic["reasons"].append("requested person count was collected before the requested orbit")
    if not isinstance(recorded_semantic, dict) and requested_building_count:
        semantic["required"].append("visual_building_count")
        semantic["reasons"].append(
            "stock YOLO11n has no building class; use qualitative current-camera VLM evidence")
    if not isinstance(recorded_semantic, dict):
        semantic["success"] = not semantic["reasons"]
    semantic_actions = {"detect", "inspect_scene", "assess_landing_zone"}
    flight_actions = [item for item in actions
                      if item.get("action") not in semantic_actions]
    terminal_flight = any(item.get("action") in ("return_home", "land") and
                          item.get("outcome") == "SUCCEEDED"
                          for item in flight_actions)
    flight_success = (bool(flight_actions) and terminal_flight and
                      all(item.get("outcome") == "SUCCEEDED"
                          for item in flight_actions))
    return {
        "schema": "icarus.edge.report.v1", "episode_id": manifest["episode_id"],
        "simulation_only": True, "flight_execution_success": flight_success,
        "mission_execution_success": (
            semantic["success"] and manifest.get("outcome") in ("completed", "passed")),
        "mission_outcome": manifest["outcome"],
        "semantic_completion": semantic,
        "person_count": {"expected": expected, "observed": observed,
                         "absolute_error": abs(expected - observed) if observed is not None else None},
        "object_tracking": {"frames": len(count_detections),
                            "final_track_count": len(last.get("tracks", [])),
                            "cumulative_person_track_ids": last.get("unique_person_count"),
                            "observer_events": len(observer),
                            "count_camera": count_camera},
        "vlm_qualitative_output": [{key: item.get(key) for key in ("question", "answer", "latency_ms")}
                                     for item in vlm],
        "safety_events": safety,
        "qwen_action_latency_ms": _mean([item["latency_ms"] for item in decisions
                                           if item.get("latency_ms") is not None]),
        "yolo": {"mean_inference_latency_ms": _mean(yolo_latencies),
                 "observed_fps": round((len(timestamps) - 1) / elapsed_s, 3) if elapsed_s else None,
                 "max_fps": 5,
                 "camera_feeds": sorted({item.get("camera", {}).get("id", "downward_brio")
                                         for item in detections})},
        "vlm_latency_ms": _mean([item["latency_ms"] for item in vlm if item.get("latency_ms") is not None]),
        "resources": resources,
        "limitations": "Simulation-only research evidence; no real-world readiness or unattended hardware approval.",
    }


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("episode", type=Path)
    parser.add_argument("--output", type=Path)
    args = parser.parse_args()
    rendered = json.dumps(report(args.episode), indent=2, sort_keys=True) + "\n"
    if args.output:
        args.output.write_text(rendered)
    print(rendered, end="")


if __name__ == "__main__":
    main()

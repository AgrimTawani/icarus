import json

from python.dataset_tools.edge_report import report


def test_edge_report_keeps_vision_evidence_and_resource_truth(tmp_path):
    (tmp_path / "config").mkdir()
    (tmp_path / "config/edge_people_building.json").write_text(json.dumps({
        "ground_truth": {"unique_person_count": 4}}))
    (tmp_path / "edge_vision.jsonl").write_text(json.dumps({
        "schema": "icarus.edge.yolo.v1", "unique_person_count": 3, "tracks": [{"track_id": 1}],
        "inference_latency_ms": 21, "observed_at_unix_ms": 1000}) + "\n")
    (tmp_path / "events.jsonl").write_text("\n".join(json.dumps(item) for item in [
        {"kind": "dcm_decision", "payload": {"latency_ms": 120}},
        {"kind": "semantic_visual_inspection", "payload": {"question": "people?", "answer": "Three people.", "latency_ms": 50}},
    ]))
    (tmp_path / "manifest.json").write_text(json.dumps({
        "episode_id": tmp_path.name, "outcome": "completed",
        "edge_vision_snapshot": {"path": "edge_vision.jsonl"},
        "score": {"missions": [{"resources": {"peak_vram_mib": 5000}}]},
    }))
    result = report(tmp_path)
    assert result["person_count"] == {"expected": 4, "observed": 3, "absolute_error": 1}
    assert result["resources"]["peak_vram_mib"] == 5000
    assert result["vlm_qualitative_output"][0]["answer"] == "Three people."
    assert result["object_tracking"]["count_camera"] == "downward_brio"


def test_edge_report_prefers_executed_count_window_and_recorded_completion(tmp_path):
    (tmp_path / "config").mkdir()
    (tmp_path / "config/edge_people_building.json").write_text(json.dumps({
        "ground_truth": {"unique_person_count": 4}}))
    (tmp_path / "edge_vision.jsonl").write_text(json.dumps({
        "schema": "icarus.edge.yolo.v1", "unique_person_count": 11,
        "tracks": [], "detections": [], "inference_latency_ms": 10,
        "observed_at_unix_ms": 1000, "camera": {
            "id": "downward_brio", "count_eligible": True}}) + "\n")
    semantic = {"required": ["building_visual_inspection", "unique_person_count"],
                "satisfied": ["unique_person_count"],
                "reasons": ["inspection missing"], "success": False}
    mission = {
        "mission": "inspect the building and count four people",
        "executed": [
            {"action": "takeoff", "arguments": {}, "outcome": "SUCCEEDED"},
            {"action": "detect", "arguments": {"classes": ["person"]},
             "outcome": "SUCCEEDED", "result": {
                 "unique_person_count": 4,
                 "camera_evidence": {
                     "downward_brio": {"max_simultaneous_persons": 0},
                     "forward_d435i": {"max_simultaneous_persons": 4}}}},
            {"action": "return_home", "arguments": {}, "outcome": "SUCCEEDED"}],
        "semantic_completion": semantic,
    }
    (tmp_path / "events.jsonl").write_text("")
    (tmp_path / "manifest.json").write_text(json.dumps({
        "episode_id": tmp_path.name, "outcome": "failed",
        "edge_vision_snapshot": {"path": "edge_vision.jsonl"},
        "score": {"missions": [mission]}}))
    result = report(tmp_path)
    assert result["person_count"]["observed"] == 4
    assert result["object_tracking"]["cumulative_person_track_ids"] == 11
    assert result["object_tracking"]["count_camera"] == "forward_d435i"
    assert result["semantic_completion"] == semantic
    assert result["flight_execution_success"] is True
    assert result["mission_execution_success"] is False


def test_edge_report_uses_maximum_declared_scan_viewpoint_not_last(tmp_path):
    (tmp_path / "config").mkdir()
    (tmp_path / "config/edge_people_building.json").write_text(json.dumps({
        "ground_truth": {"unique_person_count": 4}}))
    (tmp_path / "events.jsonl").write_text("")
    semantic = {"required": ["scan"], "satisfied": ["scan"],
                "reasons": [], "success": True}
    detects = []
    for count in (0, 4, 2, 0):
        detects.append({
            "action": "detect", "arguments": {"classes": ["person"]},
            "outcome": "SUCCEEDED", "result": {
                "unique_person_count": count,
                "camera_evidence": {
                    "downward_brio": {"max_simultaneous_persons": 0},
                    "forward_d435i": {"max_simultaneous_persons": count},
                },
            },
        })
    mission = {
        "mission": "scan and detect people after every step",
        "executed": [
            {"action": "takeoff", "arguments": {}, "outcome": "SUCCEEDED"},
            *detects,
            {"action": "return_home", "arguments": {},
             "outcome": "SUCCEEDED"},
        ],
        "semantic_completion": semantic,
    }
    (tmp_path / "manifest.json").write_text(json.dumps({
        "episode_id": tmp_path.name, "outcome": "completed",
        "score": {"missions": [mission]},
    }))
    result = report(tmp_path)
    assert result["person_count"] == {
        "expected": 4, "observed": 4, "absolute_error": 0}
    assert result["object_tracking"]["count_camera"] == "forward_d435i"


def test_failed_episode_cannot_be_reported_as_success_when_score_is_incomplete(tmp_path):
    (tmp_path / "config").mkdir()
    (tmp_path / "config/edge_people_building.json").write_text(json.dumps({
        "ground_truth": {"unique_person_count": 4}}))
    (tmp_path / "events.jsonl").write_text(json.dumps({
        "kind": "dcm_decision", "payload": {
            "executed": True, "proposal": {"action": "takeoff", "arguments": {}},
            "outcome": "SUCCEEDED", "latency_ms": 10}}) + "\n")
    (tmp_path / "manifest.json").write_text(json.dumps({
        "episode_id": tmp_path.name, "mission": "count the four people",
        "outcome": "failed", "score": {"status": "failed", "missions": []}}))
    result = report(tmp_path)
    assert result["flight_execution_success"] is False
    assert result["mission_execution_success"] is False
    assert "detect evidence" in result["semantic_completion"]["reasons"][0]

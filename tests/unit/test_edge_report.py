import json
from pathlib import Path

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
    assert result["object_tracking"]["count_camera"] == "downward_rgbd"

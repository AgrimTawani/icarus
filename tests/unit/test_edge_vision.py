import hashlib
import tempfile
import unittest
from pathlib import Path

from python.perception.edge_vision import RateLimiter, UniqueTrackCounter, normalize_yolo_result, select_device, summarize_observer_window, validate_edge_manifest, verify_edge_manifest_files
from python.perception.edge_service import CAMERA_FEEDS


class EdgeVisionTests(unittest.TestCase):
    def test_unique_tracks_do_not_recount_person(self):
        counter = UniqueTrackCounter()
        frame = [{"class": "person", "confidence": .9, "bbox_xyxy_px": [0, 0, 10, 20]}]
        counter.update(frame); counter.update([{**frame[0], "bbox_xyxy_px": [1, 0, 11, 20]}])
        self.assertEqual(len(counter.seen_person_track_ids), 1)

    def test_result_is_compact_and_pixel_free(self):
        with tempfile.TemporaryDirectory() as tmp:
            image = Path(tmp) / "frame.bin"; image.write_bytes(b"frame")
            result = normalize_yolo_result({"labels": ["person"], "scores": [.8], "boxes": [[1, 2, 3, 4]], "image_dimensions": [640, 480]}, image, UniqueTrackCounter())
            self.assertEqual(result["unique_person_count"], 1)
            self.assertEqual(result["source_frame_sha256"], hashlib.sha256(b"frame").hexdigest())
            self.assertFalse(result["flight_authority"])

    def test_ppm_camera_frame_is_a_supported_pillow_conversion_source(self):
        with tempfile.TemporaryDirectory() as temporary:
            source, destination = Path(temporary) / "frame.ppm", Path(temporary) / "frame.png"
            source.write_bytes(b"P6\n1 1\n255\n\xff\xff\xff")
            from PIL import Image
            Image.open(source).convert("RGB").save(destination)
            self.assertTrue(destination.is_file())

    def test_rate_limit(self):
        clock = iter([0, 1, 6]).__next__
        limiter = RateLimiter(clock=clock)
        limiter.admit()
        with self.assertRaises(RuntimeError):
            limiter.admit()
        limiter.admit()

    def test_cpu_fallback_is_explicit(self):
        self.assertEqual(select_device("auto", cuda_available=False), "cpu")
        with self.assertRaises(RuntimeError):
            select_device("cuda", cuda_available=False)

    def test_observer_round_robins_both_rgb_feeds_with_one_global_budget(self):
        self.assertEqual([feed["id"] for feed in CAMERA_FEEDS],
                         ["forward_rgbd", "downward_rgbd"])
        self.assertEqual(sum(feed["count_eligible"] for feed in CAMERA_FEEDS), 1)

    def test_count_window_never_sums_camera_totals(self):
        records = [
            {"schema": "icarus.edge.yolo.v1", "camera": {"id": "forward_rgbd"},
             "detections": [{"class": "person"}, {"class": "person"}, {"class": "person"}]},
            {"schema": "icarus.edge.yolo.v1", "camera": {"id": "downward_rgbd"},
             "detections": [{"class": "person"}, {"class": "person"}]},
        ]
        result = summarize_observer_window(records, ["person"])
        self.assertEqual(result["unique_person_count"], 3)
        self.assertEqual(result["camera_evidence"]["downward_rgbd"]["max_simultaneous_persons"], 2)

    def test_manifest_is_simulation_only_and_roles_are_pinned(self):
        manifest = {"profile": "edge-vision", "simulation_only": True, "llama_cpp_revision": "x", "artifacts": [
            {"role": role, "path": "/x", "sha256": "a" * 64, "repository": "r", "revision": "v", "runtime": "r"}
            for role in ("qwen_edge_primary", "yolo", "vlm")
        ]}
        self.assertTrue(validate_edge_manifest(manifest))

    def test_edge_module_has_no_flight_authority(self):
        source = Path(__file__).resolve().parents[2] / "python/perception/edge_vision.py"
        text = source.read_text().lower()
        self.assertNotIn("import mavlink", text)
        self.assertNotIn("actionservice", text)

    def test_checksum_verification_refuses_swapped_artifact(self):
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            qwen = root / "qwen.gguf"; yolo = root / "yolo.pt"; vlm = root / "vlm"
            vlm.mkdir(); weights = vlm / "weights.bin"
            for path in (qwen, yolo, weights): path.write_bytes(path.name.encode())
            entry = lambda role, path: {"role": role, "path": str(path),
                                        "sha256": hashlib.sha256(path.read_bytes()).hexdigest(),
                                        "repository": "r", "revision": "v", "runtime": "r"}
            manifest = {"profile": "edge-vision", "simulation_only": True, "llama_cpp_revision": "x",
                        "artifacts": [entry("qwen_edge_primary", qwen), entry("yolo", yolo),
                                      {"role": "vlm", "path": str(vlm), "sha256": "a" * 64,
                                       "repository": "r", "revision": "v", "runtime": "r"}]}
            manifest["artifacts"][-1]["file_sha256"] = {"weights.bin": hashlib.sha256(weights.read_bytes()).hexdigest()}
            self.assertTrue(verify_edge_manifest_files(manifest))
            yolo.write_bytes(b"swapped")
            with self.assertRaises(ValueError): verify_edge_manifest_files(manifest)


if __name__ == "__main__": unittest.main()

"""Pure-contract tests for the open-vocabulary detector boundary."""

import hashlib
import tempfile
import unittest
from pathlib import Path

from python.perception.vision import normalize_classes, summarize_detections


class VisionContractTests(unittest.TestCase):
    def test_classes_are_normalized_deduplicated_and_stable(self):
        self.assertEqual(normalize_classes([" Person ", "tree", "person"]),
                         ["person", "tree"])

    def test_invalid_or_excessive_class_requests_are_refused(self):
        for values in ([], [""], ["x" * 65], list(map(str, range(17)))):
            with self.subTest(values=values):
                with self.assertRaises(ValueError):
                    normalize_classes(values)

    def test_result_has_complete_counts_and_only_requested_classes(self):
        with tempfile.TemporaryDirectory() as temporary:
            image = Path(temporary) / "frame.ppm"
            image.write_bytes(b"P6\n1 1\n255\n\0\0\0")
            result = summarize_detections({
                "labels": ["person", "unknown", "tree"],
                "scores": [0.9, 0.8, 0.7],
                "boxes": [[1, 2, 3, 4], [0, 0, 1, 1], [5, 6, 7, 8]],
            }, ["person", "tree", "building"], image, {"repository": "test"})
            self.assertEqual(result["counts"],
                             {"person": 1, "tree": 1, "building": 0})
            self.assertEqual([item["class"] for item in result["detections"]],
                             ["person", "tree"])
            self.assertEqual(result["image"]["sha256"],
                             hashlib.sha256(image.read_bytes()).hexdigest())


if __name__ == "__main__":
    unittest.main()

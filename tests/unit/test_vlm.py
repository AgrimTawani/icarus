"""Pure contract tests for the qualitative VLM boundary."""

import subprocess
import unittest
from pathlib import Path
from tempfile import TemporaryDirectory
from types import SimpleNamespace
from unittest.mock import patch

from python.perception.vlm import (
    MAX_QUESTION_CHARS,
    extract_answer,
    inspect_image,
    normalize_question,
)


class VlmContractTests(unittest.TestCase):
    def test_question_is_compact_and_bounded(self):
        self.assertEqual(normalize_question("  count   visible trees "),
                         "count visible trees")

    def test_invalid_question_is_rejected(self):
        for value in ("", "   ", None, "x" * (MAX_QUESTION_CHARS + 1)):
            with self.subTest(value=value):
                with self.assertRaises(ValueError):
                    normalize_question(value)

    def test_llama_footer_is_not_treated_as_answer(self):
        stdout = "banner\nA grey building and grass.\n[ Prompt: 1.0 t/s ]\n>\nExiting...\n"
        self.assertEqual(extract_answer(stdout), "A grey building and grass.")

    def test_wrapped_answer_after_echoed_question_is_reassembled(self):
        output = ("> system prompt\nQuestion: Is the building damaged?\n"
                  "The building is rectangular and no damage\n"
                  "or wear is visible.\n[ Prompt: 1.0 t/s ]\n")
        self.assertEqual(
            extract_answer(output),
            "The building is rectangular and no damage or wear is visible.")

    def test_cpu_mode_disables_projector_and_operation_offload(self):
        with TemporaryDirectory() as temporary:
            root = Path(temporary)
            image = root / "frame.ppm"
            binary = root / "llama-cli"
            image.write_bytes(b"P6\n1 1\n255\n\0\0\0")
            binary.write_text("")
            artifact = {
                "path": "/model.gguf", "mmproj_path": "/mmproj.gguf",
                "family": "qwen2-vl", "quantization": "q4_k_m",
                "sha256": "model", "mmproj_sha256": "projector",
            }
            completed = SimpleNamespace(
                returncode=0,
                stdout="A grey building.\n[ Prompt: 1.0 t/s ]\n",
                stderr="",
            )
            with patch("python.perception.vlm.load_vlm", return_value=artifact), \
                    patch("python.perception.vlm.subprocess.run",
                          return_value=completed) as run:
                inspect_image(image, "Describe the building", root / "manifest",
                              binary=binary, gpu_layers=0)
            command = run.call_args.args[0]
            self.assertIs(run.call_args.kwargs["stdin"], subprocess.DEVNULL)
            self.assertTrue(run.call_args.kwargs["start_new_session"])
            self.assertIn("--device", command)
            self.assertEqual(command[command.index("--device") + 1], "none")
            self.assertIn("--no-op-offload", command)
            self.assertIn("--no-mmproj-offload", command)
            self.assertEqual(command[command.index("--mmproj-device") + 1],
                             "none")


if __name__ == "__main__":
    unittest.main()

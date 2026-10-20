"""Pure contract tests for the qualitative VLM boundary."""

import subprocess
import unittest
from pathlib import Path
from tempfile import TemporaryDirectory
from types import SimpleNamespace
from typing import ClassVar
from unittest.mock import patch

from python.perception.vlm import (
    MAX_QUESTION_CHARS,
    _GroundingServer,
    extract_answer,
    ground_description,
    inspect_image,
    inspection_prompt,
    normalize_question,
    presence_answer_is_yes,
)


class VlmContractTests(unittest.TestCase):
    def test_scene_prompt_distinguishes_camera_view_from_named_subject(self):
        prompt = inspection_prompt("What is visible ahead?", True, "description")
        self.assertIn("image is the camera view", prompt)
        self.assertIn("does not require the drone itself to appear", prompt)
        self.assertIn("general scene question", prompt)
        self.assertIn("attribute of an unseen subject is not answerable", prompt)
        self.assertIn("do not invent", prompt)
        self.assertTrue(prompt.endswith("Question: What is visible ahead?"))

    def test_scene_prompt_keeps_negative_and_count_contracts_distinct(self):
        boolean = inspection_prompt("Is the subject visible?", True, "boolean")
        count = inspection_prompt("How many are visible?", True, "count")
        self.assertIn('"found":true or false or null', boolean)
        self.assertIn("False is a usable negative answer", boolean)
        self.assertIn("individually visible instances", count)
        self.assertIn("never infer invisible instances", count)

    def test_question_is_compact_and_bounded(self):
        self.assertEqual(normalize_question("  count   visible trees "),
                         "count visible trees")

    def test_invalid_question_is_rejected(self):
        for value in ("", "   ", None, "x" * (MAX_QUESTION_CHARS + 1)):
            with self.subTest(value=value), self.assertRaises(ValueError):
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
            self.assertEqual(command[command.index("--threads") + 1], "4")
            self.assertEqual(command[command.index("--threads-batch") + 1], "4")

    def test_inspection_budgets_image_text_and_answer_together(self):
        with TemporaryDirectory() as temporary:
            root = Path(temporary)
            image, binary = root / "frame.ppm", root / "llama-cli"
            image.write_bytes(b"P6\n1 1\n255\n\0\0\0")
            binary.write_text("")
            artifact = {"path": "/model.gguf", "mmproj_path": "/mmproj.gguf"}
            completed = SimpleNamespace(returncode=0, stdout="Visible building.", stderr="")
            contexts = []
            for question in ("Describe the scene", "界" * MAX_QUESTION_CHARS):
                with patch("python.perception.vlm.load_vlm", return_value=artifact), \
                        patch("python.perception.vlm.subprocess.run", return_value=completed) as run:
                    result = inspect_image(image, question, root / "manifest", binary=binary)
                command = run.call_args.args[0]
                context = int(command[command.index("-c") + 1])
                image_budget = int(command[command.index("--image-max-tokens") + 1])
                answer = int(command[command.index("-n") + 1])
                prompt = command[command.index("-p") + 1]
                self.assertGreater(context, 1267 + answer)  # observed failing request
                self.assertGreaterEqual(context, image_budget + len(prompt.encode("utf-8")) + answer)
                self.assertEqual(result["runtime"]["context_tokens"], context)
                contexts.append(context)
            self.assertGreater(contexts[1], contexts[0])


class _FakeServer:
    """Stands in for the short-lived llama-server; records every question."""

    answers: ClassVar[list] = []
    questions: ClassVar[list] = []

    def __init__(self, *args, **kwargs):
        pass

    def __enter__(self):
        return self

    def __exit__(self, *_):
        return None

    def complete(self, image_b64, question, prefill, n_predict):
        _FakeServer.questions.append((question, prefill, n_predict))
        return _FakeServer.answers.pop(0)


class VlmGroundingTests(unittest.TestCase):
    ARTIFACT: ClassVar[dict] = {"path": "/model.gguf", "mmproj_path": "/mmproj.gguf",
                "family": "qwen2-vl", "quantization": "q4_k_m",
                "sha256": "model", "mmproj_sha256": "projector"}

    def _ground(self, answers, artifact=None, view_context=None):
        _FakeServer.answers = list(answers)
        _FakeServer.questions = []
        with TemporaryDirectory() as temporary:
            root = Path(temporary)
            image = root / "frame.ppm"
            binary = root / "llama-server"
            image.write_bytes(b"P6\n1 1\n255\n\0\0\0")
            binary.write_text("")
            with patch("python.perception.vlm.load_vlm",
                       return_value=artifact or self.ARTIFACT), \
                    patch("python.perception.vlm._GroundingServer", _FakeServer):
                return ground_description(image, "building behind the people",
                                          root / "manifest", binary=binary, view_context=view_context)

    def test_navigation_context_is_retained_without_becoming_a_visual_constraint(self):
        context = {'camera': 'forward_d435i', 'aircraft_heading_deg': 5.0}
        result = self._ground(['Yes', '300,200),(700,800)'], view_context=context)
        self.assertEqual(result['view_context'], context)
        for question, _, _ in _FakeServer.questions:
            self.assertNotIn('5.0 degrees', question)
            self.assertNotIn('forward_d435i', question)
            self.assertIn('building behind the people', question)

    def test_presence_gate_then_forced_native_grounding_box(self):
        result = self._ground(["Yes.", "319,402),(648,859)"])
        self.assertTrue(result["present"])
        self.assertEqual(result["grounding_text"], "(319,402),(648,859)")
        self.assertFalse(result["flight_authority"])
        (presence, presence_prefill, _), (_question, prefill, _) = _FakeServer.questions
        self.assertIn("building behind the people", presence)
        self.assertEqual(presence_prefill, "")
        self.assertTrue(prefill.startswith("<|object_ref_start|>the building behind"))
        self.assertTrue(prefill.endswith("<|box_start|>("))

    def test_no_presence_skips_grounding(self):
        result = self._ground(["No"])
        self.assertFalse(result["present"])
        self.assertIsNone(result["grounding_text"])
        self.assertEqual(len(_FakeServer.questions), 1)

    def test_unknown_presence_is_not_reported_as_target_absence(self):
        from python.perception.vlm import VlmInferenceError
        for answer in ('', 'Maybe', 'I cannot tell'):
            with self.subTest(answer=answer), self.assertRaises(VlmInferenceError):
                self._ground([answer])

    def test_non_qwen2_vl_artifact_is_refused(self):
        with self.assertRaises(ValueError):
            self._ground(["Yes."], artifact=dict(self.ARTIFACT, family="smolvlm"))

    def test_presence_parsing_is_strict_about_the_first_word(self):
        self.assertTrue(presence_answer_is_yes(" Yes, a building."))
        for answer in ("No.", "", "Maybe yes", "yesterday"):
            with self.subTest(answer=answer):
                self.assertFalse(presence_answer_is_yes(answer))

    def test_server_is_loopback_only_and_cpu_only(self):
        server = _GroundingServer(Path("/bin/llama-server"), self.ARTIFACT, 0, 0.0)
        command = server.command
        self.assertEqual(command[command.index("--host") + 1], "127.0.0.1")
        self.assertEqual(command[command.index("--device") + 1], "none")
        self.assertEqual(command[command.index("--mmproj-device") + 1], "none")
        self.assertIn("--no-mmproj-offload", command)


if __name__ == "__main__":
    unittest.main()

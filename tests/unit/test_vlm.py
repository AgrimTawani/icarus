"""Pure contract tests for the qualitative VLM boundary."""

import unittest

from python.perception.vlm import MAX_QUESTION_CHARS, extract_answer, normalize_question


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


if __name__ == "__main__":
    unittest.main()

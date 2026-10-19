"""Constrained JSON generation without feeding assistant-template tokens to it."""

import importlib.util
import json
import shutil
from pathlib import Path

import pytest

from python.perception.vlm import COUNT_SCHEMA, VlmInferenceError, measurement_grammar

ROOT = Path(__file__).resolve().parents[2]


def test_stored_grammars_match_pinned_converter_without_loading_a_model():
    if not shutil.which("g++") or not (ROOT / "third_party/llama.cpp/build/bin/libllama-common.so").is_file():
        pytest.skip("offline grammar converter requires the pinned common library and C++ compiler")
    spec = importlib.util.spec_from_file_location(
        "generate_measurement_grammars", ROOT / "scripts/autonomy/generate_vlm_grammars.py")
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    stored = json.loads((ROOT / "python/perception/vlm_grammars.json").read_text())
    assert module.generate() == stored


@pytest.mark.parametrize("kind", ["boolean", "count"])
def test_measurement_grammar_is_complete_and_constrains_explanation(kind):
    grammar = measurement_grammar(kind)
    assert 'root ::= "{"' in grammar
    assert 'char{0,300}' in grammar
    assert 'null ::= "null"' in grammar
    assert ('found ::= boolean | null' if kind == "boolean" else 'count ::= count-0 | null') in grammar


def test_schema_drift_is_rejected_instead_of_silently_using_obsolete_grammar(monkeypatch):
    monkeypatch.setitem(COUNT_SCHEMA, "description", "changed measurement contract")
    with pytest.raises(VlmInferenceError) as error:
        measurement_grammar("count")
    assert error.value.code == "grammar_contract_mismatch"


def test_unknown_measurement_type_has_no_free_text_fallback():
    with pytest.raises(ValueError):
        measurement_grammar("arbitrary_number")

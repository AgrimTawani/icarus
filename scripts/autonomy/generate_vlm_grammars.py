#!/usr/bin/env python3
"""Regenerate constrained measurement grammars with the installed pinned converter.

Offline development utility only: compiles a tiny converter, reads no model,
launches no inference, and writes the schema-digested grammar artifact.
"""

import hashlib
import json
import subprocess
import sys
import tempfile
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT))

from python.perception.vlm import CONDITION_SCHEMA, COUNT_SCHEMA

CONVERTER = r'''
#include "json-schema-to-grammar.h"
#include <iostream>
#include <iterator>
int main() {
    std::string schema((std::istreambuf_iterator<char>(std::cin)), {});
    std::cout << json_schema_to_grammar(common_json::parse(schema), true);
}
'''


def generate():
    checkout = ROOT / "third_party/llama.cpp"
    library = checkout / "build/bin"
    artifact = {"generator": {
        "converter": "pinned llama.cpp json_schema_to_grammar(force_gbnf=true)",
        "source_sha256": hashlib.sha256(
            (checkout / "common/json-schema-to-grammar.cpp").read_bytes()).hexdigest(),
    }}
    with tempfile.TemporaryDirectory(prefix="icarus-grammar-") as directory:
        source, executable = Path(directory) / "convert.cpp", Path(directory) / "convert"
        source.write_text(CONVERTER)
        subprocess.run(["g++", "-std=c++17", "-I", str(checkout / "common"),
                        str(source), "-L", str(library), f"-Wl,-rpath,{library}",
                        "-lllama-common", "-o", str(executable)], check=True)
        for kind, schema in (("boolean", CONDITION_SCHEMA), ("count", COUNT_SCHEMA)):
            encoded = json.dumps(schema, sort_keys=True, separators=(",", ":"))
            grammar = subprocess.run([str(executable)], input=json.dumps(schema),
                                     text=True, capture_output=True, check=True).stdout
            artifact[kind] = {"schema_sha256": hashlib.sha256(encoded.encode()).hexdigest(),
                              "grammar": grammar}
    return artifact


if __name__ == "__main__":
    (ROOT / "python/perception/vlm_grammars.json").write_text(
        json.dumps(generate(), indent=2) + "\n")

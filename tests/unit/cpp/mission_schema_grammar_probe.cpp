// No model or inference: validate example bytes using llama.cpp's native schema
// converter and grammar matcher, including the converter's property ordering.
#include "json-schema-to-grammar.h"
#include "llama-grammar.h"
#include <fstream>
#include <iostream>
#include <iterator>
#include <string>
int main(int argc, char** argv) {
  if (argc != 3) return 2;
  std::ifstream schema_file(argv[1]);
  const auto schema = common_json::parse(std::string(std::istreambuf_iterator<char>(schema_file), {}));
  const auto source = json_schema_to_grammar(schema, true);
  std::ifstream examples(argv[2]);
  std::string line;
  while (std::getline(examples, line)) {
    auto* grammar = llama_grammar_init_impl(nullptr, source.c_str(), "root",
                                            false, nullptr, 0, nullptr, 0);
    if (!grammar) return 3;
    bool accepted = false;
    try {
      llama_grammar_accept_str(*grammar, line);
      for (const auto& stack : llama_grammar_get_stacks(grammar)) {
        if (stack.empty()) accepted = true;
      }
    } catch (const std::exception&) { accepted = false; }
    llama_grammar_free_impl(grammar);
    std::cout << (accepted ? "accepted" : "rejected") << '\n';
  }
}

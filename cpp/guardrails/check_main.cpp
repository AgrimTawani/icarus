// Validate one arbitrary command against the real safety policy and a given
// state, independent of any recorded episode.
//
// This exists because offline model evaluation could previously only check
// that a proposal was well-formed JSON with in-bounds arguments; it never
// asked whether the flight safety policy would actually accept the action.
// A proposal can pass the model contract's bounds check and still be
// something the guardrails reject outright, such as a takeoff while already
// airborne. This binary is how python/dcm/evaluate.py finds out.
//
// Reads one JSON object from stdin: {"command", "state", "authorized",
// "now_unix_ms"}, using the same message shapes as an episode's
// guardrail_validation payload. Writes the ValidationResult as JSON to
// stdout and always exits 0 when the input parsed; a rejection is data, not
// a process failure. Exit code is nonzero only for a usage or parse error.
#include <cstdint>
#include <iostream>
#include <sstream>
#include <stdexcept>
#include <string>

#include <google/protobuf/struct.pb.h>
#include <google/protobuf/util/json_util.h>

#include "guardrails/guardrails.hpp"

namespace {

google::protobuf::Value Field(const google::protobuf::Value& value,
                              const std::string& name) {
  return value.struct_value().fields().at(name);
}

template <typename Message>
Message Parse(const google::protobuf::Value& value) {
  std::string json;
  if (!google::protobuf::util::MessageToJsonString(value, &json).ok()) {
    throw std::runtime_error("unable to encode input message");
  }
  Message message;
  if (!google::protobuf::util::JsonStringToMessage(json, &message).ok()) {
    throw std::runtime_error("unable to decode input message");
  }
  return message;
}

std::int64_t AsInt64(const google::protobuf::Value& value) {
  return value.kind_case() == google::protobuf::Value::kStringValue
             ? std::stoll(value.string_value())
             : static_cast<std::int64_t>(value.number_value());
}

}  // namespace

int main(int argc, char** argv) {
  if (argc != 2) {
    std::cerr << "Usage: icarus-check-guardrail POLICY < request.json\n";
    return 2;
  }
  try {
    const auto policy = icarus::guardrails::SafetyPolicy::Load(argv[1]);
    const icarus::guardrails::Guardrails guardrails(policy);

    std::ostringstream buffer;
    buffer << std::cin.rdbuf();
    google::protobuf::Value request;
    if (!google::protobuf::util::JsonStringToMessage(buffer.str(), &request)
             .ok()) {
      throw std::runtime_error("invalid request JSON on stdin");
    }
    const auto command = Parse<icarus::v1::ActionCommand>(Field(request, "command"));
    const auto state = Parse<icarus::v1::DroneState>(Field(request, "state"));
    const bool authorized = Field(request, "authorized").bool_value();
    const std::int64_t now = AsInt64(Field(request, "now_unix_ms"));

    const auto result = guardrails.Validate(command, state, authorized, now);
    std::string json;
    if (!google::protobuf::util::MessageToJsonString(result, &json).ok()) {
      throw std::runtime_error("unable to encode validation result");
    }
    std::cout << json << '\n';
  } catch (const std::exception& error) {
    std::cerr << "icarus-check-guardrail FAILED: " << error.what() << '\n';
    return 1;
  }
  return 0;
}

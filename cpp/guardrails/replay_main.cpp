#include <fstream>
#include <iostream>
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
    throw std::runtime_error("unable to encode replay message");
  }
  Message message;
  if (!google::protobuf::util::JsonStringToMessage(json, &message).ok()) {
    throw std::runtime_error("unable to decode replay message");
  }
  return message;
}

}  // namespace

int main(int argc, char** argv) {
  if (argc != 3) {
    std::cerr << "Usage: icarus-replay-guardrails POLICY EVENTS_JSONL\n";
    return 2;
  }
  try {
    const auto policy = icarus::guardrails::SafetyPolicy::Load(argv[1]);
    const icarus::guardrails::Guardrails guardrails(policy);
    std::ifstream events(argv[2]);
    if (!events) throw std::runtime_error("unable to open episode stream");
    std::string line;
    std::size_t checks = 0;
    while (std::getline(events, line)) {
      google::protobuf::Value event;
      if (!google::protobuf::util::JsonStringToMessage(line, &event).ok()) {
        throw std::runtime_error("invalid episode JSON");
      }
      if (Field(event, "kind").string_value() != "guardrail_validation") continue;
      const auto& payload = Field(event, "payload");
      const auto command = Parse<icarus::v1::ActionCommand>(Field(payload, "command"));
      const auto state = Parse<icarus::v1::DroneState>(Field(payload, "state"));
      const auto expected = Parse<icarus::v1::ValidationResult>(Field(payload, "result"));
      const auto& time_value = Field(payload, "evaluated_at_unix_ms");
      const std::int64_t now = time_value.kind_case() == google::protobuf::Value::kStringValue
                                   ? std::stoll(time_value.string_value())
                                   : static_cast<std::int64_t>(time_value.number_value());
      const bool authorized = Field(payload, "authorized").bool_value();
      const auto actual = guardrails.Validate(command, state, authorized, now);
      if (actual.valid() != expected.valid() ||
          actual.reason_code() != expected.reason_code() ||
          actual.message() != expected.message()) {
        throw std::runtime_error("guardrail decision mismatch at validation " +
                                 std::to_string(checks + 1) + ": recorded=" +
                                 expected.message() + " replayed=" + actual.message());
      }
      ++checks;
    }
    if (checks == 0) throw std::runtime_error("no guardrail decisions to replay");
    std::cout << "Guardrail replay passed: " << checks << " decisions\n";
  } catch (const std::exception& error) {
    std::cerr << "Guardrail replay FAILED: " << error.what() << '\n';
    return 1;
  }
  return 0;
}

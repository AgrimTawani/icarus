#pragma once

#include <cstdint>

#include "guardrails/safety_policy.hpp"
#include "icarus/v1/action.pb.h"

namespace icarus::guardrails {

class Guardrails {
 public:
  explicit Guardrails(SafetyPolicy policy);

  [[nodiscard]] v1::ValidationResult Validate(
      const v1::ActionCommand& command, const v1::DroneState& state,
      bool lease_authorized, std::int64_t now_unix_ms) const;
  [[nodiscard]] const SafetyPolicy& policy() const { return policy_; }

 private:
  [[nodiscard]] v1::ValidationResult ValidateContext(
      const v1::CommandContext& context, const v1::DroneState& state,
      bool lease_authorized, std::int64_t now_unix_ms) const;
  [[nodiscard]] v1::ValidationResult ValidatePosition(
      const v1::Position& position, const v1::DroneState& state) const;
  [[nodiscard]] v1::ValidationResult ValidateLimits(
      const v1::ActionLimits& limits, const v1::DroneState& state) const;
  [[nodiscard]] static v1::ValidationResult Accept(
      std::uint64_t sequence, std::string message = "action is valid");
  [[nodiscard]] static v1::ValidationResult Reject(
      v1::ReasonCode code, std::string message, std::uint64_t sequence);
  [[nodiscard]] static const v1::CommandContext* ContextOf(
      const v1::ActionCommand& command);

  SafetyPolicy policy_;
};

}  // namespace icarus::guardrails

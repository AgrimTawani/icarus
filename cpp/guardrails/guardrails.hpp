#pragma once

#include <cstdint>
#include <optional>
#include <utility>

#include "guardrails/safety_policy.hpp"
#include "icarus/v1/action.pb.h"

namespace icarus::guardrails {

// Closer than this, "look from here toward that point" has no stable heading.
inline constexpr double kMinimumFacePointDistanceM = 0.5;
inline constexpr double kMaximumOrbitRevolutions = 10.0;
inline constexpr std::uint32_t kMaximumOrbitViewpoints = 24;
inline constexpr std::uint32_t kMaximumViewpointDwellMs = 30'000;

// Horizontal (north, east) metres from `from` to `to` when both are local NED
// or both geographic; nullopt for mixed or empty positions.
[[nodiscard]] std::optional<std::pair<double, double>> HorizontalOffsetM(
    const v1::Position& from, const v1::Position& to);

// Measured local down coordinate of home. Requires fresh coherent local and
// above-home telemetry; never assumes the EKF origin is the home altitude.
[[nodiscard]] std::optional<double> LocalHomeDownM(const v1::DroneState& state);
[[nodiscard]] std::optional<v1::LocalPositionNed> LocalHomePosition(
    const v1::DroneState& state);
[[nodiscard]] std::optional<v1::Position> ResolveLocalAltitude(
    const v1::Position& target, const v1::DroneState& state);
[[nodiscard]] std::optional<v1::Position> OrbitAltitudePosition(
    const v1::OrbitRequest& orbit, const v1::DroneState& state);

// A passive hold never arms or sends mode/position commands.
[[nodiscard]] bool IsPassiveGroundHoldState(const v1::DroneState& state);

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
      const v1::Position& position, const v1::DroneState& state,
      double maximum_distance_from_home_m) const;
  [[nodiscard]] v1::ValidationResult ValidateHeading(
      const v1::HeadingPolicy& heading, const v1::Position& anchor,
      const v1::DroneState& state, bool orbit) const;
  [[nodiscard]] v1::ValidationResult ValidateOrbit(
      const v1::OrbitRequest& orbit, const v1::DroneState& state) const;
  [[nodiscard]] v1::ValidationResult ValidateLimits(
      const v1::ActionLimits& limits, const v1::DroneState& state) const;
  [[nodiscard]] double FenceFor(v1::TargetSource source) const;
  [[nodiscard]] static v1::ValidationResult Accept(
      std::uint64_t sequence, std::string message = "action is valid");
  [[nodiscard]] static v1::ValidationResult Reject(
      v1::ReasonCode code, std::string message, std::uint64_t sequence);
  [[nodiscard]] static const v1::CommandContext* ContextOf(
      const v1::ActionCommand& command);

  SafetyPolicy policy_;
};

}  // namespace icarus::guardrails

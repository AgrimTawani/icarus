#include "guardrails/guardrails.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>
#include <utility>

#include <yaml-cpp/yaml.h>

namespace icarus::guardrails {
namespace {

constexpr double kEarthRadiusM = 6'371'000.0;
constexpr double kPi = 3.14159265358979323846;

double DegreesToRadians(double degrees) { return degrees * kPi / 180.0; }

double HorizontalDistanceM(const v1::GeoPosition& first,
                           const v1::GeoPosition& second) {
  const double latitude_delta =
      DegreesToRadians(second.latitude_deg() - first.latitude_deg());
  const double longitude_delta =
      DegreesToRadians(second.longitude_deg() - first.longitude_deg());
  const double latitude_a = DegreesToRadians(first.latitude_deg());
  const double latitude_b = DegreesToRadians(second.latitude_deg());
  const double haversine = std::sin(latitude_delta / 2.0) *
                               std::sin(latitude_delta / 2.0) +
                           std::cos(latitude_a) * std::cos(latitude_b) *
                               std::sin(longitude_delta / 2.0) *
                               std::sin(longitude_delta / 2.0);
  return 2.0 * kEarthRadiusM * std::asin(std::sqrt(haversine));
}

bool IsNavigationCommand(const v1::ActionCommand& command) {
  return command.has_goto_() || command.has_execute_route() ||
         command.has_orbit();
}

bool IsAirborneAction(const v1::ActionCommand& command) {
  return command.has_takeoff() || command.has_goto_() ||
         command.has_execute_route() ||
         command.has_hold() || command.has_return_home() ||
         command.has_land() || command.has_orbit();
}

}  // namespace

SafetyPolicy SafetyPolicy::Load(const std::filesystem::path& path) {
  const auto root = YAML::LoadFile(path.string());
  SafetyPolicy policy;
  policy.schema_version = root["schema_version"].as<std::string>();
  policy.profile = root["profile"].as<std::string>();
  policy.maximum_altitude_m = root["geofence"]["max_altitude_m"].as<double>();
  policy.maximum_distance_from_home_m =
      root["geofence"]["max_distance_from_home_m"].as<double>();
  policy.maximum_horizontal_speed_mps =
      root["motion"]["max_horizontal_speed_m_s"].as<double>();
  policy.maximum_climb_speed_mps =
      root["motion"]["max_climb_speed_m_s"].as<double>();
  policy.maximum_descent_speed_mps =
      root["motion"]["max_descent_speed_m_s"].as<double>();
  policy.minimum_takeoff_battery_percent =
      root["battery"]["minimum_takeoff_percent"].as<double>();
  policy.rtl_battery_percent = root["battery"]["rtl_percent"].as<double>();
  policy.land_battery_percent = root["battery"]["land_percent"].as<double>();
  policy.maximum_state_age_ms =
      root["state_health"]["max_state_age_ms"].as<std::uint32_t>();
  policy.stale_state_rtl_delay_ms = static_cast<std::uint32_t>(
      root["failure_actions"]["state_stale_rtl_delay_s"].as<double>() *
      1000.0);
  policy.maximum_perception_age_ms =
      root["perception"]["maximum_age_ms"].as<std::uint32_t>();
  policy.minimum_obstacle_clearance_m =
      root["perception"]["minimum_clearance_m"].as<double>();
  policy.emergency_stop_distance_m =
      root["perception"]["emergency_stop_distance_m"].as<double>();
  policy.require_estimator_healthy =
      root["state_health"]["require_ekf_healthy"].as<bool>();
  policy.require_position_for_navigation =
      root["state_health"]["require_position_for_navigation"].as<bool>();
  policy.require_home_for_takeoff =
      root["state_health"]["require_home_for_takeoff"].as<bool>();
  policy.manual_operator_has_priority =
      root["manual_override"]["operator_has_priority"].as<bool>();
  if (policy.schema_version != "icarus.safety.v1" ||
      policy.maximum_altitude_m <= 0.0 ||
      policy.maximum_distance_from_home_m <= 0.0 ||
      policy.maximum_horizontal_speed_mps <= 0.0 ||
      policy.maximum_state_age_ms == 0 ||
      policy.maximum_perception_age_ms == 0 ||
      policy.minimum_obstacle_clearance_m <= 0.0 ||
      policy.emergency_stop_distance_m < policy.minimum_obstacle_clearance_m ||
      policy.land_battery_percent <= 0.0 ||
      policy.rtl_battery_percent <= policy.land_battery_percent) {
    throw std::runtime_error("invalid or unsupported safety policy");
  }
  return policy;
}

Guardrails::Guardrails(SafetyPolicy policy) : policy_(std::move(policy)) {}

v1::ValidationResult Guardrails::Validate(
    const v1::ActionCommand& command, const v1::DroneState& state,
    bool lease_authorized, std::int64_t now_unix_ms) const {
  const auto* context = ContextOf(command);
  if (context == nullptr) {
    return Reject(v1::REASON_CODE_INVALID_ARGUMENT,
                  "exactly one supported action is required", state.sequence());
  }
  auto common = ValidateContext(*context, state, lease_authorized, now_unix_ms);
  if (!common.valid()) return common;

  if (state.manual_override_active()) {
    return Reject(v1::REASON_CODE_MANUAL_OVERRIDE,
                  "manual operator currently owns flight authority",
                  state.sequence());
  }
  if (IsAirborneAction(command) && !state.armed()) {
    return Reject(v1::REASON_CODE_NOT_ARMED, "aircraft is not armed",
                  state.sequence());
  }
  if (IsNavigationCommand(command) && policy_.require_position_for_navigation &&
      (!state.has_estimator() ||
       !state.estimator().horizontal_position_valid())) {
    return Reject(v1::REASON_CODE_POSITION_UNAVAILABLE,
                  "navigation requires a valid horizontal position",
                  state.sequence());
  }

  if (command.has_arm()) {
    if (state.armed()) {
      return Reject(v1::REASON_CODE_ALREADY_ARMED, "aircraft is already armed",
                    state.sequence());
    }
    if (!state.landed()) {
      return Reject(v1::REASON_CODE_NOT_LANDED,
                    "arming requires a confirmed landed state", state.sequence());
    }
    if (state.battery().remaining_percent() <
        policy_.minimum_takeoff_battery_percent) {
      return Reject(v1::REASON_CODE_BATTERY_BELOW_THRESHOLD,
                    "battery is below the takeoff threshold", state.sequence());
    }
    if (policy_.require_estimator_healthy &&
        state.estimator().level() != v1::HEALTH_LEVEL_HEALTHY) {
      return Reject(v1::REASON_CODE_ESTIMATOR_UNHEALTHY,
                    "estimator is not healthy", state.sequence());
    }
    if (policy_.require_home_for_takeoff && !state.has_home()) {
      return Reject(v1::REASON_CODE_HOME_UNAVAILABLE,
                    "home position is required", state.sequence());
    }
  } else if (command.has_disarm()) {
    if (!state.landed()) {
      return Reject(v1::REASON_CODE_NOT_LANDED,
                    "normal disarm is permitted only when landed",
                    state.sequence());
    }
  } else if (command.has_takeoff()) {
    const auto altitude = command.takeoff().target_altitude_agl_m();
    if (altitude <= 0.0 || altitude > policy_.maximum_altitude_m) {
      return Reject(v1::REASON_CODE_ALTITUDE_LIMIT,
                    "takeoff altitude violates the active safety policy",
                    state.sequence());
    }
    if (!state.landed()) {
      return Reject(v1::REASON_CODE_NOT_LANDED,
                    "takeoff requires a confirmed landed state", state.sequence());
    }
    if (command.takeoff().has_limits()) {
      auto result = ValidateLimits(command.takeoff().limits(), state);
      if (!result.valid()) return result;
    }
  } else if (command.has_goto_()) {
    auto result = ValidatePosition(command.goto_().destination(), state);
    if (!result.valid()) return result;
    if (command.goto_().acceptance_radius_m() <= 0.0) {
      return Reject(v1::REASON_CODE_INVALID_ARGUMENT,
                    "acceptance radius must be positive", state.sequence());
    }
    if (command.goto_().has_limits()) {
      result = ValidateLimits(command.goto_().limits(), state);
      if (!result.valid()) return result;
    }
  } else if (command.has_execute_route()) {
    if (command.execute_route().route_id().empty() ||
        command.execute_route().points().empty() ||
        command.execute_route().points_size() > 256) {
      return Reject(v1::REASON_CODE_INVALID_ARGUMENT,
                    "route requires an id and between 1 and 256 points",
                    state.sequence());
    }
    for (const auto& point : command.execute_route().points()) {
      auto result = ValidatePosition(point.position(), state);
      if (!result.valid()) return result;
      if (point.acceptance_radius_m() <= 0.0) {
        return Reject(v1::REASON_CODE_INVALID_ARGUMENT,
                      "every route point requires a positive acceptance radius",
                      state.sequence());
      }
      if (point.has_limits()) {
        result = ValidateLimits(point.limits(), state);
        if (!result.valid()) return result;
      }
    }
  } else if (command.has_return_home() && !state.has_home()) {
    return Reject(v1::REASON_CODE_HOME_UNAVAILABLE,
                  "return requires a valid home position", state.sequence());
  } else if (command.has_orbit()) {
    auto result = ValidatePosition(command.orbit().center(), state);
    if (!result.valid()) return result;
    if (command.orbit().radius_m() <= 0.0 ||
        command.orbit().revolutions() <= 0.0) {
      return Reject(v1::REASON_CODE_INVALID_ARGUMENT,
                    "orbit radius and revolutions must be positive",
                    state.sequence());
    }
    if (command.orbit().ground_speed_mps() <= 0.0 ||
        command.orbit().ground_speed_mps() >
            policy_.maximum_horizontal_speed_mps) {
      return Reject(v1::REASON_CODE_SPEED_LIMIT,
                    "orbit speed violates the active safety policy",
                    state.sequence());
    }
    if (command.orbit().altitude_frame() ==
            v1::COORDINATE_FRAME_WGS84_ABOVE_HOME &&
        (command.orbit().altitude_m() < 0.0 ||
         command.orbit().altitude_m() > policy_.maximum_altitude_m)) {
      return Reject(v1::REASON_CODE_ALTITUDE_LIMIT,
                    "orbit altitude violates the active safety policy",
                    state.sequence());
    }
  }
  return Accept(state.sequence());
}

v1::ValidationResult Guardrails::ValidateLimits(
    const v1::ActionLimits& limits, const v1::DroneState& state) const {
  if (limits.has_maximum_ground_speed_mps() &&
      (limits.maximum_ground_speed_mps() <= 0.0 ||
       limits.maximum_ground_speed_mps() >
           policy_.maximum_horizontal_speed_mps)) {
    return Reject(v1::REASON_CODE_SPEED_LIMIT,
                  "requested horizontal speed violates safety policy",
                  state.sequence());
  }
  if (limits.has_maximum_climb_rate_mps() &&
      (limits.maximum_climb_rate_mps() <= 0.0 ||
       limits.maximum_climb_rate_mps() > policy_.maximum_climb_speed_mps)) {
    return Reject(v1::REASON_CODE_SPEED_LIMIT,
                  "requested climb rate violates safety policy",
                  state.sequence());
  }
  if (limits.has_maximum_descent_rate_mps() &&
      (limits.maximum_descent_rate_mps() <= 0.0 ||
       limits.maximum_descent_rate_mps() > policy_.maximum_descent_speed_mps)) {
    return Reject(v1::REASON_CODE_SPEED_LIMIT,
                  "requested descent rate violates safety policy",
                  state.sequence());
  }
  if (limits.has_minimum_clearance_m() &&
      limits.minimum_clearance_m() <= 0.0) {
    return Reject(v1::REASON_CODE_INVALID_ARGUMENT,
                  "minimum clearance must be positive", state.sequence());
  }
  return Accept(state.sequence());
}

v1::ValidationResult Guardrails::ValidateContext(
    const v1::CommandContext& context, const v1::DroneState& state,
    bool lease_authorized, std::int64_t now_unix_ms) const {
  if (context.request_id().empty() || context.idempotency_key().empty() ||
      context.vehicle_id().empty() || context.client_id().empty()) {
    return Reject(v1::REASON_CODE_INVALID_ARGUMENT,
                  "request, idempotency, vehicle and client identifiers are required",
                  state.sequence());
  }
  if (context.vehicle_id() != state.vehicle_id()) {
    return Reject(v1::REASON_CODE_INVALID_ARGUMENT,
                  "command vehicle does not match state vehicle", state.sequence());
  }
  if (!lease_authorized) {
    return Reject(v1::REASON_CODE_CONTROL_LEASE_REQUIRED,
                  "a current matching control lease is required", state.sequence());
  }
  if (context.expires_at_unix_ms() <= now_unix_ms ||
      context.issued_at_unix_ms() > now_unix_ms + 1'000) {
    return Reject(v1::REASON_CODE_REQUEST_EXPIRED,
                  "command is expired or issued in the future", state.sequence());
  }
  if (context.minimum_state_sequence() != 0 &&
      context.minimum_state_sequence() > state.sequence()) {
    return Reject(v1::REASON_CODE_STATE_CHANGED,
                  "server has not observed the state used to prepare the command",
                  state.sequence());
  }
  const auto age = std::max<std::int64_t>(
      0, now_unix_ms - state.observed_at_unix_ms());
  if (age > policy_.maximum_state_age_ms) {
    return Reject(v1::REASON_CODE_STATE_STALE,
                  "vehicle state is too old for command execution",
                  state.sequence());
  }
  return Accept(state.sequence(), "command context is valid");
}

v1::ValidationResult Guardrails::ValidatePosition(
    const v1::Position& position, const v1::DroneState& state) const {
  if (position.has_geo()) {
    const auto& geo = position.geo();
    if (geo.latitude_deg() < -90.0 || geo.latitude_deg() > 90.0 ||
        geo.longitude_deg() < -180.0 || geo.longitude_deg() > 180.0 ||
        geo.frame() == v1::COORDINATE_FRAME_UNSPECIFIED ||
        geo.frame() == v1::COORDINATE_FRAME_LOCAL_NED) {
      return Reject(v1::REASON_CODE_INVALID_ARGUMENT,
                    "invalid geographic position or coordinate frame",
                    state.sequence());
    }
    if (geo.frame() == v1::COORDINATE_FRAME_WGS84_ABOVE_HOME &&
        (geo.altitude_m() < 0.0 ||
         geo.altitude_m() > policy_.maximum_altitude_m)) {
      return Reject(v1::REASON_CODE_ALTITUDE_LIMIT,
                    "destination altitude violates the active safety policy",
                    state.sequence());
    }
    if (state.has_home() && state.home().has_geo() &&
        HorizontalDistanceM(state.home().geo(), geo) >
            policy_.maximum_distance_from_home_m) {
      return Reject(v1::REASON_CODE_GEOFENCE_VIOLATION,
                    "destination exceeds maximum distance from home",
                    state.sequence());
    }
    return Accept(state.sequence());
  }
  if (position.has_local_ned()) {
    const auto& local = position.local_ned();
    if (local.origin_id().empty()) {
      return Reject(v1::REASON_CODE_INVALID_ARGUMENT,
                    "local position requires an origin id", state.sequence());
    }
    const double horizontal = std::hypot(local.north_m(), local.east_m());
    const double altitude = -local.down_m();
    if (horizontal > policy_.maximum_distance_from_home_m) {
      return Reject(v1::REASON_CODE_GEOFENCE_VIOLATION,
                    "local destination exceeds maximum distance from origin",
                    state.sequence());
    }
    if (altitude < 0.0 || altitude > policy_.maximum_altitude_m) {
      return Reject(v1::REASON_CODE_ALTITUDE_LIMIT,
                    "local destination altitude violates safety policy",
                    state.sequence());
    }
    return Accept(state.sequence());
  }
  return Reject(v1::REASON_CODE_INVALID_ARGUMENT,
                "destination position is required", state.sequence());
}

v1::ValidationResult Guardrails::Accept(std::uint64_t sequence,
                                        std::string message) {
  v1::ValidationResult result;
  result.set_valid(true);
  result.set_reason_code(v1::REASON_CODE_OK);
  result.set_message(std::move(message));
  result.set_evaluated_state_sequence(sequence);
  return result;
}

v1::ValidationResult Guardrails::Reject(v1::ReasonCode code,
                                        std::string message,
                                        std::uint64_t sequence) {
  v1::ValidationResult result;
  result.set_valid(false);
  result.set_reason_code(code);
  result.set_message(std::move(message));
  result.set_evaluated_state_sequence(sequence);
  return result;
}

const v1::CommandContext* Guardrails::ContextOf(
    const v1::ActionCommand& command) {
  switch (command.request_case()) {
    case v1::ActionCommand::kArm:
      return &command.arm().context();
    case v1::ActionCommand::kDisarm:
      return &command.disarm().context();
    case v1::ActionCommand::kTakeoff:
      return &command.takeoff().context();
    case v1::ActionCommand::kGoto:
      return &command.goto_().context();
    case v1::ActionCommand::kExecuteRoute:
      return &command.execute_route().context();
    case v1::ActionCommand::kHold:
      return &command.hold().context();
    case v1::ActionCommand::kReturnHome:
      return &command.return_home().context();
    case v1::ActionCommand::kLand:
      return &command.land().context();
    case v1::ActionCommand::kOrbit:
      return &command.orbit().context();
    case v1::ActionCommand::REQUEST_NOT_SET:
      return nullptr;
    default:
      return nullptr;
  }
}

}  // namespace icarus::guardrails

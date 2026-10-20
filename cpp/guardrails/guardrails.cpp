#include "guardrails/guardrails.hpp"
#include "state_engine/battery_validity.hpp"

#include <algorithm>
#include <cmath>
#include <optional>
#include <stdexcept>
#include <utility>
#include <vector>

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

// Altitude above home when the frame makes it knowable from state alone.
std::optional<double> AltitudeAboveHomeM(const v1::GeoPosition& geo,
                                         const v1::DroneState& state) {
  if (geo.frame() == v1::COORDINATE_FRAME_WGS84_ABOVE_HOME) {
    return geo.altitude_m();
  }
  if (geo.frame() == v1::COORDINATE_FRAME_WGS84_AMSL && state.has_home() &&
      state.home().has_geo() &&
      state.home().geo().frame() == v1::COORDINATE_FRAME_WGS84_AMSL &&
      std::isfinite(state.home().geo().altitude_m())) {
    return geo.altitude_m() - state.home().geo().altitude_m();
  }
  return std::nullopt;
}

bool WellFormed(const v1::Position& position) {
  if (position.has_geo()) {
    const auto& geo = position.geo();
    return std::isfinite(geo.latitude_deg()) &&
           std::isfinite(geo.longitude_deg()) &&
           std::isfinite(geo.altitude_m()) && geo.latitude_deg() >= -90.0 &&
           geo.latitude_deg() <= 90.0 && geo.longitude_deg() >= -180.0 &&
           geo.longitude_deg() <= 180.0 &&
           (geo.frame() == v1::COORDINATE_FRAME_WGS84_AMSL ||
            geo.frame() == v1::COORDINATE_FRAME_WGS84_ABOVE_HOME ||
            geo.frame() == v1::COORDINATE_FRAME_WGS84_ABOVE_TERRAIN);
  }
  if (position.has_local_ned()) {
    const auto& local = position.local_ned();
    return !local.origin_id().empty() && std::isfinite(local.north_m()) &&
           std::isfinite(local.east_m()) && std::isfinite(local.down_m());
  }
  return false;
}

// Horizontal distance of a target from measured home, independent of where
// the estimator's local origin was initialized.
std::optional<double> DistanceFromHomeM(const v1::Position& position,
                                        const v1::DroneState& state) {
  if (position.has_local_ned()) {
    const auto home = LocalHomePosition(state);
    if (!home || home->origin_id() != position.local_ned().origin_id()) return std::nullopt;
    return std::hypot(position.local_ned().north_m() - home->north_m(),
                      position.local_ned().east_m() - home->east_m());
  }
  if (position.has_geo() && state.has_home() && state.home().has_geo()) {
    return HorizontalDistanceM(state.home().geo(), position.geo());
  }
  return std::nullopt;
}

// Every minimum clearance a command requests, wherever its limits live.
std::vector<double> RequestedClearances(const v1::ActionCommand& command) {
  std::vector<double> requested;
  const auto collect = [&requested](const v1::ActionLimits& limits) {
    if (limits.has_minimum_clearance_m()) {
      requested.push_back(limits.minimum_clearance_m());
    }
  };
  if (command.has_goto_()) collect(command.goto_().limits());
  if (command.has_orbit()) collect(command.orbit().limits());
  if (command.has_execute_route()) {
    for (const auto& point : command.execute_route().points()) {
      collect(point.limits());
    }
  }
  return requested;
}

bool IsNavigationCommand(const v1::ActionCommand& command) {
  return command.has_goto_() || command.has_execute_route() ||
         command.has_orbit();
}

bool IsAirborneAction(const v1::ActionCommand& command) {
  return command.has_takeoff() || command.has_goto_() ||
         command.has_execute_route() ||
         command.has_hold() || command.has_return_home() ||
         command.has_land() || command.has_orbit() || command.has_yaw();
}

}  // namespace

std::optional<std::pair<double, double>> HorizontalOffsetM(
    const v1::Position& from, const v1::Position& to) {
  if (from.has_local_ned() && to.has_local_ned()) {
    return std::pair{to.local_ned().north_m() - from.local_ned().north_m(),
                     to.local_ned().east_m() - from.local_ned().east_m()};
  }
  if (from.has_geo() && to.has_geo()) {
    // Equirectangular offsets are exact to well under a centimetre across the
    // few hundred metres a geofence allows.
    const double latitude = DegreesToRadians(from.geo().latitude_deg());
    return std::pair{
        DegreesToRadians(to.geo().latitude_deg() - from.geo().latitude_deg()) *
            kEarthRadiusM,
        DegreesToRadians(std::remainder(to.geo().longitude_deg() -
                         from.geo().longitude_deg(), 360.0)) *
            kEarthRadiusM * std::cos(latitude)};
  }
  return std::nullopt;
}

std::optional<double> LocalHomeDownM(const v1::DroneState& state) {
  if (state.has_home_local_position_ned()) {
    const auto& home = state.home_local_position_ned();
    if (state.home_position_received_at_unix_ms() <= 0 ||
        state.home_position_received_at_unix_ms() > state.observed_at_unix_ms() ||
        home.origin_id().empty() || !state.has_local_position_ned() ||
        home.origin_id() != state.local_position_ned().origin_id() ||
        !std::isfinite(home.down_m()) || home.has_altitude_above_home_m())
      return std::nullopt;
    return home.down_m();
  }
  if (!state.has_local_position_ned() ||
      state.local_position_ned().origin_id().empty() ||
      state.local_position_ned().has_altitude_above_home_m() ||
      !state.has_velocity_ned_mps() ||
      !std::isfinite(state.local_position_ned().down_m()) ||
      !std::isfinite(state.relative_altitude_m()) ||
      !std::isfinite(state.velocity_ned_mps().z())) return std::nullopt;
  const auto now = state.observed_at_unix_ms();
  for (const auto received : {state.local_position_received_at_unix_ms(),
                             state.relative_altitude_received_at_unix_ms()}) {
    if (received <= 0 || received > now || now - received > 1000)
      return std::nullopt;
  }
  // Unsigned subtraction then signed cast also handles the uint32 boot wrap.
  const auto delta_ms = static_cast<std::int32_t>(
      state.local_position_time_boot_ms() - state.relative_altitude_time_boot_ms());
  if (std::abs(static_cast<std::int64_t>(delta_ms)) > 250) return std::nullopt;
  // Align the global above-home measurement to the local sample instant.
  // NED vertical velocity is positive down; above-home altitude decreases.
  const double aligned_relative = state.relative_altitude_m() -
      state.velocity_ned_mps().z() * static_cast<double>(delta_ms) / 1000.0;
  return state.local_position_ned().down_m() + aligned_relative;
}

std::optional<v1::LocalPositionNed> LocalHomePosition(const v1::DroneState& state) {
  const auto down = LocalHomeDownM(state);
  if (state.has_home_local_position_ned()) {
    const auto& home = state.home_local_position_ned();
    if (!down || !std::isfinite(home.north_m()) || !std::isfinite(home.east_m()))
      return std::nullopt;
    return home;
  }
  if (!down || !state.has_position() || !state.position().has_geo() ||
      !state.has_home() || !state.home().has_geo() ||
      !WellFormed(state.position()) || !WellFormed(state.home()) ||
      !std::isfinite(state.local_position_ned().north_m()) ||
      !std::isfinite(state.local_position_ned().east_m()) ||
      !std::isfinite(state.velocity_ned_mps().x()) ||
      !std::isfinite(state.velocity_ned_mps().y())) return std::nullopt;
  const auto offset = HorizontalOffsetM(state.position(), state.home());
  if (!offset) return std::nullopt;
  const auto delta_ms = static_cast<std::int32_t>(
      state.local_position_time_boot_ms() - state.relative_altitude_time_boot_ms());
  const double delta_s = static_cast<double>(delta_ms) / 1000.0;
  v1::LocalPositionNed home;
  home.set_origin_id(state.local_position_ned().origin_id());
  home.set_north_m(state.local_position_ned().north_m() + offset->first -
                   state.velocity_ned_mps().x() * delta_s);
  home.set_east_m(state.local_position_ned().east_m() + offset->second -
                  state.velocity_ned_mps().y() * delta_s);
  home.set_down_m(*down);
  return home;
}

std::optional<v1::Position> ResolveLocalAltitude(
    const v1::Position& target, const v1::DroneState& state) {
  if (!target.has_local_ned()) return target;
  const auto& local = target.local_ned();
  if (!state.has_local_position_ned() || local.origin_id().empty() ||
      local.origin_id() != state.local_position_ned().origin_id()) return std::nullopt;
  if (!local.has_altitude_above_home_m()) return target;
  const auto home_down = LocalHomeDownM(state);
  if (!home_down || !std::isfinite(local.altitude_above_home_m())) return std::nullopt;
  auto resolved = target;
  resolved.mutable_local_ned()->set_down_m(*home_down - local.altitude_above_home_m());
  resolved.mutable_local_ned()->clear_altitude_above_home_m();
  return resolved;
}

std::optional<v1::Position> OrbitAltitudePosition(
    const v1::OrbitRequest& orbit, const v1::DroneState& state) {
  auto flown = orbit.center();
  if (flown.has_local_ned()) {
    double above_home = orbit.altitude_m();
    if (orbit.altitude_frame() == v1::COORDINATE_FRAME_WGS84_AMSL) {
      if (!state.has_home() || !state.home().has_geo() ||
          state.home().geo().frame() != v1::COORDINATE_FRAME_WGS84_AMSL ||
          !std::isfinite(state.home().geo().altitude_m())) return std::nullopt;
      above_home -= state.home().geo().altitude_m();
    } else if (orbit.altitude_frame() != v1::COORDINATE_FRAME_UNSPECIFIED &&
               orbit.altitude_frame() != v1::COORDINATE_FRAME_WGS84_ABOVE_HOME) {
      return std::nullopt;
    }
    flown.mutable_local_ned()->set_altitude_above_home_m(above_home);
    return ResolveLocalAltitude(flown, state);
  }
  if (flown.has_geo()) {
    flown.mutable_geo()->set_altitude_m(orbit.altitude_m());
    flown.mutable_geo()->set_frame(orbit.altitude_frame() == v1::COORDINATE_FRAME_UNSPECIFIED
        ? v1::COORDINATE_FRAME_WGS84_ABOVE_HOME : orbit.altitude_frame());
  }
  return flown;
}

SafetyPolicy SafetyPolicy::Load(const std::filesystem::path& path) {
  const auto root = YAML::LoadFile(path.string());
  SafetyPolicy policy;
  policy.schema_version = root["schema_version"].as<std::string>();
  policy.profile = root["profile"].as<std::string>();
  policy.maximum_altitude_m = root["geofence"]["max_altitude_m"].as<double>();
  policy.maximum_distance_from_home_m =
      root["geofence"]["max_distance_from_home_m"].as<double>();
  // Optional so that policies recorded in older episodes still load: absent,
  // runtime targets share the ordinary fence.
  policy.maximum_runtime_target_distance_from_home_m =
      root["geofence"]["max_runtime_target_distance_from_home_m"]
          ? root["geofence"]["max_runtime_target_distance_from_home_m"]
                .as<double>()
          : policy.maximum_distance_from_home_m;
  policy.maximum_horizontal_speed_mps =
      root["motion"]["max_horizontal_speed_m_s"].as<double>();
  policy.maximum_climb_speed_mps =
      root["motion"]["max_climb_speed_m_s"].as<double>();
  policy.maximum_descent_speed_mps =
      root["motion"]["max_descent_speed_m_s"].as<double>();
  policy.maximum_tilt_deg = root["motion"]["max_tilt_deg"].as<double>();
  policy.minimum_takeoff_battery_percent =
      root["battery"]["minimum_takeoff_percent"].as<double>();
  policy.rtl_battery_percent = root["battery"]["rtl_percent"].as<double>();
  policy.land_battery_percent = root["battery"]["land_percent"].as<double>();
  policy.maximum_state_age_ms =
      root["state_health"]["max_state_age_ms"].as<std::uint32_t>();
  policy.stale_state_rtl_delay_ms = static_cast<std::uint32_t>(
      root["failure_actions"]["state_stale_rtl_delay_s"].as<double>() *
      1000.0);
  policy.control_lease_lost_rtl_delay_ms = static_cast<std::uint32_t>(
      root["failure_actions"]["control_lease_lost_rtl_delay_s"].as<double>() *
      1000.0);
  policy.maximum_perception_age_ms =
      root["perception"]["maximum_age_ms"].as<std::uint32_t>();
  policy.minimum_obstacle_clearance_m =
      root["perception"]["minimum_clearance_m"].as<double>();
  policy.emergency_stop_distance_m =
      root["perception"]["emergency_stop_distance_m"].as<double>();
  if (root["motion"]["braking_deceleration_mps2"])
    policy.braking_deceleration_mps2 = root["motion"]["braking_deceleration_mps2"].as<double>();
  if (root["motion"]["braking_reaction_s"])
    policy.braking_reaction_s = root["motion"]["braking_reaction_s"].as<double>();
  // Optional for the same reason; absent, the floor is the 2 m default capped
  // by the planner envelope.
  policy.hard_minimum_clearance_m =
      root["perception"]["hard_minimum_clearance_m"]
          ? root["perception"]["hard_minimum_clearance_m"].as<double>()
          : std::min(2.0, policy.minimum_obstacle_clearance_m);
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
      !std::isfinite(policy.maximum_tilt_deg) || policy.maximum_tilt_deg <= 0.0 ||
      policy.maximum_tilt_deg >= 90.0 ||
      policy.maximum_distance_from_home_m <= 0.0 ||
      !(policy.maximum_runtime_target_distance_from_home_m > 0.0) ||
      policy.maximum_runtime_target_distance_from_home_m >
          policy.maximum_distance_from_home_m ||
      policy.maximum_horizontal_speed_mps <= 0.0 ||
      !std::isfinite(policy.braking_deceleration_mps2) || policy.braking_deceleration_mps2 <= 0.0 ||
      !std::isfinite(policy.braking_reaction_s) || policy.braking_reaction_s < 0.0 ||
      policy.maximum_state_age_ms == 0 ||
      policy.control_lease_lost_rtl_delay_ms == 0 ||
      policy.maximum_perception_age_ms == 0 ||
      policy.minimum_obstacle_clearance_m <= 0.0 ||
      !(policy.hard_minimum_clearance_m > 0.0) ||
      policy.hard_minimum_clearance_m > policy.minimum_obstacle_clearance_m ||
      policy.emergency_stop_distance_m < policy.minimum_obstacle_clearance_m ||
      policy.land_battery_percent <= 0.0 ||
      policy.rtl_battery_percent <= policy.land_battery_percent) {
    throw std::runtime_error("invalid or unsupported safety policy");
  }
  return policy;
}

bool IsPassiveGroundHoldState(const v1::DroneState& state) {
  return !state.armed() && state.landed() &&
         state.flight_phase() == v1::FLIGHT_PHASE_DISARMED;
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

  const bool recovery = command.has_land() || command.has_return_home() ||
                        command.has_hold() || command.has_disarm();
  if (!recovery && !state_engine::BatteryAvailable(state.battery(), now_unix_ms)) {
    return Reject(v1::REASON_CODE_STATE_STALE,
                  "battery telemetry is missing, invalid or stale", state.sequence());
  }

  if (state.manual_override_active()) {
    return Reject(v1::REASON_CODE_MANUAL_OVERRIDE,
                  "manual operator currently owns flight authority",
                  state.sequence());
  }
  const bool passive_hold = command.has_hold() && IsPassiveGroundHoldState(state);
  if (passive_hold && (!command.hold().has_duration_ms() ||
                       command.hold().duration_ms() == 0 ||
                       command.hold().duration_ms() > 600'000)) {
    return Reject(v1::REASON_CODE_INVALID_ARGUMENT,
                  "grounded hold requires a bounded positive duration up to 600 seconds",
                  state.sequence());
  }
  if (IsAirborneAction(command) && !state.armed() && !passive_hold) {
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
    auto result = ValidatePosition(command.goto_().destination(), state,
                                   FenceFor(command.goto_().target_source()));
    if (!result.valid()) return result;
    if (!(command.goto_().acceptance_radius_m() > 0.0) ||
        !std::isfinite(command.goto_().acceptance_radius_m())) {
      return Reject(v1::REASON_CODE_INVALID_ARGUMENT,
                    "acceptance radius must be positive", state.sequence());
    }
    result = ValidateHeading(command.goto_().heading(),
                             command.goto_().destination(), state, false);
    if (!result.valid()) return result;
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
      auto result = ValidatePosition(point.position(), state,
                                     policy_.maximum_distance_from_home_m);
      if (!result.valid()) return result;
      if (point.acceptance_radius_m() <= 0.0) {
        return Reject(v1::REASON_CODE_INVALID_ARGUMENT,
                      "every route point requires a positive acceptance radius",
                      state.sequence());
      }
      result = ValidateHeading(point.heading(), point.position(), state, false);
      if (!result.valid()) return result;
      if (point.fly_through() && (point.hold_time_ms() > 0 ||
          (point.heading().mode() != v1::HEADING_MODE_FACE_PATH &&
           point.heading().mode() != v1::HEADING_MODE_UNSPECIFIED))) {
        return Reject(v1::REASON_CODE_INVALID_ARGUMENT,
                      "fly-through points cannot promise a stopped heading or dwell", state.sequence());
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
    auto result = ValidateOrbit(command.orbit(), state);
    if (!result.valid()) return result;
  } else if (command.has_yaw()) {
    if (!std::isfinite(command.yaw().relative_angle_deg()) ||
        std::abs(command.yaw().relative_angle_deg()) < 1.0 ||
        std::abs(command.yaw().relative_angle_deg()) > 360.0) {
      return Reject(v1::REASON_CODE_INVALID_ARGUMENT,
                    "relative yaw magnitude must be in 1..360 degrees",
                    state.sequence());
    }
    if (!std::isfinite(command.yaw().rate_deg_s()) ||
        command.yaw().rate_deg_s() < 5.0 ||
        command.yaw().rate_deg_s() > 60.0) {
      return Reject(v1::REASON_CODE_SPEED_LIMIT,
                    "yaw rate must be in 5..60 degrees per second",
                    state.sequence());
    }
  }
  auto accepted = Accept(state.sequence());
  // A lower clearance is not refused, because it cannot relax anything: the
  // executor always enforces max(hard floor, request). Say so, though.
  for (const double requested : RequestedClearances(command)) {
    if (requested < policy_.hard_minimum_clearance_m) {
      accepted.add_warnings(
          "requested minimum clearance is below the hard policy floor; the "
          "floor applies");
      break;
    }
  }
  return accepted;
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
      (!(limits.minimum_clearance_m() > 0.0) ||
       !std::isfinite(limits.minimum_clearance_m()))) {
    return Reject(v1::REASON_CODE_INVALID_ARGUMENT,
                  "minimum clearance must be positive", state.sequence());
  }
  return Accept(state.sequence());
}

v1::ValidationResult Guardrails::ValidateHeading(
    const v1::HeadingPolicy& heading, const v1::Position& anchor,
    const v1::DroneState& state, bool orbit) const {
  switch (heading.mode()) {
    case v1::HEADING_MODE_UNSPECIFIED:
    case v1::HEADING_MODE_KEEP_CURRENT:
    // Face-path is the autopilot's own GUIDED yaw behaviour: it looks along
    // the path in transit and needs no explicit yaw command.
    case v1::HEADING_MODE_FACE_PATH:
      return Accept(state.sequence());
    case v1::HEADING_MODE_ABSOLUTE:
      if (!heading.has_heading_deg() || !std::isfinite(heading.heading_deg()) ||
          heading.heading_deg() < 0.0 || heading.heading_deg() > 360.0) {
        return Reject(v1::REASON_CODE_INVALID_ARGUMENT,
                      "absolute heading must be in 0..360 degrees",
                      state.sequence());
      }
      return Accept(state.sequence());
    case v1::HEADING_MODE_FACE_POINT: {
      // An orbit without an explicit point faces its own centre.
      if (!heading.has_face_point()) {
        if (orbit) return Accept(state.sequence());
        return Reject(v1::REASON_CODE_INVALID_ARGUMENT,
                      "face-point heading requires a face point",
                      state.sequence());
      }
      if (!WellFormed(heading.face_point())) {
        return Reject(v1::REASON_CODE_INVALID_ARGUMENT,
                      "invalid face point", state.sequence());
      }
      const auto offset = HorizontalOffsetM(anchor, heading.face_point());
      if (!offset) {
        return Reject(v1::REASON_CODE_INVALID_ARGUMENT,
                      "face point must use the same frame as the target",
                      state.sequence());
      }
      if (!orbit && std::hypot(offset->first, offset->second) <
                        kMinimumFacePointDistanceM) {
        return Reject(v1::REASON_CODE_INVALID_ARGUMENT,
                      "face point must be at least 0.5 m horizontally from "
                      "the destination",
                      state.sequence());
      }
      return Accept(state.sequence());
    }
    default:
      return Reject(v1::REASON_CODE_INVALID_ARGUMENT, "unknown heading mode",
                    state.sequence());
  }
}

v1::ValidationResult Guardrails::ValidateOrbit(
    const v1::OrbitRequest& orbit, const v1::DroneState& state) const {
  const double fence = FenceFor(orbit.target_source());
  if (orbit.altitude_frame() == v1::COORDINATE_FRAME_WGS84_ABOVE_TERRAIN) {
    return Reject(v1::REASON_CODE_UNSUPPORTED,
                  "terrain-relative orbit needs a measured terrain altitude transform",
                  state.sequence());
  }
  const auto flown = OrbitAltitudePosition(orbit, state);
  if (!flown) return Reject(v1::REASON_CODE_POSITION_UNAVAILABLE,
      "orbit altitude requires a fresh coherent home/origin transform", state.sequence());
  auto result = ValidatePosition(*flown, state, fence);
  if (!result.valid()) return result;
  if (!(orbit.radius_m() > 0.0) || !std::isfinite(orbit.radius_m()) ||
      !(orbit.revolutions() > 0.0) ||
      orbit.revolutions() > kMaximumOrbitRevolutions) {
    return Reject(v1::REASON_CODE_INVALID_ARGUMENT,
                  "orbit radius must be positive and revolutions in (0, 10]",
                  state.sequence());
  }
  // The centre being inside the fence is not enough: every waypoint is on
  // the circle, so the far side of it is what must stay inside.
  const auto centre_distance = DistanceFromHomeM(orbit.center(), state);
  if (centre_distance && *centre_distance + orbit.radius_m() > fence) {
    return Reject(v1::REASON_CODE_GEOFENCE_VIOLATION,
                  "orbit circle exceeds maximum distance from home",
                  state.sequence());
  }
  if (!(orbit.ground_speed_mps() > 0.0) ||
      orbit.ground_speed_mps() > policy_.maximum_horizontal_speed_mps) {
    return Reject(v1::REASON_CODE_SPEED_LIMIT,
                  "orbit speed violates the active safety policy",
                  state.sequence());
  }
  if (orbit.viewpoints() > kMaximumOrbitViewpoints ||
      (orbit.viewpoints() > 0 && orbit.revolutions() > 1.0) ||
      orbit.viewpoint_dwell_ms() > kMaximumViewpointDwellMs) {
    return Reject(v1::REASON_CODE_INVALID_ARGUMENT,
                  "orbit viewpoints must be 0..24 over at most one revolution "
                  "with a dwell of at most 30 s",
                  state.sequence());
  }
  result = ValidateHeading(orbit.heading(), orbit.center(), state, true);
  if (!result.valid()) return result;
  if (orbit.has_limits()) return ValidateLimits(orbit.limits(), state);
  return Accept(state.sequence());
}

double Guardrails::FenceFor(v1::TargetSource source) const {
  return source == v1::TARGET_SOURCE_RUNTIME_ESTIMATE
             ? policy_.maximum_runtime_target_distance_from_home_m
             : policy_.maximum_distance_from_home_m;
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
    const v1::Position& position, const v1::DroneState& state,
    double maximum_distance_from_home_m) const {
  if (position.has_geo()) {
    const auto& geo = position.geo();
    if (!WellFormed(position)) {
      return Reject(v1::REASON_CODE_INVALID_ARGUMENT,
                    "invalid geographic position or coordinate frame",
                    state.sequence());
    }
    if (geo.frame() == v1::COORDINATE_FRAME_WGS84_ABOVE_TERRAIN) {
      return Reject(v1::REASON_CODE_UNSUPPORTED,
                    "terrain-relative navigation needs a measured terrain altitude transform",
                    state.sequence());
    }
    if (!state.has_home() || !state.home().has_geo() || !WellFormed(state.home())) {
      return Reject(v1::REASON_CODE_HOME_UNAVAILABLE,
                    "geographic navigation requires measured home for its geofence",
                    state.sequence());
    }
    const auto altitude = AltitudeAboveHomeM(geo, state);
    if (!altitude || !std::isfinite(*altitude)) {
      return Reject(v1::REASON_CODE_POSITION_UNAVAILABLE,
                    "AMSL altitude requires a measured AMSL home altitude",
                    state.sequence());
    }
    if (*altitude < 0.0 || *altitude > policy_.maximum_altitude_m) {
      return Reject(v1::REASON_CODE_ALTITUDE_LIMIT,
                    "destination altitude violates the active safety policy",
                    state.sequence());
    }
    if (state.has_home() && state.home().has_geo() &&
        HorizontalDistanceM(state.home().geo(), geo) >
            maximum_distance_from_home_m) {
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
    if (!WellFormed(position)) {
      return Reject(v1::REASON_CODE_INVALID_ARGUMENT,
                    "local position must be finite", state.sequence());
    }
    const auto resolved = ResolveLocalAltitude(position, state);
    const auto home_down = LocalHomeDownM(state);
    if (!resolved || !home_down) {
      return Reject(v1::REASON_CODE_POSITION_UNAVAILABLE,
                    "local altitude requires a fresh coherent home/origin transform",
                    state.sequence());
    }
    const auto horizontal = DistanceFromHomeM(position, state);
    if (!horizontal) return Reject(v1::REASON_CODE_POSITION_UNAVAILABLE,
        "local geofence requires a measured home/origin transform", state.sequence());
    const double altitude = *home_down - resolved->local_ned().down_m();
    if (*horizontal > maximum_distance_from_home_m) {
      return Reject(v1::REASON_CODE_GEOFENCE_VIOLATION,
                    "local destination exceeds maximum distance from home",
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
    case v1::ActionCommand::kYaw:
      return &command.yaw().context();
    case v1::ActionCommand::REQUEST_NOT_SET:
      return nullptr;
    default:
      return nullptr;
  }
}

}  // namespace icarus::guardrails

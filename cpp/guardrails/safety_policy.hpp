#pragma once

#include <cstdint>
#include <filesystem>
#include <string>

namespace icarus::guardrails {

struct SafetyPolicy {
  std::string schema_version;
  std::string profile;
  double maximum_altitude_m = 0.0;
  double maximum_distance_from_home_m = 0.0;
  // Tighter fence for targets computed in flight from perception
  // (TARGET_SOURCE_RUNTIME_ESTIMATE). Never larger than the ordinary fence.
  double maximum_runtime_target_distance_from_home_m = 0.0;
  double maximum_horizontal_speed_mps = 0.0;
  double maximum_climb_speed_mps = 0.0;
  double maximum_descent_speed_mps = 0.0;
  double maximum_tilt_deg = 30.0;
  double minimum_takeoff_battery_percent = 0.0;
  double rtl_battery_percent = 0.0;
  double land_battery_percent = 0.0;
  std::uint32_t maximum_state_age_ms = 0;
  std::uint32_t stale_state_rtl_delay_ms = 0;
  std::uint32_t control_lease_lost_rtl_delay_ms = 0;
  std::uint32_t maximum_perception_age_ms = 0;
  double minimum_obstacle_clearance_m = 0.0;
  // Live LiDAR floor enforced while goto/route/orbit/yaw actions move the
  // aircraft. A per-action minimum_clearance_m can raise it, never lower it.
  double hard_minimum_clearance_m = 0.0;
  double emergency_stop_distance_m = 0.0;
  double braking_deceleration_mps2 = 1.0;
  double braking_reaction_s = 1.5;
  bool require_estimator_healthy = true;
  bool require_position_for_navigation = true;
  bool require_home_for_takeoff = true;
  bool manual_operator_has_priority = true;

  [[nodiscard]] static SafetyPolicy Load(const std::filesystem::path& path);
};

}  // namespace icarus::guardrails

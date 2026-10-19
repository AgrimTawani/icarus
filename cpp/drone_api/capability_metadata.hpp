#pragma once

#include <iomanip>
#include <limits>
#include <sstream>

#include "guardrails/safety_policy.hpp"
#include "icarus/v1/drone_api.pb.h"

namespace icarus::drone_api {

// The same active policy supplies validation and capability discovery. These
// attributes describe enforced limits; they do not perform mission admission.
inline v1::Capability MotionLimitsCapability(const guardrails::SafetyPolicy& policy) {
  v1::Capability capability;
  capability.set_name("motion_limits");
  capability.set_supported(true);
  capability.set_version("1");
  auto& attributes = *capability.mutable_attributes();
  const auto add = [&](const char* name, double value) {
    std::ostringstream text;
    text << std::setprecision(std::numeric_limits<double>::max_digits10) << value;
    attributes[name] = text.str();
  };
  add("maximum_climb_speed_mps", policy.maximum_climb_speed_mps);
  add("maximum_horizontal_speed_mps", policy.maximum_horizontal_speed_mps);
  add("maximum_descent_speed_mps", policy.maximum_descent_speed_mps);
  add("maximum_distance_from_home_m", policy.maximum_distance_from_home_m);
  add("maximum_runtime_target_distance_from_home_m",
      policy.maximum_runtime_target_distance_from_home_m);
  return capability;
}

}  // namespace icarus::drone_api

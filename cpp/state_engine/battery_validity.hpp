#pragma once

#include <cmath>
#include <cstdint>

#include "icarus/v1/state.pb.h"

namespace icarus::state_engine {
// SYS_STATUS commonly arrives at 1 Hz, independently of faster pose frames.
// Five missed nominal updates invalidate power availability; pose traffic
// must never refresh this measurement's timestamp.
inline constexpr std::int64_t kMaximumBatteryAgeMs = 5'000;

inline bool BatteryAvailable(const v1::BatteryState& battery,
                             std::int64_t now_unix_ms) {
  const auto age = now_unix_ms - battery.received_at_unix_ms();
  return battery.received_at_unix_ms() > 0 && age >= 0 &&
         age <= kMaximumBatteryAgeMs &&
         (battery.level() == v1::HEALTH_LEVEL_HEALTHY ||
          battery.level() == v1::HEALTH_LEVEL_DEGRADED) &&
         battery.has_voltage_v() && std::isfinite(battery.voltage_v()) &&
         battery.voltage_v() > 0 && battery.has_current_a() &&
         std::isfinite(battery.current_a()) && battery.has_remaining_percent() &&
         std::isfinite(battery.remaining_percent()) &&
         battery.remaining_percent() >= 0 && battery.remaining_percent() <= 100;
}
}  // namespace icarus::state_engine

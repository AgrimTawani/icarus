#include "state_engine/state_engine.hpp"

#include <algorithm>

namespace icarus::state_engine {

StateEngine::StateEngine(const core::Clock& clock) : clock_(clock) {}

void StateEngine::Publish(v1::DroneState state) {
  std::lock_guard<std::mutex> lock(mutex_);
  state.set_sequence(next_sequence_++);
  if (state.observed_at_unix_ms() == 0) {
    state.set_observed_at_unix_ms(clock_.NowUnixMs());
  }
  state_ = std::move(state);
}

std::optional<v1::DroneState> StateEngine::Snapshot() const {
  std::lock_guard<std::mutex> lock(mutex_);
  return state_;
}

bool StateEngine::IsFresh(std::uint32_t maximum_age_ms) const {
  std::lock_guard<std::mutex> lock(mutex_);
  if (!state_) {
    return false;
  }
  const auto age = std::max<std::int64_t>(
      0, clock_.NowUnixMs() - state_->observed_at_unix_ms());
  return age <= maximum_age_ms;
}

v1::HealthReport StateEngine::Health(std::uint32_t maximum_age_ms) const {
  std::lock_guard<std::mutex> lock(mutex_);
  v1::HealthReport report;
  if (!state_) {
    report.set_overall(v1::HEALTH_LEVEL_UNAVAILABLE);
    report.add_blocking_reason_codes("STATE_UNAVAILABLE");
    return report;
  }

  report.set_vehicle_id(state_->vehicle_id());
  report.set_state_sequence(state_->sequence());
  report.set_observed_at_unix_ms(state_->observed_at_unix_ms());
  const auto age = std::max<std::int64_t>(
      0, clock_.NowUnixMs() - state_->observed_at_unix_ms());
  const bool fresh = age <= maximum_age_ms;
  const bool link = state_->has_mavlink() && state_->mavlink().connected() &&
                    state_->mavlink().level() != v1::HEALTH_LEVEL_FAILED;
  const bool estimator = state_->has_estimator() &&
                         state_->estimator().level() == v1::HEALTH_LEVEL_HEALTHY;
  const bool battery = state_->has_battery() &&
                       state_->battery().level() != v1::HEALTH_LEVEL_FAILED;
  report.set_ready_to_arm(fresh && link && estimator && battery && state_->landed());
  report.set_ready_to_navigate(fresh && link && estimator &&
                               state_->estimator().horizontal_position_valid());
  report.set_overall(report.ready_to_arm() || report.ready_to_navigate()
                         ? v1::HEALTH_LEVEL_HEALTHY
                         : v1::HEALTH_LEVEL_DEGRADED);
  if (!fresh) report.add_blocking_reason_codes("STATE_STALE");
  if (!link) report.add_blocking_reason_codes("MAVLINK_UNHEALTHY");
  if (!estimator) report.add_blocking_reason_codes("ESTIMATOR_UNHEALTHY");
  if (!battery) report.add_blocking_reason_codes("BATTERY_UNHEALTHY");
  if (!state_->landed()) report.add_blocking_reason_codes("NOT_LANDED");
  for (const auto& source : state_->sources()) {
    *report.add_subsystems() = source;
  }
  return report;
}

}  // namespace icarus::state_engine

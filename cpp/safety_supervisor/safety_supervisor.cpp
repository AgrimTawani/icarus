#include "safety_supervisor/safety_supervisor.hpp"

#include <chrono>
#include <cmath>

namespace icarus::safety_supervisor {
namespace {
constexpr std::uint32_t kModeRtl = 6;
constexpr std::uint32_t kModeLand = 9;
constexpr std::uint32_t kModeBrake = 17;
}

SafetySupervisor::SafetySupervisor(
    const core::Clock& clock, const guardrails::SafetyPolicy& policy,
    state_engine::StateEngine& state_engine,
    mission_executor::ActionStore& actions,
    mavlink_gateway::MavlinkGateway& gateway,
    perception::PerceptionEngine* perception)
    : clock_(clock),
      policy_(policy),
      state_engine_(state_engine),
      actions_(actions),
      gateway_(gateway),
      perception_(perception) {}

SafetySupervisor::~SafetySupervisor() { Stop(); }

void SafetySupervisor::Start() {
  if (running_.exchange(true)) return;
  worker_ = std::thread(&SafetySupervisor::Run, this);
}

void SafetySupervisor::Stop() {
  if (!running_.exchange(false)) return;
  if (worker_.joinable()) worker_.join();
}

void SafetySupervisor::AbortActive(v1::ReasonCode reason, const char* message,
                                   const v1::DroneState* state) {
  const auto active = actions_.ActiveConflictingAction();
  if (active) {
    actions_.Transition(*active, v1::ACTION_STATE_ABORTED_BY_SAFETY, reason,
                        message, 0.0, state);
  }
}

void SafetySupervisor::Run() {
  std::int64_t stale_since = 0;
  int last_response = 0;
  while (running_) {
    const auto state = state_engine_.Snapshot();
    if (!state || !state->armed()) {
      stale_since = 0;
      last_response = 0;
      std::this_thread::sleep_for(std::chrono::milliseconds(100));
      continue;
    }

    const auto now = clock_.NowUnixMs();
    const bool stale = !gateway_.IsConnected() ||
                       now - state->observed_at_unix_ms() >
                           policy_.maximum_state_age_ms;
    int response = 0;
    v1::ReasonCode reason = v1::REASON_CODE_OK;
    const char* message = "";
    if (state->manual_override_active()) {
      AbortActive(v1::REASON_CODE_MANUAL_OVERRIDE,
                  "manual operator preempted autonomous action", &*state);
      std::this_thread::sleep_for(std::chrono::milliseconds(100));
      continue;
    } else if (stale) {
      if (stale_since == 0) stale_since = now;
      response = now - stale_since >= policy_.stale_state_rtl_delay_ms ? 2 : 1;
      reason = gateway_.IsConnected() ? v1::REASON_CODE_STATE_STALE
                                      : v1::REASON_CODE_NOT_CONNECTED;
      message = response == 1 ? "state stale: safe hold commanded"
                              : "state stale: RTL escalation commanded";
    } else if (perception_ != nullptr && !perception_->Healthy()) {
      response = 1;
      reason = v1::REASON_CODE_SAFETY_INTERVENTION;
      message = "perception stale: safe hold commanded";
    } else if (perception_ != nullptr) {
      const auto perception = perception_->Summary();
      bool recovery_action = false;
      if (const auto active = actions_.ActiveConflictingAction()) {
        if (const auto status = actions_.Find(*active)) {
          recovery_action = status->type() == v1::ACTION_TYPE_LAND ||
                            status->type() == v1::ACTION_TYPE_RETURN_HOME;
        }
      }
      if (!recovery_action &&
          perception.nearest_obstacle_distance_m() >= 0.0 &&
          perception.nearest_obstacle_distance_m() <=
              policy_.emergency_stop_distance_m &&
          std::abs(perception.nearest_obstacle_bearing_deg()) <= 50.0) {
        response = 1;
        reason = v1::REASON_CODE_SAFETY_INTERVENTION;
        message = "obstacle inside emergency envelope: safe hold commanded";
      }
    }
    if (response == 0) {
      stale_since = 0;
      const double battery = state->battery().remaining_percent();
      if (battery <= policy_.land_battery_percent) {
        response = 3;
        reason = v1::REASON_CODE_BATTERY_BELOW_THRESHOLD;
        message = "critical battery: landing commanded";
      } else if (battery <= policy_.rtl_battery_percent) {
        response = 2;
        reason = v1::REASON_CODE_BATTERY_BELOW_THRESHOLD;
        message = "low battery: RTL commanded";
      } else if (state->has_local_position_ned() &&
                 std::hypot(state->local_position_ned().north_m(),
                            state->local_position_ned().east_m()) >
                     policy_.maximum_distance_from_home_m) {
        response = 2;
        reason = v1::REASON_CODE_GEOFENCE_VIOLATION;
        message = "geofence breached: RTL commanded";
      }
    }
    if (response != 0 && response != last_response) {
      AbortActive(reason, message, &*state);
      if (response == 1) gateway_.SetMode(kModeBrake);
      if (response == 2 && gateway_.IsConnected()) gateway_.SetMode(kModeRtl);
      if (response == 3 && gateway_.IsConnected()) gateway_.SetMode(kModeLand);
    }
    last_response = response;
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
  }
}

}  // namespace icarus::safety_supervisor

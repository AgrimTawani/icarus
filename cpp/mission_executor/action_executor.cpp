#include "mission_executor/action_executor.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <stdexcept>

namespace icarus::mission_executor {
namespace {

constexpr std::uint32_t kModeGuided = 4;
constexpr std::uint32_t kModeRtl = 6;
constexpr std::uint32_t kModeLand = 9;
constexpr std::uint32_t kModeBrake = 17;
constexpr double kEarthRadiusM = 6'371'000.0;
constexpr double kPi = 3.14159265358979323846;

std::uint32_t TimeoutOf(const v1::ActionLimits& limits,
                        std::uint32_t fallback) {
  return limits.has_execution_timeout_ms() ? limits.execution_timeout_ms()
                                            : fallback;
}

v1::ActionReceipt SimpleReceipt(const std::string& action_id,
                                v1::ActionState state,
                                v1::ReasonCode reason,
                                const std::string& message,
                                std::int64_t now) {
  v1::ActionReceipt receipt;
  receipt.set_action_id(action_id);
  receipt.set_disposition(state);
  receipt.set_reason_code(reason);
  receipt.set_message(message);
  receipt.set_received_at_unix_ms(now);
  return receipt;
}

}  // namespace

ActionExecutor::ActionExecutor(
    const core::Clock& clock, ActionStore& store,
    state_engine::StateEngine& state_engine,
    drone_api::AuthorityManager& authority,
    const guardrails::Guardrails& guardrails,
    mavlink_gateway::MavlinkGateway& gateway,
    perception::PerceptionEngine* perception,
    const local_planner::LocalPlanner* planner)
    : clock_(clock),
      store_(store),
      state_engine_(state_engine),
      authority_(authority),
      guardrails_(guardrails),
      gateway_(gateway),
      perception_(perception),
      planner_(planner) {}

ActionExecutor::~ActionExecutor() { Stop(); }

void ActionExecutor::Start() {
  if (running_.exchange(true)) return;
  worker_ = std::thread(&ActionExecutor::Run, this);
}

void ActionExecutor::Stop() {
  if (!running_.exchange(false)) return;
  queue_changed_.notify_all();
  if (worker_.joinable()) worker_.join();
}

v1::ActionReceipt ActionExecutor::Submit(const v1::ActionCommand& command) {
  const auto existing =
      store_.FindByIdempotencyKey(ContextOf(command).idempotency_key());
  if (existing) {
    return SimpleReceipt(existing->action_id(), existing->state(),
                         existing->reason_code(), existing->message(),
                         clock_.NowUnixMs());
  }
  const auto state = state_engine_.Snapshot();
  if (!state) {
    v1::ValidationResult unavailable;
    unavailable.set_valid(false);
    unavailable.set_reason_code(v1::REASON_CODE_STATE_STALE);
    unavailable.set_message("vehicle state is unavailable");
    return store_.Reject(command, unavailable);
  }
  const bool authorized = authority_.IsAuthorized(ContextOf(command));
  const auto validation =
      guardrails_.Validate(command, *state, authorized, clock_.NowUnixMs());
  if (!validation.valid()) return store_.Reject(command, validation);
  if (store_.ActiveConflictingAction()) {
    v1::ValidationResult conflict;
    conflict.set_valid(false);
    conflict.set_reason_code(v1::REASON_CODE_ACTION_CONFLICT);
    conflict.set_message("another conflicting vehicle action is active");
    conflict.set_evaluated_state_sequence(state->sequence());
    return store_.Reject(command, conflict);
  }
  const auto receipt = store_.Accept(command);
  {
    std::lock_guard<std::mutex> lock(queue_mutex_);
    queue_.emplace_back(receipt.action_id(), command);
  }
  queue_changed_.notify_one();
  return receipt;
}

v1::ActionReceipt ActionExecutor::Cancel(
    const v1::CancelActionRequest& request) {
  if (!authority_.IsAuthorized(request.context())) {
    return SimpleReceipt(request.action_id(), v1::ACTION_STATE_REJECTED,
                         v1::REASON_CODE_CONTROL_LEASE_REQUIRED,
                         "a current control lease is required",
                         clock_.NowUnixMs());
  }
  const auto status = store_.Find(request.action_id());
  if (!status) {
    return SimpleReceipt(request.action_id(), v1::ACTION_STATE_REJECTED,
                         v1::REASON_CODE_ACTION_NOT_FOUND,
                         "action does not exist", clock_.NowUnixMs());
  }
  const auto hold = gateway_.SetMode(kModeBrake);
  if (!hold.accepted) {
    return SimpleReceipt(request.action_id(), v1::ACTION_STATE_REJECTED,
                         v1::REASON_CODE_AUTOPILOT_REJECTED,
                         "safe hold could not be established before cancellation",
                         clock_.NowUnixMs());
  }
  const auto state = state_engine_.Snapshot();
  if (!store_.Transition(request.action_id(), v1::ACTION_STATE_CANCELLED,
                         v1::REASON_CODE_OK,
                         "action cancelled after establishing hold", 1.0,
                         state ? &*state : nullptr)) {
    return SimpleReceipt(request.action_id(), v1::ACTION_STATE_REJECTED,
                         v1::REASON_CODE_ACTION_NOT_CANCELLABLE,
                         "action is already terminal", clock_.NowUnixMs());
  }
  return SimpleReceipt(request.action_id(), v1::ACTION_STATE_CANCELLED,
                       v1::REASON_CODE_OK, "action cancelled safely",
                       clock_.NowUnixMs());
}

void ActionExecutor::Run() {
  while (running_) {
    std::pair<std::string, v1::ActionCommand> item;
    {
      std::unique_lock<std::mutex> lock(queue_mutex_);
      queue_changed_.wait(lock,
                          [this] { return !running_ || !queue_.empty(); });
      if (!running_) break;
      item = std::move(queue_.front());
      queue_.pop_front();
    }
    Execute(item.first, item.second);
  }
}

void ActionExecutor::Execute(const std::string& action_id,
                             const v1::ActionCommand& command) {
  if (!store_.Transition(action_id, v1::ACTION_STATE_EXECUTING,
                         v1::REASON_CODE_OK, "executing", 0.0)) {
    return;
  }
  const auto& context = ContextOf(command);
  if (command.has_arm()) {
    if (!SetMode(action_id, kModeGuided, "GUIDED")) return;
    const auto result = gateway_.Arm(true);
    if (!result.accepted) {
      store_.Transition(action_id, v1::ACTION_STATE_FAILED,
                        result.timed_out ? v1::REASON_CODE_AUTOPILOT_TIMEOUT
                                         : v1::REASON_CODE_AUTOPILOT_REJECTED,
                        result.message);
      return;
    }
    WaitUntil(action_id, context, 45'000,
              [](const v1::DroneState& state) { return state.armed(); },
              "aircraft armed");
  } else if (command.has_disarm()) {
    const auto result = gateway_.Arm(false);
    if (!result.accepted) {
      store_.Transition(action_id, v1::ACTION_STATE_FAILED,
                        v1::REASON_CODE_AUTOPILOT_REJECTED, result.message);
      return;
    }
    WaitUntil(action_id, context, 10'000,
              [](const v1::DroneState& state) { return !state.armed(); },
              "aircraft disarmed");
  } else if (command.has_takeoff()) {
    if (!SetMode(action_id, kModeGuided, "GUIDED")) return;
    const auto result = gateway_.Takeoff(command.takeoff().target_altitude_agl_m());
    if (!result.accepted) {
      store_.Transition(action_id, v1::ACTION_STATE_FAILED,
                        v1::REASON_CODE_AUTOPILOT_REJECTED, result.message);
      return;
    }
    const double target = command.takeoff().target_altitude_agl_m();
    WaitUntil(action_id, context,
              TimeoutOf(command.takeoff().limits(), 60'000),
              [target](const v1::DroneState& state) {
                return state.relative_altitude_m() >= target * 0.95;
              },
              "takeoff altitude reached");
  } else if (command.has_goto_()) {
    ExecuteGoto(action_id, context, command.goto_().destination(),
                command.goto_().acceptance_radius_m(),
                TimeoutOf(command.goto_().limits(), 120'000));
  } else if (command.has_execute_route()) {
    const auto& route = command.execute_route();
    const auto timeout_per_point = 120'000U;
    for (int index = 0; index < route.points_size(); ++index) {
      if (!ExecuteGoto(action_id, context, route.points(index).position(),
                       route.points(index).acceptance_radius_m(),
                       TimeoutOf(route.points(index).limits(), timeout_per_point),
                       false)) {
        return;
      }
    }
    if (route.completion_behavior() ==
        v1::ROUTE_COMPLETION_BEHAVIOR_RETURN_HOME) {
      if (!SetMode(action_id, kModeRtl, "RTL")) return;
      WaitUntil(action_id, context, 120'000,
                [](const v1::DroneState& state) {
                  return state.landed() && !state.armed();
                },
                "route completed; returned home and landed");
      return;
    } else if (route.completion_behavior() ==
               v1::ROUTE_COMPLETION_BEHAVIOR_LAND) {
      if (!SetMode(action_id, kModeLand, "LAND")) return;
      WaitUntil(action_id, context, 90'000,
                [](const v1::DroneState& state) {
                  return state.landed() && !state.armed();
                },
                "route completed; landed");
      return;
    } else if (!SetMode(action_id, kModeBrake, "BRAKE")) {
      return;
    }
    const auto state = state_engine_.Snapshot();
    store_.Transition(action_id, v1::ACTION_STATE_SUCCEEDED,
                      v1::REASON_CODE_OK, "route completed", 1.0,
                      state ? &*state : nullptr);
  } else if (command.has_hold()) {
    if (!SetMode(action_id, kModeBrake, "BRAKE")) return;
    const auto duration = command.hold().has_duration_ms()
                              ? command.hold().duration_ms()
                              : std::numeric_limits<std::uint32_t>::max();
    const auto hold_started = std::chrono::steady_clock::now();
    WaitUntil(action_id, context, duration,
              [hold_started, duration](const v1::DroneState&) {
                return duration != std::numeric_limits<std::uint32_t>::max() &&
                       std::chrono::steady_clock::now() - hold_started >=
                           std::chrono::milliseconds(duration);
              },
              "hold completed");
  } else if (command.has_return_home()) {
    if (command.return_home().completion_behavior() ==
        v1::RETURN_COMPLETION_BEHAVIOR_HOLD_AT_HOME) {
      const auto state = state_engine_.Snapshot();
      if (!state || !state->has_home()) return;
      ExecuteGoto(action_id, context, state->home(), 3.0, 120'000);
    } else {
      if (!SetMode(action_id, kModeRtl, "RTL")) return;
      WaitUntil(action_id, context, 120'000,
                [](const v1::DroneState& state) {
                  return state.landed() && !state.armed();
                },
                "returned home and landed");
    }
  } else if (command.has_land()) {
    if (!SetMode(action_id, kModeLand, "LAND")) return;
    WaitUntil(action_id, context, 90'000,
              [](const v1::DroneState& state) {
                return state.landed() && !state.armed();
              },
              "landed and disarmed");
  } else if (command.has_orbit()) {
    const auto& orbit = command.orbit();
    constexpr int kSegmentsPerRevolution = 24;
    const int segment_count = static_cast<int>(
        std::ceil(kSegmentsPerRevolution * orbit.revolutions()));
    for (int index = 0; index < segment_count; ++index) {
      const double fraction = orbit.clockwise() ? -1.0 : 1.0;
      const double angle =
          fraction * 2.0 * kPi * index / kSegmentsPerRevolution;
      v1::Position point;
      if (orbit.center().has_local_ned()) {
        auto* local = point.mutable_local_ned();
        local->set_origin_id(orbit.center().local_ned().origin_id());
        local->set_north_m(orbit.center().local_ned().north_m() +
                           orbit.radius_m() * std::cos(angle));
        local->set_east_m(orbit.center().local_ned().east_m() +
                          orbit.radius_m() * std::sin(angle));
        local->set_down_m(-orbit.altitude_m());
      } else {
        auto* geo = point.mutable_geo();
        *geo = orbit.center().geo();
        geo->set_latitude_deg(
            geo->latitude_deg() +
            (orbit.radius_m() * std::cos(angle) / kEarthRadiusM) * 180.0 /
                kPi);
        const double latitude = orbit.center().geo().latitude_deg() * kPi / 180.0;
        geo->set_longitude_deg(
            geo->longitude_deg() +
            (orbit.radius_m() * std::sin(angle) /
             (kEarthRadiusM * std::cos(latitude))) *
                180.0 / kPi);
        geo->set_altitude_m(orbit.altitude_m());
        geo->set_frame(orbit.altitude_frame());
      }
      if (!ExecuteGoto(action_id, context, point, 2.0, 60'000, false)) return;
    }
    const auto state = state_engine_.Snapshot();
    store_.Transition(action_id, v1::ACTION_STATE_SUCCEEDED,
                      v1::REASON_CODE_OK, "orbit completed", 1.0,
                      state ? &*state : nullptr);
  }
}

bool ActionExecutor::WaitUntil(
    const std::string& action_id, const v1::CommandContext& context,
    std::uint32_t timeout_ms,
    const std::function<bool(const v1::DroneState&)>& predicate,
    const std::string& completion_message, bool complete_action) {
  const auto started = std::chrono::steady_clock::now();
  while (running_) {
    const auto status = store_.Find(action_id);
    if (!status || status->state() != v1::ACTION_STATE_EXECUTING) return false;
    const auto state = state_engine_.Snapshot();
    if (!state || !state_engine_.IsFresh(guardrails_.policy().maximum_state_age_ms)) {
      store_.Transition(action_id, v1::ACTION_STATE_ABORTED_BY_SAFETY,
                        v1::REASON_CODE_STATE_STALE,
                        "state became stale during execution", 0.0,
                        state ? &*state : nullptr);
      gateway_.SetMode(kModeBrake);
      return false;
    }
    if (!authority_.IsAuthorized(context) || state->manual_override_active()) {
      store_.Transition(action_id, v1::ACTION_STATE_PREEMPTED,
                        state->manual_override_active()
                            ? v1::REASON_CODE_MANUAL_OVERRIDE
                            : v1::REASON_CODE_CONTROL_LEASE_EXPIRED,
                        "control authority changed during execution", 0.0,
                        &*state);
      gateway_.SetMode(kModeBrake);
      return false;
    }
    if (predicate(*state)) {
      if (complete_action) {
        store_.Transition(action_id, v1::ACTION_STATE_SUCCEEDED,
                          v1::REASON_CODE_OK, completion_message, 1.0, &*state);
      }
      return true;
    }
    if (std::chrono::steady_clock::now() - started >=
        std::chrono::milliseconds(timeout_ms)) {
      store_.Transition(action_id, v1::ACTION_STATE_TIMED_OUT,
                        v1::REASON_CODE_COMPLETION_TIMEOUT,
                        "physical completion condition timed out", 0.0,
                        &*state);
      gateway_.SetMode(kModeBrake);
      return false;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(50));
  }
  return false;
}

bool ActionExecutor::ExecuteGoto(const std::string& action_id,
                                 const v1::CommandContext& context,
                                 const v1::Position& destination,
                                 double acceptance_radius_m,
                                 std::uint32_t timeout_ms,
                                 bool complete_action) {
  if (!SetMode(action_id, kModeGuided, "GUIDED")) return false;
  if (destination.has_local_ned() && perception_ != nullptr && planner_ != nullptr) {
    const auto state = state_engine_.Snapshot();
    if (!state || !state->has_local_position_ned() || !perception_->Healthy()) {
      store_.Transition(action_id, v1::ACTION_STATE_ABORTED_BY_SAFETY,
                        v1::REASON_CODE_SAFETY_INTERVENTION,
                        "navigation blocked: perception unavailable");
      gateway_.SetMode(kModeBrake);
      return false;
    }
    const auto plan = planner_->Plan(state->local_position_ned(),
                                     destination.local_ned(),
                                     perception_->Obstacles());
    if (!plan.success) {
      store_.Transition(action_id, v1::ACTION_STATE_ABORTED_BY_SAFETY,
                        v1::REASON_CODE_SAFETY_INTERVENTION,
                        "navigation blocked: " + plan.reason);
      gateway_.SetMode(kModeBrake);
      return false;
    }
    for (std::size_t index = 0; index < plan.waypoints.size(); ++index) {
      const auto waypoint = plan.waypoints[index];
      if (!gateway_.GotoLocal(waypoint)) {
        store_.Transition(action_id, v1::ACTION_STATE_FAILED,
                          v1::REASON_CODE_AUTOPILOT_REJECTED,
                          "failed to send planned position target");
        return false;
      }
      const bool last = index + 1 == plan.waypoints.size();
      if (!WaitUntil(
              action_id, context, timeout_ms,
              [waypoint, acceptance_radius_m](const v1::DroneState& current) {
                if (!current.has_local_position_ned()) return false;
                const auto& actual = current.local_position_ned();
                return std::sqrt(
                           std::pow(waypoint.north_m() - actual.north_m(), 2) +
                           std::pow(waypoint.east_m() - actual.east_m(), 2) +
                           std::pow(waypoint.down_m() - actual.down_m(), 2)) <=
                       acceptance_radius_m;
              },
              last && !plan.direct_path ? "destination reached via safe detour"
                                        : "destination reached",
              last ? complete_action : false)) {
        return false;
      }
    }
    return true;
  }
  const bool sent = destination.has_geo()
                        ? gateway_.GotoGlobal(destination.geo())
                        : gateway_.GotoLocal(destination.local_ned());
  if (!sent) {
    store_.Transition(action_id, v1::ACTION_STATE_FAILED,
                      v1::REASON_CODE_AUTOPILOT_REJECTED,
                      "failed to send position target");
    return false;
  }
  return WaitUntil(
      action_id, context, timeout_ms,
      [destination, acceptance_radius_m](const v1::DroneState& state) {
        if (destination.has_local_ned() && state.has_local_position_ned()) {
          const auto& target = destination.local_ned();
          const auto& actual = state.local_position_ned();
          return std::sqrt(std::pow(target.north_m() - actual.north_m(), 2) +
                           std::pow(target.east_m() - actual.east_m(), 2) +
                           std::pow(target.down_m() - actual.down_m(), 2)) <=
                 acceptance_radius_m;
        }
        if (destination.has_geo() && state.position().has_geo()) {
          const double horizontal =
              GeoDistanceM(destination.geo(), state.position().geo());
          double target_altitude = destination.geo().altitude_m();
          double actual_altitude = state.position().geo().altitude_m();
          if (destination.geo().frame() ==
              v1::COORDINATE_FRAME_WGS84_ABOVE_HOME) {
            actual_altitude = state.relative_altitude_m();
          }
          return horizontal <= acceptance_radius_m &&
                 std::abs(target_altitude - actual_altitude) <=
                     acceptance_radius_m;
        }
        return false;
      },
      "destination reached", complete_action);
}

bool ActionExecutor::SetMode(const std::string& action_id, std::uint32_t mode,
                             const std::string& name) {
  const auto result = gateway_.SetMode(mode);
  if (result.accepted) return true;
  store_.Transition(action_id, v1::ACTION_STATE_FAILED,
                    result.timed_out ? v1::REASON_CODE_AUTOPILOT_TIMEOUT
                                     : v1::REASON_CODE_AUTOPILOT_REJECTED,
                    name + " mode failed: " + result.message);
  return false;
}

const v1::CommandContext& ActionExecutor::ContextOf(
    const v1::ActionCommand& command) {
  switch (command.request_case()) {
    case v1::ActionCommand::kArm:
      return command.arm().context();
    case v1::ActionCommand::kDisarm:
      return command.disarm().context();
    case v1::ActionCommand::kTakeoff:
      return command.takeoff().context();
    case v1::ActionCommand::kGoto:
      return command.goto_().context();
    case v1::ActionCommand::kExecuteRoute:
      return command.execute_route().context();
    case v1::ActionCommand::kHold:
      return command.hold().context();
    case v1::ActionCommand::kReturnHome:
      return command.return_home().context();
    case v1::ActionCommand::kLand:
      return command.land().context();
    case v1::ActionCommand::kOrbit:
      return command.orbit().context();
    default:
      throw std::invalid_argument("action command is empty");
  }
}

double ActionExecutor::GeoDistanceM(const v1::GeoPosition& first,
                                    const v1::GeoPosition& second) {
  const double latitude_delta =
      (second.latitude_deg() - first.latitude_deg()) * kPi / 180.0;
  const double longitude_delta =
      (second.longitude_deg() - first.longitude_deg()) * kPi / 180.0;
  const double latitude_a = first.latitude_deg() * kPi / 180.0;
  const double latitude_b = second.latitude_deg() * kPi / 180.0;
  const double value =
      std::sin(latitude_delta / 2.0) * std::sin(latitude_delta / 2.0) +
      std::cos(latitude_a) * std::cos(latitude_b) *
          std::sin(longitude_delta / 2.0) *
          std::sin(longitude_delta / 2.0);
  return 2.0 * kEarthRadiusM * std::asin(std::sqrt(value));
}

}  // namespace icarus::mission_executor

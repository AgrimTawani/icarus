#include "mission_executor/action_executor.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <limits>
#include <optional>
#include <stdexcept>

namespace icarus::mission_executor {
namespace {

constexpr std::uint32_t kModeGuided = 4;
constexpr std::uint32_t kModeRtl = 6;
constexpr std::uint32_t kModeLand = 9;
constexpr std::uint32_t kModeBrake = 17;
constexpr double kEarthRadiusM = 6'371'000.0;
constexpr double kPi = 3.14159265358979323846;
// A viewpoint is a camera pose, so it is held tighter than a transit
// waypoint and must settle before its dwell starts.
constexpr double kViewpointAcceptanceM = 0.75;
constexpr std::uint32_t kViewpointSettleTimeoutMs = 20'000;

std::uint32_t TimeoutOf(const v1::ActionLimits& limits,
                        std::uint32_t fallback) {
  return limits.has_execution_timeout_ms() ? limits.execution_timeout_ms()
                                            : fallback;
}

double GroundSpeedOf(const v1::ActionLimits& limits, double fallback = 3.0) {
  return limits.has_maximum_ground_speed_mps()
             ? limits.maximum_ground_speed_mps() : fallback;
}

double ClearanceOf(const v1::ActionLimits& limits) {
  return limits.has_minimum_clearance_m() ? limits.minimum_clearance_m() : 0.0;
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

bool ClosedOrbit(const v1::OrbitRequest& orbit) {
  return orbit.revolutions() >= 1.0 - 1e-9;
}

// Arcs between consecutive viewpoints: N on a closed circle (the last one
// returns to the first), N - 1 on a partial arc (both ends are viewpoints).
int ViewpointIntervals(const v1::OrbitRequest& orbit) {
  const int count = static_cast<int>(orbit.viewpoints());
  if (count <= 0) return 0;
  return ClosedOrbit(orbit) ? count : std::max(1, count - 1);
}

// Keep adjacent acceptance regions separate; full-segment radii let targets
// race ahead of the aircraft and accumulate lateral momentum.
double OrbitAcceptanceRadiusM(const v1::OrbitRequest& orbit) {
  const double segment_angle =
      2.0 * kPi * orbit.revolutions() / OrbitSegmentCount(orbit);
  const double chord = 2.0 * orbit.radius_m() * std::sin(segment_angle / 2.0);
  return std::min(0.75, chord * 0.4);
}

}  // namespace

int OrbitSegmentCount(const v1::OrbitRequest& orbit) {
  const int base = std::max(
      1, static_cast<int>(std::ceil(
             kOrbitSegmentsPerRevolution * orbit.revolutions() - 1e-9)));
  const int intervals = ViewpointIntervals(orbit);
  if (intervals <= 0) return base;
  return intervals *
         static_cast<int>(std::ceil(static_cast<double>(base) / intervals));
}

int OrbitViewpointAt(const v1::OrbitRequest& orbit, int index) {
  const int intervals = ViewpointIntervals(orbit);
  if (intervals <= 0) return 0;
  const int step = OrbitSegmentCount(orbit) / intervals;
  if (index < 0 || index % step != 0) return 0;
  const int number = index / step + 1;
  return number <= static_cast<int>(orbit.viewpoints()) ? number : 0;
}

std::optional<double> HeadingTowardDeg(const v1::Position& from,
                                       const v1::Position& target) {
  const auto offset = guardrails::HorizontalOffsetM(from, target);
  if (!offset || std::hypot(offset->first, offset->second) <
                     guardrails::kMinimumFacePointDistanceM) {
    return std::nullopt;
  }
  double heading = std::atan2(offset->second, offset->first) * 180.0 / kPi;
  if (heading < 0.0) heading += 360.0;
  return heading;
}

double EnforcedClearanceM(double required_m, double best_observed_m) {
  return std::min(required_m, best_observed_m - kClearanceToleranceM);
}

v1::Position OrbitWaypoint(const v1::OrbitRequest& orbit,
                           double start_angle_rad, int index) {
  // Azimuth increases north -> east, which is clockwise seen from above.
  const double direction = orbit.clockwise() ? 1.0 : -1.0;
  const double angle = start_angle_rad +
      direction * 2.0 * kPi * orbit.revolutions() * index /
          OrbitSegmentCount(orbit);
  v1::Position point;
  if (orbit.center().has_local_ned()) {
    auto* local = point.mutable_local_ned();
    local->set_origin_id(orbit.center().local_ned().origin_id());
    local->set_north_m(orbit.center().local_ned().north_m() +
                       orbit.radius_m() * std::cos(angle));
    local->set_east_m(orbit.center().local_ned().east_m() +
                      orbit.radius_m() * std::sin(angle));
    // Pure geometry returns an explicitly framed target, never pretends that
    // above-home altitude is the negative EKF-origin down coordinate.
    local->set_altitude_above_home_m(orbit.altitude_m());
  } else {
    auto* geo = point.mutable_geo();
    *geo = orbit.center().geo();
    geo->set_latitude_deg(
        geo->latitude_deg() +
        (orbit.radius_m() * std::cos(angle) / kEarthRadiusM) * 180.0 / kPi);
    const double latitude = orbit.center().geo().latitude_deg() * kPi / 180.0;
    geo->set_longitude_deg(
        geo->longitude_deg() +
        (orbit.radius_m() * std::sin(angle) /
         (kEarthRadiusM * std::cos(latitude))) *
            180.0 / kPi);
    geo->set_altitude_m(orbit.altitude_m());
    geo->set_frame(orbit.altitude_frame() == v1::COORDINATE_FRAME_UNSPECIFIED
                       ? v1::COORDINATE_FRAME_WGS84_ABOVE_HOME
                       : orbit.altitude_frame());
  }
  return point;
}

NativeRoutePlan BuildNativeRoute(const v1::ExecuteRouteRequest& route,
                                 const v1::DroneState& state) {
  if (!state.has_position() || !state.position().has_geo() ||
      !state.has_local_position_ned() || state.relative_altitude_received_at_unix_ms()<=0 ||
      !std::isfinite(state.relative_altitude_m()))
    throw std::invalid_argument("native mission needs measured geographic/local position and altitude");
  const auto& anchor = state.position().geo();
  const double lat = anchor.latitude_deg() * kPi / 180;
  if (!std::isfinite(lat) || std::abs(anchor.latitude_deg())>90 ||
      !std::isfinite(anchor.longitude_deg()) || std::abs(anchor.longitude_deg())>180 ||
      std::abs(std::cos(lat)) < 0.001)
    throw std::invalid_argument("native mission geographic anchor is unavailable near pole");
  NativeRoutePlan plan;
  // ArduPilot reserves mission item zero for home. Do not redefine home.
  mavlink_gateway::MissionItem home;
  home.command = 16; home.frame = 6;
  home.x = std::lround(anchor.latitude_deg() * 1e7);
  home.y = std::lround(anchor.longitude_deg() * 1e7);
  home.z = state.relative_altitude_m();
  plan.items.push_back(home);
  // Activate AUTO at the measured current pose. BRAKE does not accept a speed
  // change; this stationary checkpoint permits setting speed before departure.
  auto activation = home;
  activation.command = 17;
  plan.items.push_back(activation);
  for (const auto& point : route.points()) {
    mavlink_gateway::MissionItem item;
    // Stopped observation stations remain in AUTO LOITER_UNLIM until measured
    // arrival/heading/dwell are verified. Fly-through points progress natively.
    item.command = point.fly_through() ? 16 : 17;
    item.frame = 6;  // MAV_FRAME_GLOBAL_RELATIVE_ALT_INT
    double latitude, longitude, altitude;
    if (point.position().has_local_ned()) {
      const auto& p = point.position().local_ned();
      const auto& origin = state.local_position_ned();
      if (p.origin_id() != origin.origin_id())
        throw std::invalid_argument("native route origin changed before upload");
      latitude = anchor.latitude_deg() + (p.north_m()-origin.north_m()) / kEarthRadiusM * 180/kPi;
      longitude = anchor.longitude_deg() + (p.east_m()-origin.east_m()) / (kEarthRadiusM*std::cos(lat)) * 180/kPi;
      altitude = p.has_altitude_above_home_m() ? p.altitude_above_home_m()
                 : state.relative_altitude_m() + origin.down_m() - p.down_m();
    } else if (point.position().has_geo()) {
      const auto& p = point.position().geo();
      latitude=p.latitude_deg(); longitude=p.longitude_deg(); altitude=p.altitude_m();
      if (p.frame()==v1::COORDINATE_FRAME_WGS84_AMSL && state.has_home() && state.home().has_geo() &&
          state.home().geo().frame()==v1::COORDINATE_FRAME_WGS84_AMSL)
        altitude -= state.home().geo().altitude_m();
      else if (p.frame()!=v1::COORDINATE_FRAME_WGS84_ABOVE_HOME)
        throw std::invalid_argument("native route requires above-home or AMSL altitude");
    } else throw std::invalid_argument("native route point is missing position");
    if (!std::isfinite(latitude) || !std::isfinite(longitude) || !std::isfinite(altitude) ||
        std::abs(latitude)>90 || (point.position().has_geo() && std::abs(longitude)>180))
      throw std::invalid_argument("native route coordinates must be finite");
    longitude=std::remainder(longitude,360.0);
    item.x=std::lround(latitude*1e7); item.y=std::lround(longitude*1e7); item.z=altitude;
    plan.waypoint_sequences.push_back(plan.items.size());
    plan.items.push_back(item);
  }
  if (route.points_size()==0) throw std::invalid_argument("empty native route");
  // Never let AUTO fall off the end into an unspecified mission-end behavior.
  auto terminal=plan.items.back(); terminal.command=17;
  plan.items.push_back(terminal);
  return plan;
}

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
  if (status->state()!=v1::ACTION_STATE_EXECUTING && status->state()!=v1::ACTION_STATE_ACCEPTED &&
      status->state()!=v1::ACTION_STATE_QUEUED && status->state()!=v1::ACTION_STATE_VALIDATING)
    return SimpleReceipt(request.action_id(),v1::ACTION_STATE_REJECTED,
                         v1::REASON_CODE_ACTION_NOT_CANCELLABLE,"action is already terminal",clock_.NowUnixMs());
  struct PendingCancel {
    std::mutex& mutex; std::condition_variable& changed;
    std::unordered_set<std::string>& pending; std::string id;
    ~PendingCancel() { std::lock_guard<std::mutex> lock(mutex); pending.erase(id); changed.notify_all(); }
  } pending_cancel{cancellation_mutex_,cancellation_changed_,cancellation_pending_,request.action_id()};
  {
    std::lock_guard<std::mutex> lock(cancellation_mutex_);
    cancellation_pending_.insert(request.action_id());
  }
  const auto cancel_state = state_engine_.Snapshot();
  const bool passive_hold = status->type() == v1::ACTION_TYPE_HOLD && cancel_state &&
      state_engine_.IsFresh(guardrails_.policy().maximum_state_age_ms) &&
      guardrails::IsPassiveGroundHoldState(*cancel_state);
  const auto hold = passive_hold
      ? mavlink_gateway::CommandResult{true, false, 0, "grounded hold needs no mode command"}
      : gateway_.SetMode(kModeBrake);
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
  navigation_prepared_ = false;
  BeginClearanceGuard(command);
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
    const auto initial_position = state_engine_.Snapshot();
    const auto resolved = initial_position
        ? guardrails::ResolveLocalAltitude(command.goto_().destination(), *initial_position)
        : std::nullopt;
    if (!resolved) {
      store_.Transition(action_id, v1::ACTION_STATE_ABORTED_BY_SAFETY,
                        v1::REASON_CODE_POSITION_UNAVAILABLE,
                        "navigation altitude transform is unavailable");
      gateway_.SetMode(kModeBrake);
      return;
    }
    const auto destination = *resolved;
    const double acceptance_radius_m = command.goto_().acceptance_radius_m();
    const auto timeout_ms = TimeoutOf(command.goto_().limits(), 120'000);
    auto heading = command.goto_().heading();
    std::optional<double> retained_heading;
    if (heading.mode() == v1::HEADING_MODE_KEEP_CURRENT) {
      const auto initial = state_engine_.Snapshot();
      if (!initial || !initial->has_attitude() ||
          !std::isfinite(initial->attitude().yaw_rad())) {
        store_.Transition(action_id, v1::ACTION_STATE_ABORTED_BY_SAFETY,
                          v1::REASON_CODE_STATE_STALE,
                          "current heading is unavailable for navigation");
        gateway_.SetMode(kModeBrake);
        return;
      }
      retained_heading = std::fmod(initial->attitude().yaw_rad() * 180.0 / kPi +
                                      360.0, 360.0);
      heading.set_mode(v1::HEADING_MODE_ABSOLUTE);
      heading.set_heading_deg(*retained_heading);
    }
    if (!ExecuteGoto(action_id, context, destination,
                     acceptance_radius_m,
                     timeout_ms,
                     false, ClearanceOf(command.goto_().limits()), retained_heading,
                     GroundSpeedOf(command.goto_().limits()))) {
      return;
    }
    if (!ApplyHeading(action_id, context, heading, destination)) {
      return;
    }
    // Crossing the edge of an acceptance sphere at flight speed is not a
    // stable observation waypoint.  Keep the GUIDED position target active
    // until position, horizontal/vertical velocity and heading have remained
    // settled for a short dwell.  This prevents camera/VLM actions from
    // sampling while the vehicle is still overshooting the waypoint.
    std::optional<std::chrono::steady_clock::time_point> stable_since;
    WaitUntil(
        action_id, context, timeout_ms,
        [destination, acceptance_radius_m,
         stable_since](const v1::DroneState& state) mutable {
          const bool position_reached =
              Reached(destination, state, acceptance_radius_m);
          const bool motion_settled =
              state.has_velocity_ned_mps() && state.ground_speed_mps() <= 0.35 &&
              std::abs(state.velocity_ned_mps().z()) <= 0.35;
          if (!position_reached || !motion_settled) {
            stable_since.reset();
            return false;
          }
          const auto now = std::chrono::steady_clock::now();
          if (!stable_since) stable_since = now;
          return now - *stable_since >= std::chrono::milliseconds(500);
        },
        "destination and heading reached; vehicle settled", true);
  } else if (command.has_execute_route()) {
    const auto& route = command.execute_route();
    if (route.delegate_to_autopilot()) {
      ExecuteNativeRoute(action_id, route);
      return;
    }
    const auto timeout_per_point = 120'000U;
    for (int index = 0; index < route.points_size(); ++index) {
      auto heading = route.points(index).heading();
      if (heading.mode()==v1::HEADING_MODE_KEEP_CURRENT) {
        const auto snapshot=state_engine_.Snapshot();
        if (!snapshot || !snapshot->has_attitude()) return;
        heading.set_mode(v1::HEADING_MODE_ABSOLUTE);
        heading.set_heading_deg(std::fmod(snapshot->attitude().yaw_rad()*180/kPi+360.0,360.0));
      }
      if (!ExecuteGoto(action_id, context, route.points(index).position(),
                       route.points(index).acceptance_radius_m(),
                       TimeoutOf(route.points(index).limits(), timeout_per_point),
                       false, ClearanceOf(route.points(index).limits()), std::nullopt,
                       GroundSpeedOf(route.points(index).limits()))) {
        return;
      }
      const auto snapshot=state_engine_.Snapshot();
      if (!snapshot) return;
      const auto resolved = guardrails::ResolveLocalAltitude(route.points(index).position(), *snapshot);
      if (!resolved || !ApplyHeading(action_id, context, heading, *resolved)) return;
      if (route.points(index).hold_time_ms() > 0 &&
          !HoldViewpoint(action_id, context, *resolved, std::nullopt,
                         route.points(index).hold_time_ms(), index+1, route.points_size(),
                         double(index+1)/route.points_size())) return;
    }
    if (route.completion_behavior() ==
        v1::ROUTE_COMPLETION_BEHAVIOR_RETURN_HOME) {
      if (!SetMode(action_id, kModeRtl, "RTL")) return;
      WaitForLanding(action_id, context, 120'000, "route completed; returned home and landed");
      return;
    } else if (route.completion_behavior() ==
               v1::ROUTE_COMPLETION_BEHAVIOR_LAND) {
      if (!SetMode(action_id, kModeLand, "LAND")) return;
      WaitForLanding(action_id, context, 90'000, "route completed; landed");
      return;
    } else if (!SetMode(action_id, kModeBrake, "BRAKE")) {
      return;
    }
    const auto state = state_engine_.Snapshot();
    store_.Transition(action_id, v1::ACTION_STATE_SUCCEEDED,
                      v1::REASON_CODE_OK, "route completed", 1.0,
                      state ? &*state : nullptr);
  } else if (command.has_hold()) {
    const auto hold_state = state_engine_.Snapshot();
    const bool passive_hold = hold_state && guardrails::IsPassiveGroundHoldState(*hold_state);
    if (!hold_state || (!hold_state->armed() && !passive_hold)) {
      store_.Transition(action_id, v1::ACTION_STATE_ABORTED_BY_SAFETY,
                        v1::REASON_CODE_NOT_ARMED, "hold state is neither armed nor safely grounded");
      return;
    }
    if (!passive_hold && !SetMode(action_id, kModeBrake, "BRAKE")) return;
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
              "hold completed", true, passive_hold);
  } else if (command.has_return_home()) {
    if (command.return_home().completion_behavior() ==
        v1::RETURN_COMPLETION_BEHAVIOR_HOLD_AT_HOME) {
      const auto state = state_engine_.Snapshot();
      if (!state || !state->has_home()) return;
      ExecuteGoto(action_id, context, state->home(), 3.0, 120'000);
    } else {
      if (!SetMode(action_id, kModeRtl, "RTL")) return;
      WaitForLanding(action_id, context, 120'000, "returned home and landed");
    }
  } else if (command.has_land()) {
    if (!SetMode(action_id, kModeLand, "LAND")) return;
    WaitForLanding(action_id, context, 90'000, "landed and disarmed");
  } else if (command.has_orbit()) {
    const auto& orbit = command.orbit();
    const int segment_count = OrbitSegmentCount(orbit);
    // Enter the circle at the point nearest the aircraft. Starting every
    // local orbit at angle zero can command a path straight through the
    // inspected object when the aircraft is waiting on its opposite side.
    double start_angle = 0.0;
    const auto initial_state = state_engine_.Snapshot();
    const auto altitude_position = initial_state
        ? guardrails::OrbitAltitudePosition(orbit, *initial_state) : std::nullopt;
    if (!altitude_position) {
      store_.Transition(action_id, v1::ACTION_STATE_ABORTED_BY_SAFETY,
                        v1::REASON_CODE_POSITION_UNAVAILABLE,
                        "orbit altitude transform is unavailable");
      gateway_.SetMode(kModeBrake);
      return;
    }
    if (initial_state && orbit.center().has_local_ned() &&
        initial_state->has_local_position_ned()) {
      const auto& center = orbit.center().local_ned();
      const auto& current = initial_state->local_position_ned();
      start_angle = std::atan2(current.east_m() - center.east_m(),
                               current.north_m() - center.north_m());
    } else if (initial_state && orbit.center().has_geo() &&
               initial_state->position().has_geo()) {
      const auto& center = orbit.center().geo();
      const auto& current = initial_state->position().geo();
      const double north =
          (current.latitude_deg() - center.latitude_deg()) * kPi / 180.0 *
          kEarthRadiusM;
      const double east =
          (current.longitude_deg() - center.longitude_deg()) * kPi / 180.0 *
          kEarthRadiusM * std::cos(center.latitude_deg() * kPi / 180.0);
      start_angle = std::atan2(east, north);
    }
    // Without a heading policy the autopilot keeps looking along the path.
    // FACE_POINT commands, per waypoint, the absolute heading from that
    // waypoint toward the face point (the centre unless one is given), so the
    // camera stays on the inspected object all the way round.
    const auto& heading = orbit.heading();
    const bool face = heading.mode() == v1::HEADING_MODE_FACE_POINT;
    const v1::Position& focus =
        face && heading.has_face_point() ? heading.face_point() : orbit.center();
    std::optional<double> waypoint_heading;
    if (heading.mode() == v1::HEADING_MODE_ABSOLUTE &&
        heading.has_heading_deg()) {
      waypoint_heading = heading.heading_deg();
    }
    if (heading.mode() == v1::HEADING_MODE_KEEP_CURRENT) {
      if (!initial_state || !initial_state->has_attitude() ||
          !std::isfinite(initial_state->attitude().yaw_rad())) {
        store_.Transition(action_id, v1::ACTION_STATE_ABORTED_BY_SAFETY,
                          v1::REASON_CODE_STATE_STALE,
                          "current heading is unavailable for orbit");
        gateway_.SetMode(kModeBrake);
        return;
      }
      waypoint_heading = std::fmod(initial_state->attitude().yaw_rad() *
                                      180.0 / kPi + 360.0, 360.0);
    }
    if (orbit.center().has_local_ned() && orbit.viewpoints() == 0) {
      ExecuteContinuousOrbit(action_id, orbit, start_angle, *altitude_position,
                             waypoint_heading);
      return;
    }
    const double acceptance_radius_m = OrbitAcceptanceRadiusM(orbit);
    const int viewpoint_total = static_cast<int>(orbit.viewpoints());
    for (int index = 0; index <= segment_count; ++index) {
      auto point = OrbitWaypoint(orbit, start_angle, index);
      if (point.has_local_ned()) {
        point.mutable_local_ned()->set_down_m(altitude_position->local_ned().down_m());
        point.mutable_local_ned()->clear_altitude_above_home_m();
      }
      if (face) {
        // A waypoint on the face point itself keeps the previous heading.
        if (const auto toward = HeadingTowardDeg(point, focus)) {
          waypoint_heading = toward;
        }
      }
      if (!ExecuteGoto(action_id, context, point, acceptance_radius_m, 60'000,
                       false, ClearanceOf(orbit.limits()), waypoint_heading,
                       std::min(orbit.ground_speed_mps(),
                                GroundSpeedOf(orbit.limits(), orbit.ground_speed_mps())))) {
        return;
      }
      // A radius crossing at speed is not a settled arrival. Preserve each
      // polygon vertex until the measured controller state is stable.
      std::optional<std::chrono::steady_clock::time_point> stable_since;
      if (!WaitUntil(
              action_id, context, kViewpointSettleTimeoutMs,
              [point, acceptance_radius_m, stable_since](
                  const v1::DroneState& state) mutable {
                const bool settled =
                    Reached(point, state, acceptance_radius_m) &&
                    state.has_velocity_ned_mps() &&
                    state.ground_speed_mps() <= 0.35 &&
                    std::abs(state.velocity_ned_mps().z()) <= 0.35;
                if (!settled) {
                  stable_since.reset();
                  return false;
                }
                const auto now = std::chrono::steady_clock::now();
                if (!stable_since) stable_since = now;
                return now - *stable_since >= std::chrono::milliseconds(500);
              },
              "orbit waypoint physically settled", false)) {
        return;
      }
      const double progress = static_cast<double>(index) / segment_count;
      if (const int viewpoint = OrbitViewpointAt(orbit, index); viewpoint > 0) {
        if (!HoldViewpoint(action_id, context, point, waypoint_heading,
                           orbit.viewpoint_dwell_ms(), viewpoint,
                           viewpoint_total, progress)) {
          return;
        }
      } else {
        store_.UpdateProgress(action_id, progress,
                              "orbiting: waypoint " + std::to_string(index) +
                                  "/" + std::to_string(segment_count));
      }
    }
    if (heading.mode() == v1::HEADING_MODE_KEEP_CURRENT && waypoint_heading) {
      v1::HeadingPolicy final_heading;
      final_heading.set_mode(v1::HEADING_MODE_ABSOLUTE);
      final_heading.set_heading_deg(*waypoint_heading);
      if (!ApplyHeading(action_id, context, final_heading, orbit.center())) return;
    }
    const auto state = state_engine_.Snapshot();
    store_.Transition(action_id, v1::ACTION_STATE_SUCCEEDED,
                      v1::REASON_CODE_OK,
                      viewpoint_total > 0
                          ? "orbit completed; held " +
                                std::to_string(viewpoint_total) + " viewpoints"
                          : "orbit completed",
                      1.0, state ? &*state : nullptr);
  } else if (command.has_yaw()) {
    const auto initial = state_engine_.Snapshot();
    // Arming enables motors; it does not make a ground-supported quadcopter
    // capable of controlled yaw. Fail this action before any mode or movement
    // command, and leave the mission agent to clarify or choose the next step.
    if (initial && initial->landed()) {
      store_.Transition(action_id, v1::ACTION_STATE_FAILED,
                        v1::REASON_CODE_STATE_CHANGED,
                        "yaw requires an airborne aircraft; it cannot rotate while landed",
                        0.0, &*initial);
      return;
    }
    if (!SetMode(action_id, kModeGuided, "GUIDED")) return;
    if (!initial || !initial->has_attitude() ||
        !std::isfinite(initial->attitude().yaw_rad()) ||
        initial->attitude_received_at_unix_ms() <= 0 ||
        clock_.NowUnixMs() < initial->attitude_received_at_unix_ms() ||
        clock_.NowUnixMs() - initial->attitude_received_at_unix_ms() > 1'000) {
      store_.Transition(action_id, v1::ACTION_STATE_ABORTED_BY_SAFETY,
                        v1::REASON_CODE_STATE_STALE,
                        "attitude is unavailable for yaw verification");
      gateway_.SetMode(kModeBrake);
      return;
    }
    const auto& yaw = command.yaw();
    const auto result = gateway_.RotateYaw(yaw.relative_angle_deg(),
                                           yaw.rate_deg_s());
    if (!result.accepted) {
      store_.Transition(action_id, v1::ACTION_STATE_FAILED,
                        result.timed_out ? v1::REASON_CODE_AUTOPILOT_TIMEOUT
                                         : v1::REASON_CODE_AUTOPILOT_REJECTED,
                        "yaw command failed: " + result.message);
      return;
    }
    double previous = initial->attitude().yaw_rad();
    double accumulated = 0.0;
    auto previous_stamp = initial->attitude_received_at_unix_ms();
    const double target = yaw.relative_angle_deg() * kPi / 180.0;
    const double endpoint = previous + target;
    // A small request must never complete at its starting orientation.
    const double tolerance = std::min(5.0 * kPi / 180.0, std::abs(target) * 0.25);
    std::optional<std::chrono::steady_clock::time_point> stable_since;
    std::int64_t stable_stamp = 0;
    const auto timeout = static_cast<std::uint32_t>(
        std::ceil(std::abs(yaw.relative_angle_deg()) / yaw.rate_deg_s() *
                  2'000.0 + 15'000.0));
    WaitUntil(
        action_id, context, timeout,
        [this, &previous, &previous_stamp, &accumulated, target, endpoint,
         tolerance, stable_since, stable_stamp, action_id](const v1::DroneState& state) mutable {
          const auto stamp = state.attitude_received_at_unix_ms();
          if (!state.has_attitude() || !std::isfinite(state.attitude().yaw_rad()) ||
              stamp <= 0 || stamp > clock_.NowUnixMs() ||
              clock_.NowUnixMs() - stamp > 1'000 || stamp < previous_stamp ||
              stamp - previous_stamp > 1'000) {
            gateway_.SetMode(kModeBrake);
            store_.Transition(action_id, v1::ACTION_STATE_ABORTED_BY_SAFETY,
                              v1::REASON_CODE_STATE_STALE,
                              "fresh continuous attitude is unavailable for yaw verification");
            return false;
          }
          // Re-reading one sample is not evidence that the aircraft settled.
          if (stamp == previous_stamp) return false;
          const double current = state.attitude().yaw_rad();
          const double delta = std::remainder(current - previous, 2.0 * kPi);
          const double rate = std::abs(delta) * 1000.0 / (stamp - previous_stamp);
          previous = current;
          previous_stamp = stamp;
          // Backtracking subtracts progress; repeated noise cannot earn a turn.
          accumulated += delta;
          if (std::abs(accumulated - target) > tolerance ||
              std::abs(std::remainder(current - endpoint, 2.0 * kPi)) > tolerance ||
              rate > 5.0 * kPi / 180.0) {
            stable_since.reset();
            stable_stamp = 0;
            return false;
          }
          const auto now = std::chrono::steady_clock::now();
          if (!stable_since) {
            stable_since = now;
            stable_stamp = stamp;
          }
          return now - *stable_since >= std::chrono::milliseconds(300) &&
                 stamp - stable_stamp >= 300;
        },
        "relative yaw completed");
  }
}

bool ActionExecutor::ApplyHeading(const std::string& action_id,
                                  const v1::CommandContext& context,
                                  const v1::HeadingPolicy& heading,
                                  const v1::Position& from) {
  // KEEP_CURRENT is captured and converted to an absolute heading before
  // navigation. Only unspecified/face-path use ArduPilot's default yaw here.
  if (heading.mode() == v1::HEADING_MODE_UNSPECIFIED ||
      heading.mode() == v1::HEADING_MODE_FACE_PATH) {
    return true;
  }
  std::optional<double> heading_deg;
  if (heading.mode() == v1::HEADING_MODE_ABSOLUTE &&
      heading.has_heading_deg()) {
    heading_deg = heading.heading_deg();
  } else if (heading.mode() == v1::HEADING_MODE_FACE_POINT &&
             heading.has_face_point()) {
    // Computed from the commanded destination, where the aircraft settles,
    // rather than from wherever it crossed the acceptance radius.
    heading_deg = HeadingTowardDeg(from, heading.face_point());
  }
  if (!heading_deg) {
    store_.Transition(action_id, v1::ACTION_STATE_FAILED,
                      v1::REASON_CODE_INVALID_ARGUMENT,
                      "executor supports keep-current, face-path, absolute, "
                      "or a resolvable face-point heading");
    return false;
  }
  if (!CommandHeading(action_id, *heading_deg)) return false;
  const double target = *heading_deg * kPi / 180.0;
  return WaitUntil(
      action_id, context, 15'000,
      [target](const v1::DroneState& state) {
        if (!state.has_attitude()) return false;
        const double delta = std::remainder(state.attitude().yaw_rad() - target,
                                            2.0 * kPi);
        return std::abs(delta) <= 5.0 * kPi / 180.0;
      },
      "heading reached", false);
}

bool ActionExecutor::WaitUntil(
    const std::string& action_id, const v1::CommandContext& context,
    std::uint32_t timeout_ms,
    const std::function<bool(const v1::DroneState&)>& predicate,
    const std::string& completion_message, bool complete_action,
    bool passive_ground_hold) {
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
      if (!passive_ground_hold) gateway_.SetMode(kModeBrake);
      return false;
    }
    if (!authority_.IsAuthorized(context) || state->manual_override_active()) {
      store_.Transition(action_id, v1::ACTION_STATE_PREEMPTED,
                        state->manual_override_active()
                            ? v1::REASON_CODE_MANUAL_OVERRIDE
                            : v1::REASON_CODE_CONTROL_LEASE_EXPIRED,
                        "control authority changed during execution", 0.0,
                        &*state);
      if (!passive_ground_hold) gateway_.SetMode(kModeBrake);
      return false;
    }
    if (passive_ground_hold && !guardrails::IsPassiveGroundHoldState(*state)) {
      store_.Transition(action_id, v1::ACTION_STATE_ABORTED_BY_SAFETY,
                        v1::REASON_CODE_NOT_ARMED,
                        "grounded hold state changed during wait", 0.0, &*state);
      return false;
    }
    if (const auto violation = ClearanceViolation()) {
      // BRAKE first, so a client that sees the abort sees a held aircraft.
      if (!passive_ground_hold) gateway_.SetMode(kModeBrake);
      store_.Transition(action_id, v1::ACTION_STATE_ABORTED_BY_SAFETY,
                        violation->first, violation->second, 0.0, &*state);
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
      if (!passive_ground_hold) gateway_.SetMode(kModeBrake);
      return false;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(50));
  }
  return false;
}

bool ActionExecutor::WaitForLanding(
    const std::string& action_id, const v1::CommandContext& context,
    std::uint32_t timeout_ms, const std::string& message) {
  std::optional<std::chrono::steady_clock::time_point> stable_since;
  const bool finished = WaitUntil(
      action_id, context, timeout_ms,
      [this, action_id, stable_since](const v1::DroneState& state) mutable {
        const auto fresh = [&](std::int64_t received) {
          return received > 0 && received <= state.observed_at_unix_ms() &&
                 state.observed_at_unix_ms() - received <=
                     guardrails_.policy().maximum_state_age_ms;
        };
        if (!state.landed() || state.armed() || !state.has_attitude() ||
            !fresh(state.attitude_received_at_unix_ms()) ||
            !fresh(state.relative_altitude_received_at_unix_ms())) {
          stable_since.reset();
          return false;
        }
        const double roll = state.attitude().roll_rad();
        const double pitch = state.attitude().pitch_rad();
        const double upright_cosine = std::cos(roll) * std::cos(pitch);
        if (!std::isfinite(roll) || !std::isfinite(pitch) ||
            upright_cosine < std::cos(guardrails_.policy().maximum_tilt_deg * kPi / 180.0)) {
          store_.Transition(action_id, v1::ACTION_STATE_FAILED,
                            v1::REASON_CODE_SAFETY_INTERVENTION,
                            "abnormal landing: disarmed aircraft is not upright", 0.0, &state);
          return true;
        }
        if (!state.has_velocity_ned_mps() ||
            !std::isfinite(state.ground_speed_mps()) ||
            !std::isfinite(state.velocity_ned_mps().z()) ||
            state.ground_speed_mps() > 0.35 ||
            std::abs(state.velocity_ned_mps().z()) > 0.35) {
          stable_since.reset();
          return false;
        }
        const auto now = std::chrono::steady_clock::now();
        if (!stable_since) stable_since = now;
        return now - *stable_since >= std::chrono::seconds(1);
      }, message, false);
  const auto status = store_.Find(action_id);
  if (!finished || !status || status->state() != v1::ACTION_STATE_EXECUTING) return false;
  const auto state = state_engine_.Snapshot();
  store_.Transition(action_id, v1::ACTION_STATE_SUCCEEDED, v1::REASON_CODE_OK,
                    message + "; upright and settled", 1.0, state ? &*state : nullptr);
  return true;
}

bool ActionExecutor::ExecuteNativeRoute(const std::string& action_id,
                                        const v1::ExecuteRouteRequest& route) {
  struct AutoRelease {
    mavlink_gateway::MavlinkGateway& gateway;
    std::optional<std::uint64_t> generation;
    ~AutoRelease() { if (generation) gateway.ReleaseMissionControl(*generation); }
  } release{gateway_, std::nullopt};
  const auto fail = [&](const std::string& message) {
    {
      // The cancelling RPC owns the terminal state until its BRAKE result.
      // A denied/failed cancellation releases this wait and leaves a concrete
      // FAILED outcome rather than stranding an EXECUTING action.
      std::unique_lock<std::mutex> lock(cancellation_mutex_);
      cancellation_changed_.wait(lock,[&]{return !cancellation_pending_.count(action_id);});
    }
    const auto status=store_.Find(action_id);
    if (!status || status->state()!=v1::ACTION_STATE_EXECUTING) return false;
    if (release.generation) gateway_.ReleaseMissionControl(*release.generation);
    store_.Transition(action_id, v1::ACTION_STATE_FAILED,
                      v1::REASON_CODE_AUTOPILOT_REJECTED, message);
    return false;
  };
  const auto authorized = [&] {
    {
      std::lock_guard<std::mutex> lock(cancellation_mutex_);
      if (cancellation_pending_.count(action_id)) return false;
    }
    const auto status=store_.Find(action_id);
    return running_ && status && status->state()==v1::ACTION_STATE_EXECUTING &&
           authority_.IsAuthorized(route.context());
  };
  const auto initial=state_engine_.Snapshot();
  if (!initial) return fail("native route requires measured state");
  const auto measurement_valid = [&](const v1::DroneState& state, bool heading_required) {
    const auto now=clock_.NowUnixMs();
    const auto fresh=[&](std::int64_t stamp) {
      return stamp>0 && stamp<=now && now-stamp<=guardrails_.policy().maximum_state_age_ms;
    };
    const auto& local=state.local_position_ned();
    return state.has_local_position_ned() &&
      local.origin_id()==initial->local_position_ned().origin_id() &&
      std::isfinite(local.north_m()) && std::isfinite(local.east_m()) && std::isfinite(local.down_m()) &&
      fresh(state.local_position_received_at_unix_ms()) &&
      fresh(state.relative_altitude_received_at_unix_ms()) &&
      std::isfinite(state.relative_altitude_m()) && state.has_velocity_ned_mps() &&
      std::isfinite(state.ground_speed_mps()) && std::isfinite(state.velocity_ned_mps().z()) &&
      (!heading_required || (state.has_attitude() && std::isfinite(state.attitude().yaw_rad()) &&
                            fresh(state.attitude_received_at_unix_ms())));
  };
  bool needs_heading=false;
  for (const auto& point:route.points())
    needs_heading=needs_heading || (point.heading().mode()!=v1::HEADING_MODE_UNSPECIFIED &&
                                   point.heading().mode()!=v1::HEADING_MODE_FACE_PATH);
  if (!measurement_valid(*initial,needs_heading)) return fail("native route measurements stale or invalid");

  NativeRoutePlan plan;
  try { plan=BuildNativeRoute(route, *initial); }
  catch (const std::exception& error) { return fail(error.what()); }
  // Uploading is deliberately not authority to fly. Recheck cancellation and
  // lease after the potentially blocking handshake, before entering AUTO.
  const auto uploaded=gateway_.UploadMission(plan.items);
  if (!uploaded.accepted) return fail("native mission upload failed: "+uploaded.message);
  if (!WaitUntil(action_id, route.context(), 1000, [](const auto&) { return true; }, "", false)) return false;
  const auto started=gateway_.StartMissionGuarded(1,authorized);
  if (started.activation_attempted) release.generation=started.mode_generation;
  if (!started.accepted) return fail("native mission start failed: "+started.message);
  // AUTO is stationary at item 1 until a speed ACK and guarded advancement.
  if (!gateway_.SetGroundSpeed(GroundSpeedOf(route.points(0).limits())).accepted)
    return fail("native mission speed command rejected");
  if (!authorized()) return fail("native mission activation cancelled");
  std::uint64_t generation=gateway_.GetMissionProgress().revision;
  for (int i=0; i<route.points_size(); ++i) {
    const auto& point=route.points(i);
    const auto seq=plan.waypoint_sequences[i];
    const auto current_state=state_engine_.Snapshot();
    if (!current_state) return fail("native route state unavailable");
    const auto resolved=guardrails::ResolveLocalAltitude(point.position(), *current_state);
    if (!resolved) return fail("native route altitude transform unavailable");
    if (!WaitUntil(action_id,route.context(),1000,[](const auto&){return true;},"",false)) return false;
    if (i>0 && !gateway_.SetGroundSpeed(GroundSpeedOf(point.limits())).accepted)
      return fail("native mission speed command rejected");
    if (!authorized()) return fail("native mission advancement cancelled");
    if (i==0 || !route.points(i-1).fly_through()) {
      if (!gateway_.SetMissionCurrentGuarded(seq,*release.generation,authorized).accepted)
        return fail("native mission checkpoint advancement failed");
    }
    std::optional<double> heading;
    if (!point.fly_through()) {
      if (point.heading().mode()==v1::HEADING_MODE_ABSOLUTE) heading=point.heading().heading_deg();
      else if (point.heading().mode()==v1::HEADING_MODE_FACE_POINT)
        heading=HeadingTowardDeg(*resolved,point.heading().face_point());
      else if (point.heading().mode()==v1::HEADING_MODE_KEEP_CURRENT) {
        if (!initial->has_attitude()) return fail("initial heading unavailable");
        heading=std::fmod(initial->attitude().yaw_rad()*180/kPi+360.0,360.0);
      }
      // Preserve the segment heading while AUTO navigates, not only on arrival.
      if (heading && !CommandHeading(action_id,*heading)) return false;
    }
    bool observed_endpoint=false;
    if (!WaitUntil(action_id, route.context(), TimeoutOf(point.limits(),120000),
      [&,seq](const v1::DroneState& state) {
        const auto progress=gateway_.GetMissionProgress();
        if (progress.custom_mode!=3) {
          store_.Transition(action_id,v1::ACTION_STATE_PREEMPTED,v1::REASON_CODE_MANUAL_OVERRIDE,
                            "ArduPilot left AUTO during native route",0.0,&state);
          return false;
        }
        if (!measurement_valid(state,heading.has_value())) {
          fail("native route measurements stale, invalid or coordinate origin changed");
          return false;
        }
        observed_endpoint=observed_endpoint || Reached(*resolved,state,point.acceptance_radius_m());
        if (point.fly_through())
          return observed_endpoint && progress.revision>=generation && progress.reached_seq>=seq;
        return progress.current_seq==seq && Reached(*resolved,state,point.acceptance_radius_m()) &&
               state.ground_speed_mps()<=0.35 && std::abs(state.velocity_ned_mps().z())<=0.35;
      }, "native waypoint reached", false)) return false;
    if (!point.fly_through()) {
      store_.UpdateProgress(action_id,double(i)/route.points_size(),
                            "native waypoint "+std::to_string(i+1)+": measured hold");
      std::optional<std::chrono::steady_clock::time_point> stable_since;
      std::int64_t stable_sample=0, previous_sample=0;
      if (!WaitUntil(action_id,route.context(),point.hold_time_ms()+20000,
          [&](const v1::DroneState& state) {
            if (gateway_.GetMissionProgress().custom_mode!=3) {
              store_.Transition(action_id,v1::ACTION_STATE_PREEMPTED,v1::REASON_CODE_MANUAL_OVERRIDE,
                                "ArduPilot left AUTO during checkpoint",0.0,&state);
              return false;
            }
            if (!measurement_valid(state,heading.has_value())) {
              fail("native checkpoint measurements stale, invalid or coordinate origin changed");
              return false;
            }
            const auto sample=std::min(state.local_position_received_at_unix_ms(),
                std::min(state.relative_altitude_received_at_unix_ms(),
                         heading ? state.attitude_received_at_unix_ms() : state.local_position_received_at_unix_ms()));
            if (sample<previous_sample) { fail("native checkpoint measurement timestamps regressed"); return false; }
            if (sample==previous_sample) return false;
            previous_sample=sample;
            const bool facing=!heading || (state.has_attitude() &&
                std::abs(std::remainder(state.attitude().yaw_rad()*180/kPi-*heading,360.0))<=5);
            if (!Reached(*resolved,state,point.acceptance_radius_m()) || !facing ||
                state.ground_speed_mps()>0.35 || std::abs(state.velocity_ned_mps().z())>0.35) {
              stable_since.reset(); stable_sample=0; return false;
            }
            const auto now=std::chrono::steady_clock::now();
            if (!stable_since) { stable_since=now; stable_sample=sample; }
            const auto dwell=std::max<std::uint32_t>(point.hold_time_ms(),500);
            return sample-stable_sample>=dwell && now-*stable_since>=std::chrono::milliseconds(dwell);
          },"native waypoint held",false)) return false;
    }
  }
  const auto finish_mode=[&](std::uint32_t mode) {
    const auto result=gateway_.SetMissionCompletionMode(mode,*release.generation,authorized);
    return result.accepted || fail("native completion mode failed: "+result.message);
  };
  if (route.completion_behavior()==v1::ROUTE_COMPLETION_BEHAVIOR_RETURN_HOME) {
    if (!finish_mode(kModeRtl)) return false;
    return WaitForLanding(action_id,route.context(),120000,"native route completed; returned home and landed");
  }
  if (route.completion_behavior()==v1::ROUTE_COMPLETION_BEHAVIOR_LAND) {
    if (!finish_mode(kModeLand)) return false;
    return WaitForLanding(action_id,route.context(),90000,"native route completed; landed");
  }
  if (!finish_mode(kModeBrake)) return false;
  const auto state=state_engine_.Snapshot();
  store_.Transition(action_id,v1::ACTION_STATE_SUCCEEDED,v1::REASON_CODE_OK,
                    "ArduPilot AUTO route completed; measured "+std::to_string(route.points_size())+
                    " waypoint arrivals",1.0,state ? &*state : nullptr);
  return true;
}

bool ActionExecutor::ExecuteGoto(const std::string& action_id,
                                 const v1::CommandContext& context,
                                 const v1::Position& requested_destination,
                                 double acceptance_radius_m,
                                 std::uint32_t timeout_ms,
                                 bool complete_action,
                                 double requested_clearance_m,
                                 std::optional<double> heading_deg,
                                 double ground_speed_mps) {
  const auto initial = state_engine_.Snapshot();
  const auto resolved = initial
      ? guardrails::ResolveLocalAltitude(requested_destination, *initial) : std::nullopt;
  if (!resolved) {
    store_.Transition(action_id, v1::ACTION_STATE_ABORTED_BY_SAFETY,
                      v1::REASON_CODE_POSITION_UNAVAILABLE,
                      "navigation altitude transform is unavailable");
    gateway_.SetMode(kModeBrake);
    return false;
  }
  const auto destination = *resolved;
  if (!SetMode(action_id, kModeGuided, "GUIDED")) return false;
  // Enter the position submode at the measured current position FIRST.
  // ArduCopter's first position target after takeoff initializes its position
  // controller and resets speed/yaw. A preceding accepted speed command would
  // therefore be lost. This zero-displacement target initializes safely; the
  // following acknowledged speed command is ordered before any destination.
  if (!navigation_prepared_) {
    const auto current = state_engine_.Snapshot();
    std::optional<double> current_heading;
    if (current && current->has_attitude() &&
        std::isfinite(current->attitude().yaw_rad())) {
      current_heading = std::fmod(current->attitude().yaw_rad() * 180.0 / kPi +
                                    360.0, 360.0);
    }
    const bool initialized = current && (
        current->has_local_position_ned()
            ? gateway_.GotoLocal(current->local_position_ned(), current_heading)
            : current->has_position() && current->position().has_geo() &&
                gateway_.GotoGlobal(current->position().geo(), current_heading));
    if (!initialized) {
      store_.Transition(action_id, v1::ACTION_STATE_FAILED,
                        v1::REASON_CODE_STATE_STALE,
                        "could not initialize position control at measured current pose");
      gateway_.SetMode(kModeBrake);
      return false;
    }
    navigation_prepared_ = true;
  }
  // Apply on every new navigation leg, including the default, so a previous
  // slow request cannot leak into a later action. Mode changes may reset the
  // controller, therefore speed is applied after GUIDED confirmation.
  const auto speed = gateway_.SetGroundSpeed(std::min(
      ground_speed_mps, guardrails_.policy().maximum_horizontal_speed_mps));
  if (!speed.accepted) {
    store_.Transition(action_id, v1::ACTION_STATE_FAILED,
                      speed.timed_out ? v1::REASON_CODE_AUTOPILOT_TIMEOUT
                                      : v1::REASON_CODE_AUTOPILOT_REJECTED,
                      "ground speed command failed: " + speed.message);
    gateway_.SetMode(kModeBrake);
    return false;
  }
  if (destination.has_local_ned() && perception_ != nullptr && planner_ != nullptr) {
    const auto state = state_engine_.Snapshot();
    if (!state || !state->has_local_position_ned() || !perception_->Healthy()) {
      store_.Transition(action_id, v1::ACTION_STATE_ABORTED_BY_SAFETY,
                        v1::REASON_CODE_SAFETY_INTERVENTION,
                        "navigation blocked: perception unavailable");
      gateway_.SetMode(kModeBrake);
      return false;
    }
    // Plan a requested clearance with the same margin the default planner
    // envelope keeps over the live floor, so a path planned at exactly the
    // requested clearance is not aborted by ordinary tracking error.
    const auto& policy = guardrails_.policy();
    const double planning_margin_m = std::max(
        0.0, policy.minimum_obstacle_clearance_m -
                 policy.hard_minimum_clearance_m);
    const auto plan = planner_->Plan(
        state->local_position_ned(), destination.local_ned(),
        perception_->Obstacles(),
        requested_clearance_m > 0.0 ? requested_clearance_m + planning_margin_m
                                    : 0.0,
        std::max(policy.hard_minimum_clearance_m, requested_clearance_m));
    if (!plan.success) {
      store_.Transition(action_id, v1::ACTION_STATE_ABORTED_BY_SAFETY,
                        v1::REASON_CODE_SAFETY_INTERVENTION,
                        "navigation blocked: " + plan.reason);
      gateway_.SetMode(kModeBrake);
      return false;
    }
    for (std::size_t index = 0; index < plan.waypoints.size(); ++index) {
      const auto waypoint = plan.waypoints[index];
      if (!gateway_.GotoLocal(waypoint, heading_deg)) {
        store_.Transition(action_id, v1::ACTION_STATE_FAILED,
                          v1::REASON_CODE_AUTOPILOT_REJECTED,
                          "failed to send planned position target");
        return false;
      }
      // Every new position target resets ArduPilot to its default yaw
      // behaviour, so a requested heading is re-sent after each one.
      if (heading_deg && !CommandHeading(action_id, *heading_deg)) {
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
                        ? gateway_.GotoGlobal(destination.geo(), heading_deg)
                        : gateway_.GotoLocal(destination.local_ned(), heading_deg);
  if (!sent) {
    store_.Transition(action_id, v1::ACTION_STATE_FAILED,
                      v1::REASON_CODE_AUTOPILOT_REJECTED,
                      "failed to send position target");
    return false;
  }
  if (heading_deg && !CommandHeading(action_id, *heading_deg)) return false;
  return WaitUntil(
      action_id, context, timeout_ms,
      [destination, acceptance_radius_m](const v1::DroneState& state) {
        return Reached(destination, state, acceptance_radius_m);
      },
      "destination reached", complete_action);
}

bool ActionExecutor::ExecuteContinuousOrbit(
    const std::string& action_id, const v1::OrbitRequest& orbit,
    double start_angle, const v1::Position& altitude_position,
    std::optional<double> heading_deg) {
  const auto& context = orbit.context();
  const bool face = orbit.heading().mode() == v1::HEADING_MODE_FACE_POINT;
  const auto& focus = face && orbit.heading().has_face_point()
                          ? orbit.heading().face_point() : orbit.center();
  auto entry = OrbitWaypoint(orbit, start_angle, 0);
  entry.mutable_local_ned()->set_down_m(altitude_position.local_ned().down_m());
  entry.mutable_local_ned()->clear_altitude_above_home_m();
  if (face) heading_deg = HeadingTowardDeg(entry, focus);
  const double speed = std::min({orbit.ground_speed_mps(),
      GroundSpeedOf(orbit.limits(), orbit.ground_speed_mps()),
      guardrails_.policy().maximum_horizontal_speed_mps});
  // Only the radial entry uses the local planner. The following circle is a
  // feedback-controlled flight primitive, not a chain of settled waypoints.
  if (!ExecuteGoto(action_id, context, entry, 0.35, 60'000, false,
                   ClearanceOf(orbit.limits()), heading_deg, speed)) return false;
  if (!HoldViewpoint(action_id, context, entry, heading_deg, 0, 0, 0, 0.0)) {
    return false;
  }
  const auto& center = orbit.center().local_ned();
  const double direction = orbit.clockwise() ? 1.0 : -1.0;
  const double sweep = orbit.revolutions() * 2.0 * kPi;
  double commanded_sweep = 0.0;
  double measured_sweep = 0.0;
  const auto entry_state = state_engine_.Snapshot();
  if (!entry_state || !entry_state->has_local_position_ned()) return false;
  double previous_angle = std::atan2(
      entry_state->local_position_ned().east_m() - center.east_m(),
      entry_state->local_position_ned().north_m() - center.north_m());
  double commanded_speed = 0.0;
  double max_radial_error = 0.0;
  double max_heading_error = 0.0;
  double min_clearance = std::numeric_limits<double>::infinity();
  std::optional<std::chrono::steady_clock::time_point> tracking_lost_since;
  auto previous_time = std::chrono::steady_clock::now();
  std::optional<std::chrono::steady_clock::time_point> settled_since;
  bool stream_failed = false;
  const double expected_seconds = sweep * orbit.radius_m() / speed;
  const auto timeout_ms = TimeoutOf(orbit.limits(), static_cast<std::uint32_t>(
      std::min(3'600'000.0, (expected_seconds * 3.0 + 30.0) * 1000.0)));
  const bool finished = WaitUntil(action_id, context, timeout_ms,
      [&](const v1::DroneState& state) {
        if (!state.has_local_position_ned()) return false;
        const auto now = std::chrono::steady_clock::now();
        const double dt = std::clamp(std::chrono::duration<double>(
            now - previous_time).count(), 0.0, 0.1);
        previous_time = now;
        const auto& actual = state.local_position_ned();
        const double angle = std::atan2(actual.east_m() - center.east_m(),
                                        actual.north_m() - center.north_m());
        measured_sweep += direction * std::remainder(angle - previous_angle, 2.0 * kPi);
        previous_angle = angle;
        const double radius = std::hypot(actual.north_m() - center.north_m(),
                                          actual.east_m() - center.east_m());
        max_radial_error = std::max(max_radial_error, std::abs(radius - orbit.radius_m()));
        // Bounded acceleration, deceleration and reference lead. If tracking
        // falls behind, the trajectory slows instead of cutting across the circle.
        const double remaining_m = std::max(0.0, sweep - commanded_sweep) * orbit.radius_m();
        const double lead_m = (commanded_sweep - measured_sweep) * orbit.radius_m();
        const double desired_speed = std::min(speed, std::sqrt(2.0 * 0.5 * remaining_m)) *
            std::clamp(1.0 - std::max(0.0, lead_m - 0.3) / 0.7, 0.0, 1.0);
        commanded_speed += std::clamp(desired_speed - commanded_speed, -dt, 0.5 * dt);
        commanded_sweep = std::min(sweep, commanded_sweep + commanded_speed * dt / orbit.radius_m());
        if (sweep - commanded_sweep < 0.001) {
          commanded_sweep = sweep;
          commanded_speed = 0.0;
        }
        const double target_angle = start_angle + direction * commanded_sweep;
        auto target = entry.local_ned();
        target.set_north_m(center.north_m() + orbit.radius_m() * std::cos(target_angle));
        target.set_east_m(center.east_m() + orbit.radius_m() * std::sin(target_angle));
        if (face) {
          v1::Position current_position;
          *current_position.mutable_local_ned() = actual;
          heading_deg = HeadingTowardDeg(current_position, focus);
        }
        double heading_error = 0.0;
        if (heading_deg) {
          heading_error = state.has_attitude()
              ? std::abs(std::remainder(state.attitude().yaw_rad() * 180.0 / kPi -
                                       *heading_deg, 360.0))
              : 180.0;
          max_heading_error = std::max(max_heading_error, heading_error);
        }
        if (perception_ != nullptr) {
          const double clearance = perception_->Summary().nearest_obstacle_distance_m();
          if (clearance >= 0.0) min_clearance = std::min(min_clearance, clearance);
        }
        const bool tracking_lost = std::abs(radius - orbit.radius_m()) > 1.0 ||
            std::abs(actual.down_m() - target.down_m()) > 0.75 ||
            heading_error > 20.0;
        if (!tracking_lost) tracking_lost_since.reset();
        else if (!tracking_lost_since) tracking_lost_since = now;
        if (tracking_lost_since && now - *tracking_lost_since > std::chrono::seconds(2)) {
          stream_failed = true;
          gateway_.SetMode(kModeBrake);
          store_.Transition(action_id, v1::ACTION_STATE_FAILED,
                            v1::REASON_CODE_COMPLETION_TIMEOUT,
                            "orbit tracking lost: measured radius, altitude or target-facing heading left tolerance");
          return true;
        }
        if (!gateway_.StreamLocal(target,
                -direction * commanded_speed * std::sin(target_angle),
                direction * commanded_speed * std::cos(target_angle), 0.0, heading_deg)) {
          stream_failed = true;
          gateway_.SetMode(kModeBrake);
          store_.Transition(action_id, v1::ACTION_STATE_FAILED,
                            v1::REASON_CODE_AUTOPILOT_REJECTED,
                            "continuous orbit setpoint could not be sent");
          return true;
        }
        store_.UpdateProgress(action_id, std::clamp(measured_sweep / sweep, 0.0, 1.0),
                              "orbiting continuously; measured sweep " +
                              std::to_string(measured_sweep * 180.0 / kPi) + " deg");
        v1::Position endpoint;
        *endpoint.mutable_local_ned() = target;
        const bool facing = !heading_deg || (state.has_attitude() &&
            std::abs(std::remainder(state.attitude().yaw_rad() * 180.0 / kPi -
                                   *heading_deg, 360.0)) <= 5.0);
        const bool settled = commanded_sweep >= sweep &&
            measured_sweep >= sweep - std::min(0.02, 0.15 / orbit.radius_m()) &&
            Reached(endpoint, state, 0.35) && state.has_velocity_ned_mps() &&
            state.ground_speed_mps() <= 0.35 &&
            std::abs(state.velocity_ned_mps().z()) <= 0.35 && facing;
        if (!settled) settled_since.reset();
        else if (!settled_since) settled_since = now;
        return settled_since && now - *settled_since >= std::chrono::milliseconds(500);
      }, "continuous orbit complete", false);
  if (!finished || stream_failed) return false;
  // Remove velocity feed-forward explicitly after the measured final settle.
  const auto final_state = state_engine_.Snapshot();
  if (!final_state || !gateway_.GotoLocal(final_state->local_position_ned(), heading_deg)) {
    gateway_.SetMode(kModeBrake);
    store_.Transition(action_id, v1::ACTION_STATE_FAILED,
                      v1::REASON_CODE_AUTOPILOT_REJECTED, "orbit final hold failed");
    return false;
  }
  char message[320];
  std::snprintf(message, sizeof(message),
      "orbit completed; measured sweep %.1f deg; max radial error %.2f m; max heading error %.1f deg; minimum observed clearance %.2f m (-1=unavailable)",
      measured_sweep * 180.0 / kPi, max_radial_error, max_heading_error,
      std::isfinite(min_clearance) ? min_clearance : -1.0);
  store_.Transition(action_id, v1::ACTION_STATE_SUCCEEDED, v1::REASON_CODE_OK,
                    message, 1.0, &*final_state);
  return true;
}

bool ActionExecutor::CommandHeading(const std::string& action_id,
                                    double heading_deg) {
  const auto result = gateway_.SetYaw(heading_deg);
  if (result.accepted) return true;
  store_.Transition(action_id, v1::ACTION_STATE_FAILED,
                    result.timed_out ? v1::REASON_CODE_AUTOPILOT_TIMEOUT
                                     : v1::REASON_CODE_AUTOPILOT_REJECTED,
                    "heading command failed: " + result.message);
  return false;
}

bool ActionExecutor::HoldViewpoint(const std::string& action_id,
                                   const v1::CommandContext& context,
                                   const v1::Position& point,
                                   std::optional<double> heading_deg,
                                   std::uint32_t dwell_ms, int number,
                                   int total, double progress) {
  const std::string label = total > 0
      ? "orbit viewpoint " + std::to_string(number) + "/" + std::to_string(total)
      : "orbit entry";
  store_.UpdateProgress(action_id, progress, label + ": settling");
  const double heading_rad = heading_deg ? *heading_deg * kPi / 180.0 : 0.0;
  std::optional<std::chrono::steady_clock::time_point> stable_since;
  if (!WaitUntil(
          action_id, context, kViewpointSettleTimeoutMs,
          [point, heading_deg, heading_rad,
           stable_since](const v1::DroneState& state) mutable {
            const bool settled =
                Reached(point, state, kViewpointAcceptanceM) &&
                state.has_velocity_ned_mps() &&
                state.ground_speed_mps() <= 0.35 &&
                std::abs(state.velocity_ned_mps().z()) <= 0.35;
            const bool facing =
                !heading_deg ||
                (state.has_attitude() &&
                 std::abs(std::remainder(state.attitude().yaw_rad() -
                                             heading_rad,
                                         2.0 * kPi)) <= 5.0 * kPi / 180.0);
            if (!settled || !facing) {
              stable_since.reset();
              return false;
            }
            const auto now = std::chrono::steady_clock::now();
            if (!stable_since) stable_since = now;
            return now - *stable_since >= std::chrono::milliseconds(500);
          },
          label + " settled", false)) {
    return false;
  }
  store_.UpdateProgress(action_id, progress,
                        label + ": holding " + std::to_string(dwell_ms) +
                            " ms",
                        static_cast<std::uint32_t>(number));
  const auto started = std::chrono::steady_clock::now();
  if (!WaitUntil(
          action_id, context, dwell_ms + 5'000,
          [started, dwell_ms](const v1::DroneState&) {
            return std::chrono::steady_clock::now() - started >=
                   std::chrono::milliseconds(dwell_ms);
          },
          label + " held", false)) {
    return false;
  }
  store_.UpdateProgress(action_id, progress, label + ": done");
  return true;
}

void ActionExecutor::BeginClearanceGuard(const v1::ActionCommand& command) {
  clearance_ = ClearanceGuard{};
  double requested = 0.0;
  if (command.has_goto_()) {
    requested = ClearanceOf(command.goto_().limits());
  } else if (command.has_orbit()) {
    requested = ClearanceOf(command.orbit().limits());
  } else if (command.has_execute_route()) {
    for (const auto& point : command.execute_route().points()) {
      requested = std::max(requested, ClearanceOf(point.limits()));
    }
  } else if (!command.has_yaw()) {
    // Takeoff, hold, land and the return-home recovery flights are exempt:
    // stopping them near an obstacle is not safer than completing them.
    return;
  }
  clearance_.active = true;
  clearance_.required_m =
      std::max(guardrails_.policy().hard_minimum_clearance_m, requested);
}

std::optional<std::pair<v1::ReasonCode, std::string>>
ActionExecutor::ClearanceViolation() {
  if (!clearance_.active || perception_ == nullptr) return std::nullopt;
  if (!perception_->Healthy()) {
    return std::pair{v1::REASON_CODE_SAFETY_INTERVENTION,
                     std::string("clearance unverifiable: perception stale; "
                                 "BRAKE commanded")};
  }
  const auto summary = perception_->Summary();
  // A negative distance means the fresh map holds no obstacle at all.
  const double nearest = summary.nearest_obstacle_distance_m() < 0.0
                             ? std::numeric_limits<double>::infinity()
                             : summary.nearest_obstacle_distance_m();
  clearance_.best_observed_m = std::max(clearance_.best_observed_m, nearest);
  const double floor =
      EnforcedClearanceM(clearance_.required_m, clearance_.best_observed_m);
  if (nearest >= floor) return std::nullopt;
  char message[192];
  std::snprintf(message, sizeof(message),
                "minimum clearance violated: nearest obstacle %.2f m at "
                "bearing %.0f deg is inside the %.2f m floor; BRAKE commanded",
                nearest, summary.nearest_obstacle_bearing_deg(), floor);
  return std::pair{v1::REASON_CODE_CLEARANCE_VIOLATION, std::string(message)};
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
    case v1::ActionCommand::kYaw:
      return command.yaw().context();
    default:
      throw std::invalid_argument("action command is empty");
  }
}

bool ActionExecutor::Reached(const v1::Position& target,
                             const v1::DroneState& state,
                             double acceptance_radius_m) {
  if (target.has_local_ned() && state.has_local_position_ned()) {
    const auto& goal = target.local_ned();
    const auto& actual = state.local_position_ned();
    return std::sqrt(std::pow(goal.north_m() - actual.north_m(), 2) +
                     std::pow(goal.east_m() - actual.east_m(), 2) +
                     std::pow(goal.down_m() - actual.down_m(), 2)) <=
           acceptance_radius_m;
  }
  if (target.has_geo() && state.position().has_geo()) {
    double actual_altitude = state.position().geo().altitude_m();
    if (target.geo().frame() == v1::COORDINATE_FRAME_WGS84_ABOVE_HOME) {
      actual_altitude = state.relative_altitude_m();
    }
    return GeoDistanceM(target.geo(), state.position().geo()) <=
               acceptance_radius_m &&
           std::abs(target.geo().altitude_m() - actual_altitude) <=
               acceptance_radius_m;
  }
  return false;
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

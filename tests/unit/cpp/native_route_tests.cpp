#include "mission_executor/action_executor.hpp"
#include <atomic>
#include <cmath>
#include <filesystem>
#include <iostream>
#include <limits>
#include <set>
#include <stdexcept>
#include <string>
#include <thread>

namespace {
using namespace icarus;
void Require(bool condition, const char *reason) {
  if (!condition)
    throw std::runtime_error(reason);
}
template <class F> void Rejects(F call, const char *reason) {
  bool rejected = false;
  try {
    call();
  } catch (const std::invalid_argument &) {
    rejected = true;
  }
  Require(rejected, reason);
}
class FakeClock final : public icarus::core::Clock {
public:
  [[nodiscard]] std::int64_t NowUnixMs() const override { return now_ms; }
  std::atomic<std::int64_t> now_ms{1'000'000};
};

class FakeGateway : public icarus::mavlink_gateway::MavlinkGateway {
public:
  bool Connect(const std::string &, std::uint16_t) override { return true; }
  void Disconnect() override {}
  [[nodiscard]] bool IsConnected() const override { return true; }
  [[nodiscard]] bool WaitForReady(std::uint32_t) override { return true; }
  void SetStateCallback(StateCallback) override {}
  void SetEventCallback(EventCallback) override {}
  [[nodiscard]] icarus::mavlink_gateway::CommandResult Arm(bool) override {
    ++arm_commands;
    return {true, false, 0, "accepted"};
  }
  [[nodiscard]] icarus::mavlink_gateway::CommandResult
  Takeoff(double) override {
    ++takeoff_commands;
    return {true, false, 0, "accepted"};
  }
  [[nodiscard]] icarus::mavlink_gateway::CommandResult
  SetMode(std::uint32_t mode) override {
    ++mode_commands;
    last_mode = mode;
    return {true, false, 0, "accepted"};
  }
  [[nodiscard]] icarus::mavlink_gateway::CommandResult
  SetYaw(double heading) override {
    last_heading = heading;
    return {true, false, 0, "accepted"};
  }
  [[nodiscard]] icarus::mavlink_gateway::CommandResult
  RotateYaw(double angle, double rate) override {
    last_relative_yaw = angle;
    last_yaw_rate = rate;
    return {true, false, 0, "accepted"};
  }
  [[nodiscard]] icarus::mavlink_gateway::CommandResult
  SetGroundSpeed(double speed) override {
    last_ground_speed = speed;
    ++speed_commands;
    return {accept_speed, false, 0,
            accept_speed ? "accepted" : "speed rejected"};
  }
  bool GotoGlobal(const icarus::v1::GeoPosition &,
                  std::optional<double> = std::nullopt) override {
    return true;
  }
  bool GotoLocal(const icarus::v1::LocalPositionNed &,
                 std::optional<double> = std::nullopt) override {
    return true;
  }
  std::atomic<int> arm_commands{0};
  std::atomic<int> takeoff_commands{0};
  std::atomic<int> mode_commands{0};
  std::atomic<std::uint32_t> last_mode{0};
  std::atomic<double> last_heading{-1.0};
  std::atomic<double> last_relative_yaw{0.0};
  std::atomic<double> last_yaw_rate{0.0};
  std::atomic<double> last_ground_speed{0.0};
  std::atomic<int> speed_commands{0};
  bool accept_speed = true;
};

icarus::v1::DroneState HealthyLandedState(std::int64_t now) {
  icarus::v1::DroneState state;
  state.set_vehicle_id("icarus-01");
  state.set_observed_at_unix_ms(now);
  state.set_landed(true);
  state.set_armed(false);
  state.set_flight_phase(icarus::v1::FLIGHT_PHASE_DISARMED);
  state.mutable_battery()->set_level(icarus::v1::HEALTH_LEVEL_HEALTHY);
  state.mutable_battery()->set_remaining_percent(90.0);
  state.mutable_battery()->set_voltage_v(24.0);
  state.mutable_battery()->set_current_a(0.0);
  state.mutable_battery()->set_received_at_unix_ms(now);
  state.mutable_estimator()->set_level(icarus::v1::HEALTH_LEVEL_HEALTHY);
  state.mutable_estimator()->set_horizontal_position_valid(true);
  state.mutable_estimator()->set_vertical_position_valid(true);
  state.mutable_estimator()->set_attitude_valid(true);
  state.mutable_mavlink()->set_level(icarus::v1::HEALTH_LEVEL_HEALTHY);
  state.mutable_mavlink()->set_connected(true);
  auto *home = state.mutable_home()->mutable_geo();
  home->set_latitude_deg(28.0);
  home->set_longitude_deg(77.0);
  home->set_altitude_m(0.0);
  home->set_frame(icarus::v1::COORDINATE_FRAME_WGS84_ABOVE_HOME);
  *state.mutable_position() = state.home();
  state.mutable_local_position_ned()->set_origin_id("ekf-origin");
  state.mutable_velocity_ned_mps()->set_z(0.0);
  state.set_local_position_received_at_unix_ms(now);
  state.set_relative_altitude_received_at_unix_ms(now);
  state.set_attitude_received_at_unix_ms(now);
  state.set_local_position_time_boot_ms(1000);
  state.set_relative_altitude_time_boot_ms(1000);
  return state;
}

icarus::v1::CommandContext Context(const FakeClock &clock,
                                   const std::string &lease_id) {
  icarus::v1::CommandContext context;
  context.set_request_id("request-1");
  context.set_idempotency_key("idempotency-1");
  context.set_vehicle_id("icarus-01");
  context.set_client_id("session-1");
  context.set_control_lease_id(lease_id);
  context.set_session_id("session-1");
  context.set_issued_at_unix_ms(clock.NowUnixMs());
  context.set_expires_at_unix_ms(clock.NowUnixMs() + 1'000);
  context.set_minimum_state_sequence(1);
  return context;
}

icarus::v1::Position Local(double north, double east, double down = -5.0) {
  icarus::v1::Position position;
  auto *local = position.mutable_local_ned();
  local->set_origin_id("ekf-origin");
  local->set_north_m(north);
  local->set_east_m(east);
  local->set_down_m(down);
  return position;
}

// Hovering, settled, facing north at the given local position.
icarus::v1::DroneState AirborneState(const FakeClock &clock, double north,
                                     double east, double down = -5.0) {
  auto state = HealthyLandedState(clock.NowUnixMs());
  state.set_armed(true);
  state.set_landed(false);
  state.set_flight_phase(icarus::v1::FLIGHT_PHASE_AIRBORNE);
  state.mutable_velocity_ned_mps()->set_x(0.0);
  state.mutable_attitude()->set_yaw_rad(0.0);
  *state.mutable_local_position_ned() = Local(north, east, down).local_ned();
  state.set_relative_altitude_m(-down);
  auto *geo = state.mutable_position()->mutable_geo();
  geo->set_latitude_deg(28.0 +
                        north / 6371000.0 * 180.0 / 3.14159265358979323846);
  geo->set_longitude_deg(
      77.0 + east /
                 (6371000.0 * std::cos(28.0 * 3.14159265358979323846 / 180.0)) *
                 180.0 / 3.14159265358979323846);
  return state;
}

std::string AcquireLease(icarus::drone_api::AuthorityManager &authority) {
  icarus::v1::AcquireControlRequest request;
  request.set_session_id("session-1");
  request.set_client_id("session-1");
  request.set_vehicle_id("icarus-01");
  request.set_requested_role(icarus::v1::CONTROL_ROLE_DCM_AUTONOMY);
  request.set_requested_lease_ms(30'000);
  const auto lease = authority.Acquire(request);
  Require(lease.granted(), "test lease should be granted");
  return lease.authority().lease_id();
}

bool Terminal(icarus::v1::ActionState state) {
  return state != icarus::v1::ACTION_STATE_ACCEPTED &&
         state != icarus::v1::ACTION_STATE_QUEUED &&
         state != icarus::v1::ACTION_STATE_VALIDATING &&
         state != icarus::v1::ACTION_STATE_EXECUTING;
}

icarus::v1::ActionStatus
WaitForTerminal(const icarus::mission_executor::ActionStore &actions,
                const std::string &action_id, int timeout_ms,
                std::set<std::uint32_t> *viewpoints = nullptr) {
  for (int waited = 0; waited < timeout_ms; waited += 2) {
    const auto status = actions.Find(action_id);
    if (status && viewpoints != nullptr && status->active_viewpoint() > 0) {
      viewpoints->insert(status->active_viewpoint());
    }
    if (status && Terminal(status->state()))
      return *status;
    std::this_thread::sleep_for(std::chrono::milliseconds(2));
  }
  throw std::runtime_error("action did not reach a terminal state");
}

v1::DroneState State() {
  v1::DroneState s;
  auto *geo = s.mutable_position()->mutable_geo();
  geo->set_latitude_deg(35.0);
  geo->set_longitude_deg(70.0);
  geo->set_altitude_m(504.0);
  geo->set_frame(v1::COORDINATE_FRAME_WGS84_AMSL);
  auto *local = s.mutable_local_position_ned();
  local->set_origin_id("origin-a");
  local->set_north_m(200);
  local->set_east_m(-300);
  local->set_down_m(-12);
  s.set_relative_altitude_m(4);
  s.set_relative_altitude_received_at_unix_ms(1000);
  s.set_local_position_received_at_unix_ms(1000);
  s.set_observed_at_unix_ms(1000);
  auto *home = s.mutable_home()->mutable_geo();
  home->set_latitude_deg(34);
  home->set_longitude_deg(69);
  home->set_altitude_m(500);
  home->set_frame(v1::COORDINATE_FRAME_WGS84_AMSL);
  return s;
}
v1::ExecuteRouteRequest LocalRoute() {
  v1::ExecuteRouteRequest route;
  auto *p = route.add_points();
  auto *local = p->mutable_position()->mutable_local_ned();
  local->set_origin_id("origin-a");
  local->set_north_m(201);
  local->set_east_m(-299);
  local->set_altitude_above_home_m(7);
  return route;
}
class NativeFake final : public FakeGateway {
public:
  bool allow_upload = true;
  bool leave_auto = false;
  bool report_reached = false;
  std::function<void()> speed_hook;
  mavlink_gateway::CommandResult SetGroundSpeed(double) override {
    if (speed_hook)
      speed_hook();
    return {true, false, 0, "speed set"};
  }
  std::atomic<int> starts{0};
  std::atomic<int> current{1};
  mavlink_gateway::CommandResult SetMissionCurrent(std::uint16_t seq) override {
    current = seq;
    return {true, false, 0, "advanced"};
  }
  mavlink_gateway::CommandResult
  UploadMission(const std::vector<mavlink_gateway::MissionItem> &,
                std::uint32_t) override {
    return {allow_upload, false, 0,
            allow_upload ? "stored" : "storage failure"};
  }
  mavlink_gateway::CommandResult StartMission(std::uint16_t) override {
    ++starts;
    last_mode = leave_auto ? 5 : 3;
    return {true, false, 0, "started"};
  }
  mavlink_gateway::MissionProgress GetMissionProgress() const override {
    mavlink_gateway::MissionProgress p;
    p.current_seq = current;
    p.reached_seq = report_reached ? current.load() : -1;
    p.revision = 1;
    p.custom_mode = last_mode;
    return p;
  }
};
class StatePublisher {
public:
  StatePublisher(FakeClock &clock, state_engine::StateEngine &states,
                 v1::DroneState state, bool refresh = true)
      : worker_([&, state, refresh]() mutable {
          while (running_) {
            clock.now_ms += 20;
            const auto now = clock.NowUnixMs();
            state.set_observed_at_unix_ms(now);
            if (refresh) {
              state.set_local_position_received_at_unix_ms(now);
              state.set_relative_altitude_received_at_unix_ms(now);
              state.set_attitude_received_at_unix_ms(now);
            }
            state.mutable_battery()->set_received_at_unix_ms(now);
            states.Publish(state);
            std::this_thread::sleep_for(std::chrono::milliseconds(20));
          }
        }) {}
  ~StatePublisher() {
    running_ = false;
    worker_.join();
  }

private:
  std::atomic<bool> running_{true};
  std::thread worker_;
};

void StoppedHeadingContract() {
  for (int scenario = 0; scenario < 3; ++scenario) {
    const bool keep = scenario != 1;
    FakeClock clock;
    auto policy = guardrails::SafetyPolicy::Load(
        std::filesystem::path(ICARUS_SOURCE_DIR) / "config/safety/v1.yaml");
    guardrails::Guardrails guards(policy);
    state_engine::StateEngine states(clock);
    auto state = AirborneState(clock, 0, 0);
    state.mutable_attitude()->set_yaw_rad(-3.14159265358979323846 / 2);
    states.Publish(state);
    drone_api::AuthorityManager authority(clock);
    auto lease = AcquireLease(authority);
    mission_executor::ActionStore store(clock);
    NativeFake gateway;
    mission_executor::ActionExecutor executor(clock, store, states, authority,
                                              guards, gateway);
    executor.Start();
    v1::ActionCommand command;
    auto *route = command.mutable_execute_route();
    *route->mutable_context() = Context(clock, lease);
    route->set_route_id("heading-test");
    route->set_delegate_to_autopilot(true);
    auto *point = route->add_points();
    *point->mutable_position() = Local(0, 0);
    point->set_acceptance_radius_m(.25);
    point->mutable_heading()->set_mode(keep ? v1::HEADING_MODE_KEEP_CURRENT
                                            : v1::HEADING_MODE_ABSOLUTE);
    if (!keep)
      point->mutable_heading()->set_heading_deg(270);
    auto receipt = executor.Submit(command);
    Require(receipt.disposition() == v1::ACTION_STATE_ACCEPTED,
            "heading route admitted");
    StatePublisher publish(clock, states, state, scenario != 2);
    auto result = WaitForTerminal(store, receipt.action_id(), 2500);
    executor.Stop();
    if (scenario == 2) {
      Require(result.state() != v1::ACTION_STATE_SUCCEEDED,
              "frozen position/attitude samples cannot earn dwell while "
              "aggregate stays fresh");
      continue;
    }
    Require(result.state() == v1::ACTION_STATE_SUCCEEDED,
            "stopped native heading completes measured dwell");
    Require(std::abs(gateway.last_heading - 270) < .001,
            "negative measured heading normalized and retained");
  }
}

void NativeProgressCannotReplaceMeasuredArrival() {
  for (const bool fly_through : {false, true}) {
    FakeClock clock;
    auto policy = guardrails::SafetyPolicy::Load(
        std::filesystem::path(ICARUS_SOURCE_DIR) / "config/safety/v1.yaml");
    guardrails::Guardrails guards(policy);
    state_engine::StateEngine states(clock);
    const auto state = AirborneState(clock, 0, 0);
    states.Publish(state);
    drone_api::AuthorityManager authority(clock);
    const auto lease = AcquireLease(authority);
    mission_executor::ActionStore store(clock);
    NativeFake gateway;
    gateway.report_reached = true;
    mission_executor::ActionExecutor executor(clock, store, states, authority,
                                              guards, gateway);
    executor.Start();
    v1::ActionCommand command;
    auto *route = command.mutable_execute_route();
    *route->mutable_context() = Context(clock, lease);
    route->set_route_id("false-native-arrival");
    route->set_delegate_to_autopilot(true);
    auto *point = route->add_points();
    *point->mutable_position() = Local(1, 0);
    point->set_acceptance_radius_m(.25);
    point->set_fly_through(fly_through);
    point->mutable_limits()->set_execution_timeout_ms(800);
    const auto receipt = executor.Submit(command);
    Require(receipt.disposition() == v1::ACTION_STATE_ACCEPTED,
            "false-progress route admitted");
    // Fresh measurements remain stationary one metre from the destination.
    // The autopilot reports both current/reached sequence for that endpoint.
    StatePublisher publish(clock, states, state);
    const auto result = WaitForTerminal(store, receipt.action_id(), 3000);
    executor.Stop();
    Require(gateway.current == 2 && gateway.starts == 1,
            "autopilot actually claimed advancement to the requested point");
    Require(result.state() != v1::ACTION_STATE_SUCCEEDED &&
                result.message().find("timed out") != std::string::npos,
            "native progress without measured endpoint arrival must time out");
  }
}

void ExecutorFailureContracts() {
  for (int scenario = 0; scenario < 7; ++scenario) {
    FakeClock clock;
    const auto policy = guardrails::SafetyPolicy::Load(
        std::filesystem::path(ICARUS_SOURCE_DIR) / "config/safety/v1.yaml");
    guardrails::Guardrails guards(policy);
    state_engine::StateEngine states(clock);
    auto state = AirborneState(clock, 0, 0);
    states.Publish(state);
    drone_api::AuthorityManager authority(clock);
    auto lease = AcquireLease(authority);
    mission_executor::ActionStore store(clock);
    NativeFake gateway;
    gateway.allow_upload = scenario != 0 && scenario != 3;
    if (scenario == 3)
      gateway.last_mode = 3;
    gateway.leave_auto = scenario == 2;
    mission_executor::ActionExecutor executor(clock, store, states, authority,
                                              guards, gateway);
    executor.Start();
    v1::ActionCommand command;
    auto *route = command.mutable_execute_route();
    *route->mutable_context() = Context(clock, lease);
    route->set_route_id("native-test");
    route->set_delegate_to_autopilot(true);
    auto *point = route->add_points();
    *point->mutable_position() =
        Local(0, 0); // Already at endpoint, but no mission progress.
    point->set_acceptance_radius_m(0.5);
    point->set_fly_through(scenario != 5 && scenario != 6);
    point->mutable_limits()->set_execution_timeout_ms(150);
    if (scenario == 4)
      gateway.speed_hook = [&] {
        v1::CancelActionRequest request;
        *request.mutable_context() = Context(clock, lease);
        request.set_action_id(*store.ActiveConflictingAction());
        const auto cancelled = executor.Cancel(request);
        Require(cancelled.disposition() == v1::ACTION_STATE_CANCELLED,
                "cancel during speed ACK accepted");
      };
    if (scenario == 5 || scenario == 6)
      gateway.speed_hook = [&] {
        auto changed = state;
        if (scenario == 5)
          changed.set_local_position_received_at_unix_ms(1);
        else
          changed.mutable_local_position_ned()->set_origin_id("changed-origin");
        states.Publish(changed);
      };
    auto receipt = executor.Submit(command);
    Require(receipt.disposition() == v1::ACTION_STATE_ACCEPTED,
            "native test route admitted");
    const auto result = WaitForTerminal(store, receipt.action_id(), 2500);
    executor.Stop();
    Require(result.state() != v1::ACTION_STATE_SUCCEEDED,
            "native failure must not claim completion");
    Require(gateway.last_mode == (scenario == 0   ? 0
                                  : scenario == 2 ? 5
                                  : scenario == 3 ? 3
                                                  : 17),
            "failure holds; external mode change retains operator control");
    if (scenario == 3)
      Require(gateway.starts == 0,
              "pre-existing AUTO remains externally owned");
    if (scenario == 4)
      Require(gateway.starts == 1 && gateway.current == 1,
              "cancel after speed cannot leave activation checkpoint");
    if (scenario == 5 || scenario == 6)
      Require(
          (result.message().find("measurements") != std::string::npos ||
           result.message().find("altitude transform") != std::string::npos),
          "invalid individual measurements cannot finish station");
    if (scenario == 0)
      Require(gateway.starts == 0,
              "upload failure cannot start native mission");
    if (scenario == 1)
      Require(result.message().find("timed out") != std::string::npos,
              "being near endpoint without mission progress times out");
    if (scenario == 2)
      Require(result.message().find("AUTO") != std::string::npos,
              "unexpected AUTO exit explicitly reported");
  }
}

void GeographicAnchorAndAltitude() {
  const auto state = State();
  auto route = LocalRoute();
  auto plan = mission_executor::BuildNativeRoute(route, state);
  const auto &p = plan.items.at(2);
  // Reconstruct displacement independently from lat/lon: one metre from the
  // measured aircraft pose, not 201 metres from home or the EKF origin.
  const double north =
      (p.x / 1e7 - 35.0) * 3.14159265358979323846 / 180 * 6378137.0;
  const double east = (p.y / 1e7 - 70.0) * 3.14159265358979323846 / 180 *
                      6378137.0 * std::cos(35 * 3.14159265358979323846 / 180);
  Require(std::abs(north - 1) < 0.025 && std::abs(east - 1) < 0.025,
          "mission anchored to measured aircraft geographic and local pose");
  Require(
      p.z == 7,
      "above-home altitude preserved despite displaced EKF vertical origin");
  auto *local =
      route.mutable_points(0)->mutable_position()->mutable_local_ned();
  local->clear_altitude_above_home_m();
  local->set_down_m(-15);
  plan = mission_executor::BuildNativeRoute(route, state);
  Require(plan.items[2].z == 7,
          "NED down altitude resolved using current home-relative offset");
  auto *geo = route.mutable_points(0)->mutable_position()->mutable_geo();
  geo->set_latitude_deg(35.1);
  geo->set_longitude_deg(70.1);
  geo->set_altitude_m(509);
  geo->set_frame(v1::COORDINATE_FRAME_WGS84_AMSL);
  plan = mission_executor::BuildNativeRoute(route, state);
  Require(plan.items[2].z == 9, "AMSL resolved using measured home altitude");
  geo->set_frame(v1::COORDINATE_FRAME_WGS84_ABOVE_HOME);
  geo->set_altitude_m(11);
  Require(mission_executor::BuildNativeRoute(route, state).items[2].z == 11,
          "geographic above-home remains above-home");
}
void WaypointSemantics() {
  auto route = LocalRoute();
  auto *p = route.add_points();
  *p = route.points(0);
  p->set_fly_through(true);
  const auto plan = mission_executor::BuildNativeRoute(route, State());
  Require(plan.items.size() == 5, "home, two navigation points, terminal hold");
  Require(plan.waypoint_sequences == std::vector<std::uint16_t>({2, 3}),
          "wire indices exclude home and terminal hold");
  Require(plan.items[0].command == 16 && plan.items[0].x == 350000000 &&
              plan.items[0].z == 4,
          "sequence zero is nonexecuted home placeholder");
  Require(plan.items[1].command == 17 && plan.items[1].x == plan.items[0].x &&
              plan.items[1].z == plan.items[0].z,
          "stationary activation checkpoint precedes actual route");
  Require(plan.items[2].command == 17,
          "stopped station loiters until explicitly released");
  Require(plan.items[3].command == 16, "flythrough station is native waypoint");
  Require(plan.items[4].command == 17 && plan.items[4].x == plan.items[3].x &&
              plan.items[4].y == plan.items[3].y &&
              plan.items[4].z == plan.items[3].z,
          "mission ends in hold at final destination");
  for (const auto &item : plan.items)
    Require(item.frame == 6,
            "mission uses relative altitude integer global frame");
}
void GeographicBoundaryCases() {
  auto state = State();
  auto route = LocalRoute();
  for (double invalid :
       {std::numeric_limits<double>::quiet_NaN(), 181.0, -181.0}) {
    state.mutable_position()->mutable_geo()->set_longitude_deg(invalid);
    Rejects([&] { (void)mission_executor::BuildNativeRoute(route, state); },
            "invalid longitude anchor rejected before integer conversion");
  }
  state = State();
  state.mutable_position()->mutable_geo()->set_longitude_deg(179.999999);
  const auto plan = mission_executor::BuildNativeRoute(route, state);
  Require(plan.items[2].y < 0 && plan.items[2].y >= -1800000000,
          "eastward local displacement wraps across dateline");
  state = State();
  state.mutable_position()->mutable_geo()->set_latitude_deg(100);
  Rejects([&] { (void)mission_executor::BuildNativeRoute(route, state); },
          "out of bounds latitude anchor rejected");
}

void InvalidMeasurements() {
  auto state = State();
  auto route = LocalRoute();
  route.mutable_points(0)
      ->mutable_position()
      ->mutable_local_ned()
      ->set_origin_id("other-origin");
  Rejects([&] { (void)mission_executor::BuildNativeRoute(route, state); },
          "origin mismatch rejected");
  route = LocalRoute();
  state.set_relative_altitude_received_at_unix_ms(0);
  Rejects([&] { (void)mission_executor::BuildNativeRoute(route, state); },
          "missing home-relative altitude rejected");
  state = State();
  state.clear_position();
  Rejects([&] { (void)mission_executor::BuildNativeRoute(route, state); },
          "missing geographic anchor rejected");
  state = State();
  state.mutable_position()->mutable_geo()->set_latitude_deg(90);
  Rejects([&] { (void)mission_executor::BuildNativeRoute(route, state); },
          "polar singularity rejected");
  state = State();
  route.mutable_points(0)->mutable_position()->mutable_local_ned()->set_north_m(
      std::numeric_limits<double>::quiet_NaN());
  Rejects([&] { (void)mission_executor::BuildNativeRoute(route, state); },
          "nonfinite coordinate rejected");
  route = LocalRoute();
  auto *amsl = route.mutable_points(0)->mutable_position()->mutable_geo();
  amsl->set_latitude_deg(35);
  amsl->set_longitude_deg(70);
  amsl->set_altitude_m(507);
  amsl->set_frame(v1::COORDINATE_FRAME_WGS84_AMSL);
  state.mutable_home()->mutable_geo()->set_frame(
      v1::COORDINATE_FRAME_WGS84_ABOVE_HOME);
  Rejects([&] { (void)mission_executor::BuildNativeRoute(route, state); },
          "AMSL requires AMSL-framed home datum");
  route.Clear();
  Rejects([&] { (void)mission_executor::BuildNativeRoute(route, state); },
          "empty route rejected");
}
} // namespace
int main() {
  try {
    ExecutorFailureContracts();
    StoppedHeadingContract();
    NativeProgressCannotReplaceMeasuredArrival();
    GeographicAnchorAndAltitude();
    WaypointSemantics();
    InvalidMeasurements();
    GeographicBoundaryCases();
    std::cout << "Native route geometry tests passed\n";
    return 0;
  } catch (const std::exception &error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
}

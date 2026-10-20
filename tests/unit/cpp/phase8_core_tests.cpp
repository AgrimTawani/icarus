#include <cmath>
#include <chrono>
#include <cstdint>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <limits>
#include <mutex>
#include <set>
#include <stdexcept>
#include <string>
#include <thread>
#include <vector>
#include <yaml-cpp/yaml.h>

#include "autonomy_core/clock.hpp"
#include "drone_api/authority_manager.hpp"
#include "drone_api/capability_metadata.hpp"
#include "guardrails/guardrails.hpp"
#include "mission_executor/action_store.hpp"
#include "mission_executor/action_executor.hpp"
#include "local_planner/local_planner.hpp"
#include "perception/perception_engine.hpp"
#include "safety_supervisor/safety_supervisor.hpp"
#include "state_engine/state_engine.hpp"
#include "state_engine/battery_validity.hpp"

namespace {

class FakeClock final : public icarus::core::Clock {
 public:
  [[nodiscard]] std::int64_t NowUnixMs() const override { return now_ms; }
  std::atomic<std::int64_t> now_ms{1'000'000};
};

void Require(bool condition, const std::string& message) {
  if (!condition) throw std::runtime_error(message);
}

void TestCapabilityPolicyParity() {
  auto policy = icarus::guardrails::SafetyPolicy::Load(
      std::filesystem::path(ICARUS_SOURCE_DIR) / "config/safety/v1.yaml");
  const auto verify = [](const auto& active) {
    const auto capability = icarus::drone_api::MotionLimitsCapability(active);
    const auto& attributes = capability.attributes();
    Require(capability.supported(), "motion policy capability must be supported");
    Require(std::stod(attributes.at("maximum_distance_from_home_m")) ==
                active.maximum_distance_from_home_m, "ordinary fence discovery must match active policy");
    Require(std::stod(attributes.at("maximum_runtime_target_distance_from_home_m")) ==
                active.maximum_runtime_target_distance_from_home_m,
            "perception-derived target fence must retain its separate active policy");
    Require(std::stod(attributes.at("maximum_climb_speed_mps")) ==
                active.maximum_climb_speed_mps, "speed discovery must match active policy exactly");
  };
  verify(policy);
  policy.maximum_distance_from_home_m = 57.125;
  policy.maximum_runtime_target_distance_from_home_m = 22.75;
  policy.maximum_climb_speed_mps = 1.23456789;
  verify(policy);
}

constexpr double kPi = 3.14159265358979323846;

class FakeGateway : public icarus::mavlink_gateway::MavlinkGateway {
 public:
  bool Connect(const std::string&, std::uint16_t) override { return true; }
  void Disconnect() override {}
  [[nodiscard]] bool IsConnected() const override { return true; }
  [[nodiscard]] bool WaitForReady(std::uint32_t) override { return true; }
  void SetStateCallback(StateCallback) override {}
  void SetEventCallback(EventCallback) override {}
  [[nodiscard]] icarus::mavlink_gateway::CommandResult Arm(bool) override {
    ++arm_commands;
    return {true, false, 0, "accepted"};
  }
  [[nodiscard]] icarus::mavlink_gateway::CommandResult Takeoff(double) override {
    ++takeoff_commands;
    return {true, false, 0, "accepted"};
  }
  [[nodiscard]] icarus::mavlink_gateway::CommandResult SetMode(
      std::uint32_t mode) override {
    ++mode_commands;
    last_mode = mode;
    return {true, false, 0, "accepted"};
  }
  [[nodiscard]] icarus::mavlink_gateway::CommandResult SetYaw(double heading) override {
    last_heading = heading;
    return {true, false, 0, "accepted"};
  }
  [[nodiscard]] icarus::mavlink_gateway::CommandResult RotateYaw(
      double angle, double rate) override {
    last_relative_yaw = angle;
    last_yaw_rate = rate;
    return {true, false, 0, "accepted"};
  }
  [[nodiscard]] icarus::mavlink_gateway::CommandResult SetGroundSpeed(double speed) override {
    last_ground_speed = speed;
    ++speed_commands;
    return {accept_speed, false, 0, accept_speed ? "accepted" : "speed rejected"};
  }
  bool GotoGlobal(const icarus::v1::GeoPosition&, std::optional<double> = std::nullopt) override { return true; }
  bool GotoLocal(const icarus::v1::LocalPositionNed&, std::optional<double> = std::nullopt) override { return true; }
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
  auto* home = state.mutable_home()->mutable_geo();
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

icarus::v1::CommandContext Context(const FakeClock& clock,
                                   const std::string& lease_id) {
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

void TestAuthorityPriority() {
  FakeClock clock;
  icarus::drone_api::AuthorityManager authority(clock);
  icarus::v1::AcquireControlRequest dcm;
  dcm.set_session_id("session-1");
  dcm.set_client_id("session-1");
  dcm.set_vehicle_id("icarus-01");
  dcm.set_requested_role(icarus::v1::CONTROL_ROLE_DCM_AUTONOMY);
  dcm.set_requested_lease_ms(2'000);
  const auto first = authority.Acquire(dcm);
  Require(first.granted(), "DCM lease should be granted");

  icarus::v1::AcquireControlRequest operator_request;
  operator_request.set_session_id("operator-session");
  operator_request.set_client_id("operator-session");
  operator_request.set_vehicle_id("icarus-01");
  operator_request.set_requested_role(
      icarus::v1::CONTROL_ROLE_MANUAL_OPERATOR);
  operator_request.set_requested_lease_ms(2'000);
  const auto operator_lease = authority.Acquire(operator_request);
  Require(operator_lease.granted(), "manual operator must preempt DCM");
  Require(!authority.IsAuthorized(Context(clock, first.authority().lease_id())),
          "preempted DCM lease must stop authorizing commands");
  icarus::v1::RenewControlRequest late;
  late.set_session_id("session-1");
  late.set_lease_id(first.authority().lease_id());
  late.set_requested_lease_ms(2'000);
  Require(!authority.Renew(late).granted(), "preempted renewal must fail");
  Require(authority.Current().has_value() &&
              authority.Current()->lease_id() == operator_lease.authority().lease_id(),
          "preempted renewal must preserve the manual operator lease");
  late.set_session_id("operator-session");
  late.set_lease_id("wrong-id");
  Require(!authority.Renew(late).granted() && authority.Current().has_value(),
          "incorrect lease ID must not clear a valid lease");
  late.set_lease_id(operator_lease.authority().lease_id());
  Require(authority.Renew(late).granted(), "current owner must still renew");
}

void TestGuardrails() {
  FakeClock clock;
  const auto policy = icarus::guardrails::SafetyPolicy::Load(
      std::filesystem::path(ICARUS_SOURCE_DIR) / "config/safety/v1.yaml");
  icarus::guardrails::Guardrails guardrails(policy);
  auto state = HealthyLandedState(clock.NowUnixMs());
  state.set_sequence(1);

  icarus::v1::ActionCommand arm;
  *arm.mutable_arm()->mutable_context() = Context(clock, "lease-1");
  Require(guardrails.Validate(arm, state, true, clock.NowUnixMs()).valid(),
          "healthy landed aircraft should pass arm validation");

  icarus::v1::ActionCommand takeoff;
  *takeoff.mutable_takeoff()->mutable_context() = Context(clock, "lease-1");
  takeoff.mutable_takeoff()->set_target_altitude_agl_m(31.0);
  const auto altitude =
      guardrails.Validate(takeoff, state, true, clock.NowUnixMs());
  Require(!altitude.valid() &&
              altitude.reason_code() == icarus::v1::REASON_CODE_NOT_ARMED,
          "takeoff must reject an unarmed aircraft before altitude evaluation");

  state.set_armed(true);
  const auto too_high =
      guardrails.Validate(takeoff, state, true, clock.NowUnixMs());
  Require(!too_high.valid() &&
              too_high.reason_code() == icarus::v1::REASON_CODE_ALTITUDE_LIMIT,
          "takeoff above policy ceiling must be rejected");

  icarus::v1::ActionCommand destination;
  *destination.mutable_goto_()->mutable_context() = Context(clock, "lease-1");
  destination.mutable_goto_()->set_acceptance_radius_m(1.0);
  auto* local = destination.mutable_goto_()
                    ->mutable_destination()
                    ->mutable_local_ned();
  local->set_origin_id("ekf-origin");
  local->set_north_m(101.0);
  local->set_down_m(-10.0);
  const auto outside =
      guardrails.Validate(destination, state, true, clock.NowUnixMs());
  Require(!outside.valid() && outside.reason_code() ==
                                  icarus::v1::REASON_CODE_GEOFENCE_VIOLATION,
          "destination outside policy radius must be rejected");
}

void TestStateAndActionLifecycle() {
  FakeClock clock;
  icarus::state_engine::StateEngine engine(clock);
  engine.Publish(HealthyLandedState(clock.NowUnixMs()));
  const auto snapshot = engine.Snapshot();
  Require(snapshot && snapshot->sequence() == 1,
          "state engine should assign monotonic sequence");
  Require(engine.Health(500).ready_to_arm(),
          "healthy landed state should be ready to arm");
  clock.now_ms += 501;
  Require(!engine.IsFresh(500), "state should expire at configured age");

  icarus::mission_executor::ActionStore actions(clock);
  icarus::v1::ActionCommand arm;
  *arm.mutable_arm()->mutable_context() = Context(clock, "lease-1");
  const auto first = actions.Accept(arm);
  const auto retry = actions.Accept(arm);
  Require(first.action_id() == retry.action_id(),
          "idempotent retry must return original action");
  Require(actions.Transition(first.action_id(),
                             icarus::v1::ACTION_STATE_EXECUTING,
                             icarus::v1::REASON_CODE_OK, "executing", 0.5),
          "accepted action should transition to executing");
  Require(actions.Transition(first.action_id(),
                             icarus::v1::ACTION_STATE_SUCCEEDED,
                             icarus::v1::REASON_CODE_OK, "complete", 1.0,
                             &*snapshot),
          "executing action should transition to success");
  Require(!actions.Transition(first.action_id(),
                              icarus::v1::ACTION_STATE_EXECUTING,
                              icarus::v1::REASON_CODE_OK, "invalid"),
          "terminal action must not restart");
}

void TestStaleStateRecovery() {
  FakeClock clock;
  const auto policy = icarus::guardrails::SafetyPolicy::Load(
      std::filesystem::path(ICARUS_SOURCE_DIR) / "config/safety/v1.yaml");
  icarus::guardrails::Guardrails guardrails(policy);
  icarus::state_engine::StateEngine states(clock);
  auto state = HealthyLandedState(clock.NowUnixMs());
  state.set_armed(true);
  state.set_landed(false);
  state.set_flight_phase(icarus::v1::FLIGHT_PHASE_AIRBORNE);
  states.Publish(state);
  icarus::drone_api::AuthorityManager authority(clock);
  icarus::v1::AcquireControlRequest request;
  request.set_session_id("session-1");
  request.set_client_id("session-1");
  request.set_vehicle_id("icarus-01");
  request.set_requested_role(icarus::v1::CONTROL_ROLE_DCM_AUTONOMY);
  request.set_requested_lease_ms(5'000);
  const auto lease = authority.Acquire(request);
  Require(lease.granted(), "test lease should be granted");
  icarus::mission_executor::ActionStore actions(clock);
  FakeGateway gateway;
  icarus::mission_executor::ActionExecutor executor(
      clock, actions, states, authority, guardrails, gateway);
  executor.Start();
  icarus::v1::ActionCommand command;
  *command.mutable_hold()->mutable_context() =
      Context(clock, lease.authority().lease_id());
  command.mutable_hold()->set_duration_ms(10'000);
  const auto receipt = executor.Submit(command);
  Require(receipt.disposition() == icarus::v1::ACTION_STATE_ACCEPTED,
          "hold should be accepted before failure injection");
  for (int attempt = 0; attempt < 100; ++attempt) {
    const auto status = actions.Find(receipt.action_id());
    if (status && status->state() == icarus::v1::ACTION_STATE_EXECUTING) break;
    std::this_thread::sleep_for(std::chrono::milliseconds(2));
  }
  clock.now_ms += policy.maximum_state_age_ms + 1;
  for (int attempt = 0; attempt < 100; ++attempt) {
    const auto status = actions.Find(receipt.action_id());
    if (status && status->state() ==
                      icarus::v1::ACTION_STATE_ABORTED_BY_SAFETY) {
      Require(status->reason_code() == icarus::v1::REASON_CODE_STATE_STALE,
              "stale state must retain its machine-readable reason");
      Require(gateway.last_mode == 17,
              "stale execution must command ArduPilot BRAKE");
      executor.Stop();
      return;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(5));
  }
  executor.Stop();
  throw std::runtime_error("stale state did not abort active action");
}

void RunRelativeYawSamples(double angle, double start_deg,
                           const std::vector<double>& offsets,
                           bool expect_success, bool stale_attitude = false,
                           bool freeze_updates = false) {
  FakeClock clock;
  const auto policy = icarus::guardrails::SafetyPolicy::Load(
      std::filesystem::path(ICARUS_SOURCE_DIR) / "config/safety/v1.yaml");
  icarus::guardrails::Guardrails guardrails(policy);
  icarus::state_engine::StateEngine states(clock);
  auto state = HealthyLandedState(clock.NowUnixMs());
  state.set_armed(true);
  state.set_landed(false);
  state.set_flight_phase(icarus::v1::FLIGHT_PHASE_AIRBORNE);
  state.mutable_attitude()->set_yaw_rad(start_deg * kPi / 180.0);
  states.Publish(state);
  icarus::drone_api::AuthorityManager authority(clock);
  icarus::v1::AcquireControlRequest request;
  request.set_session_id("session-1");
  request.set_client_id("session-1");
  request.set_vehicle_id("icarus-01");
  request.set_requested_role(icarus::v1::CONTROL_ROLE_DCM_AUTONOMY);
  request.set_requested_lease_ms(30'000);
  const auto lease = authority.Acquire(request);
  Require(lease.granted(), "yaw test lease should be granted");
  icarus::mission_executor::ActionStore actions(clock);
  FakeGateway gateway;
  icarus::mission_executor::ActionExecutor executor(
      clock, actions, states, authority, guardrails, gateway);
  executor.Start();
  icarus::v1::ActionCommand command;
  *command.mutable_yaw()->mutable_context() =
      Context(clock, lease.authority().lease_id());
  command.mutable_yaw()->set_relative_angle_deg(angle);
  command.mutable_yaw()->set_rate_deg_s(30.0);
  const auto receipt = executor.Submit(command);
  Require(receipt.disposition() == icarus::v1::ACTION_STATE_ACCEPTED,
          "bounded relative yaw should be accepted while airborne");
  for (int attempt = 0; attempt < 100 && gateway.last_relative_yaw != angle;
       ++attempt) {
    std::this_thread::sleep_for(std::chrono::milliseconds(2));
  }
  Require(gateway.last_relative_yaw == angle, "yaw command must reach gateway");
  auto publish = [&](double offset, bool stale = false) {
    clock.now_ms += 125;
    state.set_observed_at_unix_ms(clock.NowUnixMs());
    state.set_attitude_received_at_unix_ms(stale ? clock.NowUnixMs() - 1'001 : clock.NowUnixMs());
    state.mutable_attitude()->set_yaw_rad(
        std::remainder((start_deg + offset) * kPi / 180.0, 2.0 * kPi));
    states.Publish(state);
    std::this_thread::sleep_for(std::chrono::milliseconds(130));
  };
  for (double offset : offsets) {
    publish(offset);
    const auto status = actions.Find(receipt.action_id());
    Require(status && status->state() != icarus::v1::ACTION_STATE_SUCCEEDED,
            "yaw must not finish on a moving, incomplete or backtracking sample");
  }
  if (freeze_updates) {
    publish(offsets.back());
    std::this_thread::sleep_for(std::chrono::milliseconds(600));
    const auto frozen = actions.Find(receipt.action_id());
    Require(frozen && frozen->state() == icarus::v1::ACTION_STATE_EXECUTING,
            "re-reading one fresh endpoint snapshot cannot establish settling");
  }
  // Distinct fresh stationary samples, not repeated reads of one snapshot.
  for (int i = 0; i < 7; ++i) publish(offsets.empty() ? 0 : offsets.back(), stale_attitude);
  const auto status = actions.Find(receipt.action_id());
  Require(status.has_value(), "yaw terminal record must remain available");
  if (expect_success) {
    Require(status->state() == icarus::v1::ACTION_STATE_SUCCEEDED,
            "net full requested yaw and settled endpoint should complete");
  } else {
    Require(status->state() != icarus::v1::ACTION_STATE_SUCCEEDED,
            "noise, wrong route, stale attitude or incomplete yaw must not succeed");
  }
  if (stale_attitude) {
    Require(status->state() == icarus::v1::ACTION_STATE_ABORTED_BY_SAFETY,
            "stale attitude must stop yaw verification safely");
  }
  Require(gateway.last_yaw_rate == 30.0, "yaw rate must be preserved");
  executor.Stop();
}

void TestRelativeYawLifecycle() {
  RunRelativeYawSamples(90, 350, {30, 60, 90}, true);  // positive wrap
  RunRelativeYawSamples(-270, 10, {-60, -120, -180, -240, -270}, true);
  RunRelativeYawSamples(270, 350, {60, 120, 180, 240, 270}, true);
  RunRelativeYawSamples(360, 10, {90, 180, 270, 360}, true);
  RunRelativeYawSamples(360, 10, {}, false);  // same heading is not a full turn
  RunRelativeYawSamples(1, 10, {}, false);  // tolerance cannot erase small turns
  RunRelativeYawSamples(1, 10, {1}, true);
  RunRelativeYawSamples(30, 10, {6, 1, 7, 2, 8, 3, 9, 4, 10, 5, 11, 6}, false);
  RunRelativeYawSamples(-270, 10, {-60, -120, -180, -240, -258}, false);
  RunRelativeYawSamples(90, 10, {-30, -60, -90}, false);
  RunRelativeYawSamples(90, 10, {30, 60, 90}, false, true);
  RunRelativeYawSamples(90, 10, {30, 60, 90}, true, false, true);
}

void TestGotoWaitsForStableArrival() {
  FakeClock clock;
  const auto policy = icarus::guardrails::SafetyPolicy::Load(
      std::filesystem::path(ICARUS_SOURCE_DIR) / "config/safety/v1.yaml");
  icarus::guardrails::Guardrails guardrails(policy);
  icarus::state_engine::StateEngine states(clock);
  auto state = HealthyLandedState(clock.NowUnixMs());
  state.set_armed(true);
  state.set_landed(false);
  state.set_flight_phase(icarus::v1::FLIGHT_PHASE_AIRBORNE);
  state.set_ground_speed_mps(2.0);
  state.mutable_attitude()->set_yaw_rad(0.0);
  state.mutable_velocity_ned_mps()->set_x(2.0);
  state.mutable_velocity_ned_mps()->set_z(0.6);
  auto* actual = state.mutable_local_position_ned();
  actual->set_origin_id("ekf-origin");
  actual->set_north_m(4.0);
  actual->set_east_m(0.0);
  actual->set_down_m(-4.0);
  states.Publish(state);

  icarus::drone_api::AuthorityManager authority(clock);
  icarus::v1::AcquireControlRequest request;
  request.set_session_id("session-1");
  request.set_client_id("session-1");
  request.set_vehicle_id("icarus-01");
  request.set_requested_role(icarus::v1::CONTROL_ROLE_DCM_AUTONOMY);
  request.set_requested_lease_ms(5'000);
  const auto lease = authority.Acquire(request);
  Require(lease.granted(), "goto test lease should be granted");

  icarus::mission_executor::ActionStore actions(clock);
  FakeGateway gateway;
  icarus::mission_executor::ActionExecutor executor(
      clock, actions, states, authority, guardrails, gateway);
  executor.Start();
  icarus::v1::ActionCommand command;
  *command.mutable_goto_()->mutable_context() =
      Context(clock, lease.authority().lease_id());
  *command.mutable_goto_()->mutable_destination()->mutable_local_ned() = *actual;
  command.mutable_goto_()->set_acceptance_radius_m(1.5);
  command.mutable_goto_()->mutable_heading()->set_mode(
      icarus::v1::HEADING_MODE_KEEP_CURRENT);
  command.mutable_goto_()->mutable_limits()->set_execution_timeout_ms(5'000);
  const auto receipt = executor.Submit(command);
  Require(receipt.disposition() == icarus::v1::ACTION_STATE_ACCEPTED,
          "goto should be accepted while airborne");
  std::this_thread::sleep_for(std::chrono::milliseconds(150));
  auto status = actions.Find(receipt.action_id());
  Require(status && status->state() == icarus::v1::ACTION_STATE_EXECUTING,
          "goto must not finish while crossing the target at flight speed");

  state.set_ground_speed_mps(0.0);
  state.mutable_velocity_ned_mps()->set_x(0.0);
  state.mutable_velocity_ned_mps()->set_z(0.0);
  states.Publish(state);
  for (int attempt = 0; attempt < 160; ++attempt) {
    status = actions.Find(receipt.action_id());
    if (status && status->state() == icarus::v1::ACTION_STATE_SUCCEEDED) {
      executor.Stop();
      return;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(5));
  }
  executor.Stop();
  throw std::runtime_error("goto did not complete after a stable arrival dwell");
}

void TestPerceptionLossRecovery() {
  icarus::core::SystemClock clock;
  const auto policy = icarus::guardrails::SafetyPolicy::Load(
      std::filesystem::path(ICARUS_SOURCE_DIR) / "config/safety/v1.yaml");
  icarus::state_engine::StateEngine states(clock);
  auto state = HealthyLandedState(clock.NowUnixMs());
  state.set_armed(true);
  state.set_landed(false);
  state.set_flight_phase(icarus::v1::FLIGHT_PHASE_AIRBORNE);
  states.Publish(state);
  icarus::mission_executor::ActionStore actions(clock);
  icarus::drone_api::AuthorityManager authority(clock);
  icarus::v1::AcquireControlRequest request;
  request.set_session_id("session-1");
  request.set_client_id("session-1");
  request.set_vehicle_id("icarus-01");
  request.set_requested_role(icarus::v1::CONTROL_ROLE_DCM_AUTONOMY);
  request.set_requested_lease_ms(5'000);
  Require(authority.Acquire(request).granted(),
          "perception test lease should be granted");
  FakeGateway gateway;
  icarus::perception::PerceptionEngine perception(
      clock, states, policy.maximum_perception_age_ms);
  icarus::safety_supervisor::SafetySupervisor supervisor(
      clock, policy, states, actions, authority, gateway, &perception);
  supervisor.Start();
  for (int attempt = 0; attempt < 100; ++attempt) {
    if (gateway.last_mode == 17) {
      supervisor.Stop();
      return;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(5));
  }
  supervisor.Stop();
  throw std::runtime_error("perception loss did not command BRAKE");
}

void TestVerticalLandingSurvivesLateralPerceptionLoss() {
  FakeClock clock;
  const auto policy = icarus::guardrails::SafetyPolicy::Load(
      std::filesystem::path(ICARUS_SOURCE_DIR) / "config/safety/v1.yaml");
  for (const bool landing : {true, false}) {
    icarus::state_engine::StateEngine states(clock);
    auto state = HealthyLandedState(clock.NowUnixMs());
    state.set_armed(true);
    state.set_landed(false);
    states.Publish(state);
    icarus::drone_api::AuthorityManager authority(clock);
    icarus::v1::AcquireControlRequest request;
    request.set_session_id("session-1");
    request.set_client_id("session-1");
    request.set_vehicle_id("icarus-01");
    request.set_requested_role(icarus::v1::CONTROL_ROLE_DCM_AUTONOMY);
    request.set_requested_lease_ms(5000);
    Require(authority.Acquire(request).granted(), "landing recovery lease");
    icarus::mission_executor::ActionStore actions(clock);
    icarus::v1::ActionCommand command;
    if (landing) *command.mutable_land()->mutable_context() = Context(clock, "lease");
    else *command.mutable_return_home()->mutable_context() = Context(clock, "lease");
    const auto receipt = actions.Accept(command);
    Require(actions.Transition(receipt.action_id(), icarus::v1::ACTION_STATE_EXECUTING,
                icarus::v1::REASON_CODE_OK, "executing"), "start recovery action");
    FakeGateway gateway;
    icarus::perception::PerceptionEngine perception(clock, states, policy.maximum_perception_age_ms);
    icarus::safety_supervisor::SafetySupervisor supervisor(
        clock, policy, states, actions, authority, gateway, &perception);
    supervisor.Start();
    std::this_thread::sleep_for(std::chrono::milliseconds(220));
    supervisor.Stop();
    if (landing) {
      Require(gateway.mode_commands == 0 &&
              actions.Find(receipt.action_id())->state() == icarus::v1::ACTION_STATE_EXECUTING,
              "loss of lateral LiDAR must not interrupt vertical landing");
    } else {
      Require(gateway.last_mode == 17 &&
              actions.Find(receipt.action_id())->state() == icarus::v1::ACTION_STATE_ABORTED_BY_SAFETY,
              "return-home still requires fresh lateral perception");
    }
  }
}

void TestControllerHeartbeatLossRecovery() {
  FakeClock clock;
  const auto policy = icarus::guardrails::SafetyPolicy::Load(
      std::filesystem::path(ICARUS_SOURCE_DIR) / "config/safety/v1.yaml");
  icarus::state_engine::StateEngine states(clock);
  auto state = HealthyLandedState(clock.NowUnixMs());
  state.set_armed(true);
  state.set_landed(false);
  state.set_flight_phase(icarus::v1::FLIGHT_PHASE_AIRBORNE);
  states.Publish(state);
  icarus::mission_executor::ActionStore actions(clock);
  icarus::drone_api::AuthorityManager authority(clock);
  icarus::v1::AcquireControlRequest request;
  request.set_session_id("dcm-session");
  request.set_client_id("dcm-process");
  request.set_vehicle_id("icarus-01");
  request.set_requested_role(icarus::v1::CONTROL_ROLE_DCM_AUTONOMY);
  request.set_requested_lease_ms(250);
  Require(authority.Acquire(request).granted(),
          "heartbeat test lease should be granted");
  FakeGateway gateway;
  icarus::safety_supervisor::SafetySupervisor supervisor(
      clock, policy, states, actions, authority, gateway);
  supervisor.Start();

  clock.now_ms += 251;
  state.set_observed_at_unix_ms(clock.NowUnixMs());
  states.Publish(state);
  for (int attempt = 0; attempt < 100 && gateway.last_mode != 17; ++attempt) {
    std::this_thread::sleep_for(std::chrono::milliseconds(5));
  }
  Require(gateway.last_mode == 17,
          "expired DCM heartbeat must command ArduPilot BRAKE");

  clock.now_ms += policy.control_lease_lost_rtl_delay_ms + 1;
  state.set_observed_at_unix_ms(clock.NowUnixMs());
  states.Publish(state);
  for (int attempt = 0; attempt < 100 && gateway.last_mode != 6; ++attempt) {
    std::this_thread::sleep_for(std::chrono::milliseconds(5));
  }
  supervisor.Stop();
  Require(gateway.last_mode == 6,
          "sustained DCM heartbeat loss must escalate to RTL");
}

icarus::v1::Position Local(double north, double east, double down = -5.0) {
  icarus::v1::Position position;
  auto* local = position.mutable_local_ned();
  local->set_origin_id("ekf-origin");
  local->set_north_m(north);
  local->set_east_m(east);
  local->set_down_m(down);
  return position;
}

// Hovering, settled, facing north at the given local position.
icarus::v1::DroneState AirborneState(const FakeClock& clock, double north,
                                     double east, double down = -5.0) {
  auto state = HealthyLandedState(clock.NowUnixMs());
  state.set_armed(true);
  state.set_landed(false);
  state.set_flight_phase(icarus::v1::FLIGHT_PHASE_AIRBORNE);
  state.mutable_velocity_ned_mps()->set_x(0.0);
  state.mutable_attitude()->set_yaw_rad(0.0);
  *state.mutable_local_position_ned() = Local(north, east, down).local_ned();
  state.set_relative_altitude_m(-down);
  auto* geo = state.mutable_position()->mutable_geo();
  geo->set_latitude_deg(28.0 + north / 6371000.0 * 180.0 / kPi);
  geo->set_longitude_deg(77.0 + east / (6371000.0 * std::cos(28.0 * kPi / 180.0)) * 180.0 / kPi);
  return state;
}

void TestMeasuredAltitudeFrames() {
  FakeClock clock;
  auto state = AirborneState(clock, 0.0, 0.0, -8.0);
  state.set_sequence(1);
  state.set_relative_altitude_m(3.0);  // Home is down=-5, not the EKF origin.
  const auto home_down = icarus::guardrails::LocalHomeDownM(state);
  Require(home_down && *home_down == -5.0, "derive home/origin offset from measurements");
  auto target = Local(4.0, 2.0, -3.0);
  target.mutable_local_ned()->set_altitude_above_home_m(3.0);
  auto resolved = icarus::guardrails::ResolveLocalAltitude(target, state);
  Require(resolved && resolved->local_ned().down_m() == -8.0 &&
              !resolved->local_ned().has_altitude_above_home_m(),
          "above-home request must hold height despite nonzero origin offset");
  target.mutable_local_ned()->clear_altitude_above_home_m();
  resolved = icarus::guardrails::ResolveLocalAltitude(target, state);
  Require(resolved && resolved->local_ned().down_m() == -3.0,
          "genuine NED requests retain their explicit local down coordinate");
  icarus::v1::OrbitRequest orbit;
  *orbit.mutable_center() = Local(0, 0);
  orbit.set_altitude_m(3.0);
  orbit.set_altitude_frame(icarus::v1::COORDINATE_FRAME_WGS84_ABOVE_HOME);
  const auto orbital = icarus::guardrails::OrbitAltitudePosition(orbit, state);
  Require(orbital && orbital->local_ned().down_m() == -8.0,
          "orbit uses the same measured altitude transform as goto");
  state.mutable_home()->mutable_geo()->set_frame(icarus::v1::COORDINATE_FRAME_WGS84_AMSL);
  state.mutable_home()->mutable_geo()->set_altitude_m(600.0);
  orbit.set_altitude_frame(icarus::v1::COORDINATE_FRAME_WGS84_AMSL);
  orbit.set_altitude_m(603.0);
  Require(icarus::guardrails::OrbitAltitudePosition(orbit, state)->local_ned().down_m() == -8.0,
          "AMSL orbit converts via measured home and local offset");
  orbit.set_altitude_frame(icarus::v1::COORDINATE_FRAME_WGS84_ABOVE_TERRAIN);
  Require(!icarus::guardrails::OrbitAltitudePosition(orbit, state),
          "local orbit cannot guess an unmeasured terrain frame");
  const auto healthy = state;
  const auto policy = icarus::guardrails::SafetyPolicy::Load(
      std::filesystem::path(ICARUS_SOURCE_DIR) / "config/safety/v1.yaml");
  icarus::guardrails::Guardrails guardrails(policy);
  icarus::v1::ActionCommand go;
  *go.mutable_goto_()->mutable_context() = Context(clock, "lease-1");
  *go.mutable_goto_()->mutable_destination() = Local(1.0, 0.0, -53.0);
  go.mutable_goto_()->set_acceptance_radius_m(1.0);
  state.mutable_local_position_ned()->set_down_m(-53.0);
  state.set_relative_altitude_m(3.0);
  Require(guardrails.Validate(go, state, true, clock.NowUnixMs()).valid(),
          "altitude fence uses height above home, not distance above EKF origin");
  go.mutable_goto_()->mutable_destination()->mutable_local_ned()->set_altitude_above_home_m(31.0);
  Require(guardrails.Validate(go, state, true, clock.NowUnixMs()).reason_code() ==
              icarus::v1::REASON_CODE_ALTITUDE_LIMIT,
          "explicit above-home altitude must still obey the safety ceiling");
  state = healthy;
  state.set_local_position_received_at_unix_ms(0);
  Require(!icarus::guardrails::LocalHomeDownM(state), "missing local timestamp cannot be guessed");
  state = healthy;
  state.set_relative_altitude_received_at_unix_ms(clock.NowUnixMs() - 1001);
  Require(!icarus::guardrails::LocalHomeDownM(state), "fresh aggregate cannot hide stale altitude");
  state = healthy;
  state.set_local_position_time_boot_ms(1400);
  Require(!icarus::guardrails::LocalHomeDownM(state), "incoherent sample pair cannot define a frame");
  state = healthy;
  state.set_local_position_time_boot_ms(1100);
  state.mutable_velocity_ned_mps()->set_z(2.0);
  Require(std::abs(*icarus::guardrails::LocalHomeDownM(state) + 5.2) < 1e-9,
          "vertical velocity aligns nearby but asynchronous sample times");
  state = healthy;
  state.set_relative_altitude_m(std::numeric_limits<double>::quiet_NaN());
  Require(!icarus::guardrails::LocalHomeDownM(state), "nonfinite altitude cannot define a frame");
  state = healthy;
  auto* measured = state.mutable_home_local_position_ned();
  measured->set_origin_id(state.local_position_ned().origin_id());
  measured->set_north_m(7.0);
  measured->set_east_m(-3.0);
  measured->set_down_m(-5.0);
  state.set_home_position_received_at_unix_ms(clock.NowUnixMs() - 10'000);
  state.set_local_position_time_boot_ms(1400);
  Require(icarus::guardrails::LocalHomeDownM(state) == -5.0 &&
              icarus::guardrails::LocalHomePosition(state)->north_m() == 7.0,
          "direct autopilot home frame must not depend on asynchronous pose pairs");
  measured->set_origin_id("old-frame");
  Require(!icarus::guardrails::LocalHomeDownM(state), "native home must match current EKF origin");
  state = healthy;
  target.mutable_local_ned()->set_origin_id("different-origin");
  Require(!icarus::guardrails::ResolveLocalAltitude(target, state),
          "unknown local origins must not be silently merged");
}

void TestHomeGeofenceIgnoresEstimatorOriginTranslation() {
  FakeClock clock;
  auto dateline = AirborneState(clock, 0.0, 0.0);
  dateline.mutable_position()->mutable_geo()->set_latitude_deg(0.0);
  dateline.mutable_home()->mutable_geo()->set_latitude_deg(0.0);
  dateline.mutable_position()->mutable_geo()->set_longitude_deg(179.999);
  dateline.mutable_home()->mutable_geo()->set_longitude_deg(-179.999);
  const auto dateline_home = icarus::guardrails::LocalHomePosition(dateline);
  Require(dateline_home && std::abs(dateline_home->east_m() - 222.38985) < 0.01,
          "home transform wraps longitude across the international date line");
  const auto policy = icarus::guardrails::SafetyPolicy::Load(
      std::filesystem::path(ICARUS_SOURCE_DIR) / "config/safety/v1.yaml");
  icarus::guardrails::Guardrails guardrails(policy);
  for (const auto& shift : {std::pair{0.0, 0.0}, std::pair{500.0, -300.0}}) {
    auto state = AirborneState(clock, 10.0, 20.0);
    state.set_sequence(1);
    state.mutable_local_position_ned()->set_north_m(10.0 + shift.first);
    state.mutable_local_position_ned()->set_east_m(20.0 + shift.second);
    const auto home = icarus::guardrails::LocalHomePosition(state);
    Require(home && std::abs(home->north_m() - shift.first) < 0.001 &&
                std::abs(home->east_m() - shift.second) < 0.001,
            "measured home transforms into a translated EKF coordinate frame");
    icarus::v1::ActionCommand go;
    *go.mutable_goto_()->mutable_context() = Context(clock, "lease-1");
    *go.mutable_goto_()->mutable_destination() = Local(60.0 + shift.first, shift.second);
    go.mutable_goto_()->set_acceptance_radius_m(1.0);
    Require(guardrails.Validate(go, state, true, clock.NowUnixMs()).valid(),
            "same physical destination stays admitted after origin translation");
    go.mutable_goto_()->mutable_destination()->mutable_local_ned()->set_north_m(101.0 + shift.first);
    Require(guardrails.Validate(go, state, true, clock.NowUnixMs()).reason_code() ==
                icarus::v1::REASON_CODE_GEOFENCE_VIOLATION,
            "same physical out-of-fence destination stays rejected after origin translation");
    icarus::v1::ActionCommand circle;
    auto* orbit = circle.mutable_orbit();
    *orbit->mutable_context() = Context(clock, "lease-1");
    *orbit->mutable_center() = Local(
        policy.maximum_runtime_target_distance_from_home_m - 5.0 + shift.first,
        shift.second);
    orbit->set_radius_m(4.0);
    orbit->set_altitude_m(5.0);
    orbit->set_ground_speed_mps(1.0);
    orbit->set_revolutions(1.0);
    orbit->set_target_source(icarus::v1::TARGET_SOURCE_RUNTIME_ESTIMATE);
    Require(guardrails.Validate(circle, state, true, clock.NowUnixMs()).valid(),
            "runtime orbit fence is measured from home after origin translation");
    orbit->set_radius_m(6.0);
    Require(guardrails.Validate(circle, state, true, clock.NowUnixMs()).reason_code() ==
                icarus::v1::REASON_CODE_GEOFENCE_VIOLATION,
            "far side of translated orbit still obeys runtime home fence");
    state.clear_home();
    Require(!icarus::guardrails::LocalHomePosition(state),
            "unmeasured home cannot be assumed to equal estimator origin");
  }
}

void TestGeographicAltitudeNeedsKnownFrame() {
  FakeClock clock;
  auto state = AirborneState(clock, 0.0, 0.0);
  state.set_sequence(1);
  const auto policy = icarus::guardrails::SafetyPolicy::Load(
      std::filesystem::path(ICARUS_SOURCE_DIR) / "config/safety/v1.yaml");
  icarus::guardrails::Guardrails guardrails(policy);
  icarus::v1::ActionCommand command;
  auto* go = command.mutable_goto_();
  *go->mutable_context() = Context(clock, "lease-1");
  *go->mutable_destination() = state.home();
  go->set_acceptance_radius_m(1.0);
  auto* geo = go->mutable_destination()->mutable_geo();
  geo->set_altitude_m(5.0);
  geo->set_frame(icarus::v1::COORDINATE_FRAME_WGS84_ABOVE_TERRAIN);
  Require(guardrails.Validate(command, state, true, clock.NowUnixMs()).reason_code() ==
              icarus::v1::REASON_CODE_UNSUPPORTED,
          "unmeasured terrain-relative height cannot bypass the altitude fence");
  geo->set_altitude_m(605.0);
  geo->set_frame(icarus::v1::COORDINATE_FRAME_WGS84_AMSL);
  Require(guardrails.Validate(command, state, true, clock.NowUnixMs()).reason_code() ==
              icarus::v1::REASON_CODE_POSITION_UNAVAILABLE,
          "AMSL request cannot guess an above-home-only home reference");
  state.mutable_home()->mutable_geo()->set_frame(icarus::v1::COORDINATE_FRAME_WGS84_AMSL);
  state.mutable_home()->mutable_geo()->set_altitude_m(600.0);
  Require(guardrails.Validate(command, state, true, clock.NowUnixMs()).valid(),
          "measured AMSL home admits the correct above-home destination");
  geo->set_altitude_m(631.0);
  Require(guardrails.Validate(command, state, true, clock.NowUnixMs()).reason_code() ==
              icarus::v1::REASON_CODE_ALTITUDE_LIMIT,
          "AMSL destination above ceiling is rejected after frame conversion");
  state.clear_home();
  Require(guardrails.Validate(command, state, true, clock.NowUnixMs()).reason_code() ==
              icarus::v1::REASON_CODE_HOME_UNAVAILABLE,
          "missing home cannot silently disable geographic geofence");
  icarus::v1::OrbitRequest orbit;
  *orbit.mutable_center() = command.goto_().destination();
  orbit.set_radius_m(3.0);
  orbit.set_revolutions(1.0);
  orbit.set_altitude_m(5.0);
  const auto point = icarus::mission_executor::OrbitWaypoint(orbit, 0.0, 0);
  Require(point.geo().frame() == icarus::v1::COORDINATE_FRAME_WGS84_ABOVE_HOME,
          "default geographic orbit altitude is explicitly above home on the wire");
}

std::string AcquireLease(icarus::drone_api::AuthorityManager& authority) {
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

icarus::v1::ActionStatus WaitForTerminal(
    const icarus::mission_executor::ActionStore& actions,
    const std::string& action_id, int timeout_ms,
    std::set<std::uint32_t>* viewpoints = nullptr) {
  for (int waited = 0; waited < timeout_ms; waited += 2) {
    const auto status = actions.Find(action_id);
    if (status && viewpoints != nullptr && status->active_viewpoint() > 0) {
      viewpoints->insert(status->active_viewpoint());
    }
    if (status && Terminal(status->state())) return *status;
    std::this_thread::sleep_for(std::chrono::milliseconds(2));
  }
  throw std::runtime_error("action did not reach a terminal state");
}

// Moves the simulated aircraft to every position target and heading as soon
// as it is commanded, so executor sequencing is testable without autopilot
// dynamics. Records what was commanded, in order.
class TeleportGateway final : public FakeGateway {
 public:
  TeleportGateway(icarus::state_engine::StateEngine& states,
                  icarus::v1::DroneState state)
      : states_(states), state_(std::move(state)) {}
  [[nodiscard]] icarus::mavlink_gateway::CommandResult SetYaw(
      double heading) override {
    std::lock_guard<std::mutex> lock(mutex_);
    headings_.push_back(heading);
    state_.mutable_attitude()->set_yaw_rad(heading * kPi / 180.0);
    states_.Publish(state_);
    return {true, false, 0, "accepted"};
  }
  bool GotoLocal(const icarus::v1::LocalPositionNed& target,
                 std::optional<double> heading_deg = std::nullopt) override {
    std::lock_guard<std::mutex> lock(mutex_);
    targets_.push_back(target);
    const auto home_down = icarus::guardrails::LocalHomeDownM(state_);
    if (state_.position().has_geo()) {
      auto* geo = state_.mutable_position()->mutable_geo();
      const auto previous = state_.local_position_ned();
      geo->set_longitude_deg(geo->longitude_deg() +
          (target.east_m() - previous.east_m()) /
          (6371000.0 * std::cos(geo->latitude_deg() * kPi / 180.0)) * 180.0 / kPi);
      geo->set_latitude_deg(geo->latitude_deg() +
          (target.north_m() - previous.north_m()) / 6371000.0 * 180.0 / kPi);
    }
    *state_.mutable_local_position_ned() = target;
    if (home_down) state_.set_relative_altitude_m(*home_down - target.down_m());
    if (simulate_moving_arrival) state_.set_ground_speed_mps(1.5);
    if (simulate_path_yaw) state_.mutable_attitude()->set_yaw_rad(0.0);
    if (simulate_first_position_reset && targets_.size() == 1) {
      last_ground_speed = 5.0;
    }
    if (heading_deg) state_.mutable_attitude()->set_yaw_rad(*heading_deg * kPi / 180.0);
    position_speeds_.push_back(last_ground_speed);
    position_headings_.push_back(heading_deg);
    states_.Publish(state_);
    return true;
  }
  bool StreamLocal(const icarus::v1::LocalPositionNed& target,
                   double north_mps, double east_mps, double down_mps,
                   std::optional<double> heading_deg = std::nullopt) override {
    ++stream_commands;
    if (reject_stream) return false;
    if (freeze_stream) return true;
    if (std::hypot(north_mps, east_mps) > 0.01) ++moving_stream_commands;
    if (std::abs(down_mps) > 1e-9) return false;
    return GotoLocal(target, heading_deg);
  }
  std::atomic<int> stream_commands{0};
  std::atomic<int> moving_stream_commands{0};
  bool reject_stream = false;
  bool freeze_stream = false;
  void SetArrivalMotion(double speed, double offset = 0.0) {
    std::lock_guard<std::mutex> lock(mutex_);
    state_.set_ground_speed_mps(speed);
    if (!targets_.empty()) {
      *state_.mutable_local_position_ned() = targets_.back();
      state_.mutable_local_position_ned()->set_north_m(
          state_.local_position_ned().north_m() + offset);
    }
    states_.Publish(state_);
  }
  bool simulate_moving_arrival = false;
  bool simulate_path_yaw = false;
  bool simulate_first_position_reset = false;
  [[nodiscard]] std::vector<double> PositionSpeeds() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return position_speeds_;
  }
  [[nodiscard]] std::vector<std::optional<double>> PositionHeadings() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return position_headings_;
  }
  [[nodiscard]] std::vector<double> Headings() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return headings_;
  }
  [[nodiscard]] std::vector<icarus::v1::LocalPositionNed> Targets() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return targets_;
  }

 private:
  icarus::state_engine::StateEngine& states_;
  mutable std::mutex mutex_;
  icarus::v1::DroneState state_;
  std::vector<double> headings_;
  std::vector<double> position_speeds_;
  std::vector<std::optional<double>> position_headings_;
  std::vector<icarus::v1::LocalPositionNed> targets_;
};

void TestGroundSpeedAppliedResetAndRejected() {
  FakeClock clock;
  const auto policy = icarus::guardrails::SafetyPolicy::Load(
      std::filesystem::path(ICARUS_SOURCE_DIR) / "config/safety/v1.yaml");
  icarus::guardrails::Guardrails guardrails(policy);
  icarus::state_engine::StateEngine states(clock);
  const auto state = AirborneState(clock, 0.0, 0.0);
  states.Publish(state);
  icarus::drone_api::AuthorityManager authority(clock);
  const auto lease = AcquireLease(authority);
  icarus::mission_executor::ActionStore actions(clock);
  TeleportGateway gateway(states, state);
  gateway.simulate_first_position_reset = true;
  icarus::mission_executor::ActionExecutor executor(
      clock, actions, states, authority, guardrails, gateway);
  executor.Start();
  const auto submit = [&](double north, std::optional<double> speed) {
    icarus::v1::ActionCommand command;
    auto* go = command.mutable_goto_();
    *go->mutable_context() = Context(clock, lease);
    go->mutable_context()->set_request_id("speed-" + std::to_string(north));
    go->mutable_context()->set_idempotency_key("speed-" + std::to_string(north));
    *go->mutable_destination() = Local(north, 0.0);
    go->set_acceptance_radius_m(1.0);
    if (speed) go->mutable_limits()->set_maximum_ground_speed_mps(*speed);
    const auto receipt = executor.Submit(command);
    Require(receipt.disposition() == icarus::v1::ACTION_STATE_ACCEPTED,
            "speed test goto must be accepted: " + receipt.message());
    return WaitForTerminal(actions, receipt.action_id(), 3000);
  };
  auto result = submit(2.0, 0.75);
  Require(result.state() == icarus::v1::ACTION_STATE_SUCCEEDED &&
              std::abs(gateway.last_ground_speed - 0.75) < 1e-9,
          "requested speed must reach acknowledged controller before navigation");
  Require(gateway.Targets().size() == 2 &&
              std::abs(gateway.Targets()[0].north_m()) < 1e-9 &&
              std::abs(gateway.PositionSpeeds()[1] - 0.75) < 1e-9,
          "hold must initialize/reset controller before destination speed is applied");
  result = submit(4.0, std::nullopt);
  Require(result.state() == icarus::v1::ACTION_STATE_SUCCEEDED &&
              std::abs(gateway.last_ground_speed - 3.0) < 1e-9,
          "omitted speed must reset previous slow controller state");
  const auto before = gateway.Targets().size();
  gateway.accept_speed = false;
  result = submit(6.0, 1.0);
  executor.Stop();
  Require(result.state() == icarus::v1::ACTION_STATE_FAILED &&
              result.message().find("ground speed command failed") != std::string::npos,
          "a rejected speed request must not be reported successful");
  Require(gateway.Targets().size() == before + 1 && gateway.last_mode == 17 &&
              std::abs(gateway.Targets().back().north_m() - 4.0) < 1e-9,
          "speed rejection may initialize current hold but must not send requested destination");
}

void TestKeepCurrentHeadingSurvivesNavigationTargets() {
  FakeClock clock;
  const auto policy = icarus::guardrails::SafetyPolicy::Load(
      std::filesystem::path(ICARUS_SOURCE_DIR) / "config/safety/v1.yaml");
  icarus::guardrails::Guardrails guardrails(policy);
  icarus::state_engine::StateEngine states(clock);
  auto state = AirborneState(clock, 5.0, 0.0, -8.0);
  state.set_relative_altitude_m(5.0);  // Home down is -3m.
  state.mutable_attitude()->set_yaw_rad(kPi / 2.0);
  states.Publish(state);
  icarus::drone_api::AuthorityManager authority(clock);
  const auto lease = AcquireLease(authority);
  icarus::mission_executor::ActionStore actions(clock);
  TeleportGateway gateway(states, state);
  gateway.simulate_path_yaw = true;
  icarus::mission_executor::ActionExecutor executor(
      clock, actions, states, authority, guardrails, gateway);
  executor.Start();
  icarus::v1::ActionCommand go;
  *go.mutable_goto_()->mutable_context() = Context(clock, lease);
  *go.mutable_goto_()->mutable_destination() = Local(6.0, 0.0);
  go.mutable_goto_()->mutable_destination()->mutable_local_ned()->set_altitude_above_home_m(5.0);
  go.mutable_goto_()->set_acceptance_radius_m(1.0);
  go.mutable_goto_()->mutable_heading()->set_mode(icarus::v1::HEADING_MODE_KEEP_CURRENT);
  auto receipt = executor.Submit(go);
  auto status = WaitForTerminal(actions, receipt.action_id(), 3000);
  Require(status.state() == icarus::v1::ACTION_STATE_SUCCEEDED,
          "goto must restore captured heading after position target changes yaw");
  icarus::v1::ActionCommand arc;
  auto* orbit = arc.mutable_orbit();
  *orbit->mutable_context() = Context(clock, lease);
  orbit->mutable_context()->set_idempotency_key("retained-orbit");
  *orbit->mutable_center() = Local(0.0, 0.0);
  orbit->set_radius_m(5.0);
  orbit->set_altitude_m(5.0);
  orbit->set_ground_speed_mps(2.0);
  orbit->set_clockwise(true);
  orbit->set_revolutions(0.25);
  orbit->mutable_heading()->set_mode(icarus::v1::HEADING_MODE_KEEP_CURRENT);
  receipt = executor.Submit(arc);
  status = WaitForTerminal(actions, receipt.action_id(), 12'000);
  executor.Stop();
  Require(status.state() == icarus::v1::ACTION_STATE_SUCCEEDED,
          "orbit must retain entry heading on every segment");
  for (const auto& target : gateway.Targets()) {
    Require(std::abs(target.down_m() + 8.0) < 1e-9 &&
                !target.has_altitude_above_home_m(),
            "goto and orbit send resolved EKF down while preserving above-home height");
  }
  const auto headings = gateway.Headings();
  const auto position_headings = gateway.PositionHeadings();
  Require(position_headings.size() == gateway.Targets().size(),
          "each position target must carry its heading atomically");
  for (const auto heading : position_headings) {
    Require(heading && std::abs(*heading - 90.0) < 1e-9,
            "position target must not reset heading before a later yaw command");
  }
  for (const double heading : headings) {
    Require(std::abs(heading - 90.0) < 1e-9,
            "keep-current must use entry heading, not changed path heading");
  }
  Require(std::abs(states.Snapshot()->attitude().yaw_rad() - kPi / 2.0) < 1e-9,
          "completed navigation must leave requested heading restored");
}

void TestTargetRelativeGuardrails() {
  FakeClock clock;
  const auto policy = icarus::guardrails::SafetyPolicy::Load(
      std::filesystem::path(ICARUS_SOURCE_DIR) / "config/safety/v1.yaml");
  Require(policy.hard_minimum_clearance_m == 2.0 &&
              policy.maximum_runtime_target_distance_from_home_m ==
                  policy.maximum_distance_from_home_m,
          "absent runtime override must inherit the established general fence");
  Require(policy.maximum_distance_from_home_m == 100.0,
          "the established general simulation fence remains 100 m");
  icarus::guardrails::Guardrails default_guardrails(policy);
  // Exercise explicit policy-file configuration separately from the default.
  auto configured = YAML::LoadFile(
      (std::filesystem::path(ICARUS_SOURCE_DIR) / "config/safety/v1.yaml").string());
  configured["geofence"]["max_runtime_target_distance_from_home_m"] = 30.0;
  const auto temporary = std::filesystem::temp_directory_path() /
      ("icarus-runtime-policy-" + std::to_string(
          std::chrono::steady_clock::now().time_since_epoch().count()) + ".yaml");
  { std::ofstream output(temporary); output << configured; }
  const auto explicit_policy = icarus::guardrails::SafetyPolicy::Load(temporary);
  std::filesystem::remove(temporary);
  Require(explicit_policy.maximum_runtime_target_distance_from_home_m == 30.0,
          "an operator-configured tighter runtime fence must still load");
  icarus::guardrails::Guardrails guardrails(explicit_policy);
  auto state = AirborneState(clock, 0.0, 0.0);
  state.set_sequence(1);
  const auto validate = [&](const icarus::v1::ActionCommand& command) {
    return guardrails.Validate(command, state, true, clock.NowUnixMs());
  };
  const auto goto_to = [&](double north, double east, double down = -5.0) {
    icarus::v1::ActionCommand command;
    *command.mutable_goto_()->mutable_context() = Context(clock, "lease-1");
    command.mutable_goto_()->set_acceptance_radius_m(1.0);
    *command.mutable_goto_()->mutable_destination() = Local(north, east, down);
    return command;
  };

  auto runtime = goto_to(29.0, 0.0);
  runtime.mutable_goto_()->set_target_source(
      icarus::v1::TARGET_SOURCE_RUNTIME_ESTIMATE);
  runtime.mutable_goto_()->mutable_destination()->mutable_local_ned()->set_north_m(99.0);
  Require(default_guardrails.Validate(runtime, state, true, clock.NowUnixMs()).valid(),
          "default measured targets use the same 100 m fence as other targets");
  runtime.mutable_goto_()->mutable_destination()->mutable_local_ned()->set_north_m(101.0);
  Require(default_guardrails.Validate(runtime, state, true, clock.NowUnixMs()).reason_code() ==
              icarus::v1::REASON_CODE_GEOFENCE_VIOLATION,
          "default measured targets still obey the established 100 m fence");
  runtime.mutable_goto_()->mutable_destination()->mutable_local_ned()->set_north_m(29.0);
  Require(validate(runtime).valid(),
          "a runtime target inside 30 m must be accepted");
  runtime.mutable_goto_()->mutable_destination()->mutable_local_ned()
      ->set_north_m(31.0);
  Require(validate(runtime).reason_code() ==
              icarus::v1::REASON_CODE_GEOFENCE_VIOLATION,
          "a runtime target beyond 30 m must breach the runtime fence");
  Require(validate(goto_to(31.0, 0.0)).valid(),
          "a declared target keeps the ordinary 100 m fence");
  Require(validate(goto_to(5.0, 0.0, -31.0)).reason_code() ==
              icarus::v1::REASON_CODE_ALTITUDE_LIMIT,
          "a target above the ceiling must be refused");
  Require(validate(goto_to(std::numeric_limits<double>::quiet_NaN(), 0.0))
                  .reason_code() == icarus::v1::REASON_CODE_INVALID_ARGUMENT,
          "a non-finite target must not slip past the fence comparison");

  auto face = goto_to(5.0, 0.0);
  face.mutable_goto_()->mutable_heading()->set_mode(
      icarus::v1::HEADING_MODE_FACE_POINT);
  Require(validate(face).reason_code() ==
              icarus::v1::REASON_CODE_INVALID_ARGUMENT,
          "goto face-point heading requires a point");
  *face.mutable_goto_()->mutable_heading()->mutable_face_point() =
      Local(5.2, 0.1);
  Require(validate(face).reason_code() ==
              icarus::v1::REASON_CODE_INVALID_ARGUMENT,
          "a face point on top of the destination has no heading");
  *face.mutable_goto_()->mutable_heading()->mutable_face_point() =
      state.home();
  Require(validate(face).reason_code() ==
              icarus::v1::REASON_CODE_INVALID_ARGUMENT,
          "a geographic face point cannot aim a local destination");
  *face.mutable_goto_()->mutable_heading()->mutable_face_point() =
      Local(60.0, 0.0);
  Require(validate(face).valid(),
          "a distinct face point in the destination frame is valid, even "
          "outside the fence");
  face.mutable_goto_()->mutable_heading()->set_mode(
      icarus::v1::HEADING_MODE_FACE_PATH);
  Require(validate(face).valid(),
          "face-path is the autopilot's default transit yaw and stays valid");

  auto relaxed = goto_to(5.0, 0.0);
  relaxed.mutable_goto_()->mutable_limits()->set_minimum_clearance_m(1.0);
  const auto relaxed_result = validate(relaxed);
  Require(relaxed_result.valid() && relaxed_result.warnings_size() == 1,
          "a clearance below the floor is accepted with a warning, because "
          "the floor still applies");
  relaxed.mutable_goto_()->mutable_limits()->set_minimum_clearance_m(
      std::numeric_limits<double>::infinity());
  Require(!validate(relaxed).valid(), "clearance must be finite");

  const auto orbit_around = [&](double north, double radius) {
    icarus::v1::ActionCommand command;
    auto* orbit = command.mutable_orbit();
    *orbit->mutable_context() = Context(clock, "lease-1");
    *orbit->mutable_center() = Local(north, 0.0);
    orbit->set_radius_m(radius);
    orbit->set_altitude_m(5.0);
    orbit->set_ground_speed_mps(2.0);
    orbit->set_revolutions(1.0);
    return command;
  };
  auto circle = orbit_around(25.0, 4.0);
  circle.mutable_orbit()->set_target_source(
      icarus::v1::TARGET_SOURCE_RUNTIME_ESTIMATE);
  circle.mutable_orbit()->mutable_heading()->set_mode(
      icarus::v1::HEADING_MODE_FACE_POINT);
  circle.mutable_orbit()->set_viewpoints(4);
  circle.mutable_orbit()->set_viewpoint_dwell_ms(2'000);
  Require(validate(circle).valid(),
          "a centre-facing 4-viewpoint runtime orbit inside 30 m is valid");
  circle.mutable_orbit()->set_radius_m(6.0);
  Require(validate(circle).reason_code() ==
              icarus::v1::REASON_CODE_GEOFENCE_VIOLATION,
          "the far side of a runtime orbit must stay inside 30 m");
  Require(validate(orbit_around(95.0, 6.0)).reason_code() ==
              icarus::v1::REASON_CODE_GEOFENCE_VIOLATION,
          "an orbit centre inside the fence does not admit a circle beyond it");
  auto high = orbit_around(10.0, 5.0);
  high.mutable_orbit()->set_altitude_m(31.0);
  Require(validate(high).reason_code() ==
              icarus::v1::REASON_CODE_ALTITUDE_LIMIT,
          "a local-centre orbit altitude is bounded whatever its frame");
  auto spiral = orbit_around(10.0, 5.0);
  spiral.mutable_orbit()->set_revolutions(1.5);
  spiral.mutable_orbit()->set_viewpoints(4);
  Require(validate(spiral).reason_code() ==
              icarus::v1::REASON_CODE_INVALID_ARGUMENT,
          "viewpoints are defined over at most one revolution");
  spiral.mutable_orbit()->set_revolutions(11.0);
  spiral.mutable_orbit()->set_viewpoints(0);
  Require(validate(spiral).reason_code() ==
              icarus::v1::REASON_CODE_INVALID_ARGUMENT,
          "revolutions are bounded");
  auto strict = orbit_around(10.0, 5.0);
  strict.mutable_orbit()->mutable_limits()->set_minimum_clearance_m(-1.0);
  Require(validate(strict).reason_code() ==
              icarus::v1::REASON_CODE_INVALID_ARGUMENT,
          "orbit limits are validated");
}

void TestFacePointGeometry() {
  using icarus::mission_executor::HeadingTowardDeg;
  const auto from = Local(0.0, 0.0);
  Require(std::abs(*HeadingTowardDeg(from, Local(10.0, 0.0)) - 0.0) < 1e-9 &&
              std::abs(*HeadingTowardDeg(from, Local(0.0, 10.0)) - 90.0) < 1e-9 &&
              std::abs(*HeadingTowardDeg(from, Local(-10.0, 0.0)) - 180.0) <
                  1e-9 &&
              std::abs(*HeadingTowardDeg(from, Local(0.0, -10.0)) - 270.0) <
                  1e-9,
          "face-point heading is the NED azimuth toward the point");
  Require(!HeadingTowardDeg(from, Local(0.2, 0.2)),
          "a coincident face point has no heading");
  const auto home = HealthyLandedState(0).home();
  Require(!HeadingTowardDeg(from, home), "mixed frames have no heading");
  auto east = home;
  east.mutable_geo()->set_longitude_deg(77.001);
  Require(std::abs(*HeadingTowardDeg(home, east) - 90.0) < 1e-6,
          "geographic face points resolve to the same azimuth convention");

  using icarus::mission_executor::EnforcedClearanceM;
  Require(EnforcedClearanceM(2.0, std::numeric_limits<double>::infinity()) ==
                  2.0 &&
              EnforcedClearanceM(4.0, 10.0) == 4.0 &&
              std::abs(EnforcedClearanceM(2.0, 1.5) - 1.25) < 1e-9,
          "inside the floor an action may keep, not reduce, its clearance");
}

void TestOrbitViewpointLayout() {
  using icarus::mission_executor::OrbitSegmentCount;
  using icarus::mission_executor::OrbitViewpointAt;
  using icarus::mission_executor::OrbitWaypoint;
  icarus::v1::OrbitRequest orbit;
  *orbit.mutable_center() = Local(10.0, 0.0);
  orbit.set_radius_m(5.0);
  orbit.set_altitude_m(5.0);
  orbit.set_revolutions(1.0);
  orbit.set_clockwise(true);
  orbit.set_viewpoints(4);
  Require(OrbitSegmentCount(orbit) == 24, "four sides fit 24 segments");
  for (int index = 0; index <= 24; ++index) {
    const int expected = index % 6 == 0 && index < 24 ? index / 6 + 1 : 0;
    Require(OrbitViewpointAt(orbit, index) == expected,
            "closed-orbit viewpoints are 90 degrees apart from the entry and "
            "the closing point is not held twice");
  }
  const auto east_side = OrbitWaypoint(orbit, 0.0, 6);
  Require(std::abs(east_side.local_ned().north_m() - 10.0) < 1e-9 &&
              std::abs(east_side.local_ned().east_m() - 5.0) < 1e-9,
          "clockwise from north, the second viewpoint is due east");
  Require(std::abs(*icarus::mission_executor::HeadingTowardDeg(
                       east_side, orbit.center()) -
                   270.0) < 1e-9,
          "from the east side the centre is due west");
  orbit.set_viewpoints(5);
  Require(OrbitSegmentCount(orbit) == 25 &&
              OrbitViewpointAt(orbit, 5) == 2 &&
              OrbitViewpointAt(orbit, 20) == 5 &&
              OrbitViewpointAt(orbit, 25) == 0,
          "segments are rounded up so viewpoints stay exactly 72 degrees apart");
  orbit.set_revolutions(0.5);
  orbit.set_viewpoints(3);
  Require(OrbitSegmentCount(orbit) == 12 && OrbitViewpointAt(orbit, 0) == 1 &&
              OrbitViewpointAt(orbit, 6) == 2 &&
              OrbitViewpointAt(orbit, 12) == 3,
          "a partial arc holds both of its ends");
  orbit.set_viewpoints(0);
  orbit.set_revolutions(0.3);
  const auto end = OrbitWaypoint(orbit, 0.0, OrbitSegmentCount(orbit));
  Require(std::abs(std::atan2(end.local_ned().east_m(),
                              end.local_ned().north_m() - 10.0) -
                   0.3 * 2.0 * kPi) < 1e-9,
          "a partial orbit ends exactly at its requested sweep");
}

void TestGotoFacesPoint() {
  FakeClock clock;
  const auto policy = icarus::guardrails::SafetyPolicy::Load(
      std::filesystem::path(ICARUS_SOURCE_DIR) / "config/safety/v1.yaml");
  icarus::guardrails::Guardrails guardrails(policy);
  icarus::state_engine::StateEngine states(clock);
  const auto state = AirborneState(clock, 0.0, 0.0);
  states.Publish(state);
  icarus::drone_api::AuthorityManager authority(clock);
  const auto lease = AcquireLease(authority);
  icarus::mission_executor::ActionStore actions(clock);
  TeleportGateway gateway(states, state);
  icarus::mission_executor::ActionExecutor executor(
      clock, actions, states, authority, guardrails, gateway);
  executor.Start();
  icarus::v1::ActionCommand command;
  *command.mutable_goto_()->mutable_context() = Context(clock, lease);
  *command.mutable_goto_()->mutable_destination() = Local(4.0, 4.0);
  command.mutable_goto_()->set_acceptance_radius_m(1.0);
  command.mutable_goto_()->mutable_heading()->set_mode(
      icarus::v1::HEADING_MODE_FACE_POINT);
  *command.mutable_goto_()->mutable_heading()->mutable_face_point() =
      Local(4.0, 14.0);
  const auto receipt = executor.Submit(command);
  Require(receipt.disposition() == icarus::v1::ACTION_STATE_ACCEPTED,
          "face-point goto should be accepted");
  const auto status = WaitForTerminal(actions, receipt.action_id(), 5'000);
  executor.Stop();
  Require(status.state() == icarus::v1::ACTION_STATE_SUCCEEDED,
          "face-point goto should complete: " + status.message());
  const auto headings = gateway.Headings();
  Require(headings.size() == 1 && std::abs(headings[0] - 90.0) < 1e-9,
          "the goto must yaw from its destination toward the face point");
}

void TestLandingRequiresUprightSettledTelemetry() {
  FakeClock clock;
  const auto policy = icarus::guardrails::SafetyPolicy::Load(
      std::filesystem::path(ICARUS_SOURCE_DIR) / "config/safety/v1.yaml");
  icarus::guardrails::Guardrails guardrails(policy);
  icarus::state_engine::StateEngine states(clock);
  auto state = AirborneState(clock, 0.0, 0.0);
  states.Publish(state);
  icarus::drone_api::AuthorityManager authority(clock);
  const auto lease = AcquireLease(authority);
  icarus::mission_executor::ActionStore actions(clock);
  FakeGateway gateway;
  icarus::mission_executor::ActionExecutor executor(
      clock, actions, states, authority, guardrails, gateway);
  executor.Start();
  icarus::v1::ActionCommand command;
  *command.mutable_land()->mutable_context() = Context(clock, lease);
  auto receipt = executor.Submit(command);
  state.set_landed(true);
  state.set_armed(false);
  state.mutable_attitude()->set_roll_rad(2.5);
  states.Publish(state);
  auto status = WaitForTerminal(actions, receipt.action_id(), 1500);
  Require(status.state() == icarus::v1::ACTION_STATE_FAILED &&
              status.message().find("not upright") != std::string::npos,
          "inverted disarming must never report a successful landing");
  state = AirborneState(clock, 0.0, 0.0);
  states.Publish(state);
  command.mutable_land()->mutable_context()->set_idempotency_key("upright-land");
  receipt = executor.Submit(command);
  state.set_landed(true);
  state.set_armed(false);
  state.set_ground_speed_mps(1.0);
  states.Publish(state);
  std::this_thread::sleep_for(std::chrono::milliseconds(1200));
  Require(actions.Find(receipt.action_id())->state() == icarus::v1::ACTION_STATE_EXECUTING,
          "disarmed but sliding aircraft is not a settled landing");
  state.set_ground_speed_mps(0.0);
  state.set_attitude_received_at_unix_ms(0);
  states.Publish(state);
  std::this_thread::sleep_for(std::chrono::milliseconds(1200));
  Require(actions.Find(receipt.action_id())->state() == icarus::v1::ACTION_STATE_EXECUTING,
          "landing success requires fresh attitude evidence");
  state.set_attitude_received_at_unix_ms(clock.NowUnixMs());
  states.Publish(state);
  status = WaitForTerminal(actions, receipt.action_id(), 1800);
  Require(status.state() == icarus::v1::ACTION_STATE_SUCCEEDED,
          "upright stationary disarmed aircraft completes after sustained measurement");
  executor.Stop();
}

void TestOrbitWaitsForMeasuredSettling() {
  FakeClock clock;
  const auto policy = icarus::guardrails::SafetyPolicy::Load(
      std::filesystem::path(ICARUS_SOURCE_DIR) / "config/safety/v1.yaml");
  icarus::guardrails::Guardrails guardrails(policy);
  icarus::state_engine::StateEngine states(clock);
  const auto state = AirborneState(clock, 3.0, 0.0);
  states.Publish(state);
  icarus::drone_api::AuthorityManager authority(clock);
  const auto lease = AcquireLease(authority);
  icarus::mission_executor::ActionStore actions(clock);
  TeleportGateway gateway(states, state);
  gateway.simulate_moving_arrival = true;
  icarus::mission_executor::ActionExecutor executor(
      clock, actions, states, authority, guardrails, gateway);
  executor.Start();
  icarus::v1::ActionCommand command;
  auto* orbit = command.mutable_orbit();
  *orbit->mutable_context() = Context(clock, lease);
  *orbit->mutable_center() = Local(0.0, 0.0);
  orbit->set_radius_m(3.0);
  orbit->set_altitude_m(5.0);
  orbit->set_ground_speed_mps(1.0);
  orbit->set_revolutions(0.1);
  orbit->set_viewpoints(2);
  const auto receipt = executor.Submit(command);
  std::this_thread::sleep_for(std::chrono::milliseconds(700));
  Require(gateway.Targets().size() == 2,
          "crossing an orbit waypoint at speed must not advance the target");
  gateway.SetArrivalMotion(0.0, 0.8);
  std::this_thread::sleep_for(std::chrono::milliseconds(700));
  Require(gateway.Targets().size() == 2,
          "stopping outside waypoint tolerance must not advance the target");
  gateway.SetArrivalMotion(0.0);
  std::this_thread::sleep_for(std::chrono::milliseconds(250));
  Require(gateway.Targets().size() == 2,
          "instantaneous arrival must not advance the target");
  std::this_thread::sleep_for(std::chrono::milliseconds(1000));
  Require(gateway.Targets().size() == 3,
          "sustained measured arrival must advance exactly one segment");
  Require(actions.Find(receipt.action_id())->state() ==
              icarus::v1::ACTION_STATE_EXECUTING,
          "an unsettled later segment cannot report orbit success");
  executor.Stop();
}

void TestContinuousOrbitStreamsAndMeasuresCompletion() {
  for (const bool clockwise : {true, false}) {
    FakeClock clock;
    const auto policy = icarus::guardrails::SafetyPolicy::Load(
        std::filesystem::path(ICARUS_SOURCE_DIR) / "config/safety/v1.yaml");
    icarus::guardrails::Guardrails guardrails(policy);
    icarus::state_engine::StateEngine states(clock);
    const auto state = AirborneState(clock, 3.0, 0.0);
    states.Publish(state);
    icarus::drone_api::AuthorityManager authority(clock);
    const auto lease = AcquireLease(authority);
    icarus::mission_executor::ActionStore actions(clock);
    TeleportGateway gateway(states, state);
    icarus::mission_executor::ActionExecutor executor(
        clock, actions, states, authority, guardrails, gateway);
    executor.Start();
    icarus::v1::ActionCommand command;
    auto* orbit = command.mutable_orbit();
    *orbit->mutable_context() = Context(clock, lease);
    *orbit->mutable_center() = Local(0.0, 0.0);
    orbit->set_radius_m(3.0);
    orbit->set_altitude_m(5.0);
    orbit->set_ground_speed_mps(1.0);
    orbit->set_revolutions(0.05);
    orbit->set_clockwise(clockwise);
    orbit->mutable_heading()->set_mode(icarus::v1::HEADING_MODE_FACE_POINT);
    const auto receipt = executor.Submit(command);
    const auto result = WaitForTerminal(actions, receipt.action_id(), 12'000);
    executor.Stop();
    Require(result.state() == icarus::v1::ACTION_STATE_SUCCEEDED,
            "continuous orbit succeeds from measured motion: " + result.message());
    Require(gateway.stream_commands > 20 && gateway.moving_stream_commands > 10,
            "continuous orbit streams velocity feed-forward between endpoints");
    Require(gateway.speed_commands == 1,
            "continuous orbit initializes navigation once, not at every vertex");
    const auto targets = gateway.Targets();
    const auto headings = gateway.PositionHeadings();
    Require((targets.back().east_m() > 0.0) == clockwise,
            "streamed orbit uses requested direction");
    for (std::size_t index = 3; index + 1 < targets.size(); ++index) {
      icarus::v1::Position measured;
      *measured.mutable_local_ned() = targets[index - 1];
      const auto expected = icarus::mission_executor::HeadingTowardDeg(measured, orbit->center());
      Require(headings[index] && expected &&
              std::abs(std::remainder(*headings[index] - *expected, 360.0)) < 1e-6,
              "streamed yaw faces centre from measured pose, not future setpoint");
      Require(std::abs(std::hypot(targets[index].north_m(), targets[index].east_m()) - 3.0) < 1e-9,
              "streamed positions lie on circle");
    }
    Require(result.message().find("measured sweep") != std::string::npos,
            "orbit result includes measured sweep evidence");
  }
}

void TestContinuousOrbitCannotSucceedWithoutMotion() {
  for (const bool reject : {false, true}) {
    FakeClock clock;
    const auto policy = icarus::guardrails::SafetyPolicy::Load(
        std::filesystem::path(ICARUS_SOURCE_DIR) / "config/safety/v1.yaml");
    icarus::guardrails::Guardrails guardrails(policy);
    icarus::state_engine::StateEngine states(clock);
    const auto state = AirborneState(clock, 3.0, 0.0);
    states.Publish(state);
    icarus::drone_api::AuthorityManager authority(clock);
    const auto lease = AcquireLease(authority);
    icarus::mission_executor::ActionStore actions(clock);
    TeleportGateway gateway(states, state);
    gateway.reject_stream = reject;
    gateway.freeze_stream = !reject;
    icarus::mission_executor::ActionExecutor executor(
        clock, actions, states, authority, guardrails, gateway);
    executor.Start();
    icarus::v1::ActionCommand command;
    auto* orbit = command.mutable_orbit();
    *orbit->mutable_context() = Context(clock, lease);
    *orbit->mutable_center() = Local(0.0, 0.0);
    orbit->set_radius_m(3.0);
    orbit->set_altitude_m(5.0);
    orbit->set_ground_speed_mps(1.0);
    orbit->set_revolutions(0.05);
    orbit->mutable_limits()->set_execution_timeout_ms(1000);
    const auto receipt = executor.Submit(command);
    const auto result = WaitForTerminal(actions, receipt.action_id(), 4'000);
    executor.Stop();
    Require(result.state() == (reject ? icarus::v1::ACTION_STATE_FAILED
                                     : icarus::v1::ACTION_STATE_TIMED_OUT),
            "unaccepted or unflown stream cannot become orbit success");
    Require(gateway.last_mode == 17, "failed continuous orbit commands BRAKE");
  }
}

void TestOrbitFacesCentreAndHoldsViewpoints() {
  FakeClock clock;
  const auto policy = icarus::guardrails::SafetyPolicy::Load(
      std::filesystem::path(ICARUS_SOURCE_DIR) / "config/safety/v1.yaml");
  icarus::guardrails::Guardrails guardrails(policy);
  icarus::state_engine::StateEngine states(clock);
  const auto state = AirborneState(clock, 15.0, 0.0);
  states.Publish(state);
  icarus::drone_api::AuthorityManager authority(clock);
  const auto lease = AcquireLease(authority);
  icarus::mission_executor::ActionStore actions(clock);
  TeleportGateway gateway(states, state);
  icarus::mission_executor::ActionExecutor executor(
      clock, actions, states, authority, guardrails, gateway);
  executor.Start();
  icarus::v1::ActionCommand command;
  auto* orbit = command.mutable_orbit();
  *orbit->mutable_context() = Context(clock, lease);
  *orbit->mutable_center() = Local(10.0, 0.0);
  orbit->set_radius_m(5.0);
  orbit->set_altitude_m(5.0);
  orbit->set_ground_speed_mps(0.75);
  orbit->set_clockwise(true);
  orbit->set_revolutions(1.0);
  orbit->mutable_heading()->set_mode(icarus::v1::HEADING_MODE_FACE_POINT);
  orbit->set_viewpoints(4);
  orbit->set_viewpoint_dwell_ms(200);
  const auto receipt = executor.Submit(command);
  Require(receipt.disposition() == icarus::v1::ACTION_STATE_ACCEPTED,
          "centre-facing orbit should be accepted");
  std::set<std::uint32_t> held;
  const auto status =
      WaitForTerminal(actions, receipt.action_id(), 25'000, &held);
  executor.Stop();
  Require(status.state() == icarus::v1::ACTION_STATE_SUCCEEDED &&
              status.message().find("held 4 viewpoints") != std::string::npos,
          "orbit should complete after holding its viewpoints: " +
              status.message());
  Require(held == std::set<std::uint32_t>{1, 2, 3, 4},
          "every viewpoint must be reported while it is held");
  auto targets = gateway.Targets();
  targets.erase(targets.begin());  // current-position initialization hold
  const auto headings = gateway.Headings();
  Require(std::abs(gateway.last_ground_speed - 0.75) < 1e-9 &&
              gateway.speed_commands == static_cast<int>(targets.size()),
          "every orbit leg must apply its requested navigation speed");
  Require(targets.size() == 25 && headings.size() == targets.size(),
          "one heading must follow every orbit position target");
  for (std::size_t index = 0; index < targets.size(); ++index) {
    icarus::v1::Position waypoint;
    *waypoint.mutable_local_ned() = targets[index];
    Require(std::abs(*icarus::mission_executor::HeadingTowardDeg(
                         waypoint, orbit->center()) -
                     headings[index]) < 1e-9,
            "each orbit waypoint must face the centre");
  }
  Require(std::abs(targets[6].east_m() - 5.0) < 1e-9 &&
              std::abs(headings[6] - 270.0) < 1e-9,
          "clockwise from north the aircraft reaches the east side looking "
          "west");
}

enum class Motion { kGoto, kYaw };

// Starts a motion action at `start_north` with one obstacle return at 6 m
// north, then moves the aircraft to `moved_north` and reports the outcome.
icarus::v1::ActionStatus RunClearanceScenario(Motion motion,
                                              double requested_clearance_m,
                                              double start_north,
                                              double moved_north,
                                              bool stale_perception,
                                              std::uint32_t* last_mode,
                                              double velocity_north_mps = 0.0) {
  FakeClock clock;
  const auto policy = icarus::guardrails::SafetyPolicy::Load(
      std::filesystem::path(ICARUS_SOURCE_DIR) / "config/safety/v1.yaml");
  icarus::guardrails::Guardrails guardrails(policy);
  icarus::state_engine::StateEngine states(clock);
  auto state = AirborneState(clock, start_north, 0.0);
  states.Publish(state);
  icarus::perception::PerceptionEngine perception(
      clock, states, policy.maximum_perception_age_ms);
  icarus::perception::RangeScan scan;
  scan.sensor_id = "test-lidar";
  scan.captured_at_unix_ms = clock.NowUnixMs();
  scan.frame = icarus::perception::SensorFrame::kLocalNed;
  scan.points.push_back({6.0, 0.0, -5.0, 1.0});
  perception.Ingest(scan);
  icarus::drone_api::AuthorityManager authority(clock);
  const auto lease = AcquireLease(authority);
  icarus::mission_executor::ActionStore actions(clock);
  FakeGateway gateway;
  icarus::mission_executor::ActionExecutor executor(
      clock, actions, states, authority, guardrails, gateway, &perception);
  executor.Start();
  icarus::v1::ActionCommand command;
  if (motion == Motion::kGoto) {
    *command.mutable_goto_()->mutable_context() = Context(clock, lease);
    *command.mutable_goto_()->mutable_destination() = Local(moved_north, 0.0);
    command.mutable_goto_()->set_acceptance_radius_m(0.5);
    command.mutable_goto_()->mutable_limits()->set_execution_timeout_ms(5'000);
    if (requested_clearance_m > 0.0) {
      command.mutable_goto_()->mutable_limits()->set_minimum_clearance_m(
          requested_clearance_m);
    }
  } else {
    *command.mutable_yaw()->mutable_context() = Context(clock, lease);
    command.mutable_yaw()->set_relative_angle_deg(90.0);
    command.mutable_yaw()->set_rate_deg_s(30.0);
  }
  const auto receipt = executor.Submit(command);
  Require(receipt.disposition() == icarus::v1::ACTION_STATE_ACCEPTED,
          "clearance scenario action should be accepted");
  for (int attempt = 0; attempt < 100; ++attempt) {
    const auto status = actions.Find(receipt.action_id());
    if (status && status->state() == icarus::v1::ACTION_STATE_EXECUTING) break;
    std::this_thread::sleep_for(std::chrono::milliseconds(2));
  }
  // Let the guard observe the starting clearance before anything moves.
  std::this_thread::sleep_for(std::chrono::milliseconds(120));
  if (stale_perception) {
    // Keep vehicle state fresh while only the LiDAR map ages out.
    state.set_observed_at_unix_ms(clock.NowUnixMs() + 800);
    states.Publish(state);
    clock.now_ms += 800;
  } else {
    state.mutable_local_position_ned()->set_north_m(moved_north);
    state.mutable_velocity_ned_mps()->set_x(velocity_north_mps);
    states.Publish(state);
  }
  const auto status = WaitForTerminal(actions, receipt.action_id(), 5'000);
  executor.Stop();
  *last_mode = gateway.last_mode;
  return status;
}

void TestDetourCornerWaitsForRest() {
  FakeClock clock;
  const auto policy = icarus::guardrails::SafetyPolicy::Load(
      std::filesystem::path(ICARUS_SOURCE_DIR) / "config/safety/v1.yaml");
  icarus::guardrails::Guardrails guardrails(policy);
  icarus::state_engine::StateEngine states(clock);
  auto state = AirborneState(clock, 0.0, 0.0);
  states.Publish(state);
  icarus::perception::PerceptionEngine perception(clock, states, policy.maximum_perception_age_ms);
  icarus::perception::RangeScan scan;
  scan.sensor_id = "detour-test";
  scan.captured_at_unix_ms = clock.NowUnixMs();
  scan.frame = icarus::perception::SensorFrame::kLocalNed;
  scan.points.push_back({6.0, 0.0, -5.0, 1.0});
  perception.Ingest(scan);
  icarus::local_planner::LocalPlanner planner(0.5, 2.5, 8.0);
  icarus::drone_api::AuthorityManager authority(clock);
  const auto lease = AcquireLease(authority);
  icarus::mission_executor::ActionStore actions(clock);
  TeleportGateway gateway(states, state);
  gateway.simulate_moving_arrival = true;
  icarus::mission_executor::ActionExecutor executor(clock, actions, states, authority, guardrails, gateway, &perception, &planner);
  executor.Start();
  icarus::v1::ActionCommand command;
  auto* go = command.mutable_goto_();
  *go->mutable_context() = Context(clock, lease);
  *go->mutable_destination() = Local(12.0, 0.0);
  go->set_acceptance_radius_m(1.0);
  go->mutable_limits()->set_execution_timeout_ms(5000);
  const auto receipt = executor.Submit(command);
  Require(receipt.disposition() == icarus::v1::ACTION_STATE_ACCEPTED, "detour accepted");
  for (int i=0; i<100 && gateway.Targets().size()<2; ++i)
    std::this_thread::sleep_for(std::chrono::milliseconds(5));
  std::this_thread::sleep_for(std::chrono::milliseconds(150));
  const auto while_moving = gateway.Targets().size();
  Require(while_moving == 2, "moving arrival at first detour corner must not advance to next target");
  gateway.SetArrivalMotion(0.0);
  for (int i=0; i<100 && gateway.Targets().size()==while_moving; ++i)
    std::this_thread::sleep_for(std::chrono::milliseconds(5));
  Require(gateway.Targets().size()>while_moving, "settled corner must release the next segment");
  executor.Stop();
}

void TestPlannerRetreatUsesExecutorClearanceContract() {
  icarus::local_planner::LocalPlanner planner(0.5, 2.5, 8.0);
  const auto position = [](double north, double east) {
    icarus::v1::LocalPositionNed result;
    result.set_north_m(north); result.set_east_m(east);
    result.set_down_m(-5); result.set_origin_id("measured-map");
    return result;
  };
  const auto start = position(-0.6, 0);
  const auto goal = position(5, 8);
  std::vector<icarus::perception::Point3> wall;
  for (double east = -3; east <= 3; east += 0.25) {
    wall.push_back({0, east, -5, 1});
  }
  const double floor = icarus::mission_executor::EnforcedClearanceM(2.0, 0.6);
  Require(!planner.Plan(start, goal, wall, 2.5, 2.0).success,
          "fixed admission floor cannot represent an existing clearance violation");
  const auto result = planner.Plan(start, goal, wall, 2.5, floor);
  Require(result.success && !result.waypoints.empty(),
          "planner must support the executor's monotonic-retreat contract");
  icarus::local_planner::LocalPlanner departure_probe(0.5, floor, 8.0);
  auto previous = start;
  for (std::size_t index = 0; index < result.waypoints.size(); ++index) {
    const auto& next = result.waypoints[index];
    const auto& probe = index == 0 ? departure_probe : planner;
    Require(probe.SegmentClear(previous, next, wall, index == 0 ? floor : 2.5),
            "departure respects existing live floor; later legs regain full planning margin");
    if (index == 0) {
      for (const auto& point : wall) {
        const double initial = std::hypot(start.north_m()-point.x_m, start.east_m()-point.y_m);
        if (initial >= 2.5) continue;
        for (int sample = 1; sample <= 100; ++sample) {
          const double ratio = sample / 100.0;
          const double north = start.north_m() + ratio * (next.north_m()-start.north_m());
          const double east = start.east_m() + ratio * (next.east_m()-start.east_m());
          Require(std::hypot(north-point.x_m,east-point.y_m) >= initial-1e-9,
                  "initial departure never approaches any nearby obstacle return");
        }
      }
    }
    previous = next;
  }
  Require(!planner.Plan(start, position(-0.1, 0), wall, 2.5, floor).success,
          "retreat permission cannot authorize an endpoint inside the obstacle envelope");
}

void TestHardMinimumClearance() {
  using icarus::v1::ACTION_STATE_ABORTED_BY_SAFETY;
  using icarus::v1::ACTION_STATE_SUCCEEDED;
  using icarus::v1::REASON_CODE_CLEARANCE_VIOLATION;
  std::uint32_t mode = 0;
  auto status = RunClearanceScenario(Motion::kGoto, 0.0, 0.0, 4.2, false, &mode);
  Require(status.state() == ACTION_STATE_ABORTED_BY_SAFETY &&
              status.reason_code() == REASON_CODE_CLEARANCE_VIOLATION &&
              status.message().find("2.00 m floor") != std::string::npos &&
              mode == 17,
          "closing inside the 2 m hard floor must BRAKE and abort: " +
              status.message());
  status = RunClearanceScenario(Motion::kGoto, 1.0, 0.0, 4.5, false, &mode);
  Require(status.state() == ACTION_STATE_ABORTED_BY_SAFETY &&
              status.reason_code() == REASON_CODE_CLEARANCE_VIOLATION,
          "a request below the floor must not relax it");
  status = RunClearanceScenario(Motion::kGoto, 4.0, 0.0, 2.5, false, &mode);
  Require(status.state() == ACTION_STATE_ABORTED_BY_SAFETY &&
              status.message().find("4.00 m floor") != std::string::npos,
          "a stricter per-action clearance must be enforced: " +
              status.message());
  status = RunClearanceScenario(Motion::kGoto, 0.0, 0.0, 3.5, false, &mode);
  Require(status.state() == ACTION_STATE_SUCCEEDED && mode == 4,
          "2.5 m of clearance satisfies the 2 m floor: " + status.message());
  status = RunClearanceScenario(Motion::kGoto, 0.0, 0.0, 2.5, false, &mode, 2.0);
  Require(status.state() == ACTION_STATE_ABORTED_BY_SAFETY && mode == 17 &&
              status.message().find("stopping path") != std::string::npos,
          "closing at speed must brake while current clearance is still 3.5 m: " + status.message());
  status = RunClearanceScenario(Motion::kGoto, 0.0, 4.5, 1.0, false, &mode);
  Require(status.state() == ACTION_STATE_SUCCEEDED,
          "an aircraft already inside the floor may retreat: " +
              status.message());
  status = RunClearanceScenario(Motion::kGoto, 0.0, 4.5, 5.0, false, &mode);
  Require(status.state() == ACTION_STATE_ABORTED_BY_SAFETY &&
              status.reason_code() == REASON_CODE_CLEARANCE_VIOLATION,
          "an aircraft already inside the floor must not close in further");
  status = RunClearanceScenario(Motion::kYaw, 0.0, 0.0, 4.2, false, &mode);
  Require(status.state() == ACTION_STATE_ABORTED_BY_SAFETY &&
              status.reason_code() == REASON_CODE_CLEARANCE_VIOLATION &&
              mode == 17,
          "drifting inside the floor during a yaw must BRAKE and abort");
  status = RunClearanceScenario(Motion::kGoto, 0.0, 0.0, 0.0, true, &mode);
  Require(status.state() == ACTION_STATE_ABORTED_BY_SAFETY &&
              status.reason_code() ==
                  icarus::v1::REASON_CODE_SAFETY_INTERVENTION &&
              status.message().find("perception stale") != std::string::npos,
          "motion with unverifiable clearance must BRAKE and abort: " +
              status.message());
}

}  // namespace

void TestOrbitDirection() {
  icarus::v1::OrbitRequest orbit;
  auto* center = orbit.mutable_center()->mutable_local_ned();
  center->set_origin_id("ekf-origin");
  center->set_north_m(10.0);
  center->set_east_m(0.0);
  orbit.set_radius_m(5.0);
  orbit.set_altitude_m(5.0);
  orbit.set_revolutions(1.0);
  // Enter due north of the centre; a clockwise (seen from above) orbit then
  // moves east first, a counter-clockwise one moves west first.
  orbit.set_clockwise(true);
  const auto start = icarus::mission_executor::OrbitWaypoint(orbit, 0.0, 0);
  const auto clockwise =
      icarus::mission_executor::OrbitWaypoint(orbit, 0.0, 1);
  Require(std::abs(start.local_ned().north_m() - 15.0) < 1e-9 &&
              std::abs(start.local_ned().east_m()) < 1e-9,
          "orbit must enter at the start angle");
  Require(clockwise.local_ned().east_m() > 0.5,
          "clockwise orbit must move east from north of the centre");
  Require(clockwise.local_ned().has_altitude_above_home_m() &&
              clockwise.local_ned().altitude_above_home_m() == 5.0 &&
              clockwise.local_ned().origin_id() == "ekf-origin",
          "orbit waypoint must keep altitude and origin");
  orbit.set_clockwise(false);
  const auto counter =
      icarus::mission_executor::OrbitWaypoint(orbit, 0.0, 1);
  Require(counter.local_ned().east_m() < -0.5,
          "counter-clockwise orbit must move west from north of the centre");
  Require(std::abs(counter.local_ned().north_m() -
                   clockwise.local_ned().north_m()) < 1e-9,
          "both directions must stay on the same circle");
  const auto full_turn = icarus::mission_executor::OrbitWaypoint(
      orbit, 0.0, icarus::mission_executor::kOrbitSegmentsPerRevolution);
  Require(std::abs(full_turn.local_ned().north_m() - 15.0) < 1e-9 &&
              std::abs(full_turn.local_ned().east_m()) < 1e-9,
          "one revolution must return to the entry point");
}

void TestMeasuredBatteryAvailability() {
  using namespace icarus;
  FakeClock clock;
  const auto now = clock.NowUnixMs();
  const auto good = mavlink_gateway::DecodeBatteryTelemetry(24000, 0, 90, now);
  Require(state_engine::BatteryAvailable(good, now), "measured zero current is valid");
  auto charging = mavlink_gateway::DecodeBatteryTelemetry(24000, -200, 90, now);
  Require(state_engine::BatteryAvailable(charging, now) && charging.current_a() == -2,
          "negative charging current is valid");
  const auto empty = mavlink_gateway::DecodeBatteryTelemetry(24000, 200, 0, now);
  Require(state_engine::BatteryAvailable(empty, now) && empty.remaining_percent() == 0,
          "known empty battery is distinct from unavailable");
  for (const auto& bad : {
      mavlink_gateway::DecodeBatteryTelemetry(0, 200, 100, now, 100),
      mavlink_gateway::DecodeBatteryTelemetry(UINT16_MAX, 200, 100, now, 100),
      mavlink_gateway::DecodeBatteryTelemetry(24000, -1, 100, now, 100),
      mavlink_gateway::DecodeBatteryTelemetry(24000, 200, -1, now, 100),
      mavlink_gateway::DecodeBatteryTelemetry(24000, 200, 127, now, 100)}) {
    Require(!state_engine::BatteryAvailable(bad, now) && !bad.has_remaining_percent(),
            "unknown raw telemetry must not become a healthy overridden percentage");
  }
  auto missing = good;
  missing.clear_current_a();
  Require(!state_engine::BatteryAvailable(missing, now), "absent current is unknown");
  missing = good;
  missing.set_current_a(std::numeric_limits<double>::quiet_NaN());
  Require(!state_engine::BatteryAvailable(missing, now), "nonfinite current is invalid");
  Require(!state_engine::BatteryAvailable(good, now + 5001), "pose traffic cannot refresh battery");
  Require(!state_engine::BatteryAvailable(good, now - 1), "future battery timestamp invalid");
  auto state = HealthyLandedState(now);
  state.set_sequence(1);
  state_engine::StateEngine engine(clock);
  engine.Publish(state);
  Require(engine.Health(1000).ready_to_arm(), "healthy measured battery permits readiness");
  state.mutable_battery()->clear_voltage_v();
  engine.Publish(state);
  Require(!engine.Health(1000).ready_to_arm() &&
              engine.Snapshot()->battery().level() == v1::HEALTH_LEVEL_UNAVAILABLE,
          "missing voltage invalidates readiness and published health");
  const auto policy = guardrails::SafetyPolicy::Load(
      std::filesystem::path(ICARUS_SOURCE_DIR) / "config/safety/v1.yaml");
  guardrails::Guardrails guards(policy);
  v1::ActionCommand arm;
  *arm.mutable_arm()->mutable_context() = Context(clock, "lease-1");
  const auto invalid_power = guards.Validate(arm, state, true, now);
  Require(!invalid_power.valid() && invalid_power.message().find("battery") != std::string::npos,
          "missing battery blocks arm for measured power reason");
  state.set_armed(true);
  state.set_landed(false);
  state.set_flight_phase(v1::FLIGHT_PHASE_AIRBORNE);
  v1::ActionCommand land;
  *land.mutable_land()->mutable_context() = Context(clock, "lease-1");
  Require(guards.Validate(land, state, true, now).valid(), "unknown battery must not block landing");
}

void TestSupervisorMeasuredHomeAndBattery() {
  using namespace icarus;
  for (int scenario = 0; scenario < 4; ++scenario) {
    FakeClock clock;
    auto policy = guardrails::SafetyPolicy::Load(
        std::filesystem::path(ICARUS_SOURCE_DIR) / "config/safety/v1.yaml");
    auto state = HealthyLandedState(clock.NowUnixMs());
    state.set_armed(true);
    state.set_landed(false);
    // Translate the local origin by kilometres while staying physically home.
    state.mutable_local_position_ned()->set_north_m(4000);
    state.mutable_local_position_ned()->set_east_m(-3000);
    if (scenario == 1) {
      // A real geographic offset must trigger RTL independent of translation.
      state.mutable_position()->mutable_geo()->set_latitude_deg(28.01);
    } else if (scenario == 2) {
      state.clear_relative_altitude_received_at_unix_ms();
    } else if (scenario == 3) {
      state.mutable_battery()->clear_current_a();
    }
    state_engine::StateEngine states(clock);
    states.Publish(state);
    drone_api::AuthorityManager authority(clock);
    v1::AcquireControlRequest request;
    request.set_session_id("session-1");
    request.set_client_id("session-1");
    request.set_vehicle_id("icarus-01");
    request.set_requested_role(v1::CONTROL_ROLE_DCM_AUTONOMY);
    request.set_requested_lease_ms(5000);
    Require(authority.Acquire(request).granted(), "supervisor test lease");
    mission_executor::ActionStore actions(clock);
    FakeGateway gateway;
    safety_supervisor::SafetySupervisor supervisor(clock, policy, states, actions, authority, gateway);
    supervisor.Start();
    std::this_thread::sleep_for(std::chrono::milliseconds(250));
    supervisor.Stop();
    if (scenario == 0) Require(gateway.mode_commands == 0, "translated home inside fence must not RTL");
    if (scenario == 1) Require(gateway.last_mode == 6, "actual home distance outside fence must RTL");
    if (scenario >= 2) Require(gateway.last_mode == 17, "unknown home or power must hold, never assume zero/full");
  }
}

void TestPassiveGroundedHold() {
  FakeClock clock;
  const auto policy = icarus::guardrails::SafetyPolicy::Load(
      std::filesystem::path(ICARUS_SOURCE_DIR) / "config/safety/v1.yaml");
  icarus::guardrails::Guardrails guardrails(policy);
  icarus::state_engine::StateEngine states(clock);
  auto state = HealthyLandedState(clock.NowUnixMs());
  state.set_sequence(1);
  states.Publish(state);
  icarus::drone_api::AuthorityManager authority(clock);
  const auto lease = AcquireLease(authority);
  icarus::mission_executor::ActionStore actions(clock);
  FakeGateway gateway;
  icarus::mission_executor::ActionExecutor executor(clock, actions, states, authority, guardrails, gateway);
  executor.Start();
  icarus::v1::ActionCommand command;
  *command.mutable_hold()->mutable_context() = Context(clock, lease);
  command.mutable_hold()->set_duration_ms(150);
  Require(guardrails.Validate(command, state, true, clock.NowUnixMs()).valid(),
          "known disarmed landed aircraft admits bounded passive hold");
  auto unknown = state; unknown.set_landed(false);
  Require(!guardrails.Validate(command, unknown, true, clock.NowUnixMs()).valid(),
          "disarmed airborne or unknown landing state cannot pass hold validation");
  unknown = state; unknown.set_flight_phase(icarus::v1::FLIGHT_PHASE_UNSPECIFIED);
  Require(!guardrails.Validate(command, unknown, true, clock.NowUnixMs()).valid(),
          "unknown flight phase cannot be treated as safely grounded");
  auto receipt = executor.Submit(command);
  const auto start = std::chrono::steady_clock::now();
  auto status = WaitForTerminal(actions, receipt.action_id(), 1000);
  Require(status.state() == icarus::v1::ACTION_STATE_SUCCEEDED &&
          std::chrono::steady_clock::now() - start >= std::chrono::milliseconds(140),
          "passive hold waits its actual duration");
  Require(gateway.arm_commands == 0 && gateway.takeoff_commands == 0 &&
          gateway.mode_commands == 0 && gateway.speed_commands == 0 &&
          gateway.last_heading == -1.0 && gateway.last_relative_yaw == 0.0,
          "passive hold emits no mode, speed or yaw commands");
  command.mutable_hold()->mutable_context()->set_idempotency_key("ground-cancel");
  command.mutable_hold()->set_duration_ms(2000);
  receipt = executor.Submit(command);
  std::this_thread::sleep_for(std::chrono::milliseconds(60));
  icarus::v1::CancelActionRequest cancel;
  *cancel.mutable_context() = Context(clock, lease);
  cancel.set_action_id(receipt.action_id());
  Require(executor.Cancel(cancel).disposition() == icarus::v1::ACTION_STATE_CANCELLED,
          "grounded wait cancellation is responsive");
  Require(gateway.mode_commands == 0, "grounded cancellation emits no BRAKE command");
  command.mutable_hold()->mutable_context()->set_idempotency_key("ground-state-change");
  receipt = executor.Submit(command);
  std::this_thread::sleep_for(std::chrono::milliseconds(60));
  state.set_landed(false); states.Publish(state);
  status = WaitForTerminal(actions, receipt.action_id(), 1000);
  Require(status.state() == icarus::v1::ACTION_STATE_ABORTED_BY_SAFETY,
          "state change during passive wait cannot report success");
  Require(gateway.mode_commands == 0, "passive wait abort never commands a mode");
  executor.Stop();
}

void TestLandedYawFailsBeforeModeOrMovement() {
  FakeClock clock;
  const auto policy = icarus::guardrails::SafetyPolicy::Load(
      std::filesystem::path(ICARUS_SOURCE_DIR) / "config/safety/v1.yaml");
  icarus::guardrails::Guardrails guardrails(policy);
  icarus::state_engine::StateEngine states(clock);
  auto state = HealthyLandedState(clock.NowUnixMs());
  state.set_armed(true);
  state.set_landed(true);
  state.set_flight_phase(icarus::v1::FLIGHT_PHASE_ARMED_GROUND);
  states.Publish(state);
  icarus::drone_api::AuthorityManager authority(clock);
  const auto lease = AcquireLease(authority);
  icarus::mission_executor::ActionStore actions(clock);
  FakeGateway gateway;
  icarus::mission_executor::ActionExecutor executor(
      clock, actions, states, authority, guardrails, gateway);
  executor.Start();
  icarus::v1::ActionCommand command;
  *command.mutable_yaw()->mutable_context() = Context(clock, lease);
  command.mutable_yaw()->set_relative_angle_deg(30);
  command.mutable_yaw()->set_rate_deg_s(5);
  const auto receipt = executor.Submit(command);
  Require(receipt.disposition() == icarus::v1::ACTION_STATE_ACCEPTED,
          "landed yaw reaches its local execution precondition");
  const auto result = WaitForTerminal(actions, receipt.action_id(), 1000);
  Require(result.state() == icarus::v1::ACTION_STATE_FAILED,
          "landed yaw fails immediately instead of waiting for physical movement");
  Require(result.reason_code() == icarus::v1::REASON_CODE_STATE_CHANGED,
          "landed yaw has a machine-readable state precondition reason");
  Require(result.message().find("yaw requires an airborne aircraft") != std::string::npos,
          "landed yaw explains the actionable physical limitation");
  Require(gateway.mode_commands == 0 && gateway.last_relative_yaw == 0 &&
          gateway.last_heading == -1 && gateway.arm_commands == 0 &&
          gateway.takeoff_commands == 0,
          "landed yaw emits no mode, yaw, arm or implicit takeoff command");
  executor.Stop();
}

int main(int argc, char** argv) {
  try {
    if (argc == 2 && std::string(argv[1]) == "--capability-only") {
      TestCapabilityPolicyParity();
      std::cout << "Capability policy parity tests passed\n";
      return 0;
    }
    if (argc == 2 && std::string(argv[1]) == "--clearance-planner-only") {
      TestPlannerRetreatUsesExecutorClearanceContract();
      TestHardMinimumClearance();
      std::cout << "Clearance planner recovery tests passed\n";
      return 0;
    }
    if (argc == 2 && std::string(argv[1]) == "--yaw-only") {
      TestLandedYawFailsBeforeModeOrMovement();
      TestRelativeYawLifecycle();
      std::cout << "Focused yaw tests passed\n";
      return 0;
    }
    TestLandedYawFailsBeforeModeOrMovement();
    TestPassiveGroundedHold();
    TestMeasuredBatteryAvailability();
    TestSupervisorMeasuredHomeAndBattery();
    TestLandingRequiresUprightSettledTelemetry();
    TestCapabilityPolicyParity();
    TestGeographicAltitudeNeedsKnownFrame();
    TestHomeGeofenceIgnoresEstimatorOriginTranslation();
    TestMeasuredAltitudeFrames();
    TestAuthorityPriority();
    TestGuardrails();
    TestStateAndActionLifecycle();
    TestStaleStateRecovery();
    TestRelativeYawLifecycle();
    TestGotoWaitsForStableArrival();
    TestOrbitDirection();
    TestTargetRelativeGuardrails();
    TestGroundSpeedAppliedResetAndRejected();
    TestDetourCornerWaitsForRest();
    TestKeepCurrentHeadingSurvivesNavigationTargets();
    TestFacePointGeometry();
    TestOrbitViewpointLayout();
    TestGotoFacesPoint();
    TestOrbitFacesCentreAndHoldsViewpoints();
    TestOrbitWaitsForMeasuredSettling();
    TestContinuousOrbitStreamsAndMeasuresCompletion();
    TestContinuousOrbitCannotSucceedWithoutMotion();
    TestHardMinimumClearance();
    TestPlannerRetreatUsesExecutorClearanceContract();
    TestPerceptionLossRecovery();
    TestVerticalLandingSurvivesLateralPerceptionLoss();
    TestControllerHeartbeatLossRecovery();
    std::cout << "Phase 8 core tests passed\n";
    return 0;
  } catch (const std::exception& error) {
    std::cerr << "Phase 8 core test failure: " << error.what() << '\n';
    return 1;
  }
}

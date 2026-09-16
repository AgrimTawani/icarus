#include <cstdint>
#include <filesystem>
#include <iostream>
#include <stdexcept>
#include <string>
#include <thread>

#include "autonomy_core/clock.hpp"
#include "drone_api/authority_manager.hpp"
#include "guardrails/guardrails.hpp"
#include "mission_executor/action_store.hpp"
#include "mission_executor/action_executor.hpp"
#include "perception/perception_engine.hpp"
#include "safety_supervisor/safety_supervisor.hpp"
#include "state_engine/state_engine.hpp"

namespace {

class FakeClock final : public icarus::core::Clock {
 public:
  [[nodiscard]] std::int64_t NowUnixMs() const override { return now_ms; }
  std::int64_t now_ms = 1'000'000;
};

void Require(bool condition, const std::string& message) {
  if (!condition) throw std::runtime_error(message);
}

class FakeGateway final : public icarus::mavlink_gateway::MavlinkGateway {
 public:
  bool Connect(const std::string&, std::uint16_t) override { return true; }
  void Disconnect() override {}
  [[nodiscard]] bool IsConnected() const override { return true; }
  [[nodiscard]] bool WaitForReady(std::uint32_t) override { return true; }
  void SetStateCallback(StateCallback) override {}
  void SetEventCallback(EventCallback) override {}
  [[nodiscard]] icarus::mavlink_gateway::CommandResult Arm(bool) override {
    return {true, false, 0, "accepted"};
  }
  [[nodiscard]] icarus::mavlink_gateway::CommandResult Takeoff(double) override {
    return {true, false, 0, "accepted"};
  }
  [[nodiscard]] icarus::mavlink_gateway::CommandResult SetMode(
      std::uint32_t mode) override {
    ++mode_commands;
    last_mode = mode;
    return {true, false, 0, "accepted"};
  }
  bool GotoGlobal(const icarus::v1::GeoPosition&) override { return true; }
  bool GotoLocal(const icarus::v1::LocalPositionNed&) override { return true; }
  std::atomic<int> mode_commands{0};
  std::atomic<std::uint32_t> last_mode{0};
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
  local->set_origin_id("home");
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
  FakeGateway gateway;
  icarus::perception::PerceptionEngine perception(
      clock, states, policy.maximum_perception_age_ms);
  icarus::safety_supervisor::SafetySupervisor supervisor(
      clock, policy, states, actions, gateway, &perception);
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

}  // namespace

int main() {
  try {
    TestAuthorityPriority();
    TestGuardrails();
    TestStateAndActionLifecycle();
    TestStaleStateRecovery();
    TestPerceptionLossRecovery();
    std::cout << "Phase 8 core tests passed\n";
    return 0;
  } catch (const std::exception& error) {
    std::cerr << "Phase 8 core test failure: " << error.what() << '\n';
    return 1;
  }
}

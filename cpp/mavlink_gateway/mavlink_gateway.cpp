#include "mavlink_gateway/mavlink_gateway.hpp"

#include <arpa/inet.h>
#include <netdb.h>
#include <sys/socket.h>
#include <unistd.h>

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstring>
#include <deque>
#include <limits>
#include <utility>

extern "C" {
#include <ardupilotmega/mavlink.h>
}

namespace icarus::mavlink_gateway {
v1::BatteryState DecodeBatteryTelemetry(
    std::uint16_t voltage_mv, std::int16_t current_centiamp,
    std::int8_t remaining_percent, std::int64_t received_at_unix_ms,
    double simulation_percent) {
  v1::BatteryState battery;
  battery.set_received_at_unix_ms(received_at_unix_ms);
  // MAVLink reserves UINT16_MAX voltage, -1 current and -1 remaining for
  // unknown. Other negative currents represent charging and remain valid.
  const bool valid = voltage_mv > 0 && voltage_mv != UINT16_MAX &&
                     current_centiamp != -1 && remaining_percent >= 0 &&
                     remaining_percent <= 100;
  if (!valid) {
    battery.set_level(v1::HEALTH_LEVEL_UNAVAILABLE);
    return battery;
  }
  battery.set_voltage_v(voltage_mv / 1000.0);
  battery.set_current_a(current_centiamp / 100.0);
  const double remaining = std::isfinite(simulation_percent) &&
      simulation_percent >= 0 && simulation_percent <= 100
      ? simulation_percent : static_cast<double>(remaining_percent);
  battery.set_remaining_percent(remaining);
  battery.set_level(remaining >= 20.0 ? v1::HEALTH_LEVEL_HEALTHY
                                     : v1::HEALTH_LEVEL_DEGRADED);
  return battery;
}

namespace {

constexpr std::uint8_t kSourceSystem = 255;
constexpr std::uint8_t kSourceComponent = MAV_COMP_ID_MISSIONPLANNER;
constexpr std::uint32_t kModeGuided = 4;

v1::SourceHealth* MeasurementSource(v1::DroneState& state, const char* id) {
  for (auto& source : *state.mutable_sources()) {
    if (source.source_id() == id) return &source;
  }
  auto* source = state.add_sources();
  source->set_source_id(id);
  return source;
}

bool IsAccepted(std::uint8_t result) {
  return result == MAV_RESULT_ACCEPTED || result == MAV_RESULT_IN_PROGRESS;
}

}  // namespace

ArdupilotGateway::ArdupilotGateway(std::string vehicle_id)
    : vehicle_id_(std::move(vehicle_id)) {
  state_.set_vehicle_id(vehicle_id_);
  state_.mutable_mavlink()->set_level(v1::HEALTH_LEVEL_UNAVAILABLE);
}

ArdupilotGateway::~ArdupilotGateway() {
  // Destruction is cleanup, not a new state notification to captured owners.
  SetStateCallback({});
  SetEventCallback({});
  Disconnect();
}

bool ArdupilotGateway::Connect(const std::string& host, std::uint16_t port) {
  if (running_) return true;
  // A peer close stops the loops but leaves joinable thread objects and the
  // descriptor owned here. Reap those before assigning replacement threads.
  Disconnect();
  addrinfo hints{};
  hints.ai_family = AF_INET;
  hints.ai_socktype = SOCK_STREAM;
  addrinfo* addresses = nullptr;
  if (getaddrinfo(host.c_str(), std::to_string(port).c_str(), &hints,
                  &addresses) != 0) {
    return false;
  }
  int connected_socket = -1;
  for (auto* address = addresses; address != nullptr;
       address = address->ai_next) {
    connected_socket =
        socket(address->ai_family, address->ai_socktype, address->ai_protocol);
    if (connected_socket >= 0 &&
        connect(connected_socket, address->ai_addr, address->ai_addrlen) == 0) {
      break;
    }
    if (connected_socket >= 0) close(connected_socket);
    connected_socket = -1;
  }
  freeaddrinfo(addresses);
  if (connected_socket < 0) return false;

  {
    std::lock_guard<std::mutex> lock(state_mutex_);
    heartbeat_received_ = false;
    position_received_ = false;
    local_position_received_ = false;
    target_system_ = 0;
    target_component_ = 0;
    custom_mode_ = 0;
    requested_mode_.reset();
    ++mode_generation_;
    state_.Clear();
    global_origin_.reset();
    origin_revision_ = 0;
    local_origin_id_ = "ekf-origin";
    state_.set_vehicle_id(vehicle_id_);
    state_.mutable_mavlink()->set_level(v1::HEALTH_LEVEL_UNAVAILABLE);
  }
  {
    std::lock_guard<std::mutex> lock(ack_mutex_);
    latest_ack_.reset();
  }
  {
    std::lock_guard<std::mutex> lock(mission_mutex_);
    mission_ready_ = mission_started_ = mission_uploading_ = false;
    mission_progress_ = {};
    mission_requests_.clear();
    mission_ack_.reset();
  }
  socket_ = connected_socket;
  running_ = true;
  reader_thread_ = std::thread(&ArdupilotGateway::ReaderLoop, this);
  heartbeat_thread_ = std::thread(&ArdupilotGateway::HeartbeatLoop, this);
  return true;
}

void ArdupilotGateway::Disconnect() {
  const bool was_running = running_.exchange(false);
  const int descriptor = socket_.exchange(-1);
  if (descriptor >= 0) shutdown(descriptor, SHUT_RDWR);
  state_changed_.notify_all();
  ack_received_.notify_all();
  mission_changed_.notify_all();
  if (reader_thread_.joinable()) reader_thread_.join();
  if (heartbeat_thread_.joinable()) heartbeat_thread_.join();
  // The reader can still hold the descriptor until it leaves recv(). Delay
  // close until it exits, preventing a reused descriptor from being read.
  if (descriptor >= 0) close(descriptor);
  {
    std::lock_guard<std::mutex> lock(state_mutex_);
    state_.mutable_mavlink()->set_connected(false);
    state_.mutable_mavlink()->set_level(v1::HEALTH_LEVEL_FAILED);
  }
  // Repeated cleanup (including the destructor after explicit Disconnect)
  // must not call a subscriber whose lifetime may already have ended.
  if (was_running) PublishState();
}

bool ArdupilotGateway::IsConnected() const {
  return running_ && socket_ >= 0 && !TestLinkLossActive();
}

void ArdupilotGateway::ConfigureTestLinkFault(std::string mode,
                                              std::uint32_t start_after_ms,
                                              std::uint32_t duration_ms,
                                              std::uint32_t latency_ms) {
  if ((mode != "loss" && mode != "delay") || duration_ms == 0 ||
      (mode == "delay" && latency_ms == 0)) {
    throw std::invalid_argument("invalid simulator MAVLink fault configuration");
  }
  test_fault_is_loss_ = mode == "loss";
  test_fault_latency_ms_ = mode == "delay" ? latency_ms : 0;
  test_fault_duration_ms_ = duration_ms;
  test_fault_start_unix_ms_ = NowUnixMs() + start_after_ms;
}

void ArdupilotGateway::ConfigureSimulationBatteryPercent(double percent) {
  if (!std::isfinite(percent) || percent < 0.0 || percent > 100.0) {
    throw std::invalid_argument("invalid simulator battery percentage");
  }
  simulation_battery_percent_ = percent;
}

bool ArdupilotGateway::TestLinkLossActive() const {
  if (!test_fault_is_loss_) return false;
  const auto start = test_fault_start_unix_ms_.load();
  const auto duration = test_fault_duration_ms_.load();
  const auto now = NowUnixMs();
  return start >= 0 && now >= start && now < start + duration;
}

std::uint32_t ArdupilotGateway::TestLinkDelayMs() const {
  if (test_fault_is_loss_) return 0;
  const auto start = test_fault_start_unix_ms_.load();
  const auto duration = test_fault_duration_ms_.load();
  const auto now = NowUnixMs();
  if (start < 0 || now < start || now >= start + duration) return 0;
  return test_fault_latency_ms_.load();
}

bool ArdupilotGateway::WaitForReady(std::uint32_t timeout_ms) {
  std::unique_lock<std::mutex> lock(state_mutex_);
  return state_changed_.wait_for(
      lock, std::chrono::milliseconds(timeout_ms),
      [this] {
        return !running_ || (heartbeat_received_ && position_received_ &&
                             local_position_received_);
      }) &&
         running_ && heartbeat_received_ && position_received_ &&
         local_position_received_;
}

void ArdupilotGateway::SetStateCallback(StateCallback callback) {
  std::lock_guard<std::mutex> lock(state_mutex_);
  state_callback_ = std::move(callback);
}

void ArdupilotGateway::SetEventCallback(EventCallback callback) {
  std::lock_guard<std::mutex> lock(state_mutex_);
  event_callback_ = std::move(callback);
}

CommandResult ArdupilotGateway::Arm(bool arm) {
  const float params[7] = {arm ? 1.0F : 0.0F, 0, 0, 0, 0, 0, 0};
  return SendCommand(MAV_CMD_COMPONENT_ARM_DISARM, params);
}

CommandResult ArdupilotGateway::Takeoff(double altitude_agl_m) {
  const float params[7] = {0, 0, 0, 0, 0, 0,
                           static_cast<float>(altitude_agl_m)};
  return SendCommand(MAV_CMD_NAV_TAKEOFF, params);
}

CommandResult ArdupilotGateway::SetMode(std::uint32_t mode) {
  return SetModeGuarded(mode, std::nullopt, [] { return true; });
}

CommandResult ArdupilotGateway::SetModeGuarded(std::uint32_t mode,
    std::optional<std::uint64_t> expected, const std::function<bool()>& guard) {
  std::uint64_t generation;
  {
    std::lock_guard<std::mutex> ordering(mode_command_mutex_);
    if ((expected && *expected != mode_generation_.load()) || !guard())
      return {false, false, MAV_RESULT_DENIED, "mode activation superseded or cancelled"};
    mavlink_message_t message{};
    std::lock_guard<std::mutex> state_lock(state_mutex_);
    // Reader-side external mode changes share state_mutex_. Recheck and send
    // within that lock so a newly observed operator mode cannot be overwritten.
    if (expected && *expected != mode_generation_.load())
      return {false,false,MAV_RESULT_DENIED,"mode activation superseded"};
    generation = ++mode_generation_;
    requested_mode_ = mode;
    mavlink_msg_set_mode_pack(kSourceSystem, kSourceComponent, &message,
        target_system_, MAV_MODE_FLAG_CUSTOM_MODE_ENABLED, mode);
    state_changed_.notify_all();
    if (!SendMessage(&message))
      return {false, false, MAV_RESULT_FAILED, "failed to send mode request", true, generation};
  }
  std::unique_lock<std::mutex> lock(state_mutex_);
  const bool changed = state_changed_.wait_for(lock, std::chrono::seconds(10), [&] {
    return !running_ || mode_generation_ != generation || custom_mode_ == mode;
  });
  if (mode_generation_ != generation)
    return {false, false, MAV_RESULT_DENIED, "mode request superseded", true, generation};
  if (!changed || !running_)
    return {false, true, MAV_RESULT_FAILED, "mode confirmation timed out", true, generation};
  return {true, false, MAV_RESULT_ACCEPTED, "mode confirmed", true, generation};
}

CommandResult ArdupilotGateway::SetMissionCompletionMode(std::uint32_t mode,
    std::uint64_t generation, const std::function<bool()>& guard) {
  return SetModeGuarded(mode,generation,guard);
}

void ArdupilotGateway::ReleaseMissionControl(std::uint64_t generation) {
  // Even an unconfirmed AUTO send needs cancellation; a later operator or
  // supervisor request owns a different generation and cannot be overwritten.
  (void)SetModeGuarded(17, generation, [] { return true; });
}

CommandResult ArdupilotGateway::SetYaw(double heading_deg) {
  if (!std::isfinite(heading_deg) || heading_deg < 0.0 || heading_deg > 360.0) {
    return {false, false, MAV_RESULT_FAILED, "invalid absolute heading"};
  }
  // ArduPilot's CONDITION_YAW: target degrees, 30 deg/s, shortest direction,
  // absolute heading. The executor separately confirms the observed attitude
  // before declaring the containing action complete.
  const float params[7] = {static_cast<float>(heading_deg), 30.0F, 0.0F,
                           0.0F, 0.0F, 0.0F, 0.0F};
  return SendCommand(MAV_CMD_CONDITION_YAW, params);
}

CommandResult ArdupilotGateway::RotateYaw(double angle_deg,
                                          double rate_deg_s) {
  if (!std::isfinite(angle_deg) || std::abs(angle_deg) < 1.0 ||
      std::abs(angle_deg) > 360.0 || !std::isfinite(rate_deg_s) ||
      rate_deg_s < 5.0 || rate_deg_s > 60.0) {
    return {false, false, MAV_RESULT_FAILED, "invalid relative yaw request"};
  }
  // MAV_CMD_CONDITION_YAW uses a positive magnitude plus an explicit
  // direction. param4=1 makes the angle relative, which preserves full turns.
  const float direction = angle_deg > 0.0 ? 1.0F : -1.0F;
  const float params[7] = {static_cast<float>(std::abs(angle_deg)),
                           static_cast<float>(rate_deg_s), direction,
                           1.0F, 0.0F, 0.0F, 0.0F};
  return SendCommand(MAV_CMD_CONDITION_YAW, params);
}

CommandResult ArdupilotGateway::SetGroundSpeed(double speed_mps) {
  if (!std::isfinite(speed_mps) || speed_mps <= 0.0) {
    return {false, false, MAV_RESULT_FAILED, "invalid ground speed"};
  }
  // MAV_CMD_DO_CHANGE_SPEED: groundspeed type=1, m/s, throttle unchanged=-1.
  // ArduCopter applies this to the active GUIDED horizontal position controller.
  // SendCommand waits for COMMAND_ACK; navigation never assumes acceptance.
  const float params[7] = {1.0F, static_cast<float>(speed_mps), -1.0F,
                           0.0F, 0.0F, 0.0F, 0.0F};
  return SendCommand(MAV_CMD_DO_CHANGE_SPEED, params);
}

bool ArdupilotGateway::GotoGlobal(const v1::GeoPosition& position,
                                  std::optional<double> heading_deg) {
  if (position.frame() != v1::COORDINATE_FRAME_WGS84_AMSL &&
      position.frame() != v1::COORDINATE_FRAME_WGS84_ABOVE_HOME) return false;
  mavlink_message_t message{};
  std::uint8_t frame = MAV_FRAME_GLOBAL_RELATIVE_ALT_INT;
  if (position.frame() == v1::COORDINATE_FRAME_WGS84_AMSL) {
    frame = MAV_FRAME_GLOBAL_INT;
  }
  const std::uint16_t mask =
      POSITION_TARGET_TYPEMASK_VX_IGNORE | POSITION_TARGET_TYPEMASK_VY_IGNORE |
      POSITION_TARGET_TYPEMASK_VZ_IGNORE | POSITION_TARGET_TYPEMASK_AX_IGNORE |
      POSITION_TARGET_TYPEMASK_AY_IGNORE | POSITION_TARGET_TYPEMASK_AZ_IGNORE |
      (heading_deg ? 0 : POSITION_TARGET_TYPEMASK_YAW_IGNORE) |
      POSITION_TARGET_TYPEMASK_YAW_RATE_IGNORE;
  mavlink_msg_set_position_target_global_int_pack(
      kSourceSystem, kSourceComponent, &message, 0, target_system_,
      target_component_, frame, mask,
      static_cast<std::int32_t>(std::llround(position.latitude_deg() * 1e7)),
      static_cast<std::int32_t>(std::llround(position.longitude_deg() * 1e7)),
      static_cast<float>(position.altitude_m()), 0, 0, 0, 0, 0, 0,
      heading_deg ? static_cast<float>(*heading_deg * 3.14159265358979323846 / 180.0) : 0.0F, 0);
  return SendMessage(&message);
}

bool ArdupilotGateway::GotoLocal(const v1::LocalPositionNed& position,
                                 std::optional<double> heading_deg) {
  // Frame conversion belongs to the executor, never silently discard an
  // unresolved above-home target at the wire boundary.
  if (position.has_altitude_above_home_m()) return false;
  mavlink_message_t message{};
  const std::uint16_t mask =
      POSITION_TARGET_TYPEMASK_VX_IGNORE | POSITION_TARGET_TYPEMASK_VY_IGNORE |
      POSITION_TARGET_TYPEMASK_VZ_IGNORE | POSITION_TARGET_TYPEMASK_AX_IGNORE |
      POSITION_TARGET_TYPEMASK_AY_IGNORE | POSITION_TARGET_TYPEMASK_AZ_IGNORE |
      (heading_deg ? 0 : POSITION_TARGET_TYPEMASK_YAW_IGNORE) |
      POSITION_TARGET_TYPEMASK_YAW_RATE_IGNORE;
  mavlink_msg_set_position_target_local_ned_pack(
      kSourceSystem, kSourceComponent, &message, 0, target_system_,
      target_component_, MAV_FRAME_LOCAL_NED, mask,
      static_cast<float>(position.north_m()),
      static_cast<float>(position.east_m()),
      static_cast<float>(position.down_m()), 0, 0, 0, 0, 0, 0,
      heading_deg ? static_cast<float>(*heading_deg * 3.14159265358979323846 / 180.0) : 0.0F, 0);
  return SendMessage(&message);
}

bool ArdupilotGateway::StreamLocal(const v1::LocalPositionNed& position,
                                   double north_mps, double east_mps,
                                   double down_mps,
                                   std::optional<double> heading_deg) {
  if (position.has_altitude_above_home_m() ||
      !std::isfinite(position.north_m()) || !std::isfinite(position.east_m()) ||
      !std::isfinite(position.down_m()) || !std::isfinite(north_mps) ||
      !std::isfinite(east_mps) || !std::isfinite(down_mps) ||
      (heading_deg && !std::isfinite(*heading_deg))) return false;
  mavlink_message_t message{};
  const std::uint16_t mask =
      POSITION_TARGET_TYPEMASK_AX_IGNORE | POSITION_TARGET_TYPEMASK_AY_IGNORE |
      POSITION_TARGET_TYPEMASK_AZ_IGNORE |
      (heading_deg ? 0 : POSITION_TARGET_TYPEMASK_YAW_IGNORE) |
      POSITION_TARGET_TYPEMASK_YAW_RATE_IGNORE;
  mavlink_msg_set_position_target_local_ned_pack(
      kSourceSystem, kSourceComponent, &message, 0, target_system_,
      target_component_, MAV_FRAME_LOCAL_NED, mask,
      static_cast<float>(position.north_m()),
      static_cast<float>(position.east_m()), static_cast<float>(position.down_m()),
      static_cast<float>(north_mps), static_cast<float>(east_mps),
      static_cast<float>(down_mps), 0, 0, 0,
      heading_deg ? static_cast<float>(*heading_deg * 3.14159265358979323846 / 180.0) : 0.0F,
      0);
  return SendMessage(&message);
}

CommandResult ArdupilotGateway::UploadMission(
    const std::vector<MissionItem>& items, std::uint32_t timeout_ms) {
  std::lock_guard<std::mutex> operation(mission_operation_mutex_);
  std::uint8_t system, component;
  {
    std::lock_guard<std::mutex> lock(state_mutex_);
    system = target_system_;
    component = target_component_;
    mission_mode_generation_ = mode_generation_;
    if (!heartbeat_received_ || custom_mode_ == 3) {
      return {false, false, MAV_MISSION_DENIED,
              "upload requires a connected vehicle outside AUTO"};
    }
  }
  std::unique_lock<std::mutex> lock(mission_mutex_);
  mission_ready_ = false;
  mission_started_ = false;
  mission_progress_ = {};
  mission_requests_.clear();
  mission_ack_.reset();
  if (items.size() < 2 || items.size() > 512 || timeout_ms == 0 || !IsConnected()) {
    return {false, false, MAV_MISSION_ERROR, "invalid mission size or disconnected vehicle"};
  }
  for (const auto& item : items) {
    if (!std::isfinite(item.z) ||
        std::any_of(item.params.begin(), item.params.end(),
                    [](float value) { return std::isinf(value); })) {
      return {false, false, MAV_MISSION_INVALID, "invalid mission values"};
    }
  }
  mission_uploading_ = true;
  mission_count_ = items.size();
  std::vector<bool> sent(items.size(), false);
  mavlink_message_t last{};
  mavlink_msg_mission_count_pack(kSourceSystem, kSourceComponent, &last,
      system, component, static_cast<std::uint16_t>(items.size()), MAV_MISSION_TYPE_MISSION);
  const auto deadline = std::chrono::steady_clock::now() +
      std::chrono::milliseconds(std::min(timeout_ms, 30000U));
  auto finish = [&](bool accepted, bool timed_out, std::uint8_t result,
                    const char* reason) {
    mission_uploading_ = false;
    mission_ready_ = accepted;
    return CommandResult{accepted, timed_out, result, reason};
  };
  if (!SendMessage(&last)) return finish(false, false, MAV_MISSION_ERROR, "mission count send failed");
  while (running_ && std::chrono::steady_clock::now() < deadline) {
    mission_changed_.wait_until(lock, std::min(deadline,
        std::chrono::steady_clock::now() + std::chrono::milliseconds(500)), [&] {
      return !running_ || mission_ack_ || !mission_requests_.empty();
    });
    if (!running_) break;
    if (mission_ack_) {
      const auto result = *mission_ack_;
      if (result != MAV_MISSION_ACCEPTED)
        return finish(false, false, result, "autopilot rejected mission upload");
      if (!std::all_of(sent.begin(), sent.end(), [](bool value) { return value; }))
        return finish(false, false, MAV_MISSION_ERROR, "premature mission acknowledgement");
      return finish(true, false, result, "autopilot stored complete mission");
    }
    if (!mission_requests_.empty()) {
      const auto seq = mission_requests_.front();
      mission_requests_.pop_front();
      if (seq >= items.size()) {
        mavlink_message_t error{};
        mavlink_msg_mission_ack_pack(kSourceSystem, kSourceComponent, &error,
            system, component, MAV_MISSION_INVALID_SEQUENCE, MAV_MISSION_TYPE_MISSION);
        SendMessage(&error);
        return finish(false, false, MAV_MISSION_INVALID_SEQUENCE, "autopilot requested invalid mission index");
      }
      const auto& item = items[seq];
      // ArduPilot can send the legacy MISSION_REQUEST even with INT support.
      // Always retain integer geographic precision: float degrees can move a
      // waypoint by metres. Unsupported INT transfers fail; never downgrade.
      mavlink_msg_mission_item_int_pack(kSourceSystem, kSourceComponent, &last,
          system, component, seq, item.frame, item.command, 0, 1,
          item.params[0], item.params[1], item.params[2], item.params[3],
          item.x, item.y, item.z, MAV_MISSION_TYPE_MISSION);
      sent[seq] = true;
    }
    // Resend the last payload if its request/ACK was lost. Repeated requests
    // are idempotent; only a complete acknowledged transfer enables start.
    if (!SendMessage(&last))
      return finish(false, false, MAV_MISSION_ERROR, "mission transfer send failed");
  }
  return finish(false, running_, MAV_MISSION_ERROR,
                running_ ? "mission upload timed out" : "disconnected during mission upload");
}

CommandResult ArdupilotGateway::StartMission(std::uint16_t seq) {
  return StartMissionGuarded(seq, [] { return true; });
}

CommandResult ArdupilotGateway::StartMissionGuarded(std::uint16_t seq,
    const std::function<bool()>& guard) {
  std::lock_guard<std::mutex> operation(mission_operation_mutex_);
  const auto generation = mission_mode_generation_;
  const auto selected = SelectMissionCurrent(seq, generation, guard);
  if (!selected.accepted) return selected;
  {
    std::lock_guard<std::mutex> lock(mission_mutex_);
    mission_started_ = true;
  }
  auto result = SetModeGuarded(3, generation, guard);
  if (!result.accepted) {
    std::lock_guard<std::mutex> lock(mission_mutex_);
    mission_started_ = false;
  }
  return result;
}

CommandResult ArdupilotGateway::SetMissionCurrent(std::uint16_t seq) {
  return SetMissionCurrentGuarded(seq, mode_generation_, [] { return true; });
}

CommandResult ArdupilotGateway::SetMissionCurrentGuarded(std::uint16_t seq,
    std::uint64_t generation, const std::function<bool()>& guard) {
  std::lock_guard<std::mutex> operation(mission_operation_mutex_);
  return SelectMissionCurrent(seq, generation, guard);
}

CommandResult ArdupilotGateway::SelectMissionCurrent(std::uint16_t first_seq,
    std::optional<std::uint64_t> generation, const std::function<bool()>& guard) {
  {
    std::unique_lock<std::mutex> ordering(mode_command_mutex_);
    if (!guard()) return {false,false,MAV_RESULT_DENIED,"mission selection cancelled"};
    std::unique_lock<std::mutex> state_lock(state_mutex_);
    if (generation && *generation != mode_generation_)
      return {false,false,MAV_RESULT_DENIED,"mission selection superseded"};
    const auto system=target_system_, component=target_component_;
    std::unique_lock<std::mutex> lock(mission_mutex_);
    if (!mission_ready_ || first_seq == 0 || first_seq >= mission_count_)
      return {false, false, MAV_RESULT_DENIED, "no accepted mission or invalid first index"};
    mission_progress_.current_seq = -1;
    mavlink_message_t message{};
    mavlink_msg_mission_set_current_pack(kSourceSystem, kSourceComponent,
        &message, system, component, first_seq);
    if (!SendMessage(&message))
      return {false, false, MAV_RESULT_FAILED, "mission selection send failed"};
    state_lock.unlock();
    ordering.unlock();
    if (!mission_changed_.wait_for(lock, std::chrono::seconds(2), [&] {
          return !running_ || mission_progress_.current_seq == first_seq;
        }) || !running_)
      return {false, true, MAV_RESULT_FAILED, "mission selection not confirmed"};
    mission_progress_.reached_seq = -1;
  }
  return {true, false, MAV_RESULT_ACCEPTED, "mission selection confirmed"};
}

MissionProgress ArdupilotGateway::GetMissionProgress() const {
  std::lock_guard<std::mutex> state_lock(state_mutex_);
  std::lock_guard<std::mutex> lock(mission_mutex_);
  auto progress = mission_progress_;
  progress.custom_mode = custom_mode_;
  progress.mode_generation = mode_generation_;
  return progress;
}

void ArdupilotGateway::ReaderLoop() {
  mavlink_message_t message{};
  mavlink_status_t parser_status{};
  std::array<std::uint8_t, 4096> buffer{};
  using DelayedMessage =
      std::pair<std::chrono::steady_clock::time_point, mavlink_message_t>;
  std::deque<DelayedMessage> delayed_messages;
  const auto flush_delayed = [&] {
    const auto now = std::chrono::steady_clock::now();
    while (!delayed_messages.empty() &&
           delayed_messages.front().first <= now) {
      auto delayed = std::move(delayed_messages.front().second);
      delayed_messages.pop_front();
      if (!TestLinkLossActive()) HandleMessage(&delayed);
    }
  };
  while (running_) {
    const auto count = recv(socket_.load(), buffer.data(), buffer.size(), 0);
    if (count <= 0) break;
    flush_delayed();
    for (ssize_t index = 0; index < count; ++index) {
      if (mavlink_parse_char(MAVLINK_COMM_0, buffer[index], &message,
                             &parser_status)) {
        if (TestLinkLossActive()) continue;
        if (const auto delay_ms = TestLinkDelayMs(); delay_ms > 0) {
          delayed_messages.emplace_back(
              std::chrono::steady_clock::now() +
                  std::chrono::milliseconds(delay_ms),
              message);
          continue;
        }
        HandleMessage(&message);
      }
    }
    flush_delayed();
  }
  running_ = false;
  state_changed_.notify_all();
  ack_received_.notify_all();
  mission_changed_.notify_all();
}

void ArdupilotGateway::HeartbeatLoop() {
  while (running_) {
    mavlink_message_t message{};
    mavlink_msg_heartbeat_pack(kSourceSystem, kSourceComponent, &message,
                               MAV_TYPE_GCS, MAV_AUTOPILOT_INVALID, 0, 0,
                               MAV_STATE_ACTIVE);
    SendMessage(&message);
    std::this_thread::sleep_for(std::chrono::milliseconds(500));
  }
}

void ArdupilotGateway::HandleMessage(const void* opaque) {
  const auto& message = *static_cast<const mavlink_message_t*>(opaque);
  bool publish = false;
  bool request_streams = false;
  bool request_home = false;
  std::optional<std::pair<v1::EventSeverity, std::string>> autopilot_event;
  {
    std::lock_guard<std::mutex> lock(state_mutex_);
    state_.set_observed_at_unix_ms(NowUnixMs());
    state_.mutable_mavlink()->set_connected(true);
    state_.mutable_mavlink()->set_level(v1::HEALTH_LEVEL_HEALTHY);
    state_.mutable_mavlink()->set_heartbeat_age_ms(0);
    state_.mutable_mavlink()->set_transport("tcp");
    switch (message.msgid) {
      case MAVLINK_MSG_ID_HEARTBEAT: {
        mavlink_heartbeat_t heartbeat{};
        mavlink_msg_heartbeat_decode(&message, &heartbeat);
        if (heartbeat.type == MAV_TYPE_GCS || heartbeat.autopilot == MAV_AUTOPILOT_INVALID ||
            message.compid != MAV_COMP_ID_AUTOPILOT1 ||
            (target_system_ != 0 && message.sysid != target_system_)) break;
        target_system_ = message.sysid;
        target_component_ = message.compid;
        request_streams = !heartbeat_received_;
        heartbeat_received_ = true;
        const bool armed =
            (heartbeat.base_mode & MAV_MODE_FLAG_SAFETY_ARMED) != 0;
        state_.set_armed(armed);
        if (state_.landed()) {
          state_.set_flight_phase(armed ? v1::FLIGHT_PHASE_ARMED_GROUND
                                        : v1::FLIGHT_PHASE_DISARMED);
        }
        if (heartbeat_received_ && custom_mode_ != heartbeat.custom_mode &&
            (!requested_mode_ || *requested_mode_ != heartbeat.custom_mode))
          ++mode_generation_;
        custom_mode_ = heartbeat.custom_mode;
        if (requested_mode_ == custom_mode_) requested_mode_.reset();
        publish = true;
        break;
      }
      case MAVLINK_MSG_ID_GLOBAL_POSITION_INT: {
        mavlink_global_position_int_t position{};
        mavlink_msg_global_position_int_decode(&message, &position);
        auto* geo = state_.mutable_position()->mutable_geo();
        geo->set_latitude_deg(position.lat / 1e7);
        geo->set_longitude_deg(position.lon / 1e7);
        geo->set_altitude_m(position.alt / 1000.0);
        geo->set_frame(v1::COORDINATE_FRAME_WGS84_AMSL);
        state_.set_relative_altitude_m(position.relative_alt / 1000.0);
        state_.set_relative_altitude_received_at_unix_ms(NowUnixMs());
        state_.set_relative_altitude_time_boot_ms(position.time_boot_ms);
        state_.mutable_velocity_ned_mps()->set_x(position.vx / 100.0);
        state_.mutable_velocity_ned_mps()->set_y(position.vy / 100.0);
        state_.mutable_velocity_ned_mps()->set_z(position.vz / 100.0);
        state_.set_ground_speed_mps(std::hypot(position.vx, position.vy) / 100.0);
        position_received_ = true;
        publish = true;
        break;
      }
      case MAVLINK_MSG_ID_ATTITUDE: {
        mavlink_attitude_t attitude{};
        mavlink_msg_attitude_decode(&message, &attitude);
        state_.set_attitude_received_at_unix_ms(NowUnixMs());
        state_.mutable_attitude()->set_roll_rad(attitude.roll);
        state_.mutable_attitude()->set_pitch_rad(attitude.pitch);
        state_.mutable_attitude()->set_yaw_rad(attitude.yaw);
        publish = true;
        break;
      }
      case MAVLINK_MSG_ID_LOCAL_POSITION_NED: {
        mavlink_local_position_ned_t position{};
        mavlink_msg_local_position_ned_decode(&message, &position);
        auto* local = state_.mutable_local_position_ned();
        local->set_north_m(position.x);
        local->set_east_m(position.y);
        local->set_down_m(position.z);
        local->set_origin_id(local_origin_id_);
        state_.set_local_position_received_at_unix_ms(NowUnixMs());
        state_.set_local_position_time_boot_ms(position.time_boot_ms);
        local_position_received_ = true;
        publish = true;
        break;
      }
      case MAVLINK_MSG_ID_DISTANCE_SENSOR: {
        mavlink_distance_sensor_t reading{};
        mavlink_msg_distance_sensor_decode(&message, &reading);
        if (reading.orientation != MAV_SENSOR_ROTATION_PITCH_270) break;
        auto* sensors = state_.mutable_navigation_sensors();
        sensors->set_source("autopilot_mavlink");
        sensors->set_range_received_at_unix_ms(NowUnixMs());
        sensors->set_downward_range_m(reading.current_distance / 100.0);
        sensors->set_range_valid(reading.current_distance >= reading.min_distance &&
                                reading.current_distance <= reading.max_distance &&
                                reading.current_distance > 0);
        publish = true;
        break;
      }
      case MAVLINK_MSG_ID_OPTICAL_FLOW: {
        mavlink_optical_flow_t reading{};
        mavlink_msg_optical_flow_decode(&message, &reading);
        auto* sensors = state_.mutable_navigation_sensors();
        sensors->set_source("autopilot_mavlink");
        sensors->set_flow_received_at_unix_ms(NowUnixMs());
        sensors->set_flow_velocity_x_mps(reading.flow_comp_m_x);
        sensors->set_flow_velocity_y_mps(reading.flow_comp_m_y);
        sensors->set_flow_quality(reading.quality);
        publish = true;
        break;
      }
      case MAVLINK_MSG_ID_SCALED_PRESSURE: {
        mavlink_scaled_pressure_t reading{};
        mavlink_msg_scaled_pressure_decode(&message, &reading);
        auto* sensors = state_.mutable_navigation_sensors();
        sensors->set_source("autopilot_mavlink");
        sensors->set_pressure_received_at_unix_ms(NowUnixMs());
        sensors->set_pressure_hpa(reading.press_abs);
        publish = true;
        break;
      }
      case MAVLINK_MSG_ID_SYS_STATUS: {
        mavlink_sys_status_t status{};
        mavlink_msg_sys_status_decode(&message, &status);
        *state_.mutable_battery() = DecodeBatteryTelemetry(
            status.voltage_battery, status.current_battery,
            status.battery_remaining, NowUnixMs(), simulation_battery_percent_.load());
        publish = true;
        break;
      }
      case MAVLINK_MSG_ID_GPS_RAW_INT: {
        mavlink_gps_raw_int_t gps{};
        mavlink_msg_gps_raw_int_decode(&message, &gps);
        // A receiver fix is not evidence that the EKF has aligned or fused it.
        // Keep raw receiver diagnostics separate from fused navigation validity.
        auto* source = MeasurementSource(state_, "autopilot_gps");
        source->set_sample_time_unix_ms(NowUnixMs());
        source->set_level(gps.fix_type >= GPS_FIX_TYPE_3D_FIX
                              ? v1::HEALTH_LEVEL_HEALTHY : v1::HEALTH_LEVEL_DEGRADED);
        source->set_detail("fix_type=" + std::to_string(gps.fix_type) +
                           "; satellites=" + std::to_string(gps.satellites_visible) +
                           "; raw receiver status, not EKF fusion");
        publish = true;
        break;
      }
      case MAVLINK_MSG_ID_EKF_STATUS_REPORT: {
        mavlink_ekf_status_report_t ekf{};
        mavlink_msg_ekf_status_report_decode(&message, &ekf);
        auto* estimator = state_.mutable_estimator();
        // Only the estimator report owns fused validity. Position/attitude
        // packets continue publishing even while EKF alignment is incomplete.
        const bool initialized = (ekf.flags & EKF_UNINITIALIZED) == 0;
        const bool attitude = initialized && (ekf.flags & EKF_ATTITUDE) != 0;
        const bool horizontal = initialized &&
            (ekf.flags & (EKF_POS_HORIZ_ABS | EKF_POS_HORIZ_REL)) != 0;
        const bool vertical = initialized && (ekf.flags & EKF_POS_VERT_ABS) != 0;
        estimator->set_attitude_valid(attitude);
        estimator->set_horizontal_position_valid(horizontal);
        estimator->set_vertical_position_valid(vertical);
        estimator->set_absolute_position_valid(initialized && (ekf.flags & EKF_POS_HORIZ_ABS) != 0);
        estimator->set_primary_horizontal_source("autopilot_EKF");
        estimator->set_level(attitude && horizontal && vertical
                                 ? v1::HEALTH_LEVEL_HEALTHY
                                 : v1::HEALTH_LEVEL_DEGRADED);
        auto* source = MeasurementSource(state_, "autopilot_ekf");
        source->set_sample_time_unix_ms(NowUnixMs());
        source->set_level(estimator->level());
        source->set_detail("flags=" + std::to_string(ekf.flags) +
                           "; fused validity from EKF_STATUS_REPORT");
        publish = true;
        break;
      }
      case MAVLINK_MSG_ID_GPS_GLOBAL_ORIGIN: {
        mavlink_gps_global_origin_t origin{};
        mavlink_msg_gps_global_origin_decode(&message, &origin);
        const std::array<std::int32_t, 3> coordinates{
            origin.latitude, origin.longitude, origin.altitude};
        if (global_origin_ && *global_origin_ != coordinates) {
          local_origin_id_ = "ekf-origin-" + std::to_string(++origin_revision_);
          state_.clear_home_local_position_ned();
          state_.set_home_position_received_at_unix_ms(0);
          state_.clear_local_position_ned();
          local_position_received_ = false;
          request_home = true;
        }
        global_origin_ = coordinates;
        publish = true;
        break;
      }
      case MAVLINK_MSG_ID_HOME_POSITION: {
        mavlink_home_position_t home{};
        mavlink_msg_home_position_decode(&message, &home);
        auto* geo = state_.mutable_home()->mutable_geo();
        geo->set_latitude_deg(home.latitude / 1e7);
        geo->set_longitude_deg(home.longitude / 1e7);
        geo->set_altitude_m(home.altitude / 1000.0);
        geo->set_frame(v1::COORDINATE_FRAME_WGS84_AMSL);
        state_.clear_home_local_position_ned();
        state_.set_home_position_received_at_unix_ms(0);
        if (std::isfinite(home.x) && std::isfinite(home.y) && std::isfinite(home.z)) {
          auto* local = state_.mutable_home_local_position_ned();
          local->set_north_m(home.x);
          local->set_east_m(home.y);
          local->set_down_m(home.z);
          local->set_origin_id(local_origin_id_);
          state_.set_home_position_received_at_unix_ms(NowUnixMs());
        }
        publish = true;
        break;
      }
      case MAVLINK_MSG_ID_EXTENDED_SYS_STATE: {
        mavlink_extended_sys_state_t extended{};
        mavlink_msg_extended_sys_state_decode(&message, &extended);
        const bool landed = extended.landed_state == MAV_LANDED_STATE_ON_GROUND;
        state_.set_landed(landed);
        state_.set_flight_phase(
            landed ? (state_.armed() ? v1::FLIGHT_PHASE_ARMED_GROUND
                                     : v1::FLIGHT_PHASE_DISARMED)
                   : v1::FLIGHT_PHASE_AIRBORNE);
        publish = true;
        break;
      }
      case MAVLINK_MSG_ID_MISSION_REQUEST_INT:
      case MAVLINK_MSG_ID_MISSION_REQUEST: {
        if (message.sysid != target_system_ || message.compid != target_component_) break;
        std::uint16_t seq;
        std::uint8_t system, component, type;
        if (message.msgid == MAVLINK_MSG_ID_MISSION_REQUEST_INT) {
          mavlink_mission_request_int_t request{};
          mavlink_msg_mission_request_int_decode(&message, &request);
          seq = request.seq; system = request.target_system;
          component = request.target_component; type = request.mission_type;
        } else {
          mavlink_mission_request_t request{};
          mavlink_msg_mission_request_decode(&message, &request);
          seq = request.seq; system = request.target_system;
          component = request.target_component; type = request.mission_type;
        }
        if (system != kSourceSystem || (component != 0 && component != kSourceComponent) ||
            type != MAV_MISSION_TYPE_MISSION) break;
        std::lock_guard<std::mutex> mission_lock(mission_mutex_);
        if (mission_uploading_ && mission_requests_.size() < 1024) {
          mission_requests_.push_back(seq);
          mission_changed_.notify_all();
        }
        break;
      }
      case MAVLINK_MSG_ID_MISSION_ACK: {
        if (message.sysid != target_system_ || message.compid != target_component_) break;
        mavlink_mission_ack_t ack{};
        mavlink_msg_mission_ack_decode(&message, &ack);
        if (ack.target_system != kSourceSystem ||
            (ack.target_component != 0 && ack.target_component != kSourceComponent) ||
            ack.mission_type != MAV_MISSION_TYPE_MISSION) break;
        std::lock_guard<std::mutex> mission_lock(mission_mutex_);
        if (mission_uploading_) { mission_ack_ = ack.type; mission_changed_.notify_all(); }
        break;
      }
      case MAVLINK_MSG_ID_MISSION_CURRENT:
      case MAVLINK_MSG_ID_MISSION_ITEM_REACHED: {
        if (message.sysid != target_system_ || message.compid != target_component_) break;
        std::lock_guard<std::mutex> mission_lock(mission_mutex_);
        if (mission_uploading_ || !mission_ready_) break;
        if (message.msgid == MAVLINK_MSG_ID_MISSION_CURRENT) {
          mavlink_mission_current_t current{};
          mavlink_msg_mission_current_decode(&message, &current);
          mission_progress_.current_seq = current.seq;
          mission_progress_.mission_state = current.mission_state;
        } else {
          mavlink_mission_item_reached_t reached{};
          mavlink_msg_mission_item_reached_decode(&message, &reached);
          if (!mission_started_ || reached.seq >= mission_count_) break;
          mission_progress_.reached_seq = reached.seq;
        }
        ++mission_progress_.revision;
        mission_progress_.received_at_unix_ms = NowUnixMs();
        mission_changed_.notify_all();
        break;
      }
      case MAVLINK_MSG_ID_COMMAND_ACK: {
        mavlink_command_ack_t ack{};
        mavlink_msg_command_ack_decode(&message, &ack);
        {
          std::lock_guard<std::mutex> ack_lock(ack_mutex_);
          latest_ack_ = std::make_pair(ack.command, ack.result);
        }
        ack_received_.notify_all();
        break;
      }
      case MAVLINK_MSG_ID_STATUSTEXT: {
        mavlink_statustext_t text{};
        mavlink_msg_statustext_decode(&message, &text);
        const auto length = strnlen(text.text, sizeof(text.text));
        autopilot_event = std::make_pair(
            text.severity <= MAV_SEVERITY_ERROR ? v1::EVENT_SEVERITY_ERROR
                                                : v1::EVENT_SEVERITY_INFO,
            std::string(text.text, length));
        break;
      }
      default:
        break;
    }
  }
  if (publish) {
    state_changed_.notify_all();
    PublishState();
  }
  if (request_home && !request_streams) {
    mavlink_message_t request{};
    mavlink_msg_command_long_pack(
        kSourceSystem, kSourceComponent, &request, target_system_, target_component_,
        MAV_CMD_REQUEST_MESSAGE, 0,
        static_cast<float>(MAVLINK_MSG_ID_HOME_POSITION), 0, 0, 0, 0, 0, 0);
    SendMessage(&request);
  }
  if (request_streams) {
    mavlink_message_t request{};
    mavlink_msg_request_data_stream_pack(
        kSourceSystem, kSourceComponent, &request, target_system_,
        target_component_, MAV_DATA_STREAM_ALL, 20, 1);
    SendMessage(&request);
    mavlink_message_t home_request{};
    mavlink_msg_command_long_pack(
        kSourceSystem, kSourceComponent, &home_request, target_system_,
        target_component_, MAV_CMD_REQUEST_MESSAGE, 0,
        static_cast<float>(MAVLINK_MSG_ID_HOME_POSITION), 0, 0, 0, 0, 0, 0);
    SendMessage(&home_request);
    mavlink_message_t origin_request{};
    mavlink_msg_command_long_pack(
        kSourceSystem, kSourceComponent, &origin_request, target_system_,
        target_component_, MAV_CMD_SET_MESSAGE_INTERVAL, 0,
        static_cast<float>(MAVLINK_MSG_ID_GPS_GLOBAL_ORIGIN), 1'000'000, 0, 0, 0, 0, 0);
    SendMessage(&origin_request);
    mavlink_message_t landed_request{};
    mavlink_msg_command_long_pack(
        kSourceSystem, kSourceComponent, &landed_request, target_system_,
        target_component_, MAV_CMD_SET_MESSAGE_INTERVAL, 0,
        static_cast<float>(MAVLINK_MSG_ID_EXTENDED_SYS_STATE), 200'000, 0, 0, 0,
        0, 0);
    SendMessage(&landed_request);
    // Perception projects body-frame ranges through the vehicle's local NED
    // pose. Request that prerequisite explicitly instead of relying on the
    // implementation-defined contents/timing of MAV_DATA_STREAM_ALL.
    mavlink_message_t local_position_request{};
    mavlink_msg_command_long_pack(
        kSourceSystem, kSourceComponent, &local_position_request,
        target_system_, target_component_, MAV_CMD_SET_MESSAGE_INTERVAL, 0,
        static_cast<float>(MAVLINK_MSG_ID_LOCAL_POSITION_NED), 200'000, 0, 0,
        0, 0, 0);
    SendMessage(&local_position_request);
    for (const auto message_id : {MAVLINK_MSG_ID_DISTANCE_SENSOR,
                                  MAVLINK_MSG_ID_OPTICAL_FLOW,
                                  MAVLINK_MSG_ID_SCALED_PRESSURE}) {
      mavlink_message_t sensor_request{};
      mavlink_msg_command_long_pack(
          kSourceSystem, kSourceComponent, &sensor_request, target_system_,
          target_component_, MAV_CMD_SET_MESSAGE_INTERVAL, 0,
          static_cast<float>(message_id), 200'000, 0, 0, 0, 0, 0);
      SendMessage(&sensor_request);
    }
  }
  if (autopilot_event) {
    PublishEvent(autopilot_event->first, "AUTOPILOT_STATUS",
                 std::move(autopilot_event->second));
  }
}

bool ArdupilotGateway::SendMessage(const void* opaque) {
  if (!IsConnected()) return false;
  if (const auto delay_ms = TestLinkDelayMs(); delay_ms > 0) {
    std::this_thread::sleep_for(std::chrono::milliseconds(delay_ms));
  }
  const auto& message = *static_cast<const mavlink_message_t*>(opaque);
  std::array<std::uint8_t, MAVLINK_MAX_PACKET_LEN> bytes{};
  const auto length = mavlink_msg_to_send_buffer(bytes.data(), &message);
  std::lock_guard<std::mutex> lock(send_mutex_);
  std::size_t sent = 0;
  while (sent < length) {
    const auto count =
        send(socket_.load(), bytes.data() + sent, length - sent, MSG_NOSIGNAL);
    if (count <= 0) return false;
    sent += static_cast<std::size_t>(count);
  }
  return true;
}

CommandResult ArdupilotGateway::SendCommand(std::uint16_t command,
                                            const float params[7]) {
  std::unique_lock<std::mutex> command_lock(ack_mutex_);
  for (std::uint8_t attempt = 0; attempt < 2; ++attempt) {
    latest_ack_.reset();
    mavlink_message_t message{};
    mavlink_msg_command_long_pack(
        kSourceSystem, kSourceComponent, &message, target_system_,
        target_component_, command, attempt, params[0], params[1], params[2],
        params[3], params[4], params[5], params[6]);
    if (!SendMessage(&message)) {
      return {false, false, MAV_RESULT_FAILED, "failed to send MAVLink command"};
    }
    const bool received = ack_received_.wait_for(
        command_lock, std::chrono::seconds(2), [this, command] {
          return !running_ ||
                 (latest_ack_ && latest_ack_->first == command);
        });
    if (received && latest_ack_) {
      const auto result = latest_ack_->second;
      return {IsAccepted(result), false, result,
              IsAccepted(result) ? "autopilot accepted command"
                                 : "autopilot rejected command"};
    }
  }
  return {false, true, MAV_RESULT_FAILED, "MAVLink command acknowledgement timed out"};
}

void ArdupilotGateway::PublishState() {
  StateCallback callback;
  v1::DroneState snapshot;
  {
    std::lock_guard<std::mutex> lock(state_mutex_);
    callback = state_callback_;
    snapshot = state_;
  }
  if (callback) callback(snapshot);
}

void ArdupilotGateway::PublishEvent(v1::EventSeverity severity, std::string code,
                                    std::string message) {
  EventCallback callback;
  {
    std::lock_guard<std::mutex> lock(state_mutex_);
    callback = event_callback_;
  }
  if (!callback) return;
  v1::VehicleEvent event;
  event.set_vehicle_id(vehicle_id_);
  event.set_occurred_at_unix_ms(NowUnixMs());
  event.set_severity(severity);
  event.set_code(std::move(code));
  event.set_message(std::move(message));
  callback(event);
}

std::int64_t ArdupilotGateway::NowUnixMs() {
  return std::chrono::duration_cast<std::chrono::milliseconds>(
             std::chrono::system_clock::now().time_since_epoch())
      .count();
}

}  // namespace icarus::mavlink_gateway

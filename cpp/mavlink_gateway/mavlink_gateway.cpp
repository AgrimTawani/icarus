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
namespace {

constexpr std::uint8_t kSourceSystem = 255;
constexpr std::uint8_t kSourceComponent = MAV_COMP_ID_MISSIONPLANNER;
constexpr std::uint32_t kModeGuided = 4;

bool IsAccepted(std::uint8_t result) {
  return result == MAV_RESULT_ACCEPTED || result == MAV_RESULT_IN_PROGRESS;
}

}  // namespace

ArdupilotGateway::ArdupilotGateway(std::string vehicle_id)
    : vehicle_id_(std::move(vehicle_id)) {
  state_.set_vehicle_id(vehicle_id_);
  state_.mutable_mavlink()->set_level(v1::HEALTH_LEVEL_UNAVAILABLE);
}

ArdupilotGateway::~ArdupilotGateway() { Disconnect(); }

bool ArdupilotGateway::Connect(const std::string& host, std::uint16_t port) {
  if (running_) return true;
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

  socket_ = connected_socket;
  running_ = true;
  reader_thread_ = std::thread(&ArdupilotGateway::ReaderLoop, this);
  heartbeat_thread_ = std::thread(&ArdupilotGateway::HeartbeatLoop, this);
  return true;
}

void ArdupilotGateway::Disconnect() {
  if (!running_.exchange(false)) return;
  const int descriptor = socket_.exchange(-1);
  if (descriptor >= 0) {
    shutdown(descriptor, SHUT_RDWR);
    close(descriptor);
  }
  state_changed_.notify_all();
  ack_received_.notify_all();
  if (reader_thread_.joinable()) reader_thread_.join();
  if (heartbeat_thread_.joinable()) heartbeat_thread_.join();
  {
    std::lock_guard<std::mutex> lock(state_mutex_);
    state_.mutable_mavlink()->set_connected(false);
    state_.mutable_mavlink()->set_level(v1::HEALTH_LEVEL_FAILED);
  }
  PublishState();
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
      [this] { return !running_ || (heartbeat_received_ && position_received_); }) &&
         running_ && heartbeat_received_ && position_received_;
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

CommandResult ArdupilotGateway::SetMode(std::uint32_t custom_mode) {
  mavlink_message_t message{};
  mavlink_msg_set_mode_pack(kSourceSystem, kSourceComponent, &message,
                            target_system_, MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
                            custom_mode);
  if (!SendMessage(&message)) {
    return {false, false, MAV_RESULT_FAILED, "failed to send mode request"};
  }
  std::unique_lock<std::mutex> lock(state_mutex_);
  const bool changed = state_changed_.wait_for(
      lock, std::chrono::seconds(10), [this, custom_mode] {
        return !running_ || custom_mode_ == custom_mode;
      });
  if (!changed || !running_) {
    return {false, true, MAV_RESULT_FAILED, "mode confirmation timed out"};
  }
  return {true, false, MAV_RESULT_ACCEPTED, "mode confirmed"};
}

bool ArdupilotGateway::GotoGlobal(const v1::GeoPosition& position) {
  mavlink_message_t message{};
  std::uint8_t frame = MAV_FRAME_GLOBAL_RELATIVE_ALT_INT;
  if (position.frame() == v1::COORDINATE_FRAME_WGS84_AMSL) {
    frame = MAV_FRAME_GLOBAL_INT;
  } else if (position.frame() ==
             v1::COORDINATE_FRAME_WGS84_ABOVE_TERRAIN) {
    frame = MAV_FRAME_GLOBAL_TERRAIN_ALT_INT;
  }
  constexpr std::uint16_t mask =
      POSITION_TARGET_TYPEMASK_VX_IGNORE | POSITION_TARGET_TYPEMASK_VY_IGNORE |
      POSITION_TARGET_TYPEMASK_VZ_IGNORE | POSITION_TARGET_TYPEMASK_AX_IGNORE |
      POSITION_TARGET_TYPEMASK_AY_IGNORE | POSITION_TARGET_TYPEMASK_AZ_IGNORE |
      POSITION_TARGET_TYPEMASK_YAW_IGNORE |
      POSITION_TARGET_TYPEMASK_YAW_RATE_IGNORE;
  mavlink_msg_set_position_target_global_int_pack(
      kSourceSystem, kSourceComponent, &message, 0, target_system_,
      target_component_, frame, mask,
      static_cast<std::int32_t>(std::llround(position.latitude_deg() * 1e7)),
      static_cast<std::int32_t>(std::llround(position.longitude_deg() * 1e7)),
      static_cast<float>(position.altitude_m()), 0, 0, 0, 0, 0, 0, 0, 0);
  return SendMessage(&message);
}

bool ArdupilotGateway::GotoLocal(const v1::LocalPositionNed& position) {
  mavlink_message_t message{};
  constexpr std::uint16_t mask =
      POSITION_TARGET_TYPEMASK_VX_IGNORE | POSITION_TARGET_TYPEMASK_VY_IGNORE |
      POSITION_TARGET_TYPEMASK_VZ_IGNORE | POSITION_TARGET_TYPEMASK_AX_IGNORE |
      POSITION_TARGET_TYPEMASK_AY_IGNORE | POSITION_TARGET_TYPEMASK_AZ_IGNORE |
      POSITION_TARGET_TYPEMASK_YAW_IGNORE |
      POSITION_TARGET_TYPEMASK_YAW_RATE_IGNORE;
  mavlink_msg_set_position_target_local_ned_pack(
      kSourceSystem, kSourceComponent, &message, 0, target_system_,
      target_component_, MAV_FRAME_LOCAL_NED, mask,
      static_cast<float>(position.north_m()),
      static_cast<float>(position.east_m()),
      static_cast<float>(position.down_m()), 0, 0, 0, 0, 0, 0, 0, 0);
  return SendMessage(&message);
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
        if (heartbeat.type == MAV_TYPE_GCS) break;
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
        custom_mode_ = heartbeat.custom_mode;
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
        state_.mutable_velocity_ned_mps()->set_x(position.vx / 100.0);
        state_.mutable_velocity_ned_mps()->set_y(position.vy / 100.0);
        state_.mutable_velocity_ned_mps()->set_z(position.vz / 100.0);
        state_.set_ground_speed_mps(std::hypot(position.vx, position.vy) / 100.0);
        position_received_ = true;
        state_.mutable_estimator()->set_vertical_position_valid(true);
        publish = true;
        break;
      }
      case MAVLINK_MSG_ID_ATTITUDE: {
        mavlink_attitude_t attitude{};
        mavlink_msg_attitude_decode(&message, &attitude);
        state_.mutable_attitude()->set_roll_rad(attitude.roll);
        state_.mutable_attitude()->set_pitch_rad(attitude.pitch);
        state_.mutable_attitude()->set_yaw_rad(attitude.yaw);
        state_.mutable_estimator()->set_attitude_valid(true);
        if (state_.estimator().horizontal_position_valid()) {
          state_.mutable_estimator()->set_level(v1::HEALTH_LEVEL_HEALTHY);
        }
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
        local->set_origin_id("ekf-origin");
        publish = true;
        break;
      }
      case MAVLINK_MSG_ID_SYS_STATUS: {
        mavlink_sys_status_t status{};
        mavlink_msg_sys_status_decode(&message, &status);
        auto* battery = state_.mutable_battery();
        battery->set_voltage_v(status.voltage_battery / 1000.0);
        battery->set_current_a(status.current_battery / 100.0);
        const double simulated = simulation_battery_percent_.load();
        const double remaining = simulated >= 0.0
                                     ? simulated
                                     : static_cast<double>(status.battery_remaining);
        if (remaining >= 0.0) {
          battery->set_remaining_percent(remaining);
        }
        battery->set_level(remaining >= 20.0
                               ? v1::HEALTH_LEVEL_HEALTHY
                               : v1::HEALTH_LEVEL_DEGRADED);
        publish = true;
        break;
      }
      case MAVLINK_MSG_ID_GPS_RAW_INT: {
        mavlink_gps_raw_int_t gps{};
        mavlink_msg_gps_raw_int_decode(&message, &gps);
        auto* estimator = state_.mutable_estimator();
        estimator->set_horizontal_position_valid(gps.fix_type >= GPS_FIX_TYPE_3D_FIX);
        estimator->set_absolute_position_valid(gps.fix_type >= GPS_FIX_TYPE_3D_FIX);
        estimator->set_primary_horizontal_source("GNSS");
        estimator->set_horizontal_uncertainty_m(
            gps.eph == std::numeric_limits<std::uint16_t>::max()
                ? 0.0
                : gps.eph / 100.0);
        if (gps.fix_type >= GPS_FIX_TYPE_3D_FIX && estimator->attitude_valid()) {
          estimator->set_level(v1::HEALTH_LEVEL_HEALTHY);
        }
        publish = true;
        break;
      }
      case MAVLINK_MSG_ID_EKF_STATUS_REPORT: {
        mavlink_ekf_status_report_t ekf{};
        mavlink_msg_ekf_status_report_decode(&message, &ekf);
        auto* estimator = state_.mutable_estimator();
        const bool attitude = (ekf.flags & EKF_ATTITUDE) != 0;
        const bool horizontal =
            (ekf.flags & (EKF_POS_HORIZ_ABS | EKF_POS_HORIZ_REL)) != 0;
        estimator->set_attitude_valid(attitude);
        estimator->set_horizontal_position_valid(horizontal);
        estimator->set_vertical_position_valid((ekf.flags & EKF_POS_VERT_ABS) != 0);
        estimator->set_level(attitude && horizontal
                                 ? v1::HEALTH_LEVEL_HEALTHY
                                 : v1::HEALTH_LEVEL_DEGRADED);
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
    mavlink_message_t landed_request{};
    mavlink_msg_command_long_pack(
        kSourceSystem, kSourceComponent, &landed_request, target_system_,
        target_component_, MAV_CMD_SET_MESSAGE_INTERVAL, 0,
        static_cast<float>(MAVLINK_MSG_ID_EXTENDED_SYS_STATE), 200'000, 0, 0, 0,
        0, 0);
    SendMessage(&landed_request);
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

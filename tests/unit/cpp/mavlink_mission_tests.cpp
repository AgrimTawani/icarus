#include "mavlink_gateway/mavlink_gateway.hpp"
#include <arpa/inet.h>
#include <future>
#include <iostream>
#include <stdexcept>
#include <sys/socket.h>
#include <thread>
#include <unistd.h>
extern "C" {
#include <ardupilotmega/mavlink.h>
}
using namespace icarus::mavlink_gateway;
void Check(bool value, const char *message) {
  if (!value)
    throw std::runtime_error(message);
}
struct Peer {
  int listener = -1, fd = -1;
  ArdupilotGateway gateway;
  Peer() {
    listener = socket(AF_INET, SOCK_STREAM, 0);
    sockaddr_in a{};
    a.sin_family = AF_INET;
    a.sin_addr.s_addr = htonl(INADDR_LOOPBACK);
    Check(bind(listener, reinterpret_cast<sockaddr *>(&a), sizeof(a)) == 0,
          "bind");
    Check(listen(listener, 1) == 0, "listen");
    socklen_t size = sizeof(a);
    getsockname(listener, reinterpret_cast<sockaddr *>(&a), &size);
    Check(gateway.Connect("127.0.0.1", ntohs(a.sin_port)), "connect");
    fd = accept(listener, nullptr, nullptr);
    timeval timeout{2, 0};
    setsockopt(fd, SOL_SOCKET, SO_RCVTIMEO, &timeout, sizeof(timeout));
    Heartbeat(4);
    std::this_thread::sleep_for(std::chrono::milliseconds(30));
  }
  ~Peer() {
    gateway.Disconnect();
    if (fd >= 0)
      close(fd);
    close(listener);
  }
  void Send(mavlink_message_t &m) {
    uint8_t b[MAVLINK_MAX_PACKET_LEN];
    const auto n = mavlink_msg_to_send_buffer(b, &m);
    Check(send(fd, b, n, MSG_NOSIGNAL) == n, "send");
  }
  void Heartbeat(uint32_t mode) {
    mavlink_message_t m{};
    mavlink_msg_heartbeat_pack(1, 1, &m, MAV_TYPE_QUADROTOR,
                               MAV_AUTOPILOT_ARDUPILOTMEGA, 0, mode,
                               MAV_STATE_ACTIVE);
    Send(m);
  }
  mavlink_message_t Receive(uint32_t wanted) {
    mavlink_message_t m{};
    mavlink_status_t status{};
    uint8_t c;
    while (recv(fd, &c, 1, 0) == 1) {
      if (mavlink_parse_char(MAVLINK_COMM_1, c, &m, &status) &&
          m.msgid == wanted)
        return m;
    }
    throw std::runtime_error("receive timeout");
  }
  void Request(uint16_t seq, uint8_t sys = 1) {
    mavlink_message_t m{};
    mavlink_msg_mission_request_int_pack(sys, 1, &m, 255,
                                         MAV_COMP_ID_MISSIONPLANNER, seq,
                                         MAV_MISSION_TYPE_MISSION);
    Send(m);
  }
  void Ack(uint8_t result) {
    mavlink_message_t m{};
    mavlink_msg_mission_ack_pack(1, 1, &m, 255, MAV_COMP_ID_MISSIONPLANNER,
                                 result, MAV_MISSION_TYPE_MISSION);
    Send(m);
  }
  void Reached(uint16_t seq) {
    mavlink_message_t m{};
    mavlink_msg_mission_item_reached_pack(1, 1, &m, seq);
    Send(m);
  }
};
std::vector<MissionItem> Items() {
  std::vector<MissionItem> v(3);
  for (auto &i : v) {
    i.x = 473977420;
    i.y = 85455940;
    i.z = 4;
  }
  return v;
}
void SuccessfulTransfer() {
  Peer p;
  auto items = Items();
  auto task = std::async(std::launch::async,
                         [&] { return p.gateway.UploadMission(items, 2000); });
  p.Receive(MAVLINK_MSG_ID_MISSION_COUNT);
  // Wrong vehicle must not control this transaction; valid out-of-order and
  // duplicate requests still produce the requested integer-coordinate item.
  p.Request(999, 2);
  for (int seq : {2, 0, 0, 1}) {
    p.Request(seq);
    auto m = p.Receive(MAVLINK_MSG_ID_MISSION_ITEM_INT);
    mavlink_mission_item_int_t item{};
    mavlink_msg_mission_item_int_decode(&m, &item);
    Check(item.seq == seq && item.x == items[seq].x,
          "request index and integer precision");
  }
  p.Ack(MAV_MISSION_ACCEPTED);
  Check(task.get().accepted, "complete upload accepted");
  p.Reached(2);
  std::this_thread::sleep_for(std::chrono::milliseconds(20));
  Check(p.gateway.GetMissionProgress().reached_seq == -1,
        "stale reached before start ignored");
  auto start =
      std::async(std::launch::async, [&] { return p.gateway.StartMission(1); });
  p.Receive(MAVLINK_MSG_ID_MISSION_SET_CURRENT);
  mavlink_message_t current{};
  mavlink_msg_mission_current_pack(1, 1, &current, 1, 3, 0, 0);
  p.Send(current);
  p.Receive(MAVLINK_MSG_ID_SET_MODE);
  p.Heartbeat(3);
  const auto started = start.get();
  Check(started.accepted && started.activation_attempted,
        "AUTO confirmed and owned");
  Check(p.gateway.GetMissionProgress().custom_mode == 3,
        "reported current mode is AUTO");
  p.Reached(2);
  for (int tries = 0;
       tries < 100 && p.gateway.GetMissionProgress().reached_seq != 2; ++tries)
    std::this_thread::sleep_for(std::chrono::milliseconds(5));
  Check(p.gateway.GetMissionProgress().reached_seq == 2,
        "mission progress received");
  auto advance = std::async(std::launch::async,
                            [&] { return p.gateway.SetMissionCurrent(2); });
  p.Receive(MAVLINK_MSG_ID_MISSION_SET_CURRENT);
  mavlink_msg_mission_current_pack(1, 1, &current, 2, 3, 0, 0);
  p.Send(current);
  Check(advance.get().accepted,
        "checkpoint advancement confirmed without AUTO restart");
  Check(!p.gateway.UploadMission(items, 100).accepted,
        "cannot replace mission in AUTO");
  auto rtl =
      std::async(std::launch::async, [&] { return p.gateway.SetMode(6); });
  p.Receive(MAVLINK_MSG_ID_SET_MODE);
  p.Heartbeat(6);
  Check(rtl.get().accepted, "external RTL owns mode");
  p.gateway.ReleaseMissionControl(started.mode_generation);
  Check(p.gateway.GetMissionProgress().custom_mode == 6,
        "old AUTO cleanup cannot override RTL");
}
void LegacyRequestPreservesIntegerPrecision() {
  Peer p;
  auto items = Items();
  for (auto &item : items)
    item.y = 1491652375;
  auto task = std::async(std::launch::async,
                         [&] { return p.gateway.UploadMission(items, 2000); });
  p.Receive(MAVLINK_MSG_ID_MISSION_COUNT);
  for (std::uint16_t seq = 0; seq < items.size(); ++seq) {
    mavlink_message_t request{};
    mavlink_msg_mission_request_pack(1, 1, &request, 255,
                                     MAV_COMP_ID_MISSIONPLANNER, seq,
                                     MAV_MISSION_TYPE_MISSION);
    p.Send(request);
    auto received = p.Receive(MAVLINK_MSG_ID_MISSION_ITEM_INT);
    mavlink_mission_item_int_t decoded{};
    mavlink_msg_mission_item_int_decode(&received, &decoded);
    Check(decoded.seq == seq && decoded.x == items[seq].x &&
              decoded.y == 1491652375,
          "legacy request retains exact integer geocoordinates without float "
          "downgrade");
  }
  p.Ack(MAV_MISSION_ACCEPTED);
  Check(task.get().accepted, "legacy-request INT upload accepted");
}

void RejectedTransfers() {
  for (int scenario = 0; scenario < 4; ++scenario) {
    Peer p;
    auto task = std::async(std::launch::async, [&] {
      return p.gateway.UploadMission(Items(), 150);
    });
    p.Receive(MAVLINK_MSG_ID_MISSION_COUNT);
    if (scenario == 0)
      p.Ack(MAV_MISSION_ACCEPTED); // Premature ACK.
    if (scenario == 1) {
      p.Request(99);
      p.Receive(MAVLINK_MSG_ID_MISSION_ACK);
    }
    if (scenario == 2)
      p.Ack(MAV_MISSION_NO_SPACE);
    const auto result = task.get();
    Check(!result.accepted, "invalid transfer rejected");
    Check(!p.gateway.StartMission().accepted, "failed upload cannot start");
    if (scenario == 3)
      Check(result.timed_out, "timeout bounded");
  }
}
void RetriesAndNewUploadReset() {
  Peer p;
  auto task = std::async(std::launch::async, [&] {
    return p.gateway.UploadMission(Items(), 2000);
  });
  p.Receive(MAVLINK_MSG_ID_MISSION_COUNT);
  p.Receive(MAVLINK_MSG_ID_MISSION_COUNT); // Initial request was lost.
  p.Request(0);
  p.Receive(MAVLINK_MSG_ID_MISSION_ITEM_INT);
  p.Receive(MAVLINK_MSG_ID_MISSION_ITEM_INT); // Next request was lost.
  for (int seq : {1, 2}) {
    p.Request(seq);
    p.Receive(MAVLINK_MSG_ID_MISSION_ITEM_INT);
  }
  p.Ack(MAV_MISSION_ACCEPTED);
  Check(task.get().accepted, "loss retries finish transfer");
  auto replacement = std::async(std::launch::async, [&] {
    return p.gateway.UploadMission(Items(), 500);
  });
  p.Receive(MAVLINK_MSG_ID_MISSION_COUNT);
  p.Reached(2);
  p.Ack(MAV_MISSION_DENIED);
  Check(!replacement.get().accepted, "replacement failure reported");
  Check(p.gateway.GetMissionProgress().reached_seq == -1,
        "replacement does not inherit completed telemetry");
  Check(!p.gateway.StartMission().accepted,
        "failed replacement cannot start old mission");
}

void CancelDuringSelectionCannotRestoreAuto() {
  for (bool change_mode : {false, true}) {
    Peer p;
    auto upload = std::async(std::launch::async, [&] {
      return p.gateway.UploadMission(Items(), 2000);
    });
    p.Receive(MAVLINK_MSG_ID_MISSION_COUNT);
    for (int seq : {0, 1, 2}) {
      p.Request(seq);
      p.Receive(MAVLINK_MSG_ID_MISSION_ITEM_INT);
    }
    p.Ack(MAV_MISSION_ACCEPTED);
    Check(upload.get().accepted, "upload before cancellation");
    std::atomic<bool> authorized{true};
    auto start = std::async(std::launch::async, [&] {
      return p.gateway.StartMissionGuarded(1,
                                           [&] { return authorized.load(); });
    });
    p.Receive(MAVLINK_MSG_ID_MISSION_SET_CURRENT);
    if (change_mode) {
      auto rtl =
          std::async(std::launch::async, [&] { return p.gateway.SetMode(6); });
      const auto wire = p.Receive(MAVLINK_MSG_ID_SET_MODE);
      mavlink_set_mode_t mode{};
      mavlink_msg_set_mode_decode(&wire, &mode);
      Check(mode.custom_mode == 6, "RTL packet sent while selection waiting");
      p.Heartbeat(6);
      Check(rtl.get().accepted, "RTL confirmed");
    } else
      authorized = false;
    mavlink_message_t current{};
    mavlink_msg_mission_current_pack(1, 1, &current, 1, 3, 0, 0);
    p.Send(current);
    const auto result = start.get();
    Check(!result.accepted && !result.activation_attempted,
          "late selection ACK cannot activate cancelled or superseded mission");
    Check(p.gateway.GetMissionProgress().custom_mode == (change_mode ? 6U : 4U),
          "cancelled start preserves latest mode");
  }
}

void DisconnectedTransfer() {
  Peer p;
  auto task = std::async(std::launch::async, [&] {
    return p.gateway.UploadMission(Items(), 2000);
  });
  p.Receive(MAVLINK_MSG_ID_MISSION_COUNT);
  shutdown(p.fd, SHUT_RDWR);
  Check(!task.get().accepted, "disconnect aborts upload");
}
void RawTelemetryCannotOverwriteFusedEstimatorHealth() {
  Peer p;
  int iteration = 0;
  // Initialized actual position, attitude-only, predicted-only, and explicitly
  // uninitialized reports, each followed by raw telemetry which used to win.
  for (const uint16_t flags : {uint16_t(EKF_ATTITUDE | EKF_POS_HORIZ_ABS | EKF_POS_VERT_ABS),
                              uint16_t(EKF_ATTITUDE),
                              uint16_t(EKF_ATTITUDE | EKF_PRED_POS_HORIZ_ABS | EKF_POS_VERT_ABS),
                              uint16_t(EKF_ATTITUDE | EKF_POS_HORIZ_ABS | EKF_POS_VERT_ABS | EKF_UNINITIALIZED)}) {
    for (const uint8_t fix : {uint8_t(3), uint8_t(1)}) {
      const float marker = static_cast<float>(++iteration) / 8.0F;
      auto observed = std::make_shared<std::promise<icarus::v1::DroneState>>();
      auto delivered = std::make_shared<std::atomic<bool>>(false);
      p.gateway.SetStateCallback([observed, delivered, marker](const icarus::v1::DroneState& state) {
        if (state.attitude().yaw_rad() == marker && !delivered->exchange(true))
          observed->set_value(state);
      });
      mavlink_message_t packet{};
      mavlink_ekf_status_report_t ekf{};
      ekf.flags = flags;
      mavlink_msg_ekf_status_report_encode(1, 1, &packet, &ekf);
      p.Send(packet);
      mavlink_gps_raw_int_t gps{};
      gps.fix_type = fix;
      mavlink_msg_gps_raw_int_encode(1, 1, &packet, &gps);
      p.Send(packet);
      mavlink_global_position_int_t global{};
      mavlink_msg_global_position_int_encode(1, 1, &packet, &global);
      p.Send(packet);
      mavlink_attitude_t attitude{};
      attitude.yaw = marker;
      mavlink_msg_attitude_encode(1, 1, &packet, &attitude);
      p.Send(packet);
      auto future = observed->get_future();
      Check(future.wait_for(std::chrono::seconds(2)) == std::future_status::ready,
            "estimator packet batch delivered");
      auto state = future.get();
      p.gateway.SetStateCallback({});
      const bool initialized = !(flags & EKF_UNINITIALIZED);
      const bool horizontal = initialized && (flags & EKF_POS_HORIZ_ABS);
      const bool vertical = initialized && (flags & EKF_POS_VERT_ABS);
      Check(state.estimator().attitude_valid() == initialized,
            "raw attitude cannot overwrite uninitialized EKF");
      Check(state.estimator().horizontal_position_valid() == horizontal,
            "raw GPS fix cannot overwrite fused horizontal validity");
      Check(state.estimator().absolute_position_valid() == horizontal,
            "raw GPS fix cannot overwrite fused absolute validity");
      Check(state.estimator().vertical_position_valid() == vertical,
            "global position packet cannot overwrite fused vertical validity");
      Check((state.estimator().level() == icarus::v1::HEALTH_LEVEL_HEALTHY) ==
                (initialized && horizontal && vertical),
            "raw telemetry cannot change fused EKF health");
      Check(state.sources_size() == 2, "EKF and GPS diagnostics remain separate and deduplicated");
    }
  }
}

void HomePositionCarriesExplicitGlobalFrame() {
  Peer p;
  auto observed = std::make_shared<std::promise<icarus::v1::DroneState>>();
  auto delivered = std::make_shared<std::atomic<bool>>(false);
  p.gateway.SetStateCallback([observed, delivered](const icarus::v1::DroneState& state) {
    if (state.has_home() && state.home().has_geo() && !delivered->exchange(true)) {
      observed->set_value(state);
    }
  });
  mavlink_home_position_t home{};
  home.latitude = -353632610;
  home.longitude = 1491652300;
  home.altitude = 584120;
  home.x = 7.0F;
  home.y = -4.0F;
  home.z = -2.0F;
  mavlink_message_t message{};
  mavlink_msg_home_position_encode(1, 1, &message, &home);
  p.Send(message);
  auto result = observed->get_future();
  Check(result.wait_for(std::chrono::seconds(2)) == std::future_status::ready,
        "home position telemetry delivered");
  auto state = result.get();
  p.gateway.SetStateCallback({});
  Check(state.home().geo().frame() == icarus::v1::COORDINATE_FRAME_WGS84_AMSL,
        "HOME_POSITION is explicitly AMSL, never unspecified");
  Check(state.home().geo().altitude_m() == 584.12,
        "HOME_POSITION altitude converts millimeters to AMSL meters");
  Check(state.home_local_position_ned().north_m() == 7.0 &&
        state.home_local_position_ned().east_m() == -4.0 &&
        state.home_local_position_ned().down_m() == -2.0 &&
        state.home_local_position_ned().origin_id() == "ekf-origin" &&
        state.home_position_received_at_unix_ms() > 0,
        "HOME_POSITION carries the native measured local home transform");
}

void OriginResetInvalidatesMeasuredHome() {
  std::mutex mutex;
  std::condition_variable changed;
  icarus::v1::DroneState state;
  Peer p;
  p.gateway.SetStateCallback([&](const auto& value) {
    std::lock_guard<std::mutex> lock(mutex);
    state = value;
    changed.notify_all();
  });
  const auto wait = [&](const auto& predicate) {
    std::unique_lock<std::mutex> lock(mutex);
    Check(changed.wait_for(lock, std::chrono::seconds(2), [&] { return predicate(state); }),
          "reference telemetry delivered");
  };
  mavlink_message_t message{};
  mavlink_gps_global_origin_t origin{};
  origin.latitude = 100;
  mavlink_msg_gps_global_origin_encode(1, 1, &message, &origin);
  p.Send(message);
  mavlink_home_position_t home{};
  home.x = 3.0F;
  mavlink_msg_home_position_encode(1, 1, &message, &home);
  p.Send(message);
  wait([](const auto& s) { return s.has_home_local_position_ned(); });
  origin.latitude = 200;
  mavlink_msg_gps_global_origin_encode(1, 1, &message, &origin);
  p.Send(message);
  wait([](const auto& s) { return !s.has_home_local_position_ned(); });
  home.x = 4.0F;
  mavlink_msg_home_position_encode(1, 1, &message, &home);
  p.Send(message);
  wait([](const auto& s) { return s.has_home_local_position_ned() &&
      s.home_local_position_ned().origin_id() == "ekf-origin-1"; });
  p.gateway.Disconnect();
  p.gateway.SetStateCallback({});
}

int main() {
  try {
    HomePositionCarriesExplicitGlobalFrame();
    OriginResetInvalidatesMeasuredHome();
    RawTelemetryCannotOverwriteFusedEstimatorHealth();
    SuccessfulTransfer();
    CancelDuringSelectionCannotRestoreAuto();
    LegacyRequestPreservesIntegerPrecision();
    RejectedTransfers();
    DisconnectedTransfer();
    RetriesAndNewUploadReset();
    std::cout << "Mission protocol tests passed\n";
    return 0;
  } catch (const std::exception &e) {
    std::cerr << e.what() << '\n';
    return 1;
  }
}

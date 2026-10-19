#pragma once

#include <atomic>
#include <array>
#include <deque>
#include <vector>
#include <condition_variable>
#include <cstdint>
#include <functional>
#include <mutex>
#include <optional>
#include <string>
#include <thread>

#include "icarus/v1/state.pb.h"

namespace icarus::mavlink_gateway {

// Decode SYS_STATUS sentinel values before publishing battery availability.
[[nodiscard]] v1::BatteryState DecodeBatteryTelemetry(
    std::uint16_t voltage_mv, std::int16_t current_centiamp,
    std::int8_t remaining_percent, std::int64_t received_at_unix_ms,
    double simulation_percent = -1.0);

struct CommandResult {
  bool accepted = false;
  bool timed_out = false;
  std::uint8_t mavlink_result = 0;
  std::string message;
  bool activation_attempted = false;
  std::uint64_t mode_generation = 0;
};

struct MissionItem {
  std::uint16_t command = 16;
  std::uint8_t frame = 3;  // GLOBAL_RELATIVE_ALT; x/y are degrees * 1e7.
  std::array<float, 4> params{};
  std::int32_t x = 0, y = 0;
  float z = 0;
};
struct MissionProgress {
  int current_seq = -1, reached_seq = -1;
  std::uint8_t mission_state = 0;
  std::uint32_t custom_mode = 0;
  std::uint64_t mode_generation = 0;
  std::uint64_t revision = 0;
  std::int64_t received_at_unix_ms = 0;
};

class MavlinkGateway {
 public:
  using StateCallback = std::function<void(const v1::DroneState&)>;
  using EventCallback = std::function<void(const v1::VehicleEvent&)>;

  virtual ~MavlinkGateway() = default;
  virtual bool Connect(const std::string& host, std::uint16_t port) = 0;
  virtual void Disconnect() = 0;
  [[nodiscard]] virtual bool IsConnected() const = 0;
  [[nodiscard]] virtual bool WaitForReady(std::uint32_t timeout_ms) = 0;
  virtual void SetStateCallback(StateCallback callback) = 0;
  virtual void SetEventCallback(EventCallback callback) = 0;
  [[nodiscard]] virtual CommandResult Arm(bool arm) = 0;
  [[nodiscard]] virtual CommandResult Takeoff(double altitude_agl_m) = 0;
  [[nodiscard]] virtual CommandResult SetMode(std::uint32_t custom_mode) = 0;
  [[nodiscard]] virtual CommandResult SetYaw(double heading_deg) = 0;
  [[nodiscard]] virtual CommandResult RotateYaw(double angle_deg,
                                                double rate_deg_s) = 0;
  // Acknowledged horizontal navigation setpoint, not a measured-speed guarantee.
  [[nodiscard]] virtual CommandResult SetGroundSpeed(double speed_mps) = 0;
  virtual bool GotoGlobal(const v1::GeoPosition& position,
                          std::optional<double> heading_deg = std::nullopt) = 0;
  virtual bool GotoLocal(const v1::LocalPositionNed& position,
                         std::optional<double> heading_deg = std::nullopt) = 0;
  virtual CommandResult UploadMission(const std::vector<MissionItem>&,
                                      std::uint32_t = 10000) {
    return {false, false, 0, "native mission upload unsupported"};
  }
  virtual CommandResult StartMission(std::uint16_t = 1) {
    return {false, false, 0, "native mission start unsupported"};
  }
  virtual CommandResult SetMissionCurrent(std::uint16_t) {
    return {false, false, 0, "native mission selection unsupported"};
  }
  virtual CommandResult StartMissionGuarded(std::uint16_t seq,
                                            const std::function<bool()>& guard) {
    if (!guard()) return {false, false, 0, "mission activation cancelled"};
    auto result = StartMission(seq);
    result.activation_attempted = result.accepted;
    result.mode_generation = GetMissionProgress().mode_generation;
    return result;
  }
  virtual CommandResult SetMissionCurrentGuarded(std::uint16_t seq,
      std::uint64_t, const std::function<bool()>& guard) {
    if (!guard()) return {false, false, 0, "mission advancement cancelled"};
    return SetMissionCurrent(seq);
  }
  virtual CommandResult SetMissionCompletionMode(std::uint32_t mode, std::uint64_t,
      const std::function<bool()>& guard) {
    if (!guard()) return {false,false,0,"mission completion cancelled"};
    return SetMode(mode);
  }
  virtual void ReleaseMissionControl(std::uint64_t) {
    if (GetMissionProgress().custom_mode == 3) (void)SetMode(17);
  }
  virtual MissionProgress GetMissionProgress() const { return {}; }
  // Continuous local trajectory: velocity is feed-forward in NED m/s and
  // yaw is absolute degrees. Unsupported backends fail explicitly.
  virtual bool StreamLocal(const v1::LocalPositionNed&,
                           double, double, double,
                           std::optional<double> = std::nullopt) { return false; }
};

class ArdupilotGateway final : public MavlinkGateway {
 public:
  explicit ArdupilotGateway(std::string vehicle_id = "icarus-01");
  ~ArdupilotGateway() override;

  bool Connect(const std::string& host, std::uint16_t port) override;
  void Disconnect() override;
  [[nodiscard]] bool IsConnected() const override;
  [[nodiscard]] bool WaitForReady(std::uint32_t timeout_ms) override;
  void SetStateCallback(StateCallback callback) override;
  void SetEventCallback(EventCallback callback) override;
  [[nodiscard]] CommandResult Arm(bool arm) override;
  [[nodiscard]] CommandResult Takeoff(double altitude_agl_m) override;
  [[nodiscard]] CommandResult SetMode(std::uint32_t custom_mode) override;
  [[nodiscard]] CommandResult SetYaw(double heading_deg) override;
  [[nodiscard]] CommandResult RotateYaw(double angle_deg,
                                        double rate_deg_s) override;
  [[nodiscard]] CommandResult SetGroundSpeed(double speed_mps) override;
  bool GotoGlobal(const v1::GeoPosition& position,
                  std::optional<double> heading_deg = std::nullopt) override;
  bool GotoLocal(const v1::LocalPositionNed& position,
                 std::optional<double> heading_deg = std::nullopt) override;

  bool StreamLocal(const v1::LocalPositionNed& position,
                   double north_mps, double east_mps, double down_mps,
                   std::optional<double> heading_deg = std::nullopt) override;

  CommandResult UploadMission(const std::vector<MissionItem>& items,
                              std::uint32_t timeout_ms = 10000) override;
  CommandResult StartMission(std::uint16_t first_seq = 1) override;
  MissionProgress GetMissionProgress() const override;
  CommandResult SetMissionCurrent(std::uint16_t seq) override;
  CommandResult StartMissionGuarded(std::uint16_t seq,
      const std::function<bool()>& guard) override;
  CommandResult SetMissionCurrentGuarded(std::uint16_t seq,
      std::uint64_t generation, const std::function<bool()>& guard) override;
  void ReleaseMissionControl(std::uint64_t generation) override;
  CommandResult SetMissionCompletionMode(std::uint32_t mode, std::uint64_t generation,
      const std::function<bool()>& guard) override;

  // Simulator-test only. This emulates a bidirectional MAVLink control-path
  // loss or latency window; production launchers never pass these options.
  void ConfigureTestLinkFault(std::string mode, std::uint32_t start_after_ms,
                              std::uint32_t duration_ms,
                              std::uint32_t latency_ms = 0);
  // Legacy test-only SOC override. Never substitutes for missing/invalid
  // voltage, current, or percentage telemetry from the measured source.
  void ConfigureSimulationBatteryPercent(double percent);

 private:
  CommandResult SelectMissionCurrent(std::uint16_t seq,
      std::optional<std::uint64_t> generation = std::nullopt,
      const std::function<bool()>& guard = [] { return true; });
  CommandResult SetModeGuarded(std::uint32_t mode,
      std::optional<std::uint64_t> generation,
      const std::function<bool()>& guard);
  void ReaderLoop();
  void HeartbeatLoop();
  void HandleMessage(const void* message);
  bool SendMessage(const void* message);
  [[nodiscard]] CommandResult SendCommand(std::uint16_t command,
                                          const float params[7]);
  void PublishState();
  void PublishEvent(v1::EventSeverity severity, std::string code,
                    std::string message);
  [[nodiscard]] static std::int64_t NowUnixMs();
  [[nodiscard]] bool TestLinkLossActive() const;
  [[nodiscard]] std::uint32_t TestLinkDelayMs() const;

  std::string vehicle_id_;
  std::atomic<int> socket_{-1};
  std::atomic<bool> running_{false};
  std::thread reader_thread_;
  std::thread heartbeat_thread_;
  mutable std::mutex send_mutex_;
  mutable std::mutex state_mutex_;
  std::condition_variable state_changed_;
  v1::DroneState state_;
  std::optional<std::array<std::int32_t, 3>> global_origin_;
  std::uint64_t origin_revision_ = 0;
  std::string local_origin_id_ = "ekf-origin";
  bool heartbeat_received_ = false;
  bool position_received_ = false;
  bool local_position_received_ = false;
  std::uint32_t custom_mode_ = 0;
  std::uint8_t target_system_ = 0;
  std::uint8_t target_component_ = 0;
  StateCallback state_callback_;
  EventCallback event_callback_;
  // Upload/start are serialized independently of telemetry delivery.
  std::mutex mode_command_mutex_;  // Packet ordering only, never ACK waits.
  std::atomic<std::uint64_t> mode_generation_{0};
  std::uint64_t mission_mode_generation_ = 0;
  std::optional<std::uint32_t> requested_mode_;  // state_mutex_
  std::mutex mission_operation_mutex_;
  mutable std::mutex mission_mutex_;
  std::condition_variable mission_changed_;
  std::deque<std::uint16_t> mission_requests_;
  std::optional<std::uint8_t> mission_ack_;
  bool mission_uploading_ = false;
  bool mission_ready_ = false;
  bool mission_started_ = false;
  std::size_t mission_count_ = 0;
  MissionProgress mission_progress_;
  std::mutex ack_mutex_;
  std::condition_variable ack_received_;
  std::optional<std::pair<std::uint16_t, std::uint8_t>> latest_ack_;
  std::atomic<std::int64_t> test_fault_start_unix_ms_{-1};
  std::atomic<std::uint32_t> test_fault_duration_ms_{0};
  std::atomic<std::uint32_t> test_fault_latency_ms_{0};
  std::atomic<bool> test_fault_is_loss_{false};
  std::atomic<double> simulation_battery_percent_{-1.0};
};

}  // namespace icarus::mavlink_gateway

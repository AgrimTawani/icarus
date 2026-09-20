#pragma once

#include <atomic>
#include <condition_variable>
#include <cstdint>
#include <functional>
#include <mutex>
#include <optional>
#include <string>
#include <thread>

#include "icarus/v1/state.pb.h"

namespace icarus::mavlink_gateway {

struct CommandResult {
  bool accepted = false;
  bool timed_out = false;
  std::uint8_t mavlink_result = 0;
  std::string message;
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
  virtual bool GotoGlobal(const v1::GeoPosition& position) = 0;
  virtual bool GotoLocal(const v1::LocalPositionNed& position) = 0;
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
  bool GotoGlobal(const v1::GeoPosition& position) override;
  bool GotoLocal(const v1::LocalPositionNed& position) override;

  // Simulator-test only. This emulates a bidirectional MAVLink control-path
  // loss or latency window; production launchers never pass these options.
  void ConfigureTestLinkFault(std::string mode, std::uint32_t start_after_ms,
                              std::uint32_t duration_ms,
                              std::uint32_t latency_ms = 0);

 private:
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
  bool heartbeat_received_ = false;
  bool position_received_ = false;
  std::uint32_t custom_mode_ = 0;
  std::uint8_t target_system_ = 0;
  std::uint8_t target_component_ = 0;
  StateCallback state_callback_;
  EventCallback event_callback_;
  std::mutex ack_mutex_;
  std::condition_variable ack_received_;
  std::optional<std::pair<std::uint16_t, std::uint8_t>> latest_ack_;
  std::atomic<std::int64_t> test_fault_start_unix_ms_{-1};
  std::atomic<std::uint32_t> test_fault_duration_ms_{0};
  std::atomic<std::uint32_t> test_fault_latency_ms_{0};
  std::atomic<bool> test_fault_is_loss_{false};
};

}  // namespace icarus::mavlink_gateway

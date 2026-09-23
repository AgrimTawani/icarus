#pragma once

#include <atomic>
#include <cstddef>
#include <cstdint>
#include <mutex>
#include <string>
#include <thread>

#include "perception/sensor_types.hpp"

namespace icarus::perception {

// Receives the deliberately small, sensor-only UDP protocol emitted by
// scripts/simulation/airsim/sensor_bridge.py. It does not speak AirSim RPC and
// therefore cannot issue simulator control commands.
class AirSimLidarAdapter final : public RangeSource {
 public:
  explicit AirSimLidarAdapter(std::string bind_address = "127.0.0.1",
                              std::uint16_t port = 41452);
  ~AirSimLidarAdapter() override;
  bool Start(Callback callback) override;
  void Stop() override;
  [[nodiscard]] bool Healthy(std::uint32_t maximum_age_ms) const override;

  [[nodiscard]] static bool DecodePacket(const std::uint8_t* data,
                                         std::size_t size,
                                         RangeScan* scan);

 private:
  void ReceiveLoop();
  [[nodiscard]] static std::int64_t NowMs();

  std::string bind_address_;
  std::uint16_t port_;
  int socket_ = -1;
  std::thread receive_thread_;
  mutable std::mutex callback_mutex_;
  Callback callback_;
  std::atomic<bool> running_{false};
  std::atomic<std::int64_t> last_sample_ms_{0};
};

}  // namespace icarus::perception

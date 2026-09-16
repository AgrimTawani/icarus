#pragma once

#include <atomic>
#include <cstdint>
#include <mutex>
#include <string>

#include <gz/msgs/laserscan.pb.h>
#include <gz/msgs/stringmsg.pb.h>
#include <gz/transport/Node.hh>

#include "perception/sensor_types.hpp"

namespace icarus::perception {

class GazeboLidarAdapter final : public RangeSource {
 public:
  explicit GazeboLidarAdapter(
      std::string topic = "/icarus/sensors/lidar");
  ~GazeboLidarAdapter() override;
  bool Start(Callback callback) override;
  void Stop() override;
  [[nodiscard]] bool Healthy(std::uint32_t maximum_age_ms) const override;

 private:
  void OnScan(const gz::msgs::LaserScan& message);
  void OnFault(const gz::msgs::StringMsg& message);
  [[nodiscard]] static std::int64_t NowMs();

  std::string topic_;
  gz::transport::Node node_;
  mutable std::mutex callback_mutex_;
  Callback callback_;
  std::atomic<bool> running_{false};
  std::atomic<std::int64_t> last_sample_ms_{0};
  std::atomic<std::uint64_t> sequence_{0};
  std::atomic<bool> drop_injected_{false};
};

}  // namespace icarus::perception

#include "perception/lidar/gazebo_lidar_adapter.hpp"

#include <chrono>
#include <cmath>
#include <limits>
#include <utility>

namespace icarus::perception {

GazeboLidarAdapter::GazeboLidarAdapter(std::string topic)
    : topic_(std::move(topic)) {}

GazeboLidarAdapter::~GazeboLidarAdapter() { Stop(); }

bool GazeboLidarAdapter::Start(Callback callback) {
  if (running_.exchange(true)) return true;
  {
    std::lock_guard<std::mutex> lock(callback_mutex_);
    callback_ = std::move(callback);
  }
  if (!node_.Subscribe(topic_, &GazeboLidarAdapter::OnScan, this)) {
    running_ = false;
    return false;
  }
  if (!node_.Subscribe("/icarus/test/sensor_fault",
                       &GazeboLidarAdapter::OnFault, this)) {
    node_.Unsubscribe(topic_);
    running_ = false;
    return false;
  }
  return true;
}

void GazeboLidarAdapter::Stop() {
  if (!running_.exchange(false)) return;
  node_.Unsubscribe(topic_);
  node_.Unsubscribe("/icarus/test/sensor_fault");
  std::lock_guard<std::mutex> lock(callback_mutex_);
  callback_ = nullptr;
}

bool GazeboLidarAdapter::Healthy(std::uint32_t maximum_age_ms) const {
  const auto last = last_sample_ms_.load();
  return running_ && last != 0 && NowMs() - last <= maximum_age_ms;
}

void GazeboLidarAdapter::OnScan(const gz::msgs::LaserScan& message) {
  if (!running_ || drop_injected_) return;
  RangeScan scan;
  scan.sensor_id = "gazebo-lidar";
  scan.frame = SensorFrame::kBodyFlu;
  scan.captured_at_unix_ms = NowMs();
  scan.received_at_unix_ms = scan.captured_at_unix_ms;
  scan.sequence = ++sequence_;
  scan.minimum_range_m = message.range_min();
  scan.maximum_range_m = message.range_max();
  const int horizontal = std::max(1, static_cast<int>(message.count()));
  const int vertical =
      std::max(1, static_cast<int>(message.vertical_count()));
  scan.points.reserve(message.ranges_size());
  for (int v = 0; v < vertical; ++v) {
    const double elevation = message.vertical_angle_min() +
                             v * message.vertical_angle_step();
    for (int h = 0; h < horizontal; ++h) {
      const int sample = v * horizontal + h;
      if (sample >= message.ranges_size()) break;
      const double range = message.ranges(sample);
      // Ignore the airframe/propeller envelope; physical adapters apply the
      // same calibrated self-mask before producing a normalized scan.
      if (!std::isfinite(range) || range < std::max(1.0, message.range_min()) ||
          range >= message.range_max() * 0.995) continue;
      const double azimuth = message.angle_min() + h * message.angle_step();
      const double horizontal_range = range * std::cos(elevation);
      scan.points.push_back({horizontal_range * std::cos(azimuth),
                             horizontal_range * std::sin(azimuth),
                             range * std::sin(elevation), 1.0});
    }
  }
  last_sample_ms_ = scan.captured_at_unix_ms;
  Callback callback;
  {
    std::lock_guard<std::mutex> lock(callback_mutex_);
    callback = callback_;
  }
  if (callback) callback(scan);
}

void GazeboLidarAdapter::OnFault(const gz::msgs::StringMsg& message) {
  const auto& payload = message.data();
  if (payload.find("lidar") == std::string::npos) return;
  if (payload.find("drop") != std::string::npos) drop_injected_ = true;
  if (payload.find("normal") != std::string::npos) drop_injected_ = false;
}

std::int64_t GazeboLidarAdapter::NowMs() {
  return std::chrono::duration_cast<std::chrono::milliseconds>(
             std::chrono::system_clock::now().time_since_epoch())
      .count();
}

}  // namespace icarus::perception

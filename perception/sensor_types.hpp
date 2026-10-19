#pragma once

#include <cstdint>
#include <functional>
#include <limits>
#include <string>
#include <vector>

namespace icarus::perception {

struct Point3 {
  double x_m = 0.0;
  double y_m = 0.0;
  double z_m = 0.0;
  double confidence = 1.0;
};

enum class SensorFrame { kBodyFlu, kBodyFrd, kLocalNed, kCameraOptical };

struct RangeScan {
  std::string sensor_id;
  std::int64_t captured_at_unix_ms = 0;
  std::int64_t received_at_unix_ms = 0;
  std::uint64_t sequence = 0;
  SensorFrame frame = SensorFrame::kBodyFlu;
  double minimum_range_m = 0.0;
  double maximum_range_m = 0.0;
  // Highest beam elevation of the sensor in its own frame. NaN when the
  // adapter cannot state it; cluster tops are then reported as clipped.
  double maximum_elevation_rad = std::numeric_limits<double>::quiet_NaN();
  std::vector<Point3> points;
};

struct ImageFrame {
  std::string sensor_id;
  std::int64_t captured_at_unix_ms = 0;
  std::uint64_t sequence = 0;
  SensorFrame frame = SensorFrame::kCameraOptical;
  std::uint32_t width = 0;
  std::uint32_t height = 0;
  std::string encoding;
  const void* data = nullptr;
  std::size_t size = 0;
};

class RangeSource {
 public:
  using Callback = std::function<void(const RangeScan&)>;
  virtual ~RangeSource() = default;
  virtual bool Start(Callback callback) = 0;
  virtual void Stop() = 0;
  [[nodiscard]] virtual bool Healthy(std::uint32_t maximum_age_ms) const = 0;
};

class ImageSource {
 public:
  using Callback = std::function<void(const ImageFrame&)>;
  virtual ~ImageSource() = default;
  virtual bool Start(Callback callback) = 0;
  virtual void Stop() = 0;
  [[nodiscard]] virtual bool Healthy(std::uint32_t maximum_age_ms) const = 0;
};

}  // namespace icarus::perception

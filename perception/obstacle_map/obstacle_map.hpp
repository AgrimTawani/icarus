#pragma once

#include <cstdint>
#include <mutex>
#include <vector>

#include "icarus/v1/perception.pb.h"
#include "perception/sensor_types.hpp"

namespace icarus::perception {

class ObstacleMap {
 public:
  explicit ObstacleMap(std::uint32_t expiry_ms = 750);
  void Ingest(const RangeScan& scan, double vehicle_north_m,
              double vehicle_east_m, double vehicle_down_m,
              double vehicle_yaw_rad);
  [[nodiscard]] std::vector<Point3> Points(std::int64_t now_ms) const;
  [[nodiscard]] v1::PerceptionSummary Summary(
      std::int64_t now_ms, double vehicle_north_m, double vehicle_east_m,
      double vehicle_yaw_rad) const;
  [[nodiscard]] bool Fresh(std::int64_t now_ms) const;

 private:
  mutable std::mutex mutex_;
  std::vector<Point3> points_ned_;
  std::int64_t observed_at_ms_ = 0;
  std::uint64_t sequence_ = 0;
  std::uint32_t expiry_ms_;
};

}  // namespace icarus::perception

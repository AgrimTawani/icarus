#include "perception/obstacle_map/obstacle_map.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

namespace icarus::perception {
namespace {
constexpr double kPi = 3.14159265358979323846;
// The frontal map is deliberately a 2D collision slice.  Returns below this
// slice are terrain / landing-clearance observations, not obstacles.  Keeping
// the tolerance small rejects the Gazebo sensor's -3 degree ring, which hits
// level ground about 2.85 m away when the vehicle is landed, while retaining
// the horizontal ring and obstacles extending to the vehicle's height.
constexpr double kMaximumDownwardReturnM = 0.05;
}

ObstacleMap::ObstacleMap(std::uint32_t expiry_ms) : expiry_ms_(expiry_ms) {}

void ObstacleMap::Ingest(const RangeScan& scan, double north, double east,
                         double down, double roll, double pitch, double yaw) {
  std::vector<Point3> transformed;
  transformed.reserve(scan.points.size());
  for (const auto& point : scan.points) {
    Point3 local;
    if (scan.frame == SensorFrame::kLocalNed) {
      local = point;
    } else {
      const double forward = point.x_m;
      const double right = scan.frame == SensorFrame::kBodyFlu
                               ? -point.y_m
                               : point.y_m;
      const double body_down = scan.frame == SensorFrame::kBodyFlu
                                   ? -point.z_m
                                   : point.z_m;
      // Rotate body FRD into local NED in roll-pitch-yaw order. Ignoring pitch
      // makes a level LiDAR beam look level in the map even when the aircraft
      // noses down and the beam actually strikes terrain during takeoff.
      const double rolled_right = std::cos(roll) * right -
                                  std::sin(roll) * body_down;
      const double rolled_down = std::sin(roll) * right +
                                 std::cos(roll) * body_down;
      const double pitched_forward = std::cos(pitch) * forward +
                                     std::sin(pitch) * rolled_down;
      const double pitched_down = -std::sin(pitch) * forward +
                                  std::cos(pitch) * rolled_down;
      local.x_m = north + std::cos(yaw) * pitched_forward -
                  std::sin(yaw) * rolled_right;
      local.y_m = east + std::sin(yaw) * pitched_forward +
                  std::cos(yaw) * rolled_right;
      local.z_m = down + pitched_down;
      local.confidence = point.confidence;
    }
    // Retain the forward/upward collision envelope. Downward returns belong to
    // terrain/landing clearance and must not be classified as frontal hazards.
    const double relative_down = local.z_m - down;
    if (relative_down >= -1.0 &&
        relative_down <= kMaximumDownwardReturnM) {
      transformed.push_back(local);
    }
  }
  std::lock_guard<std::mutex> lock(mutex_);
  points_ned_ = std::move(transformed);
  observed_at_ms_ = scan.captured_at_unix_ms;
  ++sequence_;
}

std::vector<Point3> ObstacleMap::Points(std::int64_t now_ms) const {
  std::lock_guard<std::mutex> lock(mutex_);
  if (observed_at_ms_ == 0 || now_ms - observed_at_ms_ > expiry_ms_) return {};
  return points_ned_;
}

bool ObstacleMap::Fresh(std::int64_t now_ms) const {
  std::lock_guard<std::mutex> lock(mutex_);
  return observed_at_ms_ != 0 && now_ms >= observed_at_ms_ &&
         now_ms - observed_at_ms_ <= expiry_ms_;
}

v1::PerceptionSummary ObstacleMap::Summary(std::int64_t now_ms, double north,
                                           double east, double yaw) const {
  v1::PerceptionSummary summary;
  std::lock_guard<std::mutex> lock(mutex_);
  summary.set_observed_at_unix_ms(observed_at_ms_);
  summary.set_sequence(sequence_);
  const auto age = observed_at_ms_ == 0 || now_ms < observed_at_ms_
                       ? std::numeric_limits<std::uint32_t>::max()
                       : static_cast<std::uint32_t>(now_ms - observed_at_ms_);
  summary.set_local_map_age_ms(age);
  const bool fresh = observed_at_ms_ != 0 && age <= expiry_ms_;
  summary.set_overall(fresh ? v1::HEALTH_LEVEL_HEALTHY
                            : v1::HEALTH_LEVEL_UNAVAILABLE);
  summary.set_local_map_available(fresh);
  double nearest = std::numeric_limits<double>::infinity();
  double bearing = 0.0;
  if (fresh) {
    for (const auto& point : points_ned_) {
      const double dn = point.x_m - north;
      const double de = point.y_m - east;
      const double distance = std::hypot(dn, de);
      if (distance < nearest) {
        nearest = distance;
        bearing = std::atan2(de, dn) * 180.0 / kPi - yaw * 180.0 / kPi;
      }
    }
  }
  if (std::isfinite(nearest)) {
    summary.set_nearest_obstacle_distance_m(nearest);
    summary.set_nearest_obstacle_bearing_deg(std::remainder(bearing, 360.0));
    summary.set_path_ahead_clear(nearest > 4.0 || std::abs(bearing) > 35.0);
  } else {
    summary.set_nearest_obstacle_distance_m(-1.0);
    summary.set_path_ahead_clear(fresh);
  }
  return summary;
}

}  // namespace icarus::perception

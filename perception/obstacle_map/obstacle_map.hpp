#pragma once

#include <cstddef>
#include <cstdint>
#include <mutex>
#include <string>
#include <vector>

#include "icarus/v1/perception.pb.h"
#include "perception/sensor_types.hpp"

namespace icarus::perception {

// One connected group of LiDAR returns on the local horizontal plane.  These
// are candidate object proposals for evidence consumers (for example target
// localization); they carry measurements only and never gate navigation, which
// continues to use the frontal collision slice.
struct ObstacleCluster {
  struct Vertex {
    double north_m = 0.0;
    double east_m = 0.0;
  };
  std::uint32_t point_count = 0;
  double centroid_north_m = 0.0;
  double centroid_east_m = 0.0;
  double centroid_down_m = 0.0;
  double min_north_m = 0.0;
  double max_north_m = 0.0;
  double min_east_m = 0.0;
  double max_east_m = 0.0;
  double top_down_m = 0.0;
  double bottom_down_m = 0.0;
  bool top_clipped = true;
  std::vector<Vertex> hull;
};

struct ClusterOptions {
  // Grid cell edge on the horizontal plane; 8-connected occupied cells join,
  // so returns up to about two cells apart merge into one object.
  double cell_m = 0.5;
  double maximum_range_m = 40.0;
  std::uint32_t minimum_points = 3;
  // Cities hold many small objects; too low a cap drops a farther building.
  std::size_t maximum_clusters = 64;
  std::size_t maximum_hull_vertices = 24;
  // Returns lower than this height above the terrain reference passed to
  // Ingest are treated as terrain.  The reference assumes locally level ground
  // at the home (arming) altitude; it is a proposal filter, not a safety bound.
  double ground_clearance_m = 1.0;
};

struct ClusterPoint {
  Point3 point;
  bool on_top_ring = true;
};

// Deterministic grid connected-components clustering, nearest-first and
// bounded by options.maximum_clusters.  Exposed for unit tests.
[[nodiscard]] std::vector<ObstacleCluster> ClusterReturns(
    const std::vector<ClusterPoint>& points, double vehicle_north_m,
    double vehicle_east_m, const ClusterOptions& options,
    std::uint32_t* total_clusters = nullptr);

class ObstacleMap {
 public:
  explicit ObstacleMap(std::uint32_t expiry_ms = 750,
                       ClusterOptions cluster_options = {});
  // terrain_down_m is the local-NED down of the terrain reference used only
  // to reject ground returns from object proposals (default: the origin).
  void Ingest(const RangeScan& scan, double vehicle_north_m,
              double vehicle_east_m, double vehicle_down_m,
              double vehicle_roll_rad, double vehicle_pitch_rad,
              double vehicle_yaw_rad, double terrain_down_m = 0.0);
  [[nodiscard]] std::vector<Point3> Points(std::int64_t now_ms) const;
  [[nodiscard]] std::vector<ObstacleCluster> Clusters(std::int64_t now_ms) const;
  [[nodiscard]] v1::PerceptionSummary Summary(
      std::int64_t now_ms, double vehicle_north_m, double vehicle_east_m,
      double vehicle_yaw_rad) const;
  [[nodiscard]] bool Fresh(std::int64_t now_ms) const;

 private:
  mutable std::mutex mutex_;
  std::vector<Point3> points_ned_;
  std::vector<ObstacleCluster> clusters_;
  std::uint32_t cluster_total_ = 0;
  std::string sensor_id_;
  std::int64_t observed_at_ms_ = 0;
  std::uint64_t sequence_ = 0;
  std::uint32_t expiry_ms_;
  ClusterOptions cluster_options_;
};

}  // namespace icarus::perception

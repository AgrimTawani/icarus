#include "perception/obstacle_map/obstacle_map.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <unordered_map>
#include <unordered_set>
#include <utility>

namespace icarus::perception {
namespace {
constexpr double kPi = 3.14159265358979323846;
// The frontal map is deliberately a 2D collision slice.  Returns below this
// slice are terrain / landing-clearance observations, not obstacles.  Keeping
// the tolerance small rejects the Gazebo sensor's -3 degree ring, which hits
// level ground about 2.85 m away when the vehicle is landed, while retaining
// the horizontal ring and obstacles extending to the vehicle's height.
constexpr double kMaximumDownwardReturnM = 0.05;
// A return within this angle of the sensor's highest beam is "on the top
// ring": the object may continue above what the sensor can see.
constexpr double kTopRingToleranceRad = 1.0 * kPi / 180.0;

std::uint64_t CellKey(std::int64_t north, std::int64_t east) {
  return (static_cast<std::uint64_t>(north) << 32) ^
         (static_cast<std::uint64_t>(east) & 0xffffffffULL);
}

double Cross(const ObstacleCluster::Vertex& origin,
             const ObstacleCluster::Vertex& a,
             const ObstacleCluster::Vertex& b) {
  return (a.north_m - origin.north_m) * (b.east_m - origin.east_m) -
         (a.east_m - origin.east_m) * (b.north_m - origin.north_m);
}

// Andrew's monotone chain; collinear input collapses to its two endpoints.
std::vector<ObstacleCluster::Vertex> ConvexHull(
    std::vector<ObstacleCluster::Vertex> points) {
  std::sort(points.begin(), points.end(), [](const auto& a, const auto& b) {
    return a.north_m < b.north_m ||
           (a.north_m == b.north_m && a.east_m < b.east_m);
  });
  points.erase(std::unique(points.begin(), points.end(),
                           [](const auto& a, const auto& b) {
                             return a.north_m == b.north_m &&
                                    a.east_m == b.east_m;
                           }),
               points.end());
  if (points.size() < 3) return points;
  std::vector<ObstacleCluster::Vertex> hull(2 * points.size());
  std::size_t count = 0;
  for (const auto& point : points) {
    while (count >= 2 && Cross(hull[count - 2], hull[count - 1], point) <= 0.0)
      --count;
    hull[count++] = point;
  }
  for (std::size_t index = points.size() - 1, lower = count + 1; index > 0;
       --index) {
    const auto& point = points[index - 1];
    while (count >= lower &&
           Cross(hull[count - 2], hull[count - 1], point) <= 0.0)
      --count;
    hull[count++] = point;
  }
  hull.resize(count - 1);
  return hull;
}

std::vector<ObstacleCluster::Vertex> Decimate(
    std::vector<ObstacleCluster::Vertex> hull, std::size_t maximum) {
  if (maximum < 3 || hull.size() <= maximum) return hull;
  std::vector<ObstacleCluster::Vertex> kept;
  kept.reserve(maximum);
  for (std::size_t index = 0; index < maximum; ++index) {
    kept.push_back(hull[index * hull.size() / maximum]);
  }
  return kept;
}

ObstacleCluster::Vertex NearestOnHull(
    const std::vector<ObstacleCluster::Vertex>& hull, double north,
    double east) {
  ObstacleCluster::Vertex best = hull.front();
  double best_distance = std::numeric_limits<double>::infinity();
  const std::size_t edges = hull.size() == 1 ? 1 : hull.size();
  for (std::size_t index = 0; index < edges; ++index) {
    const auto& a = hull[index];
    const auto& b = hull[(index + 1) % hull.size()];
    const double dn = b.north_m - a.north_m;
    const double de = b.east_m - a.east_m;
    const double length = dn * dn + de * de;
    double t = 0.0;
    if (length > 0.0) {
      t = ((north - a.north_m) * dn + (east - a.east_m) * de) / length;
      t = std::clamp(t, 0.0, 1.0);
    }
    const ObstacleCluster::Vertex candidate{a.north_m + t * dn,
                                            a.east_m + t * de};
    const double distance =
        std::hypot(candidate.north_m - north, candidate.east_m - east);
    if (distance < best_distance) {
      best_distance = distance;
      best = candidate;
    }
  }
  return best;
}

bool InsideHull(const std::vector<ObstacleCluster::Vertex>& hull, double north,
                double east) {
  if (hull.size() < 3) return false;
  const ObstacleCluster::Vertex point{north, east};
  for (std::size_t index = 0; index < hull.size(); ++index) {
    if (Cross(hull[index], hull[(index + 1) % hull.size()], point) < 0.0)
      return false;
  }
  return true;
}
}  // namespace

std::vector<ObstacleCluster> ClusterReturns(
    const std::vector<ClusterPoint>& points, double vehicle_north,
    double vehicle_east, const ClusterOptions& options,
    std::uint32_t* total_clusters) {
  if (total_clusters != nullptr) *total_clusters = 0;
  if (points.empty() || !(options.cell_m > 0.0)) return {};
  std::unordered_map<std::uint64_t, std::vector<std::size_t>> cells;
  std::vector<std::pair<std::int64_t, std::int64_t>> occupied;
  for (std::size_t index = 0; index < points.size(); ++index) {
    const auto& point = points[index].point;
    if (!std::isfinite(point.x_m) || !std::isfinite(point.y_m) ||
        !std::isfinite(point.z_m) ||
        std::hypot(point.x_m - vehicle_north, point.y_m - vehicle_east) >
            options.maximum_range_m)
      continue;
    const auto north = static_cast<std::int64_t>(
        std::floor(point.x_m / options.cell_m));
    const auto east = static_cast<std::int64_t>(
        std::floor(point.y_m / options.cell_m));
    auto& members = cells[CellKey(north, east)];
    if (members.empty()) occupied.emplace_back(north, east);
    members.push_back(index);
  }
  // Visit cells in sorted order so identical scans yield identical clusters.
  std::sort(occupied.begin(), occupied.end());
  std::unordered_set<std::uint64_t> visited;
  std::vector<std::pair<double, ObstacleCluster>> found;
  for (const auto& seed : occupied) {
    if (!visited.insert(CellKey(seed.first, seed.second)).second) continue;
    std::vector<std::pair<std::int64_t, std::int64_t>> frontier{seed};
    std::vector<std::size_t> members;
    while (!frontier.empty()) {
      const auto cell = frontier.back();
      frontier.pop_back();
      const auto& contained = cells[CellKey(cell.first, cell.second)];
      members.insert(members.end(), contained.begin(), contained.end());
      for (std::int64_t dn = -1; dn <= 1; ++dn) {
        for (std::int64_t de = -1; de <= 1; ++de) {
          const auto next = std::make_pair(cell.first + dn, cell.second + de);
          const auto key = CellKey(next.first, next.second);
          if (cells.count(key) != 0 && visited.insert(key).second)
            frontier.push_back(next);
        }
      }
    }
    if (members.size() < options.minimum_points) continue;
    ObstacleCluster cluster;
    cluster.point_count = static_cast<std::uint32_t>(members.size());
    cluster.min_north_m = cluster.min_east_m = cluster.top_down_m =
        std::numeric_limits<double>::infinity();
    cluster.max_north_m = cluster.max_east_m = cluster.bottom_down_m =
        -std::numeric_limits<double>::infinity();
    cluster.top_clipped = false;
    double nearest = std::numeric_limits<double>::infinity();
    std::vector<ObstacleCluster::Vertex> planar;
    planar.reserve(members.size());
    for (const auto index : members) {
      const auto& point = points[index].point;
      cluster.centroid_north_m += point.x_m;
      cluster.centroid_east_m += point.y_m;
      cluster.centroid_down_m += point.z_m;
      cluster.min_north_m = std::min(cluster.min_north_m, point.x_m);
      cluster.max_north_m = std::max(cluster.max_north_m, point.x_m);
      cluster.min_east_m = std::min(cluster.min_east_m, point.y_m);
      cluster.max_east_m = std::max(cluster.max_east_m, point.y_m);
      cluster.top_down_m = std::min(cluster.top_down_m, point.z_m);
      cluster.bottom_down_m = std::max(cluster.bottom_down_m, point.z_m);
      cluster.top_clipped = cluster.top_clipped || points[index].on_top_ring;
      nearest = std::min(nearest, std::hypot(point.x_m - vehicle_north,
                                             point.y_m - vehicle_east));
      planar.push_back({point.x_m, point.y_m});
    }
    const double count = static_cast<double>(members.size());
    cluster.centroid_north_m /= count;
    cluster.centroid_east_m /= count;
    cluster.centroid_down_m /= count;
    cluster.hull = Decimate(ConvexHull(std::move(planar)),
                            options.maximum_hull_vertices);
    found.emplace_back(nearest, std::move(cluster));
  }
  if (total_clusters != nullptr)
    *total_clusters = static_cast<std::uint32_t>(found.size());
  std::stable_sort(found.begin(), found.end(),
                   [](const auto& a, const auto& b) { return a.first < b.first; });
  std::vector<ObstacleCluster> clusters;
  for (auto& item : found) {
    if (clusters.size() >= options.maximum_clusters) break;
    clusters.push_back(std::move(item.second));
  }
  return clusters;
}

ObstacleMap::ObstacleMap(std::uint32_t expiry_ms, ClusterOptions cluster_options)
    : expiry_ms_(expiry_ms), cluster_options_(cluster_options) {}

void ObstacleMap::Ingest(const RangeScan& scan, double north, double east,
                         double down, double roll, double pitch, double yaw,
                         double terrain_down) {
  std::vector<Point3> transformed;
  transformed.reserve(scan.points.size());
  std::vector<ClusterPoint> structure;
  structure.reserve(scan.points.size());
  std::vector<ClusterPoint> returns;
  returns.reserve(scan.points.size());
  const bool elevation_known = std::isfinite(scan.maximum_elevation_rad);
  for (const auto& point : scan.points) {
    Point3 local;
    bool on_top_ring = true;
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
      if (elevation_known) {
        const double elevation =
            std::atan2(-body_down, std::hypot(forward, right));
        on_top_ring =
            elevation >= scan.maximum_elevation_rad - kTopRingToleranceRad;
      }
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
    returns.push_back({local, on_top_ring});
  }
  // Object proposals use every above-terrain return, including structure
  // below or well above the collision slice, so heights can be measured.
  // Terrain is the deeper of home level and the lowest return within about
  // 12 m: ground slopes, and a wall standing on lower ground is not terrain.
  // On level ground this equals the home reference.
  constexpr double kGroundCellM = 4.0;
  const auto cell_key = [](double north_m, double east_m) {
    const auto row = static_cast<std::int64_t>(std::floor(north_m / kGroundCellM));
    const auto col = static_cast<std::int64_t>(std::floor(east_m / kGroundCellM));
    return (row << 32) ^ (col & 0xffffffff);
  };
  std::unordered_map<std::int64_t, double> lowest;
  for (const auto& item : returns) {
    auto [entry, added] = lowest.emplace(cell_key(item.point.x_m, item.point.y_m), item.point.z_m);
    if (!added) entry->second = std::max(entry->second, item.point.z_m);
  }
  for (const auto& item : returns) {
    double local_terrain = terrain_down;
    for (int dn = -1; dn <= 1; ++dn) {
      for (int de = -1; de <= 1; ++de) {
        const auto found = lowest.find(cell_key(item.point.x_m + dn * kGroundCellM,
                                                item.point.y_m + de * kGroundCellM));
        if (found != lowest.end()) local_terrain = std::max(local_terrain, found->second);
      }
    }
    if (item.point.z_m <= local_terrain - cluster_options_.ground_clearance_m) {
      structure.push_back(item);
    }
  }
  std::uint32_t total = 0;
  auto clusters = ClusterReturns(structure, north, east, cluster_options_, &total);
  std::lock_guard<std::mutex> lock(mutex_);
  points_ned_ = std::move(transformed);
  clusters_ = std::move(clusters);
  cluster_total_ = total;
  sensor_id_ = scan.sensor_id;
  observed_at_ms_ = scan.captured_at_unix_ms;
  ++sequence_;
}

std::vector<Point3> ObstacleMap::Points(std::int64_t now_ms) const {
  std::lock_guard<std::mutex> lock(mutex_);
  if (observed_at_ms_ == 0 || now_ms - observed_at_ms_ > expiry_ms_) return {};
  return points_ned_;
}

std::vector<ObstacleCluster> ObstacleMap::Clusters(std::int64_t now_ms) const {
  std::lock_guard<std::mutex> lock(mutex_);
  if (observed_at_ms_ == 0 || now_ms - observed_at_ms_ > expiry_ms_) return {};
  return clusters_;
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
  if (!fresh) return summary;
  summary.set_obstacle_cluster_total(cluster_total_);
  const double yaw_deg = yaw * 180.0 / kPi;
  for (std::size_t index = 0; index < clusters_.size(); ++index) {
    const auto& cluster = clusters_[index];
    if (cluster.hull.empty()) continue;
    auto* item = summary.add_nearby_obstacles();
    item->set_obstacle_id("lidar-" + std::to_string(sequence_) + "-" +
                          std::to_string(index));
    item->set_observed_at_unix_ms(observed_at_ms_);
    item->set_sensor_id(sensor_id_);
    item->set_point_count(cluster.point_count);
    const auto closest = NearestOnHull(cluster.hull, north, east);
    auto* nearest_position = item->mutable_nearest_position()->mutable_local_ned();
    nearest_position->set_north_m(closest.north_m);
    nearest_position->set_east_m(closest.east_m);
    nearest_position->set_down_m(cluster.top_down_m);
    const double dn = closest.north_m - north;
    const double de = closest.east_m - east;
    item->set_distance_m(std::hypot(dn, de));
    item->set_relative_bearing_deg(
        std::remainder(std::atan2(de, dn) * 180.0 / kPi - yaw_deg, 360.0));
    auto* centroid = item->mutable_centroid();
    centroid->set_north_m(cluster.centroid_north_m);
    centroid->set_east_m(cluster.centroid_east_m);
    centroid->set_down_m(cluster.centroid_down_m);
    item->set_min_north_m(cluster.min_north_m);
    item->set_max_north_m(cluster.max_north_m);
    item->set_min_east_m(cluster.min_east_m);
    item->set_max_east_m(cluster.max_east_m);
    item->set_top_down_m(cluster.top_down_m);
    item->set_bottom_down_m(cluster.bottom_down_m);
    item->set_top_clipped(cluster.top_clipped);
    double radius = 0.0;
    const double centre_bearing =
        std::atan2(cluster.centroid_east_m - east,
                   cluster.centroid_north_m - north);
    double span_min = std::numeric_limits<double>::infinity();
    double span_max = -std::numeric_limits<double>::infinity();
    for (const auto& vertex : cluster.hull) {
      auto* hull = item->add_hull();
      hull->set_north_m(vertex.north_m);
      hull->set_east_m(vertex.east_m);
      hull->set_down_m(cluster.top_down_m);
      radius = std::max(radius, std::hypot(vertex.north_m - cluster.centroid_north_m,
                                           vertex.east_m - cluster.centroid_east_m));
      const double offset = std::remainder(
          std::atan2(vertex.east_m - east, vertex.north_m - north) -
              centre_bearing,
          2.0 * kPi);
      span_min = std::min(span_min, offset);
      span_max = std::max(span_max, offset);
    }
    item->set_estimated_radius_m(radius);
    if (InsideHull(cluster.hull, north, east)) {
      item->set_relative_bearing_min_deg(-180.0);
      item->set_relative_bearing_max_deg(180.0);
    } else {
      item->set_relative_bearing_min_deg(std::remainder(
          (centre_bearing + span_min) * 180.0 / kPi - yaw_deg, 360.0));
      item->set_relative_bearing_max_deg(std::remainder(
          (centre_bearing + span_max) * 180.0 / kPi - yaw_deg, 360.0));
    }
  }
  return summary;
}

}  // namespace icarus::perception

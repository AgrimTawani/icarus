#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <iostream>
#include <stdexcept>
#include <string>
#include <vector>

#include "local_planner/local_planner.hpp"
#include "perception/lidar/airsim_lidar_adapter.hpp"
#include "perception/obstacle_map/obstacle_map.hpp"

namespace {

void Require(bool condition, const std::string& message) {
  if (!condition) throw std::runtime_error(message);
}

icarus::v1::LocalPositionNed Position(double north, double east,
                                      double down = -5.0) {
  icarus::v1::LocalPositionNed point;
  point.set_origin_id("home");
  point.set_north_m(north);
  point.set_east_m(east);
  point.set_down_m(down);
  return point;
}

void TestFrameParityAndFreshness() {
  icarus::perception::RangeScan simulation;
  simulation.sensor_id = "gazebo-lidar";
  simulation.captured_at_unix_ms = 1'000;
  simulation.frame = icarus::perception::SensorFrame::kBodyFlu;
  simulation.points.push_back({5.0, 2.0, 0.0, 1.0});
  icarus::perception::RangeScan physical = simulation;
  physical.sensor_id = "mid360";
  physical.frame = icarus::perception::SensorFrame::kBodyFrd;
  physical.points[0].y_m = -2.0;

  icarus::perception::ObstacleMap sim_map(750);
  icarus::perception::ObstacleMap real_map(750);
  sim_map.Ingest(simulation, 10.0, 20.0, -5.0, 0.0, 0.0, 0.0);
  real_map.Ingest(physical, 10.0, 20.0, -5.0, 0.0, 0.0, 0.0);
  const auto sim_points = sim_map.Points(1'100);
  const auto real_points = real_map.Points(1'100);
  Require(sim_points.size() == 1 && real_points.size() == 1,
          "both adapters should produce one normalized return");
  Require(std::abs(sim_points[0].x_m - real_points[0].x_m) < 1e-9 &&
              std::abs(sim_points[0].y_m - real_points[0].y_m) < 1e-9,
          "Gazebo FLU and physical FRD inputs must normalize identically");
  const auto summary = sim_map.Summary(1'100, 10.0, 20.0, 0.0);
  Require(summary.local_map_available() &&
              std::abs(summary.nearest_obstacle_distance_m() -
                       std::hypot(5.0, 2.0)) < 1e-9,
          "summary should expose nearest obstacle and healthy map");
  Require(!sim_map.Fresh(1'751) && sim_map.Points(1'751).empty(),
          "expired sensor data must not remain navigable");
}

void TestCollisionFreePlanning() {
  icarus::local_planner::LocalPlanner planner(0.5, 2.0, 8.0);
  const auto start = Position(0.0, 0.0);
  const auto goal = Position(20.0, 0.0);
  std::vector<icarus::perception::Point3> wall;
  for (double east = -4.0; east <= 4.0; east += 0.25) {
    wall.push_back({8.0, east, -5.0, 1.0});
  }
  Require(!planner.SegmentClear(start, goal, wall),
          "direct path through wall must be blocked");
  const auto plan = planner.Plan(start, goal, wall);
  Require(plan.success && !plan.direct_path && plan.waypoints.size() >= 2,
          "planner should generate a bounded detour around the wall");
  auto anchor = start;
  for (const auto& waypoint : plan.waypoints) {
    Require(planner.SegmentClear(anchor, waypoint, wall),
            "every simplified path segment must retain clearance");
    anchor = waypoint;
  }
  Require(std::hypot(anchor.north_m() - goal.north_m(),
                     anchor.east_m() - goal.east_m()) < 1e-9,
          "detour must terminate at requested destination");

  std::vector<icarus::perception::Point3> occupied_goal{{20.0, 0.0, -5.0, 1.0}};
  const auto rejected = planner.Plan(start, goal, occupied_goal);
  Require(!rejected.success,
          "planner must reject a destination inside the safety envelope");

  // A caller may request stricter clearance, but can never lower policy.
  std::vector<icarus::perception::Point3> near_path{{10.0, 2.5, -5.0, 1.0}};
  Require(planner.Plan(start, goal, near_path).direct_path,
          "policy-clear path should remain direct");
  Require(!planner.Plan(start, goal, near_path, 3.0).direct_path,
          "requested clearance must reach planner occupancy");
}

void RequireClearRoute(const icarus::local_planner::LocalPlanner& planner,
                       const icarus::v1::LocalPositionNed& start,
                       const icarus::v1::LocalPositionNed& goal,
                       const std::vector<icarus::perception::Point3>& obstacles) {
  const auto plan = planner.Plan(start, goal, obstacles);
  Require(plan.success && !plan.direct_path,
          "a clear detour must remain available around isolated obstacles");
  auto anchor = start;
  for (const auto& point : plan.waypoints) {
    Require(planner.SegmentClear(anchor, point, obstacles),
            "neither a grid edge nor a simplified edge may cut obstacle clearance");
    anchor = point;
  }
  Require(std::hypot(anchor.north_m()-goal.north_m(), anchor.east_m()-goal.east_m()) < 1e-9,
          "off-grid goal must be reached exactly");
}

void TestExactEdgesAndOffGridAttachments() {
  // Vary obstacle subcell offsets: rounded occupancy is not the geometric
  // collision contract, especially for diagonal edges and terminal attachments.
  icarus::local_planner::LocalPlanner planner(1.0, 0.6, 4.0);
  for (double shift : {0.0, 0.24, 0.49, 0.76}) {
    const auto start = Position(0.17, 0.31);
    const auto goal = Position(6.23, 6.37);
    std::vector<icarus::perception::Point3> obstacles{
        {2.17+shift, 2.31, -5, 1}, {3.17, 3.31+shift, -5, 1}};
    RequireClearRoute(planner, start, goal, obstacles);
  }
  // Diagonal movement between unoccupied grid vertices can intersect a return
  // that rounds to an adjacent vertex rather than either endpoint.
  RequireClearRoute(planner, Position(0,0), Position(5,5),
                    {{2.49, 2.01, -5, 1}, {3.01, 3.49, -5, 1}});
}

void TestNoClearStartCannotBecomeSuccessfulSimplification() {
  icarus::local_planner::LocalPlanner planner(1.0, 0.6, 4.0);
  const auto start = Position(0,0);
  const std::vector<icarus::perception::Point3> obstacles{{0,0.25,-5,1}};
  const auto result = planner.Plan(start, Position(5,0), obstacles);
  Require(!result.success && result.waypoints.empty(),
          "clearing occupied start cell must not fabricate an uncleared first segment");
}

void TestSoftMarginDeparturePreservesLiveFloor() {
  icarus::local_planner::LocalPlanner planner(0.5, 2.5, 8.0);
  const auto start = Position(-2.82, 0);
  std::vector<icarus::perception::Point3> wall;
  for (double east = -3; east <= 3; east += 0.25) wall.push_back({0,east,-5,1});
  Require(!planner.Plan(start, Position(-5, 0), wall, 3.25).success,
          "ordinary planning cannot depart an occupied envelope without explicit live floor");
  for (auto goal : {Position(-5, 0), Position(5, 8)}) {
    const auto result = planner.Plan(start, goal, wall, 3.25, 2.75);
    Require(result.success && !result.waypoints.empty(), "soft-margin departure must find a route");
    auto anchor = start;
    for (std::size_t index = 0; index < result.waypoints.size(); ++index) {
      const auto& waypoint = result.waypoints[index];
      Require(planner.SegmentClear(anchor, waypoint, wall, index == 0 ? 2.75 : 3.25),
              "departure never crosses live floor and later legs retain full margin");
      if (index == 0) {
        for (const auto& point : wall) {
          const double initial = std::hypot(start.north_m()-point.x_m, start.east_m()-point.y_m);
          if (initial >= 3.25) continue;
          double previous = initial;
          for (int step=1; step<=100; ++step) {
            const double t = step / 100.0;
            const double distance = std::hypot(start.north_m()+t*(waypoint.north_m()-start.north_m())-point.x_m,
                start.east_m()+t*(waypoint.east_m()-start.east_m())-point.y_m);
            Require(distance >= previous - 1e-9, "departure cannot move closer to nearby returns");
            previous = distance;
          }
        }
      }
      anchor = waypoint;
    }
  }
  Require(!planner.Plan(Position(-2.5,0), Position(-5,0), wall,3.25,2.75).success,
          "explicit departure permission cannot relax requested live floor");
  Require(!planner.Plan(start, Position(-2.9,0), wall,3.25,2.75).success,
          "departure endpoint must regain full planning clearance");
}

void TestGroundReturnsDoNotBecomeFrontalObstacles() {
  icarus::perception::RangeScan scan;
  scan.sensor_id = "gazebo-lidar";
  scan.captured_at_unix_ms = 1'000;
  scan.frame = icarus::perception::SensorFrame::kBodyFlu;
  // Gazebo's -3 degree ring intersects level terrain about 2.85 m away when
  // landed.  It must not trip the 2.5 m frontal emergency envelope during
  // takeoff.  The level return is a real wall and must remain in the map.
  scan.points.push_back({2.85, 0.0, -0.149, 1.0});
  scan.points.push_back({7.72, 0.0, 0.0, 1.0});

  icarus::perception::ObstacleMap map(750);
  map.Ingest(scan, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0);
  const auto points = map.Points(1'100);
  Require(points.size() == 1,
          "ground-grazing LiDAR rings must be excluded from frontal map");
  const auto summary = map.Summary(1'100, 0.0, 0.0, 0.0);
  Require(std::abs(summary.nearest_obstacle_distance_m() - 7.72) < 1e-9,
          "real level obstacle must remain after terrain filtering");
}

void TestPitchedHorizontalBeamHitsTerrain() {
  icarus::perception::RangeScan scan;
  scan.sensor_id = "gazebo-lidar";
  scan.captured_at_unix_ms = 1'000;
  scan.frame = icarus::perception::SensorFrame::kBodyFlu;
  scan.points.push_back({2.3, 0.0, 0.0, 1.0});
  scan.points.push_back({7.7, 0.0, 1.0, 1.0});
  icarus::perception::ObstacleMap map(750);
  map.Ingest(scan, 0.0, 0.0, -0.05, 0.0, -0.08, 0.0);
  const auto points = map.Points(1'100);
  Require(points.size() == 1,
          "pitch-down level beam striking terrain must not become an obstacle");
  Require(points[0].x_m > 7.0,
          "elevated wall return must remain after full attitude transform");
}

// A hovering vehicle at 5 m sees a building face 14 m north, a thin pole to
// the east and level terrain.  Terrain must not become a proposal, the two
// objects must stay separate, and the summary must expose measured geometry.
icarus::perception::RangeScan HoverScan() {
  icarus::perception::RangeScan scan;
  scan.sensor_id = "gazebo-lidar";
  scan.captured_at_unix_ms = 1'000;
  scan.frame = icarus::perception::SensorFrame::kBodyFlu;
  scan.maximum_elevation_rad = 52.0 * 3.14159265358979323846 / 180.0;
  for (int column = 0; column <= 32; ++column) {
    for (int row = 0; row <= 11; ++row) {
      // 0.25 m columns from 4 m left to 4 m right, 0.25 m rows from 2 m below
      // to 0.75 m above the sensor.
      scan.points.push_back({14.0, -4.0 + 0.25 * column, -2.0 + 0.25 * row, 1.0});
    }
  }
  for (double up = -3.0; up <= 0.0; up += 0.5) {
    scan.points.push_back({6.0, -8.0, up, 1.0});  // pole at north 6, east 8
  }
  for (double forward = 20.0; forward <= 30.0; forward += 0.5) {
    scan.points.push_back({forward, 20.0, -5.0, 1.0});  // level terrain
  }
  return scan;
}

void TestClustersExposeMeasuredObjects() {
  icarus::perception::ObstacleMap map(750);
  map.Ingest(HoverScan(), 0.0, 0.0, -5.0, 0.0, 0.0, 0.0);
  const auto summary = map.Summary(1'100, 0.0, 0.0, 0.0);
  Require(summary.obstacle_cluster_total() == 2 &&
              summary.nearby_obstacles_size() == 2,
          "terrain must be rejected and two objects must remain separate");
  const auto& pole = summary.nearby_obstacles(0);
  const auto& face = summary.nearby_obstacles(1);
  Require(pole.distance_m() < face.distance_m(),
          "clusters must be reported nearest first");
  Require(std::abs(pole.distance_m() - 10.0) < 1e-6 &&
              std::abs(pole.relative_bearing_deg() - 53.130102) < 1e-3,
          "pole range and bearing must match its returns");
  Require(pole.confidence() == 0.0 && pole.point_count() == 7,
          "clusters report point counts, never an invented confidence");
  Require(std::abs(face.distance_m() - 14.0) < 1e-6 &&
              std::abs(face.relative_bearing_deg()) < 1e-6,
          "nearest face point must lie straight ahead");
  Require(std::abs(face.min_east_m() + 4.0) < 1e-9 &&
              std::abs(face.max_east_m() - 4.0) < 1e-9 &&
              std::abs(face.min_north_m() - 14.0) < 1e-9 &&
              face.point_count() == 33 * 12,
          "face extent must match its returns");
  Require(std::abs(face.relative_bearing_min_deg() + 15.9454) < 1e-3 &&
              std::abs(face.relative_bearing_max_deg() - 15.9454) < 1e-3,
          "face angular span must cover both edges");
  Require(face.hull_size() >= 2 && face.hull_size() <= 24,
          "hull must be present and bounded");
  Require(std::abs(face.top_down_m() - (-5.0 - 0.75)) < 1e-9 &&
              std::abs(face.bottom_down_m() - (-3.0)) < 1e-6 &&
              !face.top_clipped(),
          "an object below the top beam must report a measured top");
  Require(face.sensor_id() == "gazebo-lidar" &&
              face.observed_at_unix_ms() == 1'000,
          "clusters must carry sensor provenance and scan time");
  // Yawing right by 90 degrees must move the face to the vehicle's left.
  const auto yawed = map.Summary(1'100, 0.0, 0.0, 3.14159265358979323846 / 2.0);
  Require(std::abs(yawed.nearby_obstacles(1).relative_bearing_deg() + 90.0) < 1e-6,
          "relative bearings must follow the current heading");
  Require(map.Summary(1'751, 0.0, 0.0, 0.0).nearby_obstacles_size() == 0 &&
              map.Clusters(1'751).empty(),
          "expired scans must not publish object proposals");
  // Collision behaviour is unchanged: only the frontal slice is navigable.
  for (const auto& point : map.Points(1'100)) {
    Require(point.z_m - (-5.0) <= 0.05 && point.z_m - (-5.0) >= -1.0,
            "collision slice must keep its vertical bounds");
  }
}

void TestClusterTopClippingAndBounds() {
  icarus::perception::RangeScan scan;
  scan.sensor_id = "gazebo-lidar";
  scan.captured_at_unix_ms = 1'000;
  scan.frame = icarus::perception::SensorFrame::kBodyFlu;
  scan.maximum_elevation_rad = 0.5;
  // A close wall reaching the top beam: its real top is not observed.
  for (double left = -1.0; left <= 1.0; left += 0.25) {
    scan.points.push_back({3.0, left, 3.0 * std::tan(0.5), 1.0});
    scan.points.push_back({3.0, left, 0.0, 1.0});
  }
  // Twenty separated poles plus one beyond the 40 m proposal range.
  for (int index = 0; index < 20; ++index) {
    const double angle = index * 2.0 * 3.14159265358979323846 / 20.0;
    for (int sample = 0; sample < 3; ++sample) {
      scan.points.push_back({20.0 * std::cos(angle), 20.0 * std::sin(angle),
                             -0.5 * sample, 1.0});
    }
  }
  for (int sample = 0; sample < 5; ++sample) {
    scan.points.push_back({45.0, 0.0, -0.3 * sample, 1.0});
  }
  icarus::perception::ObstacleMap map(750);
  map.Ingest(scan, 0.0, 0.0, -5.0, 0.0, 0.0, 0.0);
  const auto summary = map.Summary(1'100, 0.0, 0.0, 0.0);
  Require(summary.obstacle_cluster_total() == 21,
          "all in-range clusters must be counted");
  Require(summary.nearby_obstacles_size() == 16,
          "published proposals must be bounded to the nearest sixteen");
  Require(summary.nearby_obstacles(0).top_clipped() &&
              std::abs(summary.nearby_obstacles(0).distance_m() - 3.0) < 1e-6,
          "a return on the top beam must mark the top as clipped");
  for (const auto& item : summary.nearby_obstacles()) {
    Require(item.distance_m() <= 40.0, "proposals beyond range must be dropped");
  }
  // Unknown elevation limits (for example local-frame input) never claim a
  // measured top.
  icarus::perception::RangeScan local;
  local.captured_at_unix_ms = 1'000;
  local.frame = icarus::perception::SensorFrame::kLocalNed;
  for (int sample = 0; sample < 4; ++sample) {
    local.points.push_back({8.0, 0.1 * sample, -6.0, 1.0});
  }
  icarus::perception::ObstacleMap local_map(750);
  local_map.Ingest(local, 0.0, 0.0, -5.0, 0.0, 0.0, 0.0);
  const auto local_summary = local_map.Summary(1'100, 0.0, 0.0, 0.0);
  Require(local_summary.nearby_obstacles_size() == 1 &&
              local_summary.nearby_obstacles(0).top_clipped(),
          "unknown sensor elevation limits must report a clipped top");
}

void TestClusterTerrainReferenceFollowsHomeAltitude() {
  // Barometer drift can leave the EKF origin 1.6 m below the arming point.
  // Ground returns must be judged against the home terrain reference, not the
  // drifted origin, or level terrain becomes a false object proposal.
  icarus::perception::RangeScan scan;
  scan.captured_at_unix_ms = 1'000;
  scan.frame = icarus::perception::SensorFrame::kBodyFlu;
  scan.maximum_elevation_rad = 0.9;
  for (int index = 0; index < 20; ++index) {
    scan.points.push_back({20.0 + 0.25 * index, 0.0, -2.0, 1.0});  // terrain
  }
  icarus::perception::ObstacleMap origin_map(750);
  origin_map.Ingest(scan, 0.0, 0.0, -3.6, 0.0, 0.0, 0.0);
  Require(origin_map.Summary(1'100, 0.0, 0.0, 0.0).nearby_obstacles_size() == 1,
          "with the drifted origin as terrain the ground looks like an object");
  icarus::perception::ObstacleMap home_map(750);
  home_map.Ingest(scan, 0.0, 0.0, -3.6, 0.0, 0.0, 0.0, -1.6);
  Require(home_map.Summary(1'100, 0.0, 0.0, 0.0).nearby_obstacles_size() == 0,
          "terrain at the home reference must not become an object proposal");
}

void TestClusterReturnsIsDeterministic() {
  std::vector<icarus::perception::ClusterPoint> points;
  for (int index = 0; index < 30; ++index) {
    points.push_back({{10.0 + 0.125 * index, -3.0 + 0.25 * index, -6.0, 1.0},
                      false});
  }
  icarus::perception::ClusterOptions options;
  std::uint32_t total = 0;
  const auto first = icarus::perception::ClusterReturns(points, 0.0, 0.0,
                                                        options, &total);
  std::reverse(points.begin(), points.end());
  const auto second = icarus::perception::ClusterReturns(points, 0.0, 0.0,
                                                         options);
  Require(total == 1 && first.size() == 1 && second.size() == 1,
          "a continuous line of returns must form one cluster");
  Require(first[0].hull.size() == second[0].hull.size() &&
              std::abs(first[0].centroid_north_m - second[0].centroid_north_m) <
                  1e-9,
          "input order must not change the cluster");
  Require(first[0].hull.size() == 2,
          "collinear returns must collapse to a two-vertex hull");
}

template <typename T>
void AppendNative(std::vector<std::uint8_t>* bytes, T value) {
  const auto start = bytes->size();
  bytes->resize(start + sizeof(T));
  std::memcpy(bytes->data() + start, &value, sizeof(T));
}

void TestAirSimPacketContract() {
  std::vector<std::uint8_t> packet{'I', 'C', 'A', 'R'};
  AppendNative<std::uint16_t>(&packet, 1);
  AppendNative<std::uint16_t>(&packet, 2);
  AppendNative<std::int64_t>(&packet, 1'700'000'000'000);
  AppendNative<std::uint64_t>(&packet, 7);
  AppendNative<float>(&packet, 3.0F);
  AppendNative<float>(&packet, 1.0F);
  AppendNative<float>(&packet, -0.2F);
  AppendNative<float>(&packet, 5.0F);
  AppendNative<float>(&packet, -2.0F);
  AppendNative<float>(&packet, 0.0F);
  icarus::perception::RangeScan scan;
  Require(icarus::perception::AirSimLidarAdapter::DecodePacket(
              packet.data(), packet.size(), &scan),
          "valid AirSim sensor packet must decode");
  Require(scan.sensor_id == "airsim-lidar" && scan.sequence == 7 &&
              scan.frame == icarus::perception::SensorFrame::kBodyFrd &&
              scan.points.size() == 2,
          "AirSim packet must retain sensor identity, FRD frame and points");
  packet[0] = 'X';
  Require(!icarus::perception::AirSimLidarAdapter::DecodePacket(
              packet.data(), packet.size(), &scan),
          "wrong packet magic must fail closed");
  packet[0] = 'I';
  Require(!icarus::perception::AirSimLidarAdapter::DecodePacket(
              packet.data(), packet.size() - 1, &scan),
          "truncated packet must fail closed");
}

}  // namespace

int main() {
  try {
    {
      using icarus::local_planner::StoppingPathClearance;
      const auto start = Position(0, 0);
      const std::vector<icarus::perception::Point3> side_wall{{0, 3, -5, 1}};
      Require(StoppingPathClearance(start, 0, 2, 0.25, 1, side_wall) < 2,
              "lateral closing motion must brake before the fixed floor is crossed");
      Require(std::isinf(StoppingPathClearance(start, 0, -2, 0.25, 1, side_wall)),
              "retreat must not be mistaken for closing motion");
      Require(std::isinf(StoppingPathClearance(start, 2, 0, 0.25, 1, side_wall)),
              "parallel motion must not be stopped by a side wall");
      Require(StoppingPathClearance(start, 0, 0.5, 0.25, 1, side_wall) > 2,
              "a slower approach retains sufficient stopping clearance");
      Require(StoppingPathClearance(start, 0, 1, 1, 1, side_wall) <
              StoppingPathClearance(start, 0, 1, 0, 1, side_wall),
              "sensor and command latency consume stopping clearance");
      Require(StoppingPathClearance(start, 0, 2, 0.25, 0.5, side_wall) <
              StoppingPathClearance(start, 0, 2, 0.25, 2, side_wall),
              "weaker braking must predict a longer stopping path");
      Require(StoppingPathClearance(start, NAN, 0, 0.25, 1, side_wall) == 0,
              "nonfinite motion cannot prove stopping safety");
      Require(std::isinf(StoppingPathClearance(start, 0, 0, 0.25, 1, side_wall)),
              "stationary aircraft has no predicted closing path");
      const std::vector<icarus::perception::Point3> floor{{0, 3, 0, 1}};
      Require(std::isinf(StoppingPathClearance(start, 0, 2, 0.25, 1, floor)),
              "terrain below the collision slice must not block horizontal motion");
    }
    TestFrameParityAndFreshness();
    TestCollisionFreePlanning();
    TestExactEdgesAndOffGridAttachments();
    TestNoClearStartCannotBecomeSuccessfulSimplification();
    TestSoftMarginDeparturePreservesLiveFloor();
    TestGroundReturnsDoNotBecomeFrontalObstacles();
    TestPitchedHorizontalBeamHitsTerrain();
    TestAirSimPacketContract();
    TestClustersExposeMeasuredObjects();
    TestClusterTopClippingAndBounds();
    TestClusterTerrainReferenceFollowsHomeAltitude();
    TestClusterReturnsIsDeterministic();
    std::cout << "Phase 9 perception and planning tests passed\n";
    return 0;
  } catch (const std::exception& error) {
    std::cerr << "Phase 9 test failure: " << error.what() << '\n';
    return 1;
  }
}

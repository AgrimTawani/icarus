#include <cmath>
#include <iostream>
#include <stdexcept>
#include <string>
#include <vector>

#include "local_planner/local_planner.hpp"
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

}  // namespace

int main() {
  try {
    TestFrameParityAndFreshness();
    TestCollisionFreePlanning();
    TestGroundReturnsDoNotBecomeFrontalObstacles();
    TestPitchedHorizontalBeamHitsTerrain();
    std::cout << "Phase 9 perception and planning tests passed\n";
    return 0;
  } catch (const std::exception& error) {
    std::cerr << "Phase 9 test failure: " << error.what() << '\n';
    return 1;
  }
}

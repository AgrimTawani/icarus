#pragma once

#include <cstdint>
#include <string>
#include <vector>

#include "icarus/v1/state.pb.h"
#include "perception/sensor_types.hpp"

namespace icarus::local_planner {

struct PlanResult {
  bool success = false;
  bool direct_path = false;
  std::string reason;
  std::vector<v1::LocalPositionNed> waypoints;
};

class LocalPlanner {
 public:
  LocalPlanner(double grid_resolution_m = 1.0, double clearance_m = 2.0,
               double planning_margin_m = 8.0,
               std::uint32_t maximum_cells = 40'000);
  [[nodiscard]] PlanResult Plan(
      const v1::LocalPositionNed& start,
      const v1::LocalPositionNed& goal,
      const std::vector<perception::Point3>& obstacles) const;
  [[nodiscard]] bool SegmentClear(
      const v1::LocalPositionNed& start,
      const v1::LocalPositionNed& goal,
      const std::vector<perception::Point3>& obstacles) const;

 private:
  double resolution_m_;
  double clearance_m_;
  double margin_m_;
  std::uint32_t maximum_cells_;
};

}  // namespace icarus::local_planner

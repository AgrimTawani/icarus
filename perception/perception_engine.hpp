#pragma once

#include <cstdint>

#include "autonomy_core/clock.hpp"
#include "perception/obstacle_map/obstacle_map.hpp"
#include "perception/sensor_types.hpp"
#include "state_engine/state_engine.hpp"

namespace icarus::perception {

class PerceptionEngine {
 public:
  PerceptionEngine(const core::Clock& clock,
                   state_engine::StateEngine& state_engine,
                   std::uint32_t maximum_age_ms = 750);
  void Ingest(const RangeScan& scan);
  [[nodiscard]] v1::PerceptionSummary Summary() const;
  [[nodiscard]] bool Healthy() const;
  [[nodiscard]] std::vector<Point3> Obstacles() const;

 private:
  const core::Clock& clock_;
  state_engine::StateEngine& state_engine_;
  ObstacleMap map_;
};

}  // namespace icarus::perception

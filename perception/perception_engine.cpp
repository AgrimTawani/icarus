#include "perception/perception_engine.hpp"

namespace icarus::perception {

PerceptionEngine::PerceptionEngine(const core::Clock& clock,
                                   state_engine::StateEngine& state_engine,
                                   std::uint32_t maximum_age_ms)
    : clock_(clock), state_engine_(state_engine), map_(maximum_age_ms) {}

void PerceptionEngine::Ingest(const RangeScan& scan) {
  const auto state = state_engine_.Snapshot();
  if (!state || !state->has_local_position_ned()) return;
  map_.Ingest(scan, state->local_position_ned().north_m(),
              state->local_position_ned().east_m(),
              state->local_position_ned().down_m(),
              state->attitude().roll_rad(), state->attitude().pitch_rad(),
              state->attitude().yaw_rad());
}

v1::PerceptionSummary PerceptionEngine::Summary() const {
  const auto state = state_engine_.Snapshot();
  if (!state || !state->has_local_position_ned()) {
    v1::PerceptionSummary summary;
    summary.set_overall(v1::HEALTH_LEVEL_UNAVAILABLE);
    return summary;
  }
  return map_.Summary(clock_.NowUnixMs(),
                      state->local_position_ned().north_m(),
                      state->local_position_ned().east_m(),
                      state->attitude().yaw_rad());
}

bool PerceptionEngine::Healthy() const { return map_.Fresh(clock_.NowUnixMs()); }

std::vector<Point3> PerceptionEngine::Obstacles() const {
  return map_.Points(clock_.NowUnixMs());
}

}  // namespace icarus::perception

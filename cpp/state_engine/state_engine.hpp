#pragma once

#include <mutex>
#include <optional>

#include "autonomy_core/clock.hpp"
#include "icarus/v1/state.pb.h"

namespace icarus::state_engine {

class StateEngine {
 public:
  explicit StateEngine(const core::Clock& clock);

  void Publish(v1::DroneState state);
  [[nodiscard]] std::optional<v1::DroneState> Snapshot() const;
  [[nodiscard]] bool IsFresh(std::uint32_t maximum_age_ms) const;
  [[nodiscard]] v1::HealthReport Health(std::uint32_t maximum_age_ms) const;

 private:
  const core::Clock& clock_;
  mutable std::mutex mutex_;
  std::optional<v1::DroneState> state_;
  std::uint64_t next_sequence_ = 1;
};

}  // namespace icarus::state_engine

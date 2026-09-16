#pragma once

#include <atomic>
#include <cstdint>
#include <thread>

#include "autonomy_core/clock.hpp"
#include "guardrails/safety_policy.hpp"
#include "mavlink_gateway/mavlink_gateway.hpp"
#include "mission_executor/action_store.hpp"
#include "state_engine/state_engine.hpp"
#include "perception/perception_engine.hpp"

namespace icarus::safety_supervisor {

// Independent last-resort monitor. Normal command validation remains in
// Guardrails; this loop watches the aircraft even when no action RPC is active.
class SafetySupervisor {
 public:
  SafetySupervisor(const core::Clock& clock,
                   const guardrails::SafetyPolicy& policy,
                   state_engine::StateEngine& state_engine,
                   mission_executor::ActionStore& actions,
                   mavlink_gateway::MavlinkGateway& gateway,
                   perception::PerceptionEngine* perception = nullptr);
  ~SafetySupervisor();

  void Start();
  void Stop();

 private:
  void Run();
  void AbortActive(v1::ReasonCode reason, const char* message,
                   const v1::DroneState* state);

  const core::Clock& clock_;
  const guardrails::SafetyPolicy& policy_;
  state_engine::StateEngine& state_engine_;
  mission_executor::ActionStore& actions_;
  mavlink_gateway::MavlinkGateway& gateway_;
  perception::PerceptionEngine* perception_;
  std::atomic<bool> running_{false};
  std::thread worker_;
};

}  // namespace icarus::safety_supervisor

#pragma once

#include <atomic>
#include <condition_variable>
#include <cstdint>
#include <deque>
#include <functional>
#include <mutex>
#include <string>
#include <thread>
#include <utility>

#include "autonomy_core/clock.hpp"
#include "drone_api/authority_manager.hpp"
#include "guardrails/guardrails.hpp"
#include "local_planner/local_planner.hpp"
#include "mavlink_gateway/mavlink_gateway.hpp"
#include "mission_executor/action_store.hpp"
#include "state_engine/state_engine.hpp"
#include "perception/perception_engine.hpp"

namespace icarus::mission_executor {

class ActionExecutor {
 public:
  ActionExecutor(const core::Clock& clock, ActionStore& store,
                 state_engine::StateEngine& state_engine,
                 drone_api::AuthorityManager& authority,
                 const guardrails::Guardrails& guardrails,
                 mavlink_gateway::MavlinkGateway& gateway,
                 perception::PerceptionEngine* perception = nullptr,
                 const local_planner::LocalPlanner* planner = nullptr);
  ~ActionExecutor();

  void Start();
  void Stop();
  [[nodiscard]] v1::ActionReceipt Submit(const v1::ActionCommand& command);
  [[nodiscard]] v1::ActionReceipt Cancel(
      const v1::CancelActionRequest& request);

 private:
  void Run();
  void Execute(const std::string& action_id,
               const v1::ActionCommand& command);
  bool WaitUntil(
      const std::string& action_id, const v1::CommandContext& context,
      std::uint32_t timeout_ms,
      const std::function<bool(const v1::DroneState&)>& predicate,
      const std::string& completion_message, bool complete_action = true);
  bool ExecuteGoto(const std::string& action_id,
                   const v1::CommandContext& context,
                   const v1::Position& destination,
                   double acceptance_radius_m, std::uint32_t timeout_ms,
                   bool complete_action = true);
  [[nodiscard]] bool SetMode(const std::string& action_id,
                             std::uint32_t mode,
                             const std::string& name);
  [[nodiscard]] static const v1::CommandContext& ContextOf(
      const v1::ActionCommand& command);
  [[nodiscard]] static double GeoDistanceM(const v1::GeoPosition& first,
                                           const v1::GeoPosition& second);

  const core::Clock& clock_;
  ActionStore& store_;
  state_engine::StateEngine& state_engine_;
  drone_api::AuthorityManager& authority_;
  const guardrails::Guardrails& guardrails_;
  mavlink_gateway::MavlinkGateway& gateway_;
  perception::PerceptionEngine* perception_;
  const local_planner::LocalPlanner* planner_;
  std::atomic<bool> running_{false};
  std::thread worker_;
  std::mutex queue_mutex_;
  std::condition_variable queue_changed_;
  std::deque<std::pair<std::string, v1::ActionCommand>> queue_;
};

}  // namespace icarus::mission_executor

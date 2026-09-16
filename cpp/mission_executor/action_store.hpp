#pragma once

#include <mutex>
#include <optional>
#include <string>
#include <unordered_map>

#include "autonomy_core/clock.hpp"
#include "icarus/v1/action.pb.h"

namespace icarus::mission_executor {

class ActionStore {
 public:
  explicit ActionStore(const core::Clock& clock);

  [[nodiscard]] v1::ActionReceipt Accept(const v1::ActionCommand& command);
  [[nodiscard]] v1::ActionReceipt Reject(const v1::ActionCommand& command,
                                         const v1::ValidationResult& validation);
  [[nodiscard]] std::optional<v1::ActionStatus> Find(
      const std::string& action_id) const;
  [[nodiscard]] std::optional<v1::ActionStatus> FindByIdempotencyKey(
      const std::string& idempotency_key) const;
  bool Transition(const std::string& action_id, v1::ActionState next,
                  v1::ReasonCode reason, std::string message,
                  double progress = 0.0,
                  const v1::DroneState* terminal_state = nullptr);
  [[nodiscard]] std::optional<std::string> ActiveConflictingAction() const;

 private:
  [[nodiscard]] static v1::ActionType TypeOf(const v1::ActionCommand& command);
  [[nodiscard]] static const v1::CommandContext* ContextOf(
      const v1::ActionCommand& command);
  [[nodiscard]] static std::string NewId();
  [[nodiscard]] static bool IsTerminal(v1::ActionState state);
  [[nodiscard]] static bool TransitionAllowed(v1::ActionState from,
                                              v1::ActionState to);
  [[nodiscard]] static v1::ActionReceipt ReceiptOf(
      const v1::ActionStatus& status);

  const core::Clock& clock_;
  mutable std::mutex mutex_;
  std::unordered_map<std::string, v1::ActionStatus> actions_;
  std::unordered_map<std::string, std::string> idempotency_index_;
};

}  // namespace icarus::mission_executor

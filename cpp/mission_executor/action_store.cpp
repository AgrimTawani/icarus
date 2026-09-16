#include "mission_executor/action_store.hpp"

#include <array>
#include <iomanip>
#include <random>
#include <sstream>
#include <stdexcept>
#include <utility>

#include <openssl/evp.h>

namespace icarus::mission_executor {
namespace {

std::string Sha256(const std::string& data) {
  std::array<unsigned char, EVP_MAX_MD_SIZE> digest{};
  unsigned int length = 0;
  auto* context = EVP_MD_CTX_new();
  if (context == nullptr ||
      EVP_DigestInit_ex(context, EVP_sha256(), nullptr) != 1 ||
      EVP_DigestUpdate(context, data.data(), data.size()) != 1 ||
      EVP_DigestFinal_ex(context, digest.data(), &length) != 1) {
    if (context != nullptr) EVP_MD_CTX_free(context);
    throw std::runtime_error("unable to hash action request");
  }
  EVP_MD_CTX_free(context);
  std::ostringstream stream;
  stream << "sha256:" << std::hex << std::setfill('0');
  for (unsigned int index = 0; index < length; ++index) {
    stream << std::setw(2) << static_cast<unsigned int>(digest[index]);
  }
  return stream.str();
}

}  // namespace

ActionStore::ActionStore(const core::Clock& clock) : clock_(clock) {}

v1::ActionReceipt ActionStore::Accept(const v1::ActionCommand& command) {
  const auto* context = ContextOf(command);
  std::lock_guard<std::mutex> lock(mutex_);
  if (context != nullptr) {
    const auto existing = idempotency_index_.find(context->idempotency_key());
    if (existing != idempotency_index_.end()) {
      return ReceiptOf(actions_.at(existing->second));
    }
  }

  v1::ActionStatus status;
  status.set_action_id(NewId());
  status.set_type(TypeOf(command));
  status.set_state(v1::ACTION_STATE_ACCEPTED);
  status.set_reason_code(v1::REASON_CODE_OK);
  status.set_message("action accepted for deterministic execution");
  status.set_accepted_at_unix_ms(clock_.NowUnixMs());
  status.set_progress(0.0);
  status.set_request_hash(Sha256(command.SerializeAsString()));
  const auto action_id = status.action_id();
  actions_.emplace(action_id, status);
  if (context != nullptr) {
    idempotency_index_[context->idempotency_key()] = action_id;
  }
  return ReceiptOf(status);
}

v1::ActionReceipt ActionStore::Reject(
    const v1::ActionCommand& command,
    const v1::ValidationResult& validation) {
  v1::ActionStatus status;
  status.set_action_id(NewId());
  status.set_type(TypeOf(command));
  status.set_state(v1::ACTION_STATE_REJECTED);
  status.set_reason_code(validation.reason_code());
  status.set_message(validation.message());
  status.set_accepted_at_unix_ms(clock_.NowUnixMs());
  status.set_finished_at_unix_ms(clock_.NowUnixMs());
  status.set_progress(0.0);

  std::lock_guard<std::mutex> lock(mutex_);
  actions_.emplace(status.action_id(), status);
  return ReceiptOf(status);
}

std::optional<v1::ActionStatus> ActionStore::Find(
    const std::string& action_id) const {
  std::lock_guard<std::mutex> lock(mutex_);
  const auto found = actions_.find(action_id);
  if (found == actions_.end()) return std::nullopt;
  return found->second;
}

std::optional<v1::ActionStatus> ActionStore::FindByIdempotencyKey(
    const std::string& idempotency_key) const {
  std::lock_guard<std::mutex> lock(mutex_);
  const auto index = idempotency_index_.find(idempotency_key);
  if (index == idempotency_index_.end()) return std::nullopt;
  return actions_.at(index->second);
}

bool ActionStore::Transition(const std::string& action_id, v1::ActionState next,
                             v1::ReasonCode reason, std::string message,
                             double progress,
                             const v1::DroneState* terminal_state) {
  std::lock_guard<std::mutex> lock(mutex_);
  const auto found = actions_.find(action_id);
  if (found == actions_.end() ||
      !TransitionAllowed(found->second.state(), next)) {
    return false;
  }
  auto& status = found->second;
  status.set_state(next);
  status.set_reason_code(reason);
  status.set_message(std::move(message));
  status.set_progress(progress);
  if (next == v1::ACTION_STATE_EXECUTING &&
      status.started_at_unix_ms() == 0) {
    status.set_started_at_unix_ms(clock_.NowUnixMs());
  }
  if (IsTerminal(next)) {
    status.set_finished_at_unix_ms(clock_.NowUnixMs());
    if (terminal_state != nullptr) {
      *status.mutable_terminal_state() = *terminal_state;
    }
  }
  return true;
}

std::optional<std::string> ActionStore::ActiveConflictingAction() const {
  std::lock_guard<std::mutex> lock(mutex_);
  for (const auto& [id, action] : actions_) {
    if (!IsTerminal(action.state())) return id;
  }
  return std::nullopt;
}

v1::ActionType ActionStore::TypeOf(const v1::ActionCommand& command) {
  switch (command.request_case()) {
    case v1::ActionCommand::kArm:
      return v1::ACTION_TYPE_ARM;
    case v1::ActionCommand::kDisarm:
      return v1::ACTION_TYPE_DISARM;
    case v1::ActionCommand::kTakeoff:
      return v1::ACTION_TYPE_TAKEOFF;
    case v1::ActionCommand::kGoto:
      return v1::ACTION_TYPE_GOTO;
    case v1::ActionCommand::kExecuteRoute:
      return v1::ACTION_TYPE_EXECUTE_ROUTE;
    case v1::ActionCommand::kHold:
      return v1::ACTION_TYPE_HOLD;
    case v1::ActionCommand::kReturnHome:
      return v1::ACTION_TYPE_RETURN_HOME;
    case v1::ActionCommand::kLand:
      return v1::ACTION_TYPE_LAND;
    case v1::ActionCommand::kOrbit:
      return v1::ACTION_TYPE_ORBIT;
    case v1::ActionCommand::REQUEST_NOT_SET:
      return v1::ACTION_TYPE_UNSPECIFIED;
    default:
      return v1::ACTION_TYPE_UNSPECIFIED;
  }
}

const v1::CommandContext* ActionStore::ContextOf(
    const v1::ActionCommand& command) {
  switch (command.request_case()) {
    case v1::ActionCommand::kArm:
      return &command.arm().context();
    case v1::ActionCommand::kDisarm:
      return &command.disarm().context();
    case v1::ActionCommand::kTakeoff:
      return &command.takeoff().context();
    case v1::ActionCommand::kGoto:
      return &command.goto_().context();
    case v1::ActionCommand::kExecuteRoute:
      return &command.execute_route().context();
    case v1::ActionCommand::kHold:
      return &command.hold().context();
    case v1::ActionCommand::kReturnHome:
      return &command.return_home().context();
    case v1::ActionCommand::kLand:
      return &command.land().context();
    case v1::ActionCommand::kOrbit:
      return &command.orbit().context();
    case v1::ActionCommand::REQUEST_NOT_SET:
      return nullptr;
    default:
      return nullptr;
  }
}

std::string ActionStore::NewId() {
  std::array<unsigned char, 16> bytes{};
  std::random_device device;
  for (auto& byte : bytes) byte = static_cast<unsigned char>(device());
  std::ostringstream stream;
  stream << std::hex << std::setfill('0');
  for (const auto byte : bytes) {
    stream << std::setw(2) << static_cast<unsigned int>(byte);
  }
  return stream.str();
}

bool ActionStore::IsTerminal(v1::ActionState state) {
  return state == v1::ACTION_STATE_REJECTED ||
         state == v1::ACTION_STATE_SUCCEEDED ||
         state == v1::ACTION_STATE_CANCELLED ||
         state == v1::ACTION_STATE_TIMED_OUT ||
         state == v1::ACTION_STATE_FAILED ||
         state == v1::ACTION_STATE_PREEMPTED ||
         state == v1::ACTION_STATE_ABORTED_BY_SAFETY;
}

bool ActionStore::TransitionAllowed(v1::ActionState from, v1::ActionState to) {
  if (IsTerminal(from)) return false;
  switch (to) {
    case v1::ACTION_STATE_QUEUED:
      return from == v1::ACTION_STATE_ACCEPTED;
    case v1::ACTION_STATE_VALIDATING:
      return from == v1::ACTION_STATE_ACCEPTED ||
             from == v1::ACTION_STATE_QUEUED;
    case v1::ACTION_STATE_EXECUTING:
      return from == v1::ACTION_STATE_ACCEPTED ||
             from == v1::ACTION_STATE_QUEUED ||
             from == v1::ACTION_STATE_VALIDATING;
    case v1::ACTION_STATE_SUCCEEDED:
    case v1::ACTION_STATE_CANCELLED:
    case v1::ACTION_STATE_TIMED_OUT:
    case v1::ACTION_STATE_FAILED:
    case v1::ACTION_STATE_PREEMPTED:
    case v1::ACTION_STATE_ABORTED_BY_SAFETY:
      return true;
    case v1::ACTION_STATE_REJECTED:
    case v1::ACTION_STATE_ACCEPTED:
    case v1::ACTION_STATE_UNSPECIFIED:
      return false;
    default:
      return false;
  }
}

v1::ActionReceipt ActionStore::ReceiptOf(const v1::ActionStatus& status) {
  v1::ActionReceipt receipt;
  receipt.set_action_id(status.action_id());
  receipt.set_disposition(status.state());
  receipt.set_reason_code(status.reason_code());
  receipt.set_message(status.message());
  receipt.set_received_at_unix_ms(status.accepted_at_unix_ms());
  return receipt;
}

}  // namespace icarus::mission_executor

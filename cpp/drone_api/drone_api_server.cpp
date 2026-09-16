#include "drone_api/drone_api_server.hpp"

#include <algorithm>
#include <array>
#include <chrono>
#include <iomanip>
#include <random>
#include <sstream>
#include <thread>
#include <utility>

namespace icarus::drone_api {
namespace {

grpc::Status InvalidSession() {
  return {grpc::StatusCode::UNAUTHENTICATED,
          "a valid session is required"};
}

void AddCapability(v1::VehicleCapabilities* response, const std::string& name,
                   bool supported, const std::string& version = "1") {
  auto* capability = response->add_capabilities();
  capability->set_name(name);
  capability->set_supported(supported);
  capability->set_version(version);
}

}  // namespace

DroneApiServer::DroneApiServer(
    const core::Clock& clock, AuthorityManager& authority,
    state_engine::StateEngine& state_engine,
    const guardrails::Guardrails& guardrails,
    mission_executor::ActionStore& actions,
    mission_executor::ActionExecutor& executor,
    perception::PerceptionEngine& perception)
    : clock_(clock),
      authority_(authority),
      state_engine_(state_engine),
      guardrails_(guardrails),
      actions_(actions),
      executor_(executor),
      perception_(perception) {}

void DroneApiServer::PublishEvent(v1::VehicleEvent event) {
  std::lock_guard<std::mutex> lock(events_mutex_);
  if (event.event_id().empty()) {
    event.set_event_id(std::to_string(next_event_id_++));
  }
  events_.push_back(std::move(event));
  while (events_.size() > 1'000) events_.pop_front();
  events_changed_.notify_all();
}

grpc::Status DroneApiServer::Connect(grpc::ServerContext*,
                                     const v1::ConnectRequest* request,
                                     v1::ConnectResponse* response) {
  if (request->client_id().empty() ||
      (!request->requested_api_major().empty() &&
       request->requested_api_major() != "1")) {
    return {grpc::StatusCode::INVALID_ARGUMENT,
            "client_id and compatible API major are required"};
  }
  const auto session_id = NewId();
  {
    std::lock_guard<std::mutex> lock(sessions_mutex_);
    sessions_[session_id] = request->client_id();
  }
  response->set_session_id(session_id);
  response->set_api_version("1.0.0-alpha.1");
  response->set_server_instance_id("icarus-local");
  response->set_server_time_unix_ms(clock_.NowUnixMs());
  return grpc::Status::OK;
}

grpc::Status DroneApiServer::CloseSession(
    grpc::ServerContext*, const v1::CloseSessionRequest* request, v1::Empty*) {
  std::lock_guard<std::mutex> lock(sessions_mutex_);
  if (sessions_.erase(request->session_id()) == 0) return InvalidSession();
  return grpc::Status::OK;
}

grpc::Status DroneApiServer::GetCapabilities(
    grpc::ServerContext*, const v1::GetCapabilitiesRequest* request,
    v1::VehicleCapabilities* response) {
  if (!SessionValid(request->session_id())) return InvalidSession();
  response->set_vehicle_id(request->vehicle_id());
  response->set_vehicle_type("MULTIROTOR");
  response->set_autopilot("ARDUPILOT_COPTER");
  response->set_autopilot_version("runtime-discovered");
  for (const auto* name : {"arm", "disarm", "takeoff", "goto", "route",
                           "hold", "return_home", "land", "orbit",
                           "state_stream", "event_stream"}) {
    AddCapability(response, name, true);
  }
  AddCapability(response, "perception", true, "1");
  AddCapability(response, "local_avoidance", true, "1");
  AddCapability(response, "precision_land", false, "reserved-phase-9");
  return grpc::Status::OK;
}

grpc::Status DroneApiServer::AcquireControl(
    grpc::ServerContext*, const v1::AcquireControlRequest* request,
    v1::ControlLeaseResponse* response) {
  std::string client_id;
  {
    std::lock_guard<std::mutex> lock(sessions_mutex_);
    const auto found = sessions_.find(request->session_id());
    if (found == sessions_.end()) return InvalidSession();
    client_id = found->second;
  }
  auto trusted = *request;
  trusted.set_client_id(client_id);
  *response = authority_.Acquire(trusted);
  return grpc::Status::OK;
}

grpc::Status DroneApiServer::RenewControl(
    grpc::ServerContext*, const v1::RenewControlRequest* request,
    v1::ControlLeaseResponse* response) {
  if (!SessionValid(request->session_id())) return InvalidSession();
  *response = authority_.Renew(*request);
  return grpc::Status::OK;
}

grpc::Status DroneApiServer::ReleaseControl(
    grpc::ServerContext*, const v1::ReleaseControlRequest* request, v1::Empty*) {
  if (!SessionValid(request->session_id())) return InvalidSession();
  if (!authority_.Release(*request)) {
    return {grpc::StatusCode::FAILED_PRECONDITION,
            "lease is absent or belongs to another session"};
  }
  return grpc::Status::OK;
}

grpc::Status DroneApiServer::GetState(grpc::ServerContext*,
                                      const v1::GetStateRequest* request,
                                      v1::DroneState* response) {
  if (!SessionValid(request->session_id())) return InvalidSession();
  const auto state = state_engine_.Snapshot();
  if (!state) {
    return {grpc::StatusCode::UNAVAILABLE, "vehicle state unavailable"};
  }
  *response = *state;
  if (const auto authority = authority_.Current()) {
    *response->mutable_authority() = *authority;
  }
  return grpc::Status::OK;
}

grpc::Status DroneApiServer::WatchState(
    grpc::ServerContext* context, const v1::WatchStateRequest* request,
    grpc::ServerWriter<v1::DroneState>* writer) {
  if (!SessionValid(request->session_id())) return InvalidSession();
  const auto rate = std::clamp(request->maximum_rate_hz(), 1U, 20U);
  std::uint64_t last_sequence = 0;
  while (!context->IsCancelled()) {
    const auto state = state_engine_.Snapshot();
    if (state && state->sequence() != last_sequence) {
      auto snapshot = *state;
      if (const auto authority = authority_.Current()) {
        *snapshot.mutable_authority() = *authority;
      }
      if (!writer->Write(snapshot)) break;
      last_sequence = state->sequence();
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(1'000 / rate));
  }
  return grpc::Status::OK;
}

grpc::Status DroneApiServer::GetHealth(grpc::ServerContext*,
                                       const v1::GetHealthRequest* request,
                                       v1::HealthReport* response) {
  if (!SessionValid(request->session_id())) return InvalidSession();
  *response = state_engine_.Health(guardrails_.policy().maximum_state_age_ms);
  return grpc::Status::OK;
}

grpc::Status DroneApiServer::WatchEvents(
    grpc::ServerContext* context, const v1::WatchEventsRequest* request,
    grpc::ServerWriter<v1::VehicleEvent>* writer) {
  if (!SessionValid(request->session_id())) return InvalidSession();
  std::size_t cursor = 0;
  while (!context->IsCancelled()) {
    std::unique_lock<std::mutex> lock(events_mutex_);
    events_changed_.wait_for(lock, std::chrono::milliseconds(250),
                             [this, cursor] { return events_.size() > cursor; });
    while (cursor < events_.size()) {
      const auto event = events_[cursor++];
      if (event.severity() >= request->minimum_severity()) {
        lock.unlock();
        if (!writer->Write(event)) return grpc::Status::OK;
        lock.lock();
      }
    }
  }
  return grpc::Status::OK;
}

grpc::Status DroneApiServer::GetPerception(
    grpc::ServerContext*, const v1::GetPerceptionRequest* request,
    v1::PerceptionSummary* response) {
  if (!SessionValid(request->session_id())) return InvalidSession();
  *response = perception_.Summary();
  return grpc::Status::OK;
}

grpc::Status DroneApiServer::WatchPerception(
    grpc::ServerContext* context, const v1::WatchPerceptionRequest* request,
    grpc::ServerWriter<v1::PerceptionSummary>* writer) {
  if (!SessionValid(request->session_id())) return InvalidSession();
  const auto rate = std::clamp(request->maximum_rate_hz(), 1U, 20U);
  std::uint64_t last_sequence = 0;
  while (!context->IsCancelled()) {
    const auto summary = perception_.Summary();
    if (summary.sequence() != last_sequence) {
      if (!writer->Write(summary)) break;
      last_sequence = summary.sequence();
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(1'000 / rate));
  }
  return grpc::Status::OK;
}

grpc::Status DroneApiServer::ValidateAction(
    grpc::ServerContext*, const v1::ValidateActionRequest* request,
    v1::ValidationResult* response) {
  if (!SessionValid(request->session_id())) return InvalidSession();
  const auto state = state_engine_.Snapshot();
  if (!state) {
    response->set_valid(false);
    response->set_reason_code(v1::REASON_CODE_STATE_STALE);
    response->set_message("vehicle state unavailable");
    return grpc::Status::OK;
  }
  // The guardrail layer performs the authoritative context extraction.
  bool authorized = false;
  const v1::CommandContext* command_context = nullptr;
  switch (request->command().request_case()) {
    case v1::ActionCommand::kArm:
      command_context = &request->command().arm().context();
      break;
    case v1::ActionCommand::kDisarm:
      command_context = &request->command().disarm().context();
      break;
    case v1::ActionCommand::kTakeoff:
      command_context = &request->command().takeoff().context();
      break;
    case v1::ActionCommand::kGoto:
      command_context = &request->command().goto_().context();
      break;
    case v1::ActionCommand::kExecuteRoute:
      command_context = &request->command().execute_route().context();
      break;
    case v1::ActionCommand::kHold:
      command_context = &request->command().hold().context();
      break;
    case v1::ActionCommand::kReturnHome:
      command_context = &request->command().return_home().context();
      break;
    case v1::ActionCommand::kLand:
      command_context = &request->command().land().context();
      break;
    case v1::ActionCommand::kOrbit:
      command_context = &request->command().orbit().context();
      break;
    default:
      break;
  }
  if (command_context == nullptr) {
    return {grpc::StatusCode::INVALID_ARGUMENT, "action is empty"};
  }
  if (!ContextValid(*command_context) ||
      command_context->session_id() != request->session_id()) {
    return InvalidSession();
  }
  authorized = authority_.IsAuthorized(*command_context);
  *response = guardrails_.Validate(request->command(), *state, authorized,
                                   clock_.NowUnixMs());
  return grpc::Status::OK;
}

#define ICARUS_SUBMIT_METHOD(Name, field, Type)                              \
  grpc::Status DroneApiServer::Name(grpc::ServerContext*,                   \
                                    const v1::Type* request,                \
                                    v1::ActionReceipt* response) {          \
    v1::ActionCommand command;                                               \
    *command.mutable_##field() = *request;                                  \
    return Submit(std::move(command), response);                             \
  }

ICARUS_SUBMIT_METHOD(Arm, arm, ArmRequest)
ICARUS_SUBMIT_METHOD(Disarm, disarm, DisarmRequest)
ICARUS_SUBMIT_METHOD(Takeoff, takeoff, TakeoffRequest)
ICARUS_SUBMIT_METHOD(Goto, goto_, GotoRequest)
ICARUS_SUBMIT_METHOD(ExecuteRoute, execute_route, ExecuteRouteRequest)
ICARUS_SUBMIT_METHOD(Hold, hold, HoldRequest)
ICARUS_SUBMIT_METHOD(ReturnHome, return_home, ReturnHomeRequest)
ICARUS_SUBMIT_METHOD(Land, land, LandRequest)
ICARUS_SUBMIT_METHOD(Orbit, orbit, OrbitRequest)

#undef ICARUS_SUBMIT_METHOD

grpc::Status DroneApiServer::CancelAction(
    grpc::ServerContext*, const v1::CancelActionRequest* request,
    v1::ActionReceipt* response) {
  if (!ContextValid(request->context())) return InvalidSession();
  *response = executor_.Cancel(*request);
  return grpc::Status::OK;
}

grpc::Status DroneApiServer::GetActionStatus(
    grpc::ServerContext*, const v1::GetActionStatusRequest* request,
    v1::ActionStatus* response) {
  if (!SessionValid(request->session_id())) return InvalidSession();
  const auto status = actions_.Find(request->action_id());
  if (!status) {
    return {grpc::StatusCode::NOT_FOUND, "action does not exist"};
  }
  *response = *status;
  return grpc::Status::OK;
}

grpc::Status DroneApiServer::WatchActionStatus(
    grpc::ServerContext* context,
    const v1::WatchActionStatusRequest* request,
    grpc::ServerWriter<v1::ActionStatus>* writer) {
  if (!SessionValid(request->session_id())) return InvalidSession();
  v1::ActionState last = v1::ACTION_STATE_UNSPECIFIED;
  while (!context->IsCancelled()) {
    const auto status = actions_.Find(request->action_id());
    if (!status) {
      return {grpc::StatusCode::NOT_FOUND, "action does not exist"};
    }
    if (status->state() != last) {
      if (!writer->Write(*status)) break;
      last = status->state();
    }
    if (Terminal(status->state())) break;
    std::this_thread::sleep_for(std::chrono::milliseconds(50));
  }
  return grpc::Status::OK;
}

bool DroneApiServer::SessionValid(const std::string& session_id) const {
  std::lock_guard<std::mutex> lock(sessions_mutex_);
  return !session_id.empty() && sessions_.find(session_id) != sessions_.end();
}

bool DroneApiServer::ContextValid(const v1::CommandContext& context) const {
  std::lock_guard<std::mutex> lock(sessions_mutex_);
  const auto found = sessions_.find(context.session_id());
  return found != sessions_.end() && found->second == context.client_id();
}

bool DroneApiServer::Terminal(v1::ActionState state) {
  return state == v1::ACTION_STATE_REJECTED ||
         state == v1::ACTION_STATE_SUCCEEDED ||
         state == v1::ACTION_STATE_CANCELLED ||
         state == v1::ACTION_STATE_TIMED_OUT ||
         state == v1::ACTION_STATE_FAILED ||
         state == v1::ACTION_STATE_PREEMPTED ||
         state == v1::ACTION_STATE_ABORTED_BY_SAFETY;
}

std::string DroneApiServer::NewId() {
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

grpc::Status DroneApiServer::Submit(v1::ActionCommand command,
                                    v1::ActionReceipt* response) {
  const v1::CommandContext* context = nullptr;
  switch (command.request_case()) {
    case v1::ActionCommand::kArm:
      context = &command.arm().context();
      break;
    case v1::ActionCommand::kDisarm:
      context = &command.disarm().context();
      break;
    case v1::ActionCommand::kTakeoff:
      context = &command.takeoff().context();
      break;
    case v1::ActionCommand::kGoto:
      context = &command.goto_().context();
      break;
    case v1::ActionCommand::kExecuteRoute:
      context = &command.execute_route().context();
      break;
    case v1::ActionCommand::kHold:
      context = &command.hold().context();
      break;
    case v1::ActionCommand::kReturnHome:
      context = &command.return_home().context();
      break;
    case v1::ActionCommand::kLand:
      context = &command.land().context();
      break;
    case v1::ActionCommand::kOrbit:
      context = &command.orbit().context();
      break;
    default:
      return {grpc::StatusCode::INVALID_ARGUMENT, "action is empty"};
  }
  if (!ContextValid(*context)) return InvalidSession();
  *response = executor_.Submit(command);
  return grpc::Status::OK;
}

}  // namespace icarus::drone_api

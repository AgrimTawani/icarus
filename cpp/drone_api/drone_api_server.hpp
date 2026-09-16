#pragma once

#include <condition_variable>
#include <deque>
#include <mutex>
#include <string>
#include <unordered_map>

#include <grpcpp/grpcpp.h>

#include "autonomy_core/clock.hpp"
#include "drone_api/authority_manager.hpp"
#include "guardrails/guardrails.hpp"
#include "icarus/v1/drone_api.grpc.pb.h"
#include "mission_executor/action_executor.hpp"
#include "mission_executor/action_store.hpp"
#include "state_engine/state_engine.hpp"
#include "perception/perception_engine.hpp"

namespace icarus::drone_api {

class DroneApiServer final : public v1::SessionService::Service,
                             public v1::AuthorityService::Service,
                             public v1::StateService::Service,
                             public v1::PerceptionService::Service,
                             public v1::ActionService::Service {
 public:
  DroneApiServer(const core::Clock& clock, AuthorityManager& authority,
                 state_engine::StateEngine& state_engine,
                 const guardrails::Guardrails& guardrails,
                 mission_executor::ActionStore& actions,
                 mission_executor::ActionExecutor& executor,
                 perception::PerceptionEngine& perception);

  void PublishEvent(v1::VehicleEvent event);

  grpc::Status Connect(grpc::ServerContext*, const v1::ConnectRequest*,
                       v1::ConnectResponse*) override;
  grpc::Status CloseSession(grpc::ServerContext*,
                            const v1::CloseSessionRequest*, v1::Empty*) override;
  grpc::Status GetCapabilities(grpc::ServerContext*,
                               const v1::GetCapabilitiesRequest*,
                               v1::VehicleCapabilities*) override;
  grpc::Status AcquireControl(grpc::ServerContext*,
                              const v1::AcquireControlRequest*,
                              v1::ControlLeaseResponse*) override;
  grpc::Status RenewControl(grpc::ServerContext*,
                            const v1::RenewControlRequest*,
                            v1::ControlLeaseResponse*) override;
  grpc::Status ReleaseControl(grpc::ServerContext*,
                              const v1::ReleaseControlRequest*,
                              v1::Empty*) override;
  grpc::Status GetState(grpc::ServerContext*, const v1::GetStateRequest*,
                        v1::DroneState*) override;
  grpc::Status WatchState(grpc::ServerContext*, const v1::WatchStateRequest*,
                          grpc::ServerWriter<v1::DroneState>*) override;
  grpc::Status GetHealth(grpc::ServerContext*, const v1::GetHealthRequest*,
                         v1::HealthReport*) override;
  grpc::Status WatchEvents(grpc::ServerContext*, const v1::WatchEventsRequest*,
                           grpc::ServerWriter<v1::VehicleEvent>*) override;
  grpc::Status GetPerception(grpc::ServerContext*,
                             const v1::GetPerceptionRequest*,
                             v1::PerceptionSummary*) override;
  grpc::Status WatchPerception(
      grpc::ServerContext*, const v1::WatchPerceptionRequest*,
      grpc::ServerWriter<v1::PerceptionSummary>*) override;
  grpc::Status ValidateAction(grpc::ServerContext*,
                              const v1::ValidateActionRequest*,
                              v1::ValidationResult*) override;
  grpc::Status Arm(grpc::ServerContext*, const v1::ArmRequest*,
                   v1::ActionReceipt*) override;
  grpc::Status Disarm(grpc::ServerContext*, const v1::DisarmRequest*,
                      v1::ActionReceipt*) override;
  grpc::Status Takeoff(grpc::ServerContext*, const v1::TakeoffRequest*,
                       v1::ActionReceipt*) override;
  grpc::Status Goto(grpc::ServerContext*, const v1::GotoRequest*,
                    v1::ActionReceipt*) override;
  grpc::Status ExecuteRoute(grpc::ServerContext*,
                            const v1::ExecuteRouteRequest*,
                            v1::ActionReceipt*) override;
  grpc::Status Hold(grpc::ServerContext*, const v1::HoldRequest*,
                    v1::ActionReceipt*) override;
  grpc::Status ReturnHome(grpc::ServerContext*,
                          const v1::ReturnHomeRequest*,
                          v1::ActionReceipt*) override;
  grpc::Status Land(grpc::ServerContext*, const v1::LandRequest*,
                    v1::ActionReceipt*) override;
  grpc::Status Orbit(grpc::ServerContext*, const v1::OrbitRequest*,
                     v1::ActionReceipt*) override;
  grpc::Status CancelAction(grpc::ServerContext*,
                            const v1::CancelActionRequest*,
                            v1::ActionReceipt*) override;
  grpc::Status GetActionStatus(grpc::ServerContext*,
                               const v1::GetActionStatusRequest*,
                               v1::ActionStatus*) override;
  grpc::Status WatchActionStatus(
      grpc::ServerContext*, const v1::WatchActionStatusRequest*,
      grpc::ServerWriter<v1::ActionStatus>*) override;

 private:
  [[nodiscard]] bool SessionValid(const std::string& session_id) const;
  [[nodiscard]] bool ContextValid(const v1::CommandContext& context) const;
  [[nodiscard]] static bool Terminal(v1::ActionState state);
  [[nodiscard]] static std::string NewId();
  grpc::Status Submit(v1::ActionCommand command, v1::ActionReceipt* response);

  const core::Clock& clock_;
  AuthorityManager& authority_;
  state_engine::StateEngine& state_engine_;
  const guardrails::Guardrails& guardrails_;
  mission_executor::ActionStore& actions_;
  mission_executor::ActionExecutor& executor_;
  perception::PerceptionEngine& perception_;
  mutable std::mutex sessions_mutex_;
  std::unordered_map<std::string, std::string> sessions_;
  mutable std::mutex events_mutex_;
  std::condition_variable events_changed_;
  std::deque<v1::VehicleEvent> events_;
  std::uint64_t next_event_id_ = 1;
};

}  // namespace icarus::drone_api

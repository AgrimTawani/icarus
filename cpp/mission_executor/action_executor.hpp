#pragma once

#include <atomic>
#include <condition_variable>
#include <cstdint>
#include <deque>
#include <functional>
#include <mutex>
#include <optional>
#include <string>
#include <thread>
#include <utility>
#include <unordered_set>

#include "autonomy_core/clock.hpp"
#include "drone_api/authority_manager.hpp"
#include "guardrails/guardrails.hpp"
#include "local_planner/local_planner.hpp"
#include "mavlink_gateway/mavlink_gateway.hpp"
#include "mission_executor/action_store.hpp"
#include "state_engine/state_engine.hpp"
#include "perception/perception_engine.hpp"

namespace icarus::mission_executor {

struct NativeRoutePlan {
  std::vector<mavlink_gateway::MissionItem> items;
  std::vector<std::uint16_t> waypoint_sequences;
};
// Convert metres in one measured local frame to mission coordinates. No
// language interpretation; geographic anchoring is captured once per segment.
[[nodiscard]] NativeRoutePlan BuildNativeRoute(
    const v1::ExecuteRouteRequest& route, const v1::DroneState& state);

inline constexpr int kOrbitSegmentsPerRevolution = 24;
// Allowance for LiDAR/position noise when an action starts inside the live
// clearance floor and may therefore only retreat.
inline constexpr double kClearanceToleranceM = 0.25;

// Waypoint layout for explicit viewpoint or geographic orbits: 24 per
// revolution, rounded up to place viewpoints exactly. Ordinary local orbits
// use the continuous position/velocity controller instead.
[[nodiscard]] int OrbitSegmentCount(const v1::OrbitRequest& orbit);

// Waypoint `index` (0..OrbitSegmentCount) of an orbit that enters the circle
// at `start_angle_rad` and sweeps `orbit.revolutions()` turns. Angles are NED
// azimuths (0 = north, +pi/2 = east), so increasing angle is clockwise seen
// from above; `orbit.clockwise()` selects that direction and false flies
// counter-clockwise.
[[nodiscard]] v1::Position OrbitWaypoint(const v1::OrbitRequest& orbit,
                                         double start_angle_rad, int index);

// 1-based viewpoint number of waypoint `index`, or 0 when it is not one. A
// closed orbit spaces N viewpoints 360/N apart starting at the entry point; a
// partial arc spaces them evenly including both ends.
[[nodiscard]] int OrbitViewpointAt(const v1::OrbitRequest& orbit, int index);

// Absolute heading in [0, 360) degrees (0 = north, 90 = east) that looks
// horizontally from `from` toward `target`; nullopt for mixed frames or when
// the two are too close for the heading to be stable.
[[nodiscard]] std::optional<double> HeadingTowardDeg(const v1::Position& from,
                                                     const v1::Position& target);

// Live clearance floor of a motion action that must keep `required_m` and has
// seen at best `best_observed_m` so far. An action that began inside the
// floor is held to what it had (less a noise allowance): it may retreat, but
// never close in.
[[nodiscard]] double EnforcedClearanceM(double required_m,
                                        double best_observed_m);

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
      const std::string& completion_message, bool complete_action = true,
      bool passive_ground_hold = false);
  bool WaitForLanding(const std::string& action_id,
                      const v1::CommandContext& context,
                      std::uint32_t timeout_ms, const std::string& message);
  bool ExecuteGoto(const std::string& action_id,
                   const v1::CommandContext& context,
                   const v1::Position& destination,
                   double acceptance_radius_m, std::uint32_t timeout_ms,
                   bool complete_action = true,
                   double requested_clearance_m = 0.0,
                   std::optional<double> heading_deg = std::nullopt,
                   double ground_speed_mps = 3.0);
  bool ExecuteContinuousOrbit(const std::string& action_id,
                              const v1::OrbitRequest& orbit,
                              double start_angle,
                              const v1::Position& altitude_position,
                              std::optional<double> heading_deg);
  bool ExecuteNativeRoute(const std::string& action_id,
                          const v1::ExecuteRouteRequest& route);
  bool ApplyHeading(const std::string& action_id,
                    const v1::CommandContext& context,
                    const v1::HeadingPolicy& heading,
                    const v1::Position& from);
  bool CommandHeading(const std::string& action_id, double heading_deg);
  bool HoldViewpoint(const std::string& action_id,
                     const v1::CommandContext& context,
                     const v1::Position& point,
                     std::optional<double> heading_deg,
                     std::uint32_t dwell_ms, int number, int total,
                     double progress);
  void BeginClearanceGuard(const v1::ActionCommand& command);
  [[nodiscard]] std::optional<std::pair<v1::ReasonCode, std::string>>
  ClearanceViolation();
  [[nodiscard]] bool SetMode(const std::string& action_id,
                             std::uint32_t mode,
                             const std::string& name);
  [[nodiscard]] static const v1::CommandContext& ContextOf(
      const v1::ActionCommand& command);
  [[nodiscard]] static double GeoDistanceM(const v1::GeoPosition& first,
                                           const v1::GeoPosition& second);
  [[nodiscard]] static bool Reached(const v1::Position& target,
                                    const v1::DroneState& state,
                                    double acceptance_radius_m);

  // Live LiDAR clearance floor of the running goto/route/orbit/yaw action;
  // inactive for every other action. Touched only by the worker thread.
  struct ClearanceGuard {
    bool active = false;
    double required_m = 0.0;
    double best_observed_m = -1.0;
  };

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
  std::mutex cancellation_mutex_;
  std::condition_variable cancellation_changed_;
  std::unordered_set<std::string> cancellation_pending_;
  std::mutex queue_mutex_;
  std::condition_variable queue_changed_;
  std::deque<std::pair<std::string, v1::ActionCommand>> queue_;
  ClearanceGuard clearance_;
  bool navigation_prepared_ = false;  // worker-owned, reset for each action
};

}  // namespace icarus::mission_executor

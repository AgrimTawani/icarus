#include <csignal>
#include <chrono>
#include <filesystem>
#include <iostream>
#include <memory>
#include <string>
#include <thread>

#include <grpcpp/grpcpp.h>

#include "autonomy_core/clock.hpp"
#include "drone_api/authority_manager.hpp"
#include "drone_api/drone_api_server.hpp"
#include "guardrails/guardrails.hpp"
#include "mavlink_gateway/mavlink_gateway.hpp"
#include "mission_executor/action_executor.hpp"
#include "mission_executor/action_store.hpp"
#include "safety_supervisor/safety_supervisor.hpp"
#include "state_engine/state_engine.hpp"
#include "local_planner/local_planner.hpp"
#include "perception/lidar/gazebo_lidar_adapter.hpp"
#include "perception/perception_engine.hpp"

namespace {

volatile std::sig_atomic_t stop_requested = 0;

void RequestStop(int) { stop_requested = 1; }

}  // namespace

int main(int argc, char** argv) {
  std::string listen = "127.0.0.1:50051";
  std::string mavlink_host = "127.0.0.1";
  std::uint16_t mavlink_port = 5760;
  std::filesystem::path policy = "config/safety/v1.yaml";
  std::string test_mavlink_fault_mode;
  std::uint32_t test_mavlink_fault_after_ms = 0;
  std::uint32_t test_mavlink_fault_duration_ms = 0;
  std::uint32_t test_mavlink_fault_latency_ms = 0;
  double simulation_battery_percent = -1.0;
  for (int index = 1; index < argc; ++index) {
    const std::string argument = argv[index];
    if (argument == "--listen" && index + 1 < argc) {
      listen = argv[++index];
    } else if (argument == "--mavlink-host" && index + 1 < argc) {
      mavlink_host = argv[++index];
    } else if (argument == "--mavlink-port" && index + 1 < argc) {
      mavlink_port = static_cast<std::uint16_t>(std::stoi(argv[++index]));
    } else if (argument == "--safety-policy" && index + 1 < argc) {
      policy = argv[++index];
    } else if (argument == "--test-mavlink-fault-mode" && index + 1 < argc) {
      test_mavlink_fault_mode = argv[++index];
    } else if (argument == "--test-mavlink-fault-after-ms" && index + 1 < argc) {
      test_mavlink_fault_after_ms = static_cast<std::uint32_t>(std::stoul(argv[++index]));
    } else if (argument == "--test-mavlink-fault-duration-ms" && index + 1 < argc) {
      test_mavlink_fault_duration_ms = static_cast<std::uint32_t>(std::stoul(argv[++index]));
    } else if (argument == "--test-mavlink-fault-latency-ms" && index + 1 < argc) {
      test_mavlink_fault_latency_ms = static_cast<std::uint32_t>(std::stoul(argv[++index]));
    } else if (argument == "--simulation-battery-percent" && index + 1 < argc) {
      simulation_battery_percent = std::stod(argv[++index]);
    } else {
      std::cerr << "Usage: icarus-drone-api [--listen address] "
                   "[--mavlink-host host] [--mavlink-port port] "
                   "[--safety-policy path] [--test-mavlink-fault-mode loss|delay] "
                   "[--test-mavlink-fault-after-ms ms] "
                   "[--test-mavlink-fault-duration-ms ms] "
                   "[--test-mavlink-fault-latency-ms ms] "
                   "[--simulation-battery-percent 0..100]\n";
      return 2;
    }
  }

  try {
    icarus::core::SystemClock clock;
    const auto safety_policy =
        icarus::guardrails::SafetyPolicy::Load(policy);
    icarus::guardrails::Guardrails guardrails(safety_policy);
    icarus::state_engine::StateEngine state_engine(clock);
    icarus::drone_api::AuthorityManager authority(clock, 30'000);
    icarus::mission_executor::ActionStore actions(clock);
    icarus::mavlink_gateway::ArdupilotGateway gateway;
    if (simulation_battery_percent >= 0.0) {
      gateway.ConfigureSimulationBatteryPercent(simulation_battery_percent);
    }

    gateway.SetStateCallback([&state_engine](const icarus::v1::DroneState& state) {
      state_engine.Publish(state);
    });
    if (!gateway.Connect(mavlink_host, mavlink_port)) {
      std::cerr << "Unable to connect to MAVLink at " << mavlink_host << ':'
                << mavlink_port << '\n';
      return 1;
    }
    if (!gateway.WaitForReady(45'000)) {
      std::cerr << "Timed out waiting for ArduPilot heartbeat and position\n";
      return 1;
    }

    icarus::perception::PerceptionEngine perception(
        clock, state_engine, safety_policy.maximum_perception_age_ms);
    icarus::perception::GazeboLidarAdapter lidar;
    if (!lidar.Start([&perception](const icarus::perception::RangeScan& scan) {
          perception.Ingest(scan);
        })) {
      std::cerr << "Unable to subscribe to the normalized lidar source\n";
      return 1;
    }
    const auto perception_deadline =
        std::chrono::steady_clock::now() + std::chrono::seconds(15);
    while (!perception.Healthy() &&
           std::chrono::steady_clock::now() < perception_deadline) {
      std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
    if (!perception.Healthy()) {
      std::cerr << "Timed out waiting for fresh lidar perception\n";
      return 1;
    }
    icarus::local_planner::LocalPlanner planner(
        1.0, safety_policy.minimum_obstacle_clearance_m);

    icarus::mission_executor::ActionExecutor executor(
        clock, actions, state_engine, authority, guardrails, gateway,
        &perception, &planner);
    icarus::safety_supervisor::SafetySupervisor safety_supervisor(
        clock, safety_policy, state_engine, actions, gateway, &perception);
    icarus::drone_api::DroneApiServer services(
        clock, authority, state_engine, guardrails, actions, executor,
        perception);
    gateway.SetEventCallback(
        [&services](icarus::v1::VehicleEvent event) {
          services.PublishEvent(std::move(event));
        });

    grpc::ServerBuilder builder;
    builder.AddListeningPort(listen, grpc::InsecureServerCredentials());
    builder.RegisterService(
        static_cast<icarus::v1::SessionService::Service*>(&services));
    builder.RegisterService(
        static_cast<icarus::v1::AuthorityService::Service*>(&services));
    builder.RegisterService(
        static_cast<icarus::v1::StateService::Service*>(&services));
    builder.RegisterService(
        static_cast<icarus::v1::PerceptionService::Service*>(&services));
    builder.RegisterService(
        static_cast<icarus::v1::ActionService::Service*>(&services));
    auto server = builder.BuildAndStart();
    if (!server) {
      std::cerr << "Unable to start Drone API at " << listen << '\n';
      return 1;
    }
    // Start a simulator fault window only after the full API is ready. Starting
    // it after the MAVLink handshake would let CMake/protobuf/perception setup
    // consume the entire configured interval before a flight client connects.
    const bool test_fault_requested = !test_mavlink_fault_mode.empty();
    if (test_fault_requested && test_mavlink_fault_duration_ms == 0) {
      std::cerr << "A simulator MAVLink test fault needs a duration\n";
      return 2;
    }
    if (test_fault_requested) {
      gateway.ConfigureTestLinkFault(
          test_mavlink_fault_mode, test_mavlink_fault_after_ms,
          test_mavlink_fault_duration_ms, test_mavlink_fault_latency_ms);
    }
    std::signal(SIGINT, RequestStop);
    std::signal(SIGTERM, RequestStop);
    executor.Start();
    safety_supervisor.Start();
    std::thread shutdown_thread([&server] {
      while (!stop_requested) {
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
      }
      server->Shutdown();
    });
    std::cout << "ICARUS_DRONE_API_READY " << listen << '\n' << std::flush;
    server->Wait();
    stop_requested = 1;
    shutdown_thread.join();
    safety_supervisor.Stop();
    executor.Stop();
    lidar.Stop();
    gateway.Disconnect();
  } catch (const std::exception& error) {
    std::cerr << "Drone API failure: " << error.what() << '\n';
    return 1;
  }
  return 0;
}

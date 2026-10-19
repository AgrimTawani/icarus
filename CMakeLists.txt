cmake_minimum_required(VERSION 3.16)
project(icarus LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

find_package(Protobuf REQUIRED)
find_package(gRPC CONFIG REQUIRED)
find_package(OpenSSL REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(gz-transport13 REQUIRED)
find_package(gz-msgs10 REQUIRED)
find_package(Threads REQUIRED)

set(ICARUS_GENERATED_DIR "${CMAKE_CURRENT_SOURCE_DIR}/build/generated/cpp")
if(NOT EXISTS "${ICARUS_GENERATED_DIR}/icarus/v1/drone_api.grpc.pb.cc")
  message(FATAL_ERROR "Generated protobuf files are missing; run ./scripts/generate-proto")
endif()

add_library(icarus_proto
  "${ICARUS_GENERATED_DIR}/icarus/v1/action.pb.cc"
  "${ICARUS_GENERATED_DIR}/icarus/v1/drone_api.pb.cc"
  "${ICARUS_GENERATED_DIR}/icarus/v1/drone_api.grpc.pb.cc"
  "${ICARUS_GENERATED_DIR}/icarus/v1/perception.pb.cc"
  "${ICARUS_GENERATED_DIR}/icarus/v1/state.pb.cc"
)
target_include_directories(icarus_proto PUBLIC "${ICARUS_GENERATED_DIR}")
target_link_libraries(icarus_proto PUBLIC protobuf::libprotobuf gRPC::grpc++)

add_library(icarus_phase8_core
  cpp/autonomy_core/clock.cpp
  cpp/drone_api/authority_manager.cpp
  cpp/guardrails/guardrails.cpp
  cpp/mission_executor/action_store.cpp
  cpp/state_engine/state_engine.cpp
  cpp/local_planner/local_planner.cpp
  perception/obstacle_map/obstacle_map.cpp
  perception/perception_engine.cpp
)
target_include_directories(icarus_phase8_core PUBLIC cpp "${CMAKE_CURRENT_SOURCE_DIR}")
target_link_libraries(icarus_phase8_core PUBLIC icarus_proto yaml-cpp OpenSSL::Crypto)
target_compile_options(icarus_phase8_core PRIVATE -Wall -Wextra -Wpedantic -Werror)

add_executable(icarus-replay-guardrails cpp/guardrails/replay_main.cpp)
target_include_directories(icarus-replay-guardrails PRIVATE cpp)
target_link_libraries(icarus-replay-guardrails PRIVATE icarus_phase8_core)
target_compile_options(icarus-replay-guardrails PRIVATE
  -Wall -Wextra -Wpedantic -Werror)

add_executable(icarus-check-guardrail cpp/guardrails/check_main.cpp)
target_include_directories(icarus-check-guardrail PRIVATE cpp)
target_link_libraries(icarus-check-guardrail PRIVATE icarus_phase8_core)
target_compile_options(icarus-check-guardrail PRIVATE
  -Wall -Wextra -Wpedantic -Werror)

set(ICARUS_MAVLINK_INCLUDE
  "${CMAKE_CURRENT_SOURCE_DIR}/third_party/ardupilot/build/sitl/libraries/GCS_MAVLink/include/mavlink/v2.0")
if(EXISTS "${ICARUS_MAVLINK_INCLUDE}/ardupilotmega/mavlink.h")
  add_library(icarus_mavlink_gateway cpp/mavlink_gateway/mavlink_gateway.cpp)
  target_include_directories(icarus_mavlink_gateway SYSTEM PRIVATE
    "${ICARUS_MAVLINK_INCLUDE}")
  target_include_directories(icarus_mavlink_gateway PUBLIC cpp)
  target_link_libraries(icarus_mavlink_gateway PUBLIC icarus_phase8_core)
  target_compile_options(icarus_mavlink_gateway PRIVATE
    -Wall -Wextra -Wpedantic -Werror -Wno-address-of-packed-member)

  add_library(icarus_mission_runtime
    cpp/mission_executor/action_executor.cpp
    cpp/safety_supervisor/safety_supervisor.cpp
    perception/lidar/gazebo_lidar_adapter.cpp
    perception/lidar/airsim_lidar_adapter.cpp)
  target_include_directories(icarus_mission_runtime PUBLIC cpp)
  target_link_libraries(icarus_mission_runtime PUBLIC icarus_mavlink_gateway
    gz-transport13::gz-transport13 gz-msgs10::gz-msgs10 Threads::Threads)
  target_compile_options(icarus_mission_runtime PRIVATE
    -Wall -Wextra -Wpedantic -Werror)

  add_executable(icarus-drone-api
    cpp/drone_api/drone_api_server.cpp
    cpp/drone_api/main.cpp)
  target_include_directories(icarus-drone-api PRIVATE cpp)
  target_link_libraries(icarus-drone-api PRIVATE icarus_mission_runtime)
  target_compile_options(icarus-drone-api PRIVATE
    -Wall -Wextra -Wpedantic -Werror)
endif()

include(CTest)
if(BUILD_TESTING)
  add_executable(phase8_core_tests tests/unit/cpp/phase8_core_tests.cpp)
  target_link_libraries(phase8_core_tests PRIVATE icarus_mission_runtime)
  target_compile_definitions(phase8_core_tests PRIVATE
    ICARUS_SOURCE_DIR="${CMAKE_CURRENT_SOURCE_DIR}")
  target_compile_options(phase8_core_tests PRIVATE -Wall -Wextra -Wpedantic -Werror)
  add_test(NAME phase8_core_tests COMMAND phase8_core_tests)
  add_executable(mavlink_gateway_lifecycle_tests
    tests/unit/cpp/mavlink_gateway_lifecycle_tests.cpp)
  target_link_libraries(mavlink_gateway_lifecycle_tests PRIVATE
    icarus_mavlink_gateway Threads::Threads)
  target_compile_options(mavlink_gateway_lifecycle_tests PRIVATE
    -Wall -Wextra -Wpedantic -Werror)
  add_test(NAME mavlink_gateway_lifecycle_tests COMMAND mavlink_gateway_lifecycle_tests)
  set_tests_properties(mavlink_gateway_lifecycle_tests PROPERTIES TIMEOUT 10)

  add_executable(native_route_tests tests/unit/cpp/native_route_tests.cpp)
  target_link_libraries(native_route_tests PRIVATE icarus_mission_runtime)
  target_compile_definitions(native_route_tests PRIVATE ICARUS_SOURCE_DIR="${CMAKE_CURRENT_SOURCE_DIR}")
  target_compile_options(native_route_tests PRIVATE -Wall -Wextra -Wpedantic -Werror)
  add_test(NAME native_route_tests COMMAND native_route_tests)
  set_tests_properties(native_route_tests PROPERTIES TIMEOUT 20)

  add_executable(mavlink_mission_tests tests/unit/cpp/mavlink_mission_tests.cpp)
  target_include_directories(mavlink_mission_tests SYSTEM PRIVATE "${ICARUS_MAVLINK_INCLUDE}")
  target_link_libraries(mavlink_mission_tests PRIVATE icarus_mavlink_gateway Threads::Threads)
  target_compile_options(mavlink_mission_tests PRIVATE -Wall -Wextra -Wpedantic -Werror -Wno-address-of-packed-member)
  add_test(NAME mavlink_mission_tests COMMAND mavlink_mission_tests)
  set_tests_properties(mavlink_mission_tests PROPERTIES TIMEOUT 20)

  add_executable(phase9_perception_tests
    tests/unit/cpp/phase9_perception_tests.cpp)
  target_link_libraries(phase9_perception_tests PRIVATE icarus_phase8_core)
  target_sources(phase9_perception_tests PRIVATE
    perception/lidar/airsim_lidar_adapter.cpp)
  target_link_libraries(phase9_perception_tests PRIVATE Threads::Threads)
  target_compile_options(phase9_perception_tests PRIVATE
    -Wall -Wextra -Wpedantic -Werror)
  add_test(NAME phase9_perception_tests COMMAND phase9_perception_tests)
endif()

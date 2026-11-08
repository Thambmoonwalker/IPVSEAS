# IPVSEAS Part 1: road-user and obstacle collision prediction (C++ core)

add_library(ipvseas_part1 STATIC
    tracking/kalman_ca.cpp
    tracking/hungarian.cpp
    tracking/tracker.cpp
    behaviour_prediction/trajectory_predictor.cpp
    collision_prediction/collision_risk.cpp
    collision_prediction/warning_manager.cpp
    lidar/obstacle_detector.cpp
    sensor_fusion/camera_lidar_fusion.cpp
    sensor_fusion/radar_fusion.cpp
    radar/radar_processor.cpp
    synthetic/scenarios.cpp
)
target_include_directories(ipvseas_part1 PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})
target_link_libraries(ipvseas_part1 PUBLIC Eigen3::Eigen)
target_compile_features(ipvseas_part1 PUBLIC cxx_std_17)
if(MSVC)
    target_compile_options(ipvseas_part1 PRIVATE /W4 /permissive- /utf-8)
    target_compile_definitions(ipvseas_part1 PUBLIC NOMINMAX)
else()
    target_compile_options(ipvseas_part1 PRIVATE -Wall -Wextra -Wpedantic)
endif()

# Synthetic scenario runner: prints the acceptance table, writes per-frame CSVs.
add_executable(part1_scenarios tools/scenario_runner.cpp)
target_link_libraries(part1_scenarios PRIVATE ipvseas_part1)

# Tests
add_executable(part1_tests tests/test_part1.cpp)
target_link_libraries(part1_tests PRIVATE ipvseas_part1)
add_test(NAME part1_tests COMMAND part1_tests)
add_test(NAME part1_scenarios_robustness COMMAND part1_scenarios --seeds 20)

# Replay a recorded run folder (CARLA or synthetic) through the full pipeline.
add_executable(part1_replay tools/replay_run.cpp)
target_link_libraries(part1_replay PRIVATE ipvseas_part1)

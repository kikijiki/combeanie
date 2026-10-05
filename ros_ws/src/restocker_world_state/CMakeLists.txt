# Copyright 2026 Matteo Bernacchia <dev@kikijiki.com>
# SPDX-License-Identifier: MIT

cmake_minimum_required(VERSION 3.16)
project(restocker_world_state VERSION 0.1.0 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

find_package(ament_cmake REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(rcl_interfaces REQUIRED)
find_package(rclcpp REQUIRED)
find_package(restocker_interfaces REQUIRED)
find_package(yaml-cpp REQUIRED)

add_library(world_state src/world_state.cpp src/clock_authority.cpp)
add_library(restocker_world_state::world_state ALIAS world_state)
target_include_directories(
  world_state
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)
target_link_libraries(world_state PUBLIC Eigen3::Eigen rclcpp::rclcpp)
target_compile_options(world_state PRIVATE -Wall -Wextra -Wpedantic)

add_library(lane_config src/lane_config.cpp)
add_library(restocker_world_state::lane_config ALIAS lane_config)
target_include_directories(
  lane_config
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)
target_link_libraries(lane_config PUBLIC world_state PRIVATE yaml-cpp::yaml-cpp)
target_compile_options(lane_config PRIVATE -Wall -Wextra -Wpedantic)

add_library(ros_conversions src/ros_conversions.cpp)
add_library(restocker_world_state::ros_conversions ALIAS ros_conversions)
target_include_directories(
  ros_conversions
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(ros_conversions restocker_interfaces)
target_link_libraries(ros_conversions world_state)
target_compile_options(ros_conversions PRIVATE -Wall -Wextra -Wpedantic)

add_executable(world_state_node src/world_state_node.cpp)
ament_target_dependencies(world_state_node rcl_interfaces rclcpp restocker_interfaces)
target_link_libraries(world_state_node lane_config ros_conversions world_state)
target_compile_options(world_state_node PRIVATE -Wall -Wextra -Wpedantic)

install(DIRECTORY config DESTINATION share/${PROJECT_NAME})
install(DIRECTORY include/ DESTINATION include)
install(
  TARGETS world_state lane_config ros_conversions world_state_node
  EXPORT export_restocker_world_state
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
  INCLUDES DESTINATION include
)

ament_export_targets(export_restocker_world_state HAS_LIBRARY_TARGET)
ament_export_dependencies(Eigen3 rcl_interfaces rclcpp restocker_interfaces yaml-cpp)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  find_package(ament_lint_auto REQUIRED)
  find_package(launch_testing_ament_cmake REQUIRED)
  # Launch tests lease their ROS domain and Gazebo partition at run time.
  # See cmake/restocker_test_isolation.cmake.
  include(${CMAKE_CURRENT_SOURCE_DIR}/../../../cmake/restocker_test_isolation.cmake)

  ament_add_gtest(test_world_state test/test_world_state.cpp)
  target_link_libraries(test_world_state restocker_world_state::world_state)
  target_compile_options(test_world_state PRIVATE -Wall -Wextra -Wpedantic)

  ament_add_gtest(test_task_reservations test/test_task_reservations.cpp)
  target_link_libraries(test_task_reservations restocker_world_state::world_state)
  target_compile_options(test_task_reservations PRIVATE -Wall -Wextra -Wpedantic)

  ament_add_gtest(test_attachment_temporal_admission test/test_attachment_temporal_admission.cpp)
  target_link_libraries(test_attachment_temporal_admission restocker_world_state::ros_conversions)
  target_compile_options(test_attachment_temporal_admission PRIVATE -Wall -Wextra -Wpedantic)

  ament_add_gtest(test_clock_authority test/test_clock_authority.cpp)
  target_link_libraries(test_clock_authority restocker_world_state::world_state)
  target_compile_options(test_clock_authority PRIVATE -Wall -Wextra -Wpedantic)

  ament_add_gtest(test_ros_conversions test/test_ros_conversions.cpp)
  target_link_libraries(test_ros_conversions restocker_world_state::ros_conversions)
  target_compile_options(test_ros_conversions PRIVATE -Wall -Wextra -Wpedantic)

  ament_add_gtest(test_lane_ledger test/test_lane_ledger.cpp)
  target_link_libraries(
    test_lane_ledger
    restocker_world_state::world_state
    restocker_world_state::lane_config
  )
  target_compile_options(test_lane_ledger PRIVATE -Wall -Wextra -Wpedantic)

  ament_add_gtest(test_lane_config test/test_lane_config.cpp)
  target_link_libraries(test_lane_config restocker_world_state::lane_config)
  target_compile_definitions(
    test_lane_config
    PRIVATE
      RESTOCKER_TEST_LANE_SEMANTICS="${CMAKE_CURRENT_SOURCE_DIR}/config/baseline_lanes.yaml"
      RESTOCKER_TEST_SAME_SHAPE_LANE_SEMANTICS="${CMAKE_CURRENT_SOURCE_DIR}/config/same_shape_lanes.yaml"
      RESTOCKER_TEST_WORKCELL_GEOMETRY="${CMAKE_CURRENT_SOURCE_DIR}/../restocker_description/config/workcell_geometry.yaml"
      RESTOCKER_TEST_PRODUCT_CATALOG="${CMAKE_CURRENT_SOURCE_DIR}/../restocker_description/config/product_collision_catalog.yaml"
  )
  target_compile_options(test_lane_config PRIVATE -Wall -Wextra -Wpedantic)

  ament_add_gtest(test_lane_policy test/test_lane_policy.cpp)
  target_link_libraries(
    test_lane_policy
    restocker_world_state::world_state
    restocker_world_state::lane_config
  )
  target_compile_options(test_lane_policy PRIVATE -Wall -Wextra -Wpedantic)

  add_launch_test(
    test/test_world_state_node_telemetry.py
    TIMEOUT 60
  )

  add_launch_test(
    test/test_world_state_node_object_observations.py
    TIMEOUT 60
  )

  add_launch_test(
    test/test_world_state_node_clock.py
    TIMEOUT 90
  )

  ament_lint_auto_find_test_dependencies()
endif()

ament_package()

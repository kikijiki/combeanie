# Copyright 2026 Matteo Bernacchia <dev@kikijiki.com>
# SPDX-License-Identifier: MIT

cmake_minimum_required(VERSION 3.16)
project(restocker_control VERSION 0.1.0 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

find_package(ament_cmake REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(control_msgs REQUIRED)
find_package(controller_manager_msgs REQUIRED)
find_package(rcl_interfaces REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(restocker_interfaces REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(trajectory_msgs REQUIRED)

add_library(control_contract src/control_contract.cpp)
target_include_directories(
  control_contract
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)
target_compile_options(control_contract PRIVATE -Wall -Wextra -Wpedantic)

add_library(controller_contract_probe src/controller_contract_probe.cpp)
target_include_directories(
  controller_contract_probe
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)
target_link_libraries(controller_contract_probe PUBLIC control_contract)
target_compile_options(controller_contract_probe PRIVATE -Wall -Wextra -Wpedantic)

add_library(controller_contract_probe_ros src/controller_contract_probe_ros.cpp)
target_include_directories(
  controller_contract_probe_ros
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)
target_link_libraries(controller_contract_probe_ros controller_contract_probe)
ament_target_dependencies(
  controller_contract_probe_ros
  controller_manager_msgs
  rclcpp
)
target_compile_options(controller_contract_probe_ros PRIVATE -Wall -Wextra -Wpedantic)

add_library(smoke_trajectory_contract src/smoke_trajectory_contract.cpp)
target_include_directories(
  smoke_trajectory_contract
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)
target_link_libraries(smoke_trajectory_contract PUBLIC control_contract)
target_compile_options(smoke_trajectory_contract PRIVATE -Wall -Wextra -Wpedantic)

add_library(execution_controller_authority src/execution_controller_authority.cpp)
add_library(
  restocker_control::execution_controller_authority ALIAS execution_controller_authority)
target_include_directories(
  execution_controller_authority
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)
target_compile_options(execution_controller_authority PRIVATE -Wall -Wextra -Wpedantic)
ament_target_dependencies(execution_controller_authority rclcpp)

add_library(joint_state_telemetry src/joint_state_telemetry.cpp)
target_include_directories(
  joint_state_telemetry
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(
  joint_state_telemetry
  builtin_interfaces
  restocker_interfaces
  sensor_msgs
)
target_compile_options(joint_state_telemetry PRIVATE -Wall -Wextra -Wpedantic)

add_executable(joint_state_telemetry_adapter src/joint_state_telemetry_adapter.cpp)
target_link_libraries(joint_state_telemetry_adapter joint_state_telemetry)
ament_target_dependencies(
  joint_state_telemetry_adapter
  rcl_interfaces
  rclcpp
  restocker_interfaces
  sensor_msgs
)
target_compile_options(joint_state_telemetry_adapter PRIVATE -Wall -Wextra -Wpedantic)

add_executable(control_smoke_test src/control_smoke_test.cpp)
target_link_libraries(
  control_smoke_test
  controller_contract_probe_ros
  smoke_trajectory_contract
)
target_compile_options(control_smoke_test PRIVATE -Wall -Wextra -Wpedantic)
ament_target_dependencies(
  control_smoke_test
  control_msgs
  controller_manager_msgs
  rclcpp
  rclcpp_action
  sensor_msgs
  trajectory_msgs
)

install(DIRECTORY config DESTINATION share/${PROJECT_NAME})
install(DIRECTORY include/ DESTINATION include)
install(
  TARGETS
    control_contract
    controller_contract_probe
    controller_contract_probe_ros
    execution_controller_authority
    joint_state_telemetry
    smoke_trajectory_contract
  EXPORT export_restocker_control
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
)
install(
  TARGETS control_smoke_test joint_state_telemetry_adapter
  RUNTIME DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  find_package(ament_lint_auto REQUIRED)
  find_package(launch_testing_ament_cmake REQUIRED)
  # Launch tests lease a ROS domain and Gazebo partition at run time so concurrent runs do not
  # see each other's nodes. See cmake/restocker_test_isolation.cmake.
  include(${CMAKE_CURRENT_SOURCE_DIR}/../../../cmake/restocker_test_isolation.cmake)

  ament_add_gtest(test_control_contract test/test_control_contract.cpp)
  target_link_libraries(test_control_contract control_contract)
  target_compile_options(test_control_contract PRIVATE -Wall -Wextra -Wpedantic)

  ament_add_gtest(
    test_smoke_trajectory_contract test/test_smoke_trajectory_contract.cpp)
  target_link_libraries(test_smoke_trajectory_contract smoke_trajectory_contract)
  target_compile_options(
    test_smoke_trajectory_contract PRIVATE -Wall -Wextra -Wpedantic)

  ament_add_gtest(
    test_controller_contract_probe test/test_controller_contract_probe.cpp)
  target_link_libraries(test_controller_contract_probe controller_contract_probe)
  target_compile_options(
    test_controller_contract_probe PRIVATE -Wall -Wextra -Wpedantic)

  ament_add_gtest(
    test_controller_contract_probe_ros test/test_controller_contract_probe_ros.cpp)
  target_link_libraries(test_controller_contract_probe_ros controller_contract_probe_ros)
  ament_target_dependencies(
    test_controller_contract_probe_ros
    controller_manager_msgs
    rclcpp
  )
  target_compile_options(
    test_controller_contract_probe_ros PRIVATE -Wall -Wextra -Wpedantic)

  ament_add_gtest(
    test_execution_controller_authority test/test_execution_controller_authority.cpp)
  target_link_libraries(
    test_execution_controller_authority restocker_control::execution_controller_authority)
  target_compile_options(
    test_execution_controller_authority PRIVATE -Wall -Wextra -Wpedantic)

  ament_add_gtest(test_joint_state_telemetry test/test_joint_state_telemetry.cpp)
  target_link_libraries(test_joint_state_telemetry joint_state_telemetry)
  target_compile_options(test_joint_state_telemetry PRIVATE -Wall -Wextra -Wpedantic)

  add_launch_test(
    test/test_joint_state_telemetry_adapter.py
    TIMEOUT 60
  )
  add_launch_test(
    test/test_control_smoke_initial_state.py
    TIMEOUT 60
  )

  ament_lint_auto_find_test_dependencies()
endif()

ament_export_targets(export_restocker_control HAS_LIBRARY_TARGET)
ament_export_dependencies(controller_manager_msgs rclcpp)
ament_package()

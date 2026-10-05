# Copyright 2026 Matteo Bernacchia <dev@kikijiki.com>
# SPDX-License-Identifier: MIT

cmake_minimum_required(VERSION 3.16)
project(restocker_task_executor VERSION 0.1.0 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

find_package(ament_cmake REQUIRED)
find_package(action_msgs REQUIRED)
find_package(control_msgs REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(moveit_core REQUIRED)
find_package(moveit_msgs REQUIRED)
find_package(moveit_ros_planning REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(rcl_interfaces REQUIRED)
find_package(restocker_control REQUIRED)
find_package(restocker_interfaces REQUIRED)
find_package(restocker_perception REQUIRED)
find_package(restocker_reasoner REQUIRED)
find_package(restocker_world_state REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(shape_msgs REQUIRED)
find_package(tf2_eigen REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(trajectory_msgs REQUIRED)
find_package(Threads REQUIRED)
find_package(urdf REQUIRED)
find_package(yaml-cpp REQUIRED)

add_library(coordinator_process_control src/coordinator_process_control.cpp)
add_library(
  restocker_task_executor::coordinator_process_control ALIAS coordinator_process_control)
target_include_directories(
  coordinator_process_control
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)
target_compile_options(coordinator_process_control PRIVATE -Wall -Wextra -Wpedantic)

add_library(coordinator_steady_clock src/coordinator_steady_clock.cpp)
add_library(restocker_task_executor::coordinator_steady_clock ALIAS coordinator_steady_clock)
target_include_directories(
  coordinator_steady_clock
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)
target_compile_options(coordinator_steady_clock PRIVATE -Wall -Wextra -Wpedantic)

add_library(
  coordinator_async_steady_clock_failure_latch
  src/coordinator_async_steady_clock_failure_latch.cpp)
add_library(
  restocker_task_executor::coordinator_async_steady_clock_failure_latch ALIAS
  coordinator_async_steady_clock_failure_latch)
target_include_directories(
  coordinator_async_steady_clock_failure_latch
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)
target_compile_options(
  coordinator_async_steady_clock_failure_latch PRIVATE -Wall -Wextra -Wpedantic)
target_link_libraries(
  coordinator_async_steady_clock_failure_latch PUBLIC coordinator_steady_clock)

add_library(coordinator_startup src/coordinator_startup.cpp)
add_library(restocker_task_executor::coordinator_startup ALIAS coordinator_startup)
target_include_directories(
  coordinator_startup
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)
target_compile_options(coordinator_startup PRIVATE -Wall -Wextra -Wpedantic)
target_link_libraries(
  coordinator_startup
  PUBLIC restock_coordinator_primitives restocker_world_state::world_state)

add_library(coordinator_signal_waiter src/termination_signal_waiter.cpp)
target_include_directories(
  coordinator_signal_waiter PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/src>)
target_compile_options(coordinator_signal_waiter PRIVATE -Wall -Wextra -Wpedantic)
target_link_libraries(coordinator_signal_waiter PUBLIC Threads::Threads)

add_library(coordinator_startup_transport src/coordinator_startup_transport.cpp)
add_library(
  restocker_task_executor::coordinator_startup_transport ALIAS coordinator_startup_transport)
target_include_directories(
  coordinator_startup_transport
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)
target_compile_options(coordinator_startup_transport PRIVATE -Wall -Wextra -Wpedantic)
target_link_libraries(
  coordinator_startup_transport coordinator_startup world_state_async_port)
ament_target_dependencies(coordinator_startup_transport restocker_interfaces)

add_library(attachment_transaction_machine src/attachment_transaction_machine.cpp)
add_library(
  restocker_task_executor::attachment_transaction_machine ALIAS attachment_transaction_machine)
target_include_directories(
  attachment_transaction_machine
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)
target_compile_options(attachment_transaction_machine PRIVATE -Wall -Wextra -Wpedantic)

add_library(restock_task_machine src/restock_task_machine.cpp)
add_library(restocker_task_executor::restock_task_machine ALIAS restock_task_machine)
target_include_directories(
  restock_task_machine
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)
target_compile_options(restock_task_machine PRIVATE -Wall -Wextra -Wpedantic)

add_library(motion_operation_evidence src/motion_operation_evidence.cpp)
add_library(
  restocker_task_executor::motion_operation_evidence ALIAS motion_operation_evidence)
target_include_directories(
  motion_operation_evidence
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)
target_compile_options(motion_operation_evidence PRIVATE -Wall -Wextra -Wpedantic)
target_link_libraries(motion_operation_evidence PUBLIC restock_task_machine)

add_library(goal_termination_authority src/goal_termination_authority.cpp)
add_library(
  restocker_task_executor::goal_termination_authority ALIAS goal_termination_authority)
target_include_directories(
  goal_termination_authority
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)
target_compile_options(goal_termination_authority PRIVATE -Wall -Wextra -Wpedantic)
target_link_libraries(goal_termination_authority PUBLIC motion_operation_evidence)
add_library(restock_coordinator_primitives src/restock_coordinator_primitives.cpp)
add_library(
  restocker_task_executor::restock_coordinator_primitives ALIAS restock_coordinator_primitives)
target_include_directories(
  restock_coordinator_primitives
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)
target_compile_options(restock_coordinator_primitives PRIVATE -Wall -Wextra -Wpedantic)
target_link_libraries(
  restock_coordinator_primitives PUBLIC goal_termination_authority motion_operation_evidence)

add_library(restock_goal_context src/restock_goal_context.cpp)
add_library(restocker_task_executor::restock_goal_context ALIAS restock_goal_context)
target_include_directories(
  restock_goal_context
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)
target_compile_options(restock_goal_context PRIVATE -Wall -Wextra -Wpedantic)
target_link_libraries(
  restock_goal_context
  grasp_candidates placement_candidates pregrasp_planning_authority
  restock_coordinator_primitives world_state_port_contract)
ament_target_dependencies(restock_goal_context rclcpp restocker_interfaces)

add_library(manipulation_geometry src/manipulation_geometry.cpp)
add_library(restocker_task_executor::manipulation_geometry ALIAS manipulation_geometry)
target_include_directories(
  manipulation_geometry
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)
target_compile_options(manipulation_geometry PRIVATE -Wall -Wextra -Wpedantic)
target_link_libraries(manipulation_geometry PUBLIC Eigen3::Eigen PRIVATE yaml-cpp::yaml-cpp)

add_library(planning_contract src/planning_contract.cpp)
target_include_directories(
  planning_contract
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)
target_compile_options(planning_contract PRIVATE -Wall -Wextra -Wpedantic)
ament_target_dependencies(planning_contract moveit_msgs)

# Card 053: the pure decision layer of the free-space race (Milestone 10 §6).
add_library(free_space_race src/free_space_race.cpp)
add_library(restocker_task_executor::free_space_race ALIAS free_space_race)
target_include_directories(
  free_space_race
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)
target_compile_options(free_space_race PRIVATE -Wall -Wextra -Wpedantic)
target_link_libraries(free_space_race planning_contract)
ament_target_dependencies(free_space_race moveit_msgs)

# Card 068: the race's local scene = move_group's collision model (Milestone 10 §6).
add_library(race_planning_scene src/race_planning_scene.cpp)
add_library(restocker_task_executor::race_planning_scene ALIAS race_planning_scene)
target_include_directories(
  race_planning_scene
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)
target_compile_options(race_planning_scene PRIVATE -Wall -Wextra -Wpedantic)
ament_target_dependencies(race_planning_scene moveit_core moveit_msgs)

add_library(trajectory_scene_validation src/trajectory_scene_validation.cpp)
add_library(restocker_task_executor::trajectory_scene_validation ALIAS trajectory_scene_validation)
target_include_directories(
  trajectory_scene_validation PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>)
target_compile_options(trajectory_scene_validation PRIVATE -Wall -Wextra -Wpedantic)
target_link_libraries(trajectory_scene_validation race_planning_scene pregrasp_planning_authority)
ament_target_dependencies(trajectory_scene_validation moveit_core moveit_msgs)

add_library(attachment_port src/attachment_port.cpp)
add_library(restocker_task_executor::attachment_port ALIAS attachment_port)
target_include_directories(
  attachment_port
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)
target_compile_options(attachment_port PRIVATE -Wall -Wextra -Wpedantic)
target_link_libraries(attachment_port PUBLIC restock_coordinator_primitives Eigen3::Eigen)

add_library(ros_attachment_port src/ros_attachment_port.cpp)
add_library(restocker_task_executor::ros_attachment_port ALIAS ros_attachment_port)
target_include_directories(
  ros_attachment_port
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)
target_compile_options(ros_attachment_port PRIVATE -Wall -Wextra -Wpedantic)
target_link_libraries(ros_attachment_port attachment_port)
ament_target_dependencies(
  ros_attachment_port geometry_msgs rclcpp restocker_interfaces tf2_eigen)

add_library(gripper_port src/gripper_port.cpp)
add_library(restocker_task_executor::gripper_port ALIAS gripper_port)
target_include_directories(
  gripper_port
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)
target_compile_options(gripper_port PRIVATE -Wall -Wextra -Wpedantic)
target_link_libraries(gripper_port PUBLIC restock_coordinator_primitives)

add_library(ros_gripper_port src/ros_gripper_port.cpp)
add_library(restocker_task_executor::ros_gripper_port ALIAS ros_gripper_port)
target_include_directories(
  ros_gripper_port
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)
target_compile_options(ros_gripper_port PRIVATE -Wall -Wextra -Wpedantic)
target_link_libraries(ros_gripper_port gripper_port)
ament_target_dependencies(
  ros_gripper_port control_msgs rclcpp rclcpp_action sensor_msgs trajectory_msgs)

add_library(motion_port src/motion_port.cpp)
add_library(restocker_task_executor::motion_port ALIAS motion_port)
target_include_directories(
  motion_port
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)
target_compile_options(motion_port PRIVATE -Wall -Wextra -Wpedantic)
target_link_libraries(motion_port PUBLIC restock_coordinator_primitives Eigen3::Eigen)

add_library(transfer_path_constraints src/transfer_path_constraints.cpp)
add_library(restocker_task_executor::transfer_path_constraints ALIAS transfer_path_constraints)
target_include_directories(
  transfer_path_constraints
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)
target_compile_options(transfer_path_constraints PRIVATE -Wall -Wextra -Wpedantic)
target_link_libraries(
  transfer_path_constraints PUBLIC motion_port manipulation_geometry Eigen3::Eigen)

add_library(recovery_advice_gate src/recovery_advice_gate.cpp)
add_library(restocker_task_executor::recovery_advice_gate ALIAS recovery_advice_gate)
target_include_directories(
  recovery_advice_gate
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)
target_compile_options(recovery_advice_gate PRIVATE -Wall -Wextra -Wpedantic)
ament_target_dependencies(recovery_advice_gate restocker_reasoner)

add_library(recovery_classification src/recovery_classification.cpp)
add_library(
  restocker_task_executor::recovery_classification ALIAS recovery_classification)
target_include_directories(
  recovery_classification
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)
target_compile_options(recovery_classification PRIVATE -Wall -Wextra -Wpedantic)

# Milestone 10 §6, Card 052: the campaign-side survey/confirm failure evidence →
# RecoveryContext mapping. Pure, so the fail-closed rules are unit-testable without ROS.
add_library(survey_recovery_context src/survey_recovery_context.cpp)
add_library(
  restocker_task_executor::survey_recovery_context ALIAS survey_recovery_context)
target_include_directories(
  survey_recovery_context
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)
target_compile_options(survey_recovery_context PRIVATE -Wall -Wextra -Wpedantic)
target_link_libraries(survey_recovery_context PUBLIC recovery_classification)

add_library(moveit_path_constraint_conversion src/moveit_path_constraint_conversion.cpp)
add_library(
  restocker_task_executor::moveit_path_constraint_conversion ALIAS
  moveit_path_constraint_conversion)
target_include_directories(
  moveit_path_constraint_conversion
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)
target_compile_options(moveit_path_constraint_conversion PRIVATE -Wall -Wextra -Wpedantic)
target_link_libraries(moveit_path_constraint_conversion motion_port)
ament_target_dependencies(moveit_path_constraint_conversion moveit_msgs shape_msgs geometry_msgs)

# Card 062: the grasp escape's straight line on a caller-owned scene (Milestone 10 §6).
add_library(grasp_escape_planner src/grasp_escape_planner.cpp)
add_library(restocker_task_executor::grasp_escape_planner ALIAS grasp_escape_planner)
target_include_directories(
  grasp_escape_planner
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)
target_compile_options(grasp_escape_planner PRIVATE -Wall -Wextra -Wpedantic)
target_link_libraries(grasp_escape_planner motion_port race_planning_scene)
ament_target_dependencies(grasp_escape_planner moveit_core moveit_msgs)

# Card 061: dividing a planned trajectory into bounded stop-to-stop controller goals
# (Milestone 10 §6), separate from the port so it is testable against MoveIt's own TOTG.
add_library(controller_trajectory_slicing src/controller_trajectory_slicing.cpp)
add_library(
  restocker_task_executor::controller_trajectory_slicing ALIAS controller_trajectory_slicing)
target_include_directories(
  controller_trajectory_slicing
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)
target_compile_options(controller_trajectory_slicing PRIVATE -Wall -Wextra -Wpedantic)
target_link_libraries(controller_trajectory_slicing planning_contract)
ament_target_dependencies(controller_trajectory_slicing moveit_core moveit_msgs)

# Card 046: the linear-path refinement ladder (Milestone 10 §6) — a partial straight line is
# retried at a finer Cartesian step inside the segment's planning budget before it is refused.
add_library(linear_path_refinement src/linear_path_refinement.cpp)
add_library(restocker_task_executor::linear_path_refinement ALIAS linear_path_refinement)
target_include_directories(
  linear_path_refinement
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)
target_compile_options(linear_path_refinement PRIVATE -Wall -Wextra -Wpedantic)
ament_target_dependencies(linear_path_refinement moveit_msgs)

add_library(moveit_motion_port src/moveit_motion_port.cpp)
add_library(restocker_task_executor::moveit_motion_port ALIAS moveit_motion_port)
target_include_directories(
  moveit_motion_port
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)
target_compile_options(moveit_motion_port PRIVATE -Wall -Wextra -Wpedantic)
target_link_libraries(
  moveit_motion_port motion_port moveit_path_constraint_conversion planning_contract
  motion_execute_verdict
  free_space_race pregrasp_planning_authority grasp_escape_planner
  race_planning_scene controller_trajectory_slicing linear_path_refinement
  trajectory_scene_validation)
ament_target_dependencies(
  moveit_motion_port
  control_msgs
  geometry_msgs
  moveit_core
  moveit_msgs
  moveit_ros_planning
  moveit_ros_planning_interface
  rcl_interfaces
  rclcpp
  rclcpp_action
  restocker_interfaces
  sensor_msgs
  shape_msgs
  tf2_eigen
  urdf)

add_library(pregrasp_planning_authority src/pregrasp_planning_authority.cpp)
add_library(
  restocker_task_executor::pregrasp_planning_authority ALIAS pregrasp_planning_authority)
target_include_directories(
  pregrasp_planning_authority
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)
target_compile_options(pregrasp_planning_authority PRIVATE -Wall -Wextra -Wpedantic)
target_link_libraries(pregrasp_planning_authority restocker_world_state::world_state)
ament_target_dependencies(pregrasp_planning_authority rclcpp restocker_interfaces)

add_library(scene_geometry src/scene_geometry.cpp)
add_library(restocker_task_executor::scene_geometry ALIAS scene_geometry)
target_include_directories(
  scene_geometry
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)
target_compile_options(scene_geometry PRIVATE -Wall -Wextra -Wpedantic)
ament_target_dependencies(scene_geometry geometry_msgs moveit_msgs shape_msgs urdf)
target_link_libraries(scene_geometry Eigen3::Eigen yaml-cpp::yaml-cpp)

add_library(task_selection src/task_selection.cpp)
add_library(restocker_task_executor::task_selection ALIAS task_selection)
target_include_directories(
  task_selection
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)
target_compile_options(task_selection PRIVATE -Wall -Wextra -Wpedantic)
target_link_libraries(
  task_selection
  PUBLIC manipulation_geometry scene_geometry restocker_world_state::world_state)

add_library(action_client_finality src/action_client_finality.cpp)
add_library(restocker_task_executor::action_client_finality ALIAS action_client_finality)
target_include_directories(
  action_client_finality
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)
target_compile_options(action_client_finality PRIVATE -Wall -Wextra -Wpedantic)
ament_target_dependencies(action_client_finality action_msgs rclcpp rclcpp_action)

# Card 086 stage 1 (CMB-SPEC-13): the campaign's pure unresolved-motion attempt registry.
add_library(unresolved_motion src/unresolved_motion.cpp)
add_library(restocker_task_executor::unresolved_motion ALIAS unresolved_motion)
target_include_directories(
  unresolved_motion
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)
target_compile_options(unresolved_motion PRIVATE -Wall -Wextra -Wpedantic)

# Card 086 stage 1: what each endpoint's delivered terminal proves, pinned against the real
# action messages by test_campaign_motion_evidence.
add_library(campaign_motion_evidence src/campaign_motion_evidence.cpp)
add_library(restocker_task_executor::campaign_motion_evidence ALIAS campaign_motion_evidence)
target_include_directories(
  campaign_motion_evidence
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)
target_compile_options(campaign_motion_evidence PRIVATE -Wall -Wextra -Wpedantic)
target_link_libraries(campaign_motion_evidence unresolved_motion)
ament_target_dependencies(campaign_motion_evidence rclcpp_action restocker_interfaces)

# Stage 1a (Card 086): the MotionOutcome and terminal-stop claim a failed MoveIt execute() maps
# to, shared by the MoveIt port and test_motion_execute_verdict.
add_library(motion_execute_verdict src/motion_execute_verdict.cpp)
add_library(restocker_task_executor::motion_execute_verdict ALIAS motion_execute_verdict)
target_include_directories(
  motion_execute_verdict
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)
target_compile_options(motion_execute_verdict PRIVATE -Wall -Wextra -Wpedantic)
target_link_libraries(motion_execute_verdict PUBLIC planning_contract motion_port)

add_library(restock_action_contract src/restock_action_contract.cpp)
add_library(restocker_task_executor::restock_action_contract ALIAS restock_action_contract)
target_include_directories(
  restock_action_contract
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)
target_compile_options(restock_action_contract PRIVATE -Wall -Wextra -Wpedantic)
target_link_libraries(restock_action_contract restock_task_machine task_selection)
ament_target_dependencies(restock_action_contract restocker_interfaces)

add_library(world_state_port_contract src/world_state_port_contract.cpp)
add_library(restocker_task_executor::world_state_port_contract ALIAS world_state_port_contract)
target_include_directories(
  world_state_port_contract
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)
target_compile_options(world_state_port_contract PRIVATE -Wall -Wextra -Wpedantic)
target_link_libraries(
  world_state_port_contract task_selection restocker_world_state::ros_conversions)
ament_target_dependencies(world_state_port_contract restocker_interfaces)

add_library(world_state_async_port src/world_state_async_port.cpp)
add_library(restocker_task_executor::world_state_async_port ALIAS world_state_async_port)
target_include_directories(
  world_state_async_port
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)
target_compile_options(world_state_async_port PRIVATE -Wall -Wextra -Wpedantic)
target_link_libraries(world_state_async_port restock_coordinator_primitives)
ament_target_dependencies(world_state_async_port rclcpp restocker_interfaces)

add_library(restock_coordinator_inbox src/restock_coordinator_inbox.cpp)
add_library(restocker_task_executor::restock_coordinator_inbox ALIAS restock_coordinator_inbox)
target_include_directories(
  restock_coordinator_inbox
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)
target_compile_options(restock_coordinator_inbox PRIVATE -Wall -Wextra -Wpedantic)
target_link_libraries(
  restock_coordinator_inbox
  attachment_port gripper_port motion_port restock_coordinator_primitives task_selection)
ament_target_dependencies(
  restock_coordinator_inbox moveit_msgs rclcpp restocker_interfaces)

add_library(coordinator_termination_router src/coordinator_termination_router.cpp)
add_library(
  restocker_task_executor::coordinator_termination_router ALIAS coordinator_termination_router)
target_include_directories(
  coordinator_termination_router
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)
target_compile_options(coordinator_termination_router PRIVATE -Wall -Wextra -Wpedantic)
target_link_libraries(
  coordinator_termination_router
  PUBLIC restock_coordinator_inbox restock_coordinator_primitives)

add_library(restock_coordinator_driver src/restock_coordinator_driver.cpp)
add_library(restocker_task_executor::restock_coordinator_driver ALIAS restock_coordinator_driver)
target_include_directories(
  restock_coordinator_driver
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)
target_compile_options(restock_coordinator_driver PRIVATE -Wall -Wextra -Wpedantic)
target_link_libraries(
  restock_coordinator_driver
  coordinator_termination_router
  recovery_advice_gate
  recovery_classification
  restock_action_contract
  restock_coordinator_inbox
  restock_goal_context
  transfer_path_constraints
  world_state_async_port
  restocker_world_state::ros_conversions
)
ament_target_dependencies(
  restock_coordinator_driver rclcpp rclcpp_action restocker_interfaces restocker_reasoner)

add_library(action_result_publisher src/action_result_publisher.cpp)
add_library(restocker_task_executor::action_result_publisher ALIAS action_result_publisher)
target_include_directories(
  action_result_publisher
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)
target_compile_options(action_result_publisher PRIVATE -Wall -Wextra -Wpedantic)
ament_target_dependencies(action_result_publisher rclcpp_action restocker_interfaces)

add_library(coordinator_active_fault_epoch src/coordinator_active_fault_epoch.cpp)
add_library(
  restocker_task_executor::coordinator_active_fault_epoch ALIAS coordinator_active_fault_epoch)
target_include_directories(
  coordinator_active_fault_epoch
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)
target_compile_options(coordinator_active_fault_epoch PRIVATE -Wall -Wextra -Wpedantic)
target_link_libraries(coordinator_active_fault_epoch PUBLIC restock_action_contract)
get_target_property(
  coordinator_active_fault_epoch_links
  coordinator_active_fault_epoch
  LINK_LIBRARIES)
if("restock_coordinator_driver" IN_LIST coordinator_active_fault_epoch_links)
  message(FATAL_ERROR "coordinator_active_fault_epoch must not depend on the driver")
endif()

add_library(coordinator_generation_quiescence src/coordinator_generation_quiescence.cpp)
add_library(
  restocker_task_executor::coordinator_generation_quiescence ALIAS
  coordinator_generation_quiescence)
target_include_directories(
  coordinator_generation_quiescence
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)
target_compile_options(coordinator_generation_quiescence PRIVATE -Wall -Wextra -Wpedantic)
target_link_libraries(
  coordinator_generation_quiescence PUBLIC coordinator_active_fault_epoch)
target_link_libraries(restock_coordinator_driver coordinator_generation_quiescence)

add_library(coordinator_pump_lease_gate src/coordinator_pump_lease_gate.cpp)
add_library(
  restocker_task_executor::coordinator_pump_lease_gate ALIAS coordinator_pump_lease_gate)
target_include_directories(
  coordinator_pump_lease_gate
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)
target_compile_options(coordinator_pump_lease_gate PRIVATE -Wall -Wextra -Wpedantic)

add_library(coordinator_handoff_policy INTERFACE)
add_library(
  restocker_task_executor::coordinator_handoff_policy ALIAS coordinator_handoff_policy)
target_include_directories(
  coordinator_handoff_policy
  INTERFACE
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)
target_link_libraries(
  coordinator_handoff_policy INTERFACE coordinator_pump_lease_gate)

add_library(coordinator_handoff_reducer src/coordinator_handoff_reducer.cpp)
add_library(
  restocker_task_executor::coordinator_handoff_reducer ALIAS coordinator_handoff_reducer)
target_include_directories(
  coordinator_handoff_reducer
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)
target_compile_options(coordinator_handoff_reducer PRIVATE -Wall -Wextra -Wpedantic)
target_link_libraries(coordinator_handoff_reducer PUBLIC coordinator_handoff_policy)

add_library(pending_activation_validator src/detail/pending_activation_validator.cpp)
target_include_directories(
  pending_activation_validator
  PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/include ${CMAKE_CURRENT_SOURCE_DIR}/src)
target_compile_options(pending_activation_validator PRIVATE -Wall -Wextra -Wpedantic)
target_link_libraries(
  pending_activation_validator
  PUBLIC restock_coordinator_primitives coordinator_handoff_reducer)

add_library(restock_action_coordinator_node src/restock_action_coordinator_node.cpp)
add_library(
  restocker_task_executor::restock_action_coordinator_node ALIAS restock_action_coordinator_node)
target_include_directories(
  restock_action_coordinator_node
  PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/src
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)
target_compile_options(restock_action_coordinator_node PRIVATE -Wall -Wextra -Wpedantic)
target_link_libraries(
  restock_action_coordinator_node
  action_result_publisher
  coordinator_active_fault_epoch
  coordinator_async_steady_clock_failure_latch
  coordinator_generation_quiescence
  coordinator_handoff_reducer
  coordinator_pump_lease_gate
  pending_activation_validator
  coordinator_steady_clock
  coordinator_startup_transport
  coordinator_termination_router
  gripper_geometry
  moveit_motion_port
  placement_candidates
  ros_attachment_port
  ros_gripper_port
  restock_coordinator_driver
  manipulation_geometry
  scene_geometry
  task_selection
  restocker_perception::viewpoint
)
ament_target_dependencies(
  restock_action_coordinator_node
  rclcpp
  rclcpp_action
  rcl_interfaces
  restocker_interfaces
  restocker_perception
  restocker_reasoner
  sensor_msgs
  tf2_eigen
  tf2_ros
)

add_executable(restock_action_coordinator src/restock_action_coordinator_main.cpp)
target_compile_options(restock_action_coordinator PRIVATE -Wall -Wextra -Wpedantic)
target_link_libraries(
  restock_action_coordinator
  coordinator_process_control
  coordinator_signal_waiter
  restock_action_coordinator_node)
ament_target_dependencies(restock_action_coordinator rclcpp)

add_library(grasp_candidates src/grasp_candidates.cpp)
add_library(restocker_task_executor::grasp_candidates ALIAS grasp_candidates)
target_include_directories(
  grasp_candidates
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)
target_compile_options(grasp_candidates PRIVATE -Wall -Wextra -Wpedantic)
target_link_libraries(grasp_candidates PUBLIC task_selection)

add_library(gripper_geometry src/gripper_geometry.cpp)
add_library(restocker_task_executor::gripper_geometry ALIAS gripper_geometry)
target_include_directories(
  gripper_geometry
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)
target_compile_options(gripper_geometry PRIVATE -Wall -Wextra -Wpedantic)
target_link_libraries(
  gripper_geometry PUBLIC grasp_candidates Eigen3::Eigen PRIVATE yaml-cpp::yaml-cpp)

add_library(placement_candidates src/placement_candidates.cpp)
add_library(restocker_task_executor::placement_candidates ALIAS placement_candidates)
target_include_directories(
  placement_candidates
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)
target_compile_options(placement_candidates PRIVATE -Wall -Wextra -Wpedantic)
target_link_libraries(placement_candidates PUBLIC task_selection)

add_library(world_snapshot_projection src/world_snapshot_projection.cpp)
add_library(restocker_task_executor::world_snapshot_projection ALIAS world_snapshot_projection)
target_include_directories(
  world_snapshot_projection
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)
target_compile_options(world_snapshot_projection PRIVATE -Wall -Wextra -Wpedantic)
target_link_libraries(world_snapshot_projection scene_geometry)
ament_target_dependencies(world_snapshot_projection restocker_interfaces)

add_library(planning_scene_reconciliation src/planning_scene_reconciliation.cpp)
add_library(
  restocker_task_executor::planning_scene_reconciliation
  ALIAS planning_scene_reconciliation
)
target_include_directories(
  planning_scene_reconciliation
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)
target_compile_options(planning_scene_reconciliation PRIVATE -Wall -Wextra -Wpedantic)
target_link_libraries(planning_scene_reconciliation world_snapshot_projection)
ament_target_dependencies(planning_scene_reconciliation moveit_msgs shape_msgs)

add_library(obstacle_projection src/obstacle_projection.cpp)
add_library(restocker_task_executor::obstacle_projection ALIAS obstacle_projection)
target_include_directories(
  obstacle_projection
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)
target_compile_options(obstacle_projection PRIVATE -Wall -Wextra -Wpedantic)
target_link_libraries(obstacle_projection world_snapshot_projection Eigen3::Eigen)
ament_target_dependencies(
  obstacle_projection geometry_msgs moveit_msgs restocker_interfaces shape_msgs)

add_library(planning_scene_attachment src/planning_scene_attachment.cpp)
add_library(restocker_task_executor::planning_scene_attachment ALIAS planning_scene_attachment)
target_include_directories(
  planning_scene_attachment
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)
target_compile_options(planning_scene_attachment PRIVATE -Wall -Wextra -Wpedantic)
target_link_libraries(planning_scene_attachment planning_scene_reconciliation)
ament_target_dependencies(planning_scene_attachment geometry_msgs moveit_msgs shape_msgs)

add_library(planning_scene_lease src/planning_scene_lease.cpp)
add_library(restocker_task_executor::planning_scene_lease ALIAS planning_scene_lease)
target_include_directories(
  planning_scene_lease
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)
target_compile_options(planning_scene_lease PRIVATE -Wall -Wextra -Wpedantic)

add_library(planning_scene_lease_ros src/planning_scene_lease_ros.cpp)
add_library(restocker_task_executor::planning_scene_lease_ros ALIAS planning_scene_lease_ros)
target_include_directories(
  planning_scene_lease_ros
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)
target_compile_options(planning_scene_lease_ros PRIVATE -Wall -Wextra -Wpedantic)
target_link_libraries(planning_scene_lease_ros planning_scene_lease)
ament_target_dependencies(planning_scene_lease_ros rclcpp restocker_interfaces)

add_executable(planning_scene_projector_node src/planning_scene_projector_node.cpp)
target_link_libraries(
  planning_scene_projector_node planning_scene_attachment planning_scene_reconciliation
  planning_scene_lease_ros obstacle_projection manipulation_geometry)
target_compile_options(planning_scene_projector_node PRIVATE -Wall -Wextra -Wpedantic)
ament_target_dependencies(
  planning_scene_projector_node
  moveit_msgs
  rclcpp
  restocker_interfaces
  tf2_eigen
  tf2_ros
)

# --- Aiming the wrist camera -------------------------------------------------------------------

# One survey station visit: a ViewpointPort resolution, then an ordinary kFreeSpace motion segment.
# Adds no authority; the segment goes through the same planning-scene gate as any other.
add_library(viewpoint_survey src/viewpoint_survey.cpp)
add_library(restocker_task_executor::viewpoint_survey ALIAS viewpoint_survey)
target_include_directories(
  viewpoint_survey
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)
target_link_libraries(
  viewpoint_survey motion_port restocker_perception::viewpoint)
target_compile_options(viewpoint_survey PRIVATE -Wall -Wextra -Wpedantic)

add_library(survey_submit_refusal src/survey_submit_refusal.cpp)
add_library(restocker_task_executor::survey_submit_refusal ALIAS survey_submit_refusal)
target_include_directories(
  survey_submit_refusal
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)
target_compile_options(survey_submit_refusal PRIVATE -Wall -Wextra -Wpedantic)
target_link_libraries(survey_submit_refusal viewpoint_survey)
ament_target_dependencies(survey_submit_refusal restocker_interfaces)

add_executable(survey_viewpoint_node src/survey_viewpoint_node.cpp)
target_link_libraries(
  survey_viewpoint_node viewpoint_survey survey_submit_refusal moveit_motion_port
  restocker_perception::tf_viewpoint_port)
target_compile_options(survey_viewpoint_node PRIVATE -Wall -Wextra -Wpedantic)
ament_target_dependencies(
  survey_viewpoint_node
  geometry_msgs
  rclcpp
  rclcpp_action
  restocker_interfaces
  tf2_eigen
  tf2_ros
)

add_executable(lane_survey_node src/lane_survey_node.cpp)
target_link_libraries(
  lane_survey_node viewpoint_survey survey_submit_refusal moveit_motion_port
  restocker_perception::tf_viewpoint_port)
target_compile_options(lane_survey_node PRIVATE -Wall -Wextra -Wpedantic)
ament_target_dependencies(
  lane_survey_node
  geometry_msgs
  rclcpp
  rclcpp_action
  restocker_interfaces
  tf2_eigen
  tf2_ros
)

# Coordinated tray survey. The logic is a pure library over the duty streams; the server
# orchestrates existing /survey_viewpoint legs and publishes nothing itself.
add_library(tray_survey_logic src/tray_survey_logic.cpp)
add_library(restocker_task_executor::tray_survey_logic ALIAS tray_survey_logic)
target_include_directories(
  tray_survey_logic
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)
target_compile_options(tray_survey_logic PRIVATE -Wall -Wextra -Wpedantic)
ament_target_dependencies(tray_survey_logic restocker_interfaces)
target_link_libraries(tray_survey_logic Eigen3::Eigen restocker_perception::viewpoint)

add_library(tray_survey_server src/tray_survey_node.cpp)
add_library(restocker_task_executor::tray_survey_server ALIAS tray_survey_server)
target_include_directories(
  tray_survey_server
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)
target_compile_options(tray_survey_server PRIVATE -Wall -Wextra -Wpedantic)
target_link_libraries(
  tray_survey_server tray_survey_logic restocker_perception::viewpoint Eigen3::Eigen)
ament_target_dependencies(
  tray_survey_server
  geometry_msgs
  rclcpp
  rclcpp_action
  restocker_interfaces
  sensor_msgs
  tf2_eigen
  tf2_ros
)

add_executable(tray_survey_node src/tray_survey_node_main.cpp)
target_link_libraries(tray_survey_node tray_survey_server)
target_compile_options(tray_survey_node PRIVATE -Wall -Wextra -Wpedantic)
ament_target_dependencies(tray_survey_node rclcpp)

# Continuous driver above the three authority boundaries. Owns no mutable world-state authority
# (the survey actions and single-transfer action remain the boundaries); its read-only snapshot
# is decoded through the world state's own converter and measured with the same lane_deficit the
# coordinator's selection orders pairs with.
add_executable(autonomous_restock_campaign src/autonomous_restock_campaign_node.cpp)
target_link_libraries(
  autonomous_restock_campaign
  action_client_finality
  campaign_motion_evidence
  unresolved_motion
  manipulation_geometry
  scene_geometry
  task_selection
  survey_recovery_context
  tray_survey_logic
  restocker_world_state::ros_conversions
  restocker_perception::viewpoint)
target_compile_options(autonomous_restock_campaign PRIVATE -Wall -Wextra -Wpedantic)
ament_target_dependencies(
  autonomous_restock_campaign
  rclcpp
  rclcpp_action
  restocker_interfaces
  restocker_perception
  shape_msgs
)

add_executable(planning_smoke_test src/planning_smoke_test.cpp)
target_link_libraries(planning_smoke_test planning_contract)
target_compile_options(planning_smoke_test PRIVATE -Wall -Wextra -Wpedantic)
ament_target_dependencies(
  planning_smoke_test
  geometry_msgs
  moveit_core
  moveit_msgs
  moveit_ros_planning_interface
  rclcpp
  sensor_msgs
  shape_msgs
)

# Card 053 Phase 2: headless planning-only benchmark (no Gazebo, no move_group, no execution).
add_executable(planning_only_benchmark src/planning_only_benchmark.cpp)
target_link_libraries(
  planning_only_benchmark
  scene_geometry
  manipulation_geometry
  Eigen3::Eigen
  yaml-cpp::yaml-cpp
)
target_compile_options(planning_only_benchmark PRIVATE -Wall -Wextra -Wpedantic)
ament_target_dependencies(
  planning_only_benchmark
  geometry_msgs
  moveit_core
  moveit_msgs
  moveit_ros_planning
  rclcpp
  restocker_interfaces
  shape_msgs
)

install(DIRECTORY include/ DESTINATION include)
install(DIRECTORY config DESTINATION share/${PROJECT_NAME})
install(
  TARGETS
    action_client_finality
    action_result_publisher
    attachment_transaction_machine
    coordinator_async_steady_clock_failure_latch
    coordinator_process_control
    coordinator_active_fault_epoch
    coordinator_generation_quiescence
    coordinator_handoff_policy
    coordinator_handoff_reducer
    coordinator_pump_lease_gate
    coordinator_steady_clock
    coordinator_startup
    coordinator_startup_transport
    coordinator_termination_router
    controller_trajectory_slicing
    trajectory_scene_validation
    free_space_race
    grasp_escape_planner
    race_planning_scene
    goal_termination_authority
    grasp_candidates
    gripper_geometry
    linear_path_refinement
    manipulation_geometry
    attachment_port
    ros_attachment_port
    gripper_port
    ros_gripper_port
    motion_operation_evidence
    motion_port
    moveit_motion_port
    moveit_path_constraint_conversion
    transfer_path_constraints
    obstacle_projection
    pending_activation_validator
    placement_candidates
    recovery_advice_gate
    recovery_classification
    motion_execute_verdict
    planning_contract
    pregrasp_planning_authority
    planning_scene_attachment
    planning_scene_lease
    planning_scene_lease_ros
    planning_scene_reconciliation
    restock_action_contract
    restock_action_coordinator_node
    restock_coordinator_driver
    restock_coordinator_inbox
    restock_coordinator_primitives
    restock_goal_context
    restock_task_machine
    scene_geometry
    task_selection
    tray_survey_logic
    tray_survey_server
    world_state_async_port
    world_state_port_contract
    world_snapshot_projection
    viewpoint_survey
  EXPORT export_restocker_task_executor
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
)
install(
  TARGETS
    planning_scene_projector_node
    planning_smoke_test
    planning_only_benchmark
    restock_action_coordinator
          survey_viewpoint_node lane_survey_node tray_survey_node autonomous_restock_campaign
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  find_package(ament_cmake_gtest REQUIRED)
  find_package(ament_lint_auto REQUIRED)
  find_package(launch_testing_ament_cmake REQUIRED)
  find_package(rosgraph_msgs REQUIRED)
  # Launch tests lease their ROS domain and Gazebo partition at run time; see
  # cmake/restocker_test_isolation.cmake.
  include(${CMAKE_CURRENT_SOURCE_DIR}/../../../cmake/restocker_test_isolation.cmake)

  ament_add_gtest(test_campaign_confirm_report test/test_campaign_confirm_report.cpp TIMEOUT 30)
  target_include_directories(test_campaign_confirm_report PRIVATE include)
  target_link_libraries(test_campaign_confirm_report Threads::Threads)
  target_compile_options(test_campaign_confirm_report PRIVATE -Wall -Wextra -Wpedantic)

  ament_add_gtest(test_attachment_port test/test_attachment_port.cpp)
  target_link_libraries(test_attachment_port restocker_task_executor::attachment_port)
  target_compile_options(test_attachment_port PRIVATE -Wall -Wextra -Wpedantic)

  ament_add_gtest(test_ros_attachment_port test/test_ros_attachment_port.cpp TIMEOUT 30)
  target_link_libraries(test_ros_attachment_port restocker_task_executor::ros_attachment_port)
  target_compile_options(test_ros_attachment_port PRIVATE -Wall -Wextra -Wpedantic)

  ament_add_gtest(test_gripper_port test/test_gripper_port.cpp)
  target_link_libraries(test_gripper_port restocker_task_executor::gripper_port)
  target_compile_options(test_gripper_port PRIVATE -Wall -Wextra -Wpedantic)

  ament_add_gtest(test_motion_port test/test_motion_port.cpp)
  target_link_libraries(test_motion_port restocker_task_executor::motion_port)
  target_compile_options(test_motion_port PRIVATE -Wall -Wextra -Wpedantic)

  ament_add_gtest(test_transfer_path_constraints test/test_transfer_path_constraints.cpp)
  target_link_libraries(
    test_transfer_path_constraints restocker_task_executor::transfer_path_constraints)
  target_compile_options(test_transfer_path_constraints PRIVATE -Wall -Wextra -Wpedantic)

  ament_add_gtest(
    test_moveit_path_constraint_conversion test/test_moveit_path_constraint_conversion.cpp)
  target_link_libraries(
    test_moveit_path_constraint_conversion
    restocker_task_executor::moveit_path_constraint_conversion
    restocker_task_executor::transfer_path_constraints)
  target_compile_options(
    test_moveit_path_constraint_conversion PRIVATE -Wall -Wextra -Wpedantic)

  ament_add_gtest(test_survey_submit_refusal test/test_survey_submit_refusal.cpp)
  target_link_libraries(test_survey_submit_refusal restocker_task_executor::survey_submit_refusal)
  target_compile_options(test_survey_submit_refusal PRIVATE -Wall -Wextra -Wpedantic)

  ament_add_gtest(test_viewpoint_survey test/test_viewpoint_survey.cpp)
  target_link_libraries(test_viewpoint_survey restocker_task_executor::viewpoint_survey)
  target_include_directories(test_viewpoint_survey PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/test)
  target_compile_options(test_viewpoint_survey PRIVATE -Wall -Wextra -Wpedantic)

  ament_add_gtest(test_tray_survey_logic test/test_tray_survey_logic.cpp)
  target_link_libraries(test_tray_survey_logic restocker_task_executor::tray_survey_logic)
  target_compile_options(test_tray_survey_logic PRIVATE -Wall -Wextra -Wpedantic)

  # The orchestration itself, against a fake /survey_viewpoint server and synthetic duty
  # streams: no simulator, no planner, no renderer. The live acceptance of the same contracts
  # is restocker_bringup's test_tray_survey_runtime.py.
  ament_add_gtest(test_tray_survey_node test/test_tray_survey_node.cpp TIMEOUT 120)
  target_link_libraries(test_tray_survey_node restocker_task_executor::tray_survey_server)
  ament_target_dependencies(
    test_tray_survey_node
    rclcpp
    rclcpp_action
    restocker_interfaces
    sensor_msgs
    tf2_ros
  )
  target_compile_options(test_tray_survey_node PRIVATE -Wall -Wextra -Wpedantic)
  set_tests_properties(
    test_tray_survey_node PROPERTIES ENVIRONMENT
    "RESTOCKER_TEST_WORKCELL_GEOMETRY=${CMAKE_CURRENT_SOURCE_DIR}/../restocker_description/config/workcell_geometry.yaml;RESTOCKER_TEST_SENSORS_XACRO=${CMAKE_CURRENT_SOURCE_DIR}/../restocker_description/urdf/sensors.xacro")

  ament_add_gtest(test_recovery_advice_gate test/test_recovery_advice_gate.cpp)
  target_link_libraries(test_recovery_advice_gate restocker_task_executor::recovery_advice_gate)
  target_compile_options(test_recovery_advice_gate PRIVATE -Wall -Wextra -Wpedantic)

  ament_add_gtest(
    test_recovery_classification test/test_recovery_classification.cpp)
  target_link_libraries(
    test_recovery_classification restocker_task_executor::recovery_classification)
  target_compile_options(test_recovery_classification PRIVATE -Wall -Wextra -Wpedantic)

  ament_add_gtest(
    test_survey_recovery_context test/test_survey_recovery_context.cpp)
  target_link_libraries(
    test_survey_recovery_context restocker_task_executor::survey_recovery_context)
  target_compile_options(test_survey_recovery_context PRIVATE -Wall -Wextra -Wpedantic)

  ament_add_gtest(
    test_attachment_transaction_machine test/test_attachment_transaction_machine.cpp)
  target_link_libraries(
    test_attachment_transaction_machine
    restocker_task_executor::attachment_transaction_machine)
  target_compile_options(
    test_attachment_transaction_machine PRIVATE -Wall -Wextra -Wpedantic)

  ament_add_gtest(
    test_coordinator_process_control test/test_coordinator_process_control.cpp)
  target_link_libraries(
    test_coordinator_process_control coordinator_process_control coordinator_signal_waiter)
  target_compile_options(
    test_coordinator_process_control PRIVATE -Wall -Wextra -Wpedantic)

  ament_add_gtest(
    test_coordinator_active_fault_epoch test/test_coordinator_active_fault_epoch.cpp)
  target_link_libraries(
    test_coordinator_active_fault_epoch
    restocker_task_executor::coordinator_active_fault_epoch)
  target_compile_options(
    test_coordinator_active_fault_epoch PRIVATE -Wall -Wextra -Wpedantic)

  ament_add_gtest(
    test_accepted_handle_adoption test/test_accepted_handle_adoption.cpp)
  target_include_directories(
    test_accepted_handle_adoption
    PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/include ${CMAKE_CURRENT_SOURCE_DIR}/src)
  target_compile_options(
    test_accepted_handle_adoption PRIVATE -Wall -Wextra -Wpedantic)

  ament_add_gtest(
    test_pending_route_executor test/test_pending_route_executor.cpp)
  target_include_directories(
    test_pending_route_executor
    PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/include ${CMAKE_CURRENT_SOURCE_DIR}/src)
  target_link_libraries(
    test_pending_route_executor
    restocker_task_executor::coordinator_handoff_reducer
    restocker_task_executor::coordinator_termination_router
    restocker_task_executor::restock_coordinator_inbox
    Threads::Threads)
  target_compile_options(
    test_pending_route_executor PRIVATE -Wall -Wextra -Wpedantic)

  ament_add_gtest(
    test_pending_activation_validator test/test_pending_activation_validator.cpp)
  target_include_directories(
    test_pending_activation_validator
    PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/include ${CMAKE_CURRENT_SOURCE_DIR}/src)
  target_link_libraries(
    test_pending_activation_validator pending_activation_validator)
  target_compile_options(
    test_pending_activation_validator PRIVATE -Wall -Wextra -Wpedantic)

  ament_add_gtest(
    test_coordinator_generation_quiescence
    test/test_coordinator_generation_quiescence.cpp)
  target_link_libraries(
    test_coordinator_generation_quiescence
    restocker_task_executor::coordinator_generation_quiescence)
  target_compile_options(
    test_coordinator_generation_quiescence PRIVATE -Wall -Wextra -Wpedantic)

  ament_add_gtest(
    test_generation_scoped_inbox_deposit
    test/test_generation_scoped_inbox_deposit.cpp)
  target_link_libraries(
    test_generation_scoped_inbox_deposit
    restocker_task_executor::coordinator_generation_quiescence)
  target_compile_options(
    test_generation_scoped_inbox_deposit PRIVATE -Wall -Wextra -Wpedantic)

  ament_add_gtest(
    test_coordinator_pump_lease_gate test/test_coordinator_pump_lease_gate.cpp)
  target_link_libraries(
    test_coordinator_pump_lease_gate
    restocker_task_executor::coordinator_pump_lease_gate)
  target_compile_options(
    test_coordinator_pump_lease_gate PRIVATE -Wall -Wextra -Wpedantic)

  ament_add_gtest(
    test_coordinator_handoff_policy test/test_coordinator_handoff_policy.cpp)
  target_link_libraries(
    test_coordinator_handoff_policy
    restocker_task_executor::coordinator_handoff_policy)
  target_compile_options(
    test_coordinator_handoff_policy PRIVATE -Wall -Wextra -Wpedantic)

  ament_add_gtest(
    test_coordinator_handoff_reducer test/test_coordinator_handoff_reducer.cpp)
  target_link_libraries(
    test_coordinator_handoff_reducer
    restocker_task_executor::coordinator_handoff_reducer)
  target_compile_options(
    test_coordinator_handoff_reducer PRIVATE -Wall -Wextra -Wpedantic)

  ament_add_gtest(test_coordinator_steady_clock test/test_coordinator_steady_clock.cpp)
  target_link_libraries(
    test_coordinator_steady_clock restocker_task_executor::coordinator_steady_clock)
  target_compile_options(test_coordinator_steady_clock PRIVATE -Wall -Wextra -Wpedantic)

  ament_add_gtest(
    test_coordinator_async_steady_clock_failure_latch
    test/test_coordinator_async_steady_clock_failure_latch.cpp)
  target_link_libraries(
    test_coordinator_async_steady_clock_failure_latch
    restocker_task_executor::coordinator_async_steady_clock_failure_latch
    Threads::Threads)
  target_compile_options(
    test_coordinator_async_steady_clock_failure_latch PRIVATE -Wall -Wextra -Wpedantic)

  ament_add_gtest(
    test_motion_operation_evidence test/test_motion_operation_evidence.cpp)
  target_link_libraries(
    test_motion_operation_evidence
    restocker_task_executor::motion_operation_evidence)
  target_compile_options(
    test_motion_operation_evidence PRIVATE -Wall -Wextra -Wpedantic)

  ament_add_gtest(
    test_goal_termination_authority test/test_goal_termination_authority.cpp)
  target_link_libraries(
    test_goal_termination_authority
    restocker_task_executor::goal_termination_authority)
  target_compile_options(
    test_goal_termination_authority PRIVATE -Wall -Wextra -Wpedantic)

  ament_add_gtest(
    test_coordinator_termination_router test/test_coordinator_termination_router.cpp)
  target_link_libraries(
    test_coordinator_termination_router
    restocker_task_executor::coordinator_termination_router)
  target_compile_options(
    test_coordinator_termination_router PRIVATE -Wall -Wextra -Wpedantic)

  ament_add_gtest(test_coordinator_startup test/test_coordinator_startup.cpp)
  target_link_libraries(
    test_coordinator_startup
    restocker_task_executor::coordinator_startup
    restocker_world_state::ros_conversions)
  target_compile_options(test_coordinator_startup PRIVATE -Wall -Wextra -Wpedantic)

  ament_add_gtest(
    test_coordinator_startup_transport test/test_coordinator_startup_transport.cpp)
  target_link_libraries(
    test_coordinator_startup_transport
    restocker_task_executor::coordinator_startup_transport)
  target_compile_options(
    test_coordinator_startup_transport PRIVATE -Wall -Wextra -Wpedantic)

  ament_add_pytest_test(
    test_coordinator_process test/test_coordinator_process.py
    ENV
      RESTOCKER_COORDINATOR_EXECUTABLE=$<TARGET_FILE:restock_action_coordinator>
      RESTOCKER_PRODUCT_CATALOG=${CMAKE_CURRENT_SOURCE_DIR}/../restocker_description/config/product_collision_catalog.yaml
      RESTOCKER_WORKCELL_GEOMETRY=${CMAKE_CURRENT_SOURCE_DIR}/../restocker_description/config/workcell_geometry.yaml
      RESTOCKER_GRIPPER_GEOMETRY=${CMAKE_CURRENT_SOURCE_DIR}/../restocker_description/config/gripper_geometry.yaml
    TIMEOUT 30)

  # Card 075: the fixture's world-state service must answer while a goal_callback sleeps
  # (S21's admission hang). Red when the service shares the MutuallyExclusive default group.
  ament_add_pytest_test(
    test_fixture_world_state_callback_group test/test_fixture_world_state_callback_group.py
    TIMEOUT 30)

  # Static check of the shipped coordinator budget YAML, run as a plain script: the launch-testing
  # pytest plugin imports every sibling test module in this directory during collection, and this
  # file does not need that machinery (same shape as test_demo_wiring below).
  add_test(
    NAME test_coordinator_budgets
    COMMAND ${Python3_EXECUTABLE} ${CMAKE_CURRENT_SOURCE_DIR}/test/test_coordinator_budgets.py)
  set_tests_properties(test_coordinator_budgets PROPERTIES TIMEOUT 30)

  # The domain lease must keep carrying its isolation tag and the UDP-only Fast DDS transport.
  add_test(
    NAME test_domain_isolation
    COMMAND ${Python3_EXECUTABLE} ${CMAKE_CURRENT_SOURCE_DIR}/../../../scripts/test_domain_isolation.py)
  set_tests_properties(test_domain_isolation PROPERTIES TIMEOUT 30)

  # Every launch test, gtest and pytest must reach the lease runner, and the runner must reap its
  # test before releasing the lease, even when terminated.
  add_test(
    NAME test_test_isolation_wiring
    COMMAND ${Python3_EXECUTABLE} ${CMAKE_CURRENT_SOURCE_DIR}/../../../scripts/test_test_isolation_wiring.py)
  set_tests_properties(test_test_isolation_wiring PROPERTIES TIMEOUT 90)

  # UDP-only Fast DDS is scoped to the /clock-hop participants only (clock bridge + world
  # state); camera bridges keep shared memory.
  add_test(
    NAME test_launch_transport_scopes
    COMMAND ${Python3_EXECUTABLE} ${CMAKE_CURRENT_SOURCE_DIR}/../../../scripts/test_launch_transport_scopes.py)
  set_tests_properties(test_launch_transport_scopes PROPERTIES TIMEOUT 30)

  add_launch_test(
    test/test_coordinator_live_shutdown_clean.py
    TARGET test_coordinator_live_shutdown_clean
    TIMEOUT 30)
  set_property(
    TEST test_coordinator_live_shutdown_clean
    APPEND PROPERTY ENVIRONMENT_MODIFICATION
      "PYTHONPATH=path_list_prepend:${CMAKE_CURRENT_SOURCE_DIR}/test")
  add_launch_test(
    test/test_coordinator_live_shutdown_timeout.py
    TARGET test_coordinator_live_shutdown_timeout
    TIMEOUT 30)
  set_property(
    TEST test_coordinator_live_shutdown_timeout
    APPEND PROPERTY ENVIRONMENT_MODIFICATION
      "PYTHONPATH=path_list_prepend:${CMAKE_CURRENT_SOURCE_DIR}/test")

  # The mode loop end to end: validity-driven shelf surveys, deficit-driven tray surveys with
  # confirmation and refutation, admitted-candidate transfers, cancellation, idle, and the
  # terminal cycle bound (non-renderer; runs under nix flake check).
  add_launch_test(
    test/test_autonomous_restock_campaign.py
    TARGET test_autonomous_restock_campaign
    TIMEOUT 60)
  set_property(
    TEST test_autonomous_restock_campaign
    APPEND PROPERTY ENVIRONMENT_MODIFICATION
      "PYTHONPATH=path_list_prepend:${CMAKE_CURRENT_SOURCE_DIR}/test")
  # Card 086 stage 1 (CMB-SPEC-13): unresolved-motion inhibition, the cycle bound and the
  # restart gate, each in its own campaign process (non-renderer; runs under nix flake check).
  add_launch_test(
    test/test_campaign_unresolved_motion.py
    TARGET test_campaign_unresolved_motion
    TIMEOUT 90)
  set_property(
    TEST test_campaign_unresolved_motion
    APPEND PROPERTY ENVIRONMENT_MODIFICATION
      "PYTHONPATH=path_list_prepend:${CMAKE_CURRENT_SOURCE_DIR}/test")
  add_launch_test(
    test/test_campaign_unresolved_delivered.py
    TARGET test_campaign_unresolved_delivered
    TIMEOUT 90)
  set_property(
    TEST test_campaign_unresolved_delivered
    APPEND PROPERTY ENVIRONMENT_MODIFICATION
      "PYTHONPATH=path_list_prepend:${CMAKE_CURRENT_SOURCE_DIR}/test")
  add_launch_test(
    test/test_campaign_unresolved_tray_child.py
    TARGET test_campaign_unresolved_tray_child
    TIMEOUT 90)
  set_property(
    TEST test_campaign_unresolved_tray_child
    APPEND PROPERTY ENVIRONMENT_MODIFICATION
      "PYTHONPATH=path_list_prepend:${CMAKE_CURRENT_SOURCE_DIR}/test")
  add_launch_test(
    test/test_campaign_unresolved_tray_grace.py
    TARGET test_campaign_unresolved_tray_grace
    TIMEOUT 90)
  set_property(
    TEST test_campaign_unresolved_tray_grace
    APPEND PROPERTY ENVIRONMENT_MODIFICATION
      "PYTHONPATH=path_list_prepend:${CMAKE_CURRENT_SOURCE_DIR}/test")
  add_launch_test(
    test/test_campaign_unresolved_unknown.py
    TARGET test_campaign_unresolved_unknown
    TIMEOUT 90)
  set_property(
    TEST test_campaign_unresolved_unknown
    APPEND PROPERTY ENVIRONMENT_MODIFICATION
      "PYTHONPATH=path_list_prepend:${CMAKE_CURRENT_SOURCE_DIR}/test")
  add_launch_test(
    test/test_campaign_restock_rejection.py
    TARGET test_campaign_restock_rejection
    TIMEOUT 90)
  set_property(
    TEST test_campaign_restock_rejection
    APPEND PROPERTY ENVIRONMENT_MODIFICATION
      "PYTHONPATH=path_list_prepend:${CMAKE_CURRENT_SOURCE_DIR}/test")
  add_launch_test(
    test/test_campaign_restock_proven_failures.py
    TARGET test_campaign_restock_proven_failures
    TIMEOUT 90)
  set_property(
    TEST test_campaign_restock_proven_failures
    APPEND PROPERTY ENVIRONMENT_MODIFICATION
      "PYTHONPATH=path_list_prepend:${CMAKE_CURRENT_SOURCE_DIR}/test")
  add_launch_test(
    test/test_campaign_restock_post_motion_planning_failed.py
    TARGET test_campaign_restock_post_motion_planning_failed
    TIMEOUT 90)
  set_property(
    TEST test_campaign_restock_post_motion_planning_failed
    APPEND PROPERTY ENVIRONMENT_MODIFICATION
      "PYTHONPATH=path_list_prepend:${CMAKE_CURRENT_SOURCE_DIR}/test")
  add_launch_test(
    test/test_campaign_unresolved_bound.py
    TARGET test_campaign_unresolved_bound
    TIMEOUT 90)
  set_property(
    TEST test_campaign_unresolved_bound
    APPEND PROPERTY ENVIRONMENT_MODIFICATION
      "PYTHONPATH=path_list_prepend:${CMAKE_CURRENT_SOURCE_DIR}/test")
  add_launch_test(
    test/test_campaign_restart_recovery.py
    TARGET test_campaign_restart_recovery
    TIMEOUT 90)
  set_property(
    TEST test_campaign_restart_recovery
    APPEND PROPERTY ENVIRONMENT_MODIFICATION
      "PYTHONPATH=path_list_prepend:${CMAKE_CURRENT_SOURCE_DIR}/test")
  # Cadence-zero campaign against a shelf survey that refuses everything: the failed-cycle
  # backoff floor must still separate the retries (non-renderer; runs under nix flake check).
  add_launch_test(
    test/test_autonomous_campaign_failed_survey_backoff.py
    TARGET test_autonomous_campaign_failed_survey_backoff
    TIMEOUT 45)
  set_property(
    TEST test_autonomous_campaign_failed_survey_backoff
    APPEND PROPERTY ENVIRONMENT_MODIFICATION
      "PYTHONPATH=path_list_prepend:${CMAKE_CURRENT_SOURCE_DIR}/test")
  # Card 055: the premature-NO_COMPATIBLE_PAIR contract — forced tray re-survey, budget
  # termination, genuine exhaustion untouched, unseen stock honest (non-renderer).
  add_launch_test(
    test/test_campaign_ncp_recovery.py
    TARGET test_campaign_ncp_recovery
    TIMEOUT 90)
  set_property(
    TEST test_campaign_ncp_recovery
    APPEND PROPERTY ENVIRONMENT_MODIFICATION
      "PYTHONPATH=path_list_prepend:${CMAKE_CURRENT_SOURCE_DIR}/test")
  # Milestone 10 §6, Card 066: a feed-blocked tray NO_CANDIDATE is a bounded, reset-on-progress
  # rung that never reads as stock exhaustion; Card 055's receipt names the coordinator's
  # refusal (non-renderer).
  add_launch_test(
    test/test_campaign_feed_order_recovery.py
    TARGET test_campaign_feed_order_recovery
    TIMEOUT 180)
  set_property(
    TEST test_campaign_feed_order_recovery
    APPEND PROPERTY ENVIRONMENT_MODIFICATION
      "PYTHONPATH=path_list_prepend:${CMAKE_CURRENT_SOURCE_DIR}/test")
  # Card 058: the skip-mark lift contract — forced skip-lifted tray re-survey, budget
  # termination, empty overview and confirm-refutation untouched (non-renderer).
  # Card 066 × Card 058 integration: the lift counts only feed-column fronts, runs before the
  # feed-order rung, and hands feed-blocked stock to it once spent (own process: lift budget 1).
  add_launch_test(
    test/test_campaign_feed_order_skip_lift.py
    TARGET test_campaign_feed_order_skip_lift
    TIMEOUT 120)
  set_property(
    TEST test_campaign_feed_order_skip_lift
    APPEND PROPERTY ENVIRONMENT_MODIFICATION
      "PYTHONPATH=path_list_prepend:${CMAKE_CURRENT_SOURCE_DIR}/test")
  # Card 066 (review cmbrev066b N2): the lift matches marks with the merge radius the server
  # reported (own process: the reported radius persists).
  add_launch_test(
    test/test_campaign_merge_radius.py
    TARGET test_campaign_merge_radius
    TIMEOUT 120)
  set_property(
    TEST test_campaign_merge_radius
    APPEND PROPERTY ENVIRONMENT_MODIFICATION
      "PYTHONPATH=path_list_prepend:${CMAKE_CURRENT_SOURCE_DIR}/test")
  add_launch_test(
    test/test_campaign_skip_lift_recovery.py
    TARGET test_campaign_skip_lift_recovery
    TIMEOUT 120)
  set_property(
    TEST test_campaign_skip_lift_recovery
    APPEND PROPERTY ENVIRONMENT_MODIFICATION
      "PYTHONPATH=path_list_prepend:${CMAKE_CURRENT_SOURCE_DIR}/test")
  # Card 058 review notes 1/2: the lift tied to the qualifying class across a two-class
  # fallthrough, and a gone-for-the-run mark never lifted (own process: monotone budget).
  add_launch_test(
    test/test_campaign_skip_lift_multiclass.py
    TARGET test_campaign_skip_lift_multiclass
    TIMEOUT 120)
  set_property(
    TEST test_campaign_skip_lift_multiclass
    APPEND PROPERTY ENVIRONMENT_MODIFICATION
      "PYTHONPATH=path_list_prepend:${CMAKE_CURRENT_SOURCE_DIR}/test")
  # Card 069 (Card 058 re-review note B): an armed lift lapses when its class has no front
  # deficit, or with a cycle that never reaches the survey.
  add_launch_test(
    test/test_campaign_skip_lift_lapse.py
    TARGET test_campaign_skip_lift_lapse
    TIMEOUT 120)
  set_property(
    TEST test_campaign_skip_lift_lapse
    APPEND PROPERTY ENVIRONMENT_MODIFICATION
      "PYTHONPATH=path_list_prepend:${CMAKE_CURRENT_SOURCE_DIR}/test")
  # Card 069: a completed tray survey that viewed a counted product's place empty retires it
  # after absence_confirmations consecutive surveys; reversible on a newer observation.
  add_launch_test(
    test/test_campaign_absence_retirement.py
    TARGET test_campaign_absence_retirement
    TIMEOUT 180)
  set_property(
    TEST test_campaign_absence_retirement
    APPEND PROPERTY ENVIRONMENT_MODIFICATION
      "PYTHONPATH=path_list_prepend:${CMAKE_CURRENT_SOURCE_DIR}/test")
  add_test(
    NAME test_demo_wiring
    COMMAND ${Python3_EXECUTABLE} ${CMAKE_CURRENT_SOURCE_DIR}/test/test_demo_wiring.py)
  set_tests_properties(
    test_demo_wiring
    PROPERTIES
      ENVIRONMENT "RESTOCKER_REPOSITORY_ROOT=${CMAKE_CURRENT_SOURCE_DIR}/../../.."
      TIMEOUT 30)

  add_test(
    NAME test_moveit_port_submission_pin
    COMMAND ${Python3_EXECUTABLE} ${CMAKE_CURRENT_SOURCE_DIR}/test/test_moveit_port_submission_pin.py)
  set_tests_properties(
    test_moveit_port_submission_pin
    PROPERTIES
      ENVIRONMENT "RESTOCKER_PACKAGE_ROOT=${CMAKE_CURRENT_SOURCE_DIR}"
      TIMEOUT 30)

  ament_add_gtest(test_restock_task_machine test/test_restock_task_machine.cpp)
  target_link_libraries(
    test_restock_task_machine restocker_task_executor::restock_task_machine)
  target_compile_options(test_restock_task_machine PRIVATE -Wall -Wextra -Wpedantic)

  ament_add_gtest(
    test_restock_coordinator_primitives test/test_restock_coordinator_primitives.cpp)
  target_link_libraries(
    test_restock_coordinator_primitives
    restocker_task_executor::restock_coordinator_primitives)
  target_compile_options(
    test_restock_coordinator_primitives PRIVATE -Wall -Wextra -Wpedantic)

  ament_add_gtest(
    test_restock_coordinator_inbox test/test_restock_coordinator_inbox.cpp)
  target_link_libraries(
    test_restock_coordinator_inbox
    restocker_task_executor::coordinator_generation_quiescence
    restocker_task_executor::restock_coordinator_inbox)
  target_compile_options(
    test_restock_coordinator_inbox PRIVATE -Wall -Wextra -Wpedantic)

  add_executable(
    test_coordinator_inbox_dependencies
    test/test_coordinator_inbox_dependencies.cpp)
  target_link_libraries(
    test_coordinator_inbox_dependencies
    coordinator_active_fault_epoch
    coordinator_generation_quiescence
    restock_coordinator_inbox
    restock_goal_context)
  target_compile_options(
    test_coordinator_inbox_dependencies PRIVATE -Wall -Wextra -Wpedantic)
  add_test(
    NAME test_coordinator_inbox_dependencies
    COMMAND test_coordinator_inbox_dependencies)

  ament_add_gtest(
    test_restock_coordinator_driver test/test_restock_coordinator_driver.cpp)
  target_link_libraries(
    test_restock_coordinator_driver
    restocker_task_executor::restock_coordinator_driver
    restocker_task_executor::restock_action_contract
    restocker_task_executor::campaign_motion_evidence
    restocker_task_executor::ros_attachment_port)
  target_compile_options(
    test_restock_coordinator_driver PRIVATE -Wall -Wextra -Wpedantic)

  ament_add_gtest(test_restock_goal_context test/test_restock_goal_context.cpp)
  target_link_libraries(
    test_restock_goal_context
    restocker_task_executor::coordinator_generation_quiescence
    restocker_task_executor::restock_goal_context)
  target_compile_options(test_restock_goal_context PRIVATE -Wall -Wextra -Wpedantic)

  ament_add_gtest(test_campaign_motion_evidence test/test_campaign_motion_evidence.cpp)
  target_link_libraries(
    test_campaign_motion_evidence restocker_task_executor::campaign_motion_evidence)
  target_compile_options(test_campaign_motion_evidence PRIVATE -Wall -Wextra -Wpedantic)

  ament_add_gtest(test_unresolved_motion test/test_unresolved_motion.cpp)
  target_link_libraries(test_unresolved_motion restocker_task_executor::unresolved_motion)
  target_compile_options(test_unresolved_motion PRIVATE -Wall -Wextra -Wpedantic)

  ament_add_gtest(test_action_client_finality test/test_action_client_finality.cpp)
  target_link_libraries(
    test_action_client_finality restocker_task_executor::action_client_finality)
  ament_target_dependencies(test_action_client_finality restocker_interfaces)
  target_compile_options(test_action_client_finality PRIVATE -Wall -Wextra -Wpedantic)

  ament_add_gtest(test_restock_action_contract test/test_restock_action_contract.cpp)
  target_link_libraries(
    test_restock_action_contract restocker_task_executor::restock_action_contract)
  target_compile_options(test_restock_action_contract PRIVATE -Wall -Wextra -Wpedantic)

  ament_add_gtest(
    test_restock_action_coordinator_node test/test_restock_action_coordinator_node.cpp
    TIMEOUT 90)
  target_link_libraries(
    test_restock_action_coordinator_node
    restocker_task_executor::restock_action_coordinator_node
    restocker_world_state::ros_conversions)
  ament_target_dependencies(
    test_restock_action_coordinator_node
    rclcpp
    rclcpp_action
    restocker_interfaces
    rosgraph_msgs
    tf2_ros)
  target_compile_definitions(
    test_restock_action_coordinator_node
    PRIVATE
      RESTOCKER_TEST_PRODUCT_CATALOG="${CMAKE_CURRENT_SOURCE_DIR}/../restocker_description/config/product_collision_catalog.yaml"
      RESTOCKER_TEST_WORKCELL_GEOMETRY="${CMAKE_CURRENT_SOURCE_DIR}/../restocker_description/config/workcell_geometry.yaml"
      RESTOCKER_TEST_GRIPPER_GEOMETRY="${CMAKE_CURRENT_SOURCE_DIR}/../restocker_description/config/gripper_geometry.yaml")
  target_compile_options(
    test_restock_action_coordinator_node PRIVATE -Wall -Wextra -Wpedantic)

  ament_add_gtest(test_action_result_publisher test/test_action_result_publisher.cpp)
  target_link_libraries(
    test_action_result_publisher restocker_task_executor::action_result_publisher)
  ament_target_dependencies(
    test_action_result_publisher rclcpp rclcpp_action restocker_interfaces)
  target_compile_options(test_action_result_publisher PRIVATE -Wall -Wextra -Wpedantic)

  ament_add_gtest(
    test_world_state_port_contract test/test_world_state_port_contract.cpp)
  target_link_libraries(
    test_world_state_port_contract restocker_task_executor::world_state_port_contract)
  target_compile_options(
    test_world_state_port_contract PRIVATE -Wall -Wextra -Wpedantic)

  ament_add_gtest(
    test_world_state_port_integration test/test_world_state_port_integration.cpp)
  target_link_libraries(
    test_world_state_port_integration
    restocker_task_executor::world_state_port_contract
    restocker_world_state::world_state)
  target_compile_options(
    test_world_state_port_integration PRIVATE -Wall -Wextra -Wpedantic)

  ament_add_gtest(
    test_world_state_async_port test/test_world_state_async_port.cpp)
  target_link_libraries(
    test_world_state_async_port restocker_task_executor::world_state_async_port)
  target_compile_options(test_world_state_async_port PRIVATE -Wall -Wextra -Wpedantic)

  ament_add_gtest(test_motion_execute_verdict test/test_motion_execute_verdict.cpp)
  target_link_libraries(test_motion_execute_verdict motion_execute_verdict)
  target_compile_options(test_motion_execute_verdict PRIVATE -Wall -Wextra -Wpedantic)

  ament_add_gtest(test_planning_contract test/test_planning_contract.cpp)
  target_link_libraries(test_planning_contract planning_contract)
  target_compile_options(test_planning_contract PRIVATE -Wall -Wextra -Wpedantic)

  # Card 062: the grasp escape line on the real restocker model (xacro + SRDF + KDL) and MoveIt's
  # own collision checker — slot 16's start state, and every refusal path.
  find_package(ament_index_cpp REQUIRED)
  ament_add_gtest(test_grasp_escape_planner test/test_grasp_escape_planner.cpp TIMEOUT 120)
  target_link_libraries(test_grasp_escape_planner grasp_escape_planner)
  target_compile_options(test_grasp_escape_planner PRIVATE -Wall -Wextra -Wpedantic)
  ament_target_dependencies(
    test_grasp_escape_planner ament_index_cpp moveit_core moveit_msgs moveit_ros_planning rclcpp)

  # Card 046: the refinement ladder on the real restocker model (xacro + SRDF + KDL) with MoveIt's
  # own interpolator and collision checker — Card 073's recorded fixture red at the shipped 0.005 m
  # and green with the ladder, plus the two H1 controls refused at every step.
  ament_add_gtest(test_linear_path_refinement test/test_linear_path_refinement.cpp TIMEOUT 180)
  target_link_libraries(test_linear_path_refinement linear_path_refinement)
  target_compile_options(test_linear_path_refinement PRIVATE -Wall -Wextra -Wpedantic)
  ament_target_dependencies(
    test_linear_path_refinement ament_index_cpp moveit_core moveit_msgs moveit_ros_planning rclcpp)

  # Card 068: the race scene on the real restocker model against a PlanningSceneMonitor
  # configured like move_group, with the race's production ompl/ompl_fallback pipelines.
  ament_add_gtest(test_race_planning_scene test/test_race_planning_scene.cpp TIMEOUT 180)
  target_link_libraries(test_race_planning_scene race_planning_scene free_space_race)
  target_compile_definitions(
    test_race_planning_scene
    PRIVATE
      RESTOCKER_TEST_RACE_PARAMETERS_SCRIPT="${CMAKE_CURRENT_SOURCE_DIR}/test/race_scene_parameters.py"
      RESTOCKER_TEST_MOVE_GROUP_LAUNCH="${CMAKE_CURRENT_SOURCE_DIR}/../restocker_moveit_config/launch/move_group.launch.py"
  )
  target_compile_options(test_race_planning_scene PRIVATE -Wall -Wextra -Wpedantic)
  ament_target_dependencies(
    test_race_planning_scene moveit_core moveit_msgs moveit_ros_planning rclcpp)

  # Card 061: a planned trajectory that fits one controller goal keeps the planner's timing;
  # slices that TOTG cannot time at the strict tolerance take the single fallback.
  ament_add_gtest(
    test_controller_trajectory_slicing test/test_controller_trajectory_slicing.cpp)
  target_link_libraries(
    test_controller_trajectory_slicing controller_trajectory_slicing)
  ament_target_dependencies(test_controller_trajectory_slicing moveit_core)
  target_compile_options(test_controller_trajectory_slicing PRIVATE -Wall -Wextra -Wpedantic)

  # Card 053: the race's decision layer — primary-fail/fallback-win, both-fail keeping Card
  # 043's classification, and a deterministic-seed reproducibility check.
  ament_add_gtest(test_free_space_race test/test_free_space_race.cpp)
  target_link_libraries(test_free_space_race free_space_race planning_contract)
  target_compile_options(test_free_space_race PRIVATE -Wall -Wextra -Wpedantic)

  ament_add_gtest(
    test_pregrasp_planning_authority test/test_pregrasp_planning_authority.cpp)
  target_link_libraries(
    test_pregrasp_planning_authority
    restocker_task_executor::pregrasp_planning_authority)
  target_compile_options(
    test_pregrasp_planning_authority PRIVATE -Wall -Wextra -Wpedantic)

  ament_add_gtest(test_trajectory_scene_validation test/test_trajectory_scene_validation.cpp)
  target_link_libraries(test_trajectory_scene_validation trajectory_scene_validation)
  target_compile_options(test_trajectory_scene_validation PRIVATE -Wall -Wextra -Wpedantic)

  ament_add_gtest(test_trajectory_scene_validity test/test_trajectory_scene_validity.cpp)
  target_link_libraries(
    test_trajectory_scene_validity
    restocker_task_executor::pregrasp_planning_authority
    restocker_task_executor::world_snapshot_projection
    restocker_task_executor::planning_scene_reconciliation
    restocker_task_executor::planning_scene_attachment
    restocker_task_executor::planning_scene_lease
    restocker_task_executor::obstacle_projection)
  target_compile_definitions(
    test_trajectory_scene_validity PRIVATE
      RESTOCKER_TEST_PRODUCT_CATALOG="${CMAKE_CURRENT_SOURCE_DIR}/../restocker_description/config/product_collision_catalog.yaml")
  target_compile_options(test_trajectory_scene_validity PRIVATE -Wall -Wextra -Wpedantic)

  ament_add_gtest(test_manipulation_geometry test/test_manipulation_geometry.cpp)
  target_link_libraries(
    test_manipulation_geometry restocker_task_executor::manipulation_geometry)
  target_compile_definitions(
    test_manipulation_geometry
    PRIVATE
      RESTOCKER_TEST_WORKCELL_GEOMETRY="${CMAKE_CURRENT_SOURCE_DIR}/../restocker_description/config/workcell_geometry.yaml"
  )
  target_compile_options(test_manipulation_geometry PRIVATE -Wall -Wextra -Wpedantic)

  ament_add_gtest(test_task_selection test/test_task_selection.cpp)
  target_link_libraries(test_task_selection restocker_task_executor::task_selection)
  target_compile_definitions(
    test_task_selection
    PRIVATE
      RESTOCKER_TEST_PRODUCT_CATALOG="${CMAKE_CURRENT_SOURCE_DIR}/../restocker_description/config/product_collision_catalog.yaml"
      RESTOCKER_TEST_WORKCELL_GEOMETRY="${CMAKE_CURRENT_SOURCE_DIR}/../restocker_description/config/workcell_geometry.yaml"
      RESTOCKER_TEST_BASELINE_SCENARIO="${CMAKE_CURRENT_SOURCE_DIR}/../restocker_gazebo/config/baseline_products.yaml"
  )
  target_compile_options(test_task_selection PRIVATE -Wall -Wextra -Wpedantic)

  ament_add_gtest(test_grasp_candidates test/test_grasp_candidates.cpp)
  target_link_libraries(test_grasp_candidates restocker_task_executor::grasp_candidates)
  target_compile_options(test_grasp_candidates PRIVATE -Wall -Wextra -Wpedantic)

  ament_add_gtest(test_gripper_geometry test/test_gripper_geometry.cpp)
  target_link_libraries(test_gripper_geometry restocker_task_executor::gripper_geometry)
  target_compile_definitions(
    test_gripper_geometry
    PRIVATE
      RESTOCKER_TEST_GRIPPER_GEOMETRY="${CMAKE_CURRENT_SOURCE_DIR}/../restocker_description/config/gripper_geometry.yaml"
  )
  target_compile_options(test_gripper_geometry PRIVATE -Wall -Wextra -Wpedantic)

  ament_add_gtest(test_placement_candidates test/test_placement_candidates.cpp)
  target_link_libraries(
    test_placement_candidates restocker_task_executor::placement_candidates)
  target_compile_definitions(
    test_placement_candidates
    PRIVATE
      RESTOCKER_TEST_WORKCELL_GEOMETRY="${CMAKE_CURRENT_SOURCE_DIR}/../restocker_description/config/workcell_geometry.yaml"
  )
  target_compile_options(test_placement_candidates PRIVATE -Wall -Wextra -Wpedantic)

  ament_add_gtest(test_scene_geometry test/test_scene_geometry.cpp)
  target_link_libraries(test_scene_geometry restocker_task_executor::scene_geometry)
  target_compile_definitions(
    test_scene_geometry
    PRIVATE
      RESTOCKER_TEST_PRODUCT_CATALOG="${CMAKE_CURRENT_SOURCE_DIR}/../restocker_description/config/product_collision_catalog.yaml"
  )
  target_compile_options(test_scene_geometry PRIVATE -Wall -Wextra -Wpedantic)

  ament_add_gtest(test_world_snapshot_projection test/test_world_snapshot_projection.cpp)
  target_link_libraries(
    test_world_snapshot_projection
    restocker_task_executor::world_snapshot_projection
    restocker_task_executor::planning_scene_reconciliation
  )
  target_compile_definitions(
    test_world_snapshot_projection
    PRIVATE
      RESTOCKER_TEST_PRODUCT_CATALOG="${CMAKE_CURRENT_SOURCE_DIR}/../restocker_description/config/product_collision_catalog.yaml"
  )
  target_compile_options(
    test_world_snapshot_projection PRIVATE -Wall -Wextra -Wpedantic)

  ament_add_gtest(test_obstacle_projection test/test_obstacle_projection.cpp)
  target_link_libraries(
    test_obstacle_projection
    restocker_task_executor::obstacle_projection
    restocker_task_executor::planning_scene_reconciliation)
  target_compile_options(test_obstacle_projection PRIVATE -Wall -Wextra -Wpedantic)

  ament_add_gtest(
    test_planning_scene_reconciliation test/test_planning_scene_reconciliation.cpp)
  target_link_libraries(
    test_planning_scene_reconciliation
    restocker_task_executor::planning_scene_reconciliation
  )
  target_compile_options(
    test_planning_scene_reconciliation PRIVATE -Wall -Wextra -Wpedantic)

  ament_add_gtest(test_planning_scene_attachment test/test_planning_scene_attachment.cpp)
  target_link_libraries(
    test_planning_scene_attachment restocker_task_executor::planning_scene_attachment
  )
  target_compile_options(test_planning_scene_attachment PRIVATE -Wall -Wextra -Wpedantic)

  ament_add_gtest(test_planning_scene_lease test/test_planning_scene_lease.cpp)
  target_link_libraries(
    test_planning_scene_lease restocker_task_executor::planning_scene_lease)
  target_compile_options(test_planning_scene_lease PRIVATE -Wall -Wextra -Wpedantic)

  ament_add_gtest(
    test_planning_scene_lease_ros test/test_planning_scene_lease_ros.cpp)
  target_link_libraries(
    test_planning_scene_lease_ros restocker_task_executor::planning_scene_lease_ros)
  target_compile_options(test_planning_scene_lease_ros PRIVATE -Wall -Wextra -Wpedantic)

  ament_add_gtest(test_ros_gripper_port test/test_ros_gripper_port.cpp)
  target_link_libraries(
    test_ros_gripper_port restocker_task_executor::ros_gripper_port)
  target_compile_options(test_ros_gripper_port PRIVATE -Wall -Wextra -Wpedantic)
  ament_target_dependencies(
    test_ros_gripper_port control_msgs rclcpp rclcpp_action sensor_msgs)

  ament_lint_auto_find_test_dependencies()
endif()

ament_export_targets(export_restocker_task_executor HAS_LIBRARY_TARGET)
ament_export_dependencies(
  Eigen3 geometry_msgs moveit_msgs rclcpp rclcpp_action rcl_interfaces restocker_control
  restocker_interfaces restocker_perception restocker_reasoner restocker_world_state shape_msgs
  tf2_eigen tf2_ros urdf
  yaml-cpp)
ament_package()

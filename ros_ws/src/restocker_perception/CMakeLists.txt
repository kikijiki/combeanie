# Copyright 2026 Matteo Bernacchia <dev@kikijiki.com>
# SPDX-License-Identifier: MIT

cmake_minimum_required(VERSION 3.16)
project(restocker_perception VERSION 0.1.0 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

find_package(ament_cmake REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(rclcpp REQUIRED)
find_package(restocker_interfaces REQUIRED)
find_package(restocker_world_state REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_eigen REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(yaml-cpp REQUIRED)

# This package reads one depth stream for two unrelated purposes that share nothing but the stream.
# Product perception (frame_geometry, perception_port, colour_depth_backend) answers "where is a
# catalogued product" and would feed the executor; it ships behind perception:=false.
# Obstacle reconstruction (depth_obstacle_extraction) answers "what is
# in the way that no model describes" and feeds the planning scene. The target graphs are separate:
# neither links the other, so a change to one cannot silently move the other.

# --- Product detection and 6-DoF pose estimation -------------------------------------------------

add_library(frame_geometry src/frame_geometry.cpp)
add_library(restocker_perception::frame_geometry ALIAS frame_geometry)
target_include_directories(
  frame_geometry
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)
target_link_libraries(
  frame_geometry Eigen3::Eigen restocker_world_state::world_state)
target_compile_options(frame_geometry PRIVATE -Wall -Wextra -Wpedantic)

add_library(perception_port src/perception_port.cpp)
add_library(restocker_perception::perception_port ALIAS perception_port)
target_include_directories(
  perception_port
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(perception_port rclcpp restocker_interfaces sensor_msgs)
target_link_libraries(perception_port frame_geometry)
target_compile_options(perception_port PRIVATE -Wall -Wextra -Wpedantic)

# The deterministic backend is installed rather than confined to this package's tests: other
# packages need it to exercise their own perception consumers without a camera or a GPU.
add_library(fake_perception_backend src/fake_perception_backend.cpp)
add_library(restocker_perception::fake_perception_backend ALIAS fake_perception_backend)
target_include_directories(
  fake_perception_backend
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)
target_link_libraries(fake_perception_backend perception_port)
target_compile_options(fake_perception_backend PRIVATE -Wall -Wextra -Wpedantic)

# The production backend. It sits beside the fake: the fake lets a consumer be tested without a
# camera, and this runs when there is one.
add_library(
  colour_depth_backend src/colour_depth_backend.cpp src/observation_identity.cpp
  src/object_manifest.cpp)
add_library(restocker_perception::colour_depth_backend ALIAS colour_depth_backend)
target_include_directories(
  colour_depth_backend
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)
target_link_libraries(colour_depth_backend PUBLIC perception_port PRIVATE yaml-cpp::yaml-cpp)
target_compile_options(colour_depth_backend PRIVATE -Wall -Wextra -Wpedantic)

# --- Aiming the wrist camera -------------------------------------------------------------------

# The viewpoint boundary. It converts a desired pose of wrist_camera_optical_frame into a tool0
# goal through the fixed mount transform, and is the only place that transform is applied.
add_library(
  viewpoint src/viewpoint_geometry.cpp src/viewpoint_port.cpp src/survey_stations.cpp)
add_library(restocker_perception::viewpoint ALIAS viewpoint)
target_include_directories(
  viewpoint
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)
target_link_libraries(viewpoint PUBLIC frame_geometry PRIVATE yaml-cpp::yaml-cpp)
target_compile_options(viewpoint PRIVATE -Wall -Wextra -Wpedantic)

# The TF-backed implementation, in its own target so a consumer that only needs the geometry (a
# static check, a test, an offline tool) does not link a TF listener.
add_library(tf_viewpoint_port src/tf_viewpoint_port.cpp)
add_library(restocker_perception::tf_viewpoint_port ALIAS tf_viewpoint_port)
target_include_directories(
  tf_viewpoint_port
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(tf_viewpoint_port rclcpp tf2 tf2_eigen tf2_ros)
target_link_libraries(tf_viewpoint_port viewpoint)
target_compile_options(tf_viewpoint_port PRIVATE -Wall -Wextra -Wpedantic)

add_library(pose_error_evaluation src/pose_error_evaluation.cpp)
add_library(restocker_perception::pose_error_evaluation ALIAS pose_error_evaluation)
target_include_directories(
  pose_error_evaluation
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(pose_error_evaluation rclcpp restocker_interfaces)
target_link_libraries(pose_error_evaluation frame_geometry)
target_compile_options(pose_error_evaluation PRIVATE -Wall -Wextra -Wpedantic)

add_executable(perception_node src/perception_node.cpp)
ament_target_dependencies(perception_node rclcpp sensor_msgs tf2_eigen tf2_ros)
target_link_libraries(perception_node colour_depth_backend)
target_compile_options(perception_node PRIVATE -Wall -Wextra -Wpedantic)

add_executable(pose_error_evaluator_node src/pose_error_evaluator_node.cpp)
ament_target_dependencies(
  pose_error_evaluator_node rclcpp restocker_interfaces tf2 tf2_eigen tf2_ros)
target_link_libraries(pose_error_evaluator_node pose_error_evaluation)
target_compile_options(pose_error_evaluator_node PRIVATE -Wall -Wextra -Wpedantic)

# --- Reconstruction of unmodelled obstacles ------------------------------------------------------

add_library(depth_obstacle_extraction src/depth_obstacle_extraction.cpp)
add_library(restocker_perception::depth_obstacle_extraction ALIAS depth_obstacle_extraction)
target_include_directories(
  depth_obstacle_extraction
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)
target_compile_options(depth_obstacle_extraction PRIVATE -Wall -Wextra -Wpedantic)
target_link_libraries(depth_obstacle_extraction Eigen3::Eigen)

add_executable(depth_obstacle_node src/depth_obstacle_node.cpp)
target_link_libraries(depth_obstacle_node depth_obstacle_extraction)
target_compile_options(depth_obstacle_node PRIVATE -Wall -Wextra -Wpedantic)
ament_target_dependencies(
  depth_obstacle_node
  rclcpp
  restocker_interfaces
  sensor_msgs
  tf2
  tf2_eigen
  tf2_ros
)

# --- Lane depletion, measured from the robot's own side -------------------------------------------

add_library(lane_depth_measurement src/lane_depth_measurement.cpp)
add_library(restocker_perception::lane_depth_measurement ALIAS lane_depth_measurement)
target_include_directories(
  lane_depth_measurement
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)
target_link_libraries(lane_depth_measurement depth_obstacle_extraction viewpoint)
target_compile_options(lane_depth_measurement PRIVATE -Wall -Wextra -Wpedantic)

add_library(lane_depth_evaluation src/lane_depth_evaluation.cpp)
add_library(restocker_perception::lane_depth_evaluation ALIAS lane_depth_evaluation)
target_include_directories(
  lane_depth_evaluation
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(lane_depth_evaluation rclcpp restocker_interfaces)
target_compile_options(lane_depth_evaluation PRIVATE -Wall -Wextra -Wpedantic)

add_executable(lane_observation_node src/lane_observation_node.cpp)
ament_target_dependencies(
  lane_observation_node rclcpp restocker_interfaces sensor_msgs tf2 tf2_eigen tf2_ros)
target_link_libraries(lane_observation_node lane_depth_measurement)
target_compile_options(lane_observation_node PRIVATE -Wall -Wextra -Wpedantic)

add_executable(lane_depth_error_evaluator_node src/lane_depth_error_evaluator_node.cpp)
ament_target_dependencies(lane_depth_error_evaluator_node rclcpp restocker_interfaces)
target_link_libraries(lane_depth_error_evaluator_node lane_depth_evaluation)
target_compile_options(lane_depth_error_evaluator_node PRIVATE -Wall -Wextra -Wpedantic)

# --- Installation --------------------------------------------------------------------------------

install(DIRECTORY include/ DESTINATION include)
install(DIRECTORY config DESTINATION share/${PROJECT_NAME})
install(
  TARGETS perception_node pose_error_evaluator_node depth_obstacle_node lane_observation_node
          lane_depth_error_evaluator_node
  RUNTIME DESTINATION lib/${PROJECT_NAME})
install(
  TARGETS frame_geometry perception_port fake_perception_backend colour_depth_backend
          pose_error_evaluation depth_obstacle_extraction viewpoint tf_viewpoint_port
          lane_depth_measurement lane_depth_evaluation
  EXPORT export_restocker_perception
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
  INCLUDES DESTINATION include
)

ament_export_targets(export_restocker_perception HAS_LIBRARY_TARGET)
ament_export_dependencies(
  Eigen3 rclcpp restocker_interfaces restocker_world_state sensor_msgs tf2 tf2_eigen tf2_ros
  yaml-cpp)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  find_package(ament_cmake_pytest REQUIRED)
  find_package(ament_lint_auto REQUIRED)

  ament_add_gtest(test_frame_geometry test/test_frame_geometry.cpp)
  target_link_libraries(test_frame_geometry restocker_perception::frame_geometry)
  target_compile_options(test_frame_geometry PRIVATE -Wall -Wextra -Wpedantic)

  ament_add_gtest(test_fake_perception_backend test/test_fake_perception_backend.cpp)
  target_link_libraries(
    test_fake_perception_backend restocker_perception::fake_perception_backend)
  target_compile_options(test_fake_perception_backend PRIVATE -Wall -Wextra -Wpedantic)

  ament_add_gtest(test_observation_admission test/test_observation_admission.cpp)
  target_link_libraries(
    test_observation_admission
    restocker_perception::fake_perception_backend
    restocker_world_state::ros_conversions)
  target_compile_options(test_observation_admission PRIVATE -Wall -Wextra -Wpedantic)

  ament_add_gtest(test_colour_depth_backend test/test_colour_depth_backend.cpp)
  target_link_libraries(
    test_colour_depth_backend restocker_perception::colour_depth_backend)
  target_compile_options(test_colour_depth_backend PRIVATE -Wall -Wextra -Wpedantic)

  ament_add_gtest(test_non_upright_container test/test_non_upright_container.cpp)
  target_link_libraries(
    test_non_upright_container restocker_perception::colour_depth_backend)
  target_compile_options(test_non_upright_container PRIVATE -Wall -Wextra -Wpedantic)

  ament_add_gtest(test_object_manifest test/test_object_manifest.cpp)
  target_link_libraries(test_object_manifest restocker_perception::colour_depth_backend)
  target_compile_options(test_object_manifest PRIVATE -Wall -Wextra -Wpedantic)
  set_tests_properties(
    test_object_manifest PROPERTIES ENVIRONMENT
    "RESTOCKER_TEST_BASELINE_SCENARIO=${CMAKE_CURRENT_SOURCE_DIR}/../restocker_gazebo/config/baseline_products.yaml;RESTOCKER_TEST_DENSE_SCENARIO=${CMAKE_CURRENT_SOURCE_DIR}/../restocker_gazebo/config/dense_restock_products.yaml")

  ament_add_gtest(test_viewpoint_geometry test/test_viewpoint_geometry.cpp)
  target_link_libraries(test_viewpoint_geometry restocker_perception::viewpoint)
  target_compile_options(test_viewpoint_geometry PRIVATE -Wall -Wextra -Wpedantic)

  ament_add_gtest(test_survey_stations test/test_survey_stations.cpp)
  target_link_libraries(test_survey_stations restocker_perception::viewpoint)
  target_compile_options(test_survey_stations PRIVATE -Wall -Wextra -Wpedantic)
  # The shipped workcell, not a fixture: the stations are derived from these numbers.
  set_tests_properties(
    test_survey_stations PROPERTIES ENVIRONMENT
    "RESTOCKER_TEST_WORKCELL_GEOMETRY=${CMAKE_CURRENT_SOURCE_DIR}/../restocker_description/config/workcell_geometry.yaml;RESTOCKER_TEST_SENSORS_XACRO=${CMAKE_CURRENT_SOURCE_DIR}/../restocker_description/urdf/sensors.xacro")

  ament_add_gtest(test_confirmation_viewpoint test/test_confirmation_viewpoint.cpp)
  target_link_libraries(test_confirmation_viewpoint restocker_perception::viewpoint)
  target_compile_options(test_confirmation_viewpoint PRIVATE -Wall -Wextra -Wpedantic)
  # The description's own sensor declaration: the framing check is against the sensor that ships.
  set_tests_properties(
    test_confirmation_viewpoint PROPERTIES ENVIRONMENT
    "RESTOCKER_TEST_SENSORS_XACRO=${CMAKE_CURRENT_SOURCE_DIR}/../restocker_description/urdf/sensors.xacro")

  ament_add_gtest(test_fake_viewpoint_port test/test_fake_viewpoint_port.cpp)
  target_link_libraries(test_fake_viewpoint_port restocker_perception::viewpoint)
  target_compile_options(test_fake_viewpoint_port PRIVATE -Wall -Wextra -Wpedantic)

  ament_add_gtest(test_pose_error_evaluation test/test_pose_error_evaluation.cpp)
  target_link_libraries(
    test_pose_error_evaluation restocker_perception::pose_error_evaluation)
  target_compile_options(test_pose_error_evaluation PRIVATE -Wall -Wextra -Wpedantic)

  ament_add_gtest(test_tray_duty_parameters test/test_tray_duty_parameters.cpp)
  target_link_libraries(
    test_tray_duty_parameters
    restocker_perception::colour_depth_backend
    restocker_perception::pose_error_evaluation)
  target_compile_options(test_tray_duty_parameters PRIVATE -Wall -Wextra -Wpedantic)

  ament_add_gtest(test_lane_depth_measurement test/test_lane_depth_measurement.cpp)
  target_link_libraries(
    test_lane_depth_measurement restocker_perception::lane_depth_measurement)
  target_compile_options(test_lane_depth_measurement PRIVATE -Wall -Wextra -Wpedantic)
  # The shipped workcell again. The lane window is derived from these numbers (the incline above
  # all), and a fixture would let the file and the measurement drift apart unnoticed.
  set_tests_properties(
    test_lane_depth_measurement PROPERTIES ENVIRONMENT
    "RESTOCKER_TEST_WORKCELL_GEOMETRY=${CMAKE_CURRENT_SOURCE_DIR}/../restocker_description/config/workcell_geometry.yaml")

  ament_add_gtest(test_lane_depth_evaluation test/test_lane_depth_evaluation.cpp)
  target_link_libraries(
    test_lane_depth_evaluation restocker_perception::lane_depth_evaluation)
  target_compile_options(test_lane_depth_evaluation PRIVATE -Wall -Wextra -Wpedantic)

  ament_add_gtest(
    test_depth_obstacle_extraction test/test_depth_obstacle_extraction.cpp)
  target_link_libraries(
    test_depth_obstacle_extraction restocker_perception::depth_obstacle_extraction)
  target_compile_options(test_depth_obstacle_extraction PRIVATE -Wall -Wextra -Wpedantic)

  ament_add_pytest_test(
    test_signature_catalog_agreement test/test_signature_catalog_agreement.py
    ENV
      RESTOCKER_TEST_PERCEPTION_CONFIG=${CMAKE_CURRENT_SOURCE_DIR}/config/overhead_rgbd_perception.yaml
      RESTOCKER_TEST_TRAY_PERCEPTION_CONFIG=${CMAKE_CURRENT_SOURCE_DIR}/config/wrist_tray_perception.yaml
      RESTOCKER_TEST_PRODUCT_CATALOG=${CMAKE_CURRENT_SOURCE_DIR}/../restocker_description/config/product_collision_catalog.yaml
      RESTOCKER_TEST_BASELINE_SCENARIO=${CMAKE_CURRENT_SOURCE_DIR}/../restocker_gazebo/config/baseline_products.yaml)

  ament_add_pytest_test(
    test_wrist_tray_topics test/test_wrist_tray_topics.py
    ENV
      RESTOCKER_TEST_TRAY_PERCEPTION_CONFIG=${CMAKE_CURRENT_SOURCE_DIR}/config/wrist_tray_perception.yaml)

  ament_lint_auto_find_test_dependencies()
endif()

ament_package()

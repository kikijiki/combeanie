# Copyright 2026 Matteo Bernacchia <dev@kikijiki.com>
# SPDX-License-Identifier: MIT

cmake_minimum_required(VERSION 3.16)
project(restocker_gazebo VERSION 0.1.0 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(gz-msgs10 REQUIRED)
find_package(gz-sim8 REQUIRED)
find_package(gz-transport13 REQUIRED)
find_package(Protobuf REQUIRED)
find_package(rclcpp REQUIRED)
find_package(restocker_interfaces REQUIRED)
find_package(yaml-cpp REQUIRED)

ament_python_install_package(${PROJECT_NAME})

add_library(attachment_journal src/attachment_journal.cpp)
add_library(restocker_gazebo::attachment_journal ALIAS attachment_journal)
target_include_directories(
  attachment_journal
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
)
target_compile_options(attachment_journal PRIVATE -Wall -Wextra -Wpedantic)

add_library(attachment_protocol msgs/attachment.proto src/attachment_protocol.cpp)
add_library(restocker_gazebo::attachment_protocol ALIAS attachment_protocol)
protobuf_generate(TARGET attachment_protocol)
target_include_directories(
  attachment_protocol
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<BUILD_INTERFACE:${CMAKE_CURRENT_BINARY_DIR}>
)
target_link_libraries(attachment_protocol PUBLIC attachment_journal protobuf::libprotobuf)
target_compile_options(attachment_protocol PRIVATE -Wall -Wextra -Wpedantic)

add_library(attachment_transport_client src/attachment_transport_client.cpp)
add_library(restocker_gazebo::attachment_transport_client ALIAS attachment_transport_client)
target_include_directories(
  attachment_transport_client
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
)
target_link_libraries(
  attachment_transport_client PUBLIC attachment_protocol gz-transport13::gz-transport13
)
target_compile_options(attachment_transport_client PRIVATE -Wall -Wextra -Wpedantic)

add_library(attachment_transport_worker src/attachment_transport_worker.cpp)
add_library(restocker_gazebo::attachment_transport_worker ALIAS attachment_transport_worker)
target_include_directories(
  attachment_transport_worker
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
)
target_link_libraries(attachment_transport_worker PUBLIC attachment_transport_client)
target_compile_options(attachment_transport_worker PRIVATE -Wall -Wextra -Wpedantic)

add_library(attachment_mailbox src/attachment_mailbox.cpp)
add_library(restocker_gazebo::attachment_mailbox ALIAS attachment_mailbox)
target_include_directories(
  attachment_mailbox
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
)
target_link_libraries(attachment_mailbox PUBLIC attachment_protocol)
target_compile_options(attachment_mailbox PRIVATE -Wall -Wextra -Wpedantic)

add_library(attachment_physics src/attachment_physics.cpp)
add_library(restocker_gazebo::attachment_physics ALIAS attachment_physics)
target_include_directories(
  attachment_physics
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
)
target_link_libraries(attachment_physics PUBLIC attachment_journal Eigen3::Eigen)
target_compile_options(attachment_physics PRIVATE -Wall -Wextra -Wpedantic)

add_library(attachment_config src/attachment_config.cpp)
add_library(restocker_gazebo::attachment_config ALIAS attachment_config)
target_include_directories(
  attachment_config
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
)
target_link_libraries(attachment_config PUBLIC attachment_physics yaml-cpp::yaml-cpp)
target_compile_options(attachment_config PRIVATE -Wall -Wextra -Wpedantic)

add_library(attachment_verifier src/attachment_verifier.cpp)
add_library(restocker_gazebo::attachment_verifier ALIAS attachment_verifier)
target_include_directories(
  attachment_verifier
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
)
target_link_libraries(attachment_verifier PUBLIC attachment_journal Eigen3::Eigen)
target_compile_options(attachment_verifier PRIVATE -Wall -Wextra -Wpedantic)

add_library(attachment_adapter_journal src/attachment_adapter_journal.cpp)
add_library(restocker_gazebo::attachment_adapter_journal ALIAS attachment_adapter_journal)
target_include_directories(
  attachment_adapter_journal
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
)
target_link_libraries(attachment_adapter_journal PUBLIC attachment_journal)
target_compile_options(attachment_adapter_journal PRIVATE -Wall -Wextra -Wpedantic)

add_library(attachment_adapter_ros src/attachment_adapter_ros.cpp)
add_library(restocker_gazebo::attachment_adapter_ros ALIAS attachment_adapter_ros)
target_include_directories(
  attachment_adapter_ros
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
)
ament_target_dependencies(attachment_adapter_ros restocker_interfaces)
target_link_libraries(attachment_adapter_ros attachment_adapter_journal attachment_config)
target_compile_options(attachment_adapter_ros PRIVATE -Wall -Wextra -Wpedantic)

add_library(restocker_attachment_system SHARED src/attachment_system.cpp)
target_link_libraries(
  restocker_attachment_system
  PRIVATE
    attachment_config
    attachment_mailbox
    attachment_physics
    attachment_verifier
    gz-sim8::gz-sim8
)
target_compile_options(restocker_attachment_system PRIVATE -Wall -Wextra -Wpedantic)

add_library(ground_truth_conversion src/ground_truth_conversion.cpp)
add_library(restocker_gazebo::ground_truth_conversion ALIAS ground_truth_conversion)
target_include_directories(
  ground_truth_conversion
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
)
ament_target_dependencies(ground_truth_conversion rclcpp restocker_interfaces)
target_link_libraries(ground_truth_conversion gz-msgs10::gz-msgs10 yaml-cpp::yaml-cpp)
target_compile_options(ground_truth_conversion PRIVATE -Wall -Wextra -Wpedantic)

add_library(lane_evidence src/lane_evidence.cpp)
add_library(restocker_gazebo::lane_evidence ALIAS lane_evidence)
target_include_directories(
  lane_evidence
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
)
ament_target_dependencies(lane_evidence restocker_interfaces)
target_link_libraries(
  lane_evidence Eigen3::Eigen ground_truth_conversion gz-msgs10::gz-msgs10 yaml-cpp::yaml-cpp
)
target_compile_options(lane_evidence PRIVATE -Wall -Wextra -Wpedantic)

add_library(ground_truth_adapter_ros src/ground_truth_adapter_ros.cpp)
add_library(restocker_gazebo::ground_truth_adapter_ros ALIAS ground_truth_adapter_ros)
target_include_directories(
  ground_truth_adapter_ros
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
)
ament_target_dependencies(ground_truth_adapter_ros rclcpp restocker_interfaces)
target_link_libraries(
  ground_truth_adapter_ros
  ground_truth_conversion
  lane_evidence
  gz-transport13::gz-transport13
)
target_compile_options(ground_truth_adapter_ros PRIVATE -Wall -Wextra -Wpedantic)

add_executable(ground_truth_adapter src/ground_truth_adapter.cpp)
ament_target_dependencies(ground_truth_adapter rclcpp)
target_link_libraries(ground_truth_adapter ground_truth_adapter_ros)
target_compile_options(ground_truth_adapter PRIVATE -Wall -Wextra -Wpedantic)

add_executable(attachment_adapter_node src/attachment_adapter_node.cpp)
ament_target_dependencies(
  attachment_adapter_node ament_index_cpp rclcpp restocker_interfaces
)
target_link_libraries(
  attachment_adapter_node
  attachment_adapter_ros
  attachment_transport_worker
)
target_compile_options(attachment_adapter_node PRIVATE -Wall -Wextra -Wpedantic)

install(
  DIRECTORY config launch materials urdf worlds
  DESTINATION share/${PROJECT_NAME}
  PATTERN "__pycache__" EXCLUDE
  PATTERN "*.pyc" EXCLUDE
)
install(
  TARGETS attachment_adapter_node ground_truth_adapter restocker_attachment_system
  RUNTIME DESTINATION lib/${PROJECT_NAME}
  LIBRARY DESTINATION lib
)

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  find_package(ament_cmake_gtest REQUIRED)
  find_package(ament_lint_auto REQUIRED)
  find_package(launch_testing_ament_cmake REQUIRED)
  # Launch tests lease their ROS domain and Gazebo partition at run time so concurrent worktrees
  # and successive runs cannot see each other's nodes. See cmake/restocker_test_isolation.cmake.
  include(${CMAKE_CURRENT_SOURCE_DIR}/../../../cmake/restocker_test_isolation.cmake)

  ament_add_pytest_test(
    test_simulation_description
    test/test_simulation_description.py
    ENV
      RESTOCKER_CONTROL_SOURCE_DIR=${CMAKE_CURRENT_SOURCE_DIR}/../restocker_control
      RESTOCKER_DESCRIPTION_SOURCE_DIR=${CMAKE_CURRENT_SOURCE_DIR}/../restocker_description
      RESTOCKER_GAZEBO_SOURCE_DIR=${CMAKE_CURRENT_SOURCE_DIR}
      RESTOCKER_MOVEIT_CONFIG_SOURCE_DIR=${CMAKE_CURRENT_SOURCE_DIR}/../restocker_moveit_config
      RESTOCKER_TASK_EXECUTOR_SOURCE_DIR=${CMAKE_CURRENT_SOURCE_DIR}/../restocker_task_executor
      RESTOCKER_WORLD_STATE_SOURCE_DIR=${CMAKE_CURRENT_SOURCE_DIR}/../restocker_world_state
    TIMEOUT 60
  )
  ament_add_pytest_test(
    test_scenario_config
    test/test_scenario_config.py
    ENV
      RESTOCKER_CONTROL_SOURCE_DIR=${CMAKE_CURRENT_SOURCE_DIR}/../restocker_control
      RESTOCKER_DESCRIPTION_SOURCE_DIR=${CMAKE_CURRENT_SOURCE_DIR}/../restocker_description
      RESTOCKER_GAZEBO_SOURCE_DIR=${CMAKE_CURRENT_SOURCE_DIR}
      RESTOCKER_MOVEIT_CONFIG_SOURCE_DIR=${CMAKE_CURRENT_SOURCE_DIR}/../restocker_moveit_config
      RESTOCKER_TASK_EXECUTOR_SOURCE_DIR=${CMAKE_CURRENT_SOURCE_DIR}/../restocker_task_executor
      RESTOCKER_WORLD_STATE_SOURCE_DIR=${CMAKE_CURRENT_SOURCE_DIR}/../restocker_world_state
    TIMEOUT 60
  )
  ament_add_pytest_test(
    test_scenario_random
    test/test_scenario_random.py
    ENV
      RESTOCKER_CONTROL_SOURCE_DIR=${CMAKE_CURRENT_SOURCE_DIR}/../restocker_control
      RESTOCKER_DESCRIPTION_SOURCE_DIR=${CMAKE_CURRENT_SOURCE_DIR}/../restocker_description
      RESTOCKER_GAZEBO_SOURCE_DIR=${CMAKE_CURRENT_SOURCE_DIR}
      RESTOCKER_MOVEIT_CONFIG_SOURCE_DIR=${CMAKE_CURRENT_SOURCE_DIR}/../restocker_moveit_config
      RESTOCKER_TASK_EXECUTOR_SOURCE_DIR=${CMAKE_CURRENT_SOURCE_DIR}/../restocker_task_executor
      RESTOCKER_WORLD_STATE_SOURCE_DIR=${CMAKE_CURRENT_SOURCE_DIR}/../restocker_world_state
    TIMEOUT 600
  )
  ament_add_pytest_test(
    test_lane_columns
    test/test_lane_columns.py
    ENV
      RESTOCKER_CONTROL_SOURCE_DIR=${CMAKE_CURRENT_SOURCE_DIR}/../restocker_control
      RESTOCKER_DESCRIPTION_SOURCE_DIR=${CMAKE_CURRENT_SOURCE_DIR}/../restocker_description
      RESTOCKER_GAZEBO_SOURCE_DIR=${CMAKE_CURRENT_SOURCE_DIR}
      RESTOCKER_MOVEIT_CONFIG_SOURCE_DIR=${CMAKE_CURRENT_SOURCE_DIR}/../restocker_moveit_config
      RESTOCKER_TASK_EXECUTOR_SOURCE_DIR=${CMAKE_CURRENT_SOURCE_DIR}/../restocker_task_executor
      RESTOCKER_WORLD_STATE_SOURCE_DIR=${CMAKE_CURRENT_SOURCE_DIR}/../restocker_world_state
      RESTOCKER_TEST_WORKCELL_GEOMETRY=${CMAKE_CURRENT_SOURCE_DIR}/../restocker_description/config/workcell_geometry.yaml
      RESTOCKER_TEST_PRODUCT_CATALOG=${CMAKE_CURRENT_SOURCE_DIR}/../restocker_description/config/product_collision_catalog.yaml
    TIMEOUT 60
  )
  # Plain script, not ament_add_pytest_test: the launch-testing pytest plugin imports every
  # sibling test module in this directory during collection, and this file needs none of that.
  add_test(
    NAME test_wait_for_clock
    COMMAND ${Python3_EXECUTABLE} ${CMAKE_CURRENT_SOURCE_DIR}/test/test_wait_for_clock.py)
  set_tests_properties(test_wait_for_clock PROPERTIES TIMEOUT 60)
  # Pinned stock-tray matrix arrangements: pure scenario documents, no simulator.
  # Same environment shape as test_lane_columns: launch_testing probes sibling modules in this
  # directory while resolving collection, so every module-level env read must be satisfiable.
  ament_add_pytest_test(
    test_tray_arrangements
    test/test_tray_arrangements.py
    ENV
      RESTOCKER_CONTROL_SOURCE_DIR=${CMAKE_CURRENT_SOURCE_DIR}/../restocker_control
      RESTOCKER_DESCRIPTION_SOURCE_DIR=${CMAKE_CURRENT_SOURCE_DIR}/../restocker_description
      RESTOCKER_GAZEBO_SOURCE_DIR=${CMAKE_CURRENT_SOURCE_DIR}
      RESTOCKER_MOVEIT_CONFIG_SOURCE_DIR=${CMAKE_CURRENT_SOURCE_DIR}/../restocker_moveit_config
      RESTOCKER_TASK_EXECUTOR_SOURCE_DIR=${CMAKE_CURRENT_SOURCE_DIR}/../restocker_task_executor
      RESTOCKER_WORLD_STATE_SOURCE_DIR=${CMAKE_CURRENT_SOURCE_DIR}/../restocker_world_state
      RESTOCKER_TEST_WORKCELL_GEOMETRY=${CMAKE_CURRENT_SOURCE_DIR}/../restocker_description/config/workcell_geometry.yaml
      RESTOCKER_TEST_PRODUCT_CATALOG=${CMAKE_CURRENT_SOURCE_DIR}/../restocker_description/config/product_collision_catalog.yaml
    TIMEOUT 60
  )
  ament_add_gtest(test_ground_truth_conversion test/test_ground_truth_conversion.cpp)
  target_link_libraries(
    test_ground_truth_conversion
    restocker_gazebo::ground_truth_conversion
  )
  target_compile_definitions(
    test_ground_truth_conversion
    PRIVATE
      RESTOCKER_TEST_SCENARIO="${CMAKE_CURRENT_SOURCE_DIR}/config/baseline_products.yaml"
  )
  target_compile_options(test_ground_truth_conversion PRIVATE -Wall -Wextra -Wpedantic)
  ament_add_gtest(test_attachment_journal test/test_attachment_journal.cpp)
  target_link_libraries(test_attachment_journal restocker_gazebo::attachment_journal)
  target_compile_options(test_attachment_journal PRIVATE -Wall -Wextra -Wpedantic)
  ament_add_gtest(test_attachment_protocol test/test_attachment_protocol.cpp)
  target_link_libraries(
    test_attachment_protocol
    restocker_gazebo::attachment_journal
    restocker_gazebo::attachment_protocol
  )
  target_compile_options(test_attachment_protocol PRIVATE -Wall -Wextra -Wpedantic)
  ament_add_gtest(test_attachment_transport_client test/test_attachment_transport_client.cpp)
  target_link_libraries(
    test_attachment_transport_client restocker_gazebo::attachment_transport_client
  )
  target_compile_options(test_attachment_transport_client PRIVATE -Wall -Wextra -Wpedantic)
  ament_add_gtest(test_attachment_transport_worker test/test_attachment_transport_worker.cpp)
  target_link_libraries(
    test_attachment_transport_worker restocker_gazebo::attachment_transport_worker
  )
  target_compile_options(test_attachment_transport_worker PRIVATE -Wall -Wextra -Wpedantic)
  ament_add_gtest(test_attachment_mailbox test/test_attachment_mailbox.cpp)
  target_link_libraries(test_attachment_mailbox restocker_gazebo::attachment_mailbox)
  target_compile_options(test_attachment_mailbox PRIVATE -Wall -Wextra -Wpedantic)
  ament_add_gtest(test_attachment_physics test/test_attachment_physics.cpp)
  target_link_libraries(test_attachment_physics restocker_gazebo::attachment_physics)
  target_compile_options(test_attachment_physics PRIVATE -Wall -Wextra -Wpedantic)
  ament_add_gtest(test_attachment_config test/test_attachment_config.cpp)
  target_link_libraries(test_attachment_config restocker_gazebo::attachment_config)
  target_compile_definitions(
    test_attachment_config
    PRIVATE
      RESTOCKER_TEST_ATTACHMENT_BOUNDARY="${CMAKE_CURRENT_SOURCE_DIR}/config/attachment_boundary.yaml"
      RESTOCKER_TEST_GRIPPER_GEOMETRY="${CMAKE_CURRENT_SOURCE_DIR}/../restocker_description/config/gripper_geometry.yaml"
      RESTOCKER_TEST_INVALID_BOUNDARY="${CMAKE_CURRENT_SOURCE_DIR}/test/fixtures/invalid_attachment_boundary.yaml"
      RESTOCKER_TEST_PRODUCT_CATALOG="${CMAKE_CURRENT_SOURCE_DIR}/../restocker_description/config/product_collision_catalog.yaml"
      RESTOCKER_TEST_SCENARIO="${CMAKE_CURRENT_SOURCE_DIR}/config/baseline_products.yaml"
      RESTOCKER_TEST_UNKNOWN_PRODUCT_SCENARIO="${CMAKE_CURRENT_SOURCE_DIR}/test/fixtures/unknown_attachment_product.yaml"
  )
  target_compile_options(test_attachment_config PRIVATE -Wall -Wextra -Wpedantic)
  ament_add_gtest(test_attachment_verifier test/test_attachment_verifier.cpp)
  target_link_libraries(test_attachment_verifier restocker_gazebo::attachment_verifier)
  target_compile_options(test_attachment_verifier PRIVATE -Wall -Wextra -Wpedantic)
  ament_add_gtest(
    test_attachment_adapter_journal
    test/test_attachment_adapter_journal.cpp
  )
  target_link_libraries(
    test_attachment_adapter_journal
    restocker_gazebo::attachment_adapter_journal
  )
  target_compile_options(test_attachment_adapter_journal PRIVATE -Wall -Wextra -Wpedantic)
  ament_add_gtest(test_attachment_adapter_ros test/test_attachment_adapter_ros.cpp)
  target_link_libraries(
    test_attachment_adapter_ros restocker_gazebo::attachment_adapter_ros
  )
  target_compile_options(test_attachment_adapter_ros PRIVATE -Wall -Wextra -Wpedantic)
  add_executable(attachment_physics_probe test/attachment_physics_probe.cpp)
  target_link_libraries(
    attachment_physics_probe
    PRIVATE
      restocker_gazebo::attachment_protocol
      gz-msgs10::gz-msgs10
      gz-transport13::gz-transport13
  )
  target_compile_options(attachment_physics_probe PRIVATE -Wall -Wextra -Wpedantic)
  add_executable(product_contact_probe test/product_contact_probe.cpp)
  target_link_libraries(
    product_contact_probe
    PRIVATE
      gz-msgs10::gz-msgs10
      gz-transport13::gz-transport13
  )
  target_compile_options(product_contact_probe PRIVATE -Wall -Wextra -Wpedantic)
  add_executable(release_contact_probe test/release_contact_probe.cpp)
  target_link_libraries(
    release_contact_probe
    PRIVATE
      restocker_gazebo::attachment_protocol
      gz-msgs10::gz-msgs10
      gz-transport13::gz-transport13
  )
  target_compile_options(release_contact_probe PRIVATE -Wall -Wextra -Wpedantic)
  ament_add_gtest(test_lane_evidence test/test_lane_evidence.cpp)
  target_link_libraries(test_lane_evidence restocker_gazebo::lane_evidence)
  target_compile_definitions(
    test_lane_evidence
    PRIVATE
      RESTOCKER_TEST_PRODUCT_CATALOG="${CMAKE_CURRENT_SOURCE_DIR}/../restocker_description/config/product_collision_catalog.yaml"
      RESTOCKER_TEST_SCENARIO="${CMAKE_CURRENT_SOURCE_DIR}/config/baseline_products.yaml"
      RESTOCKER_TEST_WORKCELL_GEOMETRY="${CMAKE_CURRENT_SOURCE_DIR}/../restocker_description/config/workcell_geometry.yaml"
  )
  target_compile_options(test_lane_evidence PRIVATE -Wall -Wextra -Wpedantic)
  ament_add_gtest(test_ground_truth_adapter_ros test/test_ground_truth_adapter_ros.cpp)
  target_link_libraries(
    test_ground_truth_adapter_ros restocker_gazebo::ground_truth_adapter_ros
  )
  ament_target_dependencies(test_ground_truth_adapter_ros rclcpp)
  target_compile_definitions(
    test_ground_truth_adapter_ros
    PRIVATE
      RESTOCKER_TEST_PRODUCT_CATALOG="${CMAKE_CURRENT_SOURCE_DIR}/../restocker_description/config/product_collision_catalog.yaml"
      RESTOCKER_TEST_SCENARIO="${CMAKE_CURRENT_SOURCE_DIR}/config/baseline_products.yaml"
      RESTOCKER_TEST_WORKCELL_GEOMETRY="${CMAKE_CURRENT_SOURCE_DIR}/../restocker_description/config/workcell_geometry.yaml"
  )
  target_compile_options(test_ground_truth_adapter_ros PRIVATE -Wall -Wextra -Wpedantic)
  add_launch_test(
    test/test_simulation_runtime.py
    TARGET test_simulation_runtime
    TIMEOUT 90
    ENV
      GZ_PARTITION=restocker_gazebo_test
  )
  add_launch_test(
    test/test_attachment_physics_runtime.py
    TARGET test_attachment_physics_runtime
    TIMEOUT 30
    ENV
      GZ_PARTITION=restocker_attachment_physics_test
      RESTOCKER_ATTACHMENT_PROBE=$<TARGET_FILE:attachment_physics_probe>
      RESTOCKER_ATTACHMENT_PLUGIN_DIR=$<TARGET_FILE_DIR:restocker_attachment_system>
      RESTOCKER_ATTACHMENT_TEST_WORLD=${CMAKE_CURRENT_SOURCE_DIR}/test/fixtures/attachment_physics.sdf
      RESTOCKER_TEST_ATTACHMENT_BOUNDARY=${CMAKE_CURRENT_SOURCE_DIR}/config/attachment_boundary.yaml
      RESTOCKER_TEST_GRIPPER_GEOMETRY=${CMAKE_CURRENT_SOURCE_DIR}/../restocker_description/config/gripper_geometry.yaml
      RESTOCKER_TEST_PRODUCT_CATALOG=${CMAKE_CURRENT_SOURCE_DIR}/../restocker_description/config/product_collision_catalog.yaml
      RESTOCKER_TEST_SCENARIO=${CMAKE_CURRENT_SOURCE_DIR}/config/baseline_products.yaml
  )
  add_launch_test(
    test/test_product_contact_runtime.py
    TARGET test_product_contact_runtime
    TIMEOUT 180
    ENV
      RESTOCKER_ATTACHMENT_PLUGIN_DIR=$<TARGET_FILE_DIR:restocker_attachment_system>
      RESTOCKER_PRODUCT_CONTACT_PROBE=$<TARGET_FILE:product_contact_probe>
  )
  add_launch_test(
    test/test_release_contact_runtime.py
    TARGET test_release_contact_runtime
    TIMEOUT 150
    ENV
      GZ_PARTITION=restocker_release_contact_test
      RESTOCKER_RELEASE_CONTACT_PROBE=$<TARGET_FILE:release_contact_probe>
      RESTOCKER_RELEASE_CONTACT_TEST_WORLD=${CMAKE_CURRENT_SOURCE_DIR}/test/fixtures/release_contact.sdf
      RESTOCKER_ATTACHMENT_PLUGIN_DIR=$<TARGET_FILE_DIR:restocker_attachment_system>
      RESTOCKER_TEST_ATTACHMENT_BOUNDARY=${CMAKE_CURRENT_SOURCE_DIR}/config/attachment_boundary.yaml
      RESTOCKER_TEST_GRIPPER_GEOMETRY=${CMAKE_CURRENT_SOURCE_DIR}/../restocker_description/config/gripper_geometry.yaml
      RESTOCKER_TEST_PRODUCT_CATALOG=${CMAKE_CURRENT_SOURCE_DIR}/../restocker_description/config/product_collision_catalog.yaml
      RESTOCKER_TEST_SCENARIO=${CMAKE_CURRENT_SOURCE_DIR}/config/baseline_products.yaml
  )
  # `restocking.sdf` carries the sensors system, so these need a working OpenGL stack and
  # cannot run in the Nix build sandbox; see the same label in restocker_bringup for the detail.
  # `test_attachment_physics_runtime` above is not labelled: its fixture world loads physics and
  # the attachment plugin only, so it passes under `nix flake check`. `test_product_contact_runtime`
  # loads no camera and might survive without a renderer, but it loads the world file that carries
  # the sensors system, so it is labelled. `test_release_contact_runtime` is the renderer-
  # independent product-contact test on purpose: its fixture loads physics, the scene broadcaster
  # and the attachment plugin only, so the release-path measurement runs under `nix flake check`.
  set_property(TEST test_simulation_runtime APPEND PROPERTY LABELS "renderer")
  set_property(TEST test_product_contact_runtime APPEND PROPERTY LABELS "renderer")
  # Same lock as restocker_bringup's renderer set: ctest schedules admission so a queued test's
  # TIMEOUT starts only when it actually runs; with the serial test recipe the two packages'
  # ctest pools never overlap either, leaving a single rendered world on the machine.
  set_property(
    TEST test_simulation_runtime test_product_contact_runtime
    APPEND PROPERTY RESOURCE_LOCK "restocker-renderer-simulator")

  ament_lint_auto_find_test_dependencies()
endif()

ament_package()

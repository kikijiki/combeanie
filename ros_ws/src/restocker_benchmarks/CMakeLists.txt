# Copyright 2026 Matteo Bernacchia <dev@kikijiki.com>
# SPDX-License-Identifier: MIT

cmake_minimum_required(VERSION 3.16)
project(restocker_benchmarks VERSION 0.1.0 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)

ament_python_install_package(${PROJECT_NAME})

install(
  PROGRAMS
    scripts/benchmark_run
    scripts/benchmark_report
    scripts/benchmark_paired
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()

  # The scenario table, the failure taxonomy and the report are pure: they take stored records
  # and return numbers and text, so they are tested without a simulator.
  ament_add_pytest_test(test_benchmark_taxonomy test/test_taxonomy.py TIMEOUT 60)
  ament_add_pytest_test(test_benchmark_report test/test_report.py TIMEOUT 60)
  ament_add_pytest_test(test_benchmark_recovery test/test_recovery.py TIMEOUT 60)
  # Predeclared paired UR10e transfer-reliability population, denominators, arm gate and
  # no-replacement policy: pure, no simulator, so the frozen statement is unit-testable offline.
  ament_add_pytest_test(test_transfer_reliability test/test_transfer_reliability.py TIMEOUT 60)
  # Paired driver write-side policy (fresh directory, retained attempts, aggregation): needs
  # rclpy to import the driver but starts no ROS node and no simulator.
  ament_add_pytest_test(test_paired_campaign test/test_paired_campaign.py TIMEOUT 60)
  # The runner's readiness gate (observed == stocked before dispatch, bounded timeout): needs
  # rclpy to import the runner but starts no ROS node and no simulator; fakes drive _drive.
  ament_add_pytest_test(test_runner_readiness test/test_runner_readiness.py TIMEOUT 60)

  # provenance.obstacle_perception (Card 023): each run records the effective depth-obstacle
  # pipeline value it launched with. Pure resolver checks plus the installed-default read.
  ament_add_pytest_test(
    test_runner_obstacle_provenance
    test/test_runner_obstacle_provenance.py
    TIMEOUT 60
  )
  # Predeclared wrist-tray matrix, attachment-budget arithmetic and offline summary: pure, no
  # simulator, so the campaign's acceptance comparator is unit-testable offline.
  # RESTOCKER_TEST_* pin the restated catalogue and attachment constants to the shipped YAML.
  ament_add_pytest_test(
    test_tray_perception_campaign
    test/test_tray_perception_campaign.py
    ENV
      RESTOCKER_TEST_ATTACHMENT_BOUNDARY=${CMAKE_CURRENT_SOURCE_DIR}/../restocker_gazebo/config/attachment_boundary.yaml
      RESTOCKER_TEST_GRIPPER_GEOMETRY=${CMAKE_CURRENT_SOURCE_DIR}/../restocker_description/config/gripper_geometry.yaml
      RESTOCKER_TEST_PRODUCT_CATALOG=${CMAKE_CURRENT_SOURCE_DIR}/../restocker_description/config/product_collision_catalog.yaml
      RESTOCKER_TEST_TRAY_PERCEPTION_CONFIG=${CMAKE_CURRENT_SOURCE_DIR}/../restocker_perception/config/wrist_tray_perception.yaml
    TIMEOUT 60
  )
endif()

ament_package()

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
    scripts/benchmark_perception
    scripts/benchmark_proposals
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY config DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()

  # The scenario table, the failure taxonomy and the report are pure: they take stored records
  # and return numbers and text, so they are tested without a simulator.
  ament_add_pytest_test(test_benchmark_taxonomy test/test_taxonomy.py TIMEOUT 60)
  ament_add_pytest_test(test_benchmark_report test/test_report.py TIMEOUT 60)
  ament_add_pytest_test(test_benchmark_recovery test/test_recovery.py TIMEOUT 60)
  # Perception benchmark (Card 109): scenes, labels, dataset hashing, frozen splits, run guard and
  # scorer. Pure: no simulator. The split file and catalogue are the committed ones.
  ament_add_pytest_test(
    test_perception_benchmark
    test/test_perception_benchmark.py
    ENV
      RESTOCKER_TEST_GENERATED_LABELS=${CMAKE_CURRENT_SOURCE_DIR}/../restocker_gazebo/config/generated_product_labels.yaml
      RESTOCKER_TEST_PERCEPTION_SPLITS=${CMAKE_CURRENT_SOURCE_DIR}/config/perception_splits.json
    TIMEOUT 120
  )
  # Append-only version-2 split file (Card 112 round 2): old assignments preserved, burned and
  # fresh held-out seeds, reproducible from the file alone.
  ament_add_pytest_test(
    test_perception_splits_v2
    test/test_perception_splits_v2.py
    ENV
      RESTOCKER_TEST_GENERATED_LABELS=${CMAKE_CURRENT_SOURCE_DIR}/../restocker_gazebo/config/generated_product_labels.yaml
    TIMEOUT 120
  )
  # Append-only version-3 split file (CMB-119): the box SKUs and seeds 235..284, v1 and v2 intact.
  ament_add_pytest_test(
    test_perception_splits_v3
    test/test_perception_splits_v3.py
    ENV
      RESTOCKER_TEST_GENERATED_LABELS=${CMAKE_CURRENT_SOURCE_DIR}/../restocker_gazebo/config/generated_product_labels.yaml
    TIMEOUT 180
  )
  # Version-4 split file (CMB-120 round 2): append-only over v3.
  ament_add_pytest_test(
    test_perception_splits_v4
    test/test_perception_splits_v4.py
    ENV
      RESTOCKER_TEST_GENERATED_LABELS=${CMAKE_CURRENT_SOURCE_DIR}/../restocker_gazebo/config/generated_product_labels.yaml
    TIMEOUT 180
  )
  # Version-5 split file (CMB-112 round 3): identity-round roles, burned generations, re-look views
  # and the rotation oracle of the recorder.
  ament_add_pytest_test(
    test_perception_splits_v5
    test/test_perception_splits_v5.py
    ENV
      RESTOCKER_TEST_GENERATED_LABELS=${CMAKE_CURRENT_SOURCE_DIR}/../restocker_gazebo/config/generated_product_labels.yaml
    TIMEOUT 180
  )
  # Version-6 split file (CMB-121 round 4): seeds appended, round 3's decision seeds burned.
  ament_add_pytest_test(
    test_perception_splits_v6
    test/test_perception_splits_v6.py
    TIMEOUT 60
  )
  # Proposal benchmark (Card 113): scenes, survey views, analytic ground truth, scoring, bounds
  # rule. The proposer-on-synthetic-depth cases need cv2 and skip without it.
  ament_add_pytest_test(
    test_perception_proposals
    test/test_perception_proposals.py
    ENV
      RESTOCKER_TEST_GENERATED_LABELS=${CMAKE_CURRENT_SOURCE_DIR}/../restocker_gazebo/config/generated_product_labels.yaml
    TIMEOUT 180
  )
  # Aimed barcode viewpoints (Card 112 round 2): geometry, reach filter, purity.
  ament_add_pytest_test(test_perception_barcode_views test/test_perception_barcode_views.py TIMEOUT 60)
  ament_add_pytest_test(test_perception_lane_views test/test_perception_lane_views.py TIMEOUT 60)
  ament_add_pytest_test(test_perception_stock_scorer test/test_perception_stock_scorer.py TIMEOUT 60)
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

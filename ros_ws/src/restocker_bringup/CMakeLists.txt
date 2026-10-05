# Copyright 2026 Matteo Bernacchia <dev@kikijiki.com>
# SPDX-License-Identifier: MIT

cmake_minimum_required(VERSION 3.16)
project(restocker_bringup VERSION 0.1.0 LANGUAGES NONE)

find_package(ament_cmake REQUIRED)

install(
  DIRECTORY launch rviz
  DESTINATION share/${PROJECT_NAME}
  PATTERN "__pycache__" EXCLUDE
  PATTERN "*.pyc" EXCLUDE
)

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  find_package(ament_lint_auto REQUIRED)
  find_package(launch_testing_ament_cmake REQUIRED)
  # Launch tests lease their ROS domain and Gazebo partition at run time so concurrent worktrees
  # cannot see each other's nodes. See cmake/restocker_test_isolation.cmake.
  include(${CMAKE_CURRENT_SOURCE_DIR}/../../../cmake/restocker_test_isolation.cmake)

  # Card 028: the demo camera-view RViz config and its opt-in wiring. Static file assertions
  # only — no GUI, no simulator, no lease — so it also runs under the Nix flake check.
  ament_add_pytest_test(test_demo_camera_view_config
    test/test_demo_camera_view_config.py
    ENV
      RESTOCKER_BRINGUP_SOURCE_DIR=${CMAKE_CURRENT_SOURCE_DIR}
      RESTOCKER_GAZEBO_SOURCE_DIR=${CMAKE_CURRENT_SOURCE_DIR}/../restocker_gazebo
      RESTOCKER_MOVEIT_CONFIG_SOURCE_DIR=${CMAKE_CURRENT_SOURCE_DIR}/../restocker_moveit_config
      RESTOCKER_REPO_ROOT=${CMAKE_CURRENT_SOURCE_DIR}/../../..
    TIMEOUT 60
  )

  # Card 023 review follow-up: cameras:=false must not leave the projector requiring evidence
  # that no depth node can produce. Static launch-structure and condition checks; no lease.
  # All four RESTOCKER_* dirs like test_demo_camera_view_config: collection imports sibling
  # modules in this directory, and one that raises at import poisons the whole session.
  ament_add_pytest_test(test_obstacle_perception_cameras_gate
    test/test_obstacle_perception_cameras_gate.py
    ENV
      RESTOCKER_BRINGUP_SOURCE_DIR=${CMAKE_CURRENT_SOURCE_DIR}
      RESTOCKER_GAZEBO_SOURCE_DIR=${CMAKE_CURRENT_SOURCE_DIR}/../restocker_gazebo
      RESTOCKER_MOVEIT_CONFIG_SOURCE_DIR=${CMAKE_CURRENT_SOURCE_DIR}/../restocker_moveit_config
      RESTOCKER_REPO_ROOT=${CMAKE_CURRENT_SOURCE_DIR}/../../..
    TIMEOUT 60
  )

  # Card 094: the dense gate's wiring contract (CMB-SPEC-14 section 2) — recipe, label
  # exclusions in both default host suites and the Nix check, registration without DISABLED,
  # and the dense test's own identity bound. Static file assertions only: no simulator, no
  # lease. All four RESTOCKER_* dirs like test_demo_camera_view_config: collection imports
  # sibling modules in this directory, and one that raises at import poisons the whole session.
  ament_add_pytest_test(test_dense_gate_wiring
    test/test_dense_gate_wiring.py
    ENV
      RESTOCKER_BRINGUP_SOURCE_DIR=${CMAKE_CURRENT_SOURCE_DIR}
      RESTOCKER_GAZEBO_SOURCE_DIR=${CMAKE_CURRENT_SOURCE_DIR}/../restocker_gazebo
      RESTOCKER_MOVEIT_CONFIG_SOURCE_DIR=${CMAKE_CURRENT_SOURCE_DIR}/../restocker_moveit_config
      RESTOCKER_REPO_ROOT=${CMAKE_CURRENT_SOURCE_DIR}/../../..
    TIMEOUT 60
  )

  # Card 043: the obstacle placement rule over recorded sample sets — the deterministic home of
  # the SC-003 runs 6/7 refusal (a box on the arm's own start state refuses every grasp
  # candidate). Static geometry over recorded data; no simulator, no lease. Collection imports
  # the sibling launch test module, so the source dir envs match the two tests above.
  ament_add_pytest_test(test_dynamic_obstacle_placement
    test/test_dynamic_obstacle_placement.py
    ENV
      RESTOCKER_BRINGUP_SOURCE_DIR=${CMAKE_CURRENT_SOURCE_DIR}
      RESTOCKER_GAZEBO_SOURCE_DIR=${CMAKE_CURRENT_SOURCE_DIR}/../restocker_gazebo
      RESTOCKER_MOVEIT_CONFIG_SOURCE_DIR=${CMAKE_CURRENT_SOURCE_DIR}/../restocker_moveit_config
      RESTOCKER_REPO_ROOT=${CMAKE_CURRENT_SOURCE_DIR}/../../..
    TIMEOUT 60
  )

  # Drives physics, planning and controllers through the public action: three complete transfers,
  # each roughly 40 s plus shared startup.
  add_launch_test(
    test/test_restock_manipulation_runtime.py
    TARGET test_restock_manipulation_runtime
    TIMEOUT 1800
    ENV
      GZ_PARTITION=restocker_manipulation_test
  )
  # The coordinator choosing its own product and lane, and placing into a lane that already holds
  # one. Three transfers, same budget.
  add_launch_test(
    test/test_restock_lane_routing_runtime.py
    TARGET test_restock_lane_routing_runtime
    TIMEOUT 1800
    ENV
      GZ_PARTITION=restocker_lane_routing_test
  )
  # End-to-end `just demo`: both initial surveys, the measured partial-front/dense-back inventory,
  # then transfers until every lane's surveyed capacity is filled or compatible tray stock runs out.
  add_launch_test(
    test/test_autonomous_restock_demo_runtime.py
    TARGET test_autonomous_restock_demo_runtime
    TIMEOUT 4500
    ENV
      GZ_PARTITION=restocker_autonomous_demo_acceptance
  )
  # Card 010 SC-001/SC-002: sensor-driven acceptance from an unordered mixed tray into three
  # depleted lanes, with the ground-truth subscription-graph and producer-authentication
  # assertions. Cold start on the wrist lane topic, no external transfer goal.
  add_launch_test(
    test/test_sensor_driven_acceptance_runtime.py
    TARGET test_sensor_driven_acceptance_runtime
    TIMEOUT 4500
    ENV
      GZ_PARTITION=restocker_sensor_acceptance
  )
  # Card 063: the dense restock scenario on the shipped sensor-driven default. Camera identity
  # names each of the tray's multi-instance products by pose; the front must fill and every
  # admitted observation must name the product ground truth has there.
  add_launch_test(
    test/test_dense_sensor_driven_runtime.py
    TARGET test_dense_sensor_driven_runtime
    TIMEOUT 4500
    ENV
      GZ_PARTITION=restocker_dense_sensor_driven
  )
  # Card 094 (CMB-SPEC-14 section 2): registered enabled, excluded from the default suites by
  # the `dense-gate` label instead of DISABLED — `just test` / `just test-package` pass
  # `--label-exclude dense-gate`, `nix flake check` excludes `renderer` (also carried below),
  # and `just dense-gate` is the single named path that runs it (one simulator slot, one run).
  # A DISABLED test cannot be selected by ctest at all, which forced every historical dev run
  # to edit the generated build tree; labels keep the exclusion explicit and greppable.
  set_property(TEST test_dense_sensor_driven_runtime APPEND PROPERTY LABELS "dense-gate")
  # Blocked lane: a lane that only holds products is not blocked (lanes are columns); a product
  # that overlaps it without being in it is the only condition that makes `obstructed` true. One
  # transfer.
  add_launch_test(
    test/test_restock_obstructed_lane_runtime.py
    TARGET test_restock_obstructed_lane_runtime
    TIMEOUT 900
    ENV
      GZ_PARTITION=restocker_obstructed_lane_test
  )
  # Unmodeled obstacles: two complete transfers plus a refused one (about 123 s with bring-up).
  add_launch_test(
    test/test_dynamic_obstacle_runtime.py
    TARGET test_dynamic_obstacle_runtime
    TIMEOUT 900
    ENV
      GZ_PARTITION=restocker_obstacles
  )
  # Follows an obstacle from the camera through `depth_obstacle_node` and the `obstacle_perception`
  # composition to the motion gate (the test above publishes its own observations). No transfer
  # completes: one bring-up plus sixty depth observations, about 38 s.
  add_launch_test(
    test/test_obstacle_perception_runtime.py
    TARGET test_obstacle_perception_runtime
    TIMEOUT 300
    ENV
      GZ_PARTITION=restocker_obstacle_perception
  )
  add_launch_test(
    test/test_baseline_runtime.py
    TARGET test_baseline_runtime
    TIMEOUT 120
    ENV
      GZ_PARTITION=restocker_baseline_test
  )
  add_launch_test(
    test/test_world_state_runtime.py
    TARGET test_world_state_runtime
    TIMEOUT 120
    ENV
      GZ_PARTITION=restocker_world_state_test
  )
  add_launch_test(
    test/test_planning_scene_projection_runtime.py
    TARGET test_planning_scene_projection_runtime
    TIMEOUT 120
    ENV
      GZ_PARTITION=restocker_scene_projection_test
  )
  add_launch_test(
    test/test_projector_apply_timeout_fence.py
    TARGET test_projector_apply_timeout_fence
    TIMEOUT 60
  )
  # Card 041: latency injection on the world-snapshot service — a service slower than the
  # projector's deadline must still be served, one that never answers must never certify.
  add_launch_test(
    test/test_projector_snapshot_latency.py
    TARGET test_projector_snapshot_latency
    TIMEOUT 120
  )
  # Card 071: a product older than the projector's evidence age is certified as an obstacle at
  # its last known pose, never a whole-scene stale verdict.
  add_launch_test(
    test/test_projector_aged_product.py
    TARGET test_projector_aged_product
    TIMEOUT 120
  )
  # RGB-D sensing: renders two cameras and measures calibration against simulator ground truth.
  # Needs the simulator but no planning.
  add_launch_test(
    test/test_camera_runtime.py
    TARGET test_camera_runtime
    TIMEOUT 420
    ENV
      GZ_PARTITION=restocker_camera
  )
  # Aiming the wrist camera: plans and executes to eight survey stations plus one confirmation
  # viewpoint, and sweeps IK and planning for every station against five seeds through
  # move_group's services. Separate from test_camera_runtime.py, whose publication-rate assertions
  # would turn flaky with move_group running.
  add_launch_test(
    test/test_wrist_viewpoint_runtime.py
    TARGET test_wrist_viewpoint_runtime
    TIMEOUT 2400
    ENV
      GZ_PARTITION=restocker_wrist_viewpoint
  )
  # Camera-produced object observations feed the world state, and the pose error against
  # simulator ground truth is measured. Waits on a 6 Hz sensor and three admitted products, then
  # samples for half a minute.
  add_launch_test(
    test/test_perception_runtime.py
    TARGET test_perception_runtime
    TIMEOUT 600
    ENV
      GZ_PARTITION=restocker_perception
  )
  # Lane survey: the arm visits all six lane stations and the free depth the camera reads is
  # compared against the simulator's, on known columns from empty to full. Six plan-and-execute
  # segments plus twenty-three static product spawns.
  add_launch_test(
    test/test_lane_survey_runtime.py
    TARGET test_lane_survey_runtime
    TIMEOUT 2400
    ENV
      GZ_PARTITION=restocker_lane_survey
  )
  # Coordinated tray survey: overview stations before confirmation, no world-state evidence
  # from overview alone, cancellation retaining candidates with no active motion goal, and a
  # typed refutation from a contradicted close-range identity. Six survey legs plus startup.
  add_launch_test(
    test/test_tray_survey_runtime.py
    TARGET test_tray_survey_runtime
    TIMEOUT 3600
    ENV
      GZ_PARTITION=restocker_tray_survey
  )
  # Card 050: the focused overview-dwell loop on the acceptance tray fixture — every stocked
  # station must admit frames in every dwell, with the per-station stage receipt live on the
  # real sensor graph and ground truth showing the survey's own motions never tip a product.
  # Four survey loops plus startup.
  add_launch_test(
    test/test_tray_overview_dwell_runtime.py
    TARGET test_tray_overview_dwell_runtime
    TIMEOUT 1200
    ENV
      GZ_PARTITION=restocker_tray_overview_dwell
  )
  # A blinded camera must not read as an empty lane. Uses a synthetic depth stream and one static
  # frame, so it needs no simulator and runs in the Nix sandbox.
  add_launch_test(
    test/test_lane_observation_blindness.py
    TARGET test_lane_observation_blindness
    TIMEOUT 180
  )
  # Depth pipeline, driven by a synthetic image so it needs no camera.
  add_launch_test(
    test/test_depth_obstacle_node_runtime.py
    TARGET test_depth_obstacle_node_runtime
    TIMEOUT 120
  )
  # Fail-closed obstacle evidence contract, without the simulator.
  add_launch_test(
    test/test_obstacle_evidence_retention.py
    TARGET test_obstacle_evidence_retention
    TIMEOUT 180
  )
  add_launch_test(
    test/test_attachment_adapter_runtime.py
    TARGET test_attachment_adapter_runtime
    TIMEOUT 120
    ENV
      GZ_PARTITION=restocker_attachment_adapter_test
  )

  # Tests that start `gz sim` on a world with camera sensors. Gazebo loads Ogre2 for those even
  # headless, and the Nix build sandbox has no OpenGL (no /run/opengl-driver, no libEGL), so Ogre
  # segfaults in `Ogre2RenderEngine::CreateRenderSystem`. `nix flake check` deselects this label;
  # `just test` runs them on the host driver. A simulator test added without it fails the flake
  # check.
  #
  # test_projector_apply_timeout_fence, test_projector_snapshot_latency,
  # test_projector_aged_product, test_depth_obstacle_node_runtime and
  # test_obstacle_evidence_retention start no simulator, so they run in the sandbox and are
  # unlabelled.
  set_property(TEST
    test_restock_manipulation_runtime
    test_restock_lane_routing_runtime
    test_autonomous_restock_demo_runtime
    test_sensor_driven_acceptance_runtime
    test_dense_sensor_driven_runtime
    test_restock_obstructed_lane_runtime
    test_dynamic_obstacle_runtime
    test_obstacle_perception_runtime
    test_baseline_runtime
    test_world_state_runtime
    test_planning_scene_projection_runtime
    test_camera_runtime
    test_wrist_viewpoint_runtime
    test_lane_survey_runtime
    test_tray_survey_runtime
    test_tray_overview_dwell_runtime
    test_perception_runtime
    test_attachment_adapter_runtime
    APPEND PROPERTY LABELS "renderer")
  # One rendered world at a time, enforced by ctest scheduling rather than inside the test:
  # ctest admits a queued test only when the lock is free, so its TIMEOUT starts after
  # admission. An in-test machine-wide wait was measured consuming the timeout instead
  # (test_world_state_runtime and test_planning_scene_projection_runtime each died at
  # ***Timeout 120.11 while queued behind longer worlds). Same lock name as restocker_gazebo;
  # the test recipe runs packages serially so the two packages' ctest pools cannot overlap
  # either. Non-renderer tests carry no lock and keep full ctest -j parallelism.
  set_property(TEST
    test_restock_manipulation_runtime
    test_restock_lane_routing_runtime
    test_autonomous_restock_demo_runtime
    test_sensor_driven_acceptance_runtime
    test_dense_sensor_driven_runtime
    test_restock_obstructed_lane_runtime
    test_dynamic_obstacle_runtime
    test_obstacle_perception_runtime
    test_baseline_runtime
    test_world_state_runtime
    test_planning_scene_projection_runtime
    test_camera_runtime
    test_wrist_viewpoint_runtime
    test_lane_survey_runtime
    test_tray_survey_runtime
    test_tray_overview_dwell_runtime
    test_perception_runtime
    test_attachment_adapter_runtime
    APPEND PROPERTY RESOURCE_LOCK "restocker-renderer-simulator")

  ament_lint_auto_find_test_dependencies()
endif()

ament_package()

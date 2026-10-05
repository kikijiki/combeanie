# Copyright 2026 Matteo Bernacchia <dev@kikijiki.com>
# SPDX-License-Identifier: MIT

cmake_minimum_required(VERSION 3.16)
project(restocker_interfaces VERSION 0.1.0 LANGUAGES C CXX)

find_package(ament_cmake REQUIRED)
find_package(action_msgs REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)

rosidl_generate_interfaces(
  ${PROJECT_NAME}
  "action/RestockProduct.action"
  "action/SurveyLane.action"
  "action/SurveyTray.action"
  "action/SurveyViewpoint.action"
  "msg/AutonomousRestockCampaignStatus.msg"
  "msg/LaneDepthErrorSample.msg"
  "msg/LaneObservation.msg"
  "msg/ObjectDetection.msg"
  "msg/ObjectObservation.msg"
  "msg/ObjectSegmentation.msg"
  "msg/ObstacleBox.msg"
  "msg/ObstacleObservation.msg"
  "msg/PerceptionFrame.msg"
  "msg/PlanningSceneLease.msg"
  "msg/PlanningSceneLeaseOperationStatus.msg"
  "msg/PlanningSceneProjectionStatus.msg"
  "msg/PoseErrorSample.msg"
  "msg/RobotExecutionState.msg"
  "msg/RobotTelemetry.msg"
  "msg/RestockCoordinatorStatus.msg"
  "msg/ShelfLane.msg"
  "msg/SimulationAttachmentOperationStatus.msg"
  "msg/SimulationAttachmentState.msg"
  "msg/TaskReservation.msg"
  "msg/TrackedObject.msg"
  "msg/TrayStationAcquisition.msg"
  "msg/WorldStateEvent.msg"
  "msg/WorldStateOperationStatus.msg"
  "msg/WorldStateSnapshot.msg"
  "srv/AcquireLaneObservation.srv"
  "srv/AcquirePlanningSceneLease.srv"
  "srv/CheckpointTaskState.srv"
  "srv/CommitReservedAttachment.srv"
  "srv/CommitReservedDetachment.srv"
  "srv/GetPlanningSceneProjectionStatus.srv"
  "srv/GetSimulationAttachmentState.srv"
  "srv/GetWorldState.srv"
  "srv/InvalidateLaneEvidence.srv"
  "srv/ReleasePlanningSceneLease.srv"
  "srv/ReleaseTaskReservation.srv"
  "srv/ReserveTask.srv"
  "srv/SetLanePolicy.srv"
  "srv/SetSimulationAttachment.srv"
  "srv/ValidateExecutionWorldAuthority.srv"
  "srv/ValidatePlanningSceneLease.srv"
  "srv/ValidateTaskReservation.srv"
  DEPENDENCIES action_msgs builtin_interfaces geometry_msgs std_msgs
)

ament_export_dependencies(rosidl_default_runtime)

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  find_package(ament_lint_auto REQUIRED)

  ament_add_pytest_test(test_object_observation test/test_object_observation.py TIMEOUT 30)
  ament_add_pytest_test(
    test_perception_frame test/test_perception_frame.py TIMEOUT 30)
  ament_add_pytest_test(
    test_planning_scene_lease test/test_planning_scene_lease.py TIMEOUT 30)
  ament_add_pytest_test(
    test_simulation_attachment test/test_simulation_attachment.py TIMEOUT 30)
  ament_add_pytest_test(test_task_reservation test/test_task_reservation.py TIMEOUT 30)
  ament_add_pytest_test(test_set_lane_policy test/test_set_lane_policy.py TIMEOUT 30)
  ament_add_pytest_test(test_robot_telemetry test/test_robot_telemetry.py TIMEOUT 30)
  ament_add_pytest_test(
    test_restock_coordinator_status test/test_restock_coordinator_status.py TIMEOUT 30)
  ament_add_pytest_test(
    test_restock_product_action test/test_restock_product_action.py TIMEOUT 30)
  ament_add_pytest_test(test_survey_tray_action test/test_survey_tray_action.py TIMEOUT 30)
  ament_add_pytest_test(
    test_autonomous_campaign_status test/test_autonomous_campaign_status.py TIMEOUT 30)

  ament_lint_auto_find_test_dependencies()
endif()

ament_package()

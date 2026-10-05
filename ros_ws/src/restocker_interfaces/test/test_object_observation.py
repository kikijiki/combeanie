# Copyright 2026 Matteo Bernacchia <dev@kikijiki.com>
#
# SPDX-License-Identifier: MIT

"""Contract tests for the normalized perception observation."""

from restocker_interfaces.msg import (
    LaneObservation,
    ObjectObservation,
    PlanningSceneProjectionStatus,
    RobotExecutionState,
    ShelfLane,
    TrackedObject,
    WorldStateEvent,
)
from restocker_interfaces.srv import GetWorldState


def test_semantic_enums_match_the_world_state_domain() -> None:
    assert {
        ObjectObservation.PRODUCT_CLASS_UNKNOWN,
        ObjectObservation.PRODUCT_CLASS_CAN,
        ObjectObservation.PRODUCT_CLASS_SMALL_BOTTLE,
        ObjectObservation.PRODUCT_CLASS_LARGE_BOTTLE,
    } == {0, 1, 2, 3}
    assert {
        ObjectObservation.ORIENTATION_UNKNOWN,
        ObjectObservation.ORIENTATION_UPRIGHT,
        ObjectObservation.ORIENTATION_HORIZONTAL,
        ObjectObservation.ORIENTATION_TILTED,
    } == {0, 1, 2, 3}


def test_status_values_are_unique_and_ok_is_zero() -> None:
    values = {
        ObjectObservation.STATUS_OK,
        ObjectObservation.STATUS_INVALID_POSE,
        ObjectObservation.STATUS_DUPLICATE_SOURCE,
        ObjectObservation.STATUS_MISSING_TIMESTAMP,
        ObjectObservation.STATUS_INTERNAL_ERROR,
    }
    assert len(values) == 5
    assert ObjectObservation.STATUS_OK == 0


def test_default_observation_is_not_implicitly_valid() -> None:
    observation = ObjectObservation()
    assert observation.header.frame_id == ""
    assert observation.source_object_id == ""
    assert observation.product_class == ObjectObservation.PRODUCT_CLASS_UNKNOWN
    assert observation.has_sku is False
    assert observation.backend_name == ""
    assert len(observation.pose.covariance) == 36


def test_lane_status_values_are_unique_and_ok_is_zero() -> None:
    values = {
        LaneObservation.STATUS_OK,
        LaneObservation.STATUS_INVALID_GEOMETRY,
        LaneObservation.STATUS_DUPLICATE_SOURCE,
        LaneObservation.STATUS_MISSING_TIMESTAMP,
        LaneObservation.STATUS_INTERNAL_ERROR,
    }
    assert len(values) == 5
    assert LaneObservation.STATUS_OK == 0


def test_default_lane_observation_is_not_implicitly_valid() -> None:
    observation = LaneObservation()
    assert observation.header.frame_id == ""
    assert observation.lane_id == ""
    assert observation.observed_source_object_ids == []
    assert observation.backend_name == ""
    assert observation.backend_version == ""

    lane = ShelfLane()
    assert list(lane.contents) == []
    assert lane.observed_source_object_ids == []
    assert lane.evidence_revision == 0


def test_snapshot_domain_constants_match_the_cpp_authority() -> None:
    assert TrackedObject.TRACKING_TRACKED == 0
    assert TrackedObject.TRACKING_REMOVED == 3
    assert TrackedObject.GRASP_FREE == 0
    assert TrackedObject.GRASP_ATTACHED == 1
    assert RobotExecutionState.TASK_IDLE == 0
    assert RobotExecutionState.TASK_REQUESTING_OPERATOR == 5
    assert RobotExecutionState.FAULT_NONE == 0
    assert RobotExecutionState.FAULT_EXTERNAL_INCONSISTENCY == 3
    robot_fields = RobotExecutionState().get_fields_and_field_types()
    assert robot_fields["joint_positions"] == "double[6]"
    assert robot_fields["joint_velocities"] == "double[6]"
    assert robot_fields["gripper_joint_positions"] == "double[2]"
    assert robot_fields["gripper_joint_velocities"] == "double[2]"
    assert robot_fields["rail_velocity"] == "double"
    assert "telemetry_time" in robot_fields
    assert robot_fields["telemetry_source_id"] == "string"
    assert "telemetry_revision" in robot_fields
    assert WorldStateEvent.OBJECT_OBSERVED == 0
    assert WorldStateEvent.OBJECT_DETACHED == 7
    assert WorldStateEvent.TASK_RESERVED == 8
    assert WorldStateEvent.TASK_RESERVATION_RELEASED == 10


def test_snapshot_request_defaults_to_bounded_response() -> None:
    request = GetWorldState.Request()
    assert request.include_removed is False
    assert request.include_events is False
    response = GetWorldState.Response()
    assert response.snapshot.revision == 0
    assert response.snapshot.objects == []


def test_projection_status_is_a_typed_revision_gate() -> None:
    status = PlanningSceneProjectionStatus()
    assert {
        status.STATE_STARTING,
        status.STATE_SYNCHRONIZING,
        status.STATE_APPLIED,
        status.STATE_DEGRADED,
    } == {0, 1, 2, 3}
    assert status.ERROR_NONE == 0
    assert status.ERROR_VERIFICATION_FAILED == 7
    assert status.ERROR_APPLY_OUTCOME_UNKNOWN == 8
    assert status.state == status.STATE_STARTING
    assert status.desired_revision == 0
    assert status.applied_revision == 0

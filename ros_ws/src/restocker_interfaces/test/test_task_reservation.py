# Copyright 2026 Matteo Bernacchia <dev@kikijiki.com>
#
# SPDX-License-Identifier: MIT

"""Static contracts for the world-state reservation protocol."""

from restocker_interfaces.msg import (
    TaskReservation,
    WorldStateOperationStatus,
    WorldStateSnapshot,
)
from restocker_interfaces.srv import (
    CheckpointTaskState,
    CommitReservedAttachment,
    CommitReservedDetachment,
    ReleaseTaskReservation,
    ReserveTask,
    ValidateExecutionWorldAuthority,
    ValidateTaskReservation,
)


def test_operation_status_codes_are_stable_and_unique() -> None:
    values = {
        WorldStateOperationStatus.UNSET,
        WorldStateOperationStatus.OK,
        WorldStateOperationStatus.INVALID_ARGUMENT,
        WorldStateOperationStatus.NOT_FOUND,
        WorldStateOperationStatus.REVISION_CONFLICT,
        WorldStateOperationStatus.RESERVATION_CONFLICT,
        WorldStateOperationStatus.TOKEN_MISMATCH,
        WorldStateOperationStatus.PREDICATE_FAILED,
        WorldStateOperationStatus.INVALID_TRANSITION,
        WorldStateOperationStatus.IDEMPOTENCY_CONFLICT,
        WorldStateOperationStatus.RESOURCE_EXHAUSTED,
        WorldStateOperationStatus.INTERNAL_ERROR,
    }
    assert len(values) == 12
    assert WorldStateOperationStatus.UNSET == 0
    assert WorldStateOperationStatus.OK == 1
    assert WorldStateOperationStatus.INTERNAL_ERROR == 255


def test_reservation_summary_has_no_capability_token() -> None:
    reservation = TaskReservation()
    assert reservation.STAGE_RESERVED == 0
    assert reservation.STAGE_ATTACHED == 1
    assert reservation.STAGE_DETACHED == 2
    assert {
        reservation.PRODUCT_CLASS_UNKNOWN,
        reservation.PRODUCT_CLASS_CAN,
        reservation.PRODUCT_CLASS_SMALL_BOTTLE,
        reservation.PRODUCT_CLASS_LARGE_BOTTLE,
    } == {0, 1, 2, 3}
    assert reservation.stage == reservation.STAGE_RESERVED
    assert reservation.placed_in_destination is False
    assert "token" not in reservation.get_fields_and_field_types()
    # The destination policy captured at grant rides along so revalidation can compare against
    # it after the lane's live intent has changed.
    fields = reservation.get_fields_and_field_types()
    assert fields["destination_expected_product_class"] == "uint8"
    assert fields["has_destination_expected_sku"] == "boolean"
    assert fields["destination_expected_sku"] == "string"
    assert reservation.destination_expected_product_class == (reservation.PRODUCT_CLASS_UNKNOWN)
    assert reservation.has_destination_expected_sku is False

    snapshot = WorldStateSnapshot()
    assert snapshot.has_active_reservation is False
    assert snapshot.active_reservation.reservation_id == 0


def test_reserve_request_requires_exact_snapshot_context() -> None:
    request = ReserveTask.Request()
    fields = request.get_fields_and_field_types()
    assert set(fields) == {
        "request_id",
        "selected_snapshot_revision",
        "object_id",
        "object_revision",
        "has_source_lane",
        "source_lane_id",
        "source_lane_revision",
        "destination_lane_id",
        "destination_lane_revision",
    }
    assert request.request_id == ""
    assert request.has_source_lane is False

    response = ReserveTask.Response()
    assert response.status.code == WorldStateOperationStatus.UNSET
    assert response.token == ""
    assert response.world_revision == 0


def test_mutations_carry_token_and_idempotency_identity() -> None:
    for request_type in (
        CheckpointTaskState.Request,
        CommitReservedAttachment.Request,
        CommitReservedDetachment.Request,
        ReleaseTaskReservation.Request,
    ):
        fields = request_type().get_fields_and_field_types()
        assert "token" in fields
        assert "operation_id" in fields

    validate_fields = ValidateTaskReservation.Request().get_fields_and_field_types()
    assert set(validate_fields) == {"token"}
    release_fields = ReleaseTaskReservation.Request().get_fields_and_field_types()
    assert {
        "expected_reservation_id",
        "expected_reservation_stage",
        "expected_reservation_revision",
    } <= set(release_fields)
    checkpoint_fields = CheckpointTaskState.Request().get_fields_and_field_types()
    assert {
        "expected_reservation_id",
        "expected_reservation_stage",
        "expected_reservation_revision",
    } <= set(checkpoint_fields)
    assert "joint_positions" not in checkpoint_fields
    assert "rail_position" not in checkpoint_fields


def test_execution_authority_service_has_exact_identity_and_no_returned_token() -> None:
    request_fields = ValidateExecutionWorldAuthority.Request().get_fields_and_field_types()
    assert set(request_fields) == {
        "token",
        "expected_reservation_id",
        "expected_reservation_revision",
        "expected_object_id",
        "expected_destination_lane_id",
    }
    response = ValidateExecutionWorldAuthority.Response()
    response_fields = response.get_fields_and_field_types()
    assert set(response_fields) == {
        "status",
        "world_revision",
        "planning_frame",
        "has_proof",
        "reservation",
        "object",
        "destination_lane",
        "has_source_lane",
        "source_lane",
        "robot",
    }
    assert "token" not in response_fields
    assert response.has_proof is False
    assert response.has_source_lane is False


def test_terminal_dispositions_are_closed_enums() -> None:
    detach = CommitReservedDetachment.Request()
    assert detach.PLACE_IN_RESERVED_DESTINATION == 0
    assert detach.RELEASE_WITHOUT_MEMBERSHIP == 1
    assert detach.disposition == detach.PLACE_IN_RESERVED_DESTINATION

    release = ReleaseTaskReservation.Request()
    assert {
        release.OUTCOME_SUCCEEDED,
        release.OUTCOME_CANCELED,
        release.OUTCOME_FAILED_SAFE,
    } == {0, 1, 2}
    assert {
        release.EXPECTED_STAGE_RESERVED,
        release.EXPECTED_STAGE_ATTACHED,
        release.EXPECTED_STAGE_DETACHED,
    } == {0, 1, 2}
    assert release.outcome == release.OUTCOME_SUCCEEDED
    assert release.expected_reservation_stage == release.EXPECTED_STAGE_RESERVED

# Copyright 2026 Matteo Bernacchia <dev@kikijiki.com>
#
# SPDX-License-Identifier: MIT

"""Static contracts for the planning-scene transaction lease protocol."""

from restocker_interfaces.msg import (
    PlanningSceneLease,
    PlanningSceneLeaseOperationStatus,
    PlanningSceneProjectionStatus,
)
from restocker_interfaces.srv import (
    AcquirePlanningSceneLease,
    ReleasePlanningSceneLease,
    ValidatePlanningSceneLease,
)


def test_operation_status_codes_are_stable_and_unique() -> None:
    values = {
        PlanningSceneLeaseOperationStatus.UNSET,
        PlanningSceneLeaseOperationStatus.DRAINING,
        PlanningSceneLeaseOperationStatus.GRANTED,
        PlanningSceneLeaseOperationStatus.VALID,
        PlanningSceneLeaseOperationStatus.RELEASE_ACCEPTED,
        PlanningSceneLeaseOperationStatus.INVALID_ARGUMENT,
        PlanningSceneLeaseOperationStatus.CONFLICT,
        PlanningSceneLeaseOperationStatus.TOKEN_MISMATCH,
        PlanningSceneLeaseOperationStatus.IDEMPOTENCY_CONFLICT,
        PlanningSceneLeaseOperationStatus.RESOURCE_EXHAUSTED,
        PlanningSceneLeaseOperationStatus.INTERNAL_ERROR,
    }
    assert len(values) == 11
    assert PlanningSceneLeaseOperationStatus.UNSET == 0
    assert PlanningSceneLeaseOperationStatus.DRAINING == 1
    assert PlanningSceneLeaseOperationStatus.GRANTED == 2
    assert PlanningSceneLeaseOperationStatus.VALID == 3
    assert PlanningSceneLeaseOperationStatus.RELEASE_ACCEPTED == 4
    assert PlanningSceneLeaseOperationStatus.INTERNAL_ERROR == 255


def test_lease_summary_is_token_free_and_default_inactive() -> None:
    lease = PlanningSceneLease()
    assert {
        lease.PHASE_NONE,
        lease.PHASE_DRAINING,
        lease.PHASE_HELD,
        lease.PHASE_RELEASING,
    } == {0, 1, 2, 3}
    assert lease.phase == lease.PHASE_NONE
    assert lease.lease_id == 0
    assert "token" not in lease.get_fields_and_field_types()


def test_acquire_is_pollable_and_only_success_can_return_a_token() -> None:
    request = AcquirePlanningSceneLease.Request()
    assert set(request.get_fields_and_field_types()) == {
        "operation_id",
        "minimum_applied_revision",
    }
    assert request.operation_id == ""
    assert request.minimum_applied_revision == 0

    response = AcquirePlanningSceneLease.Response()
    assert response.status.code == PlanningSceneLeaseOperationStatus.UNSET
    assert response.has_lease is False
    assert response.token == ""


def test_validate_and_release_carry_the_capability_without_echoing_it() -> None:
    validate_request = ValidatePlanningSceneLease.Request()
    assert set(validate_request.get_fields_and_field_types()) == {"token"}
    validate_response = ValidatePlanningSceneLease.Response()
    assert "token" not in validate_response.get_fields_and_field_types()
    assert validate_response.status.code == PlanningSceneLeaseOperationStatus.UNSET

    release_request = ReleasePlanningSceneLease.Request()
    assert set(release_request.get_fields_and_field_types()) == {
        "operation_id",
        "token",
        "required_semantic_revision",
    }
    release_response = ReleasePlanningSceneLease.Response()
    assert "token" not in release_response.get_fields_and_field_types()
    assert release_response.status.code == PlanningSceneLeaseOperationStatus.UNSET


def test_projection_transaction_states_preserve_existing_wire_values() -> None:
    status = PlanningSceneProjectionStatus()
    assert status.STATE_STARTING == 0
    assert status.STATE_SYNCHRONIZING == 1
    assert status.STATE_APPLIED == 2
    assert status.STATE_DEGRADED == 3
    assert status.STATE_TRANSACTION_DRAINING == 4
    assert status.STATE_TRANSACTION_HELD == 5
    assert status.STATE_TRANSACTION_RELEASING == 6
    assert status.lease_phase == PlanningSceneLease.PHASE_NONE
    assert status.lease_id == 0
    assert status.verification_epoch == 0
    assert status.projector_epoch == ""
    assert status.scene_content_generation == 0

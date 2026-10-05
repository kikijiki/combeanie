# Copyright 2026 Matteo Bernacchia <dev@kikijiki.com>
#
# SPDX-License-Identifier: MIT

"""Static contracts for the simulated physical-attachment boundary."""

from restocker_interfaces.msg import (
    SimulationAttachmentOperationStatus,
    SimulationAttachmentState,
)
from restocker_interfaces.srv import (
    GetSimulationAttachmentState,
    SetSimulationAttachment,
)


def test_operation_status_codes_are_stable_unique_and_default_invalid() -> None:
    status = SimulationAttachmentOperationStatus()
    named_codes = {
        status.UNSET,
        status.PENDING,
        status.ATTACHED,
        status.DETACHED,
        status.INVALID_ARGUMENT,
        status.AUTHORIZATION_FAILED,
        status.EXTERNAL_INCONSISTENCY,
        status.CONFLICT,
        status.TOKEN_MISMATCH,
        status.IDEMPOTENCY_CONFLICT,
        status.RESOURCE_EXHAUSTED,
        status.OBJECT_NOT_FOUND,
        status.ENTITY_AMBIGUOUS,
        status.GRIPPER_NOT_READY,
        status.OUT_OF_TOLERANCE,
        status.STATE_MISMATCH,
        status.SIMULATOR_EPOCH_CHANGED,
        status.OUTCOME_UNKNOWN,
        status.OPERATION_NOT_FOUND,
        status.INTERNAL_ERROR,
    }
    assert len(named_codes) == 20
    assert status.UNSET == 0
    assert status.PENDING == 1
    assert status.ATTACHED == 2
    assert status.DETACHED == 3
    assert status.OPERATION_NOT_FOUND == 18
    assert status.INTERNAL_ERROR == 255
    assert status.code == status.UNSET


def test_physical_state_is_token_free_and_default_unknown() -> None:
    state = SimulationAttachmentState()
    assert {
        state.PHASE_UNKNOWN,
        state.PHASE_DETACHED,
        state.PHASE_VALIDATING_ATTACH,
        state.PHASE_APPLYING_ATTACH,
        state.PHASE_VERIFYING_ATTACH,
        state.PHASE_ATTACHED,
        state.PHASE_VALIDATING_DETACH,
        state.PHASE_APPLYING_DETACH,
        state.PHASE_VERIFYING_DETACH,
        state.PHASE_INCONSISTENT,
    } == set(range(10))
    assert state.phase == state.PHASE_UNKNOWN
    assert state.joint_observed is False
    assert {
        state.MOTION_GATE_UNKNOWN,
        state.MOTION_GATE_INHIBITED,
        state.MOTION_GATE_VALID,
    } == {0, 1, 2}
    assert state.motion_gate == state.MOTION_GATE_UNKNOWN
    assert state.status.code == SimulationAttachmentOperationStatus.UNSET
    assert "token" not in " ".join(state.get_fields_and_field_types())


def test_mutation_is_pollable_and_capabilities_are_request_only() -> None:
    request = SetSimulationAttachment.Request()
    assert request.COMMAND_UNSET == 0
    assert request.COMMAND_ATTACH == 1
    assert request.COMMAND_DETACH == 2
    assert request.command == request.COMMAND_UNSET
    assert set(request.get_fields_and_field_types()) == {
        "command",
        "operation_id",
        "object_id",
        "reservation_token",
        "planning_scene_lease_token",
        "expected_grasp_center_to_child",
        "expected_pose_covariance",
    }
    # The boundary sizes its admissible residual from the uncertainty stated here, so the
    # covariance travels with the grasp.
    assert len(request.expected_pose_covariance) == 36
    assert not any(request.expected_pose_covariance)

    response = SetSimulationAttachment.Response()
    response_fields = response.get_fields_and_field_types()
    assert set(response_fields) == {"status", "has_state", "state"}
    assert "token" not in " ".join(response_fields)
    assert response.status.code == SimulationAttachmentOperationStatus.UNSET
    assert response.has_state is False


def test_query_is_read_only_epoch_aware_and_token_free() -> None:
    request = GetSimulationAttachmentState.Request()
    assert set(request.get_fields_and_field_types()) == {
        "expected_simulator_epoch",
        "operation_id",
    }
    assert request.expected_simulator_epoch == ""
    assert request.operation_id == ""
    assert "token" not in " ".join(request.get_fields_and_field_types())

    response = GetSimulationAttachmentState.Response()
    assert set(response.get_fields_and_field_types()) == {
        "status",
        "simulator_epoch",
        "operation_found",
        "has_state",
        "state",
    }
    assert response.operation_found is False
    assert response.has_state is False
    assert response.status.code == SimulationAttachmentOperationStatus.UNSET
    assert "token" not in " ".join(response.get_fields_and_field_types())


def test_state_transform_and_motion_fields_are_explicit() -> None:
    fields = SimulationAttachmentState.get_fields_and_field_types()
    assert fields["parent_to_child"] == "geometry_msgs/Pose"
    assert fields["child_pose_in_world"] == "geometry_msgs/Pose"
    assert fields["relative_twist_in_parent"] == "geometry_msgs/Twist"
    assert fields["observed_at"] == "builtin_interfaces/Time"
    assert fields["simulator_iteration"] == "uint64"
    # Reported on every attach that reached the fidelity comparison, refusals included, so the
    # residual distribution is observable.
    assert fields["has_fidelity"] == "boolean"
    assert fields["fidelity_translation_residual_m"] == "double"
    assert fields["fidelity_rotation_residual_rad"] == "double"
    assert fields["fidelity_translation_budget_m"] == "double"
    assert fields["fidelity_claimed_translation_sigma_m"] == "double"
    assert fields["fidelity_translation_ceiling_m"] == "double"

# Copyright 2026 Matteo Bernacchia <dev@kikijiki.com>
#
# SPDX-License-Identifier: MIT

"""Static contracts for the public restock action."""

from restocker_interfaces.action import RestockProduct


def test_goal_uses_explicit_optional_selectors() -> None:
    goal = RestockProduct.Goal()
    assert not goal.has_object_id
    assert goal.object_id == 0
    assert not goal.has_lane_id
    assert goal.lane_id == ""


def test_result_exposes_stable_terminal_status_and_metrics() -> None:
    result = RestockProduct.Result()
    assert result.STATUS_SUCCEEDED == 1
    assert result.STATUS_EXTERNAL_INCONSISTENCY == 9
    assert result.STATUS_SHUTDOWN == 11
    assert result.STATUS_OBSERVATION_EVIDENCE_STALE == 12
    assert result.STATUS_INTERNAL_ERROR == 255
    assert result.status == result.STATUS_UNSET
    assert result.initial_world_revision == 0
    assert result.final_world_revision == 0
    assert result.attempt_count == 0
    assert result.retry_count == 0
    assert result.minimum_clearance_m == 0.0


def test_feedback_state_values_match_the_cpp_machine_contract() -> None:
    feedback = RestockProduct.Feedback()
    expected_states = list(range(feedback.STATE_IDLE, feedback.STATE_CANCELED + 1))
    observed_states = [
        feedback.STATE_IDLE,
        feedback.STATE_VALIDATE_SCENE,
        feedback.STATE_SELECT_PAIR,
        feedback.STATE_RESERVE_TASK,
        feedback.STATE_GENERATE_GRASPS,
        feedback.STATE_PLAN_PRE_GRASP,
        feedback.STATE_EXECUTE_PRE_GRASP,
        feedback.STATE_PLAN_APPROACH,
        feedback.STATE_EXECUTE_APPROACH,
        feedback.STATE_CLOSE_GRIPPER,
        feedback.STATE_VERIFY_GRASP,
        feedback.STATE_ATTACH_TRANSACTION,
        feedback.STATE_PLAN_RETRACT,
        feedback.STATE_EXECUTE_RETRACT,
        feedback.STATE_OBSERVE_DESTINATION,
        feedback.STATE_GENERATE_PLACEMENT,
        feedback.STATE_PLAN_PRE_INSERT,
        feedback.STATE_EXECUTE_PRE_INSERT,
        feedback.STATE_PLAN_INSERT,
        feedback.STATE_EXECUTE_INSERT,
        feedback.STATE_OPEN_GRIPPER,
        feedback.STATE_DETACH_TRANSACTION,
        feedback.STATE_VERIFY_PLACEMENT,
        feedback.STATE_PLAN_RETREAT,
        feedback.STATE_EXECUTE_RETREAT,
        feedback.STATE_UPDATE_INVENTORY,
        feedback.STATE_CANCEL_ACTIVE_MOTION,
        feedback.STATE_RELEASE_TASK,
        feedback.STATE_RECOVER,
        feedback.STATE_FAULT,
        feedback.STATE_REQUEST_OPERATOR,
        feedback.STATE_COMPLETE,
        feedback.STATE_CANCELED,
    ]
    assert observed_states == expected_states
    assert feedback.state == feedback.STATE_IDLE
    assert not feedback.has_selected_object_id
    assert not feedback.has_selected_lane_id


def test_result_motion_evidence_defaults_to_unknown() -> None:
    """Card 086 stage 1b: a default result proves neither non-start nor a stop (fail-closed)."""
    result = RestockProduct.Result()
    assert result.motion_definitely_not_started is False
    assert result.execution_reached_terminal_stop is False

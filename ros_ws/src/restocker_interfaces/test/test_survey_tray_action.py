# Copyright 2026 Matteo Bernacchia <dev@kikijiki.com>
#
# SPDX-License-Identifier: MIT

"""Static contracts for the coordinated tray survey action."""

from restocker_interfaces.action import SurveyTray
from restocker_interfaces.msg import ObjectObservation


def test_goal_defaults_request_the_derived_overview_with_any_selection() -> None:
    goal = SurveyTray.Goal()
    assert goal.overview_stations == []
    assert goal.product_class == SurveyTray.Goal.PRODUCT_CLASS_ANY
    assert goal.product_class == 0
    assert not goal.has_sku
    assert goal.sku == ""
    assert not goal.overview_only
    assert goal.refuted_positions == []
    assert goal.skip_mark_positions == []
    assert goal.absence_probe_positions == []
    assert goal.absence_occluder_positions == []
    assert goal.position_tolerance_m == 0.0
    assert goal.orientation_tolerance_rad == 0.0


def test_result_exposes_one_stable_terminal_outcome_set() -> None:
    result = SurveyTray.Result()
    expected = {
        "OUTCOME_UNSET": 0,
        "OUTCOME_CONFIRMED": 1,
        "OUTCOME_NO_CANDIDATE": 2,
        "OUTCOME_REFUTED": 3,
        "OUTCOME_OVERVIEW_ONLY": 4,
        "OUTCOME_VIEWPOINT_UNREACHED": 5,
        "OUTCOME_ACQUISITION_FAILED": 6,
        "OUTCOME_CANCELED": 7,
        "OUTCOME_UNAVAILABLE": 8,
        "OUTCOME_INVALID_REQUEST": 9,
    }
    for name, value in expected.items():
        assert getattr(result, name) == value
    assert result.outcome == result.OUTCOME_UNSET
    assert result.detail == ""
    assert result.overview_stations_visited == []
    assert result.overview_candidates == []
    assert result.selected_candidate.header.frame_id == ""
    assert result.confirmed_observation.header.frame_id == ""
    assert result.refuting_observation.header.frame_id == ""
    assert result.refutation == result.REFUTATION_NONE
    assert result.refutation == 0


def test_result_refutation_codes_are_typed_and_disjoint() -> None:
    result = SurveyTray.Result()
    codes = [
        result.REFUTATION_NONE,
        result.REFUTATION_CLASS_MISMATCH,
        result.REFUTATION_SKU_MISMATCH,
        result.REFUTATION_ABSENT,
    ]
    assert codes == [0, 1, 2, 3]
    assert len(set(codes)) == len(codes)


def test_result_absence_probe_verdicts_are_typed_and_default_empty() -> None:
    result = SurveyTray.Result()
    assert list(result.absence_probe_verdicts) == []
    codes = [
        result.PROBE_NOT_EVALUATED,
        result.PROBE_SEEN,
        result.PROBE_VIEWED_EMPTY,
        result.PROBE_OCCLUDED,
        result.PROBE_NOT_COVERED,
        result.PROBE_NO_EVIDENCE,
    ]
    assert codes == [0, 1, 2, 3, 4, 5]


def test_feedback_phases_are_ordered_overview_selecting_confirming() -> None:
    feedback = SurveyTray.Feedback()
    assert feedback.PHASE_OVERVIEW == 0
    assert feedback.PHASE_SELECTING == 1
    assert feedback.PHASE_CONFIRMING == 2
    assert feedback.phase == feedback.PHASE_OVERVIEW
    assert feedback.current_station == ""
    assert feedback.candidates_collected == 0


def test_candidates_and_confirmations_carry_object_observations() -> None:
    # The action never invents a candidate message type: a candidate and the confirming
    # observation are both ordinary ObjectObservations, so a consumer that can read the
    # admitted topic can read this result without a second schema.
    result = SurveyTray.Result()
    observation = ObjectObservation()
    observation.product_class = ObjectObservation.PRODUCT_CLASS_CAN
    result.overview_candidates.append(observation)
    assert result.overview_candidates[0].product_class == ObjectObservation.PRODUCT_CLASS_CAN

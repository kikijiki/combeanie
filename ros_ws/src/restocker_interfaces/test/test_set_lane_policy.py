# Copyright 2026 Matteo Bernacchia <dev@kikijiki.com>
#
# SPDX-License-Identifier: MIT

"""Static contracts for runtime lane policy: ShelfLane.target_count and SetLanePolicy."""

from restocker_interfaces.msg import (
    ShelfLane,
    WorldStateEvent,
    WorldStateOperationStatus,
)
from restocker_interfaces.srv import InvalidateLaneEvidence, SetLanePolicy


def test_shelf_lane_carries_desired_target_count() -> None:
    lane = ShelfLane()
    fields = lane.get_fields_and_field_types()
    assert fields["target_count"] == "uint32"
    assert lane.target_count == 0
    assert {"id", "expected_product_class", "has_expected_sku", "expected_sku"} <= set(fields)


def test_shelf_lane_carries_ledger_identity_trust() -> None:
    lane = ShelfLane()
    fields = lane.get_fields_and_field_types()
    assert fields["ledger_unreliable"] == "boolean"
    assert lane.ledger_unreliable is False
    assert fields["contents"] == "sequence<uint64>"


def test_set_lane_policy_request_is_revision_fenced_with_full_policy() -> None:
    request = SetLanePolicy.Request()
    assert set(request.get_fields_and_field_types()) == {
        "lane_id",
        "expected_lane_revision",
        "expected_product_class",
        "has_expected_sku",
        "expected_sku",
        "target_count",
    }
    assert request.lane_id == ""
    assert request.expected_lane_revision == 0
    assert request.expected_product_class == ShelfLane.PRODUCT_CLASS_UNKNOWN
    assert request.has_expected_sku is False
    assert request.target_count == 0
    assert {
        ShelfLane.PRODUCT_CLASS_CAN,
        ShelfLane.PRODUCT_CLASS_SMALL_BOTTLE,
        ShelfLane.PRODUCT_CLASS_LARGE_BOTTLE,
    } == {1, 2, 3}


def test_set_lane_policy_response_matches_the_other_lane_mutation() -> None:
    response = SetLanePolicy.Response()
    assert set(response.get_fields_and_field_types()) == {"status", "world_revision"}
    assert response.status.code == WorldStateOperationStatus.UNSET
    assert response.world_revision == 0
    assert set(InvalidateLaneEvidence.Response().get_fields_and_field_types()) == {
        "status",
        "world_revision",
    }


def test_lane_policy_set_event_kind_is_appended_not_renumbered() -> None:
    assert WorldStateEvent.TASK_RESERVATION_RELEASED == 10
    assert WorldStateEvent.LANE_POLICY_SET == 11
    assert WorldStateEvent.LANE_LEDGER_UNRELIABLE == 12

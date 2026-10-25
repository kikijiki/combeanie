# Copyright 2026 Matteo Bernacchia <dev@kikijiki.com>
# SPDX-License-Identifier: MIT

"""Static contracts for the inventory messages and the QueryInventory service (CMB-115)."""

from restocker_interfaces.msg import Inventory, InventoryLane, InventorySku
from restocker_interfaces.srv import QueryInventory


def test_inventory_message_carries_lanes_skus_and_the_tray():
    fields = Inventory().get_fields_and_field_types()
    assert fields["lanes"] == "sequence<restocker_interfaces/InventoryLane>"
    assert fields["skus"] == "sequence<restocker_interfaces/InventorySku>"
    assert {"stale_after_s", "tray_source", "tray_stale", "world_available"} <= set(fields)
    assert {"observation", "age_s", "sku_basis", "count_discrepancy"} <= set(
        InventoryLane().get_fields_and_field_types()
    )
    assert {"complete", "unread_lanes", "stale_lanes", "total_estimate"} <= set(
        InventorySku().get_fields_and_field_types()
    )


def test_query_request_and_reply_shape():
    assert set(QueryInventory.Request().get_fields_and_field_types()) == {
        "sku",
        "lane_id",
        "min_confidence",
        "max_age_s",
    }
    reply = set(QueryInventory.Response().get_fields_and_field_types())
    assert {
        "ok",
        "message",
        "count_known",
        "count_min",
        "count_max",
        "complete",
        "inventory",
    } <= reply

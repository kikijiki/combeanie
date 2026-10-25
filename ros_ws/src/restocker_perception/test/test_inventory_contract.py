# Copyright 2026 Matteo Bernacchia <dev@kikijiki.com>
# SPDX-License-Identifier: MIT

"""The inventory core repeats message constants and field names; this pins them to the messages."""

import pytest

pytest.importorskip("rclpy")

from restocker_interfaces.msg import (  # noqa: E402
    Inventory,
    InventoryLane,
    InventorySku,
    LaneStock,
    ObjectObservation,
)
from restocker_interfaces.srv import QueryInventory  # noqa: E402
from restocker_perception.inventory import model  # noqa: E402


def test_core_constants_equal_the_messages():
    assert model.STATE_ESTIMATED == LaneStock.STATE_ESTIMATED
    assert model.COMPOSITION_MISPLACED == LaneStock.COMPOSITION_MISPLACED
    assert model.COMPOSITION_EMPTY == LaneStock.COMPOSITION_EMPTY
    assert model.IDENTITY_NAMED == LaneStock.IDENTITY_NAMED
    assert model.IDENTITY_UNKNOWN == LaneStock.IDENTITY_UNKNOWN
    assert model.IDENTITY_STATE_IDENTIFIED == ObjectObservation.IDENTITY_IDENTIFIED
    assert model.IDENTITY_STATE_UNKNOWN_PRODUCT == ObjectObservation.IDENTITY_UNKNOWN_PRODUCT
    for name in ("NEVER", "FRESH", "STALE"):
        assert getattr(model, f"OBSERVATION_{name}") == getattr(
            InventoryLane, f"OBSERVATION_{name}"
        )
    for name in ("UNKNOWN", "EXPECTED_ONLY", "PARTLY_CONFIRMED", "CONFIRMED", "CONTESTED"):
        assert getattr(model, f"SKU_BASIS_{name}") == getattr(InventoryLane, f"SKU_BASIS_{name}")
    for name in ("NONE", "OBSERVED_HIGHER", "OBSERVED_LOWER", "NOT_COMPARABLE"):
        assert getattr(model, f"COUNT_DISCREPANCY_{name}") == getattr(
            InventoryLane, f"COUNT_DISCREPANCY_{name}"
        )
    for name in ("NONE", "WORLD_STATE", "PERCEPTION"):
        assert getattr(model, f"TRAY_SOURCE_{name}") == getattr(Inventory, f"TRAY_SOURCE_{name}")
    for name in ("ANY", "EXPECTED", "CONFIRMED"):
        assert getattr(model, f"CONFIDENCE_{name}") == getattr(
            QueryInventory.Request, f"CONFIDENCE_{name}"
        )


def test_inventory_lines_carry_what_the_output_dataclasses_do():
    lane_fields = set(InventoryLane().get_fields_and_field_types())
    sku_fields = set(InventorySku().get_fields_and_field_types())
    line = model.LaneLine("lane_01")
    for name in vars(line):
        if name not in {"observed_at_s"}:
            assert name in lane_fields, name
    sku = model.SkuLine("A")
    for name in vars(sku):
        if name != "excluded_count":
            assert name in sku_fields, name

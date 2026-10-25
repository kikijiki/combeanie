# Copyright 2026 Matteo Bernacchia <dev@kikijiki.com>
#
# SPDX-License-Identifier: MIT

"""InventoryView <-> restocker_interfaces messages (the only inventory module that imports ROS)."""

from __future__ import annotations

from builtin_interfaces.msg import Time
from restocker_interfaces.msg import Inventory, InventoryLane, InventorySku

from restocker_perception.inventory.model import InventoryView, LaneLine, SkuLine


def to_time(seconds: float) -> Time:
    whole = int(seconds)
    return Time(sec=whole, nanosec=int(round((seconds - whole) * 1e9)) % 1_000_000_000)


def lane_message(line: LaneLine) -> InventoryLane:
    message = InventoryLane()
    message.lane_id = line.lane_id
    message.observation = line.observation
    message.observed_at = to_time(line.observed_at_s)
    message.age_s = float(line.age_s)
    message.cannot_tell = line.cannot_tell
    message.cannot_tell_reason = line.cannot_tell_reason
    message.has_expected_sku = line.has_expected_sku
    message.expected_sku = line.expected_sku
    message.sku_basis = line.sku_basis
    message.confirmed_count = line.confirmed_count
    message.confirmed_skus = list(line.confirmed_skus)
    message.unknown_count = line.unknown_count
    message.misplaced = line.misplaced
    message.misplaced_skus = list(line.misplaced_skus)
    message.count_estimate = line.count_estimate
    message.count_min = line.count_min
    message.count_max = line.count_max
    message.empty = line.empty
    message.has_world = line.has_world
    message.world_count = line.world_count
    message.world_ledger_unreliable = line.world_ledger_unreliable
    message.count_discrepancy = line.count_discrepancy
    message.count_delta = line.count_delta
    message.sku_discrepancy = line.sku_discrepancy
    message.discrepancy_detail = line.discrepancy_detail
    return message


def sku_message(line: SkuLine) -> InventorySku:
    message = InventorySku()
    for name in (
        "sku",
        "is_unknown_bucket",
        "confirmed_count",
        "expected_count_min",
        "expected_count_max",
        "reported_count",
        "total_min",
        "total_max",
        "total_estimate",
        "complete",
        "tray_count",
        "tray_source",
        "has_world",
        "world_count",
        "world_difference",
    ):
        setattr(message, name, getattr(line, name))
    message.unread_lanes = list(line.unread_lanes)
    message.stale_lanes = list(line.stale_lanes)
    message.contested_lanes = list(line.contested_lanes)
    return message


def inventory_message(view: InventoryView) -> Inventory:
    message = Inventory()
    message.header.stamp = to_time(view.now_s)
    message.stale_after_s = float(view.stale_after_s)
    message.max_age_s = float(view.max_age_s)
    message.min_confidence = view.min_confidence
    message.lanes = [lane_message(line) for line in view.lanes]
    message.skus = [sku_message(line) for line in view.skus]
    message.tray_source = view.tray_source
    message.tray_count = view.tray_count
    message.tray_observed_at = to_time(view.tray_observed_at_s)
    message.tray_age_s = float(view.tray_age_s)
    message.tray_stale = view.tray_stale
    message.world_available = view.world_available
    message.world_revision = view.world_revision
    message.lanes_total = view.lanes_total
    message.lanes_fresh = view.lanes_fresh
    message.lanes_stale = view.lanes_stale
    message.lanes_unread = view.lanes_unread
    message.discrepancy_count = view.discrepancy_count
    return message

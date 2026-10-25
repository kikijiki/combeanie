# Copyright 2026 Matteo Bernacchia <dev@kikijiki.com>
#
# SPDX-License-Identifier: MIT

"""Answer "how many of SKU X are in stock" or "what is in lane Y", uncertainty spelled out."""

from __future__ import annotations

from collections.abc import Mapping
from dataclasses import dataclass

from restocker_perception.inventory.aggregate import build_inventory
from restocker_perception.inventory.model import (
    CONFIDENCE_ANY,
    CONFIDENCE_CONFIRMED,
    CONFIDENCE_EXPECTED,
    TRAY_SOURCE_NONE,
    TRAY_SOURCE_PERCEPTION,
    TRAY_SOURCE_WORLD_STATE,
    InventoryConfig,
    InventoryView,
    LaneReading,
    TrayReading,
    WorldView,
)

CONFIDENCE_NAMES = {
    CONFIDENCE_ANY: "any",
    CONFIDENCE_EXPECTED: "expected",
    CONFIDENCE_CONFIRMED: "confirmed",
}


@dataclass
class InventoryAnswer:
    ok: bool
    message: str
    view: InventoryView | None = None
    count_known: bool = False
    count_min: int = 0
    count_max: int = 0
    count_estimate: int = 0
    confirmed_count: int = 0
    excluded_count: int = 0
    complete: bool = False
    explanation: str = ""


def query_inventory(
    readings: Mapping[str, LaneReading],
    world: WorldView | None,
    tray: TrayReading | None,
    now_s: float,
    config: InventoryConfig | None = None,
    *,
    sku: str = "",
    lane_id: str = "",
    min_confidence: int = CONFIDENCE_ANY,
    max_age_s: float = 0.0,
) -> InventoryAnswer:
    """The inventory restricted to the query, under its evidence floor and age ceiling."""
    config = config or InventoryConfig()
    if min_confidence not in CONFIDENCE_NAMES:
        return InventoryAnswer(False, f"unknown min_confidence {min_confidence}")
    if max_age_s < 0:
        return InventoryAnswer(False, "max_age_s must not be negative")
    known_lanes = set(readings) | (set(world.lanes) if world else set())
    if lane_id and lane_id not in known_lanes:
        return InventoryAnswer(False, f"unknown lane {lane_id!r}")

    if lane_id:
        # One lane: its own contribution, with no tray and nothing from the other lanes.
        only = {lane_id: readings[lane_id]} if lane_id in readings else {}
        lane_world = None
        if world is not None:
            lane_world = WorldView(
                revision=world.revision,
                lanes={lane_id: world.lanes[lane_id]} if lane_id in world.lanes else {},
            )
        view = build_inventory(
            only,
            lane_world,
            None,
            now_s,
            config,
            max_age_s=max_age_s,
            min_confidence=min_confidence,
        )
        line = view.lane(lane_id)
        for sku_line in view.skus:
            sku_line.complete = bool(line and line.counted)
    else:
        view = build_inventory(
            readings,
            world,
            tray,
            now_s,
            config,
            max_age_s=max_age_s,
            min_confidence=min_confidence,
        )

    if sku:
        line_sku = view.sku(sku)
        answer = _sku_answer(view, line_sku, sku, lane_id)
        view.skus = [] if line_sku is None else [line_sku]
        if not lane_id:
            holding = {
                line.lane_id
                for line in view.lanes
                if line.expected_sku == sku or sku in line.confirmed_skus
            }
            view.lanes = [line for line in view.lanes if line.lane_id in holding]
        answer.view = view
        return answer
    if lane_id:
        return InventoryAnswer(
            True, "ok", view=view, complete=bool(view.lanes and view.lanes[0].counted)
        )
    return InventoryAnswer(
        True,
        "ok",
        view=view,
        complete=all(s.complete for s in view.skus if not s.is_unknown_bucket),
    )


def _sku_answer(view: InventoryView, line, sku: str, lane_id: str) -> InventoryAnswer:
    scope = f"lane {lane_id}" if lane_id else "stock"
    if line is None:
        # Nothing names it and no lane's policy lists it. Say what that does and does not mean.
        unread = [x.lane_id for x in view.lanes if not x.counted]
        complete = not unread and (bool(lane_id) or view.tray_source != TRAY_SOURCE_NONE)
        text = f"no lane policy lists {sku} and no read lane named it: 0 observed"
        if unread:
            text += f"; lanes {', '.join(unread)} were not readable and could hold it"
        return InventoryAnswer(True, "ok", count_known=True, complete=complete, explanation=text)
    known = line.total_max > 0 or not (line.unread_lanes or line.stale_lanes)
    parts = []
    if line.confirmed_count:
        parts.append(f"{line.confirmed_count} confirmed by the identity channel")
    if line.expected_count_max:
        low, high = line.expected_count_min, line.expected_count_max
        span = f"{high}" if low == high else f"{low}..{high}"
        parts.append(f"{span} counted in lanes whose policy lists {sku} (not individually named)")
    if line.reported_count:
        parts.append(f"{line.reported_count} reported by the world state only (no sensor)")
    text = f"{line.total_estimate} of {sku} in {scope}"
    if line.total_min != line.total_max:
        text += f" (between {line.total_min} and {line.total_max})"
    text += ": " + ("; ".join(parts) if parts else "nothing counted")
    if line.unread_lanes:
        text += f". Lanes {', '.join(line.unread_lanes)} could not be read, so this may be low"
    if line.stale_lanes:
        text += (
            f". Lanes {', '.join(line.stale_lanes)} were last read more than "
            f"{view.max_age_s:g} s ago and are left out"
        )
    if line.contested_lanes:
        text += (
            f". Lanes {', '.join(line.contested_lanes)} hold a wrong product, "
            "so their share is widened"
        )
    if not lane_id:
        if view.tray_source == TRAY_SOURCE_NONE:
            text += ". The tray is not covered (no perception reading and no world state)"
        elif view.tray_source == TRAY_SOURCE_WORLD_STATE and line.tray_count:
            text += f". {line.tray_count} on the tray, from the world state"
        elif view.tray_source == TRAY_SOURCE_PERCEPTION and line.tray_count:
            text += f". {line.tray_count} on the tray, from perception"
        if view.tray_stale:
            text += ". The tray's perception reading is stale"
    if line.excluded_count:
        text += (
            f". {line.excluded_count} more rest on less evidence than "
            f"{CONFIDENCE_NAMES[view.min_confidence]} and are not counted"
        )
    if line.has_world and line.complete and line.world_difference:
        text += (
            f". The world state says {line.world_count} (difference {line.world_difference:+d})"
        )
    return InventoryAnswer(
        True,
        "ok",
        count_known=known,
        count_min=line.total_min,
        count_max=line.total_max,
        count_estimate=line.total_estimate,
        confirmed_count=line.confirmed_count,
        excluded_count=line.excluded_count,
        complete=line.complete,
        explanation=text,
    )


__all__ = ["InventoryAnswer", "query_inventory"]

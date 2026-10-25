# Copyright 2026 Matteo Bernacchia <dev@kikijiki.com>
#
# SPDX-License-Identifier: MIT

"""Plain-text tables of an inventory, for the operator tool."""
# Duck-typed on field names, so it prints an InventoryView or a restocker_interfaces/msg/Inventory.

from __future__ import annotations

from typing import Any

from restocker_perception.inventory import model as m

OBSERVATION = {
    m.OBSERVATION_NEVER: "never",
    m.OBSERVATION_FRESH: "fresh",
    m.OBSERVATION_STALE: "STALE",
}
BASIS = {
    m.SKU_BASIS_UNKNOWN: "unknown",
    m.SKU_BASIS_EXPECTED_ONLY: "policy only",
    m.SKU_BASIS_PARTLY_CONFIRMED: "partly confirmed",
    m.SKU_BASIS_CONFIRMED: "confirmed",
    m.SKU_BASIS_CONTESTED: "CONTESTED",
}
TRAY_SOURCE = {
    m.TRAY_SOURCE_NONE: "not covered",
    m.TRAY_SOURCE_WORLD_STATE: "world state (not sensed)",
    m.TRAY_SOURCE_PERCEPTION: "perception",
}
DISCREPANCY = {
    m.COUNT_DISCREPANCY_NONE: "agrees",
    m.COUNT_DISCREPANCY_OBSERVED_HIGHER: "more seen than ledger",
    m.COUNT_DISCREPANCY_OBSERVED_LOWER: "fewer seen than ledger",
    m.COUNT_DISCREPANCY_NOT_COMPARABLE: "not compared",
}
REASONS = {
    1: "no evidence",
    2: "low coverage",
    3: "obstructed",
    4: "inconsistent geometry",
    5: "no policy",
}


def _span(low: int, high: int) -> str:
    return f"{high}" if low == high else f"{low}..{high}"


def lane_row(line: Any) -> list[str]:
    if line.observation == m.OBSERVATION_NEVER:
        count = "-"
    elif line.cannot_tell:
        count = f"cannot tell ({REASONS.get(line.cannot_tell_reason, line.cannot_tell_reason)})"
    else:
        count = _span(line.count_min, line.count_max)
    notes = []
    if line.misplaced:
        notes.append(
            "misplaced" + (f" {', '.join(line.misplaced_skus)}" if line.misplaced_skus else "")
        )
    if line.unknown_count:
        notes.append(f"{line.unknown_count} unregistered")
    if line.world_ledger_unreliable:
        notes.append("ledger unreliable")
    if line.sku_discrepancy or line.count_discrepancy in (
        m.COUNT_DISCREPANCY_OBSERVED_HIGHER,
        m.COUNT_DISCREPANCY_OBSERVED_LOWER,
    ):
        notes.append(line.discrepancy_detail or DISCREPANCY[line.count_discrepancy])
    return [
        line.lane_id,
        line.expected_sku if line.has_expected_sku else "-",
        OBSERVATION[line.observation],
        "-" if line.observation == m.OBSERVATION_NEVER else f"{line.age_s:.0f} s",
        count,
        BASIS[line.sku_basis]
        if line.observation != m.OBSERVATION_NEVER and not line.cannot_tell
        else "-",
        str(line.world_count) if line.has_world else "-",
        "; ".join(notes),
    ]


def sku_row(line: Any) -> list[str]:
    name = "(unknown SKU)" if line.is_unknown_bucket else line.sku
    notes = []
    if line.unread_lanes:
        notes.append(f"unread {', '.join(line.unread_lanes)}")
    if line.stale_lanes:
        notes.append(f"stale {', '.join(line.stale_lanes)}")
    if line.contested_lanes:
        notes.append(f"contested {', '.join(line.contested_lanes)}")
    if line.has_world and line.complete and line.world_difference:
        notes.append(f"world says {line.world_count}")
    return [
        name,
        str(line.total_estimate),
        _span(line.total_min, line.total_max),
        str(line.confirmed_count),
        _span(line.expected_count_min, line.expected_count_max),
        str(line.reported_count),
        str(line.tray_count),
        "yes" if line.complete else "NO",
        "; ".join(notes),
    ]


def table(header: list[str], rows: list[list[str]]) -> str:
    widths = [max(len(str(r[i])) for r in [header, *rows]) for i in range(len(header))]
    return "\n".join(
        "  ".join(str(cell).ljust(width) for cell, width in zip(row, widths, strict=True)).rstrip()
        for row in [header, *rows]
    )


def format_inventory(view: Any) -> str:
    world = f"revision {view.world_revision}" if view.world_available else "not available"
    out = [
        f"stale after {view.stale_after_s:g} s (this view: {view.max_age_s:g} s); "
        f"world state {world}; "
        f"tray: {TRAY_SOURCE[view.tray_source]}"
        + (" (perception reading stale)" if view.tray_stale else ""),
        "",
        table(
            ["lane", "policy", "reading", "age", "count", "SKU basis", "ledger", "notes"],
            [lane_row(x) for x in view.lanes],
        ),
        "",
        table(
            [
                "SKU",
                "total",
                "range",
                "confirmed",
                "assumed",
                "reported",
                "tray",
                "complete",
                "notes",
            ],
            [sku_row(x) for x in view.skus],
        ),
        "",
        "total = confirmed (named by the identity channel) + assumed (counted in a lane whose "
        "policy lists the SKU, not individually named) + reported (world state only).",
        "A range covers the count, not a wrong product hidden behind the one the camera sees.",
    ]
    return "\n".join(out)

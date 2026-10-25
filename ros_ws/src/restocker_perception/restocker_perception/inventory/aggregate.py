# Copyright 2026 Matteo Bernacchia <dev@kikijiki.com>
#
# SPDX-License-Identifier: MIT

"""Aggregate lane stock readings, the tray and the world state into one inventory."""
# The rules, in the order they matter:
#
#   * A lane's count is the estimator's range. Its SKU is the lane policy's unless the identity
#     channel named the products; the two are never merged into one number, because a policy SKU is
#     an assumption (a hidden wrong product is invisible to a survey frame) and a named one is
#     evidence. SKU totals therefore split ``confirmed`` from ``expected``.
#   * A lane flagged misplaced is contested: the policy SKU is not assumed for its unnamed products
#     (their share starts at zero), and when no wrong SKU was named the wrong products' SKU is
#     unknown, which the unknown bucket carries.
#   * A refusal (cannot-tell), a lane never read and a lane older than the horizon contribute
#     nothing to any total. They make the SKUs whose policy lane they are incomplete, instead of
#     reading as zero.
#   * Tray products come from the perception path when it has a fresh reading, else from the world
#     state (``reported``: no sensor looked). A stale perception reading falls back to the world
#     state and says so.
#   * The world state is compared, never merged: a lane's observed count range that excludes the
#     ledger's count is a discrepancy; a lane that could not be compared is NOT_COMPARABLE, which
#     is not agreement.

from __future__ import annotations

from collections import Counter
from collections.abc import Iterable, Mapping

from restocker_perception.inventory.model import (
    COMPOSITION_EMPTY,
    COMPOSITION_MISPLACED,
    CONFIDENCE_ANY,
    CONFIDENCE_CONFIRMED,
    COUNT_DISCREPANCY_NONE,
    COUNT_DISCREPANCY_NOT_COMPARABLE,
    COUNT_DISCREPANCY_OBSERVED_HIGHER,
    COUNT_DISCREPANCY_OBSERVED_LOWER,
    OBSERVATION_FRESH,
    OBSERVATION_NEVER,
    OBSERVATION_STALE,
    SKU_BASIS_CONFIRMED,
    SKU_BASIS_CONTESTED,
    SKU_BASIS_EXPECTED_ONLY,
    SKU_BASIS_PARTLY_CONFIRMED,
    SKU_BASIS_UNKNOWN,
    TRAY_SOURCE_NONE,
    TRAY_SOURCE_PERCEPTION,
    TRAY_SOURCE_WORLD_STATE,
    InventoryConfig,
    InventoryView,
    LaneLine,
    LaneReading,
    SkuLine,
    TrayReading,
    WorldLane,
    WorldView,
)

UNKNOWN = ""  # key of the unknown bucket in the working tables


class _Share:
    """A SKU's running share: exact confirmed, a range of expected, an estimate, reported."""

    def __init__(self) -> None:
        self.confirmed = 0
        self.expected_lo = 0
        self.expected_hi = 0
        self.expected_est = 0
        self.reported = 0
        self.tray = 0


def _line_for(
    reading: LaneReading,
    world_lane: WorldLane | None,
    now_s: float,
    horizon_s: float,
) -> tuple[LaneLine, dict[str, tuple[int, int, int, int]]]:
    """One lane's line, and its contribution: sku -> (confirmed, expected lo, hi, estimate)."""
    line = LaneLine(lane_id=reading.lane_id)
    line.has_expected_sku = reading.has_expected_sku
    line.expected_sku = reading.expected_sku
    age = max(0.0, now_s - reading.stamp_s)
    line.observed_at_s, line.age_s = reading.stamp_s, age
    line.observation = OBSERVATION_STALE if age > horizon_s else OBSERVATION_FRESH
    line.cannot_tell = not reading.answered
    line.cannot_tell_reason = 0 if reading.answered else reading.reason
    _attach_world(line, world_lane)
    if not reading.answered:
        return line, {}

    names = reading.named_skus
    unknown_faces = reading.unknown_faces
    claimed = len(names) + unknown_faces
    # Faces cannot outnumber products; if a channel names more than the geometry counted, the
    # names are evidence of products and the count's top end follows them.
    count_min, count_max = reading.count_min, max(reading.count_max, claimed)
    count_est = max(reading.count_estimate, claimed)
    rest_lo, rest_hi, rest_est = (
        max(0, count_min - claimed),
        max(0, count_max - claimed),
        max(0, count_est - claimed),
    )
    contested = reading.misplaced or reading.composition == COMPOSITION_MISPLACED
    line.count_estimate, line.count_min, line.count_max = count_est, count_min, count_max
    line.empty = reading.composition == COMPOSITION_EMPTY or count_max == 0
    line.confirmed_count = len(names)
    line.confirmed_skus = list(names)
    line.unknown_count = unknown_faces
    line.misplaced = contested
    line.misplaced_skus = list(reading.misplaced_skus)

    expected = reading.expected_sku if reading.has_expected_sku else None
    named_expected = sum(1 for n in names if n == expected)
    if contested:
        line.sku_basis = SKU_BASIS_CONTESTED
    elif line.empty or expected is None:
        line.sku_basis = SKU_BASIS_UNKNOWN
    elif named_expected and named_expected >= count_est:
        line.sku_basis = SKU_BASIS_CONFIRMED
    elif named_expected:
        line.sku_basis = SKU_BASIS_PARTLY_CONFIRMED
    else:
        line.sku_basis = SKU_BASIS_EXPECTED_ONLY

    share: dict[str, list[int]] = {}

    def add(sku: str, confirmed: int, lo: int, hi: int, est: int) -> None:
        cell = share.setdefault(sku, [0, 0, 0, 0])
        cell[0] += confirmed
        cell[1] += lo
        cell[2] += hi
        cell[3] += est

    for sku, count in Counter(names).items():
        add(sku, count, 0, 0, 0)
    if unknown_faces:
        add(UNKNOWN, 0, unknown_faces, unknown_faces, unknown_faces)
    if rest_hi:
        if contested:
            # A wrong product is in the lane, so no unnamed product is known to be the policy's.
            if expected is not None:
                add(expected, 0, 0, rest_hi, rest_est)
            if reading.misplaced_named_count == 0:
                # Flagged by class, nobody named the wrong product: its SKU is unknown.
                add(UNKNOWN, 0, 0, rest_hi, 0 if expected is not None else rest_est)
        elif expected is not None:
            add(expected, 0, rest_lo, rest_hi, rest_est)
        else:
            add(UNKNOWN, 0, rest_lo, rest_hi, rest_est)
    return line, {k: (v[0], v[1], v[2], v[3]) for k, v in share.items()}


def _attach_world(line: LaneLine, world_lane: WorldLane | None) -> None:
    if world_lane is None:
        line.has_world = False
        line.count_discrepancy = COUNT_DISCREPANCY_NOT_COMPARABLE
        return
    line.has_world = True
    line.world_count = world_lane.count
    line.world_ledger_unreliable = world_lane.ledger_unreliable


def _compare(line: LaneLine, reading: LaneReading, world_lane: WorldLane | None) -> None:
    """Fill the discrepancy fields of a counted lane against its world ledger."""
    if world_lane is None:
        line.count_discrepancy = COUNT_DISCREPANCY_NOT_COMPARABLE
        return
    ledger = world_lane.count
    if ledger < line.count_min:
        line.count_discrepancy = COUNT_DISCREPANCY_OBSERVED_HIGHER
        line.count_delta = line.count_min - ledger
    elif ledger > line.count_max:
        line.count_discrepancy = COUNT_DISCREPANCY_OBSERVED_LOWER
        line.count_delta = line.count_max - ledger
    else:
        line.count_discrepancy = COUNT_DISCREPANCY_NONE
    notes = []
    if line.count_discrepancy != COUNT_DISCREPANCY_NONE:
        notes.append(f"observed {line.count_min}..{line.count_max}, ledger {ledger}")
    # The estimator names the rearmost product; the ledger's last entry is the rearmost.
    rear = world_lane.skus[-1] if world_lane.skus else None
    names = reading.named_skus
    if rear is not None and names and any(name != rear for name in names):
        line.sku_discrepancy = True
        notes.append(f"named {sorted(set(names))}, ledger rear {rear}")
    if world_lane.ledger_unreliable:
        notes.append("ledger flagged unreliable")
    line.discrepancy_detail = "; ".join(notes)


def build_inventory(
    readings: Mapping[str, LaneReading],
    world: WorldView | None,
    tray: TrayReading | None,
    now_s: float,
    config: InventoryConfig | None = None,
    *,
    max_age_s: float = 0.0,
    min_confidence: int = CONFIDENCE_ANY,
) -> InventoryView:
    """The inventory at ``now_s``. ``max_age_s`` of zero means the config's horizon."""
    config = config or InventoryConfig()
    horizon = max_age_s if max_age_s > 0 else config.stale_after_s
    lane_ids = sorted(set(readings) | (set(world.lanes) if world else set()))
    lines: list[LaneLine] = []
    contributions: dict[str, dict[str, tuple[int, int, int, int]]] = {}
    for lane_id in lane_ids:
        reading = readings.get(lane_id)
        world_lane = None if world is None else world.lanes.get(lane_id)
        if reading is None:
            line = LaneLine(lane_id=lane_id, observation=OBSERVATION_NEVER)
            if world_lane is not None:
                line.has_expected_sku = world_lane.expected_sku is not None
                line.expected_sku = world_lane.expected_sku or ""
            _attach_world(line, world_lane)
            lines.append(line)
            continue
        line, share = _line_for(reading, world_lane, now_s, horizon)
        if line.counted:
            _compare(line, reading, world_lane)
            contributions[lane_id] = share
        else:
            line.count_discrepancy = COUNT_DISCREPANCY_NOT_COMPARABLE
        lines.append(line)

    # Policy SKU of each lane: the reading's, else the world's.
    policy: dict[str, str] = {}
    for line in lines:
        if line.has_expected_sku:
            policy[line.lane_id] = line.expected_sku
        elif world is not None and (w := world.lanes.get(line.lane_id)) and w.expected_sku:
            policy[line.lane_id] = w.expected_sku

    shares: dict[str, _Share] = {}
    for lane_share in contributions.values():
        for sku, (confirmed, lo, hi, est) in lane_share.items():
            cell = shares.setdefault(sku, _Share())
            cell.confirmed += confirmed
            cell.expected_lo += lo
            cell.expected_hi += hi
            cell.expected_est += est

    # --- tray ------------------------------
    tray_source, tray_stamp, tray_age, tray_stale, tray_total = (
        TRAY_SOURCE_NONE,
        0.0,
        0.0,
        False,
        0,
    )
    if tray is not None:
        tray_stamp, tray_age = tray.stamp_s, max(0.0, now_s - tray.stamp_s)
        if tray_age <= horizon:
            tray_source = TRAY_SOURCE_PERCEPTION
            for item in tray.items:
                key = item.sku or UNKNOWN
                cell = shares.setdefault(key, _Share())
                if item.sku and item.identified:
                    cell.confirmed += 1
                else:  # a sensor saw it; its SKU is an assumption or unknown
                    cell.expected_lo += 1
                    cell.expected_hi += 1
                    cell.expected_est += 1
                cell.tray += 1
                tray_total += 1
        else:
            tray_stale = True
    if tray_source == TRAY_SOURCE_NONE and world is not None:
        tray_source = TRAY_SOURCE_WORLD_STATE
        for item in world.tray:
            cell = shares.setdefault(item.sku or UNKNOWN, _Share())
            cell.reported += 1
            cell.tray += 1
            tray_total += 1

    # --- world's per-SKU figure ------------------------------------------------------------------
    world_by_sku: Counter[str] = Counter()
    if world is not None:
        for w_lane in world.lanes.values():
            world_by_sku.update(s for s in w_lane.skus if s)
        world_by_sku.update(i.sku for i in world.tray if i.sku)

    # --- SKU lines ------------------------------
    sku_names = {s for s in shares if s != UNKNOWN} | set(policy.values())
    if world is not None:
        sku_names |= set(world_by_sku)
    sku_lines: list[SkuLine] = []
    for sku in sorted(sku_names) + ([UNKNOWN] if UNKNOWN in shares else []):
        cell = shares.get(sku, _Share())
        out = SkuLine(sku=sku, is_unknown_bucket=sku == UNKNOWN)
        confirmed, expected_lo, expected_hi, expected_est = (
            cell.confirmed,
            cell.expected_lo,
            cell.expected_hi,
            cell.expected_est,
        )
        reported = cell.reported
        tray_kept = cell.tray
        if min_confidence >= CONFIDENCE_CONFIRMED:
            out.excluded_count = expected_est + reported
            expected_lo = expected_hi = expected_est = reported = 0
            tray_kept = (
                cell.tray if tray_source == TRAY_SOURCE_PERCEPTION and sku != UNKNOWN else 0
            )
        elif min_confidence > CONFIDENCE_ANY:
            out.excluded_count = reported
            reported = 0
            tray_kept = cell.tray - cell.reported
        out.confirmed_count = confirmed
        out.expected_count_min, out.expected_count_max = expected_lo, expected_hi
        out.reported_count = reported
        out.total_min = confirmed + expected_lo + reported
        out.total_max = confirmed + expected_hi + reported
        out.total_estimate = confirmed + expected_est + reported
        out.tray_count = tray_kept
        out.tray_source = tray_source if cell.tray else TRAY_SOURCE_NONE
        if not out.is_unknown_bucket:
            for line in lines:
                holds = policy.get(line.lane_id) == sku
                named = sku in line.confirmed_skus or sku in line.misplaced_skus
                if not holds and not named:
                    continue
                if line.observation == OBSERVATION_NEVER or (
                    line.observation == OBSERVATION_FRESH and line.cannot_tell
                ):
                    out.unread_lanes.append(line.lane_id)
                elif line.observation == OBSERVATION_STALE:
                    out.stale_lanes.append(line.lane_id)
                elif line.misplaced:
                    out.contested_lanes.append(line.lane_id)
            out.complete = (
                not out.unread_lanes and not out.stale_lanes and tray_source != TRAY_SOURCE_NONE
            )
            if world is not None:
                out.has_world = True
                out.world_count = world_by_sku.get(sku, 0)
                out.world_difference = out.total_estimate - out.world_count if out.complete else 0
        else:
            out.complete = all(line.counted for line in lines) and tray_source != TRAY_SOURCE_NONE
        sku_lines.append(out)

    return InventoryView(
        now_s=now_s,
        stale_after_s=config.stale_after_s,
        max_age_s=horizon,
        min_confidence=min_confidence,
        lanes=lines,
        skus=sku_lines,
        tray_source=tray_source,
        tray_count=tray_total,
        tray_observed_at_s=tray_stamp,
        tray_age_s=tray_age,
        tray_stale=tray_stale,
        world_available=world is not None,
        world_revision=world.revision if world else 0,
    )


def lanes_that_can_hold(view: InventoryView, sku: str) -> Iterable[str]:
    """The lanes a SKU's total depends on (its policy lanes and any lane it was named in)."""
    for line in view.lanes:
        if line.expected_sku == sku or sku in line.confirmed_skus:
            yield line.lane_id

# Copyright 2026 Matteo Bernacchia <dev@kikijiki.com>
#
# SPDX-License-Identifier: MIT

"""Inputs and outputs of the inventory: plain dataclasses, no ROS."""
# The inputs are what the rest of the system already says: a lane stock statement per lane
# (``LaneReading``, field for field the LaneStock message), the world state's view of lanes and
# tray (``WorldView``) and, when the perception path produced one, a tray reading
# (``TrayReading``). The outputs mirror restocker_interfaces/msg/Inventory* so the node's
# conversion is mechanical. Constants repeat the messages' and are pinned to them by a contract
# test; this module must not import ROS.

from __future__ import annotations

from collections.abc import Mapping, Sequence
from dataclasses import dataclass, field
from typing import Any

# LaneStock.STATE_* / REASON_* / COMPOSITION_* / IDENTITY_*.
STATE_ESTIMATED = 0
COMPOSITION_MISPLACED = 3
COMPOSITION_EMPTY = 4
IDENTITY_NAMED = 1
IDENTITY_UNKNOWN = 3

# InventoryLane.*
OBSERVATION_NEVER = 0
OBSERVATION_FRESH = 1
OBSERVATION_STALE = 2
SKU_BASIS_UNKNOWN = 0
SKU_BASIS_EXPECTED_ONLY = 1
SKU_BASIS_PARTLY_CONFIRMED = 2
SKU_BASIS_CONFIRMED = 3
SKU_BASIS_CONTESTED = 4
COUNT_DISCREPANCY_NONE = 0
COUNT_DISCREPANCY_OBSERVED_HIGHER = 1
COUNT_DISCREPANCY_OBSERVED_LOWER = 2
COUNT_DISCREPANCY_NOT_COMPARABLE = 3

# Inventory.TRAY_SOURCE_*
TRAY_SOURCE_NONE = 0
TRAY_SOURCE_WORLD_STATE = 1
TRAY_SOURCE_PERCEPTION = 2

# QueryInventory.CONFIDENCE_*
CONFIDENCE_ANY = 0
CONFIDENCE_EXPECTED = 1
CONFIDENCE_CONFIRMED = 2

# ObjectObservation.IDENTITY_*
IDENTITY_STATE_IDENTIFIED = 1
IDENTITY_STATE_UNKNOWN_PRODUCT = 2


@dataclass(frozen=True)
class LaneReading:
    """One LaneStock statement and the time it was made (seconds on the inventory's clock)."""

    lane_id: str
    stamp_s: float
    answered: bool
    reason: int = 0
    has_expected_sku: bool = False
    expected_sku: str = ""
    count_estimate: int = 0
    count_min: int = 0
    count_max: int = 0
    composition: int = 0
    misplaced: bool = False
    face_skus: tuple[str, ...] = ()
    face_identity_status: tuple[int, ...] = ()
    misplaced_named_count: int = 0
    misplaced_skus: tuple[str, ...] = ()

    @classmethod
    def from_stock(cls, stock: Any, stamp_s: float) -> LaneReading:
        """From a LaneStock message or a LaneStockEstimate (same field names)."""
        return cls(
            lane_id=stock.lane_id,
            stamp_s=float(stamp_s),
            answered=int(stock.state) == STATE_ESTIMATED,
            reason=int(stock.reason),
            has_expected_sku=bool(stock.has_expected_sku),
            expected_sku=str(stock.expected_sku) if stock.has_expected_sku else "",
            count_estimate=int(stock.count_estimate),
            count_min=int(stock.count_min),
            count_max=int(stock.count_max),
            composition=int(stock.composition),
            misplaced=bool(stock.misplaced),
            face_skus=tuple(str(s) for s in stock.face_skus),
            face_identity_status=tuple(int(s) for s in stock.face_identity_status),
            misplaced_named_count=int(stock.misplaced_named_count),
            misplaced_skus=tuple(str(s) for s in stock.misplaced_skus),
        )

    @classmethod
    def from_row(
        cls, row: Mapping[str, Any], expected_sku: str | None, stamp_s: float
    ) -> LaneReading:
        """From a stock benchmark prediction row (restocker_perception.stock.benchmark)."""
        answered = row["state"] == "estimated"
        misplaced_skus = tuple(row.get("misplaced_skus") or ())
        return cls(
            lane_id=row["lane_id"],
            stamp_s=float(stamp_s),
            answered=answered,
            has_expected_sku=bool(expected_sku),
            expected_sku=expected_sku or "",
            count_estimate=int(row["count"]) if answered else 0,
            count_min=int(row["count_min"]) if answered else 0,
            count_max=int(row["count_max"]) if answered else 0,
            composition=int(row["composition"]) if answered else 0,
            misplaced=bool(row["misplaced"]) if answered else False,
            face_skus=tuple(row.get("face_skus") or ()),
            face_identity_status=tuple(int(s) for s in row.get("face_identity_status") or ()),
            misplaced_named_count=len(misplaced_skus),
            misplaced_skus=misplaced_skus,
        )

    @property
    def named_skus(self) -> tuple[str, ...]:
        """The SKU of every product the identity channel named, one entry per product."""
        return tuple(
            sku
            for sku, status in zip(self.face_skus, self.face_identity_status, strict=False)
            if status == IDENTITY_NAMED and sku
        )

    @property
    def unknown_faces(self) -> int:
        """Products the channel has positive evidence are not any registered SKU."""
        return sum(1 for s in self.face_identity_status if s == IDENTITY_UNKNOWN)


@dataclass(frozen=True)
class WorldLane:
    """The world state's side of one lane: policy and the FIFO ledger."""

    lane_id: str
    expected_sku: str | None
    # SKU of each ledger entry from the customer-facing front to the rearmost; None where the
    # object has no SKU (or is not tracked any more).
    skus: tuple[str | None, ...] = ()
    ledger_unreliable: bool = False

    @property
    def count(self) -> int:
        return len(self.skus)


@dataclass(frozen=True)
class WorldTrayItem:
    sku: str | None
    object_id: str = ""


@dataclass(frozen=True)
class WorldView:
    """What the authoritative world state says (in simulation it is fed ground truth)."""

    revision: int = 0
    lanes: Mapping[str, WorldLane] = field(default_factory=dict)
    tray: Sequence[WorldTrayItem] = ()


@dataclass(frozen=True)
class TrayItem:
    """One tray product from the perception path: a proposal and its identity state."""

    # The SKU on the item, and whether it is identity evidence (the channel named it) or only an
    # assumption (the manifest's name for a detected class). No SKU: nobody could name it.
    sku: str | None
    identified: bool = False


@dataclass(frozen=True)
class TrayReading:
    stamp_s: float
    items: Sequence[TrayItem] = ()


@dataclass(frozen=True)
class InventoryConfig:
    """Declared parameters. The horizon is a deployment decision, not a measured constant."""

    # An observation older than this (at the inventory's stamp) is stale and not counted.
    stale_after_s: float = 900.0


# --- outputs ----------------------------------


@dataclass
class LaneLine:
    lane_id: str
    observation: int = OBSERVATION_NEVER
    observed_at_s: float = 0.0
    age_s: float = 0.0
    cannot_tell: bool = False
    cannot_tell_reason: int = 0
    has_expected_sku: bool = False
    expected_sku: str = ""
    sku_basis: int = SKU_BASIS_UNKNOWN
    confirmed_count: int = 0
    confirmed_skus: list[str] = field(default_factory=list)
    unknown_count: int = 0
    misplaced: bool = False
    misplaced_skus: list[str] = field(default_factory=list)
    count_estimate: int = 0
    count_min: int = 0
    count_max: int = 0
    empty: bool = False
    has_world: bool = False
    world_count: int = 0
    world_ledger_unreliable: bool = False
    count_discrepancy: int = COUNT_DISCREPANCY_NOT_COMPARABLE
    count_delta: int = 0
    sku_discrepancy: bool = False
    discrepancy_detail: str = ""

    @property
    def counted(self) -> bool:
        """The line's count is an answer and is fresh: it goes into the totals."""
        return self.observation == OBSERVATION_FRESH and not self.cannot_tell

    @property
    def has_discrepancy(self) -> bool:
        return self.sku_discrepancy or self.count_discrepancy in (
            COUNT_DISCREPANCY_OBSERVED_HIGHER,
            COUNT_DISCREPANCY_OBSERVED_LOWER,
        )


@dataclass
class SkuLine:
    sku: str
    is_unknown_bucket: bool = False
    confirmed_count: int = 0
    expected_count_min: int = 0
    expected_count_max: int = 0
    reported_count: int = 0
    total_min: int = 0
    total_max: int = 0
    total_estimate: int = 0
    complete: bool = True
    unread_lanes: list[str] = field(default_factory=list)
    stale_lanes: list[str] = field(default_factory=list)
    contested_lanes: list[str] = field(default_factory=list)
    tray_count: int = 0
    tray_source: int = TRAY_SOURCE_NONE
    has_world: bool = False
    world_count: int = 0
    world_difference: int = 0
    # Products left out of the totals by the confidence floor (not in the message).
    excluded_count: int = 0


@dataclass
class InventoryView:
    now_s: float
    stale_after_s: float
    max_age_s: float
    min_confidence: int
    lanes: list[LaneLine]
    skus: list[SkuLine]
    tray_source: int = TRAY_SOURCE_NONE
    tray_count: int = 0
    tray_observed_at_s: float = 0.0
    tray_age_s: float = 0.0
    tray_stale: bool = False
    world_available: bool = False
    world_revision: int = 0

    @property
    def lanes_total(self) -> int:
        return len(self.lanes)

    @property
    def lanes_fresh(self) -> int:
        return sum(1 for line in self.lanes if line.observation == OBSERVATION_FRESH)

    @property
    def lanes_stale(self) -> int:
        return sum(1 for line in self.lanes if line.observation == OBSERVATION_STALE)

    @property
    def lanes_unread(self) -> int:
        return sum(1 for line in self.lanes if not line.counted)

    @property
    def discrepancy_count(self) -> int:
        return sum(1 for line in self.lanes if line.has_discrepancy)

    def sku(self, sku: str) -> SkuLine | None:
        return next(
            (line for line in self.skus if line.sku == sku and not line.is_unknown_bucket), None
        )

    def lane(self, lane_id: str) -> LaneLine | None:
        return next((line for line in self.lanes if line.lane_id == lane_id), None)

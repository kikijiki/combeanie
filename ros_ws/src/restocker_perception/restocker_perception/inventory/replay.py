# Copyright 2026 Matteo Bernacchia <dev@kikijiki.com>
#
# SPDX-License-Identifier: MIT

"""Replay recorded lane stock predictions as inventories, against the benchmark's ground truth."""
# Everything here is a pure function of the labels, the prediction rows and a seed, so a replay is
# reproducible and nothing is fitted to what it scores. The scene is one benchmark seed: the six
# lanes' labelled contents and the tray's. The estimator's prediction row for each lane's survey
# frame becomes a LaneReading; the world state is the labelled truth, optionally perturbed by a
# declared, seeded schedule; observation ages follow a seeded schedule so staleness can be scored.
#
# Declared perturbations (``bounds file`` states the same numbers):
#   count    a third of the lanes get a ledger that differs from the truth by 1 or 2 products,
#            removed from or added at the front (the customer-facing end), so the rear product the
#            survey sees is unchanged;
#   sku      a quarter of the lanes holding a product get the ledger's rearmost SKU replaced by
#            another SKU of the scene.
# The tray is the world state's: this dataset has no tray reading (its wrist frames at home show
# nothing and the overhead frames are black), so the tray is world-sourced and trivially right.

from __future__ import annotations

from collections import Counter
from collections.abc import Iterable, Mapping, Sequence
from dataclasses import dataclass, field
import hashlib
from typing import Any

from restocker_perception.inventory.aggregate import build_inventory
from restocker_perception.inventory.model import (
    InventoryConfig,
    InventoryView,
    LaneReading,
    WorldLane,
    WorldTrayItem,
    WorldView,
)

COUNT_PERTURBATION_SHARE = 1 / 3
SKU_PERTURBATION_SHARE = 1 / 4
COUNT_DELTAS = (-2, -1, 1, 2)
# Query time and the age schedule's span, in units of the stale horizon.
QUERY_TIME_S = 100000.0
AGE_SPAN_HORIZONS = 1.5
FRESH_AGE_SHARE = 0.9


@dataclass(frozen=True)
class LaneTruth:
    lane_id: str
    expected_sku: str | None
    skus: tuple[str, ...]  # front to rear


@dataclass(frozen=True)
class SceneTruth:
    seed: int
    lanes: Mapping[str, LaneTruth]
    tray: tuple[str, ...]

    def sku_totals(self) -> Counter[str]:
        total: Counter[str] = Counter(self.tray)
        for lane in self.lanes.values():
            total.update(lane.skus)
        return total


@dataclass
class Perturbation:
    count_delta: dict[str, int] = field(default_factory=dict)
    sku_swapped: dict[str, str] = field(default_factory=dict)  # lane -> wrong rear SKU


def draw(*key: Any) -> float:
    """A uniform [0, 1) draw that depends only on the key (the same on every machine)."""
    digest = hashlib.sha256(":".join(str(k) for k in ("inventory-replay", *key)).encode()).digest()
    return int.from_bytes(digest[:8], "little") / 2**64


def scene_truth(label: Mapping[str, Any]) -> SceneTruth:
    """The scene from any one of its labelled frames (lane contents are the same in each)."""
    lanes = {
        lane["lane_id"]: LaneTruth(
            lane["lane_id"], lane.get("expected_sku"), tuple(lane["skus_in_order"])
        )
        for lane in label["lanes"]
    }
    tray = tuple(sorted(o["sku"] for o in label["objects"] if o.get("lane_id") is None))
    return SceneTruth(int(label["seed"]), lanes, tray)


def perturbation(truth: SceneTruth) -> Perturbation:
    """The declared, seeded disagreement between the world state and the truth."""
    out = Perturbation()
    scene_skus = sorted(
        set(truth.sku_totals())
        | {lane.expected_sku for lane in truth.lanes.values() if lane.expected_sku}
    )
    for lane_id, lane in sorted(truth.lanes.items()):
        if draw("count", truth.seed, lane_id) < COUNT_PERTURBATION_SHARE:
            delta = COUNT_DELTAS[int(draw("count-delta", truth.seed, lane_id) * len(COUNT_DELTAS))]
            if len(lane.skus) + delta < 0:
                delta = abs(delta)
            out.count_delta[lane_id] = delta
        # A swap needs a ledger that still has a rearmost entry after the count perturbation.
        left = len(lane.skus) + out.count_delta.get(lane_id, 0)
        if lane.skus and left > 0 and draw("sku", truth.seed, lane_id) < SKU_PERTURBATION_SHARE:
            others = [s for s in scene_skus if s != lane.skus[-1]]
            if others:
                out.sku_swapped[lane_id] = others[
                    int(draw("sku-pick", truth.seed, lane_id) * len(others))
                ]
    return out


def world_view(truth: SceneTruth, perturb: Perturbation | None = None) -> WorldView:
    """The world state: the truth, plus the perturbation when one is given."""
    lanes = {}
    for lane_id, lane in truth.lanes.items():
        skus = list(lane.skus)
        delta = perturb.count_delta.get(lane_id, 0) if perturb else 0
        if delta < 0:
            skus = skus[-delta:]  # forget the front products
        elif delta > 0:
            filler = lane.expected_sku or (skus[0] if skus else None)
            skus = [filler] * delta + skus
        if perturb and lane_id in perturb.sku_swapped and skus:
            skus[-1] = perturb.sku_swapped[lane_id]
        lanes[lane_id] = WorldLane(lane_id, lane.expected_sku, tuple(skus))
    return WorldView(revision=1, lanes=lanes, tray=tuple(WorldTrayItem(sku) for sku in truth.tray))


def reading_ages(truth: SceneTruth, horizon_s: float, *, aged: bool) -> dict[str, float]:
    """Seconds since each lane was read. Fresh: all inside the horizon. Aged: spread past it."""
    span = horizon_s * (AGE_SPAN_HORIZONS if aged else FRESH_AGE_SHARE)
    return {
        lane_id: draw("age", truth.seed, lane_id, aged) * span for lane_id in sorted(truth.lanes)
    }


def readings_for_scene(
    truth: SceneTruth,
    rows: Iterable[Mapping[str, Any]],
    ages: Mapping[str, float],
    now_s: float = QUERY_TIME_S,
) -> dict[str, LaneReading]:
    """One LaneReading per predicted lane of the scene, stamped now - age."""
    out = {}
    for row in rows:
        lane = truth.lanes[row["lane_id"]]
        out[lane.lane_id] = LaneReading.from_row(
            row, lane.expected_sku, now_s - ages.get(lane.lane_id, 0.0)
        )
    return out


def replay_scene(
    truth: SceneTruth,
    rows: Sequence[Mapping[str, Any]],
    config: InventoryConfig,
    *,
    perturb: Perturbation | None = None,
    aged: bool = False,
    drop_stale: bool = False,
    **filters: Any,
) -> tuple[InventoryView, dict[str, float]]:
    """The inventory the system would hold for one seed, and the ages it was given."""
    ages = reading_ages(truth, config.stale_after_s, aged=aged)
    readings = readings_for_scene(truth, rows, ages)
    if drop_stale:
        readings = {k: v for k, v in readings.items() if ages[k] <= config.stale_after_s}
    view = build_inventory(
        readings, world_view(truth, perturb), None, QUERY_TIME_S, config, **filters
    )
    return view, ages

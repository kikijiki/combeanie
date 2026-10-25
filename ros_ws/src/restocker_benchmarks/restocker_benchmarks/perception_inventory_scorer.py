# Copyright 2026 Matteo Bernacchia <dev@kikijiki.com>
#
# SPDX-License-Identifier: MIT

"""Score the inventory (CMB-115) by replaying lane stock predictions against benchmark truth."""
# Additive to perception_stock_scorer.py. The estimator's prediction rows (one per lane survey
# frame; ``stock_benchmark predict``) are replayed as inventories per benchmark seed
# (restocker_perception.inventory.replay) and the aggregation is scored, not the estimator:
#
#   totals      per (seed, SKU) where the inventory says ``complete``: the total against the
#               labelled count (lanes and tray), within one product and as an interval. Pairs the
#               inventory calls incomplete are counted but not scored: it said it could not tell.
#   honesty     no SKU is complete while a lane whose policy it is was refused; no confirmed claim
#               the labels contradict.
#   staleness   lanes older than the declared horizon are flagged and leave the totals.
#   discrepancy against a world state perturbed by the declared schedule: recall on perturbed
#               lanes, false alarm on the others, refusals never counted as agreement.

from __future__ import annotations

from collections import Counter, defaultdict
from collections.abc import Iterable, Mapping, Sequence
from typing import Any

from restocker_perception.inventory import replay
from restocker_perception.inventory.model import (
    COUNT_DISCREPANCY_NONE,
    COUNT_DISCREPANCY_NOT_COMPARABLE,
    OBSERVATION_STALE,
    InventoryConfig,
)

from restocker_benchmarks.perception_scorer import rate
from restocker_benchmarks.perception_stock_scorer import _verdict


def scenes(
    labels: Sequence[Mapping[str, Any]], predictions: Iterable[Mapping[str, Any]]
) -> list[tuple[replay.SceneTruth, list[Mapping[str, Any]]]]:
    """Group lane predictions by the benchmark seed of the frame they were made for."""
    by_frame = {row["frame_id"]: row for row in labels}
    truths: dict[int, replay.SceneTruth] = {}
    rows: dict[int, list[Mapping[str, Any]]] = defaultdict(list)
    for prediction in predictions:
        if prediction.get("kind") != "lane" or "state" not in prediction:
            continue
        label = by_frame.get(prediction["frame_id"])
        if label is None:
            raise ValueError(f"prediction for unknown frame {prediction['frame_id']}")
        seed = int(label["seed"])
        truths.setdefault(seed, replay.scene_truth(label))
        rows[seed].append(prediction)
    return [(truths[s], rows[s]) for s in sorted(truths)]


def _policy_lanes(truth: replay.SceneTruth, sku: str) -> list[str]:
    return [lane.lane_id for lane in truth.lanes.values() if lane.expected_sku == sku]


def score_inventory(
    labels: Sequence[Mapping[str, Any]],
    predictions: Iterable[Mapping[str, Any]],
    config: InventoryConfig | None = None,
) -> dict[str, Any]:
    """Every inventory metric for one prediction set (one arm)."""
    config = config or InventoryConfig(stale_after_s=600.0)
    counts: Counter[str] = Counter()
    pair_errors: list[int] = []
    scene_list = scenes(labels, predictions)
    for truth, rows in scene_list:
        answered = {r["lane_id"]: r["state"] == "estimated" for r in rows}
        totals = truth.sku_totals()

        # --- totals and honesty: ideal world, everything fresh ----------------------------
        view, _ = replay.replay_scene(truth, rows, config)
        skus = set(totals) | {s.sku for s in view.skus if not s.is_unknown_bucket}
        for sku in sorted(skus):
            line = view.sku(sku)
            truth_total = totals.get(sku, 0)
            policy = _policy_lanes(truth, sku)
            counts["pairs"] += 1
            if line is None:
                continue
            refused = [lane for lane in policy if not answered.get(lane, False)]
            if refused:
                counts["pairs_with_refused_policy_lane"] += 1
                counts["refused_but_complete"] += int(line.complete)
            if not line.complete:
                counts["pairs_incomplete"] += 1
                continue
            counts["pairs_complete"] += 1
            error = line.total_estimate - truth_total
            pair_errors.append(error)
            counts["within_one"] += int(abs(error) <= 1)
            counts["exact"] += int(error == 0)
            counts["contains"] += int(line.total_min <= truth_total <= line.total_max)
        for lane in view.lanes:
            truth_lane = truth.lanes[lane.lane_id]
            if lane.confirmed_skus:
                counts["lanes_with_confirmed"] += 1
                excess = Counter(lane.confirmed_skus) - Counter(truth_lane.skus)
                counts["confirmed_contradicted"] += int(bool(excess))
            if lane.counted:
                counts["lanes_counted"] += 1
                counts["lane_contains"] += int(
                    lane.count_min <= len(truth_lane.skus) <= lane.count_max
                )

        # --- staleness: ages spread past the horizon ----------------------------
        aged_view, ages = replay.replay_scene(truth, rows, config, aged=True)
        stale_by_schedule = {k for k, age in ages.items() if age > config.stale_after_s}
        for lane in aged_view.lanes:
            is_stale = lane.observation == OBSERVATION_STALE
            if lane.lane_id in stale_by_schedule:
                counts["stale_lanes"] += 1
                counts["stale_flagged"] += int(is_stale)
            else:
                counts["fresh_lanes"] += 1
                counts["fresh_flagged_stale"] += int(is_stale)
        if stale_by_schedule:
            dropped, _ = replay.replay_scene(truth, rows, config, aged=True, drop_stale=True)
            same = all(
                (a.total_min, a.total_estimate, a.total_max)
                == (b.total_min, b.total_estimate, b.total_max)
                for a, b in zip(
                    sorted(aged_view.skus, key=lambda s: (s.is_unknown_bucket, s.sku)),
                    sorted(dropped.skus, key=lambda s: (s.is_unknown_bucket, s.sku)),
                    strict=False,
                )
            ) and len(aged_view.skus) == len(dropped.skus)
            counts["scenes_with_stale"] += 1
            counts["stale_left_out_of_totals"] += int(same)

        # --- discrepancy against a perturbed world state ----------------------------
        perturb = replay.perturbation(truth)
        disc_view, _ = replay.replay_scene(truth, rows, config, perturb=perturb)
        for lane in disc_view.lanes:
            counted = lane.counted
            changed = perturb.count_delta.get(lane.lane_id, 0) != 0
            swapped = lane.lane_id in perturb.sku_swapped
            flagged = lane.count_discrepancy not in (
                COUNT_DISCREPANCY_NONE,
                COUNT_DISCREPANCY_NOT_COMPARABLE,
            )
            if not answered.get(lane.lane_id, False):
                counts["refused_lanes"] += 1
                counts["refused_not_comparable"] += int(
                    lane.count_discrepancy == COUNT_DISCREPANCY_NOT_COMPARABLE
                    and not lane.has_discrepancy
                )
                counts["perturbed_unreadable"] += int(changed)
                continue
            if not counted:
                continue
            if changed:
                counts["count_perturbed"] += 1
                counts["count_perturbed_flagged"] += int(flagged)
            else:
                counts["count_clean"] += 1
                counts["count_clean_flagged"] += int(flagged)
            if swapped:
                counts["sku_swapped"] += 1
                counts["sku_swapped_named"] += int(bool(lane.confirmed_skus))
                if lane.confirmed_skus:
                    counts["sku_swapped_visible"] += 1
                    counts["sku_swapped_visible_flagged"] += int(lane.sku_discrepancy)
            else:
                counts["sku_clean"] += 1
                counts["sku_clean_flagged"] += int(lane.sku_discrepancy)
    c = counts
    return {
        "scenes": len(scene_list),
        "overall": {
            "sku_total_within_one": rate(c["within_one"], c["pairs_complete"]),
            "sku_total_exact": rate(c["exact"], c["pairs_complete"]),
            "sku_total_contains_truth": rate(c["contains"], c["pairs_complete"]),
            "complete_pair_share": rate(c["pairs_complete"], c["pairs"]),
            "refused_policy_lane_claimed_complete": rate(
                c["refused_but_complete"], c["pairs_with_refused_policy_lane"]
            ),
            "confirmed_claim_contradicted": rate(
                c["confirmed_contradicted"], c["lanes_with_confirmed"]
            ),
            "lane_range_contains_truth": rate(c["lane_contains"], c["lanes_counted"]),
            "stale_flagged": rate(c["stale_flagged"], c["stale_lanes"]),
            "fresh_flagged_stale": rate(c["fresh_flagged_stale"], c["fresh_lanes"]),
            "stale_left_out_of_totals": rate(
                c["stale_left_out_of_totals"], c["scenes_with_stale"]
            ),
            "count_discrepancy_recall": rate(c["count_perturbed_flagged"], c["count_perturbed"]),
            "count_discrepancy_false_alarm": rate(c["count_clean_flagged"], c["count_clean"]),
            "refusal_not_comparable": rate(c["refused_not_comparable"], c["refused_lanes"]),
            "perturbed_lanes_unreadable": rate(
                c["perturbed_unreadable"], c["perturbed_unreadable"] + c["count_perturbed"]
            ),
            "sku_swap_named_share": rate(c["sku_swapped_named"], c["sku_swapped"]),
            "sku_discrepancy_recall": rate(
                c["sku_swapped_visible_flagged"], c["sku_swapped_visible"]
            ),
            "sku_discrepancy_false_alarm": rate(c["sku_clean_flagged"], c["sku_clean"]),
        },
        "error_histogram": dict(sorted(Counter(pair_errors).items())),
        "counts": dict(c),
        "horizon_s": config.stale_after_s,
    }


def format_inventory_report(report: Mapping[str, Any]) -> str:
    lines = [f"scenes {report['scenes']}  horizon {report['horizon_s']:g} s"]
    for name, item in report["overall"].items():
        if item["value"] is None:
            lines.append(f"  {name:40s} n/a (n=0)")
            continue
        low, high = item["ci95"]
        lines.append(
            f"  {name:40s} {item['value']:.4f}  [{low:.3f}, {high:.3f}]  {item['k']}/{item['n']}"
        )
    lines.append(f"  SKU total error histogram (complete pairs): {report['error_histogram']}")
    return "\n".join(lines)


def evaluate_bounds(
    reports: Mapping[str, Mapping[str, Any]], bounds: Mapping[str, Any]
) -> list[dict[str, Any]]:
    """Apply the declared bounds to score reports keyed by arm; same pass rule as CMB-114."""
    rows = []
    for bound in bounds["bounds"]:
        for arm in bound["arms"]:
            report = reports.get(arm)
            if report is None:
                rows.append({"bound": bound["id"], "arm": arm, "verdict": "not_run"})
                continue
            item = report["overall"][bound["metric"]]
            rows.append(
                {
                    "bound": bound["id"],
                    "arm": arm,
                    "metric": bound["metric"],
                    "direction": bound["direction"],
                    "value": bound["value"],
                    "required_n": bound["required_n"],
                    "observed": item["value"],
                    "ci95": item["ci95"],
                    "k": item["k"],
                    "n": item["n"],
                    "verdict": _verdict(item, bound),
                }
            )
    return rows

# Copyright 2026 Matteo Bernacchia <dev@kikijiki.com>
#
# SPDX-License-Identifier: MIT

"""Score the lane stock estimator (CMB-114) against the benchmark's lane ground truth."""
# Additive to perception_scorer.py, which is unchanged: that scorer scores every in-view lane of
# every frame, and a lane survey frame frames one lane, so the neighbours it also sees would be
# scored as "no answer". This module scores the lane a survey frame was taken for, from rows of
# ``kind: "lane"`` that carry the stock fields (``state``, ``count``, ``measured_coverage``,
# ``misplaced``, ``composition``...; see restocker_perception.stock.benchmark).
#
# Definitions (all counts are lanes):
#   sufficient coverage   the frame's measured coverage is at or above the producer's floor
#                         (0.90). Defined on the evidence, not on the answer.
#   within one            |predicted count - true count| <= 1. A cannot-tell is a miss when it is
#                         on a sufficient-coverage lane and correct when the coverage is low.
#   misplaced detection   the estimator's ``misplaced`` flag against ``misplaced_count > 0``.
#                         It is raised only from a named wrong SKU or a class the policy does not
#                         allow; an abstention never raises it, so recall is bounded by what the
#                         camera can name. ``rear`` is the subset where the misplaced product is
#                         the one nearest the camera, the only one the survey can see.
#   conforming_but_misplaced   lanes declared COMPOSITION_CONFORMING that truly hold a misplaced
#                         product: the silent error the safe policy exists to prevent.

from __future__ import annotations

from collections import defaultdict
from collections.abc import Iterable, Mapping, Sequence
from typing import Any

from restocker_benchmarks.perception_scorer import rate, summary

MINIMUM_COVERAGE = 0.90
COMPOSITION_CONFORMING = 1
COUNT_BUCKETS = ((0, "0"), (2, "1-2"), (4, "3-4"), (99, "5+"))
CONDITION_KEYS = ("lighting", "seed_split", "product_class", "count", "arm", "degraded")


def _bucket(count: int) -> str:
    for upper, name in COUNT_BUCKETS:
        if count <= upper:
            return name
    return COUNT_BUCKETS[-1][1]


def lane_records(
    labels: Sequence[Mapping[str, Any]], predictions: Iterable[Mapping[str, Any]]
) -> list[dict[str, Any]]:
    """Join stock prediction rows to the labelled lane they were made for."""
    by_frame = {row["frame_id"]: row for row in labels}
    records = []
    for prediction in predictions:
        if prediction.get("kind") != "lane" or "state" not in prediction:
            continue
        label = by_frame.get(prediction["frame_id"])
        if label is None:
            raise ValueError(f"prediction for unknown frame {prediction['frame_id']}")
        lane = next(x for x in label["lanes"] if x["lane_id"] == prediction["lane_id"])
        members = [o for o in label["objects"] if o["lane_id"] == lane["lane_id"]]
        rear = max(members, key=lambda o: o["lane_order"]) if members else None
        records.append(
            {
                "frame_id": prediction["frame_id"],
                "lane_id": lane["lane_id"],
                "truth_count": lane["count"],
                "truth_misplaced": lane["misplaced_count"] > 0,
                "truth_rear_misplaced": bool(rear and rear["misplaced"]),
                "truth_class": members[0]["product_class"] if members else "empty",
                "prediction": dict(prediction),
                "conditions": {
                    "lighting": label["lighting"]["condition"],
                    "seed_split": label.get("seed_split") or "none",
                    "product_class": members[0]["product_class"] if members else "empty",
                    "count": _bucket(lane["count"]),
                    "arm": prediction.get("arm", "unspecified"),
                    "degraded": str(bool(prediction.get("degraded", False))).lower(),
                },
            }
        )
    return records


def aggregate(records: Sequence[Mapping[str, Any]]) -> dict[str, Any]:
    """The stock metrics of one group of lane records."""
    sufficient = [r for r in records if r["prediction"]["measured_coverage"] >= MINIMUM_COVERAGE]
    low = [r for r in records if r["prediction"]["measured_coverage"] < MINIMUM_COVERAGE]
    answered = [r for r in sufficient if r["prediction"]["state"] == "estimated"]

    def error(r: Mapping[str, Any]) -> int:
        return r["prediction"]["count"] - r["truth_count"]

    within = [r for r in answered if abs(error(r)) <= 1]
    reasons: dict[str, int] = defaultdict(int)
    for r in records:
        if r["prediction"]["state"] != "estimated":
            reasons[str(r["prediction"].get("reason"))] += 1
    empty_truth = [r for r in sufficient if r["truth_count"] == 0]
    low_answered = [r for r in low if r["prediction"]["state"] == "estimated"]
    flagged = lambda r: bool(r["prediction"]["misplaced"])  # noqa: E731
    misplaced_truth = [r for r in answered if r["truth_misplaced"]]
    clean = [r for r in answered if not r["truth_misplaced"]]
    rear = [r for r in answered if r["truth_rear_misplaced"]]
    conforming = [r for r in answered if r["prediction"]["composition"] == COMPOSITION_CONFORMING]
    non_empty = [r for r in answered if r["truth_count"] > 0]
    # Answered lanes, whatever their coverage, that are wrong: more than one product off, or any
    # product at all read in a lane that is empty (a phantom, which "within one" would hide).
    answered_any = [r for r in records if r["prediction"]["state"] == "estimated"]
    wrong = [
        r
        for r in answered_any
        if abs(error(r)) > 1 or (r["truth_count"] == 0 and r["prediction"]["count"] != 0)
    ]
    return {
        "n_lanes": len(records),
        "n_sufficient_coverage": len(sufficient),
        "n_low_coverage": len(low),
        "answered_on_sufficient": rate(len(answered), len(sufficient)),
        "within_one_on_sufficient": rate(len(within), len(sufficient)),
        "within_one_among_answered": rate(len(within), len(answered)),
        # Exact count over every lane the evidence was good enough for: a refusal is a miss here.
        "exact_on_sufficient": rate(sum(1 for r in answered if error(r) == 0), len(sufficient)),
        "exact_among_answered": rate(sum(1 for r in answered if error(r) == 0), len(answered)),
        "count_abs_error_among_answered": summary([abs(error(r)) for r in answered]),
        "empty_lane_read_empty": rate(
            sum(
                1
                for r in empty_truth
                if r["prediction"]["state"] == "estimated" and r["prediction"]["count"] == 0
            ),
            len(empty_truth),
        ),
        "low_coverage_refused": rate(len(low) - len(low_answered), len(low)),
        "low_coverage_answered_wrong": rate(
            sum(1 for r in low_answered if abs(error(r)) > 1), len(low_answered)
        ),
        "answered_wrong": rate(len(wrong), len(answered_any)),
        "cannot_tell_reasons": dict(sorted(reasons.items())),
        "misplaced_recall": rate(
            sum(1 for r in misplaced_truth if flagged(r)), len(misplaced_truth)
        ),
        "misplaced_recall_rear": rate(sum(1 for r in rear if flagged(r)), len(rear)),
        "misplaced_false_alarm": rate(sum(1 for r in clean if flagged(r)), len(clean)),
        "conforming_but_misplaced": rate(
            sum(1 for r in conforming if r["truth_misplaced"]), len(conforming)
        ),
        "lanes_with_a_named_face": rate(
            sum(1 for r in non_empty if r["prediction"]["named_count"] > 0), len(non_empty)
        ),
        "mean_unverified_share": summary(
            [
                r["prediction"]["unverified_count"] / max(r["prediction"]["count"], 1)
                for r in non_empty
            ]
        ),
    }


def score_stock(
    labels: Sequence[Mapping[str, Any]], predictions: Iterable[Mapping[str, Any]]
) -> dict[str, Any]:
    """Overall metrics plus one block per condition value (``lighting=dim``...)."""
    records = lane_records(labels, predictions)
    grouped: dict[str, list[dict[str, Any]]] = defaultdict(list)
    for record in records:
        for key in CONDITION_KEYS:
            grouped[f"{key}={record['conditions'][key]}"].append(record)
    return {
        "report_version": 1,
        "overall": aggregate(records),
        "by_condition": {name: aggregate(group) for name, group in sorted(grouped.items())},
    }


def format_stock_report(report: Mapping[str, Any]) -> str:
    """A compact text table of the overall block and each condition's count/misplaced numbers."""

    def pct(item: Mapping[str, Any]) -> str:
        if item["value"] is None:
            return "n/a (n=0)"
        low, high = item["ci95"]
        span = f"[{low * 100:4.1f},{high * 100:5.1f}]"
        return f"{item['value'] * 100:5.1f}% {span} k={item['k']}/{item['n']}"

    keys = (
        "answered_on_sufficient",
        "within_one_on_sufficient",
        "within_one_among_answered",
        "exact_among_answered",
        "empty_lane_read_empty",
        "low_coverage_refused",
        "low_coverage_answered_wrong",
        "misplaced_recall",
        "misplaced_recall_rear",
        "misplaced_false_alarm",
        "conforming_but_misplaced",
        "lanes_with_a_named_face",
    )
    lines = []
    for name, block in [("overall", report["overall"]), *report["by_condition"].items()]:
        lines.append(
            f"[{name}] lanes={block['n_lanes']} sufficient={block['n_sufficient_coverage']}"
            f" low={block['n_low_coverage']} cannot-tell={block['cannot_tell_reasons']}"
        )
        for key in keys:
            lines.append(f"    {key:30s} {pct(block[key])}")
    return "\n".join(lines)


# --------------------------------------------------------------------------------------------
# Pre-declared bounds
# --------------------------------------------------------------------------------------------


def stock_held_out_population(splits: Mapping[str, Any], round_number: int = 1) -> list[int]:
    """The CMB-114 held-out seeds: the fresh held-out seeds, ascending, every second one.

    Round 1 took the first, third... fresh seed; round 2 takes the second, fourth... (the seeds
    round 1 read are development data for round 2).
    """
    fresh = sorted(splits["held_out_generations"]["fresh"])
    return fresh[(round_number - 1) % 2 :: 2]


def _verdict(item: Mapping[str, Any], bound: Mapping[str, Any]) -> str:
    """pass / fail / inconclusive for one rate against one bound (the declared pass rule)."""
    if item["n"] == 0 or item["ci95"] is None:
        return "inconclusive"
    point, (low, high) = item["value"], item["ci95"]
    if bound["direction"] == "min":
        if low >= bound["value"] and item["n"] >= bound["required_n"]:
            return "pass"
        return "fail" if point < bound["value"] else "inconclusive"
    if high <= bound["value"] and item["n"] >= bound["required_n"]:
        return "pass"
    return "fail" if point > bound["value"] else "inconclusive"


def evaluate_bounds(
    reports: Mapping[str, Mapping[str, Any]], bounds: Mapping[str, Any]
) -> list[dict[str, Any]]:
    """Apply the declared bounds to score reports keyed by arm label (``abstain``, ``oracle``...).

    A bound whose arm has no report is listed as ``not_run``: nothing is inferred from another arm.
    """
    rows = []
    for bound in bounds["bounds"]:
        for arm in bound["arms"]:
            report = reports.get(arm)
            if report is None:
                rows.append({"bound": bound["id"], "arm": arm, "verdict": "not_run"})
                continue
            # A bound may name a group of lanes (``"group": "product_class=box"``): the same
            # metric on that condition's block, which is empty (n = 0, inconclusive) if the
            # report holds no such lane.
            group = bound.get("group")
            block = report["overall"] if group is None else report["by_condition"].get(group)
            if block is None:
                rows.append({"bound": bound["id"], "arm": arm, "verdict": "inconclusive", "n": 0})
                continue
            item = block[bound["metric"]]
            rows.append(
                {
                    "bound": bound["id"],
                    "arm": arm,
                    "group": group,
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

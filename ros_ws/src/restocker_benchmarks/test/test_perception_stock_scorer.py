# Copyright 2026 Matteo Bernacchia <dev@kikijiki.com>
#
# SPDX-License-Identifier: MIT

"""The lane stock scorer: which lane a survey frame is scored on, and what each rate means."""

import pytest
from restocker_benchmarks import perception_stock_scorer as scorer


def label(frame, lane, count, misplaced_count=0, rear_misplaced=False, lighting="nominal"):
    objects = [
        {
            "lane_id": lane,
            "lane_order": i,
            "misplaced": rear_misplaced and i == count - 1,
            "product_class": "can",
            "object_id": f"{lane}-{i}",
        }
        for i in range(count)
    ]
    other = {"lane_id": "lane_02", "count": 5, "misplaced_count": 0}
    return {
        "frame_id": frame,
        "viewpoint": lane,
        "seed_split": "train",
        "lighting": {"condition": lighting},
        "objects": objects,
        "lanes": [
            {"lane_id": lane, "count": count, "misplaced_count": misplaced_count},
            other,
        ],
    }


def prediction(
    frame,
    lane,
    count=None,
    coverage=1.0,
    misplaced=None,
    composition=None,
    named=0,
    arm="abstain",
    degraded=False,
    reason=0,
):
    return {
        "kind": "lane",
        "frame_id": frame,
        "lane_id": lane,
        "arm": arm,
        "degraded": degraded,
        "state": "estimated" if count is not None else "cannot_tell",
        "reason": reason,
        "count": count,
        "misplaced": misplaced,
        "composition": composition,
        "named_count": named,
        "unverified_count": (count or 0) - named,
        "measured_coverage": coverage,
    }


def test_only_the_lane_the_frame_was_taken_for_is_scored():
    labels = [label("f1", "lane_01", 3)]
    report = scorer.score_stock(labels, [prediction("f1", "lane_01", count=3)])
    assert report["overall"]["n_lanes"] == 1  # lane_02 (an in-view neighbour) is not scored


def test_within_one_counts_a_refusal_on_sufficient_coverage_as_a_miss():
    labels = [label(f"f{i}", "lane_01", 3) for i in range(4)]
    predictions = [
        prediction("f0", "lane_01", count=3),
        prediction("f1", "lane_01", count=4),  # within one
        prediction("f2", "lane_01", count=6),  # wrong
        prediction("f3", "lane_01", count=None, reason=4),  # refused on sufficient coverage
    ]
    block = scorer.score_stock(labels, predictions)["overall"]
    assert (
        block["within_one_on_sufficient"]["k"] == 2 and block["within_one_on_sufficient"]["n"] == 4
    )
    assert block["answered_on_sufficient"]["k"] == 3
    assert (
        block["within_one_among_answered"]["k"] == 2
        and block["within_one_among_answered"]["n"] == 3
    )
    assert block["cannot_tell_reasons"] == {"4": 1}


def test_low_coverage_refusal_is_correct_and_an_answer_there_is_judged_on_its_count():
    labels = [label(f"f{i}", "lane_01", 2) for i in range(3)]
    predictions = [
        prediction("f0", "lane_01", count=None, coverage=0.4, reason=2),
        prediction("f1", "lane_01", count=2, coverage=0.5),
        prediction("f2", "lane_01", count=0, coverage=0.5),
    ]
    block = scorer.score_stock(labels, predictions)["overall"]
    assert block["n_low_coverage"] == 3 and block["n_sufficient_coverage"] == 0
    assert block["low_coverage_refused"]["k"] == 1
    assert (
        block["low_coverage_answered_wrong"]["k"] == 1
        and block["low_coverage_answered_wrong"]["n"] == 2
    )


def test_misplaced_recall_false_alarm_and_the_silent_conforming_error():
    labels = [
        label("m1", "lane_01", 2, misplaced_count=1, rear_misplaced=True),
        label("m2", "lane_01", 2, misplaced_count=1, rear_misplaced=False),
        label("c1", "lane_01", 2),
        label("c2", "lane_01", 2),
    ]
    predictions = [
        prediction("m1", "lane_01", count=2, misplaced=True, composition=3, named=1),
        prediction("m2", "lane_01", count=2, misplaced=False, composition=1, named=2),
        prediction("c1", "lane_01", count=2, misplaced=True, composition=3, named=1),
        prediction("c2", "lane_01", count=2, misplaced=False, composition=0),
    ]
    block = scorer.score_stock(labels, predictions)["overall"]
    assert (block["misplaced_recall"]["k"], block["misplaced_recall"]["n"]) == (1, 2)
    assert (block["misplaced_recall_rear"]["k"], block["misplaced_recall_rear"]["n"]) == (1, 1)
    assert (block["misplaced_false_alarm"]["k"], block["misplaced_false_alarm"]["n"]) == (1, 2)
    assert (block["conforming_but_misplaced"]["k"], block["conforming_but_misplaced"]["n"]) == (
        1,
        1,
    )


def test_conditions_split_by_lighting_and_degradation():
    labels = [label("a", "lane_01", 1, lighting="dim"), label("b", "lane_01", 1, lighting="warm")]
    predictions = [
        prediction("a", "lane_01", count=1),
        prediction("b", "lane_01", count=1, degraded=True),
    ]
    report = scorer.score_stock(labels, predictions)
    assert report["by_condition"]["lighting=dim"]["n_lanes"] == 1
    assert report["by_condition"]["degraded=true"]["n_lanes"] == 1
    assert "lighting=warm" in report["by_condition"]
    assert "within_one_on_sufficient" in scorer.format_stock_report(report)


def test_prediction_for_an_unknown_frame_is_an_error():
    with pytest.raises(ValueError):
        scorer.score_stock([label("a", "lane_01", 1)], [prediction("zz", "lane_01", count=1)])


BOUNDS = {
    "bounds": [
        {
            "id": "count",
            "arms": ["abstain"],
            "metric": "within_one_on_sufficient",
            "direction": "min",
            "value": 0.90,
            "required_n": 50,
        },
        {
            "id": "alarm",
            "arms": ["abstain", "oracle"],
            "metric": "misplaced_false_alarm",
            "direction": "max",
            "value": 0.05,
            "required_n": 50,
        },
    ]
}


def report_with(**rates):
    overall = {
        "within_one_on_sufficient": scorer.rate(1, 1),
        "misplaced_false_alarm": scorer.rate(0, 1),
    }
    overall.update({name: scorer.rate(k, n) for name, (k, n) in rates.items()})
    return {"overall": overall}


def test_min_bound_needs_the_lower_interval_limit_and_the_sample_size():
    rows = scorer.evaluate_bounds(
        {"abstain": report_with(within_one_on_sufficient=(200, 200))}, BOUNDS
    )
    assert rows[0]["verdict"] == "pass"
    small = scorer.evaluate_bounds(
        {"abstain": report_with(within_one_on_sufficient=(30, 30))}, BOUNDS
    )
    assert small[0]["verdict"] == "inconclusive"  # right side, but n below required_n
    wide = scorer.evaluate_bounds(
        {"abstain": report_with(within_one_on_sufficient=(95, 100))}, BOUNDS
    )
    assert wide[0]["verdict"] == "inconclusive"  # point 0.95 over 0.90, lower limit 0.89 under it
    bad = scorer.evaluate_bounds(
        {"abstain": report_with(within_one_on_sufficient=(80, 100))}, BOUNDS
    )
    assert bad[0]["verdict"] == "fail"


def test_max_bound_uses_the_upper_limit_and_a_missing_arm_is_not_run():
    reports = {
        "abstain": report_with(misplaced_false_alarm=(0, 200), within_one_on_sufficient=(1, 1))
    }
    rows = {(r["bound"], r["arm"]): r for r in scorer.evaluate_bounds(reports, BOUNDS)}
    assert rows[("alarm", "abstain")]["verdict"] == "pass"
    assert rows[("alarm", "oracle")]["verdict"] == "not_run"
    worse = scorer.evaluate_bounds(
        {"abstain": report_with(misplaced_false_alarm=(20, 200))}, BOUNDS
    )
    assert [r["verdict"] for r in worse if r["bound"] == "alarm"] == ["fail", "not_run"]


def test_population_is_every_second_fresh_seed():
    splits = {"held_out_generations": {"fresh": [9, 5, 3, 7, 11, 13]}}
    assert scorer.stock_held_out_population(splits) == [3, 7, 11]


def test_answered_wrong_counts_phantoms_that_within_one_hides():
    labels = [label("e", "lane_01", 0), label("n", "lane_01", 3), label("m", "lane_01", 3)]
    predictions = [
        prediction("e", "lane_01", count=1),  # a phantom product in an empty lane
        prediction("n", "lane_01", count=3),
        prediction("m", "lane_01", count=6),  # three off
    ]
    block = scorer.score_stock(labels, predictions)["overall"]
    assert (block["answered_wrong"]["k"], block["answered_wrong"]["n"]) == (2, 3)
    assert block["within_one_on_sufficient"]["k"] == 2  # the phantom is "within one"


def test_round_two_population_is_the_other_half_of_the_fresh_seeds():
    splits = {"held_out_generations": {"fresh": [9, 5, 3, 7, 11, 13]}}
    assert scorer.stock_held_out_population(splits, 1) == [3, 7, 11]
    assert scorer.stock_held_out_population(splits, 2) == [5, 9, 13]


def test_a_bound_can_name_a_group_and_a_missing_group_is_inconclusive():
    block = {"exact_on_sufficient": {"value": 0.9, "ci95": [0.85, 0.95], "k": 27, "n": 30}}
    report = {"overall": block, "by_condition": {"product_class=box": block}}
    bound = {
        "id": "b",
        "arms": ["abstain"],
        "metric": "exact_on_sufficient",
        "direction": "min",
        "value": 0.8,
        "required_n": 20,
        "group": "product_class=box",
    }
    rows = scorer.evaluate_bounds({"abstain": report}, {"bounds": [bound]})
    assert rows[0]["verdict"] == "pass" and rows[0]["group"] == "product_class=box"
    empty = {"overall": block, "by_condition": {}}
    assert scorer.evaluate_bounds({"abstain": empty}, {"bounds": [bound]})[0]["verdict"] == (
        "inconclusive"
    )

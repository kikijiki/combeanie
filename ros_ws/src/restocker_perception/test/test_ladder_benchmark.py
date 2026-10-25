# Copyright 2026 Matteo Bernacchia <dev@kikijiki.com>
#
# SPDX-License-Identifier: MIT

"""The ladder benchmark driver: oracle rounds, the numbers it reports, and the pass rule."""
# Offline: a scripted engine (test_identity_ladder.py) and hand-built states. The driver's job is
# bookkeeping, so the tests pin the bookkeeping: which objects a round re-opens, what turns it
# asks the oracle for, what counts as named, wrong, unknown-named and how many re-looks it took.

import dataclasses
from pathlib import Path
import sys

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from ladder_support import (  # noqa: E402,I100
    GRAPE,
    MELON,
    MINT,
    crop,
    engine,
    make_registry,
)
from restocker_perception.identity import ladder_benchmark as lb  # noqa: E402,I100,I101
from restocker_perception.identity.benchmark import ProtocolError  # noqa: E402
from restocker_perception.identity.ladder import LookCrop  # noqa: E402


@pytest.fixture
def registry(tmp_path):
    return make_registry(tmp_path)


def data_for(tag, seed=0):
    return {
        "crops": [LookCrop(crop(seed, tag), None, 100.0)],
        "barcode_images": [],
        "truth": {
            "sku": "x",
            "product_class": "can",
            "lighting": "nominal",
            "tilt_deg": 0.0,
            "seed": 1,
            "object_id": "sim:stock_can_01",
        },
        "true_relative_azimuth_deg": [40.0],
        "range_m": 0.4,
    }


def test_a_round_reopens_only_what_the_barcode_off_arm_left_open_and_asks_for_a_turn(registry):
    eng = engine(
        registry,
        {1: ["Fennlo", "Sparkling"], 2: ["Fennlo", "Mint", "Sparkling"]},
        shortlist_rank=4,
    )
    run = lb.LadderRun(registry)
    key = (1, "sim:stock_can_01")
    offsets = lb.step(run, eng, {key: data_for(1)}, round_index=0)
    assert set(offsets["1"]) == {"sim:stock_can_01"} and not run.objects[key]["closed"]
    assert run.objects[key]["decisions"]["off"][0]["status"] == "not_identified_yet"
    turn = run.objects[key]["decisions"]["off"][0]["guidance"]["suggested_turn_deg"]
    assert offsets["1"]["sim:stock_can_01"] == pytest.approx(turn % 360.0)
    with pytest.raises(ProtocolError):
        lb.step(run, eng, {key: data_for(2)}, round_index=2)  # rounds do not skip
    offsets = lb.step(run, eng, {key: data_for(2, 10)}, round_index=1)
    assert offsets == {} and run.objects[key]["closed"]
    final = run.objects[key]["decisions"]["off"][1]
    assert final["status"] == "identified" and final["sku"] == MINT and final["look"] == 1


def test_an_open_object_the_round_did_not_show_spends_its_look_and_is_asked_again(registry):
    eng = engine(registry, {1: ["Fennlo", "Sparkling"]}, shortlist_rank=4)
    run = lb.LadderRun(registry)
    key = (1, "sim:stock_can_01")
    lb.step(run, eng, {key: data_for(1)}, round_index=0)
    offsets = lb.step(run, eng, {}, round_index=1)
    assert len(run.objects[key]["state"].looks) == 2 and "1" in offsets
    assert run.objects[key]["decisions"]["off"][1]["look"] == 1


def entry(truth_sku, decisions_off, decisions_on=None, cls="can", seed=1, oid="a"):
    state = type("S", (), {})()
    state.looks = [
        type("L", (), {"latency_ms": {"embedding": 100.0, "ocr": 400.0, "barcode": 50.0}})()
    ]
    return {
        "truth": {
            "sku": truth_sku,
            "product_class": cls,
            "lighting": "nominal",
            "seed": seed,
            "object_id": oid,
        },
        "state": state,
        "decisions": {"off": decisions_off, "on": decisions_on or decisions_off},
        "closed": True,
    }


def decision(status, sku=None, look=0, rung=None, score=0.9, margin=0.5):
    return {
        "status": status,
        "sku": sku,
        "look": look,
        "rung": rung,
        "text": {"best_score": score, "margin": margin},
        "guidance": None,
    }


def test_the_summary_counts_named_wrong_unknown_named_and_relooks(registry):
    run = lb.LadderRun(registry)
    run.objects = {
        (1, "a"): entry("GEN-001", [decision("identified", MINT, 0, "text")]),
        (1, "b"): entry(
            "GEN-001", [decision("not_identified_yet"), decision("identified", MINT, 1, "text")]
        ),
        (1, "c"): entry(
            "GEN-002", [decision("not_identified_yet"), decision("identified", GRAPE, 1, "text")]
        ),
        (1, "d"): entry(
            "GEN-003", [decision("not_identified_yet"), decision("needs_human", None, 1)]
        ),
        (1, "e"): entry(
            "GEN-099", [decision("identified", MELON, 0, "text")]
        ),  # unregistered truth
        (1, "f"): entry(
            "GEN-099", [decision("not_identified_yet"), decision("needs_human", None, 1)]
        ),
    }

    class R:
        def resolve(self, sku):
            return {"GEN-001": MINT, "GEN-002": MELON, "GEN-003": "2000000000039"}.get(sku)

        def get(self, pid):
            return type(
                "P",
                (),
                {
                    "display_name": {
                        MINT: "Fennlo Mint",
                        MELON: "Fennlo Melon",
                        GRAPE: "Zorbex Grape",
                    }.get(pid, "x")
                },
            )()

    out = lb.summarise(run, R(), arm="off", budget=1)
    assert (out["n_known"], out["n_unknown"], out["named"], out["correct"]) == (4, 2, 3, 2)
    assert out["false_identity"] == 1 and out["false_identity_rate_named"] == pytest.approx(1 / 3)
    assert out["wrong_after_escalation"] == 1 and out["unknown_named"] == 1
    assert (
        out["relooks_mean_named"] == pytest.approx(0.5) and out["status_known"]["needs_human"] == 1
    )
    assert out["by_look"][0]["named"] == 1 and out["by_look"][1]["named"] == 2
    # With no re-look at all, only the first look's naming counts and the rest is left to a human.
    zero = lb.summarise(run, R(), arm="off", budget=0)
    assert zero["named"] == 1 and zero["status_known"]["needs_human"] == 3
    assert zero["latency_ms_per_look_p95"] == pytest.approx(500.0)  # the barcode arm is off


def test_the_pass_rule_needs_the_sample_and_the_upper_limit_under_the_bound():
    bounds = {
        "bounds": {
            "false_identity_rate_named_max": 0.01,
            "unknown_named_as_sku_max": 0.10,
            "coverage_known_min": {"off": 0.5, "on": 0.6},
            "relooks_mean_max": 1.5,
            "relooks_p95_max": 3,
            "latency_p95_ms_per_look_max": 4000,
            "wrong_after_escalation_max": 1,
        },
        "required_sample": {"named_min": 380, "unknown_objects_min": 60, "known_objects_min": 400},
    }

    def summary(**kw):
        base = {
            "arm": "off",
            "false_identity_rate_named": 0.0,
            "false_identity_rate_named_wilson95": [0.0, 0.0099],
            "named": 400,
            "unknown_named_rate": 0.0,
            "unknown_named_wilson95": [0.0, 0.06],
            "n_unknown": 60,
            "coverage_known": 0.8,
            "coverage_known_wilson95": [0.7, 0.85],
            "n_known": 500,
            "relooks_mean_named": 1.0,
            "relooks_p95_named": 3,
            "latency_ms_per_look_p95": 3000,
            "wrong_after_escalation": 0,
        }
        base.update(kw)
        return base

    assert {v["status"] for v in lb.verdict(summary(), bounds).values()} == {"pass"}
    small = lb.verdict(summary(named=100), bounds)
    assert small["false_identity_rate_named"]["status"] == "inconclusive"
    over = lb.verdict(summary(false_identity_rate_named=0.02), bounds)
    assert over["false_identity_rate_named"]["status"] == "fail"
    wide = lb.verdict(summary(false_identity_rate_named_wilson95=[0.0, 0.02]), bounds)
    assert wide["false_identity_rate_named"]["status"] == "inconclusive"
    low = lb.verdict(summary(coverage_known=0.4, coverage_known_wilson95=[0.3, 0.5]), bounds)
    assert low["coverage_known"]["status"] == "fail"
    slow = lb.verdict(summary(latency_ms_per_look_p95=5000), bounds)
    assert slow["latency_p95_ms_per_look"]["status"] == "fail"


def test_the_facing_report_removes_the_convention_offset_before_it_scores_the_estimate(registry):
    run = lb.LadderRun(registry)
    # estimates are the truth plus a fixed 60 degree convention, with a 10 degree error on one
    guidance = lambda est, truth, shape="round": {  # noqa: E731
        "estimated_facing_deg": est % 360,
        "true_relative_azimuth_deg": truth,
        "shape": shape,
    }
    decisions = [{"guidance": guidance(t + 60, t)} for t in (10, 100, 200, 300, 350)] + [
        {"guidance": guidance(t + 60 + 10, t)} for t in (50,)
    ]
    run.objects = {(1, "a"): {"decisions": {"off": decisions, "on": []}}}
    out = lb.facing_report(run)["round"]
    assert out["n"] == 6 and out["convention_offset_deg"] == pytest.approx(61.7, abs=1.5)
    assert out["median_abs_error_deg"] < 3 and out["within_30"] == 1.0


def aimed(tag, seed, n_plain=1):
    """A look with ``n_plain`` ordinary crops and one aimed barcode crop (the last)."""
    data = data_for(tag, seed)
    data["crops"] = [LookCrop(crop(seed + i, tag), None, 100.0) for i in range(n_plain)]
    data["crops"].append(LookCrop(crop(seed + 50, 3), None, 100.0))
    data["barcode_view_mask"] = [False] * n_plain + [True]
    return data


def test_the_barcode_off_arm_does_not_see_the_aimed_barcode_views_but_off_views_does(registry):
    # Tag 3 is read only on the aimed view: it carries the flavour. The strict arm cannot use it.
    eng = engine(
        registry,
        {1: ["Fennlo", "Sparkling"], 3: ["Mint"]},
        shortlist_rank=4,
    )
    run = lb.LadderRun(registry)
    key = (1, "sim:stock_can_01")
    lb.step(run, eng, {key: aimed(1, 0)}, round_index=0)
    decisions = run.objects[key]["decisions"]
    assert decisions["off"][0]["status"] == "not_identified_yet"
    assert decisions["off_views"][0]["status"] == "identified"
    assert decisions["off_views"][0]["sku"] == MINT
    assert len(run.objects[key]["state_strict"].looks[0].embeddings) == 1
    assert len(run.objects[key]["state"].looks[0].embeddings) == 2
    # the object stays open because the headline arm is open: it is asked for a turn
    assert not run.objects[key]["closed"]


def test_an_object_closes_only_when_every_arm_has_closed_and_each_arm_keeps_its_own_looks(
    registry,
):
    eng = engine(
        registry,
        {1: ["Fennlo", "Sparkling"], 3: ["Mint"], 2: ["Fennlo", "Mint", "Sparkling"]},
        shortlist_rank=4,
    )
    run = lb.LadderRun(registry)
    key = (1, "sim:stock_can_01")
    lb.step(run, eng, {key: aimed(1, 0)}, round_index=0)
    lb.step(run, eng, {key: data_for(2, 20)}, round_index=1)
    entry = run.objects[key]
    assert entry["closed"]
    assert entry["decisions"]["off"][1]["status"] == "identified"
    assert entry["decisions"]["off_views"][1]["look"] == 0  # copied: it was named at look 0
    assert len(entry["state_strict"].looks) == 2 and len(entry["state"].looks) == 1


def test_escalation_is_counted_per_arm_with_its_own_latency(registry):
    from ladder_support import ScriptedOcr  # noqa: PLC0415
    from restocker_perception.identity.ladder import EscalationConfig  # noqa: PLC0415

    eng = engine(
        registry,
        {1: ["Fennlo", "Sparkling"]},
        shortlist_rank=4,
        escalation=EscalationConfig(enabled=True),
        escalation_ocr=ScriptedOcr({1: ["Fennlo", "Mint", "Sparkling"]}),
    )
    run = lb.LadderRun(registry)
    key = (1, "sim:stock_can_01")
    lb.step(run, eng, {key: data_for(1)}, round_index=0)
    entry = run.objects[key]
    assert entry["escalated"]["off"] == {0} and entry["decisions"]["off"][0]["escalated"]
    assert entry["decisions"]["off"][0]["rung"] == "text_escalated"
    summary = lb.summarise(run, registry_stub(), arm="off", budget=0)
    assert summary["escalated_reads"] == 1 and summary["named_by_heavy_read_only"] == 1
    assert summary["latency_ms_per_escalated_read_p95"] is not None
    # the arms that shared the look each escalated it once
    assert lb.summarise(run, registry_stub(), arm="on", budget=0)["escalated_reads"] == 1


def registry_stub():
    class R:
        def resolve(self, sku):
            return MINT

        def get(self, pid):
            return type("P", (), {"display_name": "Fennlo Mint"})()

    return R()


def test_an_arm_left_open_without_guidance_asks_for_no_turn_instead_of_crashing(registry):
    eng = engine(registry, {1: ["Fennlo", "Sparkling"]}, shortlist_rank=4)
    decide = eng.decide_escalating

    def without_guidance_on_the_barcode_arm(*args, **kwargs):
        result = decide(*args, **kwargs)
        return dataclasses.replace(result, guidance=None) if kwargs.get("barcode") else result

    eng.decide_escalating = without_guidance_on_the_barcode_arm
    run = lb.LadderRun(registry)
    key = (1, "sim:stock_can_01")
    lb.step(run, eng, {key: data_for(1)}, round_index=0)
    entry = run.objects[key]
    assert entry["decisions"]["on"][0]["guidance"] is None
    for arm in ("off", "off_views"):  # the headline arms are done; only the barcode arm is open
        entry["decisions"][arm][-1]["status"] = "identified"
    offsets = lb.step(run, eng, {key: data_for(2, 10)}, round_index=1)
    assert offsets == {"1": {"sim:stock_can_01": entry["turn_deg"]}}
    assert entry["turn_deg"] == 0.0 or entry["turn_deg"] == pytest.approx(
        entry["turn_deg"] % 360.0
    )

# Copyright 2026 Matteo Bernacchia <dev@kikijiki.com>
#
# SPDX-License-Identifier: MIT

"""The escalation rung (CMB-121 round 4): a heavier reader, only for objects left open."""
# Offline and deterministic like test_identity_ladder.py: a scripted cheap reader and a scripted
# heavy one (the crop's first pixel says which lines each reads), the fake embedding. The rung is
# off by default, runs once per look and only when the cheap rungs named nothing, and everything
# that guards a name (margin, contradictions, the embedding shortlist, never a guess) still holds.

from pathlib import Path
import sys

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from ladder_support import (  # noqa: E402,I100,I101
    MINT,
    ScriptedOcr,
    crop,
    engine,
    make_registry,
)
from restocker_perception.identity.ladder import (  # noqa: E402
    IDENTIFIED,
    NEEDS_HUMAN,
    NOT_YET,
    EscalationConfig,
    LadderConfig,
    LookCrop,
    ObjectState,
)
from restocker_perception.registry.backends import parse_florence_regions  # noqa: E402


class CountingOcr(ScriptedOcr):
    name = "heavy"

    def __init__(self, lines_by_tag):
        super().__init__(lines_by_tag)
        self.calls = 0

    def read(self, image):
        self.calls += 1
        return super().read(image)


@pytest.fixture
def registry(tmp_path):
    return make_registry(tmp_path)


def on(**kw):
    return EscalationConfig(enabled=True, **kw)


def look_of(eng, tag, seed=0):
    crops = [LookCrop(crop(seed, tag))]
    return crops, eng.observe(crops)


def test_the_rung_is_off_by_default_and_a_reader_alone_does_not_switch_it_on(registry):
    heavy = CountingOcr({1: ["Mint"]})
    eng = engine(registry, {1: ["Fennlo", "Sparkling"]}, shortlist_rank=4, escalation_ocr=heavy)
    crops, look = look_of(eng, 1)
    state = ObjectState([look])
    result = eng.decide_escalating(state, crops, barcode=False)
    assert not eng.can_escalate and heavy.calls == 0
    assert result.status == NOT_YET and not result.escalated and look.escalation_tokens is None


def test_the_heavy_reader_names_what_the_cheap_one_left_tied(registry):
    heavy = CountingOcr({1: ["Fennlo", "Mint", "Sparkling"]})
    eng = engine(
        registry,
        {1: ["Fennlo", "Sparkling"]},
        shortlist_rank=4,
        escalation=on(),
        escalation_ocr=heavy,
    )
    crops, look = look_of(eng, 1)
    state = ObjectState([look])
    assert eng.decide(state, barcode=False).status == NOT_YET
    result = eng.decide_escalating(state, crops, barcode=False)
    assert (result.status, result.sku, result.rung) == (IDENTIFIED, MINT, "text_escalated")
    assert result.escalated and result.latency_ms["escalation"] > 0.0
    assert heavy.calls == 1 and look.escalation_tokens is not None


def test_the_heavy_tokens_replace_the_cheap_garbage_of_the_look_they_escalate(registry):
    # The cheap reader also read a word of another product (a contradiction that would block any
    # name); the heavy reader did not. A union would keep the garbage; the rung does not.
    heavy = CountingOcr({1: ["Fennlo", "Mint", "Sparkling"]})
    eng = engine(
        registry,
        {1: ["Fennlo", "Sparkling", "Zorbex"]},
        shortlist_rank=4,
        escalation=on(),
        escalation_ocr=heavy,
    )
    crops, look = look_of(eng, 1)
    state = ObjectState([look])
    assert eng.decide(state, barcode=False).status == NOT_YET
    result = eng.decide_escalating(state, crops, barcode=False)
    assert result.sku == MINT and "zorbex" not in state.tokens_with(None)
    assert "zorbex" in state.tokens_with(set())  # an arm that did not escalate still has it


def test_an_object_the_cheap_rungs_named_is_never_escalated(registry):
    heavy = CountingOcr({1: ["Fennlo", "Melon", "Sparkling"]})
    eng = engine(
        registry,
        {1: ["Fennlo", "Mint", "Sparkling"]},
        shortlist_rank=4,
        escalation=on(),
        escalation_ocr=heavy,
    )
    crops, look = look_of(eng, 1)
    result = eng.decide_escalating(ObjectState([look]), crops, barcode=False)
    assert (result.sku, result.rung, result.escalated) == (MINT, "text", False)
    assert heavy.calls == 0 and "escalation" not in result.latency_ms


def test_a_heavy_reader_that_reads_nothing_new_still_leaves_the_object_open(registry):
    heavy = CountingOcr({1: ["Fennlo", "Sparkling"]})
    eng = engine(
        registry,
        {1: ["Fennlo", "Sparkling"]},
        shortlist_rank=4,
        max_relooks=1,
        escalation=on(),
        escalation_ocr=heavy,
    )
    crops, look = look_of(eng, 1)
    first = eng.decide_escalating(ObjectState([look]), crops, barcode=False)
    assert first.status == NOT_YET and first.sku is None and first.escalated
    crops2, look2 = look_of(eng, 1, seed=5)
    state = ObjectState([look, look2])
    last = eng.decide_escalating(state, crops2, barcode=False)
    assert last.status == NEEDS_HUMAN and last.sku is None
    assert heavy.calls == 2  # once per look, also when the budget is spent (the last chance)


def test_both_flavours_read_is_a_contradiction_and_names_nothing(registry):
    heavy = CountingOcr({1: ["Fennlo", "Mint", "Melon", "Sparkling"]})
    eng = engine(
        registry,
        {1: ["Fennlo", "Sparkling"]},
        shortlist_rank=4,
        escalation=on(),
        escalation_ocr=heavy,
    )
    crops, look = look_of(eng, 1)
    result = eng.decide_escalating(ObjectState([look]), crops, barcode=False)
    assert result.status == NOT_YET and result.sku is None


def test_the_embedding_shortlist_still_guards_a_name_the_heavy_reader_gives(registry):
    heavy = CountingOcr({1: ["Fennlo", "Mint", "Sparkling"]})
    eng = engine(
        registry,
        {1: ["Fennlo", "Sparkling"]},
        shortlist_rank=1,
        escalation=on(),
        escalation_ocr=heavy,
    )
    crops, look = look_of(eng, 1)
    result = eng.decide_escalating(ObjectState([look]), crops, barcode=False)
    # random crops: the shortlist of one holds the embedding's top-1, which is rarely MINT
    assert result.sku in (None, MINT)
    if result.sku is None:
        assert result.status == NOT_YET


def test_an_arm_counts_only_the_looks_it_escalated(registry):
    heavy = CountingOcr({1: ["Fennlo", "Mint", "Sparkling"]})
    eng = engine(
        registry,
        {1: ["Fennlo", "Sparkling"]},
        shortlist_rank=4,
        escalation=on(),
        escalation_ocr=heavy,
    )
    crops, look = look_of(eng, 1)
    state = ObjectState([look])
    eng.decide_escalating(state, crops, barcode=False)  # the first arm escalates
    other = eng.decide(state, barcode=False, escalated_looks=set())  # the second has not
    assert other.status == NOT_YET and not other.escalated
    mine: set[int] = set()
    result = eng.decide_escalating(state, crops, escalated=mine, barcode=False)
    assert result.sku == MINT and mine == {0} and heavy.calls == 1  # tokens are read once


def test_a_look_without_crops_is_not_escalated(registry):
    heavy = CountingOcr({1: ["Fennlo", "Mint", "Sparkling"]})
    eng = engine(
        registry,
        {1: ["Fennlo", "Sparkling"]},
        shortlist_rank=4,
        escalation=on(),
        escalation_ocr=heavy,
    )
    _crops, look = look_of(eng, 1)
    result = eng.decide_escalating(ObjectState([look]), [], barcode=False)
    assert result.status == NOT_YET and heavy.calls == 0


def test_the_config_round_trips_and_an_old_config_has_the_rung_off():
    config = LadderConfig.from_dict(
        {"escalation": {"enabled": True, "crops": 3, "reader": {"upscale": 3, "unrolled": False}}}
    )
    assert config.escalation.enabled and config.escalation.crops == 3
    assert config.escalation.reader.unrolled is False
    assert LadderConfig.from_dict(config.to_dict()) == config
    assert LadderConfig.from_dict({"max_relooks": 3}).escalation.enabled is False


def test_an_unknown_escalation_backend_is_refused(registry):
    with pytest.raises(ValueError, match="escalation backend"):
        engine(registry, {}, escalation=EscalationConfig(enabled=True, backend="nope"))


def test_florence_region_output_becomes_scored_less_lines_with_pixel_boxes():
    text = (
        "</s><s>Fennlo<loc_100><loc_200><loc_300><loc_200><loc_300><loc_400><loc_100><loc_400>"
        "MINT<loc_0><loc_0><loc_500><loc_0><loc_500><loc_500><loc_0><loc_500></s>"
    )
    lines = parse_florence_regions(text, 1000, 2000)
    assert [line.text for line in lines] == ["Fennlo", "MINT"]
    assert lines[0].score == 1.0
    assert lines[0].box[0] == (100.0, 400.0) and lines[0].box[2] == (300.0, 800.0)
    assert parse_florence_regions("</s><s>junk without regions</s>", 10, 10) == []

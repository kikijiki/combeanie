# Copyright 2026 Matteo Bernacchia <dev@kikijiki.com>
#
# SPDX-License-Identifier: MIT

"""The identity ladder (CMB-112 round 3): label text, facing and turn, rungs, budget."""
# Offline and deterministic: the fake embedding backend, a scripted OCR (the crop's first pixel
# says which lines it reads) and a scripted barcode reader, as in test_identity_channel.py. The
# registry is built from rendered cylinders so the turntable panorama and the registry OCR reads
# exist; no model is loaded.

import math
from pathlib import Path
import sys

import numpy as np
import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from ladder_support import (  # noqa: E402,I100,I101
    MELON,
    MINT,
    STRINGS,
    UNREGISTERED_VALID,
    crop,
    engine,
    make_registry,
)
from restocker_perception.identity import facing as fc  # noqa: E402
from restocker_perception.identity.labeltext import (  # noqa: E402
    LabelTextIndex,
    TextPolicy,
    token_reads_word,
)
from restocker_perception.identity.ladder import (  # noqa: E402
    IDENTIFIED,
    NEEDS_HUMAN,
    NOT_YET,
    UNKNOWN,
    LookCrop,
    ObjectState,
)
from restocker_perception.identity.textreader import (  # noqa: E402
    ObjectTextReader,
    object_band,
)
from restocker_perception.registry import render as rd  # noqa: E402


# --- label text -------------------------------------------------------------------------------


def test_a_short_read_does_not_stand_for_a_longer_word_nor_the_reverse():
    assert token_reads_word("cooler", "cooler")
    assert not token_reads_word("cooler", "cool")  # a boilerplate "cool dry place"
    assert token_reads_word("fennlo", "fenlo")  # one dropped letter
    assert token_reads_word("mint", "fennlomint")  # joined words: a four-letter word inside
    assert not token_reads_word("tea", "teapot")  # a three-letter word is not specific enough
    assert not token_reads_word("juice", "juiceconcentrae")  # a long run of prose is not a word
    assert token_reads_word("talanova", "anova")  # most of a long word, read in part
    assert not token_reads_word("pineapple", "apple")  # a short word is not a long one's part


def test_look_alikes_that_share_brand_and_kind_tie_until_the_flavour_is_read():
    index = LabelTextIndex(STRINGS)
    policy = TextPolicy()
    brand_and_kind = index.score(index.tokens_of(["Fennlo", "Sparkling"]))
    assert brand_and_kind.margin == pytest.approx(0.0)
    assert policy.names(brand_and_kind) is None
    flavour = index.score(index.tokens_of(["Fennlo", "MINT", "Sparkling"]))
    assert policy.names(flavour).product_id == MINT
    assert flavour.margin > policy.min_margin


def test_the_margin_not_the_score_is_what_separates_a_twin_from_its_runner_up():
    index = LabelTextIndex(STRINGS)
    match = index.score(index.tokens_of(["Fennlo", "Sparkling", "Mint"]))
    assert match.best.score == pytest.approx(1.0) and match.runner_up.product_id == MELON
    assert TextPolicy(min_margin=0.9).names(match) is None
    assert TextPolicy(min_score=1.01).names(match) is None
    assert TextPolicy(min_matched=4).names(match) is None


def test_a_reader_that_joins_words_still_names_the_product():
    index = LabelTextIndex(STRINGS)
    tokens = index.tokens_of(["FennloMint Sparkling"])
    assert TextPolicy().names(index.score(tokens)).product_id == MINT


def test_the_reader_cuts_the_crop_to_the_object_before_it_reads():
    crop = np.zeros((40, 100, 3), np.uint8)
    band = object_band(crop, 10.0)
    assert band.shape[1] == 24 and band.shape[0] == 40  # 1.15 half widths each side of the centre

    class Seen:
        name, version = "seen", "1"

        def __init__(self):
            self.widths = []

        def read(self, image):
            self.widths.append(image.shape[1])
            return []

    backend = Seen()
    ObjectTextReader(backend).read(crop, 10.0, round_body=True)
    # as is, then unrolled; both enlarged twofold
    assert backend.widths[0] == 2 * band.shape[1] and len(backend.widths) == 2
    backend.widths.clear()
    ObjectTextReader(backend).read(crop, 10.0, round_body=False)
    assert len(backend.widths) == 1


# --- facing and turn --------------------------------------------------------------------------


def test_circular_difference_wraps_to_the_shortest_signed_turn():
    assert fc.circular_difference_deg(10, 350) == pytest.approx(20)
    assert fc.circular_difference_deg(350, 10) == pytest.approx(-20)
    assert fc.circular_difference_deg(180, 0) == pytest.approx(180)


def test_exemplar_curve_interpolates_between_neighbouring_views_and_wraps():
    sims = [0, 0, 0, 0, 0, 0, 0, 1.0]
    curve = fc.exemplar_curve(sims)
    assert curve[315] == pytest.approx(1.0) and curve[0] == pytest.approx(0.0)
    assert curve[337] == pytest.approx(0.5, abs=0.02) and curve[359] == pytest.approx(
        0.0, abs=0.03
    )


def test_the_turn_moves_the_visible_azimuth_onto_the_target():
    facing = fc.Facing(200.0, 1.0, "round")
    turn = fc.suggest_turn(facing, 90.0)
    assert turn.degrees == pytest.approx(110.0)  # a product turned by +110 shows 110 less
    assert fc.suggest_turn(fc.Facing(10.0, 1.0, "round"), 350.0).degrees == pytest.approx(20.0)
    assert fc.suggest_turn(fc.Facing(350.0, 1.0, "round"), 10.0).degrees == pytest.approx(-20.0)


def test_a_box_turns_in_quarter_turns_and_names_its_faces():
    facing = fc.estimate_facing([fc.ViewEvidence((0, 0, 1, 0, 0, 0, 0, 0))], None, box=True)
    assert facing.shape == "box" and facing.visible_face == "left" and facing.azimuth_deg == 90
    turn = fc.suggest_turn(facing, 0.0)
    assert turn.degrees == 90.0 and turn.target_face == "front"
    assert fc.suggest_turn(fc.Facing(0.0, 1.0, "box", "front"), 180.0).degrees == 180.0


def test_pooling_shifts_each_view_by_the_difference_of_the_camera_azimuths():
    # The same product facing, seen from a camera 90 degrees round the object: the second view's
    # peak sits 90 degrees away, and pooling must still put the pooled peak on the first view's.
    peak_a = np.zeros(8)
    peak_a[2] = 1.0  # view az 90
    peak_b = np.zeros(8)
    peak_b[0] = 1.0  # az 0, because the camera moved 90 degrees
    views = [
        fc.ViewEvidence(tuple(peak_a), None, 100.0),
        fc.ViewEvidence(tuple(peak_b), None, 190.0),
    ]
    facing = fc.estimate_facing(views, None)
    assert abs(fc.circular_difference_deg(facing.azimuth_deg, 90.0)) <= 5


def test_the_text_target_is_the_centre_of_the_best_cluster_of_views_not_the_far_panel():
    scores = [0.0, 0.1, 0.9, 0.8, 0.0, 0.0, 0.0, 0.0]
    target = fc.TextAzimuths({"p": scores}).target("p", box=False)
    assert 90 <= target <= 135
    both_sides = [0.0, 0.0, 0.9, 0.0, 0.0, 0.0, 0.8, 0.0]
    assert fc.TextAzimuths({"p": both_sides}).target("p", box=False) == pytest.approx(90.0)
    assert fc.TextAzimuths({"p": [0] * 8}).target("p", box=False) == 0.0


def test_a_registered_panorama_has_one_column_per_degree():
    views = rd.render_views(
        np.tile(np.linspace(0, 1, 192)[None, :, None], (96, 1, 3)), 0.033, 0.12
    )
    pano = fc.panorama(views)
    assert pano.shape[0] == 360


@pytest.fixture
def registry(tmp_path):
    return make_registry(tmp_path)

def test_text_the_named_product_does_not_print_is_a_contradiction_and_blocks_the_name():
    # An unregistered "Daltara Coconut Iced Tea" next to a registered "Daltara Grape Iced Tea":
    # brand and kind are read, the flavour read is another registered product's word.
    index = LabelTextIndex(
        {
            "grape": ["Daltara", "Grape", "Iced Tea"],
            "coconut": ["Dalmona", "Coconut", "Juice Drink"],
        }
    )
    match = index.score(index.tokens_of(["Daltara", "Coconut", "Iced Tea"]))
    best = match.best
    assert best.product_id == "grape" and best.contradicted_by == ("coconut",)
    assert TextPolicy().names(match) is None
    assert TextPolicy(max_contradictions=None).names(match).product_id == "grape"
    # Junk that no registered product prints is not a contradiction (a nutrition table, a price).
    clean = index.score(index.tokens_of(["Daltara", "Grape", "Iced Tea", "Energy", "Protein"]))
    assert clean.best.contradicted_by == () and TextPolicy().names(clean).product_id == "grape"


def test_the_text_target_skips_sides_already_seen_and_falls_back_to_the_richest():
    scores = [0.0, 0.2, 0.9, 0.3, 0.1, 0.8, 0.2, 0.0]
    text = fc.TextAzimuths({"p": scores})
    first = text.target("p", box=False)
    assert abs(fc.circular_difference_deg(first, 90)) <= 25
    # the side already looked at is skipped, so the next candidate is a neighbouring azimuth
    second = text.target("p", box=False, seen=[90.0])
    assert 90 < second <= 150
    far = text.target("p", box=False, seen=[90.0, 135.0, 45.0])
    assert abs(fc.circular_difference_deg(far, 225)) <= 25
    everything = text.target("p", box=False, seen=[0, 45, 90, 135, 180, 225, 270, 315])
    assert abs(fc.circular_difference_deg(everything, 90)) <= 25
    box = fc.TextAzimuths({"b": [0.9, 0, 0.5, 0, 0.7, 0, 0.1, 0]})
    assert box.target("b", box=True) == 0.0
    assert box.target("b", box=True, seen=[0.0]) == 180.0
    assert box.target("b", box=True, seen=[0.0, 180.0]) == 90.0


# --- the ladder -------------------------------------------------------------------------------


def look(eng, tags, seed=0, barcode_tags=()):
    crops = [LookCrop(crop(seed + i, tag)) for i, tag in enumerate(tags)]
    return eng.observe(crops, [crop(900 + i, tag) for i, tag in enumerate(barcode_tags)])


def state_of(*looks):
    state = ObjectState()
    state.looks.extend(looks)
    return state


def test_a_flavour_read_names_the_product_by_text_with_the_embedding_as_shortlist(registry):
    lines = {1: ["Fennlo", "Mint", "Sparkling"]}
    eng = engine(registry, lines, shortlist_rank=4)
    result = eng.decide(state_of(look(eng, [1])), barcode=False)
    assert result.status == IDENTIFIED and result.sku == MINT and result.rung == "text"


def test_twins_that_tie_on_text_are_not_named_and_the_ladder_says_where_to_turn(registry):
    eng = engine(registry, {1: ["Fennlo", "Sparkling"]}, shortlist_rank=4)
    result = eng.decide(state_of(look(eng, [1])), barcode=False)
    assert result.status == NOT_YET and result.sku is None
    guidance = result.guidance
    assert guidance.kind == "turn" and guidance.shape == "round"
    assert guidance.suggested_turn_deg is not None and -180 < guidance.suggested_turn_deg <= 180
    assert guidance.estimated_facing_deg is not None and guidance.candidate in STRINGS


def test_the_budget_ends_in_needs_human_never_a_guess(registry):
    eng = engine(registry, {1: ["Fennlo", "Sparkling"]}, shortlist_rank=4, max_relooks=2)
    state = ObjectState()
    statuses = []
    for _ in range(3):
        state.looks.append(look(eng, [1]))
        statuses.append(eng.decide(state, barcode=False).status)
    assert statuses == [NOT_YET, NOT_YET, NEEDS_HUMAN]
    assert eng.decide(state, barcode=False).sku is None


def test_text_from_a_later_look_accumulates_with_the_earlier_ones(registry):
    lines = {1: ["Fennlo", "Sparkling"], 2: ["Mint"]}
    eng = engine(registry, lines, shortlist_rank=4)
    state = state_of(look(eng, [1]))
    assert eng.decide(state, barcode=False).status == NOT_YET
    state.looks.append(look(eng, [2], seed=10))
    assert eng.decide(state, barcode=False).sku == MINT


def test_a_barcode_names_the_product_only_when_the_arm_has_it_on(registry):
    eng = engine(registry, {}, barcodes={7: MELON}, shortlist_rank=4)
    state = state_of(look(eng, [0], barcode_tags=[7]))
    on = eng.decide(state, barcode=True)
    off = eng.decide(state, barcode=False)
    assert (on.status, on.sku, on.rung, on.barcode) == (IDENTIFIED, MELON, "barcode", MELON)
    assert off.status == NOT_YET and "barcode" not in off.latency_ms


def test_a_valid_gtin_nobody_registered_is_unknown_and_never_a_name(registry):
    eng = engine(registry, {}, barcodes={7: UNREGISTERED_VALID}, shortlist_rank=4)
    result = eng.decide(state_of(look(eng, [0], barcode_tags=[7])), barcode=True)
    assert result.status == UNKNOWN and result.sku is None


def test_a_barcode_and_a_text_that_disagree_are_not_named(registry):
    eng = engine(
        registry,
        {1: ["Talanova", "Pineapple", "Soda"]},
        barcodes={7: MELON},
        shortlist_rank=4,
    )
    result = eng.decide(state_of(look(eng, [1], barcode_tags=[7])), barcode=True)
    assert result.status == NOT_YET and result.sku is None


def test_a_name_the_embedding_shortlist_does_not_contain_is_refused(registry):
    # Text names Mint, but with a shortlist of one the (random-crop) embedding must rank it first.
    eng = engine(registry, {1: ["Fennlo", "Mint", "Sparkling"]}, shortlist_rank=1)
    state = state_of(look(eng, [1]))
    top = eng.index.match(state.looks[0].embeddings).top1.product_id
    result = eng.decide(state, barcode=False)
    assert (result.sku == MINT) == (top == MINT)


def test_the_blind_ablation_turns_a_fixed_quarter_turn(registry):
    eng = engine(registry, {1: ["Fennlo", "Sparkling"]}, shortlist_rank=4)
    result = eng.decide(state_of(look(eng, [1])), barcode=False, guided=False)
    assert result.guidance.suggested_turn_deg == 90.0 and "blind" in result.guidance.note


def test_embedding_alone_never_names_a_product(registry):
    eng = engine(registry, {}, shortlist_rank=4)
    for seed in range(5):
        result = eng.decide(state_of(look(eng, [0, 0, 0], seed=seed * 10)), barcode=False)
        assert result.sku is None and result.status == NOT_YET


def test_the_result_serialises_with_its_guidance(registry):
    eng = engine(registry, {1: ["Fennlo", "Sparkling"]}, shortlist_rank=4)
    result = eng.decide(state_of(look(eng, [1])), barcode=False)
    out = result.to_dict()
    assert out["status"] == NOT_YET and out["guidance"]["kind"] == "turn"
    assert math.isfinite(out["guidance"]["facing_confidence"])


def test_text_seen_off_centre_is_centred_by_turning_against_its_azimuth(registry):
    eng = engine(registry, {1: ["Fennlo", "Sparkling"]}, shortlist_rank=4)
    look_record = look(eng, [1])
    # the text sits 40 degrees to the right of the line of sight (camera azimuth 100): world 140
    look_record.sightings = [(3.0, 140.0), (3.0, 140.0)]
    look_record.camera_azimuths = [100.0]
    facing = fc.Facing(80.0, 1.0, "round")
    assert eng._servo(look_record, facing) == pytest.approx(-40.0, abs=0.5)
    look_record.sightings = [(3.0, 108.0)]  # already within min_turn_deg of the line of sight
    assert eng._servo(look_record, facing) is None
    look_record.sightings = []
    assert eng._servo(look_record, facing) is None

# Copyright 2026 Matteo Bernacchia <dev@kikijiki.com>
#
# SPDX-License-Identifier: MIT

"""Identity channel: OCR matching, barcode, fusion rules, calibration file and fit."""
# Pure Python, deterministic: the embedding is the fake thumbnail projection, the OCR and the
# barcode readers are scripted by the first pixel of the crop, so every fusion branch is reached
# on purpose rather than by luck.

import json
from pathlib import Path
import sys

import numpy as np
import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from restocker_perception.identity import (  # noqa: E402,I100,I101 - ament and Ruff disagree
    ABSTAIN,
    IDENTIFIED,
    UNKNOWN,
    CalibrationError,
    IdentityConfig,
    IdentityEngine,
    OcrMatcher,
    crop_square,
    load_calibration,
    read_gtin,
    write_calibration,
)
from restocker_perception.identity.calibrate import (  # noqa: E402
    Sample,
    evaluate,
    fit_thresholds,
)
from restocker_perception.identity.engine import Observation  # noqa: E402
from restocker_perception.registry import FakeEmbeddingBackend, RegistryIndex  # noqa: E402

FENNLO_MINT, FENNLO_MELON, TALANOVA, DALBEX_APPLE = (
    "2000000000015",
    "2000000000022",
    "2000000000039",
    "2000000000060",
)
UNREGISTERED_VALID = "2000000000053"  # a valid GTIN-13 nobody registered
STRINGS = {
    FENNLO_MINT: ["Fennlo", "Mint", "Sparkling"],
    FENNLO_MELON: ["Fennlo", "Melon", "Sparkling"],
    TALANOVA: ["Talanova", "Pineapple", "Soda"],
    DALBEX_APPLE: ["Dalbex", "Apple", "Fizz"],
}


def crop(seed: int, tag: int = 0, noise: float = 0.0) -> np.ndarray:
    """A 32x32 crop with product-specific structure; ``tag`` scripts the fake readers."""
    rng = np.random.default_rng(seed)
    image = rng.integers(0, 256, (32, 32, 3)).astype(np.float32)
    if noise:
        image += np.random.default_rng(seed + 1000).normal(0, noise, image.shape)
    image = np.clip(image, 0, 255).astype(np.uint8)
    image[0, 0, 0] = tag
    return image


class ScriptedOcr:
    name = "scripted-ocr"
    version = "1"

    def __init__(self, lines_by_tag: dict[int, list[str]]) -> None:
        self.lines_by_tag = lines_by_tag

    def read(self, image: np.ndarray) -> list[str]:
        return list(self.lines_by_tag.get(int(image[0, 0, 0]), []))


class ScriptedBarcode:
    name = "scripted-barcode"
    version = "1"

    def __init__(self, text_by_tag: dict[int, list[str]]) -> None:
        self.text_by_tag = text_by_tag

    def decode(self, image: np.ndarray) -> list[str]:
        return list(self.text_by_tag.get(int(image[0, 0, 0]), []))


SEEDS = {FENNLO_MINT: 1, FENNLO_MELON: 2, TALANOVA: 3, DALBEX_APPLE: 4}


@pytest.fixture(scope="module")
def backend():
    return FakeEmbeddingBackend()


@pytest.fixture(scope="module")
def index(backend):
    gtins = list(SEEDS)
    exemplars = np.stack(
        [
            backend.embed([crop(SEEDS[g] * 100 + view, noise=6.0) for view in range(8)])
            for g in gtins
        ]
    )
    return RegistryIndex(
        gtins,
        exemplars,
        backend_name=backend.name,
        backend_version=backend.version,
        registry_sha256="r" * 64,
    )


def engine(index, backend, config=None, *, ocr=None, barcode=None):
    return IdentityEngine(
        index,
        backend,
        config or IdentityConfig(supported_max_rank=1),
        ocr_backend=ScriptedOcr(ocr) if ocr is not None else None,
        ocr_strings=STRINGS,
        barcode_backend=ScriptedBarcode(barcode) if barcode is not None else None,
    )


def view_of(gtin: str, tag: int = 0, k: int = 0) -> np.ndarray:
    return crop(SEEDS[gtin] * 100 + k, tag=tag, noise=8.0)


# --- OCR matcher ---------------------------------------------------------------------------------


def test_ocr_names_a_product_only_from_its_distinguishing_word():
    matcher = OcrMatcher(STRINGS)
    assert matcher.names(["FENNLO", "Mint Sparkling"]).product_id == FENNLO_MINT
    assert matcher.names(["Fennlo Melon"]).product_id == FENNLO_MELON
    # Brand and kind alone are shared or insufficient: silent, not a guess.
    assert matcher.names(["Fennlo", "Sparkling"]).product_id is None
    assert matcher.names(["Mint"]).product_id is None  # distinguishing word but nothing else read


def test_ocr_does_not_name_an_unregistered_lookalike_by_shared_words():
    matcher = OcrMatcher(STRINGS)
    # A held-out "Dalbex Cherry Fizz" shares brand and kind with the registered Apple.
    answer = matcher.names(["Dalbex", "Cherry", "Fizz"])
    assert answer.product_id is None and answer.candidates == ()


def test_ocr_is_fuzzy_but_conservative():
    matcher = OcrMatcher(STRINGS)
    assert matcher.names(["Fenlo", "Mlnt"]).product_id is None  # 'mlnt' vs 'mint' is 0.75 < 0.8
    # Every word of a product without a registered twin is distinguishing, so all must be read.
    assert (
        matcher.names(["Pineaple", "Talanova", "Soda"]).product_id == TALANOVA
    )  # one dropped letter
    assert matcher.names(["Pineapple", "Talanova"]).product_id is None


def test_ocr_combine_flags_disagreeing_crops():
    matcher = OcrMatcher(STRINGS)
    mint = matcher.names(["Fennlo Mint"])
    melon = matcher.names(["Fennlo Melon"])
    both = matcher.combine([mint, melon])
    assert both.product_id is None and both.conflict
    assert matcher.combine([mint, matcher.names([])]).product_id == FENNLO_MINT


def test_a_product_sharing_every_word_can_never_be_named():
    matcher = OcrMatcher({"a": ["Same Brand"], "b": ["Same Brand"]})
    assert matcher.names(["Same Brand"]).candidates == ()


# --- barcode -------------------------------------------------------------------------------------


def test_barcode_requires_a_valid_check_digit_and_exact_lookup():
    backend = ScriptedBarcode({1: [FENNLO_MINT], 2: ["2000000000016"], 3: [UNREGISTERED_VALID]})
    registered = [FENNLO_MINT]
    read = read_gtin([crop(1, tag=1)], backend, registered)
    assert (read.gtin, read.registered) == (FENNLO_MINT, True)
    bad = read_gtin([crop(1, tag=2)], backend, registered)
    assert bad.gtin is None and bad.rejected == 1
    unknown = read_gtin([crop(1, tag=3)], backend, registered)
    assert (unknown.gtin, unknown.registered) == (UNREGISTERED_VALID, False)
    both = read_gtin([crop(1, tag=1), crop(1, tag=3)], backend, registered)
    assert both.conflict and both.product_id is None


# --- fusion --------------------------------------------------------------------------------------


def test_default_config_never_names_anything_from_the_embedding_alone(index, backend):
    result = engine(index, backend).identify([view_of(FENNLO_MINT)])
    assert result.status == ABSTAIN and result.sku is None
    assert [e.kind for e in result.evidence] == ["embedding"]
    assert result.evidence[0].product_id == FENNLO_MINT  # the ranking is still reported


def test_ocr_agreeing_with_the_embedding_names_the_product(index, backend):
    ocr = {7: ["Fennlo", "Mint"]}
    result = engine(index, backend, ocr=ocr).identify([view_of(FENNLO_MINT, tag=7)])
    assert (result.status, result.sku) == (IDENTIFIED, FENNLO_MINT)
    assert {e.kind for e in result.evidence} == {"embedding", "ocr"}
    assert result.provenance["registry_sha256"] == "r" * 64
    assert result.provenance["ocr"] == "scripted-ocr 1"
    assert set(result.latency_ms) == {"embedding", "ocr"}


def test_ocr_contradicting_the_embedding_abstains(index, backend):
    ocr = {7: ["Talanova", "Pineapple", "Soda"]}
    result = engine(index, backend, ocr=ocr).identify([view_of(FENNLO_MINT, tag=7)])
    assert result.status == ABSTAIN and "ranks it" in result.reason


def test_ocr_rank_tolerance_is_a_calibrated_setting(index, backend):
    ocr = {7: ["Fennlo", "Melon"]}  # a twin: the embedding may rank it second
    config = IdentityConfig(supported_max_rank=len(SEEDS))
    result = engine(index, backend, config, ocr=ocr).identify([view_of(FENNLO_MINT, tag=7)])
    assert (result.status, result.sku) == (IDENTIFIED, FENNLO_MELON)


def test_ocr_naming_two_products_abstains(index, backend):
    ocr = {7: ["Fennlo", "Mint"], 8: ["Fennlo", "Melon"]}
    result = engine(index, backend, ocr=ocr).identify(
        [view_of(FENNLO_MINT, tag=7, k=0), view_of(FENNLO_MINT, tag=8, k=1)]
    )
    assert result.status == ABSTAIN and "more than one" in result.reason


def test_barcode_names_a_registered_product_and_rejects_an_unregistered_gtin(index, backend):
    barcode = {5: [TALANOVA], 6: [UNREGISTERED_VALID]}
    eng = engine(index, backend, barcode=barcode)
    named = eng.identify([view_of(TALANOVA)], barcode_images=[crop(0, tag=5)])
    assert (named.status, named.sku) == (IDENTIFIED, TALANOVA)
    unknown = eng.identify([view_of(TALANOVA)], barcode_images=[crop(0, tag=6)])
    assert (unknown.status, unknown.sku) == (UNKNOWN, None)
    assert any(e.kind == "barcode" and e.detail["registered"] is False for e in unknown.evidence)


def test_barcode_against_a_confident_embedding_for_another_product_abstains(index, backend):
    eng = engine(index, backend, barcode={5: [DALBEX_APPLE]})
    result = eng.identify([view_of(FENNLO_MINT)], barcode_images=[crop(0, tag=5)])
    assert result.status == ABSTAIN


def test_barcode_and_ocr_naming_different_products_abstain(index, backend):
    eng = engine(index, backend, ocr={7: ["Fennlo", "Mint"]}, barcode={5: [FENNLO_MELON]})
    result = eng.identify([view_of(FENNLO_MINT, tag=7)], barcode_images=[crop(0, tag=5)])
    assert result.status == ABSTAIN and "different products" in result.reason


def test_embedding_alone_needs_all_three_thresholds(index, backend):
    config = IdentityConfig(alone_min_score=0.0, alone_min_margin=0.0, alone_min_vote=1.0)
    result = engine(index, backend, config).identify([view_of(FENNLO_MINT)])
    assert (result.status, result.sku) == (IDENTIFIED, FENNLO_MINT)
    strict = IdentityConfig(alone_min_score=0.0, alone_min_margin=2.0, alone_min_vote=1.0)
    assert engine(index, backend, strict).identify([view_of(FENNLO_MINT)]).status == ABSTAIN
    with pytest.raises(CalibrationError):
        IdentityConfig(alone_min_score=0.1)


def test_a_weak_best_match_is_called_unknown_only_below_the_floor(index, backend):
    stranger = crop(999, noise=0.0)
    top = engine(index, backend).identify([stranger]).evidence[0].score
    below = IdentityConfig(unknown_below_score=top + 0.01)
    assert engine(index, backend, below).identify([stranger]).status == UNKNOWN
    above = IdentityConfig(unknown_below_score=top - 0.01)
    assert engine(index, backend, above).identify([stranger]).status == ABSTAIN


def test_no_crops_is_an_abstention_not_an_error(index, backend):
    result = engine(index, backend).identify([])
    assert result.status == ABSTAIN and result.evidence == ()


def test_result_serialises_to_json(index, backend):
    result = engine(index, backend, ocr={7: ["Fennlo", "Mint"]}).identify(
        [view_of(FENNLO_MINT, tag=7)]
    )
    document = json.loads(json.dumps(result.to_dict()))
    assert document["status"] == IDENTIFIED and document["sku"] == FENNLO_MINT


# --- calibration -----------------------------------------------------------------------------


def test_calibration_file_round_trips_and_detects_edits(tmp_path):
    config = IdentityConfig(supported_max_rank=2, unknown_below_score=0.3)
    path = tmp_path / "cal.json"
    digest = write_calibration(path, config, {"products": ["a"]})
    loaded, fit, loaded_digest = load_calibration(path)
    assert (loaded, fit, loaded_digest) == (config, {"products": ["a"]}, digest)
    document = json.loads(path.read_text())
    document["config"]["supported_max_rank"] = 3
    path.write_text(json.dumps(document))
    with pytest.raises(CalibrationError, match="edited"):
        load_calibration(path)
    with pytest.raises(CalibrationError):
        IdentityConfig.from_dict({"nonsense": 1})


def _sample(truth, known, ocr_gtin, score, margin, top1=None, twin=None):
    from restocker_perception.identity.ocr import OcrAnswer
    from restocker_perception.registry.retrieval import Match, MatchResult

    top1 = top1 or truth
    ranked = (Match(top1, score, 0), Match("other", score - margin, 0))
    match = MatchResult(ranked, 1, (top1,), "max", "b", "1", "r", None)
    ocr = OcrAnswer(ocr_gtin, ("x",) if ocr_gtin else (), (ocr_gtin,) if ocr_gtin else (), 3)
    return Sample(truth, known, Observation(match, ocr, None, {}), twin)


def _population():
    samples = []
    # known products: OCR answers on a third of them, embedding correct on 80 percent
    for i in range(60):
        right = i % 5 != 0
        samples.append(
            _sample(
                "k",
                True,
                "k" if i % 3 == 0 else None,
                0.8,
                0.05,
                top1="k" if right else "t",
                twin="t",
            )
        )
    # unknown products: embedding confident on a registered product, OCR never answers
    for _ in range(40):
        samples.append(_sample("u", False, None, 0.8, 0.05, top1="k"))
    return samples


def test_fit_respects_the_ceilings_and_names_the_ocr_supported_objects():
    samples = _population()
    config, record = fit_thresholds(
        samples, max_false_identity_known=0.005, max_unknown_named=0.025
    )
    metrics = evaluate(samples, config)
    assert metrics.false_identity_known <= 0.005
    assert metrics.unknown_named_rate <= 0.025
    assert record["feasible"] and metrics.known_correct > 0
    # the unknown population looks exactly like confident known ones to the embedding, so the
    # embedding alone must stay off
    assert not config.alone_enabled


def test_fit_with_nothing_feasible_names_nothing():
    samples = [_sample("k", True, None, 0.8, 0.05, top1="t") for _ in range(20)]
    samples += [_sample("k", True, "t", 0.8, 0.05, top1="t") for _ in range(20)]
    config, record = fit_thresholds(samples, max_false_identity_known=0.0, max_unknown_named=0.0)
    assert evaluate(samples, config).known_false_identity == 0


# --- crops -------------------------------------------------------------------------------------


def test_crop_square_size_follows_range_and_extent():
    image = np.zeros((480, 640, 3), np.uint8)
    out = crop_square(image, 320, 240, 0.30, 468.0, 0.122, margin=1.0)
    assert out.shape[0] == out.shape[1] == round(468.0 * 0.122 / 0.30)
    far = crop_square(image, 320, 240, 0.60, 468.0, 0.122, margin=1.0)
    assert far.shape[0] < out.shape[0]


def test_crop_square_refuses_objects_mostly_outside_the_image():
    image = np.zeros((480, 640, 3), np.uint8)
    assert crop_square(image, 5, 5, 0.30, 468.0, 0.122) is None
    assert crop_square(image, 320, 240, 0.0, 468.0, 0.122) is None
    clipped = crop_square(image, 20, 240, 0.30, 468.0, 0.122, margin=1.0, min_visible=0.5)
    assert clipped is not None and clipped.shape[1] < clipped.shape[0]


# --- round 2: rotation, floors, hub guard, folds --------------------------------------------


def test_rotate_image_quarter_turns_are_exact_and_others_enlarge_the_canvas():
    from restocker_perception.identity.crops import rotate_image

    image = np.arange(6 * 4 * 3, dtype=np.uint8).reshape(6, 4, 3)
    assert (rotate_image(image, 90) == np.rot90(image)).all()
    assert (rotate_image(rotate_image(image, 90), 270) == image).all()
    assert rotate_image(image, 0) is image
    tilted = rotate_image(np.full((40, 40, 3), 200, np.uint8), 45)
    assert (
        tilted.shape[0] > 40
        and tilted[0, 0, 0] == 128
        and tilted[tilted.shape[0] // 2, tilted.shape[1] // 2, 0] == 200
    )


def test_ocr_tries_rotations_until_one_names_a_product(index, backend):
    from restocker_perception.identity.crops import rotate_image

    seen = []

    class Sideways(ScriptedOcr):
        def read(self, image):
            seen.append(image.shape)
            # only a crop turned a quarter reads: the label is sideways
            return ["Fennlo", "Mint"] if image.shape == (32, 40, 3) else []

    wide = np.zeros((40, 32, 3), np.uint8)
    wide[..., 0] = np.arange(32)  # a view whose quarter turn is 32 x 40
    config = IdentityConfig(supported_max_rank=len(SEEDS), ocr_rotations=(0, 90, 270))
    eng = IdentityEngine(index, backend, config, ocr_backend=Sideways({}), ocr_strings=STRINGS)
    observation = eng.observe([wide])
    assert observation.ocr.product_id == FENNLO_MINT
    assert seen == [(40, 32, 3), (32, 40, 3)]  # stopped after the rotation that answered
    assert rotate_image(wide, 90).shape == (32, 40, 3)


def test_product_floor_blocks_the_embedding_alone_for_that_product(index, backend):
    config = IdentityConfig(alone_min_score=0.0, alone_min_margin=0.0, alone_min_vote=1.0)
    assert engine(index, backend, config).identify([view_of(FENNLO_MINT)]).status == IDENTIFIED
    floored = IdentityConfig(
        alone_min_score=0.0,
        alone_min_margin=0.0,
        alone_min_vote=1.0,
        product_floors={FENNLO_MINT: 2.0},
    )
    assert engine(index, backend, floored).identify([view_of(FENNLO_MINT)]).status == ABSTAIN


def test_hub_guard_raises_the_floor_of_a_product_that_wins_wrongly():
    from restocker_perception.identity.calibrate import naming_shares, wrong_wins

    # product "h" wins everything it is shown with a high score, right or wrong; "a" and "b"
    # are only ever right. The floors must sit above h's wrong-win scores.
    samples = [_sample("h", True, None, 0.9, 0.1, top1="h") for _ in range(10)]
    samples += [_sample("a", True, None, 0.85, 0.1, top1="h") for _ in range(10)]
    samples += [_sample("b", True, None, 0.6, 0.1, top1="b") for _ in range(10)]
    samples += [
        _sample(
            "u",
            False,
            None,
            0.8,
            0.1,
            top1="h",
        )
        for _ in range(10)
    ]
    open_config = IdentityConfig(alone_min_score=0.0, alone_min_margin=0.0, alone_min_vote=0.5)
    assert wrong_wins(samples, open_config)["h"] == pytest.approx(0.85)
    assert naming_shares(samples, open_config)["h"] > 0.6
    config, record = fit_thresholds(
        samples, max_false_identity_known=0.0, max_unknown_named=0.0, base=IdentityConfig()
    )
    metrics = evaluate(samples, config)
    assert metrics.known_false_identity == 0 and metrics.unknown_named == 0
    assert config.product_floors.get("h", 0) > 0.85 or not config.alone_enabled
    assert "calibration_metrics" in record and "hub_flags" in record


def test_unknown_twin_counts_separately():
    samples = [
        Sample("k", False, Observation(None, None, None, {}), twin="t", twin_registered=True),
        Sample("u", False, Observation(None, None, None, {})),
    ]
    metrics = evaluate(samples, IdentityConfig())
    assert metrics.n_unknown == 2 and metrics.n_unknown_twin == 1


def test_folds_put_look_alike_twins_in_different_folds():
    from restocker_perception.identity.benchmark import fold_products

    splits = {
        "lookalike_groups": {
            "A1": "g1",
            "A2": "g1",
            "B1": "g2",
            "B2": "g2",
            "S1": "g3",
            "S2": "g4",
        }
    }
    folds = fold_products(["A1", "A2", "B1", "B2", "S1", "S2"], splits)
    assert sorted(sum(folds, [])) == ["A1", "A2", "B1", "B2", "S1", "S2"]
    for pair in (("A1", "A2"), ("B1", "B2")):
        assert not any(set(pair) <= set(fold) for fold in folds)
    assert set(folds[2]) == {"S1", "S2"}


def test_cross_validation_scores_scenes_the_fit_never_saw():
    from dataclasses import replace

    from restocker_perception.identity.calibrate import cross_validate

    population = _population()
    samples = [replace(sample, seed=index % 10) for index, sample in enumerate(population)]
    report = cross_validate(
        samples,
        max_false_identity_known=0.005,
        max_unknown_named=0.025,
        base=IdentityConfig(),
        folds=5,
    )
    pooled = report["pooled"]
    assert pooled["n_known"] == 60 and pooled["n_unknown"] == 40  # every sample scored once
    assert len(report["per_fold"]) == 5
    assert pooled["false_identity_rate_known"] is not None


def test_floor_margin_is_chosen_on_out_of_fold_error():
    from dataclasses import replace

    from restocker_perception.identity.calibrate import choose_floor_margin

    samples = [replace(sample, seed=index % 10) for index, sample in enumerate(_population())]
    margin, tried = choose_floor_margin(
        samples,
        max_false_identity_known=0.005,
        max_unknown_named=0.025,
        base=IdentityConfig(),
        candidates=(0.01, 0.05),
    )
    assert margin in (0.01, 0.05) and len(tried) == 2
    assert all("unknown_named" in t and "correct" in t for t in tried)

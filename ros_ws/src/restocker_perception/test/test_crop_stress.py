# Copyright 2026 Matteo Bernacchia <dev@kikijiki.com>
#
# SPDX-License-Identifier: MIT

"""Image-space stress on ladder crops (CMB-116): determinism, identity, and what each knob does."""

from pathlib import Path
import sys

import numpy as np
import pytest

sys.path.insert(0, str(Path(__file__).resolve().parent))

from restocker_perception.identity import crop_stress as cs  # noqa: E402,I100,I101
from restocker_perception.identity.ladder import LookCrop  # noqa: E402,I100,I101


def _image(value=120, size=64):
    rng = np.random.default_rng(1)
    base = np.full((size, size, 3), value, dtype=np.int16)
    return np.clip(base + rng.integers(-40, 40, size=(size, size, 3)), 0, 255).astype(np.uint8)


def test_an_empty_spec_returns_the_very_same_array():
    image = _image()
    assert cs.stress_image(image, {}, cs.rng_for(1, "o", 0, 0)) is image
    assert cs.stress_image(image, {"name": "nothing"}, cs.rng_for(1, "o", 0, 0)) is image


def test_the_draw_depends_on_the_crop_identity_not_on_the_order():
    spec = {"noise_sigma": 8.0, "occlusion": {"fraction": 0.3, "kind": "disc"}, "motion_px": 5}
    a = cs.stress_image(_image(), spec, cs.rng_for(3, "sim:stock_can_01", 1, 2))
    b = cs.stress_image(_image(), spec, cs.rng_for(3, "sim:stock_can_01", 1, 2))
    c = cs.stress_image(_image(), spec, cs.rng_for(3, "sim:stock_can_01", 2, 2))
    assert np.array_equal(a, b) and not np.array_equal(a, c)


def test_exposure_gain_clips_highlights_and_dimming_darkens():
    image = _image(200)
    bright = cs.stress_image(image, {"gain": 2.0}, cs.rng_for(1, "o", 0, 0))
    dim = cs.stress_image(image, {"gain": 0.2}, cs.rng_for(1, "o", 0, 0))
    assert (bright == 255).mean() > 0.5 and bright.dtype == np.uint8
    assert dim.mean() < 0.3 * image.mean()


def test_a_colour_cast_scales_channels():
    image = _image(150)
    out = cs.stress_image(image, {"cast": [1.0, 0.3, 0.1]}, cs.rng_for(1, "o", 0, 0))
    assert out[..., 0].mean() > 3 * out[..., 1].mean() > 0.9 * out[..., 2].mean()


def test_blur_and_motion_blur_remove_high_frequency_and_keep_the_mean():
    image = _image()

    def detail(a):
        return (
            np.abs(np.diff(a.astype(np.float32), axis=1)).mean()
            + np.abs(np.diff(a.astype(np.float32), axis=0)).mean()
        )

    for spec in ({"blur_px": 7}, {"motion_px": 9}):
        out = cs.stress_image(image, spec, cs.rng_for(1, "o", 0, 0))
        assert detail(out) < 0.8 * detail(image)
        assert abs(float(out.mean()) - float(image.mean())) < 4.0


def test_downscale_loses_detail_and_keeps_the_shape():
    image = _image(size=64)
    out = cs.stress_image(image, {"downscale": 4.0}, cs.rng_for(1, "o", 0, 0))
    assert out.shape == image.shape
    assert len({tuple(p) for p in out[:4, :4].reshape(-1, 3)}) == 1
    assert len({tuple(p) for p in image[:4, :4].reshape(-1, 3)}) > 1


@pytest.mark.parametrize("kind", ["bar", "disc"])
def test_an_occluder_covers_about_the_asked_share(kind):
    image = np.full((100, 100, 3), 220, dtype=np.uint8)
    out = cs.stress_image(
        image, {"occlusion": {"fraction": 0.4, "kind": kind}}, cs.rng_for(2, "o", 0, 0)
    )
    covered = (out[..., 0] == cs.OCCLUDER_GREY).mean()
    assert 0.3 < covered < 0.45 if kind == "bar" else 0.08 < covered < 0.2


def test_bad_specs_are_refused():
    with pytest.raises(ValueError, match="unknown stress keys"):
        cs.validate_spec({"sparkle": 1})
    with pytest.raises(ValueError, match="kind"):
        cs.validate_spec({"occlusion": {"kind": "star", "fraction": 0.2}})
    with pytest.raises(ValueError, match="downscale"):
        cs.validate_spec({"downscale": 0.5})


def test_apply_stress_keeps_the_crop_metadata_and_counts_changes():
    crop = LookCrop(_image(), 17.0, 33.0)
    inputs = {(5, "sim:stock_can_01"): {"crops": [crop, crop]}}
    assert cs.apply_stress(inputs, {}, look=0) == 0
    assert inputs[(5, "sim:stock_can_01")]["crops"][0] is crop
    assert cs.apply_stress(inputs, {"gain": 0.5}, look=0) == 2
    new = inputs[(5, "sim:stock_can_01")]["crops"][0]
    assert (new.half_width_px, new.camera_azimuth_deg) == (17.0, 33.0)
    assert new.image.mean() < crop.image.mean()


def _look(n=4):
    crops = [LookCrop(_image(100 + 10 * i), 17.0, 10.0 * i) for i in range(n)]
    return {
        "crops": crops,
        "barcode_view_mask": [False] * n,
        "true_relative_azimuth_deg": [float(i) for i in range(n)],
    }


def test_keep_views_votes_over_the_first_crops_only():
    inputs = {(5, "sim:stock_can_01"): _look(4)}
    first = inputs[(5, "sim:stock_can_01")]["crops"][0]
    assert cs.apply_stress(inputs, {"keep_views": 2}, look=0) == 2
    data = inputs[(5, "sim:stock_can_01")]
    assert len(data["crops"]) == 2 and data["crops"][0] is first
    assert data["barcode_view_mask"] == [False, False]
    assert data["true_relative_azimuth_deg"] == [0.0, 1.0]


def test_every_nth_stresses_only_the_marked_crops():
    inputs = {(5, "sim:stock_can_01"): _look(4)}
    originals = list(inputs[(5, "sim:stock_can_01")]["crops"])
    cs.apply_stress(inputs, {"gain": 0.1, "every_nth": 2}, look=0)
    crops = inputs[(5, "sim:stock_can_01")]["crops"]
    assert crops[0] is originals[0] and crops[2] is originals[2]
    assert crops[1].image.mean() < 0.2 * originals[1].image.mean()
    assert crops[3].image.mean() < 0.2 * originals[3].image.mean()
    with pytest.raises(ValueError, match="counts"):
        cs.validate_spec({"keep_views": 0})

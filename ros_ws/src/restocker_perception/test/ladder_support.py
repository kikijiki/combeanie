# Copyright 2026 Matteo Bernacchia <dev@kikijiki.com>
#
# SPDX-License-Identifier: MIT

"""Scripted backends and a small registry for the identity-ladder tests (no model is loaded)."""
# The OCR reads the crop's first pixel as a tag that scripts its lines; the barcode reader
# likewise; the embedding is the fake thumbnail projection. Shared by test_identity_ladder.py,
# test_ladder_benchmark.py and test_identify_object*.py.

import numpy as np
from restocker_perception.identity.ladder import LadderConfig, LadderEngine, build_index
from restocker_perception.registry import FakeEmbeddingBackend, Registry
from restocker_perception.registry import render as rd

MINT, MELON, PINEAPPLE, GRAPE = (
    "2000000000015",
    "2000000000022",
    "2000000000039",
    "2000000000060",
)
UNREGISTERED_VALID = "2000000000053"
STRINGS = {
    MINT: ["Fennlo", "Mint", "Sparkling"],
    MELON: ["Fennlo", "Melon", "Sparkling"],
    PINEAPPLE: ["Talanova", "Pineapple", "Soda"],
    GRAPE: ["Zorbex", "Grape", "Iced Tea"],
}


class ScriptedOcr:
    name = "scripted-ocr"
    version = "1"

    def __init__(self, lines_by_tag):
        self.lines_by_tag = lines_by_tag

    def read(self, image):
        return list(self.lines_by_tag.get(int(image[0, 0, 0]), []))


class ScriptedBarcode:
    name = "scripted-barcode"
    version = "1"

    def __init__(self, text_by_tag):
        self.text_by_tag = text_by_tag

    def decode(self, image):
        text = self.text_by_tag.get(int(image[0, 0, 0]))
        return [text] if text else []


def crop(seed, tag=0):
    rng = np.random.default_rng(seed)
    image = rng.integers(0, 256, (32, 32, 3)).astype(np.uint8)
    image[0, 0, 0] = tag
    return image


def views_of(seed):
    rng = np.random.default_rng(seed)
    image = np.tile(rng.uniform(0.1, 0.9, 3), (96, 192, 1)).astype(np.float32)
    for _ in range(8):
        x, y = int(rng.integers(0, 160)), int(rng.integers(0, 80))
        image[y : y + 12, x : x + 24] = rng.uniform(0, 1, 3)
    return rd.render_views(image, 0.033, 0.122)


def make_registry(path):
    reg = Registry.create(path / "reg")
    for seed, (gtin, strings) in enumerate(STRINGS.items()):
        reg.register(
            display_name=" ".join(strings),
            product_class="can",
            geometry_key=f"can.t{seed}",
            radius_m=0.033,
            height_m=0.122,
            images=views_of(seed),
            gtin=gtin,
            ocr_strings=strings,
        )
    return reg


def engine(registry, lines_by_tag, *, barcodes=None, escalation_ocr=None, **config):
    backend = FakeEmbeddingBackend()
    index = build_index(registry, backend, None)
    return LadderEngine(
        registry,
        index,
        backend,
        ScriptedOcr(lines_by_tag),
        LadderConfig(**config),
        barcode_backend=ScriptedBarcode(barcodes or {}),
        escalation_ocr=escalation_ocr,
    )

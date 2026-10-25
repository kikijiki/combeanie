# Copyright 2026 Matteo Bernacchia <dev@kikijiki.com>
#
# SPDX-License-Identifier: MIT

"""Read the label text of one object from its crops, and only that object's."""
# A crop is a square around the object and holds its neighbours: round 2's OCR read "Fennlo Mint"
# off a Zorbex bottle because a Fennlo can stood behind it. The reader therefore cuts the crop to
# the object's own columns (``half_width_px``, from the detector's footprint) before it reads, and
# reads each cut twice: as it is, and, for a body of revolution, unrolled so that the foreshortened
# edges of the barrel have their own width again. Both are enlarged, because the label text of a
# crop 100 to 300 pixels wide is a handful of pixels high. Lines under a confidence floor are
# dropped, and so are lines longer than a name (prose); what is left is a list of tokens for
# :class:`LabelTextIndex`.

from __future__ import annotations

from collections.abc import Sequence
from dataclasses import dataclass

import numpy as np

from restocker_perception.identity.crops import unwrap_cylinder
from restocker_perception.registry.backends import OcrBackend, OcrLine

BAND_HALF_WIDTHS = 1.15
UPSCALE = 2


def _bilinear_enlarge(image: np.ndarray, factor: int) -> np.ndarray:
    if factor == 1:
        return image
    height, width = image.shape[:2]
    ys = (np.arange(height * factor) + 0.5) / factor - 0.5
    xs = (np.arange(width * factor) + 0.5) / factor - 0.5
    y0 = np.clip(np.floor(ys).astype(int), 0, height - 2)
    x0 = np.clip(np.floor(xs).astype(int), 0, width - 2)
    fy = np.clip(ys - y0, 0.0, 1.0)[:, None, None]
    fx = np.clip(xs - x0, 0.0, 1.0)[None, :, None]
    pixels = image.astype(np.float32)
    top = pixels[y0][:, x0] * (1 - fx) + pixels[y0][:, x0 + 1] * fx
    bottom = pixels[y0 + 1][:, x0] * (1 - fx) + pixels[y0 + 1][:, x0 + 1] * fx
    return np.clip(top * (1 - fy) + bottom * fy + 0.5, 0, 255).astype(np.uint8)


def object_band(crop: np.ndarray, half_width_px: float) -> np.ndarray:
    """The crop's own columns: the object's footprint, centred, and nothing beside it."""
    centre = (crop.shape[1] - 1) / 2.0
    left = int(max(0.0, centre - BAND_HALF_WIDTHS * half_width_px))
    right = int(min(crop.shape[1], centre + BAND_HALF_WIDTHS * half_width_px + 1))
    return np.ascontiguousarray(crop[:, left:right])


@dataclass(frozen=True)
class TextReaderConfig:
    """Fixed beforehand; the reading rule has no fitted number."""

    min_line_score: float = 0.5
    # A product's name is printed on short lines; prose (ingredients, storage, address) is long and
    # shares words with other products' names ("apple juice", "cool"), so lines of more than this
    # many words are not read as label text.
    max_line_words: int = 3
    upscale: int = UPSCALE
    unrolled: bool = True


class ObjectTextReader:
    """Crops in, text lines out, through one OCR backend."""

    def __init__(self, backend: OcrBackend, config: TextReaderConfig | None = None) -> None:
        self.backend = backend
        self.config = config or TextReaderConfig()

    def _lines(self, image: np.ndarray) -> list[OcrLine]:
        read_lines = getattr(self.backend, "read_lines", None)
        if read_lines is not None:
            lines = [
                line for line in read_lines(image) if line.score >= self.config.min_line_score
            ]
        else:
            lines = [OcrLine(text, 1.0, ()) for text in self.backend.read(image)]
        return [line for line in lines if len(line.text.split()) <= self.config.max_line_words]

    def read(
        self, crop: np.ndarray, half_width_px: float | None, *, round_body: bool = True
    ) -> list[str]:
        """Text lines of the object in one crop (two reads for a round body, one for a box)."""
        factor = self.config.upscale
        if half_width_px is None or half_width_px <= 0:
            band = crop
        else:
            band = object_band(crop, half_width_px)
        texts = [line.text for line in self._lines(_bilinear_enlarge(band, factor))]
        if self.config.unrolled and round_body and half_width_px:
            centre = (crop.shape[1] - 1) / 2.0
            unrolled = unwrap_cylinder(crop, half_width_px, centre_x=centre)
            texts += [line.text for line in self._lines(_bilinear_enlarge(unrolled, factor))]
        return texts


@dataclass(frozen=True)
class TextSighting:
    """A line the reader found on a body of revolution, with where on the barrel it sits."""

    text: str
    # Degrees round the barrel from the camera's line of sight; positive = to the right in the
    # image, i.e. towards larger world azimuth.
    theta_deg: float


def read_positioned(
    reader: ObjectTextReader, crop: np.ndarray, half_width_px: float
) -> tuple[list[str], list[TextSighting]]:
    """Like ``ObjectTextReader.read`` for a round body, and where each unrolled line sits."""
    cfg = reader.config
    centre = (crop.shape[1] - 1) / 2.0
    band = object_band(crop, half_width_px)
    texts = [line.text for line in reader._lines(_bilinear_enlarge(band, cfg.upscale))]  # noqa: SLF001
    sightings: list[TextSighting] = []
    if cfg.unrolled:
        unrolled = unwrap_cylinder(crop, half_width_px, centre_x=centre)
        width = unrolled.shape[1]
        for line in reader._lines(_bilinear_enlarge(unrolled, cfg.upscale)):  # noqa: SLF001
            texts.append(line.text)
            if line.box:
                x = line.centre[0] / cfg.upscale
                theta = -80.0 + x * (160.0 / max(width - 1, 1))
                sightings.append(TextSighting(line.text, theta))
    return texts, sightings


class UpscaledOcr:
    """An OCR backend that enlarges every image first; its own name keys the registry cache."""

    def __init__(self, backend: OcrBackend, factor: int = 3) -> None:
        self.backend = backend
        self.factor = factor
        self.name = f"{backend.name}+x{factor}"
        self.version = backend.version

    def read(self, image: np.ndarray) -> list[str]:
        return self.backend.read(_bilinear_enlarge(image, self.factor))


def registry_view_text_scores(
    view_reads: Sequence[Sequence[str]], words_of_product: Sequence[str], weight, index
) -> list[float]:
    """How much of a product's label text each registered view carries (read by the same OCR)."""
    scores = []
    for lines in view_reads:
        tokens = index.tokens_of(lines)
        total = sum(weight(w) for w in words_of_product) or 1.0
        got = sum(weight(w) for w in words_of_product if index.reads(w, tokens))
        scores.append(got / total)
    return scores


__all__ = [
    "ObjectTextReader",
    "TextReaderConfig",
    "TextSighting",
    "UpscaledOcr",
    "object_band",
    "read_positioned",
    "registry_view_text_scores",
]

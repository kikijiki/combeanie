# Copyright 2026 Matteo Bernacchia <dev@kikijiki.com>
#
# SPDX-License-Identifier: MIT

"""Embedding and OCR backend interfaces, a deterministic fake, and lazy real adapters."""
# Nothing here imports torch, transformers or rapidocr at module load. The real adapters import
# them on first use, so the registry, the tests and the CLI run on a machine that has none of them
# (they are deliberately not in the Nix flake). The fake backend is what tests and offline
# development use; it has no semantic meaning, only the properties retrieval needs (deterministic,
# identical images give identical vectors, similar images give nearby vectors, unit norm).

from __future__ import annotations

from collections.abc import Mapping, Sequence
from dataclasses import dataclass
import os
from pathlib import Path
import re
import tempfile
from typing import Protocol, runtime_checkable

import numpy as np

from restocker_perception.registry.imageio import resize_area


@runtime_checkable
class EmbeddingBackend(Protocol):
    """Maps product crops to L2-normalised vectors.

    ``name`` and ``version`` key the embedding cache. ``version`` must change whenever the
    output for the same image could change (weights, preprocessing, revision), and must be an
    immutable identifier, never ``latest``.
    """

    name: str
    version: str

    @property
    def dim(self) -> int: ...

    def embed(self, images: Sequence[np.ndarray]) -> np.ndarray:
        """HxWx3 uint8 images -> float32 array (n, dim), each row unit length."""
        ...


def _unit_rows(matrix: np.ndarray) -> np.ndarray:
    norms = np.linalg.norm(matrix, axis=1, keepdims=True)
    return (matrix / np.maximum(norms, 1e-12)).astype(np.float32)


class FakeEmbeddingBackend:
    """Deterministic random projection of a 16x16 thumbnail. For tests, not for recognition."""

    name = "fake-thumbnail-projection"

    def __init__(self, dim: int = 48, seed: int = 0, grid: int = 16) -> None:
        self._dim = dim
        self._grid = grid
        self.version = f"1-d{dim}-s{seed}-g{grid}"
        rng = np.random.default_rng(seed)
        self._projection = rng.standard_normal((grid * grid * 3, dim)).astype(np.float32)
        self.calls = 0  # images embedded, so tests can prove the cache was (not) used

    @property
    def dim(self) -> int:
        return self._dim

    def embed(self, images: Sequence[np.ndarray]) -> np.ndarray:
        if not images:
            return np.zeros((0, self._dim), dtype=np.float32)
        self.calls += len(images)
        thumbs = [
            resize_area(np.asarray(image, dtype=np.uint8), self._grid, self._grid)
            .astype(np.float32)
            .reshape(-1)
            / 255.0
            - 0.5
            for image in images
        ]
        return _unit_rows(np.stack(thumbs) @ self._projection)


class SigLip2Backend:
    """SigLIP 2 base image embedding through Hugging Face ``transformers`` (lazy, optional).

    This is the identity embedding CMB-101 recommended (PCA-whitened and multi-view voted; see
    ``Whitener`` and ``RegistryIndex.match``). Install ``torch`` and ``transformers`` outside the
    flake to use it. Nothing is downloaded until ``embed`` or ``dim`` is first used, and the
    Hugging Face token is never read here: the public checkpoint needs none.

    Pin ``revision`` to a commit hash for a cache that survives upstream changes: ``version`` is
    ``<model_id>@<revision>``, and an unpinned ``main`` is recorded as such so a moving revision
    is at least visible in the cache key.
    """

    name = "siglip2"

    def __init__(
        self,
        model_id: str = "google/siglip2-base-patch16-224",
        revision: str = "main",
        device: str = "cpu",
    ) -> None:
        self.model_id = model_id
        self.revision = revision
        self.device = device
        self.version = f"{model_id}@{revision}"
        self._model = None
        self._processor = None
        self._dim: int | None = None

    def _load(self) -> None:
        if self._model is not None:
            return
        try:
            import torch  # noqa: PLC0415, F401
            from transformers import AutoModel, AutoProcessor  # noqa: PLC0415
        except ImportError as error:
            raise RuntimeError(
                "SigLip2Backend needs torch and transformers, which are not part of the "
                "flake; install them in a separate environment"
            ) from error
        self._processor = AutoProcessor.from_pretrained(self.model_id, revision=self.revision)
        self._model = AutoModel.from_pretrained(self.model_id, revision=self.revision)
        self._model.to(self.device).eval()
        self._dim = int(self._model.config.vision_config.hidden_size)

    @property
    def dim(self) -> int:
        self._load()
        assert self._dim is not None
        return self._dim

    def embed(self, images: Sequence[np.ndarray]) -> np.ndarray:
        self._load()
        import torch  # noqa: PLC0415 - _load() has already shown it is installed

        if not images:
            return np.zeros((0, self.dim), dtype=np.float32)
        inputs = self._processor(images=[np.asarray(i) for i in images], return_tensors="pt")
        inputs = {k: v.to(self.device) for k, v in inputs.items()}
        with torch.no_grad():
            features = self._model.get_image_features(**inputs)
        if not isinstance(features, torch.Tensor):
            # transformers 5 returns a model-output object; pooler_output is the embedding.
            features = features.pooler_output
        return _unit_rows(features.cpu().numpy().astype(np.float32))


@runtime_checkable
class OcrBackend(Protocol):
    """Reads printed text from one image. ``name`` and ``version`` key the OCR cache."""

    name: str
    version: str

    def read(self, image: np.ndarray) -> list[str]: ...


@dataclass(frozen=True)
class OcrLine:
    """One text line a reader found: its text, confidence and four-corner box (x, y pixels)."""

    text: str
    score: float
    box: tuple[tuple[float, float], ...]

    @property
    def centre(self) -> tuple[float, float]:
        xs = [p[0] for p in self.box]
        ys = [p[1] for p in self.box]
        return (sum(xs) / len(xs), sum(ys) / len(ys))


class RapidOcrBackend:
    """RapidOCR (ONNX) adapter, lazy and optional like ``SigLip2Backend``.

    ``overrides`` sets RapidOCR config keys by name (``text_score``, ``box_thresh``,
    ``unclip_ratio``, ``thresh``, ``limit_side_len``); they join the cache ``version`` so a read
    made with other settings is never reused. ``threads`` bounds the ONNX threads (the default,
    all cores, starves every other process on a shared machine).
    """

    name = "rapidocr"

    def __init__(
        self,
        version: str,
        *,
        overrides: Mapping[str, float] | None = None,
        threads: int = 3,
    ) -> None:
        # The caller states the installed rapidocr version: there is no portable way to read it
        # from the engine object, and an unstated one would make the cache key meaningless.
        self.overrides = dict(overrides or {})
        self.threads = threads
        suffix = ",".join(f"{k}={v}" for k, v in sorted(self.overrides.items()))
        self.version = f"{version}[{suffix}]" if suffix else version
        self._engine = None

    def _load(self) -> None:
        try:
            import rapidocr_onnxruntime  # noqa: PLC0415
            import yaml  # noqa: PLC0415
        except ImportError as error:
            raise RuntimeError("RapidOcrBackend needs rapidocr_onnxruntime and pyyaml") from error
        base = Path(rapidocr_onnxruntime.__file__).parent
        config = yaml.safe_load((base / "config.yaml").read_text(encoding="utf-8"))
        for section in ("Global", "Det", "Cls", "Rec"):
            for key in ("intra_op_num_threads", "inter_op_num_threads"):
                if key in config[section]:
                    config[section][key] = self.threads if key.startswith("intra") else 1
        for section in ("Det", "Cls", "Rec"):
            config[section]["model_path"] = str(base / config[section]["model_path"])
        for key, value in self.overrides.items():
            for section in ("Global", "Det"):
                if key in config[section]:
                    config[section][key] = value
        handle, name = tempfile.mkstemp(suffix=".yaml", prefix="rapidocr-")
        with os.fdopen(handle, "w", encoding="utf-8") as stream:
            yaml.safe_dump(config, stream)
        try:
            self._engine = rapidocr_onnxruntime.RapidOCR(config_path=name)
        finally:
            os.unlink(name)

    def read_lines(self, image: np.ndarray) -> list[OcrLine]:
        if self._engine is None:
            self._load()
        result, _elapsed = self._engine(np.asarray(image))  # type: ignore[misc]
        return [
            OcrLine(
                str(item[1]),
                float(item[2]),
                tuple((float(x), float(y)) for x, y in item[0]),
            )
            for item in (result or [])
        ]

    def read(self, image: np.ndarray) -> list[str]:
        return [line.text for line in self.read_lines(image)]


_FLORENCE_REGION = re.compile(r"([^<]+)((?:<loc_\d+>){8})")


def parse_florence_regions(text: str, width: int, height: int) -> list[OcrLine]:
    """``<OCR_WITH_REGION>`` output (label, eight location tokens) as scored-less text lines.

    Florence-2 gives no confidence, so every line carries 1.0; the location tokens are
    thousandths of the image size, four corners in reading order.
    """
    for special in ("</s>", "<s>", "<pad>"):
        text = text.replace(special, "")
    lines = []
    for match in _FLORENCE_REGION.finditer(text):
        label = match.group(1).strip()
        locs = [int(x) for x in re.findall(r"<loc_(\d+)>", match.group(2))]
        box = tuple(
            (locs[i] / 1000.0 * width, locs[i + 1] / 1000.0 * height) for i in range(0, 8, 2)
        )
        if label:
            lines.append(OcrLine(label, 1.0, box))
    return lines


class Florence2OcrBackend:
    """Florence-2 base ``<OCR_WITH_REGION>`` through ``transformers`` (lazy, optional, CPU).

    A pretrained vision-language reader (MIT licence, 0.23 B parameters) used as it is, no
    fine-tuning. It is slow on a CPU (seconds per image: it writes the text token by token), so
    the ladder uses it only as an escalation rung on objects the cheap reader left unresolved.
    Like the other real adapters it needs ``torch`` and ``transformers`` outside the flake and
    never reads a Hugging Face token (the public checkpoint needs none).
    """

    name = "florence2"

    def __init__(
        self,
        model_id: str = "florence-community/Florence-2-base",
        revision: str = "main",
        *,
        threads: int = 4,
        max_new_tokens: int = 96,
    ) -> None:
        self.model_id = model_id
        self.revision = revision
        self.threads = threads
        self.max_new_tokens = max_new_tokens
        self.version = f"{model_id}@{revision}[tokens={max_new_tokens}]"
        self._model = None
        self._processor = None

    def _load(self) -> None:
        if self._model is not None:
            return
        try:
            import torch  # noqa: PLC0415
            from transformers import (  # noqa: PLC0415
                AutoProcessor,
                Florence2ForConditionalGeneration,
            )
        except ImportError as error:
            raise RuntimeError(
                "Florence2OcrBackend needs torch and transformers (>= 5), which are not part "
                "of the flake; install them in a separate environment"
            ) from error
        torch.set_num_threads(self.threads)
        self._processor = AutoProcessor.from_pretrained(self.model_id, revision=self.revision)
        self._model = Florence2ForConditionalGeneration.from_pretrained(
            self.model_id, revision=self.revision
        ).eval()

    def read_lines(self, image: np.ndarray) -> list[OcrLine]:
        self._load()
        from PIL import Image  # noqa: PLC0415
        import torch  # noqa: PLC0415 - _load() has already shown it is installed

        pil = Image.fromarray(np.ascontiguousarray(image)).convert("RGB")
        inputs = self._processor(text="<OCR_WITH_REGION>", images=pil, return_tensors="pt")
        with torch.no_grad():
            out = self._model.generate(
                **inputs, max_new_tokens=self.max_new_tokens, num_beams=1, do_sample=False
            )
        text = self._processor.batch_decode(out, skip_special_tokens=False)[0]
        return parse_florence_regions(text, pil.width, pil.height)

    def read(self, image: np.ndarray) -> list[str]:
        return [line.text for line in self.read_lines(image)]

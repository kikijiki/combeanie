# Copyright 2026 Matteo Bernacchia <dev@kikijiki.com>
#
# SPDX-License-Identifier: MIT

"""Derived-data cache keyed by backend name and version, and by the views it was derived from."""
# Embeddings and OCR reads are never stored in the registry: they are regenerable from the views.
# An entry is valid only if (a) it sits under the requesting backend's name and version and (b)
# it records the hash of the views of that product as they are now. Replacing the encoder, or
# editing a product's views, therefore invalidates exactly the affected entries and nothing is
# ever read stale. Entries are keyed by the registry's internal product id (the GTIN for a product
# that has one, so caches written before ids existed stay valid).

from __future__ import annotations

import json
from pathlib import Path
import re

import numpy as np

from restocker_perception.registry.backends import EmbeddingBackend, OcrBackend
from restocker_perception.registry.model import Registry

_UNSAFE = re.compile(r"[^A-Za-z0-9._-]")


def _segment(text: str) -> str:
    """A path segment for ``text`` (filesystem-safe; the exact text is re-checked inside)."""
    return _UNSAFE.sub("_", text) or "_"


class DerivedCache:
    """Disk cache under ``root``; counts hits and misses so tests can see what was reused."""

    def __init__(self, root: str | Path) -> None:
        self.root = Path(root)
        self.hits = 0
        self.misses = 0

    # --- embeddings --------------------------------------------------------------------------

    def _embedding_path(self, backend_name: str, backend_version: str, product_id: str) -> Path:
        return (
            self.root
            / "embeddings"
            / _segment(backend_name)
            / _segment(backend_version)
            / f"{product_id}.npz"
        )

    def embeddings(
        self, registry: Registry, backend: EmbeddingBackend, product_id: str
    ) -> np.ndarray:
        """The (8, dim) exemplar embeddings of ``product_id`` under ``backend``, cached."""
        record = registry.get(product_id)
        path = self._embedding_path(backend.name, backend.version, product_id)
        if path.exists():
            try:
                with np.load(path, allow_pickle=False) as data:
                    valid = (
                        str(data["backend_name"]) == backend.name
                        and str(data["backend_version"]) == backend.version
                        and str(data["view_hash"]) == record.view_hash()
                        and data["embeddings"].shape == (len(record.views), backend.dim)
                    )
                    if valid:
                        self.hits += 1
                        return data["embeddings"].astype(np.float32)
            except (OSError, ValueError, KeyError):
                pass  # unreadable entry: recompute and overwrite
        self.misses += 1
        vectors = backend.embed(registry.view_images(product_id))
        path.parent.mkdir(parents=True, exist_ok=True)
        temporary = path.with_name(path.name + ".tmp.npz")
        np.savez(
            temporary,
            embeddings=vectors,
            backend_name=backend.name,
            backend_version=backend.version,
            view_hash=record.view_hash(),
        )
        temporary.replace(path)
        return vectors

    # --- OCR ---------------------------------------------------------------------------------

    def _ocr_path(self, backend_name: str, backend_version: str, product_id: str) -> Path:
        return (
            self.root
            / "ocr"
            / _segment(backend_name)
            / _segment(backend_version)
            / f"{product_id}.json"
        )

    def ocr_reads(
        self, registry: Registry, backend: OcrBackend, product_id: str
    ) -> list[list[str]]:
        """What ``backend`` reads on each of the eight views of ``product_id`` (cached)."""
        record = registry.get(product_id)
        path = self._ocr_path(backend.name, backend.version, product_id)
        if path.exists():
            try:
                data = json.loads(path.read_text(encoding="utf-8"))
                if (
                    data["backend_name"] == backend.name
                    and data["backend_version"] == backend.version
                    and data["view_hash"] == record.view_hash()
                ):
                    self.hits += 1
                    return data["reads"]
            except (OSError, ValueError, KeyError):
                pass
        self.misses += 1
        reads = [backend.read(image) for image in registry.view_images(product_id)]
        path.parent.mkdir(parents=True, exist_ok=True)
        path.write_text(
            json.dumps(
                {
                    "backend_name": backend.name,
                    "backend_version": backend.version,
                    "view_hash": record.view_hash(),
                    "reads": reads,
                },
                sort_keys=True,
            ),
            encoding="utf-8",
        )
        return reads

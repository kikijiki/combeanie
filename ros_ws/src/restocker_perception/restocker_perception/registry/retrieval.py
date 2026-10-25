# Copyright 2026 Matteo Bernacchia <dev@kikijiki.com>
#
# SPDX-License-Identifier: MIT

"""Retrieval over registry prototypes: cosine match, top-k, top-1/top-2 margin, view voting."""
# This module ranks; it does not decide. There is deliberately no threshold, no "unknown" answer
# and no accept/reject here: abstention policy belongs to CMB-112, which calibrates it. What it
# needs is exposed on ``MatchResult``: the ranked scores, the top-1/top-2 margin, how many query
# views agreed on the winner, and the identity of everything the numbers depend on (backend name
# and version, registry content hash, whitening fingerprint), so a stored decision can be traced.

from __future__ import annotations

from collections.abc import Collection, Sequence
from dataclasses import dataclass
import hashlib
from typing import Any

import numpy as np

from restocker_perception.registry.backends import EmbeddingBackend
from restocker_perception.registry.cache import DerivedCache
from restocker_perception.registry.model import (  # noqa: I100,I101 - ament and Ruff disagree
    VIEW_STEP_DEG,
    Registry,
)

SCORINGS = ("max", "mean", "top2")


def _unit(matrix: np.ndarray) -> np.ndarray:
    norms = np.linalg.norm(matrix, axis=-1, keepdims=True)
    return (matrix / np.maximum(norms, 1e-12)).astype(np.float32)


class Whitener:
    """Centre, PCA-project and (shrinkage-)whiten, then re-normalise.

    CMB-101 found this the largest training-free gain for SigLIP 2 after multi-view voting. It
    is fit on unlabeled deployment crops (the registry alone gives at most 8 x SKUs vectors, so
    k is capped at that minus one), and it is a property of the deployment, not of a product:
    the fit is stored outside the registry and its ``fingerprint`` joins the match provenance.
    """

    def __init__(self, mean: np.ndarray, components: np.ndarray, scale: np.ndarray) -> None:
        self.mean = mean.astype(np.float32)
        self.components = components.astype(np.float32)
        self.scale = scale.astype(np.float32)

    @classmethod
    def fit(cls, embeddings: np.ndarray, k: int, shrinkage: float = 0.1) -> Whitener:
        data = np.asarray(embeddings, dtype=np.float64)
        if k < 1 or k > min(data.shape) - 1:
            raise ValueError(f"k={k} needs 1 <= k <= {min(data.shape) - 1} for {data.shape}")
        mean = data.mean(axis=0)
        _u, singular, vt = np.linalg.svd(data - mean, full_matrices=False)
        variance = singular[:k] ** 2 / max(len(data) - 1, 1)
        scale = 1.0 / np.sqrt(variance + shrinkage * variance.mean())
        return cls(mean, vt[:k], scale)

    def apply(self, embeddings: np.ndarray) -> np.ndarray:
        projected = (np.asarray(embeddings, dtype=np.float32) - self.mean) @ self.components.T
        return _unit(projected * self.scale)

    @property
    def fingerprint(self) -> str:
        digest = hashlib.sha256()
        for part in (self.mean, self.components, self.scale):
            digest.update(np.ascontiguousarray(part).tobytes())
        return digest.hexdigest()[:16]


@dataclass(frozen=True)
class Match:
    """One ranked candidate."""

    product_id: str
    score: float
    # Nominal azimuth of the exemplar that matched best (an ordering, not a pose).
    best_view_deg: int | None


@dataclass(frozen=True)
class MatchResult:
    """Ranked candidates plus everything a decision policy needs."""

    ranked: tuple[Match, ...]
    n_queries: int
    # Winner of each individual query view, so a policy can see vote agreement.
    per_query_top1: tuple[str, ...]
    scoring: str
    backend_name: str
    backend_version: str
    registry_sha256: str
    whitening: str | None

    @property
    def top1(self) -> Match | None:
        return self.ranked[0] if self.ranked else None

    @property
    def top2(self) -> Match | None:
        return self.ranked[1] if len(self.ranked) > 1 else None

    @property
    def margin(self) -> float | None:
        """top-1 score minus top-2 score; None when fewer than two products are registered."""
        if self.top2 is None or self.top1 is None:
            return None
        return self.top1.score - self.top2.score

    @property
    def vote_agreement(self) -> float:
        """Fraction of query views whose own winner is the voted winner."""
        if self.top1 is None or not self.per_query_top1:
            return 0.0
        return self.per_query_top1.count(self.top1.product_id) / len(self.per_query_top1)

    def top_k(self, k: int) -> tuple[Match, ...]:
        return self.ranked[:k]

    def scores(self) -> dict[str, float]:
        return {m.product_id: m.score for m in self.ranked}


class RegistryIndex:
    """Prototypes per product (all exemplar views kept) and cosine matching against them."""

    def __init__(
        self,
        product_ids: Sequence[str],
        exemplars: np.ndarray,
        *,
        backend_name: str,
        backend_version: str,
        registry_sha256: str,
        view_azimuths: Sequence[int] | None = None,
        whitener: Whitener | None = None,
    ) -> None:
        """``exemplars`` is (n_skus, n_views, dim), already in the matching space."""
        if exemplars.ndim != 3 or exemplars.shape[0] != len(product_ids):
            raise ValueError("exemplars must be (n_skus, n_views, dim)")
        self.product_ids = list(product_ids)
        self.exemplars = _unit(exemplars)
        self.backend_name = backend_name
        self.backend_version = backend_version
        self.registry_sha256 = registry_sha256
        self.whitener = whitener
        n_views = exemplars.shape[1]
        self.view_azimuths = list(view_azimuths or [i * VIEW_STEP_DEG for i in range(n_views)])
        self._mean_prototypes = _unit(self.exemplars.mean(axis=1))

    @classmethod
    def from_registry(
        cls,
        registry: Registry,
        backend: EmbeddingBackend,
        cache: DerivedCache | None = None,
        *,
        whitener: Whitener | None = None,
        exclude: Collection[str] = (),
    ) -> RegistryIndex:
        """Build from a registry; ``exclude`` (ids, GTINs or aliases) leaves products out."""
        dropped = {registry.resolve(item) or item for item in exclude}
        product_ids = [g for g in registry.product_ids if g not in dropped]
        if not product_ids:
            raise ValueError("no products left to index")
        cache = cache or DerivedCache(registry.root / ".cache")
        stacks = []
        for product_id in product_ids:
            vectors = cache.embeddings(registry, backend, product_id)
            stacks.append(whitener.apply(vectors) if whitener else vectors)
        return cls(
            product_ids,
            np.stack(stacks),
            backend_name=backend.name,
            backend_version=backend.version,
            registry_sha256=registry.content_hash(),
            whitener=whitener,
        )

    def _sku_scores(self, queries: np.ndarray, scoring: str) -> tuple[np.ndarray, np.ndarray]:
        """Per-query SKU scores (n, S) and the best exemplar index per query and SKU (n, S)."""
        sims = np.einsum("nd,svd->nsv", queries, self.exemplars)  # cosine: all unit length
        best_view = sims.argmax(axis=2)
        if scoring == "max":
            return sims.max(axis=2), best_view
        if scoring == "mean":
            return queries @ self._mean_prototypes.T, best_view
        if scoring == "top2":
            ordered = np.sort(sims, axis=2)
            take = min(2, ordered.shape[2])
            return ordered[:, :, -take:].mean(axis=2), best_view
        raise ValueError(f"scoring must be one of {SCORINGS}, got {scoring!r}")

    def match(self, query_embeddings: np.ndarray, *, scoring: str = "max") -> MatchResult:
        """Rank the registered products for one or several views of the same unknown object.

        Queries are raw backend embeddings; the index applies its whitener itself.

        ``scoring`` turns the eight exemplars of a SKU into one score: ``max`` (best view, the
        default because a single view shows about a third of the barrel, so most exemplars
        legitimately disagree with a given query), ``mean`` (cosine to the normalised mean
        prototype) or ``top2`` (mean of the two best views). With several query rows the
        per-query SKU scores are averaged in score space (the voting CMB-101 measured).
        """
        queries = np.atleast_2d(np.asarray(query_embeddings, dtype=np.float32))
        if self.whitener is not None:
            queries = self.whitener.apply(queries)
        queries = _unit(queries)
        if queries.shape[1] != self.exemplars.shape[2]:
            raise ValueError(
                f"query dim {queries.shape[1]} does not match index dim {self.exemplars.shape[2]}"
            )
        per_query, best_view = self._sku_scores(queries, scoring)
        voted = per_query.mean(axis=0)
        order = np.argsort(-voted, kind="stable")
        # The best view reported is the exemplar that scored highest, averaged over queries.
        ranked = tuple(
            Match(
                self.product_ids[i],
                float(voted[i]),
                self.view_azimuths[int(np.bincount(best_view[:, i]).argmax())],
            )
            for i in order
        )
        return MatchResult(
            ranked=ranked,
            n_queries=len(queries),
            per_query_top1=tuple(self.product_ids[i] for i in per_query.argmax(axis=1)),
            scoring=scoring,
            backend_name=self.backend_name,
            backend_version=self.backend_version,
            registry_sha256=self.registry_sha256,
            whitening=self.whitener.fingerprint if self.whitener else None,
        )

    def identify(
        self, images: Sequence[np.ndarray], backend: EmbeddingBackend, *, scoring: str = "max"
    ) -> MatchResult:
        """Embed query crops with ``backend`` (which must be the index's backend) and match."""
        if (backend.name, backend.version) != (self.backend_name, self.backend_version):
            raise ValueError(
                f"index was built with {self.backend_name} {self.backend_version}, "
                f"query backend is {backend.name} {backend.version}"
            )
        return self.match(backend.embed(images), scoring=scoring)

    def provenance(self) -> dict[str, Any]:
        return {
            "backend_name": self.backend_name,
            "backend_version": self.backend_version,
            "registry_sha256": self.registry_sha256,
            "whitening": self.whitener.fingerprint if self.whitener else None,
        }

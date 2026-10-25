# Copyright 2026 Matteo Bernacchia <dev@kikijiki.com>
#
# SPDX-License-Identifier: MIT

"""Product registry: exemplar views per product (GTIN optional), derived cache, retrieval."""

from restocker_perception.registry.backends import (
    EmbeddingBackend,
    FakeEmbeddingBackend,
    SigLip2Backend,
)
from restocker_perception.registry.cache import DerivedCache
from restocker_perception.registry.gtin import InvalidGtin, validate_gtin13
from restocker_perception.registry.model import (  # noqa: I100,I101 - ament and Ruff disagree
    VIEW_COUNT,
    ProductRecord,
    Registry,
    RegistryError,
    ViewRef,
)
from restocker_perception.registry.retrieval import Match, MatchResult, RegistryIndex, Whitener

__all__ = [
    "VIEW_COUNT",
    "DerivedCache",
    "EmbeddingBackend",
    "FakeEmbeddingBackend",
    "InvalidGtin",
    "Match",
    "MatchResult",
    "ProductRecord",
    "Registry",
    "RegistryError",
    "RegistryIndex",
    "SigLip2Backend",
    "ViewRef",
    "Whitener",
    "validate_gtin13",
]

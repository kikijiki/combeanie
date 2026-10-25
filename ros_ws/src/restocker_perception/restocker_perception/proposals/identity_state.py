# Copyright 2026 Matteo Bernacchia <dev@kikijiki.com>
#
# SPDX-License-Identifier: MIT

"""From a proposal and an identity result to an explicit identity state (CMB-113)."""
# A proposal is something that stands where products stand; the identity channel may or may not be
# able to name it. This module is the one place that decides what the rest of the system is told:
#
#   IDENTIFIED       the channel named a registered SKU and nothing contradicts it.
#   UNKNOWN_PRODUCT  everything else: the channel said unknown, abstained, was unavailable or
#                    absent, or named a SKU whose registered shape the proposal's geometry
#                    contradicts. It carries no SKU, ever. ``reason`` says which, and the evidence
#                    lists what each channel and the geometry said.
#
# The state fills the additive identity fields of ``restocker_interfaces/msg/ObjectObservation``
# (``identity_fields``); world state refuses an observation that pairs UNKNOWN_PRODUCT with a SKU
# or IDENTIFIED without one. Nothing here is fitted, and the identity channel is consumed as is:
# an abstention is not a failure, it is the most common honest answer.

from __future__ import annotations

from collections.abc import Callable, Sequence
from dataclasses import dataclass
from typing import Any

from restocker_perception.proposals.geometry import Proposal, ProposalConfig

IDENTITY_NOT_ASSESSED = 0
IDENTITY_IDENTIFIED = 1
IDENTITY_UNKNOWN_PRODUCT = 2

# SKU (registry product id) -> (radius m, height m) of the registered product, or None if not.
GeometryOfSku = Callable[[str], "tuple[float, float] | None"]


@dataclass(frozen=True)
class ProposalIdentity:
    """The identity state of one proposal, as it goes into an ObjectObservation."""

    state: int
    sku: str | None
    reason: str
    evidence_kinds: tuple[str, ...]
    evidence_scores: tuple[float, ...]

    @property
    def state_name(self) -> str:
        return {
            IDENTITY_NOT_ASSESSED: "not_assessed",
            IDENTITY_IDENTIFIED: "identified",
            IDENTITY_UNKNOWN_PRODUCT: "unknown_product",
        }[self.state]


def geometry_evidence(proposal: Proposal) -> tuple[str, float]:
    """The geometry's evidence: 1.0 when an upright catalogued cylinder was fitted, else 0.0."""
    modelled = proposal.kind == "disc" and proposal.catalogue_class is not None
    return "geometry", 1.0 if modelled else 0.0


def _geometry_contradicts(
    proposal: Proposal, sku: str, geometry_of_sku: GeometryOfSku | None, config: ProposalConfig
) -> str | None:
    """A reason if a fitted disc cannot be the named SKU's registered cylinder, else None."""
    if geometry_of_sku is None or proposal.kind != "disc":
        return None
    registered = geometry_of_sku(sku)
    if registered is None:
        return None
    radius, height = registered
    if abs(proposal.radius_m - radius) > config.catalogue_radius_tol_m:
        return f"disc radius {proposal.radius_m:.3f} m, registered {radius:.3f} m"
    if abs(proposal.height_m - height) > config.catalogue_height_tol_m:
        return f"disc height {proposal.height_m:.3f} m, registered {height:.3f} m"
    return None


def resolve_identity(
    proposal: Proposal,
    result: Any | None,
    *,
    geometry_of_sku: GeometryOfSku | None = None,
    config: ProposalConfig | None = None,
) -> ProposalIdentity:
    """The identity state of a proposal given the channel's result (None: no result).

    ``result`` is an ``IdentityResult`` (``sku``, ``status``, ``reason``, ``evidence``) or any
    object with those fields; ``geometry_of_sku`` enables the geometry contradiction check.
    """
    config = config or ProposalConfig()
    kinds: list[str] = []
    scores: list[float] = []
    if result is not None:
        kinds += [e.kind for e in result.evidence]
        scores += [float(e.score) for e in result.evidence]
    kind, score = geometry_evidence(proposal)
    kinds.append(kind)
    scores.append(score)
    if result is None:
        return _unknown(
            "no identity result (no crop, or the channel is unavailable)", kinds, scores
        )
    if result.status == "identified" and result.sku:
        contradiction = _geometry_contradicts(proposal, result.sku, geometry_of_sku, config)
        if contradiction is not None:
            return _unknown(f"geometry contradicts identity: {contradiction}", kinds, scores)
        return ProposalIdentity(
            IDENTITY_IDENTIFIED, result.sku, result.reason, tuple(kinds), tuple(scores)
        )
    return _unknown(f"identity {result.status}: {result.reason}", kinds, scores)


def _unknown(reason: str, kinds: Sequence[str], scores: Sequence[float]) -> ProposalIdentity:
    return ProposalIdentity(IDENTITY_UNKNOWN_PRODUCT, None, reason, tuple(kinds), tuple(scores))


def identity_fields(identity: ProposalIdentity) -> dict[str, Any]:
    """The ObjectObservation fields this state sets, as plain values (no ROS import)."""
    return {
        "has_sku": identity.sku is not None,
        "sku": identity.sku or "",
        "identity_state": identity.state,
        "identity_evidence_kinds": list(identity.evidence_kinds),
        "identity_evidence_scores": [float(s) for s in identity.evidence_scores],
        "identity_reason": identity.reason,
    }

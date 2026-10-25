# Copyright 2026 Matteo Bernacchia <dev@kikijiki.com>
#
# SPDX-License-Identifier: MIT

"""Product proposals: where something that could be a product stands (CMB-113)."""

from restocker_perception.proposals.geometry import (
    CameraModel,
    Proposal,
    ProposalConfig,
    ProposalSet,
    SupportSurface,
    all_surfaces,
    lane_surface,
    propose,
    tray_surface,
)
from restocker_perception.proposals.identity_state import (
    IDENTITY_IDENTIFIED,
    IDENTITY_NOT_ASSESSED,
    IDENTITY_UNKNOWN_PRODUCT,
    ProposalIdentity,
    identity_fields,
    resolve_identity,
)

__all__ = [
    "IDENTITY_IDENTIFIED",
    "IDENTITY_NOT_ASSESSED",
    "IDENTITY_UNKNOWN_PRODUCT",
    "ProposalIdentity",
    "identity_fields",
    "resolve_identity",
    "CameraModel",
    "Proposal",
    "ProposalConfig",
    "ProposalSet",
    "SupportSurface",
    "all_surfaces",
    "lane_surface",
    "propose",
    "tray_surface",
]

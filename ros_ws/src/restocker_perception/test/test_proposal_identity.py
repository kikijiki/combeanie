# Copyright 2026 Matteo Bernacchia <dev@kikijiki.com>
#
# SPDX-License-Identifier: MIT

"""Identity state of a proposal: a proposal the channel cannot name is an unknown product."""
# Pure Python, no model: the identity result is built by hand so every branch is reached on
# purpose. The rule under test is the card's: never a forced SKU, and the reason and evidence
# always say why.

from pathlib import Path
import sys

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from restocker_perception.identity import (  # noqa: E402,I100,I101 - ament and Ruff disagree
    ABSTAIN,
    IDENTIFIED,
    UNKNOWN,
    Evidence,
    IdentityResult,
)
from restocker_perception.proposals import (  # noqa: E402
    IDENTITY_IDENTIFIED,
    IDENTITY_UNKNOWN_PRODUCT,
    Proposal,
    identity_fields,
    resolve_identity,
)

CAN_GTIN = "4006381333931"
BOTTLE_GTIN = "4006381333948"
REGISTERED = {CAN_GTIN: (0.033, 0.122), BOTTLE_GTIN: (0.045, 0.290)}


def proposal(kind="disc", radius=0.033, height=0.123, catalogue_class="can"):
    return Proposal(
        proposal_id=1,
        surface="tray",
        kind=kind,
        centre_world=(0.0, -0.8),
        top_z_m=0.69,
        height_m=height,
        radius_m=radius,
        area_m2=0.0034,
        n_pixels=5000,
        bbox=(0, 0, 10, 10),
        disc_rms_m=0.001 if kind == "disc" else None,
        catalogue_class=catalogue_class if kind == "disc" else None,
    )


def result(status, sku=None, reason="because", evidence=()):
    return IdentityResult(
        sku=sku,
        status=status,
        reason=reason,
        evidence=tuple(evidence),
        provenance={},
        latency_ms={},
    )


EMBEDDING = Evidence("embedding", CAN_GTIN, 0.81)


def test_a_named_product_that_geometry_agrees_with_is_identified():
    identity = resolve_identity(
        proposal(),
        result(IDENTIFIED, CAN_GTIN, evidence=[EMBEDDING]),
        geometry_of_sku=REGISTERED.get,
    )
    assert identity.state == IDENTITY_IDENTIFIED and identity.sku == CAN_GTIN
    assert identity.evidence_kinds == ("embedding", "geometry")
    assert identity.evidence_scores == pytest.approx((0.81, 1.0))


@pytest.mark.parametrize(("status", "word"), [(UNKNOWN, "unknown"), (ABSTAIN, "abstain")])
def test_a_proposal_the_channel_cannot_name_is_an_unknown_product_without_a_sku(status, word):
    identity = resolve_identity(proposal(), result(status, None, "weak", [EMBEDDING]))
    assert identity.state == IDENTITY_UNKNOWN_PRODUCT and identity.sku is None
    assert word in identity.reason and "weak" in identity.reason
    assert identity.evidence_kinds == ("embedding", "geometry")


def test_no_result_at_all_is_an_unknown_product_never_a_guess():
    identity = resolve_identity(proposal(), None)
    assert identity.state == IDENTITY_UNKNOWN_PRODUCT and identity.sku is None
    assert identity.evidence_kinds == ("geometry",)
    assert "no identity result" in identity.reason


def test_geometry_that_contradicts_the_named_sku_withdraws_the_name():
    big = proposal(radius=0.045, height=0.291, catalogue_class="large_bottle")
    identity = resolve_identity(
        big, result(IDENTIFIED, CAN_GTIN, evidence=[EMBEDDING]), geometry_of_sku=REGISTERED.get
    )
    assert identity.state == IDENTITY_UNKNOWN_PRODUCT and identity.sku is None
    assert identity.reason.startswith("geometry contradicts identity")
    tall = proposal(height=0.29, catalogue_class=None)
    assert (
        resolve_identity(tall, result(IDENTIFIED, CAN_GTIN), geometry_of_sku=REGISTERED.get).state
        == IDENTITY_UNKNOWN_PRODUCT
    )


def test_a_blob_cannot_be_contradicted_so_the_name_stands_with_zero_geometry_evidence():
    blob = proposal(kind="blob")
    identity = resolve_identity(
        blob, result(IDENTIFIED, BOTTLE_GTIN), geometry_of_sku=REGISTERED.get
    )
    assert identity.state == IDENTITY_IDENTIFIED
    assert identity.evidence_kinds == ("geometry",) and identity.evidence_scores == (0.0,)


def test_fields_fill_the_observation_message_consistently():
    named = identity_fields(resolve_identity(proposal(), result(IDENTIFIED, CAN_GTIN)))
    assert named["has_sku"] is True and named["sku"] == CAN_GTIN
    assert named["identity_state"] == IDENTITY_IDENTIFIED
    unknown = identity_fields(resolve_identity(proposal(), result(ABSTAIN)))
    assert unknown["has_sku"] is False and unknown["sku"] == ""
    assert unknown["identity_state"] == IDENTITY_UNKNOWN_PRODUCT
    assert len(unknown["identity_evidence_kinds"]) == len(unknown["identity_evidence_scores"])
    assert unknown["identity_reason"]


def test_unknown_is_never_paired_with_a_sku_whatever_the_result_says():
    for status in (UNKNOWN, ABSTAIN):
        # Even a result object that wrongly carries a SKU with a non-identified status is dropped.
        identity = resolve_identity(proposal(), result(status, CAN_GTIN))
        assert identity.sku is None and identity.state == IDENTITY_UNKNOWN_PRODUCT


def test_message_constants_match_the_interface():
    text = (
        Path(__file__).resolve().parents[2]
        / "restocker_interfaces"
        / "msg"
        / "ObjectObservation.msg"
    ).read_text(encoding="utf-8")
    assert "uint8 IDENTITY_NOT_ASSESSED=0" in text
    assert f"uint8 IDENTITY_IDENTIFIED={IDENTITY_IDENTIFIED}" in text
    assert f"uint8 IDENTITY_UNKNOWN_PRODUCT={IDENTITY_UNKNOWN_PRODUCT}" in text

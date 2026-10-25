# Copyright 2026 Matteo Bernacchia <dev@kikijiki.com>
#
# SPDX-License-Identifier: MIT

"""Lane stock estimator: synthetic evidence in, counts, composition and refusals out."""

import math
from pathlib import Path
import sys

import pytest
import yaml
from restocker_perception.stock import estimator as est
from restocker_perception.stock.config import (
    CLASS_ENUM,
    load_stock_config,
)
from restocker_perception.stock.identity_policy import face_from_reply

CONFIG = load_stock_config()
CAN = CONFIG.products["can"]
SMALL = CONFIG.products["small_bottle"]
LARGE = CONFIG.products["large_bottle"]
BIN = CONFIG.measurement.bin_width_m


def column_profile(columns):
    """Top profile of packed runs [(geometry, count), ...] from the rear, as the camera sees it."""
    cell = CONFIG.workcell
    depth = cell.bed_datum_depth_m
    runs = []
    for geometry, count in reversed(columns):  # build front to rear
        for _ in range(count):
            centre = depth - geometry.radius_m
            runs.append((centre - CONFIG.rear_extent_m(geometry), geometry))
            depth -= CONFIG.pitch_m(geometry)
    runs.sort(key=lambda item: item[0])
    window_front = cell.bed_datum_depth_m - CONFIG.measurement.front_inset_m
    bins = math.ceil((window_front - cell.rear_clearance_m) / BIN)
    near, top = [], []
    for index in range(bins):
        low = cell.rear_clearance_m + index * BIN
        high = low + BIN
        hit = [g for rear, g in runs if rear < high and rear + CONFIG.pitch_m(g) > low]
        if hit and low >= runs[0][0] - BIN:
            near.append(max(low, runs[0][0]) if index == 0 or low < runs[0][0] + BIN else low)
            top.append(hit[0].height_m)
        else:
            near.append(None)
            top.append(None)
    return est.TopProfile(near, top, BIN)


def evidence(geometry, count, expected=None, sku="GEN-001", **overrides):
    expected = expected or geometry
    base = dict(
        lane_id="lane_01",
        evidence_ok=True,
        coverage=1.0,
        free_depth_m=CONFIG.free_depth_of_column(geometry, count),
        obstructed=False,
        expected_class=expected.product_class,
        expected_sku=sku,
        profile=column_profile([(geometry, count)]) if count else None,
    )
    base.update(overrides)
    return est.LaneEvidence(**base)


# Box shapes (CMB-120): a lane's product class says "box"; its SKU says which one.
CARTON_200 = CONFIG.products["carton_200"]
CARTON_1000 = CONFIG.products["carton_1000"]
SLIM = CONFIG.products["slim_box"]
BOX_SKU = {"carton_200": "GEN-017", "carton_1000": "GEN-019", "slim_box": "GEN-020"}


@pytest.mark.parametrize("geometry", [CAN, SMALL, LARGE])
@pytest.mark.parametrize("count", [0, 1, 2, 3, 5, 6, 8])
def test_exact_counts_for_every_class(geometry, count):
    if geometry is LARGE and count > 8:
        pytest.skip("beyond capacity")
    result = est.estimate_lane_stock(evidence(geometry, count), CONFIG)
    assert result.state == est.STATE_ESTIMATED
    assert (result.count_min, result.count_max) == (count, count) or count == 0
    assert result.count_estimate == count
    if count == 0:
        assert result.composition == est.COMPOSITION_EMPTY


def test_low_coverage_is_a_refusal_not_an_empty_lane():
    result = est.estimate_lane_stock(evidence(CAN, 0, coverage=0.463), CONFIG)
    assert result.state == est.STATE_CANNOT_TELL
    assert result.reason == est.REASON_LOW_COVERAGE
    assert result.count_estimate == 0 and result.composition == est.COMPOSITION_UNKNOWN


def test_coverage_floor_is_the_producers():
    below = est.estimate_lane_stock(evidence(CAN, 3, coverage=0.899), CONFIG)
    at = est.estimate_lane_stock(evidence(CAN, 3, coverage=0.90), CONFIG)
    assert below.reason == est.REASON_LOW_COVERAGE and at.answered


def test_producer_refusal_and_obstruction_refuse():
    assert est.estimate_lane_stock(evidence(CAN, 3, evidence_ok=False), CONFIG).reason == (
        est.REASON_NO_EVIDENCE
    )
    assert est.estimate_lane_stock(evidence(CAN, 3, obstructed=True), CONFIG).reason == (
        est.REASON_OBSTRUCTED
    )


def test_half_a_pitch_off_is_inconsistent():
    base = CONFIG.free_depth_of_column(CAN, 3)
    result = est.estimate_lane_stock(
        evidence(CAN, 3, free_depth_m=base + 0.5 * CONFIG.pitch_m(CAN), profile=None), CONFIG
    )
    assert result.reason == est.REASON_INCONSISTENT_GEOMETRY


def test_small_measurement_error_keeps_the_count_and_widens_the_range():
    base = CONFIG.free_depth_of_column(CAN, 4)
    result = est.estimate_lane_stock(
        evidence(CAN, 4, free_depth_m=base + 0.3 * CONFIG.pitch_m(CAN), profile=None), CONFIG
    )
    assert result.answered and result.count_estimate == 4
    assert result.count_min <= 4 <= result.count_max and result.count_min < result.count_max


def test_no_policy_and_no_profile_refuses():
    result = est.estimate_lane_stock(
        evidence(CAN, 3, expected_class=0, expected_sku=None, profile=None), CONFIG
    )
    assert result.reason == est.REASON_NO_POLICY


def test_policy_free_geometry_still_counts_with_a_profile():
    result = est.estimate_lane_stock(
        evidence(LARGE, 4, expected_class=0, expected_sku=None), CONFIG
    )
    assert result.answered and result.count_estimate == 4
    assert result.product_class_assumed == CLASS_ENUM["large_bottle"]


def test_wrong_class_in_the_rear_is_flagged_without_any_identity():
    result = est.estimate_lane_stock(evidence(LARGE, 3, expected=CAN), CONFIG)
    assert result.answered and result.misplaced and result.class_mismatch
    assert result.composition == est.COMPOSITION_MISPLACED
    assert result.count_estimate == 3  # counted with the bottle's pitch, not the can's


def test_mixed_class_column_is_described_by_its_runs_and_flagged():
    # Rear three cans, two large bottles in front of them, in a lane whose policy says cans.
    profile = column_profile([(CAN, 3), (LARGE, 2)])
    first = next(i for i, d in enumerate(profile.near_depth_m) if d is not None)
    lane = est.LaneEvidence(
        lane_id="lane_02",
        evidence_ok=True,
        coverage=1.0,
        free_depth_m=profile.near_depth_m[first] - CONFIG.workcell.rear_clearance_m,
        obstructed=False,
        expected_class=CAN.product_class,
        expected_sku="GEN-001",
        profile=profile,
    )
    result = est.estimate_lane_stock(lane, CONFIG)
    assert result.answered and result.count_estimate == 5
    assert result.segments == [(CAN.product_class, 3), (LARGE.product_class, 2)]
    assert result.misplaced and result.composition == est.COMPOSITION_MISPLACED


def test_abstaining_identity_never_clears_a_lane():
    abstain = est.FaceIdentity(est.IDENTITY_ABSTAIN)
    result = est.estimate_lane_stock(evidence(CAN, 3, faces=[abstain]), CONFIG)
    assert result.composition == est.COMPOSITION_UNKNOWN
    assert not result.misplaced and result.named_count == 0 and result.unverified_count == 3


def test_named_matching_face_is_partial_until_every_product_is_named():
    named = est.FaceIdentity(est.IDENTITY_NAMED, "GEN-001")
    partial = est.estimate_lane_stock(evidence(CAN, 3, faces=[named]), CONFIG)
    whole = est.estimate_lane_stock(evidence(CAN, 1, faces=[named]), CONFIG)
    assert partial.composition == est.COMPOSITION_PARTIAL and partial.unverified_count == 2
    assert whole.composition == est.COMPOSITION_CONFORMING and whole.unverified_count == 0


def test_named_wrong_sku_is_misplaced():
    wrong = est.FaceIdentity(est.IDENTITY_NAMED, "GEN-002")
    result = est.estimate_lane_stock(evidence(CAN, 3, faces=[wrong]), CONFIG)
    assert result.misplaced and result.composition == est.COMPOSITION_MISPLACED
    assert result.misplaced_skus == ["GEN-002"] and result.misplaced_named_count == 1


def test_unknown_identity_does_not_flag():
    unknown = est.FaceIdentity(est.IDENTITY_UNKNOWN)
    assert not est.estimate_lane_stock(evidence(CAN, 3, faces=[unknown]), CONFIG).misplaced


def test_count_without_any_identity_channel():
    result = est.estimate_lane_stock(
        evidence(SMALL, 5, faces=[est.FaceIdentity(est.IDENTITY_UNAVAILABLE)]), CONFIG
    )
    assert result.answered and result.count_estimate == 5 and not result.identity_channel_reachable


def test_hole_in_the_top_surface_is_refused_not_widened():
    profile = column_profile([(CAN, 6)])
    near = list(profile.near_depth_m)
    top = list(profile.top_height_m)
    first = next(i for i, d in enumerate(near) if d is not None)
    for index in range(first + 4, first + 12):  # a product's worth missing in the middle
        near[index] = None
        top[index] = None
    holed = est.TopProfile(near, top, BIN)
    result = est.estimate_lane_stock(evidence(CAN, 6, profile=holed), CONFIG)
    assert result.reason == est.REASON_INCONSISTENT_GEOMETRY and result.count_estimate == 0
    assert not result.misplaced


def test_a_surface_one_bin_wide_is_not_a_product():
    one = est.TopProfile([0.81] + [None] * 40, [0.09] + [None] * 40, BIN)
    result = est.estimate_lane_stock(evidence(CAN, 1, free_depth_m=0.80, profile=one), CONFIG)
    assert result.reason == est.REASON_INCONSISTENT_GEOMETRY


class TestIdentityPolicy:
    def reply(self, **kwargs):
        base = {"sidecar_available": True, "status": 0, "has_sku": True, "sku": "A", "reason": ""}
        base.update(kwargs)
        return base

    def test_sidecar_absent_is_unavailable(self):
        assert face_from_reply(None).status == est.IDENTITY_UNAVAILABLE
        assert face_from_reply({"sidecar_available": False}).status == est.IDENTITY_UNAVAILABLE

    def test_embedding_only_name_is_an_abstention_under_the_safe_policy(self):
        reply = self.reply(evidence_kinds=["embedding"], evidence_gtins=["A"])
        assert face_from_reply(reply).status == est.IDENTITY_ABSTAIN
        assert face_from_reply(reply, "embedding").status == est.IDENTITY_NAMED

    def test_barcode_or_ocr_confirmation_names(self):
        for kind in ("barcode", "ocr"):
            reply = self.reply(evidence_kinds=["embedding", kind], evidence_gtins=["A", "A"])
            face = face_from_reply(reply)
            assert face.status == est.IDENTITY_NAMED and face.sku == "A"

    def test_confirmation_of_another_gtin_does_not_name(self):
        reply = self.reply(evidence_kinds=["barcode"], evidence_gtins=["B"])
        assert face_from_reply(reply).status == est.IDENTITY_ABSTAIN

    def test_unknown_and_abstain_pass_through(self):
        assert face_from_reply(self.reply(status=2, has_sku=False)).status == est.IDENTITY_UNKNOWN
        assert face_from_reply(self.reply(status=1, has_sku=False)).status == est.IDENTITY_ABSTAIN


def test_config_restates_the_catalogue_and_the_workcell():
    sys.path.insert(0, str(Path(__file__).resolve().parents[2] / "restocker_gazebo"))
    from restocker_gazebo import lane_columns as gazebo

    for entry in gazebo.CATALOGUE.values():
        mine = CONFIG.products[entry["product_class"]]
        assert (mine.radius_m, mine.height_m) == (entry["radius_m"], entry["height_m"])
    assert CONFIG.workcell.bed_datum_depth_m == gazebo.BED_DATUM_DEPTH_M
    assert CONFIG.workcell.incline_rad == pytest.approx(gazebo.LANE_INCLINE_RAD)
    assert CONFIG.workcell.lane_center_x_m == gazebo.LANE_CENTER_X_M
    for key, entry in gazebo.CATALOGUE.items():
        for count in (0, 1, 4):
            assert CONFIG.free_depth_of_column(
                CONFIG.products[entry["product_class"]], count
            ) == pytest.approx(gazebo.ground_truth_available_depth_m(key, count))


def test_a_step_in_the_top_height_shorter_than_a_product_is_noise_not_a_mixed_column():
    profile = column_profile([(CAN, 6)])
    near, top = list(profile.near_depth_m), list(profile.top_height_m)
    first = next(i for i, d in enumerate(near) if d is not None)
    for index in range(first + 6, first + 8):  # two bins read as a bottle's height
        top[index] = LARGE.height_m
    result = est.estimate_lane_stock(
        evidence(CAN, 6, profile=est.TopProfile(near, top, BIN)), CONFIG
    )
    assert result.reason == est.REASON_INCONSISTENT_GEOMETRY and not result.misplaced


def test_too_noisy_a_frame_makes_no_class_claim():
    quiet = column_profile([(LARGE, 3)])
    noisy = est.TopProfile(quiet.near_depth_m, quiet.top_height_m, BIN, bed_sigma_m=0.012)
    calm = est.TopProfile(quiet.near_depth_m, quiet.top_height_m, BIN, bed_sigma_m=0.002)
    can_lane = dict(expected=CAN, free_depth_m=CONFIG.free_depth_of_column(LARGE, 3))
    assert est.estimate_lane_stock(
        evidence(LARGE, 3, expected=CAN, profile=calm, free_depth_m=can_lane["free_depth_m"]),
        CONFIG,
    ).misplaced
    assert not est.estimate_lane_stock(
        evidence(LARGE, 3, expected=CAN, profile=noisy, free_depth_m=can_lane["free_depth_m"]),
        CONFIG,
    ).misplaced


@pytest.mark.parametrize("geometry", [CARTON_200, CARTON_1000, SLIM])
@pytest.mark.parametrize("count", [1, 2, 3, 5, 8])
def test_exact_counts_for_every_box_shape_from_its_registered_dimensions(geometry, count):
    lane = evidence(geometry, count, sku=BOX_SKU[geometry.name])
    result = est.estimate_lane_stock(lane, CONFIG)
    assert result.answered, result.reason_detail
    assert result.count_estimate == count and result.count_min == result.count_max == count
    assert result.product_name == geometry.name and not result.misplaced


@pytest.mark.parametrize("geometry", [CARTON_200, CARTON_1000, SLIM])
def test_a_box_lane_is_counted_from_the_top_height_when_the_policy_names_no_sku(geometry):
    lane = evidence(geometry, 4, sku=None, expected_class=CLASS_ENUM["box"])
    result = est.estimate_lane_stock(lane, CONFIG)
    assert result.answered and result.count_estimate == 4
    assert result.product_name == geometry.name


def test_a_carton_is_not_read_as_a_can_although_their_heights_are_17_mm_apart():
    result = est.estimate_lane_stock(evidence(CARTON_200, 3, sku="GEN-017"), CONFIG)
    assert result.product_name == "carton_200" and result.count_estimate == 3
    result = est.estimate_lane_stock(evidence(CAN, 3), CONFIG)
    assert result.product_name == "can" and result.count_estimate == 3


def test_the_wrong_box_in_a_box_lane_is_flagged_by_its_dimensions_alone():
    # The lane is registered for the 200 mL carton; the column is the 1 L carton. Both are class
    # "box", so only the registered dimensions can tell.
    lane = evidence(CARTON_1000, 3, expected=CARTON_200, sku="GEN-017")
    result = est.estimate_lane_stock(lane, CONFIG)
    assert result.answered and result.class_mismatch and result.misplaced
    assert result.count_estimate == 3 and result.product_name == "carton_1000"


def test_look_alike_registered_skus_share_their_shape_and_are_not_a_mismatch():
    for sku in ("GEN-017", "GEN-018"):
        result = est.estimate_lane_stock(evidence(CARTON_200, 2, sku=sku), CONFIG)
        assert result.answered and not result.misplaced and not result.class_mismatch


@pytest.mark.parametrize("orientation", [o for o in CARTON_200.orientations() if o.height_m < 0.07])
def test_a_registered_box_lying_on_a_face_is_counted_with_that_orientation(orientation):
    lane = evidence(orientation, 3, expected=CARTON_200, sku="GEN-017")
    result = est.estimate_lane_stock(lane, CONFIG)
    assert result.answered and result.count_estimate == 3, result.reason_detail
    assert not result.misplaced


def test_a_box_column_is_not_a_phantom_in_an_empty_lane_of_cylinders():
    # Nothing in the lane but a one-bin smear at the retainer's height (0.09 m): no catalogued
    # product is that tall, box shapes included.
    one = est.TopProfile([0.81] + [None] * 40, [0.09] + [None] * 40, BIN)
    result = est.estimate_lane_stock(
        evidence(CARTON_200, 1, free_depth_m=0.80, profile=one, sku="GEN-017"), CONFIG
    )
    assert result.reason == est.REASON_INCONSISTENT_GEOMETRY


def test_two_products_with_one_height_and_different_counts_are_not_guessed():
    # Heights 15 mm apart under noise: the slim box and the 1 L carton. A column whose length
    # fits both and counts differently is refused unless the lane's own product is one of them.
    options = [SLIM, CARTON_1000]
    nearest = CONFIG.workcell.rear_clearance_m + CONFIG.free_depth_of_column(SLIM, 5)
    picked, why = est._pick(options, CONFIG, nearest, None)
    assert picked is SLIM or picked is None
    if picked is None:
        assert "could be" in why


def test_a_quarter_turn_of_the_registered_box_is_not_a_candidate_but_its_faces_are():
    names = {p.name for p in est._catalogue(CONFIG, SLIM)}
    assert "slim_box@zup-xalong" not in names  # same top height, another pitch: noise-fits
    assert "slim_box@xup-yalong" in names and "slim_box@yup-xalong" in names  # lying on a face


@pytest.mark.parametrize("geometry", [CARTON_200, CARTON_1000, SLIM])
@pytest.mark.parametrize("count", [1, 2, 3, 4, 5, 6, 7, 8])
def test_a_box_column_is_never_counted_with_another_orientations_pitch(geometry, count):
    # CMB-120 round 2: 7 slim boxes (25 mm pitch, 175 mm) are also 3.2 boxes of the 55 mm side,
    # which fit within 4 mm, and were read as 3; at noise 6 as 3, 5 as 2 and 2 as 1.
    if CONFIG.free_depth_of_column(geometry, count) <= 0.0:
        pytest.skip("beyond capacity")
    lane = evidence(geometry, count, sku=BOX_SKU[geometry.name])
    result = est.estimate_lane_stock(lane, CONFIG)
    assert result.answered and result.count_estimate == count, result.reason_detail


@pytest.mark.parametrize("geometry", [CARTON_200, CARTON_1000, SLIM])
@pytest.mark.parametrize("count", [1, 2, 3, 4, 5, 6, 7, 8])
@pytest.mark.parametrize("sigma_mm", [2.0, 4.5, 9.0])
def test_noise_on_the_nearest_surface_never_gives_a_count_more_than_one_off(
    geometry, count, sigma_mm
):
    # The measured column length is off by up to 3 sigma and the bed sigma says so. The estimator
    # may refuse or be one product out; it must not name another product whose long pitch happens
    # to fit (6 slim boxes read as 3 cartons).
    free = CONFIG.free_depth_of_column(geometry, count)
    if free <= 0.0:
        pytest.skip("beyond capacity")
    base = column_profile([(geometry, count)])
    profile = est.TopProfile(
        base.near_depth_m, base.top_height_m, base.bin_width_m, bed_sigma_m=sigma_mm / 1000.0
    )
    for step in range(-12, 13):
        shift = step / 4.0 * sigma_mm / 1000.0
        lane = evidence(
            geometry,
            count,
            sku=BOX_SKU[geometry.name],
            free_depth_m=min(max(free + shift, 0.0), CONFIG.workcell.usable_depth_m),
            profile=profile,
        )
        result = est.estimate_lane_stock(lane, CONFIG)
        if result.answered:
            assert abs(result.count_estimate - count) <= 1, (shift, result.product_name)


def test_the_config_restates_the_box_catalogue_and_the_registered_skus():
    sys.path.insert(0, str(Path(__file__).resolve().parents[2] / "restocker_gazebo"))
    from restocker_gazebo import lane_columns as gazebo

    for key, entry in gazebo.BOX_CATALOGUE.items():
        mine = CONFIG.products[key.split(".", 1)[1]]
        assert mine.size_xyz_m == tuple(entry["size_xyz_m"])
        assert (mine.radius_m, mine.height_m) == (entry["radius_m"], entry["height_m"])
        for count in (0, 1, 4):
            assert CONFIG.free_depth_of_column(mine, count) == pytest.approx(
                gazebo.ground_truth_available_depth_m(key, count)
            )
    catalogue = yaml.safe_load(
        (Path(__file__).resolve().parents[2] / "restocker_gazebo" / "config"
         / "generated_product_labels.yaml").read_text(encoding="utf-8")
    )["products"]
    registered = {
        row["sku"]: row["box_variant"] for row in catalogue if row.get("shape") == "box"
    }
    assert {sku: CONFIG.sku_products[sku] for sku in registered} == registered
    assert CONFIG.product_by_enum(CLASS_ENUM["box"]) is None

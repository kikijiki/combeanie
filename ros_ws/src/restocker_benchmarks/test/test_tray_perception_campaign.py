# Copyright 2026 Matteo Bernacchia <dev@kikijiki.com>
#
# SPDX-License-Identifier: MIT

"""The predeclared matrix, budget arithmetic, and summary must hold without a simulator."""

import os
from pathlib import Path
import sys

import pytest
import yaml

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from restocker_benchmarks import tray_perception_campaign as campaign  # noqa: E402,I100,I101

# restocker_gazebo is a sibling package; importable when the workspace is installed so product
# counts can be pinned to the arrangement builders rather than restated twice.
try:
    from restocker_gazebo import tray_arrangements as tray  # noqa: E402,I100,I101
except ImportError:  # pragma: no cover - offline import without the gazebo package
    tray = None


def _yaml_env(name: str) -> dict:
    path = os.environ.get(name)
    if not path:
        pytest.skip(f"{name} is not set")
    return yaml.safe_load(Path(path).read_text(encoding="utf-8"))


def _diagonal_covariance(sigma_m: float) -> list[float]:
    """Build a row-major 6×6 covariance claiming sigma_m on every translational axis."""
    covariance = [0.0] * 36
    for index in (0, 7, 14):
        covariance[index] = sigma_m * sigma_m
    for index in (21, 28, 35):
        covariance[index] = 0.01
    return covariance


def test_geometry_keys_match_the_shipped_collision_catalogue():
    """Every budget geometry restates the catalogue's SKU, class, radius and height."""
    catalog = _yaml_env("RESTOCKER_TEST_PRODUCT_CATALOG")
    shipped = {entry["geometry_key"]: entry for entry in catalog["geometries"]}
    assert set(campaign.GEOMETRY) == set(shipped)
    for geometry_key, entry in campaign.GEOMETRY.items():
        assert shipped[geometry_key]["sku"] == entry["sku"]
        assert shipped[geometry_key]["product_class"] == entry["product_class"]
        assert shipped[geometry_key]["shape"]["radius_m"] == pytest.approx(entry["radius_m"])


def test_budget_constants_match_the_shipped_boundary_and_gripper():
    """SC-002's comparator is the production contract, not a lookalike constant set."""
    tolerances = _yaml_env("RESTOCKER_TEST_ATTACHMENT_BOUNDARY")["tolerances"]
    attachment = _yaml_env("RESTOCKER_TEST_GRIPPER_GEOMETRY")["attachment"]
    finger = _yaml_env("RESTOCKER_TEST_GRIPPER_GEOMETRY")["finger"]
    assert pytest.approx(tolerances["fidelity_sigma_multiplier"]) == (
        campaign.FIDELITY_SIGMA_MULTIPLIER
    )
    assert pytest.approx(tolerances["fidelity_mechanical_margin_m"]) == (
        campaign.FIDELITY_MECHANICAL_MARGIN_M
    )
    assert pytest.approx(tolerances["expected_translation_m"]) == (
        campaign.FIXED_TRANSLATION_TOLERANCE_M
    )
    assert pytest.approx(attachment["open_target_m"]) == campaign.GRIPPER_OPEN_TARGET_M
    assert pytest.approx(attachment["minimum_inner_clearance_m"]) == (
        campaign.GRIPPER_MINIMUM_INNER_CLEARANCE_M
    )
    assert pytest.approx(attachment["hold_clearance_per_side_m"]) == (
        campaign.GRIPPER_HOLD_CLEARANCE_PER_SIDE_M
    )
    # Zero gap is the distance between the finger boxes' inner faces at joint zero.
    half_thickness = finger["size_xyz_m"][1] / 2.0
    left_inner = finger["left"]["origin_xyz_m"][1] - half_thickness
    right_inner = finger["right"]["origin_xyz_m"][1] + half_thickness
    assert pytest.approx(left_inner - right_inner) == campaign.GRIPPER_ZERO_GAP_M


def test_wrist_tray_duties_declare_only_the_class_fallback_skus():
    """One SKU per class in the wrist config — the identity boundary citrus is measured against."""
    catalog = _yaml_env("RESTOCKER_TEST_PRODUCT_CATALOG")
    fallbacks = {
        entry["product_class"]: entry["sku"]
        for entry in catalog["geometries"]
        if entry.get("class_fallback")
    }
    tray_config = _yaml_env("RESTOCKER_TEST_TRAY_PERCEPTION_CONFIG")
    for duty in ("tray_overview_perception", "tray_confirm_perception"):
        parameters = tray_config[duty]["ros__parameters"]
        declared = {name: parameters[f"{name}.sku"] for name in parameters["product_classes"]}
        assert declared == fallbacks == campaign.WRIST_TRAY_DECLARED_SKUS, duty
        # citrus is deliberately absent: a second SKU of one class cannot be expressed.
        assert "SIM-CAN-CITRUS" not in declared.values()


def test_the_matrix_covers_every_catalogued_product_at_both_duties():
    """Cell A must cross all four geometry keys with overview and confirm."""
    cell = campaign.cell_by_id("A-isolated")
    assert set(cell.geometry_keys) == set(campaign.GEOMETRY)
    assert cell.duties == campaign.DUTIES
    assert cell.seeds == campaign.ISOLATED_SEEDS


def test_the_matrix_declares_every_feature_and_the_extrinsics_run():
    """Touching, neighbour, non-upright and Q7 extrinsics cells are all predeclared."""
    features = {cell.feature for cell in campaign.predeclared_matrix()}
    assert {"isolated", "touching_pair", "close_neighbour", "non_upright"} <= features
    extrinsics = campaign.cell_by_id("E-extrinsics")
    assert extrinsics.duties == ("confirm",)
    assert campaign.EXTRINSICS_CONFIGURATION_COUNT >= 2


def test_the_predeclared_launch_budget_matches_the_card():
    """12 launches: 3 isolated seeds + 3 touching + 4 neighbours + 1 non-upright + 1 extrinsics."""
    assert campaign.expected_arrangement_launches() == 12


def test_expected_products_match_the_arrangement_builders():
    """Metric 3's denominator source is the builder, not an estimate of len(rows)."""
    table = {
        "A-isolated": 4 * 3,
        "B-touching-can": 2,
        "B-touching-small": 2,
        "B-touching-large": 2,
        "C-neighbour-can.standard": 2,
        "C-neighbour-can.citrus": 2,
        "C-neighbour-bottle.small.standard": 2,
        "C-neighbour-bottle.large.standard": 2,
        "D-non-upright": 1,
        "E-extrinsics": 1,
    }
    for cell_id, expected in table.items():
        assert campaign.expected_products_for_cell(cell_id) == expected, cell_id
    if tray is None:
        pytest.skip("restocker_gazebo is not importable in this environment")
    assert campaign.products_per_arrangement("A-isolated") == len(
        tray.isolated_catalogue_scenario(0)["products"]
    )
    for cell_id in table:
        if cell_id == "A-isolated":
            continue
        cell = campaign.cell_by_id(cell_id)
        if cell.cell_id.startswith("B-touching-"):
            keys = cell.geometry_keys
            scenario = (
                tray.touching_pair_scenario(keys[0], keys[0])
                if len(keys) == 1
                else tray.touching_pair_scenario(keys[0], keys[1])
            )
        elif cell.cell_id.startswith("C-neighbour-"):
            scenario = tray.close_neighbour_scenario(cell.geometry_keys[0])
        elif cell.cell_id == "D-non-upright":
            scenario = tray.non_upright_scenario(cell.geometry_keys[0])
        else:
            scenario = tray.extrinsics_scenario(cell.geometry_keys[0])
        assert campaign.products_per_arrangement(cell_id) == len(scenario["products"]), cell_id


def test_range_bands_match_pose_error_evaluation():
    """Confirm edge at 0.50 m, overview edge at 1.20 m, non-finite is unknown not far."""
    assert campaign.classify_observation_range(0.30) == "confirm"
    assert campaign.classify_observation_range(0.499999) == "confirm"
    assert campaign.classify_observation_range(0.60) == "overview"
    assert campaign.classify_observation_range(1.199999) == "overview"
    assert campaign.classify_observation_range(2.06) == "far"
    assert campaign.classify_observation_range(float("nan")) == "unknown"
    assert campaign.classify_observation_range(-0.1) == "unknown"


def test_hold_targets_match_attachment_config_fixture_numbers():
    """0.0145 / 0.0155 / 0.0265, the numbers test_attachment_config.cpp pins."""
    assert campaign.hold_joint_target_m(0.033) == pytest.approx(0.0145, abs=1e-12)
    assert campaign.hold_joint_target_m(0.034) == pytest.approx(0.0155, abs=1e-12)
    assert campaign.hold_joint_target_m(0.045) == pytest.approx(0.0265, abs=1e-12)


def test_fidelity_budget_formula_matches_the_boundary_tests():
    """3 sigma + margin at 0.004 is 0.013; the can ceiling is 0.017; a huge claim hits it."""
    sigma = 0.004
    covariance = _diagonal_covariance(sigma)
    can = campaign.attachment_translation_budget_m(covariance, "can.standard")
    assert can is not None
    assert abs(can - (3.0 * sigma + 0.001)) < 1e-12
    assert abs(campaign.fidelity_translation_ceiling_m(0.033) - 0.017) < 1e-12

    confident = campaign.attachment_translation_budget_m(
        _diagonal_covariance(0.0001), "can.standard"
    )
    assert confident is not None
    assert confident < campaign.FIXED_TRANSLATION_TOLERANCE_M

    huge = campaign.attachment_translation_budget_m(_diagonal_covariance(1.0), "can.standard")
    assert huge is not None
    assert abs(huge - campaign.fidelity_translation_ceiling_m(0.033)) < 1e-12


def test_an_unstated_or_malformed_covariance_has_no_budget():
    """Zero covariance is unstated; a non-finite or non-PSD claim is refused, not trusted."""
    assert campaign.claimed_translation_sigma_m([0.0] * 36) is None
    broken = [0.0] * 36
    broken[0] = float("nan")
    assert campaign.claimed_translation_sigma_m(broken) is None
    indefinite = [0.0] * 36
    indefinite[0] = 1.0
    indefinite[7] = -1.0
    indefinite[14] = 1.0
    assert campaign.claimed_translation_sigma_m(indefinite) is None
    assert campaign.attachment_translation_budget_m([0.0] * 36, "can.standard") is None


def test_identity_outcome_distinguishes_match_class_and_mismatch():
    """SKU match beats class match; a wrong SKU on the right class is class_only, not match."""
    assert campaign.identity_outcome("SIM-CAN-STD", "SIM-CAN-STD", "can", "can") == "match"
    assert campaign.identity_outcome("SIM-CAN-STD", "SIM-CAN-CITRUS", "can", "can") == "class_only"
    assert (
        campaign.identity_outcome(
            "SIM-BOTTLE-SMALL", "SIM-BOTTLE-LARGE", "small_bottle", "large_bottle"
        )
        == "mismatch"
    )
    assert campaign.identity_outcome("SIM-CAN-STD", "", "can", "") == "none"


def test_orientation_axis_consistency_enforces_metric_five():
    """NOT_ESTIMATED forbids a finite axis error, including a numeric zero."""
    assert campaign.orientation_axis_consistent({"orientation_status": 0, "axis_error_rad": None})
    assert campaign.orientation_axis_consistent(
        {"orientation_status": 0, "axis_error_rad": float("nan")}
    )
    assert not campaign.orientation_axis_consistent(
        {"orientation_status": 0, "axis_error_rad": 0.0}
    )
    assert not campaign.orientation_axis_consistent(
        {"orientation_status": 0, "axis_error_rad": 0.12}
    )
    assert campaign.orientation_axis_consistent({"orientation_status": 1, "axis_error_rad": 0.05})
    assert not campaign.orientation_axis_consistent(
        {"orientation_status": 1, "axis_error_rad": None}
    )
    assert not campaign.orientation_axis_consistent(
        {"orientation_status": 9, "axis_error_rad": 0.1}
    )


def _confirm_row(**overrides):
    """One admitted confirm-band OK row with a usable covariance claim."""
    row = {
        "cell_id": "A-isolated",
        "feature": "isolated",
        "seed": 0,
        "duty": "confirm",
        "geometry_key": "can.standard",
        "sku_expected": "SIM-CAN-STD",
        "sku_reported": "SIM-CAN-STD",
        "identity": "match",
        "status": 0,
        "orientation": "upright",
        "translation_error_m": 0.001,
        "observation_range_m": 0.30,
        "range_band": "confirm",
        "axis_error_rad": None,
        "orientation_status": 0,
        "confidence": 0.9,
        "covariance": _diagonal_covariance(0.002),
        "backend_name": "wrist_rgbd_tray_confirm",
        "source_object_id": "sim:tray_can_01",
        "run_label": "seed-0",
        "commit": "deadbeef",
        "residual_xyz_m": [0.0005, -0.0003, 0.0008],
    }
    row.update(overrides)
    return row


def test_summary_counts_bands_and_budget_with_fail_closed_in_the_denominator():
    """Admitted confirm rows enter SC-002; unusable covariance stays eligible as fail_closed."""
    rows = [
        _confirm_row(),
        _confirm_row(translation_error_m=0.020),  # over budget for sigma 2 mm (budget 7 mm)
        _confirm_row(
            duty="overview",
            observation_range_m=0.70,
            range_band="overview",
            backend_name="wrist_rgbd_tray_overview",
            translation_error_m=0.004,
        ),
        _confirm_row(status=1, identity="none", translation_error_m=None, covariance=None),
        # Fail-closed: non-PSD covariance, still an admitted confirm observation.
        _confirm_row(covariance=[-1.0 if i == 0 else 0.0 for i in range(36)]),
        # Fail-closed: non-finite error.
        _confirm_row(translation_error_m=float("nan")),
    ]
    summary = campaign.summarise(rows)
    assert summary["n_rows"] == 6
    # Eligible = 4 admitted confirm-duty rows with can.standard (two ok, two fail-closed).
    assert summary["budget"]["eligible"] == 4
    assert summary["budget"]["n"] == 2
    assert summary["budget"]["fail_closed"] == 2
    assert summary["budget"]["within_budget"] == 1
    assert summary["budget"]["within_budget_fraction"] == 0.25
    assert summary["budget"]["within_fixed_3mm"] == 1
    assert summary["budget"]["within_fixed_3mm_fraction"] == 0.25
    # Refusal is status-only, over expected products × duties for the launches present.
    # One launch (A-isolated seed 0): 4 products × 2 duties = 8 expected rows; 1 refused.
    assert summary["refusal"]["refused"] == 1
    assert summary["refusal"]["expected_observation_rows"] == 8
    assert summary["refusal"]["rate"] == 0.125
    assert summary["refusal_rate"] == 0.125
    assert summary["refusal"]["admitted_identity_none"] == 0
    assert summary["identity"]["match"] == 5
    assert summary["orientation_schema_violations"] == 0
    assert summary["missing_cells"] != []
    text = campaign.format_summary(summary)
    assert "SC-002 confirm-band attachment budget" in text
    assert "fail_closed=2" in text
    assert "refusal rate: 0.125" in text
    assert "expected observation rows" in text


def test_admitted_identity_none_is_not_counted_as_a_refusal():
    """Metric 3 vs metric 4: identity=none on an accepted observation is identity, not refusal."""
    rows = [
        _confirm_row(identity="none", sku_reported="", class_reported=""),
        _confirm_row(),
    ]
    summary = campaign.summarise(rows)
    assert summary["refusal"]["refused"] == 0
    assert summary["refusal"]["admitted_identity_none"] == 1
    assert summary["identity"]["none"] == 1
    assert summary["refusal"]["expected_observation_rows"] == 8
    assert summary["refusal"]["rate"] == 0.0


def test_refusal_rate_uses_expected_products_not_row_count():
    """A log with fewer rows than the arrangement expects must not inflate the rate to 1.0."""
    # Only one refusal logged, but the A-isolated seed-0 launch still expects 4×2 = 8 rows.
    rows = [
        _confirm_row(status=1, identity="none", translation_error_m=None, covariance=None),
    ]
    summary = campaign.summarise(rows)
    assert summary["n_rows"] == 1
    assert summary["refusal"]["expected_observation_rows"] == 8
    assert summary["refusal"]["rate"] == 0.125


def test_metric_five_counts_a_numeric_zero_axis_under_not_estimated():
    """A finite axis error under ORIENTATION_NOT_ESTIMATED is a schema violation."""
    rows = [
        _confirm_row(axis_error_rad=0.0),
        _confirm_row(axis_error_rad=None),
        _confirm_row(axis_error_rad=float("nan")),
    ]
    summary = campaign.summarise(rows)
    assert summary["orientation_schema_violations"] == 1
    assert "orientation schema violations (metric 5): 1" in campaign.format_summary(summary)


def test_metric_five_applies_to_refused_rows_too():
    """Metric 5 is about what a row reports, not whether the attempt was admitted."""
    rows = [
        _confirm_row(status=1, axis_error_rad=0.0, translation_error_m=None, covariance=None),
        _confirm_row(status=1, axis_error_rad=None, translation_error_m=None, covariance=None),
    ]
    summary = campaign.summarise(rows)
    assert summary["orientation_schema_violations"] == 1


def test_refusal_ignores_rows_that_do_not_attribute_to_a_predeclared_launch():
    """An unknown cell_id must not sit in the numerator without a denominator share."""
    rows = [
        _confirm_row(status=1, identity="none", translation_error_m=None, covariance=None),
        _confirm_row(
            cell_id="NOT-A-PREDECLARED-CELL",
            status=1,
            identity="none",
            translation_error_m=None,
            covariance=None,
        ),
    ]
    summary = campaign.summarise(rows)
    assert summary["refusal"]["refused"] == 1
    assert summary["refusal"]["expected_observation_rows"] == 8
    assert summary["refusal"]["rate"] == 0.125
    assert summary["n_rows"] == 2


def test_null_seed_does_not_crash_the_refusal_denominator():
    """JSON null seed is unattributable; it must not raise out of summarise()."""
    rows = [_confirm_row(seed=None, status=1, translation_error_m=None, covariance=None)]
    summary = campaign.summarise(rows)
    assert summary["refusal"]["refused"] == 0
    assert summary["refusal"]["expected_observation_rows"] == 0
    assert summary["refusal"]["rate"] is None
    assert summary["n_rows"] == 1


def test_string_status_zero_is_admitted_like_the_numeric_status():
    """A collector that stringifies uint8 status must not turn every row into a refusal."""
    summary = campaign.summarise([_confirm_row(status="0")])
    assert summary["refusal"]["refused"] == 0
    assert summary["refusal"]["rate"] == 0.0
    assert summary["budget"]["eligible"] == 1


def test_q7_reports_residual_vector_spread_not_only_scalar_error():
    """Metric 9 requires residual_xyz_m spread alongside translation_error spread."""
    rows = [
        _confirm_row(
            cell_id="E-extrinsics",
            feature="isolated",
            arm_configuration="ik0",
            run_label="cfg-0",
            translation_error_m=0.001,
            residual_xyz_m=[0.001, 0.000, -0.002],
            observation_range_m=0.30,
        ),
        _confirm_row(
            cell_id="E-extrinsics",
            feature="isolated",
            arm_configuration="ik1",
            run_label="cfg-1",
            translation_error_m=0.004,
            residual_xyz_m=[0.004, 0.002, 0.001],
            observation_range_m=0.30,
        ),
        _confirm_row(
            cell_id="E-extrinsics",
            feature="isolated",
            arm_configuration="ik2",
            run_label="cfg-2",
            translation_error_m=0.002,
            residual_xyz_m=[0.002, -0.001, 0.000],
            observation_range_m=0.30,
        ),
    ]
    summary = campaign.summarise(rows)
    extrinsics = summary["extrinsics"]
    assert extrinsics["n"] == 3
    assert extrinsics["configurations"] == 3
    assert extrinsics["spread_m"] == pytest.approx(0.003)
    assert extrinsics["std_m"] > 0.0
    residual = extrinsics["residual"]
    assert residual is not None
    assert residual["n"] == 3
    # Per-axis max−min over the three residual vectors.
    assert residual["axis_spread_m"][0] == pytest.approx(0.003)
    assert residual["axis_spread_m"][1] == pytest.approx(0.003)
    assert residual["axis_spread_m"][2] == pytest.approx(0.003)
    assert residual["norm_spread_m"] > 0.0
    assert residual["norm_std_m"] > 0.0
    assert "axis_spread_m" in campaign.format_summary(summary)


def test_q7_residual_block_is_none_only_when_no_vector_is_usable():
    """Scalar spread still reports; residual stays None when no residual_xyz_m is present."""
    rows = [
        _confirm_row(
            cell_id="E-extrinsics",
            arm_configuration="ik0",
            residual_xyz_m=None,
            translation_error_m=0.002,
        ),
        _confirm_row(
            cell_id="E-extrinsics",
            arm_configuration="ik1",
            residual_xyz_m=None,
            translation_error_m=0.003,
        ),
    ]
    summary = campaign.summarise(rows)
    assert summary["extrinsics"]["residual"] is None
    assert summary["extrinsics"]["spread_m"] == pytest.approx(0.001)

    one_vector = [
        _confirm_row(
            cell_id="E-extrinsics",
            arm_configuration="ik0",
            residual_xyz_m=[0.002, 0.0, 0.0],
            translation_error_m=0.002,
        ),
        _confirm_row(
            cell_id="E-extrinsics",
            arm_configuration="ik1",
            residual_xyz_m=None,
            translation_error_m=0.003,
        ),
    ]
    partial = campaign.summarise(one_vector)
    assert partial["extrinsics"]["residual"]["n"] == 1
    assert partial["extrinsics"]["residual"]["axis_spread_m"] == [0.0, 0.0, 0.0]


def test_summary_reports_missing_cells_when_the_log_is_empty():
    """An empty log still lists every predeclared cell so coverage cannot be silently skipped."""
    summary = campaign.summarise([])
    assert set(summary["missing_cells"]) == {
        cell.cell_id for cell in campaign.predeclared_matrix()
    }
    assert summary["budget"] == {}
    assert summary["refusal"]["rate"] is None


def test_q11_ignores_touching_rows_that_omit_the_pair_count():
    """A missing observations_for_pair is not a measured zero; zero means a refused pair."""
    rows = [
        _confirm_row(feature="touching_pair", duty="overview", observation_range_m=0.70),
        _confirm_row(
            feature="touching_pair",
            duty="overview",
            observation_range_m=0.70,
            observations_for_pair=2,
        ),
        _confirm_row(
            feature="touching_pair",
            duty="overview",
            observation_range_m=0.70,
            observations_for_pair=0,
        ),
    ]
    summary = campaign.summarise(rows)
    # Omitted field skipped; explicit 0 (measured refusal) and 2 (separated) both counted.
    assert summary["touching_pair_detection_counts"] == {"A-isolated": {2: 1, 0: 1}}


def test_q12_separates_refusal_from_identity_outcomes():
    """Bottle breakdown reports refused (status) and identity counts as separate keys."""
    rows = [
        _confirm_row(
            geometry_key="bottle.small.standard",
            sku_expected="SIM-BOTTLE-SMALL",
            sku_reported="SIM-BOTTLE-SMALL",
            duty="overview",
            observation_range_m=0.70,
        ),
        _confirm_row(
            geometry_key="bottle.small.standard",
            sku_expected="SIM-BOTTLE-SMALL",
            sku_reported="",
            class_reported="",
            identity="none",
            duty="overview",
            observation_range_m=0.70,
        ),
        _confirm_row(
            geometry_key="bottle.large.standard",
            sku_expected="SIM-BOTTLE-LARGE",
            sku_reported="",
            status=1,
            identity="none",
            duty="confirm",
            translation_error_m=None,
            covariance=None,
        ),
    ]
    summary = campaign.summarise(rows)
    overview = summary["bottle_identity_by_duty"]["overview"]
    assert overview["refused"] == 0
    assert overview["identity"] == {"match": 1, "none": 1}
    confirm = summary["bottle_identity_by_duty"]["confirm"]
    assert confirm["refused"] == 1
    assert confirm["identity"] == {"none": 1}


def test_format_matrix_lists_every_cell():
    """The printable matrix is stable text containing every cell id."""
    text = campaign.format_matrix()
    for cell in campaign.predeclared_matrix():
        assert cell.cell_id in text
    assert "predeclared simulator launches: 12" in text

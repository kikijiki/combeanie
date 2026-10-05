# Copyright 2026 Matteo Bernacchia <dev@kikijiki.com>
#
# SPDX-License-Identifier: MIT

"""Predeclared wrist-tray perception measurement matrix and offline aggregation."""
# The matrix is frozen before any campaign sample exists so acceptance cannot be refit to
# results. The module is pure: no ROS imports, so a stored campaign re-summarises after the
# workspace that produced it is gone.
#
# The attachment-budget half restates attachment_physics.cpp and attachment_boundary.yaml /
# gripper_geometry.yaml. test_tray_perception_campaign.py pins the restatement against those
# files' own fixture numbers, the same way lane_columns.py is pinned to the workcell.
#
# ## Per-sample row schema
#
# Every admitted or refused observation attempt appends one JSONL object. Required fields:
#
# - ``cell_id``, ``feature``, ``seed`` — which predeclared matrix cell the attempt belongs to
#   (``seed`` is -1 for fixed constructions).
# - ``duty`` — ``overview`` or ``confirm``.
# - ``geometry_key``, ``sku_expected`` — the arrangement's ground truth for this product.
# - ``sku_reported``, ``class_reported``, ``identity`` — what the backend claimed, classified as
#   ``match`` | ``class_only`` | ``mismatch`` | ``none``. Admitted ``identity=none`` means the
#   observation was accepted but carried no SKU and no class; it is **not** a refusal.
# - ``status`` — ObjectObservation status; anything other than ``STATUS_OK`` is a **refusal**
#   (no admitted observation for that attempt).
# - ``orientation``, ``orientation_status``, ``axis_error_rad`` — orientation label; when
#   ``orientation_status`` is ``ORIENTATION_NOT_ESTIMATED`` the axis error must be absent or
#   non-finite (NaN), never a finite number including 0.0 (metric 5).
# - ``translation_error_m``, ``observation_range_m``, ``range_band`` — pose-error sample fields.
# - ``residual_xyz_m`` — optional ``[dx, dy, dz]`` estimate-minus-ground-truth residual in the
#   planning frame (metric 9 / Q7 vector spread). Absent when the evaluator did not emit one.
# - ``covariance`` — row-major 6×6 pose covariance. Malformed, non-finite, or non-PSD claims are
#   **fail-closed** in SC-002: they stay in the eligible denominator and never count as within
#   budget.
# - ``observations_for_pair`` — **required** on ``touching_pair`` overview rows only (Q11 /
#   metric 6): the number of distinct observations associated with the two ground-truth
#   positions. ``2`` = separated, ``1`` = merged, ``0`` = measured refusal of the pair. Rows
#   that omit the field are ignored by that metric rather than defaulted to 0.
# - ``backend_name``, ``source_object_id``, ``run_label``, ``commit``, ``arm_configuration`` —
#   provenance; ``arm_configuration`` distinguishes Q7's six IK-distinct setups when present.
#
# ## Summary metric definitions (frozen)
#
# - **Refusal rate** (metric 3): refused attempts ÷ expected observation rows. Expected rows for
#   a logged ``(cell_id, seed)`` = products the arrangement builder stands × duties on that cell.
#   Refused = status not equal to ``STATUS_OK`` (numeric or numeric-string status). Admitted
#   ``identity=none`` is counted under identity, not as a refusal. Only rows that attribute to a
#   predeclared ``(cell_id, seed)`` enter either side of the fraction.
# - **SC-002** (metric 2): every admitted confirm-duty ``status==OK`` row with a known geometry
#   is eligible. Within budget only when both the covariance claim and the error are usable;
#   anything else increments ``fail_closed`` and stays outside ``within_budget``.
# - **Q7** (metric 9): spread of ``translation_error_m`` **and** of ``residual_xyz_m`` across
#   arm configurations at one commanded optical pose.

from __future__ import annotations

from collections import Counter
from collections.abc import Iterable, Sequence
from dataclasses import dataclass
import json
import math
from pathlib import Path
import statistics
from typing import Any

# Observation-range bands, matching pose_error_evaluation.hpp. Confirm is the close view that
# authorises a grasp; overview is a tray station; far is everything else (the overhead camera).
CONFIRM_RANGE_MAXIMUM_M = 0.50
OVERVIEW_RANGE_MAXIMUM_M = 1.20

# fidelity_sigma_multiplier and fidelity_mechanical_margin_m from attachment_boundary.yaml.
FIDELITY_SIGMA_MULTIPLIER = 3.0
FIDELITY_MECHANICAL_MARGIN_M = 0.001
# expected_translation_m: the constant used only when a caller states no covariance. Recorded so
# the summary can compare against it; the budget below is the acceptance comparator.
FIXED_TRANSLATION_TOLERANCE_M = 0.003

# gripper_geometry.yaml attachment block, restated.
GRIPPER_OPEN_TARGET_M = 0.032
GRIPPER_MINIMUM_INNER_CLEARANCE_M = 0.0005
GRIPPER_HOLD_CLEARANCE_PER_SIDE_M = 0.0025
# left_inner_face_at_zero_m - right_inner_face_at_zero_m (finger boxes at ±0.03).
GRIPPER_ZERO_GAP_M = 0.042

# PoseErrorSample.orientation_status values.
ORIENTATION_NOT_ESTIMATED = 0
ORIENTATION_AXIS_ESTIMATED = 1

# ObjectObservation.STATUS_OK
STATUS_OK = 0

# Geometry keys from product_collision_catalog.yaml, with the cylinder shape a budget needs.
GEOMETRY: dict[str, dict[str, Any]] = {
    "can.standard": {"product_class": "can", "sku": "SIM-CAN-STD", "radius_m": 0.033},
    "can.citrus": {"product_class": "can", "sku": "SIM-CAN-CITRUS", "radius_m": 0.033},
    "bottle.small.standard": {
        "product_class": "small_bottle",
        "sku": "SIM-BOTTLE-SMALL",
        "radius_m": 0.034,
    },
    "bottle.large.standard": {
        "product_class": "large_bottle",
        "sku": "SIM-BOTTLE-LARGE",
        "radius_m": 0.045,
    },
}

# Same-class geometries that may form a constructed touching pair (Q11).
TOUCHING_PAIR_CLASSES: tuple[tuple[str, str], ...] = (
    ("can.standard", "can.citrus"),
    ("bottle.small.standard", "bottle.small.standard"),
    ("bottle.large.standard", "bottle.large.standard"),
)

DUTIES: tuple[str, ...] = ("overview", "confirm")
FEATURES: tuple[str, ...] = ("isolated", "touching_pair", "close_neighbour", "non_upright")
ISOLATED_SEEDS: tuple[int, ...] = (0, 1, 2)
# Q7: arm configurations that place the camera at one commanded optical pose.
EXTRINSICS_CONFIGURATION_COUNT = 6

# The wrist tray duty config declares exactly one SKU per product class — the class-fallback
# entry from product_collision_catalog.yaml. can.citrus shares can.standard's shape and differs
# only by label, so under this backend the honest outcomes are: pose still fits, reported SKU is
# SIM-CAN-STD, and two cans of a class are named by their declared stocking positions (Card 063;
# until then the one-object-per-class assigner dropped them). The matrix still stands both cans
# so that boundary is measured rather than assumed.
WRIST_TRAY_DECLARED_SKUS: dict[str, str] = {
    "can": "SIM-CAN-STD",
    "small_bottle": "SIM-BOTTLE-SMALL",
    "large_bottle": "SIM-BOTTLE-LARGE",
}
# Label textures for the same-shape can experiment (tools/generate_product_labels.py).
CAN_LABEL_TEXTURES: dict[str, str] = {
    "can.standard": "can_classic_label.png",
    "can.citrus": "can_citrus_label.png",
}


@dataclass(frozen=True)
class MatrixCell:
    """One predeclared analysis cell: a feature, geometry coverage, and seed policy."""

    cell_id: str
    feature: str
    geometry_keys: tuple[str, ...]
    seeds: tuple[int, ...]
    duties: tuple[str, ...] = DUTIES
    # Fixed constructions use seed -1 so a missing seed is visible rather than defaulted.
    fixed_construction: bool = False
    notes: str = ""


def predeclared_matrix() -> tuple[MatrixCell, ...]:
    """Return the frozen measurement matrix; every summary checks coverage against this."""
    geometries = tuple(GEOMETRY)
    return (
        MatrixCell(
            cell_id="A-isolated",
            feature="isolated",
            geometry_keys=geometries,
            seeds=ISOLATED_SEEDS,
            notes="catalogue sweep: spaced upright products, three tray-layout seeds",
        ),
        MatrixCell(
            cell_id="B-touching-can",
            feature="touching_pair",
            geometry_keys=(TOUCHING_PAIR_CLASSES[0][0], TOUCHING_PAIR_CLASSES[0][1]),
            seeds=(-1,),
            fixed_construction=True,
            notes="Q11: same-class pair with centres at r1+r2",
        ),
        MatrixCell(
            cell_id="B-touching-small",
            feature="touching_pair",
            geometry_keys=(TOUCHING_PAIR_CLASSES[1][0],),
            seeds=(-1,),
            fixed_construction=True,
            notes="Q11: two touching small bottles",
        ),
        MatrixCell(
            cell_id="B-touching-large",
            feature="touching_pair",
            geometry_keys=(TOUCHING_PAIR_CLASSES[2][0],),
            seeds=(-1,),
            fixed_construction=True,
            notes="Q11: two touching large bottles",
        ),
        *(
            MatrixCell(
                cell_id=f"C-neighbour-{key}",
                feature="close_neighbour",
                geometry_keys=(key,),
                seeds=(-1,),
                fixed_construction=True,
                notes="Q12: target plus same-class neighbour at the approach/shadow bound",
            )
            for key in geometries
        ),
        MatrixCell(
            cell_id="D-non-upright",
            feature="non_upright",
            geometry_keys=("bottle.large.standard",),
            seeds=(-1,),
            fixed_construction=True,
            notes="Q13: one horizontal large bottle; upright grasp gate must stay refused",
        ),
        MatrixCell(
            cell_id="E-extrinsics",
            feature="isolated",
            geometry_keys=("can.standard",),
            seeds=(-1,),
            duties=("confirm",),
            fixed_construction=True,
            notes=(
                f"Q7: one commanded optical pose from {EXTRINSICS_CONFIGURATION_COUNT} arm "
                "configurations; spread is the arm's extrinsics contribution"
            ),
        ),
    )


def cell_by_id(cell_id: str) -> MatrixCell:
    """Return the predeclared cell with this id, or raise when the id is unknown."""
    for cell in predeclared_matrix():
        if cell.cell_id == cell_id:
            return cell
    raise KeyError(f"unknown matrix cell {cell_id!r}")


def products_per_arrangement(cell_id: str) -> int:
    """
    Return products the arrangement builder stands in one launch for this cell.

    Pinned to restocker_gazebo.tray_arrangements: A stands one product per geometry key;
    touching-pair and close-neighbour builders always stand two products even when the cell
    lists a single geometry (two bottles / target plus neighbour); non-upright and extrinsics
    stand one.
    """
    cell = cell_by_id(cell_id)
    if cell.cell_id == "A-isolated":
        return len(cell.geometry_keys)
    if cell.feature in ("touching_pair", "close_neighbour"):
        return 2
    return 1


def expected_products_for_cell(cell_id: str) -> int:
    """Products across every predeclared seed of this cell (arrangement builder × seeds)."""
    cell = cell_by_id(cell_id)
    return products_per_arrangement(cell_id) * len(cell.seeds)


def expected_arrangement_launches() -> int:
    """Return the predeclared launch count: A per seed, B/C/D once each, E once for six configs."""
    # A is one launch per seed of the whole catalogue tray; B/C/D are one launch each; E is one
    # launch that visits the six arm configurations, not six launches.
    cells = predeclared_matrix()
    isolated = next(cell for cell in cells if cell.cell_id == "A-isolated")
    constructions = sum(
        1 for cell in cells if cell.fixed_construction and cell.cell_id != "E-extrinsics"
    )
    extrinsics = sum(1 for cell in cells if cell.cell_id == "E-extrinsics")
    return len(isolated.seeds) + constructions + extrinsics


def classify_observation_range(range_m: float) -> str:
    """Classify a camera-to-product range into confirm, overview, far, or unknown."""
    if not math.isfinite(range_m) or range_m < 0.0:
        return "unknown"
    if range_m < CONFIRM_RANGE_MAXIMUM_M:
        return "confirm"
    if range_m < OVERVIEW_RANGE_MAXIMUM_M:
        return "overview"
    return "far"


def hold_joint_target_m(radius_m: float) -> float:
    """Return the per-product hold joint target attachment_config.cpp derives."""
    # 0.5 * (diameter + 2 * hold_clearance - zero_gap), from attachment_config.cpp.
    diameter = 2.0 * radius_m
    return 0.5 * (diameter + 2.0 * GRIPPER_HOLD_CLEARANCE_PER_SIDE_M - GRIPPER_ZERO_GAP_M)


def fidelity_translation_ceiling_m(radius_m: float) -> float:
    """Return open_target − hold_joint_target − minimum_inner_clearance for this product."""
    return (
        GRIPPER_OPEN_TARGET_M - hold_joint_target_m(radius_m) - GRIPPER_MINIMUM_INNER_CLEARANCE_M
    )


def claimed_translation_sigma_m(covariance: Sequence[float]) -> float | None:
    """
    Return sqrt of the largest translational eigenvalue, or None when the claim is unusable.

    Mirrors claimed_translation_uncertainty in attachment_physics.cpp: non-finite entries and
    non-positive-semidefinite translational blocks are refused rather than read as confidence.
    """
    if len(covariance) != 36:
        return None
    for entry in covariance:
        if not isinstance(entry, (int, float)) or not math.isfinite(float(entry)):
            return None
    if not any(float(entry) != 0.0 for entry in covariance):
        return None
    # Symmetric 3×3 translational block, row-major.
    xx, xy, xz = float(covariance[0]), float(covariance[1]), float(covariance[2])
    yx, yy, yz = float(covariance[6]), float(covariance[7]), float(covariance[8])
    zx, zy, zz = float(covariance[12]), float(covariance[13]), float(covariance[14])
    a = 0.5 * (xy + yx)
    b = 0.5 * (xz + zx)
    c = 0.5 * (yz + zy)
    eigenvalues = _symmetric_eigenvalues(xx, a, b, a, yy, c, b, c, zz)
    if eigenvalues is None:
        return None
    # A covariance is positive semi-definite; one that is not describes no distribution.
    negative_limit = -1.0e-12 * max(1.0, abs(eigenvalues[-1]))
    if eigenvalues[0] < negative_limit:
        return None
    return math.sqrt(max(eigenvalues[-1], 0.0))


def _symmetric_eigenvalues(
    m00: float,
    m01: float,
    m02: float,
    m10: float,
    m11: float,
    m12: float,
    m20: float,
    m21: float,
    m22: float,
) -> list[float] | None:
    """Return the three eigenvalues of a real symmetric 3×3 matrix, or None on failure."""
    # Jacobi rotation is stable and short for a 3×3; three sweeps of off-diagonal annihilation.
    a = [[m00, m01, m02], [m10, m11, m12], [m20, m21, m22]]
    # Symmetrise defensively.
    for i in range(3):
        for j in range(3):
            a[i][j] = 0.5 * (a[i][j] + a[j][i])
    for _ in range(32):
        off = math.sqrt(a[0][1] ** 2 + a[0][2] ** 2 + a[1][2] ** 2)
        if off < 1.0e-15:
            break
        # Rotate the largest off-diagonal pair.
        pairs = ((0, 1), (0, 2), (1, 2))
        p, q = max(pairs, key=lambda ij: abs(a[ij[0]][ij[1]]))
        if abs(a[p][q]) < 1.0e-18:
            break
        tau = (a[q][q] - a[p][p]) / (2.0 * a[p][q])
        t = math.copysign(1.0, tau) / (abs(tau) + math.sqrt(1.0 + tau * tau))
        c = 1.0 / math.sqrt(1.0 + t * t)
        s = t * c
        for k in range(3):
            if k in (p, q):
                continue
            akp, akq = a[k][p], a[k][q]
            a[k][p] = a[p][k] = c * akp - s * akq
            a[k][q] = a[q][k] = s * akp + c * akq
        app, aqq, apq = a[p][p], a[q][q], a[p][q]
        a[p][p] = c * c * app - 2.0 * s * c * apq + s * s * aqq
        a[q][q] = s * s * app + 2.0 * s * c * apq + c * c * aqq
        a[p][q] = a[q][p] = 0.0
    eigenvalues = sorted((a[0][0], a[1][1], a[2][2]))
    if not all(math.isfinite(value) for value in eigenvalues):
        return None
    return eigenvalues


def attachment_translation_budget_m(
    covariance: Sequence[float], geometry_key: str
) -> float | None:
    """
    Return the covariance-derived fidelity budget, or None when the claim is unusable.

    budget = min(multiplier * sigma + margin, ceiling(geometry)); a caller that states no
    covariance is held to FIXED_TRANSLATION_TOLERANCE_M by the boundary, which is not this path.
    """
    if geometry_key not in GEOMETRY:
        return None
    sigma = claimed_translation_sigma_m(covariance)
    if sigma is None:
        return None
    claimed = FIDELITY_SIGMA_MULTIPLIER * sigma + FIDELITY_MECHANICAL_MARGIN_M
    ceiling = fidelity_translation_ceiling_m(GEOMETRY[geometry_key]["radius_m"])
    return min(claimed, ceiling)


def identity_outcome(
    sku_expected: str, sku_reported: str, class_expected: str, class_reported: str
) -> str:
    """Classify an observation's identity against the arrangement's ground-truth expectation."""
    if not sku_reported and not class_reported:
        return "none"
    if sku_reported and sku_reported == sku_expected:
        return "match"
    if class_reported and class_reported == class_expected:
        return "class_only"
    if sku_reported or class_reported:
        return "mismatch"
    return "none"


def orientation_axis_consistent(row: dict[str, Any]) -> bool:
    """
    Return whether the row's axis error matches its orientation_status (metric 5).

    ORIENTATION_NOT_ESTIMATED requires an absent or non-finite axis error; a finite number
    (including 0.0) would make an orientation-blind pipeline look perfect. ORIENTATION_AXIS_
    ESTIMATED requires a finite axis error.
    """
    status = int(row.get("orientation_status", ORIENTATION_NOT_ESTIMATED))
    axis = row.get("axis_error_rad")
    if status == ORIENTATION_NOT_ESTIMATED:
        if axis is None:
            return True
        try:
            value = float(axis)
        except (TypeError, ValueError):
            return False
        return not math.isfinite(value)
    if status == ORIENTATION_AXIS_ESTIMATED:
        if axis is None:
            return False
        try:
            value = float(axis)
        except (TypeError, ValueError):
            return False
        return math.isfinite(value)
    return False


def read_rows(path: Path) -> list[dict[str, Any]]:
    """Read a JSONL campaign log; blank lines are skipped."""
    rows: list[dict[str, Any]] = []
    for line in path.read_text(encoding="utf-8").splitlines():
        if line.strip():
            rows.append(json.loads(line))
    return rows


def _describe_mm(values: Iterable[float]) -> str:
    """Return the shape of a millimetre population, or an empty-population marker."""
    ordered = sorted(values)
    if not ordered:
        return "n=0"

    def quantile(fraction: float) -> float:
        index = min(len(ordered) - 1, round(fraction * (len(ordered) - 1)))
        return ordered[index]

    return (
        f"n={len(ordered)} min {quantile(0.0) * 1000.0:+.3f} "
        f"p50 {quantile(0.50) * 1000.0:+.3f} p95 {quantile(0.95) * 1000.0:+.3f} "
        f"max {quantile(1.0) * 1000.0:+.3f} mm"
    )


def _is_refused(row: dict[str, Any]) -> bool:
    """Report whether the attempt produced no admitted observation (status != OK)."""
    return _status_code(row) != STATUS_OK


def _status_code(row: dict[str, Any]) -> int | None:
    """Return the row's status as an int, or None when it is absent or not numeric."""
    raw = row.get("status")
    if raw is None or isinstance(raw, bool):
        return None
    try:
        return int(raw)
    except (TypeError, ValueError):
        return None


def _launch_key(row: dict[str, Any]) -> tuple[str, int] | None:
    """Return the predeclared (cell_id, seed) this row belongs to, or None when unattributable."""
    cell_id = str(row.get("cell_id") or "")
    if not cell_id:
        return None
    try:
        cell_by_id(cell_id)
    except KeyError:
        return None
    raw = row.get("seed", -1)
    if raw is None:
        return None
    try:
        return cell_id, int(raw)
    except (TypeError, ValueError):
        return None


def _expected_observation_denominator(rows: Sequence[dict[str, Any]]) -> int:
    """Return expected observation rows for every logged (cell, seed): products × duties."""
    launches: set[tuple[str, int]] = set()
    for row in rows:
        key = _launch_key(row)
        if key is not None:
            launches.add(key)
    total = 0
    for cell_id, _seed in launches:
        total += products_per_arrangement(cell_id) * len(cell_by_id(cell_id).duties)
    return total


def summarise(rows: Sequence[dict[str, Any]]) -> dict[str, Any]:
    """Aggregate campaign rows into the predeclared metrics without mutating the inputs."""
    summary: dict[str, Any] = {
        "n_rows": len(rows),
        "by_band": {},
        "budget": {},
        "refusal": {
            "refused": 0,
            "expected_observation_rows": 0,
            "rate": None,
            "admitted_identity_none": 0,
        },
        "refusal_rate": None,
        "identity": dict(Counter(row.get("identity", "none") for row in rows)),
        "orientation_status": dict(
            Counter(str(row.get("orientation_status", ORIENTATION_NOT_ESTIMATED)) for row in rows)
        ),
        "orientation_schema_violations": 0,
        "touching_pair_detection_counts": {},
        "bottle_identity_by_duty": {},
        "non_upright": {},
        "missing_cells": [],
        "extrinsics": None,
    }

    admitted = [row for row in rows if _status_code(row) == STATUS_OK]
    for band in ("confirm", "overview", "far", "unknown"):
        errors = [
            float(row["translation_error_m"])
            for row in admitted
            if classify_observation_range(float(row.get("observation_range_m", float("nan"))))
            == band
            and row.get("translation_error_m") is not None
            and math.isfinite(float(row["translation_error_m"]))
        ]
        summary["by_band"][band] = _describe_mm(errors)

    # Metric 5: orientation_status and axis_error_rad must agree on every published row; a finite
    # axis under NOT_ESTIMATED is a schema violation even when it is 0.0.
    summary["orientation_schema_violations"] = sum(
        1 for row in rows if not orientation_axis_consistent(row)
    )

    # SC-002: every admitted confirm-duty OK row with a known geometry is eligible. Usable
    # covariance + finite error may fall inside the budget; anything else is fail-closed and
    # remains in the denominator.
    budget_errors: list[float] = []
    budget_values: list[float] = []
    fixed_within = 0
    budget_within = 0
    fail_closed = 0
    eligible = 0
    for row in admitted:
        band = classify_observation_range(float(row.get("observation_range_m", float("nan"))))
        if band != "confirm" or row.get("duty") != "confirm":
            continue
        geometry_key = row.get("geometry_key", "")
        if geometry_key not in GEOMETRY:
            # Unknown geometry: still an admitted confirm observation; fail closed.
            eligible += 1
            fail_closed += 1
            continue
        eligible += 1
        covariance = row.get("covariance")
        budget = (
            attachment_translation_budget_m(covariance, geometry_key)
            if isinstance(covariance, list)
            else None
        )
        error_raw = row.get("translation_error_m")
        error: float | None
        try:
            error = float(error_raw) if error_raw is not None else None
        except (TypeError, ValueError):
            error = None
        if budget is None or error is None or not math.isfinite(error):
            fail_closed += 1
            continue
        budget_errors.append(error)
        budget_values.append(budget)
        if error <= budget:
            budget_within += 1
        if error <= FIXED_TRANSLATION_TOLERANCE_M:
            fixed_within += 1
    if eligible:
        ratios = [
            error / budget for error, budget in zip(budget_errors, budget_values, strict=True)
        ]
        summary["budget"] = {
            "eligible": eligible,
            "n": len(budget_values),
            "fail_closed": fail_closed,
            "within_budget": budget_within,
            "within_budget_fraction": budget_within / eligible,
            "within_fixed_3mm": fixed_within,
            "within_fixed_3mm_fraction": fixed_within / eligible,
            "ratio_p50": statistics.median(ratios) if ratios else None,
            "ratio_p95": (
                sorted(ratios)[min(len(ratios) - 1, round(0.95 * (len(ratios) - 1)))]
                if ratios
                else None
            ),
            "error": _describe_mm(budget_errors),
            "budget": _describe_mm(budget_values),
        }

    # Metric 3: refusal = no admitted observation, over the predeclared expected-row
    # denominator for the launches present in the log — never over len(rows). Rows that do not
    # attribute to a predeclared (cell, seed) cannot sit in that denominator, so they stay out
    # of the numerator too rather than inflating the rate.
    refused = sum(1 for row in rows if _is_refused(row) and _launch_key(row) is not None)
    expected = _expected_observation_denominator(rows)
    admitted_identity_none = sum(1 for row in admitted if row.get("identity", "none") == "none")
    summary["refusal"] = {
        "refused": refused,
        "expected_observation_rows": expected,
        "rate": (refused / expected) if expected else None,
        "admitted_identity_none": admitted_identity_none,
    }
    summary["refusal_rate"] = summary["refusal"]["rate"]

    # Q11: detection counts recorded on touching_pair overview rows. The field is required on
    # those rows: defaulting a missing value to 0 would report "refused" for a schema that simply
    # omitted it (the predeclaration reserves 0 for a measured refusal).
    for row in rows:
        if row.get("feature") != "touching_pair" or row.get("duty") != "overview":
            continue
        if "observations_for_pair" not in row:
            continue
        key = str(row.get("cell_id", "?"))
        counts = summary["touching_pair_detection_counts"].setdefault(key, Counter())
        counts[int(row["observations_for_pair"])] += 1
    summary["touching_pair_detection_counts"] = {
        key: dict(counts) for key, counts in summary["touching_pair_detection_counts"].items()
    }

    # Q12: bottle identity and refusal (status only — identity none stays separate) by duty.
    bottles = {"bottle.small.standard", "bottle.large.standard"}
    for duty in DUTIES:
        subset = [
            row for row in rows if row.get("geometry_key") in bottles and row.get("duty") == duty
        ]
        if not subset:
            continue
        summary["bottle_identity_by_duty"][duty] = {
            "refused": sum(1 for row in subset if _is_refused(row)),
            "identity": dict(Counter(str(row.get("identity", "none")) for row in subset)),
        }

    # Q13: non-upright cell rows.
    non_upright = [row for row in rows if row.get("feature") == "non_upright"]
    if non_upright:
        summary["non_upright"] = {
            "n": len(non_upright),
            "statuses": dict(Counter(str(row.get("status")) for row in non_upright)),
            "reported_orientation": dict(
                Counter(str(row.get("orientation", "unset")) for row in non_upright)
            ),
            "identity": dict(Counter(str(row.get("identity", "none")) for row in non_upright)),
        }

    # Q7 / metric 9: translation-error spread and residual-vector spread at one optical pose.
    extrinsics = [
        row
        for row in admitted
        if row.get("cell_id") == "E-extrinsics" and row.get("translation_error_m") is not None
    ]
    if extrinsics:
        errors = [float(row["translation_error_m"]) for row in extrinsics]
        residual_vectors: list[tuple[float, float, float]] = []
        for row in extrinsics:
            residual = row.get("residual_xyz_m")
            if not isinstance(residual, (list, tuple)) or len(residual) != 3:
                continue
            try:
                vector = (float(residual[0]), float(residual[1]), float(residual[2]))
            except (TypeError, ValueError):
                continue
            if all(math.isfinite(component) for component in vector):
                residual_vectors.append(vector)
        residual_block: dict[str, Any] | None = None
        if residual_vectors:
            axis_std = [
                statistics.stdev([vector[axis] for vector in residual_vectors])
                if len(residual_vectors) >= 2
                else 0.0
                for axis in range(3)
            ]
            axis_spread = [
                max(vector[axis] for vector in residual_vectors)
                - min(vector[axis] for vector in residual_vectors)
                for axis in range(3)
            ]
            norms = [
                math.sqrt(sum(component * component for component in v)) for v in residual_vectors
            ]
            residual_block = {
                "n": len(residual_vectors),
                "axis_std_m": axis_std,
                "axis_spread_m": axis_spread,
                "norm_spread_m": max(norms) - min(norms),
                "norm_std_m": statistics.stdev(norms) if len(norms) >= 2 else 0.0,
            }
        summary["extrinsics"] = {
            "n": len(errors),
            "configurations": len(
                {row.get("arm_configuration", row.get("run_label")) for row in extrinsics}
            ),
            "spread_m": max(errors) - min(errors),
            "std_m": statistics.stdev(errors) if len(errors) >= 2 else 0.0,
            "error": _describe_mm(errors),
            "residual": residual_block,
        }

    # Matrix coverage: every predeclared cell_id must appear at least once when samples exist.
    present = {str(row.get("cell_id")) for row in rows}
    declared = {cell.cell_id for cell in predeclared_matrix()}
    summary["missing_cells"] = sorted(declared - present) if rows else sorted(declared)
    return summary


def format_summary(summary: dict[str, Any]) -> str:
    """Render a summarise() result as plain text for summary.txt."""
    lines = [
        f"wrist tray perception, whole population: {summary['n_rows']} rows",
        "",
        "by range band (admitted, translation error):",
    ]
    for band, text in summary["by_band"].items():
        lines.append(f"  {band:<10} {text}")
    lines.append("")
    budget = summary.get("budget") or {}
    if budget:
        lines.append("SC-002 confirm-band attachment budget (eligible admitted confirm rows):")
        lines.append(
            f"  eligible={budget['eligible']} measured={budget['n']} "
            f"fail_closed={budget['fail_closed']}"
        )
        lines.append(
            f"  within budget: {budget['within_budget']} "
            f"({budget['within_budget_fraction']:.3f} of eligible)"
        )
        lines.append(
            f"  within fixed 3 mm (comparison only): {budget['within_fixed_3mm']} "
            f"({budget['within_fixed_3mm_fraction']:.3f} of eligible)"
        )
        if budget.get("ratio_p50") is not None:
            lines.append(
                f"  error/budget ratio p50 {budget['ratio_p50']:.3f} p95 {budget['ratio_p95']:.3f}"
            )
        lines.append(f"  error    {budget['error']}")
        lines.append(f"  budget   {budget['budget']}")
    else:
        lines.append("SC-002 confirm-band attachment budget: no eligible rows")
    lines.append("")
    refusal = summary.get("refusal") or {}
    if refusal.get("rate") is not None:
        lines.append(
            f"refusal rate: {refusal['rate']:.3f} "
            f"({refusal['refused']} refused / {refusal['expected_observation_rows']} "
            "expected observation rows)"
        )
        lines.append(
            f"admitted identity=none (not a refusal): {refusal['admitted_identity_none']}"
        )
    lines.append(f"identity: {summary['identity']}")
    lines.append(f"orientation_status: {summary['orientation_status']}")
    if summary.get("orientation_schema_violations"):
        lines.append(
            f"orientation schema violations (metric 5): {summary['orientation_schema_violations']}"
        )
    if summary.get("touching_pair_detection_counts"):
        lines.append(
            "Q11 touching_pair overview observation counts: "
            f"{summary['touching_pair_detection_counts']}"
        )
    if summary.get("bottle_identity_by_duty"):
        lines.append(f"Q12 bottle identity/refusal by duty: {summary['bottle_identity_by_duty']}")
    if summary.get("non_upright"):
        lines.append(f"Q13 non-upright: {summary['non_upright']}")
    if summary.get("extrinsics"):
        lines.append(f"Q7 extrinsics: {summary['extrinsics']}")
    if summary.get("missing_cells"):
        lines.append(f"missing predeclared cells: {summary['missing_cells']}")
    return "\n".join(lines) + "\n"


def format_matrix() -> str:
    """Render the predeclared matrix as a stable text table."""
    lines = ["cell_id        feature         seeds        geometries"]
    for cell in predeclared_matrix():
        seeds = ",".join(str(seed) for seed in cell.seeds)
        geometries = ",".join(cell.geometry_keys)
        lines.append(f"{cell.cell_id:<14} {cell.feature:<15} {seeds:<12} {geometries}")
    lines.append(f"predeclared simulator launches: {expected_arrangement_launches()}")
    return "\n".join(lines) + "\n"

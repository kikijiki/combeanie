# Copyright 2026 Matteo Bernacchia <dev@kikijiki.com>
#
# SPDX-License-Identifier: MIT

"""The lane stock estimator core: evidence in, a count, a composition and a cannot-tell out."""
# Pure Python, no ROS and no numpy dependency in the decision (the optional top-height profile is
# read through plain sequences). Count = the column's occupied depth over the product's contact
# pitch; the top-height staircase gives the rear product's class and a mixed-class column; the
# identity channel, when it names a face, gives the SKU. Nothing here guesses: an unanswered or
# abstaining identity is never read as "conforming", and anything the geometry cannot explain is a
# cannot-tell, not a low-confidence number.
#
# Order of refusals is deliberate: a producer's refusal, then coverage, then obstruction, then
# policy, then the geometry's own consistency. A lane that is refused says so in `reason`; the
# count fields of a refusal are zero and must not be read.

from __future__ import annotations

from collections.abc import Sequence
from dataclasses import dataclass, field
import math

from restocker_perception.stock.config import (
    CLASS_NAMES,
    PRODUCT_CLASS_UNKNOWN,
    ProductGeometry,
    StockConfig,
)

STATE_ESTIMATED = 0
STATE_CANNOT_TELL = 1

REASON_NONE = 0
REASON_NO_EVIDENCE = 1
REASON_LOW_COVERAGE = 2
REASON_OBSTRUCTED = 3
REASON_INCONSISTENT_GEOMETRY = 4
REASON_NO_POLICY = 5

COMPOSITION_UNKNOWN = 0
COMPOSITION_CONFORMING = 1
COMPOSITION_PARTIAL = 2
COMPOSITION_MISPLACED = 3
COMPOSITION_EMPTY = 4

IDENTITY_UNAVAILABLE = 0
IDENTITY_NAMED = 1
IDENTITY_ABSTAIN = 2
IDENTITY_UNKNOWN = 3

# Tunables, with how each was chosen; the calibration record states which data fixed them.
# A free depth this close to the full lane is an empty lane: one can is 0.09 m short of it.
EMPTY_MARGIN_M = 0.02
# Largest distance of the measured column length from a whole number of pitches (in pitches)
# that is still read as that number. 0.5 is the geometry's own limit; below it the lane is
# refused as inconsistent.
MAX_PITCH_RESIDUAL = 0.35
# Residual below which the count is exact and the range collapses to it.
EXACT_PITCH_RESIDUAL = 0.20
# Two top heights closer than this are one product class; the catalogue's classes are 0.078 m
# apart at the nearest, so 0.03 m cannot merge them.
HEIGHT_STEP_TOLERANCE_M = 0.03
# A staircase step must persist over this many consecutive bins to count (rejects speckle).
STEP_MIN_BINS = 2
# A product's footprint is at least this share of its contact pitch; a surface supported by
# fewer bins than that (at the profile's bin width), and never by fewer than MIN_SURFACE_BINS, is
# not a product: it is the retainer's face smeared into the window by noise, or a stray return.
MIN_FOOTPRINT_SHARE = 0.7
MIN_SURFACE_BINS = 2
# Above this much range noise in the frame (the bed's own height sigma) the top heights are not
# good enough to tell classes 0.078 m apart (measured: the rear top's error is about one sigma at
# 0.5-1 % range noise and tens of millimetres, with spurious steps, at 2 %), so the estimator
# does not read a class or a mixed column from them: it counts with the policy's class and makes
# no class claim.
MAX_SIGMA_FOR_CLASS_M = 0.008
# How many leading bins of a column may be the rear wall's foot rather than its top.
LEADING_RAMP_BINS = 2
# The rear product's top height is matched to a catalogued product's height within this distance
# (CMB-120: the catalogue now holds products 15 mm apart, so the 0.03 m that kept three cylinder
# classes apart no longer separates them), or within this many bed sigmas of the frame if more.
# Where two products still fall inside it, the column's own length decides (_pick).
HEIGHT_MATCH_TOLERANCE_M = 0.010
HEIGHT_MATCH_SIGMAS = 3.0
# Two products that both fit the column's top height are told apart by its length (_pick). A
# product's pitch "fits" when the column's length is within this many metres of a whole number of
# them, or within this many bed sigmas if more: a residual measured in pitches would let a product
# of a long pitch fit anything (the residual of a 55 mm pitch is up to 27 mm, the noise on the
# edge a few mm), so a column of 7 slim boxes could be read as 3 boxes lying across the lane.
LENGTH_FIT_FLOOR_M = 0.004
LENGTH_FIT_SIGMAS = 2.5


@dataclass(frozen=True)
class FaceIdentity:
    """The identity channel's outcome for one visible product face (already policy-filtered)."""

    status: int
    sku: str | None = None
    reason: str = ""


@dataclass(frozen=True)
class TopProfile:
    """Per depth bin: the least lane depth and the greatest height above the bed of the returns.

    ``None`` where the bin has no surface. Both sequences run rear to front, ``bin_width_m`` apart.
    """

    near_depth_m: Sequence[float | None]
    top_height_m: Sequence[float | None]
    bin_width_m: float
    # Robust height sigma of the bed in the frame (0 if unknown); see MAX_SIGMA_FOR_CLASS_M.
    bed_sigma_m: float = 0.0


@dataclass(frozen=True)
class LaneEvidence:
    """Everything the estimator reads about one lane."""

    lane_id: str
    # False when the depth producer refused or sent nothing (LaneObservation.status != OK).
    evidence_ok: bool
    coverage: float
    free_depth_m: float
    obstructed: bool
    expected_class: int = PRODUCT_CLASS_UNKNOWN
    expected_sku: str | None = None
    profile: TopProfile | None = None
    faces: Sequence[FaceIdentity] = ()
    identity_reachable: bool = False
    identity_registry_sha256: str = ""
    identity_calibration_sha256: str = ""


@dataclass
class LaneStockEstimate:
    """The estimator's statement, field for field the LaneStock message."""

    lane_id: str
    state: int = STATE_CANNOT_TELL
    reason: int = REASON_NONE
    reason_detail: str = ""
    expected_product_class: int = PRODUCT_CLASS_UNKNOWN
    has_expected_sku: bool = False
    expected_sku: str = ""
    free_depth_m: float = 0.0
    occupied_depth_m: float = 0.0
    usable_depth_m: float = 0.0
    coverage: float = 0.0
    product_length_m: float = 0.0
    product_class_assumed: int = PRODUCT_CLASS_UNKNOWN
    # The shape the count assumed (a key of StockConfig.products); "" where nothing was assumed.
    product_name: str = ""
    count_estimate: int = 0
    count_min: int = 0
    count_max: int = 0
    count_consistent: bool = True
    visible_faces: int = 0
    identity_channel_reachable: bool = False
    face_skus: list[str] = field(default_factory=list)
    face_identity_status: list[int] = field(default_factory=list)
    named_count: int = 0
    unverified_count: int = 0
    composition: int = COMPOSITION_UNKNOWN
    misplaced: bool = False
    misplaced_named_count: int = 0
    misplaced_skus: list[str] = field(default_factory=list)
    identity_registry_sha256: str = ""
    identity_calibration_sha256: str = ""
    class_mismatch: bool = False
    # Distance, in pitches, of the measured column length from a whole number of products.
    pitch_residual: float = 0.0
    # Per run of equal-height products from the rear: (product class enum, count). Reported so a
    # mixed-class column is described, not just flagged; empty without a profile.
    segments: list[tuple[int, int]] = field(default_factory=list)

    @property
    def answered(self) -> bool:
        return self.state == STATE_ESTIMATED


def _refuse(
    evidence: LaneEvidence, config: StockConfig, reason: int, detail: str
) -> LaneStockEstimate:
    return LaneStockEstimate(
        lane_id=evidence.lane_id,
        state=STATE_CANNOT_TELL,
        reason=reason,
        reason_detail=detail,
        expected_product_class=evidence.expected_class,
        has_expected_sku=evidence.expected_sku is not None,
        expected_sku=evidence.expected_sku or "",
        usable_depth_m=config.workcell.usable_depth_m,
        coverage=evidence.coverage,
        identity_channel_reachable=evidence.identity_reachable,
    )


def _base_name(product: ProductGeometry) -> str:
    """A shape's key in StockConfig.products (an orientation's name is ``key@...``)."""
    return product.name.split("@", 1)[0]


def _catalogue(config: StockConfig, expected: ProductGeometry | None) -> list[ProductGeometry]:
    """What a column's top height may be matched to: every catalogued product standing, and every
    orientation of the one the lane expects (a registered box may lie on a face)."""
    out = list(config.products.values())
    if expected is not None:
        seen = {p.name for p in out}
        out += [
            o
            for o in expected.orientations()
            if o.name not in seen and not _quarter_turn(o, expected)
        ]
    return out


def _quarter_turn(orientation: ProductGeometry, registered: ProductGeometry) -> bool:
    """True for the registered product turned about the vertical: same top face up, a different
    side along the lane.

    Its top height is the registered one's and only its pitch differs, so the depth cannot tell it
    from the registered orientation by height; by length it can only be told when the column's
    length is not near a whole number of both, and noise of a few millimetres on a 55 mm pitch
    makes the two fit by chance (a column of 7 slim boxes read as 3). A lane is packed with the
    depth side along it (restocker_gazebo lane_columns), so it is not a candidate.
    """
    return (
        abs(orientation.height_m - registered.height_m) < 1e-9
        and abs(orientation.radius_m - registered.radius_m) > 1e-9
    )


def _height_tolerance(profile: TopProfile | None) -> float:
    sigma = profile.bed_sigma_m if profile is not None else 0.0
    return max(HEIGHT_MATCH_TOLERANCE_M, HEIGHT_MATCH_SIGMAS * sigma)


def _length_fit(profile: TopProfile | None) -> float:
    sigma = profile.bed_sigma_m if profile is not None else 0.0
    return max(LENGTH_FIT_FLOOR_M, LENGTH_FIT_SIGMAS * sigma)


def _height_matches(
    height_m: float, catalogue: Sequence[ProductGeometry], tolerance_m: float
) -> list[ProductGeometry]:
    """Catalogued products whose height is within the tolerance, nearest first."""
    near = [p for p in catalogue if abs(p.height_m - height_m) <= tolerance_m]
    return sorted(near, key=lambda p: abs(p.height_m - height_m))


def _column_length(config: StockConfig, product: ProductGeometry, nearest_depth_m: float) -> float:
    """How many of the product the depth to the nearest surface fits (a real number)."""
    cell = config.workcell
    return 1.0 + (
        cell.bed_datum_depth_m - product.radius_m - config.rear_extent_m(product) - nearest_depth_m
    ) / config.pitch_m(product)


def _pick(
    options: Sequence[ProductGeometry],
    config: StockConfig,
    nearest_depth_m: float,
    expected: ProductGeometry | None,
    fit_m: float = LENGTH_FIT_FLOOR_M,
) -> tuple[ProductGeometry | None, str]:
    """The product a column is made of, from those whose height fits, or why none can be named.

    One option is the answer. Several (products 15 mm apart under a few millimetres of noise): the
    lane's own product when it is one of them, else the length of the column tells them apart (only
    a product whose pitch makes it a whole number of products, to within the measurement's noise,
    fits); candidates that fit and disagree on the count are not guessed between.
    """
    if not options:
        return None, ""
    if len(options) == 1:
        return options[0], ""
    if expected is not None:
        # The lane's own product, when its height is one of the candidates, is the answer: under
        # noise it counts the column in whole products or the caller refuses (a residual too large
        # for a count). Letting another candidate win because *it* happens to count whole is the
        # chance fit of a long pitch (a column of 6 slim boxes, 25 mm pitch, read as 3 cartons of
        # 60 mm), a confident wrong count; a product the lane does not expect is named only when
        # the lane's own is not one of the candidates by height.
        own = [p for p in options if _base_name(p) == _base_name(expected)]
        if own:
            return min(own, key=lambda p: _pitch_residual(config, p, nearest_depth_m)), ""
    fitting = []
    for product in options:
        n = _column_length(config, product, nearest_depth_m)
        residual_m = abs(n - round(n)) * config.pitch_m(product)
        if residual_m <= fit_m and abs(n - round(n)) <= MAX_PITCH_RESIDUAL:
            fitting.append((residual_m, product, max(int(round(n)), 0)))
    if not fitting:
        return options[0], ""  # none fits: the caller's pitch check refuses with the nearest
    if len({n for _, _, n in fitting}) == 1:
        return min(fitting, key=lambda item: item[0])[1], ""
    names = ", ".join(sorted({p.name for _, p, _ in fitting}))
    return None, f"the rear product could be {names}, which count the column differently"


def _pitch_residual(config: StockConfig, product: ProductGeometry, nearest_depth_m: float) -> float:
    n = _column_length(config, product, nearest_depth_m)
    return abs(n - round(n))


@dataclass(frozen=True)
class _Segment:
    first_bin: int
    last_bin: int
    height_m: float


def _staircase(profile: TopProfile) -> list[_Segment]:
    """Runs of equal top height from the rear, with speckle steps rejected."""
    bins = [
        (index, height)
        for index, (depth, height) in enumerate(
            zip(profile.near_depth_m, profile.top_height_m, strict=True)
        )
        if depth is not None and height is not None
    ]
    if not bins:
        return []
    # The camera sees the rear product's wall at a grazing angle, so the first bin or two hold only
    # its foot: a lower "top" that is not a step. Drop up to LEADING_RAMP_BINS of them while the
    # next bin is higher by more than the step tolerance.
    for _ in range(LEADING_RAMP_BINS):
        if len(bins) > 1 and bins[1][1] - bins[0][1] > HEIGHT_STEP_TOLERANCE_M:
            bins = bins[1:]
        else:
            break
    segments: list[_Segment] = []
    start = 0
    reference = bins[0][1]
    index = 0
    while index < len(bins):
        _, height = bins[index]
        if abs(height - reference) > HEIGHT_STEP_TOLERANCE_M:
            # A step only counts if the new height persists.
            run = bins[index : index + STEP_MIN_BINS]
            persists = len(run) == STEP_MIN_BINS and all(
                abs(h - height) <= HEIGHT_STEP_TOLERANCE_M for _, h in run
            )
            if persists:
                members = [h for _, h in bins[start:index]]
                segments.append(
                    _Segment(
                        bins[start][0], bins[index - 1][0], sorted(members)[len(members) // 2]
                    )
                )
                start = index
                reference = height
        index += 1
    members = [h for _, h in bins[start:]]
    segments.append(_Segment(bins[start][0], bins[-1][0], sorted(members)[len(members) // 2]))
    return segments


def _bins_needed(config: StockConfig, product: ProductGeometry, bin_width_m: float) -> int:
    """How many depth bins one product's top surface must cover to be believed."""
    return max(MIN_SURFACE_BINS, math.ceil(MIN_FOOTPRINT_SHARE * config.pitch_m(product) / bin_width_m))


def _identity_part(evidence: LaneEvidence, estimate: LaneStockEstimate) -> None:
    estimate.identity_channel_reachable = evidence.identity_reachable
    estimate.identity_registry_sha256 = evidence.identity_registry_sha256
    estimate.identity_calibration_sha256 = evidence.identity_calibration_sha256
    for face in evidence.faces:
        estimate.face_skus.append(face.sku or "")
        estimate.face_identity_status.append(face.status)
        if face.status == IDENTITY_NAMED and face.sku:
            estimate.named_count += 1
            if evidence.expected_sku is not None and face.sku != evidence.expected_sku:
                estimate.misplaced_named_count += 1
                estimate.misplaced_skus.append(face.sku)


def estimate_lane_stock(evidence: LaneEvidence, config: StockConfig) -> LaneStockEstimate:
    """Estimate one lane's stock from its evidence, or say that it cannot be told."""
    cell = config.workcell
    if not evidence.evidence_ok:
        return _refuse(evidence, config, REASON_NO_EVIDENCE, "the depth producer refused")
    if not math.isfinite(evidence.coverage) or evidence.coverage < config.minimum_coverage:
        return _refuse(
            evidence,
            config,
            REASON_LOW_COVERAGE,
            f"coverage {evidence.coverage:.3f} is below {config.minimum_coverage:.2f}",
        )
    if evidence.obstructed:
        return _refuse(evidence, config, REASON_OBSTRUCTED, "something protrudes from the mouth")
    free = min(max(evidence.free_depth_m, 0.0), cell.usable_depth_m)

    estimate = LaneStockEstimate(
        lane_id=evidence.lane_id,
        expected_product_class=evidence.expected_class,
        has_expected_sku=evidence.expected_sku is not None,
        expected_sku=evidence.expected_sku or "",
        free_depth_m=free,
        occupied_depth_m=cell.usable_depth_m - free,
        usable_depth_m=cell.usable_depth_m,
        coverage=evidence.coverage,
    )
    expected = config.expected_product(evidence.expected_class, evidence.expected_sku)

    if free >= cell.usable_depth_m - EMPTY_MARGIN_M:
        # Seen to be empty along its whole length (coverage passed above), which is an answer.
        estimate.state = STATE_ESTIMATED
        estimate.composition = COMPOSITION_EMPTY
        estimate.product_class_assumed = evidence.expected_class
        if expected is not None:
            estimate.product_length_m = config.pitch_m(expected)
            estimate.product_name = _base_name(expected)
        estimate.count_consistent = True
        _identity_part(evidence, estimate)
        return estimate

    segments = []
    if evidence.profile is not None and evidence.profile.bed_sigma_m <= MAX_SIGMA_FOR_CLASS_M:
        segments = _staircase(evidence.profile)
    catalogue = _catalogue(config, expected)
    tolerance = _height_tolerance(evidence.profile)
    nearest_depth = cell.rear_clearance_m + free
    rear_class: ProductGeometry | None = None
    if segments:
        options = _height_matches(segments[0].height_m, catalogue, tolerance)
        if not options:
            # A surface that is lower than every catalogued product is not a product: bed speckle
            # read through sensor noise, a divider's foot, a stray object. Counting it against the
            # policy's pitch would turn an empty lane into a full one, so refuse.
            return _refuse(
                evidence,
                config,
                REASON_INCONSISTENT_GEOMETRY,
                f"the nearest surface tops out at {segments[0].height_m:.3f} m, which no "
                "catalogued product does",
            )
        rear_class, ambiguity = _pick(
            options, config, nearest_depth, expected, _length_fit(evidence.profile)
        )
        if rear_class is None:
            return _refuse(evidence, config, REASON_INCONSISTENT_GEOMETRY, ambiguity)
    if evidence.profile is not None and evidence.profile.bin_width_m > 0.0:
        # The surface must be as wide as a product: the one read from the top height, else the
        # slimmest one the lane could hold.
        reference = rear_class or min(catalogue, key=config.pitch_m)
        needed = _bins_needed(config, reference, evidence.profile.bin_width_m)
        supported = sum(1 for d in evidence.profile.near_depth_m if d is not None)
        if supported < needed:
            return _refuse(
                evidence,
                config,
                REASON_INCONSISTENT_GEOMETRY,
                f"the nearest surface is {supported} bin(s) wide; a product is at least {needed}",
            )
    assumed = rear_class or expected
    if assumed is None:
        return _refuse(
            evidence, config, REASON_NO_POLICY, "no product class from policy or from the geometry"
        )
    estimate.product_class_assumed = assumed.product_class
    estimate.product_name = _base_name(assumed)
    estimate.product_length_m = config.pitch_m(assumed)
    if rear_class is not None and expected is not None and _base_name(rear_class) != _base_name(
        expected
    ):
        estimate.class_mismatch = True

    # --- count -------------------------------------------------------------------------------
    counts: list[tuple[int, int]] = []
    named_counts: list[tuple[ProductGeometry, int]] = []
    residual = 0.0
    if len(segments) > 1 and evidence.profile is not None:
        geometries = []
        for segment in segments:
            matches = _height_matches(segment.height_m, catalogue, tolerance)
            if not matches:
                return _refuse(
                    evidence,
                    config,
                    REASON_INCONSISTENT_GEOMETRY,
                    f"a top surface at {segment.height_m:.3f} m matches no catalogued class",
                )
            geometries.append(matches[0])
        if any(
            segment.last_bin - segment.first_bin + 1
            < _bins_needed(config, geometry, evidence.profile.bin_width_m)
            for segment, geometry in zip(segments, geometries, strict=True)
        ):
            # A step in the top height that does not last one product's footprint is noise, not
            # a different product: refuse rather than call the column mixed.
            return _refuse(
                evidence,
                config,
                REASON_INCONSISTENT_GEOMETRY,
                "the top height steps within a product's footprint",
            )
        # A mixed-class column: count each run of equal-height products against its own pitch.
        total = 0.0
        width = evidence.profile.bin_width_m
        for position, (segment, geometry) in enumerate(zip(segments, geometries, strict=True)):
            pitch = config.pitch_m(geometry)
            start = evidence.profile.near_depth_m[segment.first_bin] or nearest_depth
            if position + 1 < len(segments):
                end = evidence.profile.near_depth_m[segments[position + 1].first_bin]
                length = (end if end is not None else start + width) - start
                n = length / pitch
            else:
                n = (
                    1.0
                    + (
                        cell.bed_datum_depth_m
                        - geometry.radius_m
                        - config.rear_extent_m(geometry)
                        - start
                    )
                    / pitch
                )
            residual = max(residual, abs(n - round(n)))
            counts.append((geometry.product_class, max(int(round(n)), 1)))
            named_counts.append((geometry, max(int(round(n)), 1)))
            total += max(round(n), 1)
        n_depth = total
    else:
        n_depth = (
            1.0
            + (
                cell.bed_datum_depth_m
                - assumed.radius_m
                - config.rear_extent_m(assumed)
                - nearest_depth
            )
            / estimate.product_length_m
        )
        residual = abs(n_depth - round(n_depth))
        counts = [(assumed.product_class, max(int(round(n_depth)), 0))]
        named_counts = [(assumed, max(int(round(n_depth)), 0))]
    if residual > MAX_PITCH_RESIDUAL:
        return _refuse(
            evidence,
            config,
            REASON_INCONSISTENT_GEOMETRY,
            f"column of {n_depth:.2f} products is {residual:.2f} pitch from a whole number",
        )
    estimate.state = STATE_ESTIMATED
    estimate.pitch_residual = residual
    estimate.count_estimate = max(int(round(n_depth)), 1)
    estimate.segments = counts if segments else []
    if expected is not None and any(
        _base_name(geometry) != _base_name(expected) for geometry, _ in named_counts
    ):
        estimate.class_mismatch = True
    if residual <= EXACT_PITCH_RESIDUAL:
        estimate.count_min = estimate.count_max = estimate.count_estimate
    else:
        estimate.count_min = max(int(math.floor(n_depth)), 1)
        estimate.count_max = max(int(math.ceil(n_depth)), 1)

    # --- cross-check: continuity of the top surface ---------------------------------------------
    if evidence.profile is not None:
        surface_bins = sum(1 for d in evidence.profile.near_depth_m if d is not None)
        faces = surface_bins * evidence.profile.bin_width_m / estimate.product_length_m
        estimate.visible_faces = int(round(faces))
        if abs(faces - estimate.count_estimate) > 1.0 + EXACT_PITCH_RESIDUAL:
            # The top surface has holes or stops short of what the depth count says: part of the
            # frame did not return, so neither the count nor the rear product's class (read from
            # that surface) can be believed. Refuse; a widened range is still a guess.
            return _refuse(
                evidence,
                config,
                REASON_INCONSISTENT_GEOMETRY,
                f"the top surface supports {faces:.1f} products, the depth says "
                f"{estimate.count_estimate}",
            )

    # --- identity and composition ------------------------------------------------------------
    _identity_part(evidence, estimate)
    estimate.unverified_count = max(estimate.count_estimate - estimate.named_count, 0)
    estimate.misplaced = estimate.misplaced_named_count > 0 or estimate.class_mismatch
    if estimate.misplaced:
        estimate.composition = COMPOSITION_MISPLACED
    elif estimate.named_count > 0 and estimate.unverified_count == 0:
        estimate.composition = COMPOSITION_CONFORMING
    elif estimate.named_count > 0:
        estimate.composition = COMPOSITION_PARTIAL
    else:
        estimate.composition = COMPOSITION_UNKNOWN
    return estimate


def class_name(product_class: int) -> str:
    """Name of a ShelfLane product class enum value."""
    return CLASS_NAMES.get(product_class, "unknown")

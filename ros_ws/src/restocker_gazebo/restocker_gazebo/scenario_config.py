# Copyright 2026 Matteo Bernacchia <dev@kikijiki.com>
#
# SPDX-License-Identifier: MIT

"""Validated loading for simulator scenarios and shared collision geometry."""

from collections.abc import Sequence
from dataclasses import dataclass
import math
from pathlib import Path
from typing import Any

import yaml

ProductCatalog = dict[str, dict[str, Any]]
VALID_PRODUCT_CLASSES = {"can", "small_bottle", "large_bottle"}

# Only this generator version's bounds are understood. A scenario that bumps its version without
# this constant stops the launch instead of being drawn from a schema the generator cannot honour.
#
# A bump also announces that seeds no longer mean what they did: the version is part of the bounds
# fingerprint, so a draw-order change that left the bounds alone would otherwise reproduce a
# different scenario under an identical digest.
#
# Version 2 dropped the per-product destination lane draw that nothing read. Seeds recorded against
# version 1 produce different products and poses.
#
# Version 3 tightened the pairwise separation rule with the jaw-approach clearance below. Draw
# order is unchanged, but a candidate is rejected on a wider test, so a seed replays the same
# attempts and keeps a different subset. Every seed that accepted a too-close pair, and every seed
# drawn after one, produces different poses.
#
# Version 4 widened the same test by PLANNER_ROBOT_PADDING_M below: version 3 used the finger's
# bare geometry, which is not the finger the planner collision-checks. Same consequence; version 3
# seeds must be re-recorded. Over seeds 0..4999, 62 of 5000 scenarios (1.24%) contained a pair
# version 3 accepted and version 4 rejects.
#
# Version 5 replaced that test, added a reachability bound, and bounded the per-class product count
# by the shelf's lane capacity. All three came from a `seeded_autonomous` campaign in which four of
# five seeds failed on faults the earlier rules cannot express. No version 4 seed replays.
#
#   * The corridor is measured across the approach, not around the grasp. Versions 3 and 4 kept
#     neighbours outside a circle of radius `jaw clearance + radius` about the target, which is
#     wrong for a corridor the arm sweeps along: a neighbour between the target and the robot is
#     inside the swept tube at any distance, and both measured flange truncations were exactly
#     that. See :func:`scenario_random.required_lateral_distance`.
#   * The corridor is measured over every link that shares the tool axis, not the fingers alone.
#   * A placement whose pre-grasp the arm cannot reach is rejected. See
#     MINIMUM_PREGRASP_REACH_Y_M.
#   * A class is never drawn more times than the shelf has lanes for it, so a generated scenario
#     is satisfiable by construction.
#   * The FAR end of the drawable interval is deliberately not bounded, and that is accepted
#     rather than an oversight. The reachability bound clips only the near end; deep in the tray
#     some collision-free starts cannot finish the approach because forearm_link or wrist_1_link
#     strikes stock_tray_collision. Measured with the committed probe
#     (tools/diagnostics/planning_probe, --mode placements + reach, --seeds 60): 16 of 215
#     drawn placements, in 14 of 60 seeds, all at product y in [-1.011, -0.964] (far boundary:
#     pre-grasp depth 0.635 m, can, rail x 0.0), each failing on 1 to 10 of about 57 sampled
#     starts — so a fresh sample escapes and the live symptom is an occasional recoverable
#     approach truncation, not a stuck goal. The archived version-5 acceptance row "0 of 215
#     truncating" does not reproduce and reads 16 of 215 here and in the evidence audit. This is
#     a decision, not a gap: the shipped dense fixture stands products in the same band and the
#     demo picks them, a far-edge clip would cost a generator version bump plus about 15% of
#     the drawable interval, and shrinking stock_tray.usable_volume would falsify the surveyed
#     volume (Card 034, 2026-09-25). It reopens for a clip only on a live terminal failure whose
#     motion refusals are all this family at product y < -0.95.
SUPPORTED_GENERATOR_VERSION = 5

# MoveIt inflates every robot link by this much before collision-checking, so the planned fingers
# are larger than the ones restocker_description describes. The value is
# `robot_description_planning.default_robot_padding` in
# restocker_moveit_config/launch/move_group.launch.py, which carries the measurements; the two
# must move together. restocker_gazebo does not depend on the MoveIt configuration (`just
# launch-sim` runs with no planner), so it cannot read the value.
#
# It is not in gripper_geometry.yaml because it is what the planner does to every link, and the
# gripper datum is also read by the URDF and the attachment plugin.
#
# Only robot links are padded. Products and the tray enter the planning scene at their catalogued
# dimensions (restocker_task_executor/src/scene_geometry.cpp).
PLANNER_ROBOT_PADDING_M = 0.0015

# Maximum radial extent around the tool-side UR10e wrist chain, measured from the official
# ur_description 3.5.1 UR10e collision STLs in each link's local frame; the widest contributor is
# wrist_1_link. The gripper body and opened fingers stand less far out. Restated because
# restocker_gazebo does not depend on a mesh parser or MoveIt.
WRIST_LINK_RADIUS_M = 0.085

# How far tool0 stands back from the grasp centre at the pre-grasp, beyond the gripper's own
# grasp_center offset. Equals `grasp.pregrasp_distance_m` in
# restocker_task_executor/config/restock_action_coordinator.yaml, which carries the measurement.
# Copied because restocker_gazebo does not depend on the task executor;
# test_scenario_random.py fails if the two drift.
PREGRASP_DISTANCE_M = 0.18

# How close to the rail a pre-grasp may stand and still be reachable, measured along the approach.
#
# The approach is a straight line and the coordinator does not choose the arm configuration: the
# pre-grasp traverse is a pose goal on the redundant group, so MoveIt's sampler picks the rail and
# elbow. The requirement is that every collision-free configuration follows the line, because a run
# that lands on a bad branch truncates at the same fraction on every recovery attempt.
#
# Measured offline against the real RobotModel, SRDF and planning scene, with planner padding and
# an empty tray, sweeping a lone product over the drawable envelope. The cliff is a millimetre
# wide: every collision-free start completes the approach with the pre-grasp 0.331 m from the rail
# axis, and at 0.330 m some cannot follow the line at all. The failures are kinematic, not
# collisions, so no recovery escapes them. The boundary was identical at every rail-reachable x and
# for all three product classes: the rail cancels the lateral coordinate, so this bounds the
# approach axis alone.
#
# 0.34 rather than 0.331 because the sweep resolves the cliff to a millimetre and samples the
# arm's redundancy rather than enumerating it; 0.34 is the shallowest depth a coarser sweep
# confirmed clear across the full tray width. It costs a small bottle 71 mm of its 422 mm y
# interval, 9 mm of which is this margin.
#
# The bound is on the pre-grasp, not the product, because that is where the arm runs out of reach:
# the same product is fine at a smaller stand-off.
MINIMUM_PREGRASP_REACH_Y_M = 0.34

# The one placement envelope the generator can sample. Named in the scenario so that a scenario
# meaning something else fails instead of being placed in the tray.
STOCK_REGION_SOURCE = "stock_tray.usable_volume"


@dataclass(frozen=True)
class ApproachCorridor:
    """The tube the arm sweeps into the tray to reach one product, as the planner sees it."""

    # Both numbers are derived from the description and the coordinator's configuration, so
    # changing a finger, the grasp datum or the pre-grasp standoff moves the rejection rules.

    # Half the corridor's width, measured across the approach from the tool axis.
    half_width_m: float
    # How far tool0 stands back from the product centre at the pre-grasp, along the approach.
    pregrasp_standoff_m: float


@dataclass(frozen=True)
class StockRegion:
    """The surveyed stock-tray volume a generated product must be seatable inside."""

    # Held as a value so the placement maths and the invariant check measure the same numbers.

    center_xyz_m: tuple[float, float, float]
    size_xyz_m: tuple[float, float, float]
    floor_settle_tolerance_m: float


def load_scenario(path: str | Path) -> dict[str, Any]:
    """Load enough scenario structure to construct deterministic spawn actions."""
    scenario = yaml.safe_load(Path(path).read_text(encoding="utf-8"))
    if not isinstance(scenario, dict) or scenario.get("schema_version") != 1:
        raise RuntimeError("scenario_config requires schema_version 1")
    if len(scenario.get("workcell_pose", [])) != 6:
        raise RuntimeError("scenario_config workcell_pose must contain six values")
    products = scenario.get("products", [])
    if not isinstance(products, list) or not products:
        raise RuntimeError("scenario_config must contain at least one product")
    model_names = [item.get("model_name", "") for item in products]
    if any(not name for name in model_names) or len(set(model_names)) != len(model_names):
        raise RuntimeError("scenario_config product model names must be non-empty and unique")
    for item in products:
        _validate_label_texture(item.get("label_texture"))
    # Validated eagerly so a typo in the bounds is caught by the tests and the default launch, not
    # on the first randomized run.
    if "randomization" in scenario:
        load_randomization(scenario)
    return scenario


def _validate_label_texture(value: Any) -> None:
    """Reject a label filename the launch file could not resolve into the shipped texture set."""
    # A product without the field keeps its flat colour; the field lets two products of one shape
    # be told apart. The name is a bare filename because the launch file joins it onto
    # restocker_gazebo's installed materials/textures directory. Rejecting separators is not a
    # sandbox (scenario files are trusted); it turns the failure into a clear message instead of a
    # texture Gazebo silently declines to load.
    if value is None:
        return
    if not isinstance(value, str) or not value:
        raise RuntimeError("scenario_config label_texture must be a non-empty filename")
    if "/" in value or value in {".", ".."}:
        raise RuntimeError(f"scenario_config label_texture must be a bare filename: {value}")


def _require_positive_finite(value: Any, label: str) -> float:
    """Return one strictly positive finite length, or reject it by name."""
    number = float(value)
    if not math.isfinite(number) or number <= 0.0:
        raise RuntimeError(f"scenario_config requires a positive finite {label}")
    return number


def _require_non_negative_finite(value: Any, label: str) -> float:
    """Return one non-negative finite length, or reject it by name."""
    number = float(value)
    if not math.isfinite(number) or number < 0.0:
        raise RuntimeError(f"scenario_config requires a non-negative finite {label}")
    return number


def _validate_variant(variant: Any, seen_stems: set[str]) -> dict[str, Any]:
    """Validate one drawable product template against the same rules a fixed product obeys."""
    if not isinstance(variant, dict):
        raise RuntimeError("randomization variants must be mappings")
    if not variant.get("geometry_key"):
        raise RuntimeError("randomization variant requires a geometry_key")
    if variant.get("product_class") not in VALID_PRODUCT_CLASSES:
        raise RuntimeError(
            f"randomization variant has invalid product_class: {variant.get('product_class')}"
        )
    if not variant.get("sku"):
        raise RuntimeError("randomization variant requires a sku")
    _require_positive_finite(variant.get("mass_kg", 0.0), "variant mass_kg")
    _require_positive_finite(variant.get("friction", 0.0), "variant friction")

    rgba = variant.get("rgba", [])
    if not isinstance(rgba, list) or len(rgba) != 4:
        raise RuntimeError("randomization variant rgba must contain four components")
    for component in rgba:
        value = float(component)
        if not math.isfinite(value) or not 0.0 <= value <= 1.0:
            raise RuntimeError("randomization variant rgba components must lie in [0, 1]")

    # The stem, not the model name, is configured: the generator may draw the same variant more
    # than once and appends an occurrence index to keep backend identities unique.
    stem = variant.get("model_name_stem", "")
    if not stem or stem in seen_stems:
        raise RuntimeError(f"randomization variant model_name_stem is empty or duplicated: {stem}")
    seen_stems.add(stem)

    lanes = variant.get("destination_lanes", [])
    if not isinstance(lanes, list) or not lanes:
        raise RuntimeError(f"randomization variant {stem} requires at least one destination lane")
    if any(not lane for lane in lanes) or len(set(lanes)) != len(lanes):
        raise RuntimeError(f"randomization variant {stem} lanes must be non-empty and unique")
    return variant


def load_randomization(scenario: dict[str, Any]) -> dict[str, Any]:
    """Validate and return the seeded-generation bounds carried by a scenario."""
    # Rejects rather than defaults: a generated scenario that cannot be seated is worse than no
    # randomization, so every bound the placement search depends on must be stated.
    randomization = scenario.get("randomization")
    if not isinstance(randomization, dict):
        raise RuntimeError("scenario_config randomization must be a mapping")
    if randomization.get("generator_version") != SUPPORTED_GENERATOR_VERSION:
        raise RuntimeError(
            f"scenario_config randomization requires generator_version "
            f"{SUPPORTED_GENERATOR_VERSION}"
        )

    region = randomization.get("stock_region")
    if not isinstance(region, dict) or region.get("source") != STOCK_REGION_SOURCE:
        raise RuntimeError(f"randomization stock_region source must be {STOCK_REGION_SOURCE}")
    _require_non_negative_finite(region.get("wall_margin_m"), "stock_region wall_margin_m")
    _require_non_negative_finite(
        region.get("surface_separation_m"), "stock_region surface_separation_m"
    )
    attempts = region.get("max_placement_attempts")
    if not isinstance(attempts, int) or isinstance(attempts, bool) or attempts < 1:
        raise RuntimeError(
            "randomization stock_region max_placement_attempts must be a positive integer"
        )

    count = randomization.get("product_count")
    if not isinstance(count, dict):
        raise RuntimeError("randomization product_count must be a mapping")
    minimum = count.get("minimum")
    maximum = count.get("maximum")
    for label, value in (("minimum", minimum), ("maximum", maximum)):
        if not isinstance(value, int) or isinstance(value, bool) or value < 1:
            raise RuntimeError(f"randomization product_count {label} must be a positive integer")
    if maximum < minimum:
        raise RuntimeError("randomization product_count maximum must not be below minimum")

    variants = randomization.get("variants")
    if not isinstance(variants, list) or not variants:
        raise RuntimeError("randomization must declare at least one variant")
    seen_stems: set[str] = set()
    for variant in variants:
        _validate_variant(variant, seen_stems)

    # A lane accepts exactly one product (the baseline predicate refuses an occupied destination),
    # so a `maximum` above the lane count describes a request no robot could satisfy. Checked here
    # because it is a statement about the bounds, not about one seed.
    capacity = lane_capacity_by_class(variants)
    if maximum > sum(capacity.values()):
        raise RuntimeError(
            f"randomization product_count maximum {maximum} exceeds the "
            f"{sum(capacity.values())} lanes the declared variants can be delivered to"
        )
    return randomization


def lane_capacity_by_class(variants: Sequence[dict[str, Any]]) -> dict[str, int]:
    """Return how many products of each class the declared destination lanes can accept."""
    # Per class, over the union of the lanes: two variants of one class compete for the same lanes,
    # and a lane listed twice is one lane. `destination_lanes` is validated against
    # restocker_world_state's lane policy by test_scenario_random.py.
    capacity: dict[str, set[str]] = {}
    for variant in variants:
        capacity.setdefault(variant["product_class"], set()).update(variant["destination_lanes"])
    return {product_class: len(lanes) for product_class, lanes in capacity.items()}


def load_stock_region(path: str | Path) -> StockRegion:
    """Load the description-owned stock-tray placement envelope."""
    # The envelope lives in restocker_description because the lane and occupancy checks measure
    # against it; reading the same survey keeps a generated pose and its containment verdict
    # consistent.
    geometry = yaml.safe_load(Path(path).read_text(encoding="utf-8"))
    if not isinstance(geometry, dict) or geometry.get("schema_version") != 1:
        raise RuntimeError("workcell_geometry requires schema_version 1")
    volume = geometry.get("stock_tray", {}).get("usable_volume")
    if not isinstance(volume, dict):
        raise RuntimeError("workcell_geometry must declare stock_tray.usable_volume")
    center = volume.get("center_xyz_m", [])
    size = volume.get("size_xyz_m", [])
    if not isinstance(center, list) or len(center) != 3:
        raise RuntimeError("stock_tray usable_volume center_xyz_m must contain three values")
    if not isinstance(size, list) or len(size) != 3:
        raise RuntimeError("stock_tray usable_volume size_xyz_m must contain three values")
    for value in center:
        if not math.isfinite(float(value)):
            raise RuntimeError("stock_tray usable_volume center_xyz_m must be finite")
    return StockRegion(
        center_xyz_m=tuple(float(value) for value in center),
        size_xyz_m=tuple(
            _require_positive_finite(value, "stock_tray usable_volume size_xyz_m")
            for value in size
        ),
        floor_settle_tolerance_m=_require_non_negative_finite(
            volume.get("floor_settle_tolerance_m", 0.0),
            "stock_tray usable_volume floor_settle_tolerance_m",
        ),
    )


def load_approach_corridor(path: str | Path) -> ApproachCorridor:
    """Load the tube the arm sweeps to reach a product, as the planner collision-checks it."""
    # Reaching a product runs tool0 down a straight line into the tray with the jaws open, so every
    # link on the tool axis sweeps a tube of this radius and anything inside it is struck before
    # the grasp pose is reached. No arm configuration approaches a product whose neighbour is
    # inside the corridor, so such a scenario is unrecoverable.
    #
    # On the width: the fingers are not the widest sweeping part. They reach `open_target_m` past
    # their mounts, which is less far than `wrist_2_link`, `wrist_3_link` and `flange` stand out,
    # and the measured failures this rule was corrected for were the flange against a neighbour the
    # finger rule cleared by 70 mm. PLANNER_ROBOT_PADDING_M is part of the width, not a margin on
    # top: the planner checks links inflated on every face. The rule is a tangency bound, so at the
    # floor the difference is the whole margin.
    #
    # The finger half is read from the fields restocker_description generates the URDF from, so
    # changing the finger, its mount or the release aperture moves the rule. The aperture is
    # `open_target_m`, not the joint's upper limit: it is the widest position ever commanded and
    # where the jaws are parked for the whole approach.
    geometry = yaml.safe_load(Path(path).read_text(encoding="utf-8"))
    if not isinstance(geometry, dict) or geometry.get("schema_version") != 1:
        raise RuntimeError("gripper_geometry requires schema_version 1")
    finger = geometry.get("finger")
    attachment = geometry.get("attachment")
    if not isinstance(finger, dict) or not isinstance(attachment, dict):
        raise RuntimeError("gripper_geometry must declare finger and attachment")
    size = finger.get("size_xyz_m")
    if not isinstance(size, list) or len(size) != 3:
        raise RuntimeError("gripper_geometry finger size_xyz_m must contain three values")
    half_width = _require_positive_finite(size[1], "gripper finger width") / 2.0
    open_target = _require_non_negative_finite(
        attachment.get("open_target_m"), "gripper attachment open_target_m"
    )
    # Both jaws are measured; the corridor is bounded by whichever reaches furthest, so no symmetry
    # assumption is needed.
    offsets = []
    for side in ("left", "right"):
        mount = finger.get(side)
        origin = mount.get("origin_xyz_m") if isinstance(mount, dict) else None
        if not isinstance(origin, list) or len(origin) != 3:
            raise RuntimeError(f"gripper_geometry finger {side} origin_xyz_m must be three values")
        offsets.append(_require_positive_finite(abs(origin[1]), f"gripper {side} jaw offset"))
    finger_outer_face = max(offsets) + open_target + half_width

    grasp_center = geometry.get("grasp_center")
    datum = grasp_center.get("xyz_m") if isinstance(grasp_center, dict) else None
    if not isinstance(datum, list) or len(datum) != 3:
        raise RuntimeError("gripper_geometry grasp_center xyz_m must contain three values")
    # tool0 trails the grasp centre by the gripper's datum and the coordinator stands the pre-grasp
    # a further PREGRASP_DISTANCE_M back, so this is the whole stand-off of the planned frame.
    tool_from_grasp_center = _require_positive_finite(datum[2], "gripper grasp_center offset")

    return ApproachCorridor(
        half_width_m=max(finger_outer_face, WRIST_LINK_RADIUS_M) + PLANNER_ROBOT_PADDING_M,
        pregrasp_standoff_m=tool_from_grasp_center + PREGRASP_DISTANCE_M,
    )


def load_product_catalog(path: str | Path) -> ProductCatalog:
    """Load and validate the description-owned collision geometry catalog."""
    catalog = yaml.safe_load(Path(path).read_text(encoding="utf-8"))
    if not isinstance(catalog, dict) or catalog.get("schema_version") != 1:
        raise RuntimeError("product_catalog requires schema_version 1")
    geometries = catalog.get("geometries", [])
    if not isinstance(geometries, list) or not geometries:
        raise RuntimeError("product_catalog must contain at least one geometry")

    by_key: ProductCatalog = {}
    fallback_classes: set[str] = set()
    sku_keys: set[str] = set()
    for geometry in geometries:
        if not isinstance(geometry, dict):
            raise RuntimeError("product_catalog geometries must be mappings")
        key = geometry.get("geometry_key", "")
        product_class = geometry.get("product_class", "")
        shape = geometry.get("shape", {})
        if not key or key in by_key:
            raise RuntimeError(f"product_catalog geometry_key is empty or duplicated: {key}")
        if product_class not in VALID_PRODUCT_CLASSES:
            raise RuntimeError(f"product_catalog has invalid product_class: {product_class}")
        if not isinstance(shape, dict) or shape.get("type") != "cylinder":
            raise RuntimeError(f"product_catalog geometry {key} is not a supported cylinder")
        for field in ("radius_m", "height_m"):
            value = float(shape.get(field, 0.0))
            if not math.isfinite(value) or value <= 0.0:
                raise RuntimeError(f"product_catalog geometry {key} has invalid {field}")
        if geometry.get("class_fallback", False):
            if product_class in fallback_classes:
                raise RuntimeError(f"product_catalog has multiple fallbacks for {product_class}")
            fallback_classes.add(product_class)
        sku = geometry.get("sku")
        if sku:
            if sku in sku_keys:
                raise RuntimeError(f"product_catalog has duplicate SKU: {sku}")
            sku_keys.add(sku)
        by_key[key] = geometry
    return by_key


def resolve_product_geometry(product: dict[str, Any], catalog: ProductCatalog) -> dict[str, Any]:
    """Resolve one scenario product and reject semantic/catalog disagreement."""
    key = product.get("geometry_key", "")
    if key not in catalog:
        raise RuntimeError(f"scenario product references unknown geometry_key: {key}")
    geometry = catalog[key]
    if geometry["product_class"] != product.get("product_class"):
        raise RuntimeError(f"scenario product class disagrees with catalog geometry {key}")
    if geometry.get("sku") and geometry["sku"] != product.get("sku"):
        raise RuntimeError(f"scenario product SKU disagrees with catalog geometry {key}")
    return geometry

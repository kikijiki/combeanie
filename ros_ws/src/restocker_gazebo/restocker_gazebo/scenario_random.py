# Copyright 2026 Matteo Bernacchia <dev@kikijiki.com>
#
# SPDX-License-Identifier: MIT

"""Seeded, replayable generation of simulator scenarios from validated bounds."""
# Every value this module emits is a pure function of the seed and the validated configuration, so
# a failing randomized run can be replayed. Nothing here reads the clock, the process id, the
# environment, ``os.urandom``, ``hash()``, or any unordered container. See :class:`SplitMix64` and
# ``POSE_QUANTUM_M`` (output quantization, byte-identical on any machine).

from __future__ import annotations

from collections.abc import Sequence
import hashlib
import json
import math
from pathlib import Path
from typing import Any

import yaml

from restocker_gazebo.scenario_config import (  # noqa: I100,I101 - ament and Ruff disagree
    MINIMUM_PREGRASP_REACH_Y_M,
    ApproachCorridor,
    ProductCatalog,
    StockRegion,
    lane_capacity_by_class,
    load_approach_corridor,
    load_product_catalog,
    load_randomization,
    load_scenario,
    load_stock_region,
    resolve_product_geometry,
)

# Launch argument value that keeps the surveyed `products:` list as written. Seeding is opt-in, so
# the absence of a seed never means "randomize".
FIXED_SCENARIO_SEED = "fixed"

# Seeds are unsigned 64-bit so they round-trip through the generator state and can be quoted in a
# bug report.
SEED_MODULUS = 1 << 64

# Generated lengths are quantized to a micrometre before validation and writing, so the emitted
# YAML is byte-identical and the invariants are checked against the value Gazebo receives.
POSE_QUANTUM_DIGITS = 6
POSE_QUANTUM_M = 1e-6

_MASK64 = SEED_MODULUS - 1


class SplitMix64:
    """The SplitMix64 generator of Steele, Lea and Flood (2014), as used by Vigna."""

    # Three lines of published integer arithmetic with fixed constants, so the sequence is pinned
    # by this file rather than by a toolchain. Python's ``random`` is not used: the seed-to-output
    # mapping of ``randrange``/``sample``/``shuffle`` has changed between CPython releases.
    # ``std::default_random_engine`` and ``std::random_device`` are unusable for the same reason.
    #
    # All arithmetic is masked to 64 bits, so machine word size and overflow behaviour do not
    # participate in the result.

    __slots__ = ("_state",)

    # Odd golden-ratio increment and the two mixing multipliers (published constants).
    _GAMMA = 0x9E3779B97F4A7C15
    _MIX_A = 0xBF58476D1CE4E5B9
    _MIX_B = 0x94D049BB133111EB

    def __init__(self, seed: int) -> None:
        """Start the stream at ``seed`` reduced into the unsigned 64-bit state space."""
        self._state = int(seed) & _MASK64

    def next_u64(self) -> int:
        """Advance the stream and return the next unsigned 64-bit word."""
        self._state = (self._state + self._GAMMA) & _MASK64
        word = self._state
        word = ((word ^ (word >> 30)) * self._MIX_A) & _MASK64
        word = ((word ^ (word >> 27)) * self._MIX_B) & _MASK64
        return word ^ (word >> 31)

    def next_unit(self) -> float:
        """Return the next value in [0, 1) with 53 bits of resolution."""
        # The top 53 bits are exactly representable in binary64 and the scale is a power of two,
        # so the multiplication is exact.
        return (self.next_u64() >> 11) * (1.0 / 9007199254740992.0)

    def next_below(self, bound: int) -> int:
        """Return the next value in [0, ``bound``) with no modulo bias."""
        # Rejection rather than a plain modulo, which would over-weight the first few catalog
        # variants and lanes.
        if bound <= 0:
            raise RuntimeError("scenario_random requires a positive draw bound")
        limit = SEED_MODULUS - (SEED_MODULUS % bound)
        while True:
            word = self.next_u64()
            if word < limit:
                return word % bound

    def next_between(self, low: float, high: float) -> float:
        """Return the next value in [``low``, ``high``] as a quantized length in metres."""
        if not (math.isfinite(low) and math.isfinite(high)) or high < low:
            raise RuntimeError("scenario_random requires a finite non-empty draw interval")
        drawn = low + (high - low) * self.next_unit()
        return round(drawn, POSE_QUANTUM_DIGITS)


def parse_seed(value: str) -> int | None:
    """Interpret a ``scenario_seed`` launch argument, returning None for the fixed scenario."""
    # Fails closed on anything that is neither the fixed sentinel nor a plain unsigned integer, so
    # a mistyped seed stops the launch.
    text = value.strip()
    if text == FIXED_SCENARIO_SEED:
        return None
    if not text.isdigit():
        raise RuntimeError(
            f"scenario_seed must be '{FIXED_SCENARIO_SEED}' or a decimal unsigned integer: {value}"
        )
    seed = int(text)
    if seed >= SEED_MODULUS:
        raise RuntimeError(f"scenario_seed must be below 2^64: {value}")
    return seed


def _round_pose_value(value: float) -> float:
    """Quantize one emitted length so the written document is byte-stable."""
    return round(float(value), POSE_QUANTUM_DIGITS)


def _bounds_digest(
    randomization: dict[str, Any],
    region: StockRegion,
    catalog: ProductCatalog,
    workcell_pose: Sequence[float],
    corridor: ApproachCorridor,
) -> str:
    """Fingerprint every input that participates in a draw, so a replay can prove it matches."""
    # Recorded in the generated document so a seed that no longer reproduces shows up as a
    # configuration change. Absolute paths are excluded because they differ between machines.
    variants = randomization["variants"]
    payload = {
        "generator_version": randomization["generator_version"],
        "product_count": randomization["product_count"],
        "stock_region": randomization["stock_region"],
        "variants": variants,
        "region": {
            "center_xyz_m": list(region.center_xyz_m),
            "size_xyz_m": list(region.size_xyz_m),
            "floor_settle_tolerance_m": region.floor_settle_tolerance_m,
        },
        "workcell_pose": [float(value) for value in workcell_pose],
        # Derived from the description and the coordinator's grasp configuration, not stated in
        # the bounds. Both set which candidates are rejected, so they are fingerprinted like any
        # other input. MINIMUM_PREGRASP_REACH_Y_M is included for the same reason.
        "corridor_half_width_m": float(corridor.half_width_m),
        "pregrasp_standoff_m": float(corridor.pregrasp_standoff_m),
        "minimum_pregrasp_reach_y_m": float(MINIMUM_PREGRASP_REACH_Y_M),
        "shapes": {
            variant["geometry_key"]: catalog[variant["geometry_key"]]["shape"]
            for variant in variants
        },
    }
    canonical = json.dumps(payload, sort_keys=True, separators=(",", ":"))
    return hashlib.sha256(canonical.encode("utf-8")).hexdigest()


def _placement_interval(
    center: float, half_extent: float, radius: float, margin: float, axis: str
) -> tuple[float, float]:
    """Return the admissible interval for one horizontal centre coordinate."""
    reach = half_extent - radius - margin
    if reach < 0.0:
        raise RuntimeError(
            f"scenario_random cannot place a radius {radius} m product inside the stock region "
            f"on {axis}: the usable half extent {half_extent} m does not clear the "
            f"{margin} m wall margin"
        )
    return (center - reach, center + reach)


def reachable_y_maximum(corridor: ApproachCorridor) -> float:
    """Return the largest world y a product centre may take and still admit a pre-grasp."""
    # The approach runs along -y from a pre-grasp ``pregrasp_standoff_m`` back along it, so a
    # product at ``y`` puts tool0 at ``y + standoff``. Past MINIMUM_PREGRASP_REACH_Y_M some
    # collision-free arm branches cannot follow the straight line, and the coordinator does not
    # choose the branch. See MINIMUM_PREGRASP_REACH_Y_M for the measurement.
    #
    # World y, not tray-relative depth, because the bound is the arm's: the rail axis lies on
    # y = 0 and the tray is at negative y.
    return -(MINIMUM_PREGRASP_REACH_Y_M + corridor.pregrasp_standoff_m)


def _reachable_interval(
    interval: tuple[float, float], corridor: ApproachCorridor, geometry_key: str
) -> tuple[float, float]:
    """Clip one containment interval on y to the part of the tray a pre-grasp can reach."""
    limit = reachable_y_maximum(corridor)
    if interval[0] > limit:
        raise RuntimeError(
            f"scenario_random cannot place {geometry_key} in the stock region: no part of its "
            f"y interval {interval} is within reach of a pre-grasp, which needs y <= {limit}"
        )
    return (interval[0], min(interval[1], limit))


def required_center_distance(radius_m: float, other_radius_m: float, separation: float) -> float:
    """Return the centre distance one pair of products must keep to seat without disturbing."""
    # Physical seating only: cylinders closer than their radii plus the surface gap are in contact
    # from the first physics step. The reach bound is separate; see
    # :func:`required_lateral_distance`.
    return radius_m + other_radius_m + separation


def required_lateral_distance(
    radius_m: float, other_radius_m: float, corridor_half_width: float
) -> float:
    """Return the across-approach distance one pair must keep for either to be reachable."""
    # The bound a scenario cannot recover from, measured across the approach rather than around the
    # grasp.
    #
    # Reaching a product runs tool0 down a straight line into the tray, so a tube along that line
    # must be clear. Every link from ``wrist_2_link`` forward shares the tool axis and sweeps it;
    # ``corridor_half_width`` is the widest of them as the planner inflates it, from
    # :func:`scenario_config.load_approach_corridor`.
    #
    # The tube's length does not enter the rule (version 5 correction). Versions 3 and 4 compared
    # the full centre distance against a corridor radius, which treats a neighbour between the
    # target and the robot as far away when it is in the path. The tray is 0.51 m deep and the
    # swept tube is longer than that in the only direction a neighbour can be drawn in, so
    # whichever product stands nearer the robot is in the other's corridor. Only the
    # across-approach distance remains.
    #
    # It is asymmetric in the neighbour's radius (reaching A past B needs
    # ``corridor + radius(B)``), and both products are grasp targets because the generator cannot
    # know which the coordinator picks. The pair is admissible only if both hold, so the larger
    # radius governs.
    return corridor_half_width + max(radius_m, other_radius_m)


def _overlaps(
    x_m: float,
    y_m: float,
    radius_m: float,
    placed: list[dict[str, float]],
    separation: float,
    corridor_half_width: float,
) -> bool:
    """Report whether a candidate breaks any placed product's seating or corridor distance."""
    # Both tests are applied although at the shipped values the corridor implies the seating gap:
    # the narrowest admissible corridor is 0.1345 m and the widest gap physics asks for is
    # 0.110 m, so the tightest surface gap in a sweep is around 0.057 m rather than the configured
    # 0.020 m. They bound different geometry, and a smaller corridor or a larger product would
    # invert the implication.
    for other in placed:
        dx = x_m - other["x_m"]
        dy = y_m - other["y_m"]
        seating = required_center_distance(radius_m, other["radius_m"], separation)
        if (dx * dx) + (dy * dy) < seating * seating:
            return True
        if abs(dx) < required_lateral_distance(radius_m, other["radius_m"], corridor_half_width):
            return True
    return False


def generate_scenario(
    base_scenario: dict[str, Any],
    catalog: ProductCatalog,
    region: StockRegion,
    corridor: ApproachCorridor,
    seed: int,
) -> dict[str, Any]:
    """Draw one concrete scenario from ``base_scenario``'s bounds for ``seed``."""
    # The returned document has the same schema as the hand-written baseline, so existing
    # consumers (launch expansion, ground-truth adapter, attachment boundary loader) read it
    # unchanged.
    #
    # The draw order is part of the contract: the product count, then per product the variant
    # index, then the rejected and accepted (x, y) pairs. Reordering it, or adding or removing a
    # draw, invalidates every recorded seed and requires bumping
    # ``scenario_config.SUPPORTED_GENERATOR_VERSION``, which is fingerprinted into
    # ``bounds_digest_sha256``.
    #
    # Version 5 keeps that order (one count draw, then one variant draw and one or more (x, y)
    # pairs per product) and changes only what each draw is drawn from: the variant from the
    # classes that still have a free lane, and y from the reachable part of the tray.
    #
    # Version 2 removed a per-product draw of ``destination_lane`` that nothing read (the
    # coordinator chooses a destination at runtime from the world state).
    #
    # Version 3 widened the rejection test from surface separation to
    # :func:`required_center_distance`; a candidate the old rule accepted is now redrawn, which
    # shifts every later draw.
    #
    # Version 4 widened the same test by the planner's robot padding, with the same consequence.
    #
    # Version 5 replaced that test with :func:`required_lateral_distance`, cut the y interval to
    # the reachable part of the tray, and drew the variant from classes with a free lane. Each
    # changes what a seed means, and the first two shift the stream from the first rejected
    # candidate, so no version 4 seed replays. See ``scenario_config.SUPPORTED_GENERATOR_VERSION``.
    randomization = load_randomization(base_scenario)
    workcell_pose = [float(value) for value in base_scenario["workcell_pose"]]
    # A rotated workcell would need the region envelope and every cylinder footprint rotated with
    # it. Nothing uses one, so this fails closed instead of shipping an untested transform.
    if any(abs(angle) > 0.0 for angle in workcell_pose[3:6]):
        raise RuntimeError(
            "scenario_random supports randomized placement only for an axis-aligned workcell"
        )

    stock_region = randomization["stock_region"]
    margin = float(stock_region["wall_margin_m"])
    separation = float(stock_region["surface_separation_m"])
    attempt_budget = int(stock_region["max_placement_attempts"])
    variants = randomization["variants"]
    minimum = int(randomization["product_count"]["minimum"])
    maximum = int(randomization["product_count"]["maximum"])

    # World-frame envelope: the region is surveyed in the shelf frame and spawn poses are in world;
    # the workcell pose is the only transform between them.
    center = [workcell_pose[axis] + region.center_xyz_m[axis] for axis in range(3)]
    half = [extent / 2.0 for extent in region.size_xyz_m]
    floor_z = center[2] - half[2]
    ceiling_z = center[2] + half[2]

    stream = SplitMix64(seed)
    count = minimum + stream.next_below(maximum - minimum + 1)

    # How many more products of each class the shelf can still accept. A lane holds one product
    # (the baseline predicate refuses an occupied destination), so a class drawn more times than it
    # has lanes would leave a surplus with nowhere to go and read as a robot failure
    # (NO_COMPATIBLE_PAIR). ``load_randomization`` already refuses a ``maximum`` above the total,
    # so the pool below can never empty.
    remaining_lanes = lane_capacity_by_class(variants)

    placed: list[dict[str, float]] = []
    products: list[dict[str, Any]] = []
    used_stems: dict[str, int] = {}
    for _ in range(count):
        admissible = [
            variant for variant in variants if remaining_lanes[variant["product_class"]] > 0
        ]
        variant = admissible[stream.next_below(len(admissible))]
        remaining_lanes[variant["product_class"]] -= 1
        geometry = resolve_product_geometry(variant, catalog)
        radius_m = float(geometry["shape"]["radius_m"])
        height_m = float(geometry["shape"]["height_m"])
        if floor_z + height_m > ceiling_z:
            raise RuntimeError(
                f"scenario_random cannot stand {variant['geometry_key']} in the stock region: "
                f"its {height_m} m height exceeds the usable volume"
            )
        x_interval = _placement_interval(center[0], half[0], radius_m, margin, "x")
        y_interval = _reachable_interval(
            _placement_interval(center[1], half[1], radius_m, margin, "y"),
            corridor,
            variant["geometry_key"],
        )

        for _attempt in range(attempt_budget):
            x_m = stream.next_between(*x_interval)
            y_m = stream.next_between(*y_interval)
            # Quantization happens inside the draw; re-check containment on the value that is
            # written.
            contained = (
                x_interval[0] - POSE_QUANTUM_M <= x_m <= x_interval[1] + POSE_QUANTUM_M
                and y_interval[0] - POSE_QUANTUM_M <= y_m <= y_interval[1] + POSE_QUANTUM_M
            )
            if contained and not _overlaps(
                x_m, y_m, radius_m, placed, separation, corridor.half_width_m
            ):
                break
        else:
            raise RuntimeError(
                f"scenario_random exhausted {attempt_budget} placement attempts for "
                f"{variant['geometry_key']}; loosen the bounds or lower product_count.maximum"
            )

        stem = variant["model_name_stem"]
        used_stems[stem] = used_stems.get(stem, 0) + 1
        model_name = f"{stem}_{used_stems[stem]:02d}"
        placed.append({"x_m": x_m, "y_m": y_m, "radius_m": radius_m})
        products.append(
            {
                "model_name": model_name,
                "source_object_id": f"sim:{model_name}",
                "geometry_key": variant["geometry_key"],
                "product_class": variant["product_class"],
                "sku": variant["sku"],
                "mass_kg": float(variant["mass_kg"]),
                "rgba": [float(component) for component in variant["rgba"]],
                "friction": float(variant["friction"]),
                # The product rests on the tray surface (the floor of the usable volume). Height is
                # not drawn: a product spawned above the surface falls to a pose the seed did not
                # choose. Orientation is not drawn: every catalog shape is a cylinder, so yaw
                # changes nothing.
                "spawn_pose": [
                    x_m,
                    y_m,
                    _round_pose_value(floor_z + (height_m / 2.0)),
                    0.0,
                    0.0,
                    0.0,
                ],
            }
        )

    generated = {
        key: base_scenario[key]
        for key in (
            "schema_version",
            "pose_topic",
            "frame_id",
            "backend",
            "pose_covariance_diagonal",
            "workcell_pose",
        )
        if key in base_scenario
    }
    generated["generated_from"] = {
        "generator": "restocker_gazebo.scenario_random",
        "generator_version": randomization["generator_version"],
        "algorithm": "splitmix64",
        "seed": int(seed),
        "bounds_digest_sha256": _bounds_digest(
            randomization, region, catalog, workcell_pose, corridor
        ),
    }
    generated["products"] = products
    return generated


def generate_scenario_from_files(
    scenario_path: str | Path,
    catalog_path: str | Path,
    workcell_geometry_path: str | Path,
    gripper_geometry_path: str | Path,
    seed: int,
) -> dict[str, Any]:
    """Load every validated input from disk and generate the scenario for ``seed``."""
    return generate_scenario(
        load_scenario(scenario_path),
        load_product_catalog(catalog_path),
        load_stock_region(workcell_geometry_path),
        load_approach_corridor(gripper_geometry_path),
        seed,
    )


def serialize_scenario(scenario: dict[str, Any]) -> str:
    """Render a generated scenario to the exact text that is written and replayed."""
    # ``sort_keys=False`` keeps the baseline's key order; every scalar is an integer, a string, or
    # an already-quantized length, so the rendering is byte-stable.
    header = (
        "# Generated by restocker_gazebo.scenario_random. Do not edit: rerun the launch with the\n"
        "# same scenario_seed to reproduce this file exactly.\n"
    )
    return header + yaml.safe_dump(scenario, sort_keys=False, default_flow_style=False)


def write_generated_scenario(scenario: dict[str, Any], directory: str | Path) -> Path:
    """Write a generated scenario under ``directory``, named for its seed, and return the path."""
    # The content is a pure function of the seed and bounds, so rewriting an existing file is a
    # no-op in substance. The path is stable so an operator can archive the file a failing run
    # used.
    seed = int(scenario["generated_from"]["seed"])
    target = Path(directory)
    # Owner-only: the default is a shared temporary directory, and a scenario another account can
    # rewrite is one the seed no longer describes.
    target.mkdir(mode=0o700, parents=True, exist_ok=True)
    path = target / f"scenario_seed_{seed}.yaml"
    path.write_text(serialize_scenario(scenario), encoding="utf-8")
    return path


def placement_violations(
    scenario: dict[str, Any],
    catalog: ProductCatalog,
    region: StockRegion,
    surface_separation_m: float = 0.0,
    corridor: ApproachCorridor | None = None,
) -> list[str]:
    """Return every physical invariant a scenario's spawn poses break, empty when seatable."""
    # Independent checker for what :func:`generate_scenario` promises, run against the emitted
    # document: containment inside the usable volume, resting contact with the tray surface, a
    # reachable pre-grasp, and pairwise separation (surface gap and approach corridor).
    workcell_pose = [float(value) for value in scenario["workcell_pose"]]
    center = [workcell_pose[axis] + region.center_xyz_m[axis] for axis in range(3)]
    half = [extent / 2.0 for extent in region.size_xyz_m]
    floor_z = center[2] - half[2]
    ceiling_z = center[2] + half[2]
    # Gazebo settles a resting body a fraction of a micrometre into its support, so the surveyed
    # floor is checked with the description-owned allowance.
    settle_floor_z = floor_z - region.floor_settle_tolerance_m

    violations: list[str] = []
    placed: list[tuple[str, float, float, float]] = []
    for product in scenario["products"]:
        name = product["model_name"]
        shape = resolve_product_geometry(product, catalog)["shape"]
        radius_m = float(shape["radius_m"])
        height_m = float(shape["height_m"])
        x_m, y_m, z_m = (float(value) for value in product["spawn_pose"][:3])

        if abs(x_m - center[0]) + radius_m > half[0] + POSE_QUANTUM_M:
            violations.append(f"{name} leaves the stock region on x")
        if abs(y_m - center[1]) + radius_m > half[1] + POSE_QUANTUM_M:
            violations.append(f"{name} leaves the stock region on y")
        bottom_z = z_m - (height_m / 2.0)
        if bottom_z < settle_floor_z - POSE_QUANTUM_M:
            violations.append(f"{name} is sunk below the tray surface")
        if bottom_z > floor_z + POSE_QUANTUM_M:
            violations.append(f"{name} floats above the tray surface")
        if z_m + (height_m / 2.0) > ceiling_z + POSE_QUANTUM_M:
            violations.append(f"{name} stands out of the stock region")
        # Reported separately from containment: the product seats inside the tray, but the arm has
        # no configuration that can run the straight line in to it.
        if corridor is not None and y_m > reachable_y_maximum(corridor) + POSE_QUANTUM_M:
            violations.append(f"{name} stands where no pre-grasp is reachable")

        for other_name, other_x, other_y, other_radius in placed:
            distance = math.hypot(x_m - other_x, y_m - other_y)
            if distance - radius_m - other_radius < surface_separation_m - POSE_QUANTUM_M:
                violations.append(f"{name} is closer than the required gap to {other_name}")
            # Reported separately from the surface gap: the pair seats and settles, but neither
            # product can be approached.
            elif (
                corridor is not None
                and abs(x_m - other_x)
                < required_lateral_distance(radius_m, other_radius, corridor.half_width_m)
                - POSE_QUANTUM_M
            ):
                violations.append(f"{name} blocks the approach corridor to {other_name}")
        placed.append((name, x_m, y_m, radius_m))
    return violations

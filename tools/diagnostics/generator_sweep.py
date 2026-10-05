#!/usr/bin/env python3
# Copyright 2026 Matteo Bernacchia <dev@kikijiki.com>
# SPDX-License-Identifier: MIT

"""Sweep the seeded scenario generator, and replay superseded generator versions beside it.

The offline planning probe measures geometry. This measures the draw: how often the placement
search runs out of attempts, and what a version-4 seed would have produced so that version 5's
own before-and-after table can be recomputed rather than quoted.

Version 4 is reimplemented here rather than recovered from history because the shipped module
only knows how to draw version 5, and a version comparison needs both. Its two differences from
version 5 are stated in :class:`Version4Rules` and nowhere else, so the reimplementation cannot
quietly drift into a third thing.

Nothing here imports the probe or needs a simulator. See tools/diagnostics/README.md.
"""

from __future__ import annotations

import argparse
import csv
from dataclasses import dataclass
from pathlib import Path
import sys
from typing import Any

import yaml

REPOSITORY_ROOT = Path(__file__).resolve().parents[2]
GAZEBO_PACKAGE = REPOSITORY_ROOT / "ros_ws" / "src" / "restocker_gazebo"
DESCRIPTION_CONFIG = REPOSITORY_ROOT / "ros_ws" / "src" / "restocker_description" / "config"

if str(GAZEBO_PACKAGE) not in sys.path:
    sys.path.insert(0, str(GAZEBO_PACKAGE))

from restocker_gazebo import scenario_config, scenario_random  # noqa: E402


@dataclass(frozen=True)
class Version4Rules:
    """What generator version 4 did differently, and the whole of it.

    Three things changed in version 5, and each is switched off here:

    * the pairwise corridor was measured as a **centre distance** against a half-width taken from
      the padded finger alone, rather than as an **across-approach** distance against the widest
      coaxial link;
    * the y interval was the whole containable part of the tray, with no reachability clip;
    * the variant was drawn from every declared variant, not only from the classes whose
      destination lanes are not yet spoken for.

    The draw *order* is identical in both versions, one count draw, then one variant draw and
    one or more (x, y) pairs per product, which is what makes a seed-by-seed comparison mean
    anything at all.
    """

    corridor_half_width_m: float


def version_4_corridor(gripper_geometry_path: Path) -> Version4Rules:
    """Return version 4's corridor half-width, derived the way version 4 derived it."""
    # The padded finger outer face, and nothing else: version 4 did not know about wrist_2_link.
    # Read from the same YAML version 5 reads, so the two versions cannot be compared across a
    # stale copy of the gripper survey.
    geometry = yaml.safe_load(Path(gripper_geometry_path).read_text(encoding="utf-8"))
    finger = geometry["finger"]
    half_width = float(finger["size_xyz_m"][1]) / 2.0
    open_target = float(geometry["attachment"]["open_target_m"])
    mount = max(
        abs(float(finger["left"]["origin_xyz_m"][1])),
        abs(float(finger["right"]["origin_xyz_m"][1])),
    )
    padding = scenario_config.PLANNER_ROBOT_PADDING_M
    return Version4Rules(corridor_half_width_m=mount + open_target + half_width + padding)


def _version_4_overlaps(
    x_m: float,
    y_m: float,
    radius_m: float,
    placed: list[dict[str, float]],
    separation: float,
    corridor_half_width: float,
) -> bool:
    """Report whether a candidate breaks version 4's seating or corridor distance."""
    for other in placed:
        dx = x_m - other["x_m"]
        dy = y_m - other["y_m"]
        seating = scenario_random.required_center_distance(radius_m, other["radius_m"], separation)
        if (dx * dx) + (dy * dy) < seating * seating:
            return True
        # The version 4 rule: a circle about the grasp, compared against the full centre distance.
        corridor = corridor_half_width + max(radius_m, other["radius_m"])
        if (dx * dx) + (dy * dy) < corridor * corridor:
            return True
    return False


def generate_version_4(
    base_scenario: dict[str, Any],
    catalog: scenario_config.ProductCatalog,
    region: scenario_config.StockRegion,
    rules: Version4Rules,
    seed: int,
) -> list[dict[str, Any]]:
    """Draw one version-4 scenario, in version 4's draw order and against version 4's rules."""
    randomization = base_scenario["randomization"]
    workcell_pose = [float(value) for value in base_scenario["workcell_pose"]]
    stock_region = randomization["stock_region"]
    margin = float(stock_region["wall_margin_m"])
    separation = float(stock_region["surface_separation_m"])
    attempt_budget = int(stock_region["max_placement_attempts"])
    variants = randomization["variants"]
    minimum = int(randomization["product_count"]["minimum"])
    maximum = int(randomization["product_count"]["maximum"])

    center = [workcell_pose[axis] + region.center_xyz_m[axis] for axis in range(3)]
    half = [extent / 2.0 for extent in region.size_xyz_m]
    floor_z = center[2] - half[2]

    stream = scenario_random.SplitMix64(seed)
    count = minimum + stream.next_below(maximum - minimum + 1)

    placed: list[dict[str, float]] = []
    products: list[dict[str, Any]] = []
    for _ in range(count):
        # Version 4 drew from every variant; the free-lane pool is a version 5 addition.
        variant = variants[stream.next_below(len(variants))]
        geometry = scenario_config.resolve_product_geometry(variant, catalog)
        radius_m = float(geometry["shape"]["radius_m"])
        height_m = float(geometry["shape"]["height_m"])
        x_interval = scenario_random._placement_interval(center[0], half[0], radius_m, margin, "x")
        # Version 4 had no reachability clip: the whole containable interval was drawable.
        y_interval = scenario_random._placement_interval(center[1], half[1], radius_m, margin, "y")
        for _attempt in range(attempt_budget):
            x_m = stream.next_between(*x_interval)
            y_m = stream.next_between(*y_interval)
            if not _version_4_overlaps(
                x_m, y_m, radius_m, placed, separation, rules.corridor_half_width_m
            ):
                break
        else:
            raise RuntimeError(f"version 4 exhausted its placement budget for seed {seed}")
        placed.append({"x_m": x_m, "y_m": y_m, "radius_m": radius_m})
        products.append(
            {
                "geometry_key": variant["geometry_key"],
                "product_class": variant["product_class"],
                "x_m": x_m,
                "y_m": y_m,
                "z_m": round(floor_z + (height_m / 2.0), scenario_random.POSE_QUANTUM_DIGITS),
                "radius_m": radius_m,
                "height_m": height_m,
            }
        )
    return products


def _load_inputs(scenario_path: Path) -> tuple[dict[str, Any], Any, Any, Any]:
    """Load the four validated inputs a draw needs, from the files that own them."""
    base = scenario_config.load_scenario(scenario_path)
    catalog = scenario_config.load_product_catalog(
        DESCRIPTION_CONFIG / "product_collision_catalog.yaml"
    )
    region = scenario_config.load_stock_region(DESCRIPTION_CONFIG / "workcell_geometry.yaml")
    corridor = scenario_config.load_approach_corridor(DESCRIPTION_CONFIG / "gripper_geometry.yaml")
    return base, catalog, region, corridor


def _with_budget(base: dict[str, Any], budget: int) -> dict[str, Any]:
    """Return the scenario with one placement-attempt budget substituted."""
    randomization = dict(base["randomization"])
    randomization["stock_region"] = dict(base["randomization"]["stock_region"])
    randomization["stock_region"]["max_placement_attempts"] = budget
    return {**base, "randomization": randomization}


def run_budget(arguments: argparse.Namespace) -> int:
    """Count the seeds whose placement search runs out of attempts, at several budgets.

    The draw for one seed is the same sequence at every budget, the budget only decides how
    far down it the search is allowed to go, so a seed that survives a small budget survives
    every larger one, and the counts below are nested by construction.
    """
    base, catalog, region, corridor = _load_inputs(arguments.scenario)
    budgets = sorted(int(value) for value in arguments.budgets.split(","))
    print(f"# generator_sweep budget seeds={arguments.first}..{arguments.last}")
    print(f"# corridor_half_width_m={corridor.half_width_m:.6f}")
    print(f"# pregrasp_standoff_m={corridor.pregrasp_standoff_m:.6f}")
    print("budget,seeds,exhausted,counts_of_exhausted,exhausted_seeds")
    # Only the seeds that failed the previous budget can fail a larger one, so each pass runs
    # over the survivors of the last rather than over the whole range again.
    candidates = list(range(arguments.first, arguments.last + 1))
    span = len(candidates)
    for budget in budgets:
        scenario = _with_budget(base, budget)
        exhausted: list[int] = []
        for seed in candidates:
            try:
                scenario_random.generate_scenario(scenario, catalog, region, corridor, seed)
            except RuntimeError:
                exhausted.append(seed)
        # What the draw asked for, which is the interesting part: an exhausted seed that drew
        # fewer than the maximum count would be a much worse result than one that drew six.
        counts = sorted(
            {_drawn_count(scenario, catalog, region, corridor, seed) for seed in exhausted}
        )
        listed = " ".join(str(seed) for seed in exhausted[:20])
        counted = " ".join(str(count) for count in counts)
        print(f"{budget},{span},{len(exhausted)},{counted},{listed}")
        candidates = exhausted
    return 0


def _drawn_count(
    scenario: dict[str, Any],
    catalog: scenario_config.ProductCatalog,
    region: scenario_config.StockRegion,
    corridor: scenario_config.ApproachCorridor,
    seed: int,
) -> int:
    """Return the product count one seed drew, which the placement search never changes."""
    # The count is the first draw off the stream and no rejection touches it, so it can be read
    # off a fresh generator without replaying the placement search at all.
    randomization = scenario["randomization"]
    minimum = int(randomization["product_count"]["minimum"])
    maximum = int(randomization["product_count"]["maximum"])
    del catalog, region, corridor
    return minimum + scenario_random.SplitMix64(seed).next_below(maximum - minimum + 1)


def run_placements(arguments: argparse.Namespace) -> int:
    """Emit every drawn product of every seed as a CSV the planning probe can read back."""
    base, catalog, region, corridor = _load_inputs(arguments.scenario)
    rules = version_4_corridor(DESCRIPTION_CONFIG / "gripper_geometry.yaml")
    writer = csv.writer(sys.stdout)
    print(f"# generator_sweep placements version={arguments.version}")
    print(f"# seeds={arguments.first}..{arguments.last}")
    if arguments.version == 4:
        print(f"# version_4_corridor_half_width_m={rules.corridor_half_width_m:.6f}")
    else:
        print(f"# corridor_half_width_m={corridor.half_width_m:.6f}")
    writer.writerow(["scenario", "product", "geometry_key", "x_m", "y_m"])
    for seed in range(arguments.first, arguments.last + 1):
        if arguments.version == 4:
            products = generate_version_4(base, catalog, region, rules, seed)
            rows = [(item["geometry_key"], item["x_m"], item["y_m"]) for item in products]
        else:
            drawn = scenario_random.generate_scenario(base, catalog, region, corridor, seed)
            rows = [
                (item["geometry_key"], item["spawn_pose"][0], item["spawn_pose"][1])
                for item in drawn["products"]
            ]
        for index, (key, x_m, y_m) in enumerate(rows):
            writer.writerow([seed, index, key, f"{x_m:.6f}", f"{y_m:.6f}"])
    return 0


def run_intervals(arguments: argparse.Namespace) -> int:
    """Print the drawable y interval per product class, before and after the reachability clip."""
    _, catalog, region, corridor = _load_inputs(arguments.scenario)
    limit = scenario_random.reachable_y_maximum(corridor)
    print("# generator_sweep intervals")
    print(f"# reachable_y_maximum_m={limit:.6f}")
    print("geometry_key,y_low_m,y_high_m,unclipped_span_m,clipped_span_m,cost_m")
    workcell_pose = [0.0, 0.55, 0.75]
    center_y = workcell_pose[1] + region.center_xyz_m[1]
    half_y = region.size_xyz_m[1] / 2.0
    for key, geometry in sorted(catalog.items()):
        radius = float(geometry["shape"]["radius_m"])
        reach = half_y - radius - 0.01
        low = center_y - reach
        high = center_y + reach
        clipped_high = min(high, limit)
        print(
            f"{key},{low:.6f},{high:.6f},{high - low:.6f},"
            f"{clipped_high - low:.6f},{high - clipped_high:.6f}"
        )
    return 0


def main() -> int:
    """Parse arguments and dispatch one sweep."""
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--scenario",
        type=Path,
        default=GAZEBO_PACKAGE / "config" / "baseline_products.yaml",
        help="the scenario carrying the randomization bounds",
    )
    parser.add_argument("--first", type=int, default=0, help="first seed, inclusive")
    parser.add_argument("--last", type=int, default=19999, help="last seed, inclusive")
    parser.add_argument(
        "--mode",
        choices=("budget", "placements", "intervals"),
        required=True,
        help="budget: placement-attempt exhaustion; placements: draws for the probe; "
        "intervals: the drawable y interval per class",
    )
    parser.add_argument("--budgets", default="256,1024,16384", help="comma-separated budgets")
    parser.add_argument("--version", type=int, default=5, choices=(4, 5), help="generator version")
    arguments = parser.parse_args()
    if arguments.mode == "budget":
        return run_budget(arguments)
    if arguments.mode == "placements":
        return run_placements(arguments)
    return run_intervals(arguments)


if __name__ == "__main__":
    raise SystemExit(main())

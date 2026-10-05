# Copyright 2026 Matteo Bernacchia <dev@kikijiki.com>
#
# SPDX-License-Identifier: MIT

"""Unit tests for seeded scenario generation, replayability, and placement invariants."""

import copy
import importlib.util
import math
import os
from pathlib import Path
import re
import subprocess
import sys

from launch import LaunchContext
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    LogInfo,
    SetEnvironmentVariable,
)
from launch.utilities import perform_substitutions
import pytest
from restocker_gazebo.scenario_config import (  # noqa: I101 - ament and Ruff disagree
    MINIMUM_PREGRASP_REACH_Y_M,
    PLANNER_ROBOT_PADDING_M,
    PREGRASP_DISTANCE_M,
    WRIST_LINK_RADIUS_M,
    lane_capacity_by_class,
    load_approach_corridor,
    load_product_catalog,
    load_randomization,
    load_scenario,
    load_stock_region,
    resolve_product_geometry,
)
from restocker_gazebo.scenario_random import (  # noqa: I101 - ament and Ruff disagree
    FIXED_SCENARIO_SEED,
    POSE_QUANTUM_M,
    SplitMix64,
    generate_scenario,
    generate_scenario_from_files,
    parse_seed,
    placement_violations,
    reachable_y_maximum,
    required_lateral_distance,
    serialize_scenario,
    write_generated_scenario,
)
import yaml

GAZEBO_SOURCE = Path(os.environ["RESTOCKER_GAZEBO_SOURCE_DIR"])
DESCRIPTION_SOURCE = Path(os.environ["RESTOCKER_DESCRIPTION_SOURCE_DIR"])
MOVEIT_CONFIG_SOURCE = Path(os.environ["RESTOCKER_MOVEIT_CONFIG_SOURCE_DIR"])
WORLD_STATE_SOURCE = Path(os.environ["RESTOCKER_WORLD_STATE_SOURCE_DIR"])
TASK_EXECUTOR_SOURCE = Path(os.environ["RESTOCKER_TASK_EXECUTOR_SOURCE_DIR"])
BASELINE_SCENARIO = GAZEBO_SOURCE / "config" / "baseline_products.yaml"
PRODUCT_CATALOG = DESCRIPTION_SOURCE / "config" / "product_collision_catalog.yaml"
WORKCELL_GEOMETRY = DESCRIPTION_SOURCE / "config" / "workcell_geometry.yaml"
GRIPPER_GEOMETRY = DESCRIPTION_SOURCE / "config" / "gripper_geometry.yaml"

# Enough seeds that every product count and variant combination is drawn many times and a rule
# binding on a few pairs in a thousand is exercised. 4000 seeds draw about 14,000 products and
# 24,000 pairs and finish well inside the package test timeout.
INVARIANT_SEEDS = range(4000)


@pytest.fixture(scope="module")
def catalog():
    return load_product_catalog(PRODUCT_CATALOG)


@pytest.fixture(scope="module")
def region():
    return load_stock_region(WORKCELL_GEOMETRY)


@pytest.fixture(scope="module")
def corridor():
    return load_approach_corridor(GRIPPER_GEOMETRY)


@pytest.fixture(scope="module")
def baseline():
    return load_scenario(BASELINE_SCENARIO)


def _generate(seed: int) -> dict:
    return generate_scenario_from_files(
        BASELINE_SCENARIO, PRODUCT_CATALOG, WORKCELL_GEOMETRY, GRIPPER_GEOMETRY, seed
    )


@pytest.fixture(scope="module")
def sweep(sweep_result):
    """Return every seed the bounds could satisfy, as ``(seed, scenario)`` pairs."""
    return sweep_result[0]


@pytest.fixture(scope="module")
def exhausted_seeds(sweep_result):
    """Return the seeds whose draw the bounds could not finish."""
    return sweep_result[1]


@pytest.fixture(scope="module")
def sweep_result():
    """Generate the whole seed sweep once, keeping the few seeds the bounds cannot satisfy."""
    # Module-scoped: every invariant asks the same question of the same documents.
    # Exhausted seeds are returned, not swallowed: for a handful of seeds the first five products
    # land where no sixth position clears every corridor. The generator fails closed on such a
    # draw, which is intended while it stays rare and confined to the maximum count (both
    # asserted).
    scenarios: list[tuple[int, dict]] = []
    exhausted: list[int] = []
    for seed in INVARIANT_SEEDS:
        try:
            scenarios.append((seed, _generate(seed)))
        except RuntimeError as error:
            if "exhausted" not in str(error):
                raise
            exhausted.append(seed)
    return scenarios, exhausted


def test_the_bounds_stay_satisfiable_for_all_but_a_measured_handful_of_seeds(
    sweep, exhausted_seeds
) -> None:
    """Six products must still place, and a draw that cannot must stop rather than be emitted."""
    randomization = load_randomization(load_scenario(BASELINE_SCENARIO))
    maximum = int(randomization["product_count"]["maximum"])
    # Measured over seeds 0..19999 at this attempt budget: 6 exhausted, all at the maximum count.
    # The bound is loose enough not to flake and tight enough to fail if a rule change made the
    # tray much harder to fill.
    assert len(exhausted_seeds) <= len(INVARIANT_SEEDS) // 200, exhausted_seeds
    # The residual must be the last product of a full draw: a scenario the tray cannot hold at all
    # is a bounds error, not a tail.
    for seed in exhausted_seeds:
        assert _drawn_count(seed) == maximum, seed
    # The sweep must reach the maximum, or the assertion above is vacuous.
    assert any(len(scenario["products"]) == maximum for _, scenario in sweep)


def _drawn_count(seed: int) -> int:
    """Replay only the first draw of a seed, which is the product count."""
    randomization = load_randomization(load_scenario(BASELINE_SCENARIO))
    minimum = int(randomization["product_count"]["minimum"])
    maximum = int(randomization["product_count"]["maximum"])
    return minimum + SplitMix64(seed).next_below(maximum - minimum + 1)


def test_splitmix64_reproduces_the_published_reference_stream() -> None:
    """Pin the generator to the published algorithm, not to whatever this toolchain does."""
    # Reference output of SplitMix64 seeded with 0. If this changes, every recorded seed becomes
    # meaningless.
    stream = SplitMix64(0)
    assert [stream.next_u64() for _ in range(4)] == [
        0xE220A8397B1DCDAF,
        0x6E789E6AA1B965F4,
        0x06C45D188009454F,
        0xF88BB8A8724C81EC,
    ]
    assert SplitMix64(0xDEADBEEF).next_u64() == SplitMix64(0xDEADBEEF).next_u64()


def test_splitmix64_unit_draws_stay_in_range_and_integer_draws_are_unbiased() -> None:
    """The two derived draw shapes must respect their bounds without leaning on any bucket."""
    stream = SplitMix64(7)
    assert all(0.0 <= stream.next_unit() < 1.0 for _ in range(2000))

    buckets = [0] * 5
    counting = SplitMix64(11)
    for _ in range(20000):
        buckets[counting.next_below(5)] += 1
    assert min(buckets) > 3500
    assert sum(buckets) == 20000

    interval = SplitMix64(13)
    assert all(-0.5 <= interval.next_between(-0.5, 0.25) <= 0.25 for _ in range(2000))
    with pytest.raises(RuntimeError, match="positive draw bound"):
        SplitMix64(0).next_below(0)
    with pytest.raises(RuntimeError, match="non-empty draw interval"):
        SplitMix64(0).next_between(1.0, 0.0)


def test_seed_parsing_is_opt_in_and_fails_closed() -> None:
    """A seed the operator cannot quote back is worse than no randomization at all."""
    assert parse_seed(FIXED_SCENARIO_SEED) is None
    assert parse_seed("  fixed  ") is None
    assert parse_seed("0") == 0
    assert parse_seed("18446744073709551615") == (1 << 64) - 1
    for rejected in ("", "-1", "1.5", "0x10", "random", "1e3"):
        with pytest.raises(RuntimeError, match="scenario_seed"):
            parse_seed(rejected)
    with pytest.raises(RuntimeError, match=r"below 2\^64"):
        parse_seed("18446744073709551616")


def test_default_seed_preserves_the_surveyed_scenario_exactly(baseline: dict) -> None:
    """The launch default must not move a single product, or every recorded run is invalidated."""
    assert parse_seed(FIXED_SCENARIO_SEED) is None
    raw = yaml.safe_load(BASELINE_SCENARIO.read_text(encoding="utf-8"))
    assert [product["model_name"] for product in raw["products"]] == [
        "stock_can_01",
        "stock_small_bottle_01",
        "stock_large_bottle_01",
    ]
    assert [product["spawn_pose"] for product in raw["products"]] == [
        [-0.42, -0.80, 0.631, 0.0, 0.0, 0.0],
        [0.0, -0.80, 0.670, 0.0, 0.0, 0.0],
        [0.42, -0.80, 0.715, 0.0, 0.0, 0.0],
    ]
    # Adding the randomization bounds must not disturb anything the fixed path reads.
    assert baseline["products"] == raw["products"]
    assert baseline["workcell_pose"] == [0.0, 0.55, 0.75, 0.0, 0.0, 0.0]


def test_generated_scenario_is_byte_identical_for_the_same_seed() -> None:
    """Determinism is asserted on the serialized bytes, not on an approximate comparison."""
    first = serialize_scenario(_generate(20260908))
    second = serialize_scenario(_generate(20260908))
    assert first == second
    assert serialize_scenario(_generate(20260909)) != first


def test_generated_scenario_survives_a_process_restart(tmp_path: Path) -> None:
    """A seed recorded in a bug report has to reproduce in a brand-new interpreter."""
    program = (
        "import sys\n"
        "from restocker_gazebo.scenario_random import ("
        "generate_scenario_from_files, serialize_scenario)\n"
        "sys.stdout.write(serialize_scenario(generate_scenario_from_files("
        "sys.argv[1], sys.argv[2], sys.argv[3], sys.argv[4], int(sys.argv[5]))))\n"
    )
    environment = dict(os.environ)
    # A hash seed is the classic way a "deterministic" generator depends on the process; the child
    # gets a different one than this test.
    environment["PYTHONHASHSEED"] = "12345"
    outputs = []
    for hash_seed in ("0", "98765"):
        environment["PYTHONHASHSEED"] = hash_seed
        outputs.append(
            subprocess.run(
                [
                    sys.executable,
                    "-c",
                    program,
                    str(BASELINE_SCENARIO),
                    str(PRODUCT_CATALOG),
                    str(WORKCELL_GEOMETRY),
                    str(GRIPPER_GEOMETRY),
                    "4242",
                ],
                check=True,
                capture_output=True,
                text=True,
                env=environment,
                cwd=tmp_path,
            ).stdout
        )
    assert outputs[0] == outputs[1]
    assert outputs[0] == serialize_scenario(_generate(4242))


def test_written_scenario_round_trips_through_the_validated_loader(tmp_path: Path) -> None:
    """What is written must be loadable by the same validation the hand-written baseline passes."""
    scenario = _generate(99)
    path = write_generated_scenario(scenario, tmp_path)
    assert path.name == "scenario_seed_99.yaml"
    assert load_scenario(path)["products"] == scenario["products"]
    # Rewriting the same seed is idempotent.
    before = path.read_bytes()
    assert write_generated_scenario(_generate(99), tmp_path).read_bytes() == before


def test_generated_scenarios_are_physically_seatable(catalog, region, corridor, sweep) -> None:
    """Containment, resting contact, and separation hold for every seed, not on average."""
    randomization = load_randomization(load_scenario(BASELINE_SCENARIO))
    separation = float(randomization["stock_region"]["surface_separation_m"])
    margin = float(randomization["stock_region"]["wall_margin_m"])
    minimum = randomization["product_count"]["minimum"]
    maximum = randomization["product_count"]["maximum"]

    observed_counts: set[int] = set()
    observed_classes: set[str] = set()
    for _seed, scenario in sweep:
        products = scenario["products"]
        assert minimum <= len(products) <= maximum
        observed_counts.add(len(products))
        assert placement_violations(scenario, catalog, region, separation, corridor) == []

        names = [product["model_name"] for product in products]
        assert len(set(names)) == len(names)
        assert len({product["source_object_id"] for product in products}) == len(names)
        for product in products:
            observed_classes.add(product["product_class"])
            # The wall margin is a stated bound, asserted separately from the containment
            # invariant.
            shape = resolve_product_geometry(product, catalog)["shape"]
            radius = float(shape["radius_m"])
            for axis, half in ((0, region.size_xyz_m[0] / 2.0), (1, region.size_xyz_m[1] / 2.0)):
                center = scenario["workcell_pose"][axis] + region.center_xyz_m[axis]
                offset = abs(float(product["spawn_pose"][axis]) - center)
                assert offset + radius + margin <= half + POSE_QUANTUM_M
            assert product["spawn_pose"][3:] == [0.0, 0.0, 0.0]

    assert observed_counts == set(range(minimum, maximum + 1))
    assert observed_classes == {"can", "small_bottle", "large_bottle"}


def test_planner_padding_matches_the_move_group_launch_it_is_copied_from() -> None:
    """The generator's copy of the planner padding must equal the one move_group launches with."""
    # Nothing at runtime notices a mismatch: restocker_gazebo does not depend on the MoveIt
    # configuration, so the number is carried, not imported. Reading the launch file as text fails
    # when the padding is retuned without the generator; move both and bump
    # SUPPORTED_GENERATOR_VERSION.
    launch_source = (MOVEIT_CONFIG_SOURCE / "launch" / "move_group.launch.py").read_text(
        encoding="utf-8"
    )
    declared = re.findall(
        r'"robot_description_planning\.default_robot_padding"\s*:\s*([0-9.eE+-]+)', launch_source
    )
    assert len(declared) == 1, declared
    assert float(declared[0]) == pytest.approx(PLANNER_ROBOT_PADDING_M, abs=1e-12)


def test_approach_corridor_matches_the_description_it_is_derived_from(corridor) -> None:
    """The corridor is only correct while it equals the widest thing that actually sweeps it."""
    # Re-derived from the raw description fields the URDF is generated from, not imported from the
    # loader under test, so it fails if the loader's derivation drifts.
    gripper = yaml.safe_load(GRIPPER_GEOMETRY.read_text(encoding="utf-8"))
    finger = gripper["finger"]
    # Outer face of the widest-standing jaw at the aperture the coordinator releases at, from the
    # tool axis: mount offset, plus jaw travel, plus half the finger width.
    finger_outer_face_m = (
        max(abs(float(finger[side]["origin_xyz_m"][1])) for side in ("left", "right"))
        + float(gripper["attachment"]["open_target_m"])
        + (float(finger["size_xyz_m"][1]) / 2.0)
    )
    # The arm comes from ur_description with mesh collisions; the package constant is the measured
    # radial bound of those UR10e wrist STLs.
    wrist_radius_m = WRIST_LINK_RADIUS_M
    assert wrist_radius_m == pytest.approx(0.085, abs=1e-12)

    # MoveIt inflates every robot link by default_robot_padding before collision-checking, so the
    # corridor to keep clear is the padded one.
    assert corridor.half_width_m == pytest.approx(
        max(finger_outer_face_m, wrist_radius_m) + PLANNER_ROBOT_PADDING_M, abs=1e-12
    )
    # The correction matters only if the widest link is wider than the fingers.
    assert wrist_radius_m > finger_outer_face_m

    # And the stand-off, which turns a bound on the pre-grasp into a bound on the product.
    assert corridor.pregrasp_standoff_m == pytest.approx(
        float(gripper["grasp_center"]["xyz_m"][2]) + PREGRASP_DISTANCE_M, abs=1e-12
    )


def test_pregrasp_standoff_matches_the_coordinator_it_is_copied_from() -> None:
    """The reachability bound is only correct while the generator's stand-off is the real one."""
    # Same coupling as above: restocker_gazebo does not depend on restocker_task_executor, so the
    # number is carried, not imported. Reading the coordinator's configuration as text fails when
    # the stand-off is retuned without the generator; move both and bump
    # SUPPORTED_GENERATOR_VERSION.
    coordinator = (TASK_EXECUTOR_SOURCE / "config" / "restock_action_coordinator.yaml").read_text(
        encoding="utf-8"
    )
    declared = re.findall(r"^\s*grasp\.pregrasp_distance_m:\s*([0-9.eE+-]+)", coordinator, re.M)
    assert len(declared) == 1, declared
    assert float(declared[0]) == pytest.approx(PREGRASP_DISTANCE_M, abs=1e-12)


def test_every_generated_pair_leaves_the_approach_corridor_clear(catalog, corridor, sweep) -> None:
    """A pair the arm cannot reach between is a seed no recovery can escape, so reject it."""
    # Reaching a product runs tool0 down a straight line into the tray, so the tube swept along it
    # must be clear. A neighbour inside it blocks the approach from every arm configuration, which
    # is unrecoverable, not a planner cost to retry.
    # The bound is on the across-approach distance alone: the tube is longer than the tray is deep,
    # so whichever product stands nearer the robot is inside the other's corridor at every
    # along-approach offset, and only the lateral offset can clear it.
    observed_pairs = 0
    worst_margin = math.inf
    for seed, scenario in sweep:
        products = scenario["products"]
        radii = [
            float(resolve_product_geometry(product, catalog)["shape"]["radius_m"])
            for product in products
        ]
        for first in range(len(products)):
            for second in range(first + 1, len(products)):
                lateral = abs(
                    float(products[first]["spawn_pose"][0])
                    - float(products[second]["spawn_pose"][0])
                )
                # Both products are grasp targets (chosen at runtime), so the pair must admit an
                # approach in either direction and the larger neighbour radius governs.
                required = required_lateral_distance(
                    radii[first], radii[second], corridor.half_width_m
                )
                assert lateral + POSE_QUANTUM_M >= required, (
                    seed,
                    products[first]["model_name"],
                    products[second]["model_name"],
                )
                worst_margin = min(worst_margin, lateral - required)
                observed_pairs += 1
    # Coverage is asserted too (a sweep with no pair asserts nothing), and the tightest pair must
    # be tight: a rule nothing comes near is not exercised.
    assert observed_pairs > 10000
    assert worst_margin < 0.001


def test_every_generated_product_stands_where_a_pregrasp_is_reachable(corridor, sweep) -> None:
    """A product the arm cannot stand off from is unrecoverable, so it must never be drawn."""
    # The approach is a straight line and the coordinator does not choose the configuration it runs
    # from, so a placement is safe only when every collision-free pre-grasp can follow the line.
    # That holds while tool0's pre-grasp keeps MINIMUM_PREGRASP_REACH_Y_M from the rail (measured
    # offline).
    limit = reachable_y_maximum(corridor)
    assert limit == pytest.approx(
        -(MINIMUM_PREGRASP_REACH_Y_M + corridor.pregrasp_standoff_m), abs=1e-12
    )
    region_source = load_stock_region(WORKCELL_GEOMETRY)
    baseline_y = 0.55 + region_source.center_xyz_m[1]
    near_face = baseline_y + (region_source.size_xyz_m[1] / 2.0)
    # The bound must remove part of the tray and leave most of it so the draw keeps its range.
    assert near_face > limit > baseline_y - (region_source.size_xyz_m[1] / 2.0)

    observed = 0
    worst = -math.inf
    for seed, scenario in sweep:
        for product in scenario["products"]:
            y_m = float(product["spawn_pose"][1])
            assert y_m <= limit + POSE_QUANTUM_M, (seed, product["model_name"], y_m)
            worst = max(worst, y_m)
            observed += 1
    assert observed > 10000
    # Drawn right up to the bound, so the sweep exercises it.
    assert worst > limit - 0.001


def test_no_class_is_drawn_more_often_than_the_shelf_has_lanes_for_it(sweep) -> None:
    """A scenario asking for more products of a class than lanes exist is unsatisfiable."""
    # A lane holds one product: the baseline predicate refuses an occupied destination. The
    # benchmark issues one goal per stocked product, so a class drawn more often than it has lanes
    # strands the surplus on NO_COMPATIBLE_PAIR, counted as a robot failure. The lane count comes
    # from restocker_world_state's lane policy, not from baseline_products.yaml.
    lanes = yaml.safe_load(
        (WORLD_STATE_SOURCE / "config" / "baseline_lanes.yaml").read_text(encoding="utf-8")
    )["lanes"]
    shelf_capacity: dict[str, int] = {}
    for lane in lanes.values():
        product_class = lane["expected_product_class"]
        shelf_capacity[product_class] = shelf_capacity.get(product_class, 0) + 1
    randomization = load_randomization(load_scenario(BASELINE_SCENARIO))
    assert lane_capacity_by_class(randomization["variants"]) == shelf_capacity
    # Total products may not exceed what the shelf can take, so the per-class quota below is always
    # satisfiable.
    assert randomization["product_count"]["maximum"] <= sum(shelf_capacity.values())

    saturated = 0
    for seed, scenario in sweep:
        drawn: dict[str, int] = {}
        for product in scenario["products"]:
            product_class = product["product_class"]
            drawn[product_class] = drawn.get(product_class, 0) + 1
        for product_class, count in drawn.items():
            assert count <= shelf_capacity[product_class], (seed, product_class, count)
        if sum(drawn.values()) == sum(shelf_capacity.values()):
            saturated += 1
    # The quota binds only if the sweep reaches it: for six lanes, every class drawn twice.
    assert saturated > 0


def test_load_randomization_rejects_bounds_the_shelf_cannot_satisfy() -> None:
    """Bounds that ask for more products than lanes must fail closed, not cap silently."""
    scenario = copy.deepcopy(load_scenario(BASELINE_SCENARIO))
    scenario["randomization"]["product_count"]["maximum"] = 7
    with pytest.raises(RuntimeError, match="exceeds the .* lanes"):
        load_randomization(scenario)


def test_placement_violations_detects_a_pair_the_arm_cannot_reach_between(
    catalog, region, corridor, sweep
) -> None:
    """The corridor bound is only worth trusting if the checker rejects a scenario breaking it."""
    scenario = next(candidate for _seed, candidate in sweep if len(candidate["products"]) > 1)
    blocked = copy.deepcopy(scenario)
    first, second = blocked["products"][0], blocked["products"][1]
    radii = [
        float(resolve_product_geometry(product, catalog)["shape"]["radius_m"])
        for product in (first, second)
    ]
    separation = float(
        load_randomization(load_scenario(BASELINE_SCENARIO))["stock_region"][
            "surface_separation_m"
        ]
    )
    # Slide the second product across the approach until it is a millimetre inside the corridor,
    # and well clear along the approach, so the pair is further apart than the seating gap needs.
    # This isolates the corridor bound from the seating bound.
    lateral = required_lateral_distance(radii[0], radii[1], corridor.half_width_m) - 0.001
    second["spawn_pose"][0] = float(first["spawn_pose"][0]) + lateral
    second["spawn_pose"][1] = float(first["spawn_pose"][1]) - 0.20
    assert math.hypot(lateral, 0.20) - radii[0] - radii[1] > separation
    reported = " ".join(placement_violations(blocked, catalog, region, separation, corridor))
    assert "blocks the approach corridor" in reported
    assert "closer than the required gap" not in reported
    # Clean once the corridor is not checked, so the report above is attributable to this
    # invariant, not a containment break.
    assert placement_violations(blocked, catalog, region, separation) == []


def test_placement_violations_detects_a_product_no_pregrasp_can_reach(
    catalog, region, corridor
) -> None:
    """The reachability bound is only worth trusting if the checker rejects a breaking scenario."""
    scenario = copy.deepcopy(_generate(3))
    product = scenario["products"][0]
    # A millimetre the wrong side of the bound and inside the surveyed tray, so the report cannot
    # be a containment break.
    product["spawn_pose"][1] = reachable_y_maximum(corridor) + 0.001
    reported = " ".join(placement_violations(scenario, catalog, region, 0.0, corridor))
    assert "no pre-grasp is reachable" in reported
    assert "leaves the stock region" not in reported
    # Without a corridor the checker sees only the physical invariants, which this scenario does
    # not break, so the report above is attributable to the reachability bound.
    assert placement_violations(scenario, catalog, region) == []


def test_generated_products_rest_exactly_on_the_tray_surface(catalog, region) -> None:
    """A product spawned off the surface falls to a pose the seed never chose."""
    tray_top_z = 0.75 + region.center_xyz_m[2] - (region.size_xyz_m[2] / 2.0)
    assert tray_top_z == pytest.approx(0.57)
    for seed in (1, 2, 3, 5, 8, 13, 21):
        for product in _generate(seed)["products"]:
            height = float(resolve_product_geometry(product, catalog)["shape"]["height_m"])
            assert float(product["spawn_pose"][2]) == pytest.approx(
                tray_top_z + (height / 2.0), abs=POSE_QUANTUM_M
            )


def test_variant_destination_lanes_are_exactly_the_lanes_that_accept_the_variant() -> None:
    """The lane list is a constraint on the bounds, checked against the policy that owns it."""
    # A generated scenario says which products exist and where they stand, not where they end up
    # (the coordinator chooses a destination at runtime). ``destination_lanes`` refuses bounds that
    # declare a product class the shelf has no lane for. Measured against restocker_world_state's
    # lane policy, which decides which lane accepts what, not the scenario's copy.
    policy = yaml.safe_load(
        (WORLD_STATE_SOURCE / "config" / "baseline_lanes.yaml").read_text(encoding="utf-8")
    )["lanes"]
    for variant in load_randomization(load_scenario(BASELINE_SCENARIO))["variants"]:
        accepting = {
            lane_id
            for lane_id, lane in policy.items()
            if lane["expected_product_class"] == variant["product_class"]
            and lane["expected_sku"] == variant["sku"]
        }
        assert accepting, variant["geometry_key"]
        assert set(variant["destination_lanes"]) == accepting, variant["geometry_key"]


def test_generated_products_carry_no_destination(sweep) -> None:
    """Generator version 2 emits no per-product destination, because nothing ever read one."""
    # Re-adding the field would add a draw to the stream and change what every seed means, so this
    # pins the emitted schema.
    for _seed, scenario in sweep:
        for product in scenario["products"]:
            assert "destination_lane" not in product


def test_generation_provenance_records_the_seed_and_the_bounds() -> None:
    """The seed only replays against the same bounds, so the bounds are fingerprinted with it."""
    scenario = _generate(1234)
    provenance = scenario["generated_from"]
    assert provenance["seed"] == 1234
    assert provenance["algorithm"] == "splitmix64"
    assert provenance["generator"] == "restocker_gazebo.scenario_random"
    assert len(provenance["bounds_digest_sha256"]) == 64
    # No absolute path may leak into the document: it would differ between machines and break
    # byte-identity of the replay.
    assert str(GAZEBO_SOURCE) not in serialize_scenario(scenario)

    moved = copy.deepcopy(load_scenario(BASELINE_SCENARIO))
    moved["randomization"]["stock_region"]["wall_margin_m"] = 0.02
    changed = generate_scenario(
        moved,
        load_product_catalog(PRODUCT_CATALOG),
        load_stock_region(WORKCELL_GEOMETRY),
        load_approach_corridor(GRIPPER_GEOMETRY),
        1234,
    )
    assert changed["generated_from"]["bounds_digest_sha256"] != provenance["bounds_digest_sha256"]


def test_placement_violations_detects_a_scenario_that_cannot_be_seated(catalog, region) -> None:
    """The invariant checker is only worth trusting if it actually rejects a broken scenario."""
    scenario = _generate(31337)
    assert placement_violations(scenario, catalog, region) == []

    floating = copy.deepcopy(scenario)
    floating["products"][0]["spawn_pose"][2] += 0.05
    assert "floats above the tray surface" in " ".join(
        placement_violations(floating, catalog, region)
    )

    sunk = copy.deepcopy(scenario)
    sunk["products"][0]["spawn_pose"][2] -= 0.05
    assert "sunk below the tray surface" in " ".join(placement_violations(sunk, catalog, region))

    outside = copy.deepcopy(scenario)
    outside["products"][0]["spawn_pose"][0] = 5.0
    assert "leaves the stock region on x" in " ".join(
        placement_violations(outside, catalog, region)
    )

    outside_y = copy.deepcopy(scenario)
    outside_y["products"][0]["spawn_pose"][1] = -5.0
    assert "leaves the stock region on y" in " ".join(
        placement_violations(outside_y, catalog, region)
    )


def test_placement_violations_detects_overlap(catalog, region, sweep) -> None:
    """Two products drawn onto the same spot must be reported, not tolerated."""
    scenario = next(candidate for _seed, candidate in sweep if len(candidate["products"]) > 1)
    stacked = copy.deepcopy(scenario)
    stacked["products"][1]["spawn_pose"][0] = stacked["products"][0]["spawn_pose"][0]
    stacked["products"][1]["spawn_pose"][1] = stacked["products"][0]["spawn_pose"][1]
    assert "closer than the required gap" in " ".join(
        placement_violations(stacked, catalog, region)
    )


def test_generation_fails_closed_when_the_bounds_cannot_be_satisfied(
    catalog, region, corridor
) -> None:
    """An unsatisfiable draw must stop the launch rather than emit an unseatable scenario."""
    crowded = copy.deepcopy(load_scenario(BASELINE_SCENARIO))
    crowded["randomization"]["stock_region"]["surface_separation_m"] = 2.0
    crowded["randomization"]["product_count"] = {"minimum": 4, "maximum": 4}
    with pytest.raises(RuntimeError, match="exhausted"):
        generate_scenario(crowded, catalog, region, corridor, 5)

    wide = copy.deepcopy(load_scenario(BASELINE_SCENARIO))
    wide["randomization"]["stock_region"]["wall_margin_m"] = 1.0
    with pytest.raises(RuntimeError, match="does not clear"):
        generate_scenario(wide, catalog, region, corridor, 5)

    rotated = copy.deepcopy(load_scenario(BASELINE_SCENARIO))
    rotated["workcell_pose"] = [0.0, 0.55, 0.75, 0.0, 0.0, math.pi / 4.0]
    with pytest.raises(RuntimeError, match="axis-aligned"):
        generate_scenario(rotated, catalog, region, corridor, 5)


def test_generation_rejects_a_product_too_tall_for_the_region(catalog, region, corridor) -> None:
    """A product that cannot stand up in the tray is a spawn that will topple, so reject it."""
    shallow = region.__class__(
        center_xyz_m=region.center_xyz_m,
        size_xyz_m=(region.size_xyz_m[0], region.size_xyz_m[1], 0.05),
        floor_settle_tolerance_m=region.floor_settle_tolerance_m,
    )
    with pytest.raises(RuntimeError, match="exceeds the usable volume"):
        generate_scenario(load_scenario(BASELINE_SCENARIO), catalog, shallow, corridor, 5)


def _launch_module():
    """Import the launch file by path, the way the launch system loads it."""
    spec = importlib.util.spec_from_file_location(
        "restocker_simulation_launch", GAZEBO_SOURCE / "launch" / "simulation.launch.py"
    )
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def test_launch_declares_an_opt_in_seed_argument() -> None:
    """The declared default has to be the fixed scenario, or randomization is not opt-in."""
    description = _launch_module().generate_launch_description()
    declarations = {
        action.name: action
        for action in description.entities
        if isinstance(action, DeclareLaunchArgument)
    }
    assert "scenario_seed" in declarations
    assert declarations["scenario_seed"].default_value[0].perform(LaunchContext()) == (
        FIXED_SCENARIO_SEED
    )


def test_seeded_scenario_is_resolved_before_anything_reads_the_scenario_path() -> None:
    """The attachment plugin reads the path from the environment inside the simulator process."""
    # If the seeded document were substituted after that variable or the Gazebo include, the plugin
    # would validate the baseline while the launch spawned the generated products.
    module = _launch_module()
    entities = module.generate_launch_description().entities
    resolve_index = next(
        index
        for index, action in enumerate(entities)
        # OpaqueFunction keeps its callable in a name-mangled attribute; matching on it is the only
        # way to assert the ordering of this action.
        if getattr(action, "_OpaqueFunction__function", None) is module._resolve_scenario
    )
    environment_indices = [
        index
        for index, action in enumerate(entities)
        if isinstance(action, SetEnvironmentVariable)
    ]
    include_indices = [
        index
        for index, action in enumerate(entities)
        if isinstance(action, IncludeLaunchDescription)
    ]
    assert environment_indices and include_indices
    assert resolve_index < min(environment_indices)
    assert resolve_index < min(include_indices)


def _seed_context(seed: str, output_directory: Path) -> LaunchContext:
    context = LaunchContext()
    context.launch_configurations.update(
        {
            "scenario_seed": seed,
            "scenario_config": str(BASELINE_SCENARIO),
            "product_catalog": str(PRODUCT_CATALOG),
            "workcell_geometry": str(WORKCELL_GEOMETRY),
            "gripper_geometry": str(GRIPPER_GEOMETRY),
        }
    )
    os.environ["RESTOCKER_GENERATED_SCENARIO_DIR"] = str(output_directory)
    return context


def test_fixed_seed_leaves_the_launch_scenario_path_untouched(tmp_path: Path) -> None:
    """The default run must not write, generate, or repoint anything."""
    module = _launch_module()
    context = _seed_context(FIXED_SCENARIO_SEED, tmp_path)
    try:
        assert module._resolve_scenario(context) == []
    finally:
        os.environ.pop("RESTOCKER_GENERATED_SCENARIO_DIR", None)
    assert context.launch_configurations["scenario_config"] == str(BASELINE_SCENARIO)
    assert list(tmp_path.iterdir()) == []


def test_numeric_seed_repoints_the_launch_at_the_generated_scenario(tmp_path: Path) -> None:
    """Every downstream consumer of `scenario_config` has to see the generated document."""
    module = _launch_module()
    context = _seed_context("777", tmp_path)
    try:
        actions = module._resolve_scenario(context)
        announced = [
            perform_substitutions(context, action.msg)
            for action in actions
            if isinstance(action, LogInfo)
        ]
        for action in actions:
            action.execute(context)
    finally:
        os.environ.pop("RESTOCKER_GENERATED_SCENARIO_DIR", None)

    generated = tmp_path / "scenario_seed_777.yaml"
    assert generated.is_file()
    assert context.launch_configurations["scenario_config"] == str(generated)
    assert generated.read_text(encoding="utf-8") == serialize_scenario(_generate(777))
    # The launch must print the seed: it is the replay instruction.
    assert any("scenario_seed:=777" in message for message in announced)

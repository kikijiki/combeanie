#!/usr/bin/env python3
# Copyright 2026 Matteo Bernacchia <dev@kikijiki.com>
#
# SPDX-License-Identifier: MIT

"""Record the perception benchmark dataset: one simulator launch per frozen seed, then publish.

Usage::

    tools/record_perception_dataset.py --out /path/to/datasets --seed 3 --seed 7
    tools/record_perception_dataset.py --out DIR --split held_out        # every held-out seed

Each seed is materialised offline (scenario YAML and truth from restocker_benchmarks), launched
once through restocker_bringup's test_perception_recorder_runtime under the machine-wide
simulator slot and a leased ROS domain, and its frames are collected. The parts are then merged
into one content-hashed directory ``pbench-<hash>`` with manifest.json (seeds, git SHA, world
hash, scenario hashes). Nothing here starts more than one simulator at a time.
"""

from __future__ import annotations

import argparse
import json
import math
import os
from pathlib import Path
import subprocess  # noqa: S404 - fixed argv, no shell
import sys
import tempfile

import yaml

REPOSITORY_ROOT = Path(__file__).resolve().parent.parent
for package in ("restocker_benchmarks", "restocker_gazebo", "restocker_perception"):
    sys.path.insert(0, str(REPOSITORY_ROOT / "ros_ws" / "src" / package))

from restocker_benchmarks import perception_barcode_views as barcode_module  # noqa: E402
from restocker_benchmarks import perception_cli as cli  # noqa: E402
from restocker_benchmarks import perception_dataset as dataset  # noqa: E402
from restocker_benchmarks import perception_lane_views as lane_views_module  # noqa: E402
from restocker_benchmarks import perception_lighting as lighting_module  # noqa: E402
from restocker_benchmarks import perception_splits as splits_module  # noqa: E402
from restocker_benchmarks import perception_views as views_module  # noqa: E402

GAZEBO = REPOSITORY_ROOT / "ros_ws" / "src" / "restocker_gazebo"
TEST_FILE = (
    REPOSITORY_ROOT
    / "ros_ws"
    / "src"
    / "restocker_bringup"
    / "test"
    / "test_perception_recorder_runtime.py"
)
DEFAULT_SPLITS = (
    REPOSITORY_ROOT
    / "ros_ws"
    / "src"
    / "restocker_benchmarks"
    / "config"
    / ("perception_splits.json")
)
DEFAULT_CATALOGUE = GAZEBO / "config" / "generated_product_labels.yaml"


def prepare_conditions(
    seed: int,
    scene: dict,
    work: Path,
    *,
    lighting: bool,
    views: int,
    barcode_reach_deg: float | None = None,
    lane_views: bool = False,
    relook: tuple[int, int, list[str]] | None = None,
    lighting_condition: str | None = None,
    view_stress: views_module.ViewStress | None = None,
    lane_stress: lane_views_module.LaneStationStress | None = None,
) -> dict:
    """Write the seeded world copy and viewpoint list for one scene; return what to record.

    The lighting condition and the viewpoints are pure functions of the seed, drawn from their
    own streams, so neither can change which products a scene holds.
    """
    scenario_id = scene["scenario_id"]
    prepared: dict = {"world": None, "lighting": None, "views": []}
    if lighting:
        spec = (
            lighting_module.lighting_named(lighting_condition, seed)
            if lighting_condition
            else lighting_module.lighting_for_seed(seed)
        )
        shipped = GAZEBO / "worlds" / "restocking.sdf"
        world_path = work / f"{scenario_id}.world.sdf"
        world_path.write_text(
            lighting_module.apply_lighting(shipped.read_text(encoding="utf-8"), spec),
            encoding="utf-8",
        )
        record = dataset.world_lighting(world_path, spec["condition"], spec["intensity_scale"])
        record["world_sha256"] = dataset.sha256_file(world_path)
        prepared.update(world=world_path, lighting=record)
    if relook is not None:
        # A re-look (rotation oracle): only the turned objects are viewed, at poses drawn for this
        # look number, so no earlier camera pose repeats.
        look, per_object, object_ids = relook
        prepared["views"] = views_module.relook_views(
            seed, scene["truth"], object_ids, look=look, per_object=per_object, stress=view_stress
        )
        return prepared
    if lane_views:
        prepared["views"] = lane_views_module.lane_views(scene["truth"], lane_stress)
    if views:
        prepared["views"] = prepared["views"] + views_module.close_views(
            seed, scene["truth"], views, stress=view_stress
        )
    if barcode_reach_deg is not None:
        prepared["views"] = prepared["views"] + barcode_module.barcode_views(
            scene["truth"], max_reach_deg=barcode_reach_deg
        )
    return prepared


def apply_yaw_offsets(scene: dict, offsets: dict[str, float]) -> dict:
    """Turn tray products in place (the rotation oracle of CMB-112 round 3).

    ``offsets`` maps an object id to a yaw change in degrees about the world vertical, added to
    the scenario's own spawn yaw. Truth is derived again from the turned scenario, so the labels
    carry the new pose; positions (hence the seeded close views) do not change. A lane product
    or an unknown object id is refused: the oracle stands in for a regrasp of a tray product.
    """
    if not offsets:
        return scene
    by_object = {p["source_object_id"]: p for p in scene["scenario"]["products"]}
    for object_id, degrees in offsets.items():
        product = by_object.get(object_id)
        if product is None:
            raise SystemExit(f"yaw offset names an object the scene does not hold: {object_id}")
        pose = [float(v) for v in product["spawn_pose"]]
        if dataset.lane_of_pose(pose[:3])[0] is not None:
            raise SystemExit(f"yaw offset names a lane product, not a tray product: {object_id}")
        pose[5] = round((pose[5] + math.radians(degrees)) % (2.0 * math.pi), 6)
        product["spawn_pose"] = pose
    scene["truth"] = dataset.scene_truth(scene["scenario"], scene["lane_expected_sku"])
    return scene


def record_seed(
    seed: int,
    scene: dict,
    work: Path,
    frames: int,
    timeout_s: float,
    prepared: dict | None = None,
    view_check: str = "assert",
) -> Path:
    """Launch one seeded scene under the simulator slot; return its part directory."""
    prepared = prepared or {"world": None, "lighting": None, "views": []}
    scenario_id = scene["scenario_id"]
    scenario_path = work / f"{scenario_id}.yaml"
    truth_path = work / f"{scenario_id}.truth.json"
    part = work / f"part-{scenario_id}"
    scenario_path.write_text(yaml.safe_dump(scene["scenario"], sort_keys=True), encoding="utf-8")
    truth_path.write_text(json.dumps(scene["truth"], sort_keys=True), encoding="utf-8")
    environment = dict(os.environ)
    environment.update(
        {
            "RESTOCKER_BENCH_SCENARIO": str(scenario_path),
            "RESTOCKER_BENCH_TRUTH": str(truth_path),
            "RESTOCKER_BENCH_SCENARIO_ID": scenario_id,
            "RESTOCKER_BENCH_SEED": str(seed),
            "RESTOCKER_BENCH_SPLIT": scene["seed_split"] or "",
            "RESTOCKER_BENCH_PART_DIR": str(part),
            "RESTOCKER_BENCH_FRAMES": str(frames),
            "RESTOCKER_BENCH_VIEW_CHECK": view_check,
        }
    )
    if prepared["world"]:
        environment["RESTOCKER_BENCH_WORLD"] = str(prepared["world"])
        lighting_path = work / f"{scenario_id}.lighting.json"
        lighting_path.write_text(
            json.dumps(prepared["lighting"], sort_keys=True), encoding="utf-8"
        )
        environment["RESTOCKER_BENCH_LIGHTING_JSON"] = str(lighting_path)
    if prepared["views"]:
        views_path = work / f"{scenario_id}.views.json"
        views_path.write_text(json.dumps(prepared["views"], sort_keys=True), encoding="utf-8")
        environment["RESTOCKER_BENCH_VIEWS"] = str(views_path)
    command = [
        str(REPOSITORY_ROOT / "scripts" / "with_simulator_slot.bash"),
        "python3",
        str(REPOSITORY_ROOT / "scripts" / "domain_isolation.py"),
        "--seed",
        f"perception-bench-{scenario_id}",
        "--",
        str(REPOSITORY_ROOT / "scripts" / "with_workspace.bash"),
        "launch_test",
        str(TEST_FILE),
    ]
    print(f"--- recording {scenario_id}", flush=True)
    completed = subprocess.run(  # noqa: S603 - fixed argv, no shell
        command, env=environment, check=False, timeout=timeout_s
    )
    if completed.returncode != 0 or not (part / "rows.jsonl").is_file():
        raise SystemExit(f"recording {scenario_id} failed (exit {completed.returncode})")
    return part


def main() -> int:
    """Record the requested seeds and publish one dataset directory."""
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--out", type=Path, required=True)
    parser.add_argument("--seed", type=int, action="append", default=[])
    parser.add_argument("--split", choices=splits_module.SPLIT_NAMES)
    parser.add_argument("--frames", type=int, default=2, help="frames per camera per seed")
    parser.add_argument("--splits", type=Path, default=DEFAULT_SPLITS)
    parser.add_argument("--catalogue", type=Path, default=DEFAULT_CATALOGUE)
    parser.add_argument(
        "--close-views",
        type=int,
        default=views_module.VIEWS_PER_SCENE,
        help="seeded close-range wrist viewpoints per scene (0 records only the home pose)",
    )
    parser.add_argument(
        "--barcode-reach-deg",
        type=float,
        default=None,
        help="also request aimed barcode views (perception_barcode_views) for tray products "
        "whose barcode azimuth is within this many degrees of the robot-side approach",
    )
    parser.add_argument(
        "--lane-views",
        action="store_true",
        help="also drive the arm to each lane's survey station and record a wrist frame there "
        "(perception_lane_views; the input of the lane stock estimator, CMB-114)",
    )
    parser.add_argument(
        "--no-lighting",
        action="store_true",
        help="record under the shipped world's nominal lighting instead of the seeded condition",
    )
    parser.add_argument(
        "--lighting-condition",
        choices=(*lighting_module.CONDITION_NAMES, *lighting_module.STRESS_NAMES),
        help="CMB-116: light every scene with this named condition instead of the seed's draw "
        "(use the same value for every look of a run)",
    )
    parser.add_argument(
        "--view-stress",
        help="CMB-116: JSON of perception_views.ViewStress fields, e.g. "
        '\'{"max_azimuth_deg": 60}\'; applies to close and re-look views alike',
    )
    parser.add_argument(
        "--lane-stress",
        help="CMB-116: JSON of perception_lane_views.LaneStationStress fields (dx_m, dy_m, dz_m) "
        "moving every lane survey station off its nominal pose",
    )
    parser.add_argument(
        "--turn-error-deg",
        type=float,
        default=0.0,
        help="CMB-116: with --relook, add a seeded Gaussian error of this sigma (degrees) to each "
        "turn the rotation oracle makes (a regrasp that does not turn exactly as asked)",
    )
    parser.add_argument(
        "--yaw-offsets",
        type=Path,
        help="JSON {seed: {object_id: degrees}}: turn those tray products in place before "
        "recording (rotation oracle); other scene content and the close views are unchanged",
    )
    parser.add_argument(
        "--relook",
        type=int,
        default=0,
        help="with --yaw-offsets: this is look number N (1, 2, ...); view only the turned "
        "objects, --relook-views poses each, and no barcode views",
    )
    parser.add_argument("--relook-views", type=int, default=2)
    parser.add_argument("--timeout", type=float, default=1800.0)
    arguments = parser.parse_args()

    document = splits_module.load_splits(arguments.splits)
    products = dataset.load_catalogue_for_splits(arguments.catalogue, document)
    if document["catalogue_sha256"] != dataset.catalogue_sha256(products):
        raise SystemExit("splits were frozen against a different catalogue")
    seeds = list(arguments.seed)
    if arguments.split:
        seeds += document["seeds"][arguments.split]
    if not seeds:
        raise SystemExit("give --seed or --split")

    yaw_offsets = (
        json.loads(arguments.yaw_offsets.read_text(encoding="utf-8"))
        if arguments.yaw_offsets
        else {}
    )
    view_stress = (
        views_module.ViewStress(**json.loads(arguments.view_stress))
        if arguments.view_stress
        else None
    )
    lane_stress = (
        lane_views_module.LaneStationStress(**json.loads(arguments.lane_stress))
        if arguments.lane_stress
        else None
    )
    work = Path(tempfile.mkdtemp(prefix="perception-record-"))
    builder = dataset.DatasetBuilder(work / "stage")
    for seed in sorted(set(seeds)):
        scene = cli.scene_for_seed(seed, products, document)
        turned = yaw_offsets.get(str(seed), {})
        if arguments.turn_error_deg and arguments.relook:
            turned = {
                object_id: degrees
                + views_module.turn_error_deg(
                    seed, object_id, arguments.relook, arguments.turn_error_deg
                )
                for object_id, degrees in turned.items()
            }
        scene = apply_yaw_offsets(scene, turned)
        if arguments.relook and not turned:
            continue  # nothing to look at again in this scene
        prepared = prepare_conditions(
            seed,
            scene,
            work,
            lighting=not arguments.no_lighting,
            views=arguments.close_views,
            barcode_reach_deg=arguments.barcode_reach_deg,
            lane_views=arguments.lane_views,
            relook=(arguments.relook, arguments.relook_views, sorted(turned))
            if arguments.relook
            else None,
            lighting_condition=arguments.lighting_condition,
            view_stress=view_stress,
            lane_stress=lane_stress,
        )
        part = record_seed(
            seed,
            scene,
            work,
            arguments.frames,
            arguments.timeout,
            prepared,
            view_check="report"
            if (view_stress and not view_stress.is_default())
            or (lane_stress and not lane_stress.is_default())
            else "assert",
        )
        part_info = json.loads((part / "part.json").read_text(encoding="utf-8"))
        builder.add_scenario(
            scene["scenario_id"],
            scene["scenario"],
            {
                "seed": seed,
                "seed_split": scene["seed_split"],
                "lighting": prepared["lighting"],
                "close_views": part_info.get("views", []),
                "view_stress": view_stress.to_dict() if view_stress else None,
                "lane_stress": lane_stress.to_dict() if lane_stress else None,
            },
        )
        for row in dataset.read_labels(part / "rows.jsonl"):
            for relative in row["files"].values():
                builder.add_image(Path(relative).name, (part / relative).read_bytes())
            builder.add_label(row)
    published = builder.publish(
        arguments.out,
        seeds=seeds,
        world_sdf=GAZEBO / "worlds" / "restocking.sdf",
        repository=REPOSITORY_ROOT,
        catalogue_hash=dataset.catalogue_sha256(products),
        splits_hash=document["sha256"],
    )
    print(f"dataset: {published}")
    return 0


if __name__ == "__main__":
    sys.exit(main())

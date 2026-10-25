#!/usr/bin/env python3
# Copyright 2026 Matteo Bernacchia <dev@kikijiki.com>
#
# SPDX-License-Identifier: MIT

"""Record the proposal benchmark (CMB-113): cluttered trays and survey-station wrist views.

Usage::

    tools/record_proposal_dataset.py --out DIR --split development
    tools/record_proposal_dataset.py --out DIR --seed 1003 --seed 1004

One simulator launch per seed, under the machine-wide slot, through the same recorder test as
``record_perception_dataset.py``. The scene is ``perception_proposals.build_proposal_scene``; the
wrist goes to the two tray stations, two lane stations and one close view, besides the home pose.
The published directory is an ordinary ``pbench-<hash>`` dataset (``verify`` works on it).
"""

from __future__ import annotations

import argparse
import json
from pathlib import Path
import sys
import tempfile

REPOSITORY_ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(REPOSITORY_ROOT / "tools"))

import record_perception_dataset as recorder  # noqa: E402
from restocker_benchmarks import perception_dataset as dataset  # noqa: E402
from restocker_benchmarks import perception_lighting as lighting_module  # noqa: E402
from restocker_benchmarks import perception_proposals as proposals  # noqa: E402
from restocker_benchmarks import perception_views as views_module  # noqa: E402


def prepare(seed: int, scene: dict, work: Path, lanes: int) -> dict:
    """Lighting world copy and the survey views of one proposal scene (pure in the seed)."""
    scenario_id = scene["scenario_id"]
    spec = lighting_module.lighting_for_seed(seed)
    shipped = recorder.GAZEBO / "worlds" / "restocking.sdf"
    world_path = work / f"{scenario_id}.world.sdf"
    world_path.write_text(
        lighting_module.apply_lighting(shipped.read_text(encoding="utf-8"), spec), encoding="utf-8"
    )
    record = dataset.world_lighting(world_path, spec["condition"], spec["intensity_scale"])
    record["world_sha256"] = dataset.sha256_file(world_path)
    views = proposals.survey_views(seed, scene["truth"], lanes)
    views += views_module.close_views(seed, scene["truth"], 1)
    return {"world": world_path, "lighting": record, "views": views}


def main() -> int:
    """Record the requested proposal seeds and publish one dataset directory."""
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--out", type=Path, required=True)
    parser.add_argument("--seed", type=int, action="append", default=[])
    parser.add_argument("--split", choices=("development", "held_out"))
    parser.add_argument("--lanes", type=int, default=2, help="lane survey views per scene")
    parser.add_argument("--catalogue", type=Path, default=recorder.DEFAULT_CATALOGUE)
    parser.add_argument("--timeout", type=float, default=2400.0)
    arguments = parser.parse_args()

    import yaml  # noqa: PLC0415

    products = yaml.safe_load(arguments.catalogue.read_text(encoding="utf-8"))["products"]
    seeds = list(arguments.seed)
    if arguments.split == "development":
        seeds += proposals.DEVELOPMENT_SEEDS
    elif arguments.split == "held_out":
        seeds += proposals.HELD_OUT_SEEDS
    if not seeds:
        raise SystemExit("give --seed or --split")

    work = Path(tempfile.mkdtemp(prefix="proposal-record-"))
    builder = dataset.DatasetBuilder(work / "stage")
    for seed in sorted(set(seeds)):
        scene = proposals.build_proposal_scene(seed, products)
        prepared = prepare(seed, scene, work, arguments.lanes)
        sim_scene = {**scene, "scenario": proposals.simulator_scenario(scene["scenario"])}
        part = recorder.record_seed(seed, sim_scene, work, 1, arguments.timeout, prepared)
        part_info = json.loads((part / "part.json").read_text(encoding="utf-8"))
        builder.add_scenario(
            scene["scenario_id"],
            scene["scenario"],
            {
                "seed": seed,
                "seed_split": scene["seed_split"],
                "tray_kind": scene["tray_kind"],
                "lighting": prepared["lighting"],
                "views": part_info.get("views", []),
            },
        )
        for row in dataset.read_labels(part / "rows.jsonl"):
            row["tray_kind"] = scene["tray_kind"]
            for relative in row["files"].values():
                builder.add_image(Path(relative).name, (part / relative).read_bytes())
            builder.add_label(row)
    published = builder.publish(
        arguments.out,
        seeds=seeds,
        world_sdf=recorder.GAZEBO / "worlds" / "restocking.sdf",
        repository=REPOSITORY_ROOT,
        catalogue_hash=dataset.sha256_file(arguments.catalogue),
        splits_hash=None,
        extra={
            "kind_detail": "proposal_benchmark",
            "proposal_scenes_version": proposals.PROPOSAL_SCENES_VERSION,
        },
    )
    print(f"dataset: {published}")
    return 0


if __name__ == "__main__":
    sys.exit(main())

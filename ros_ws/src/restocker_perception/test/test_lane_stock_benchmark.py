# Copyright 2026 Matteo Bernacchia <dev@kikijiki.com>
#
# SPDX-License-Identifier: MIT

"""The benchmark adapter: recorded-frame layout in, prediction rows out, three identity arms."""

import json
from pathlib import Path
import sys

import numpy as np
import pytest

sys.path.insert(0, str(Path(__file__).resolve().parent))
sys.path.insert(0, str(Path(__file__).resolve().parents[2] / "restocker_benchmarks"))

from PIL import Image  # noqa: E402,I100,I101
from restocker_perception.stock import benchmark  # noqa: E402
from test_lane_stock_pipeline import CONFIG, HEIGHT, WIDTH, render, station_pose  # noqa: E402,I100

CATALOGUE = (
    Path(__file__).resolve().parents[2] / "restocker_gazebo/config/generated_product_labels.yaml"
)


def quaternion(matrix):
    import math

    w = math.sqrt(max(0.0, 1.0 + matrix[0, 0] + matrix[1, 1] + matrix[2, 2])) / 2.0
    return [
        (matrix[2, 1] - matrix[1, 2]) / (4 * w),
        (matrix[0, 2] - matrix[2, 0]) / (4 * w),
        (matrix[1, 0] - matrix[0, 1]) / (4 * w),
        w,
    ]


def make_dataset(root: Path, count: int, misplaced_rear: bool):
    """One lane survey frame of lane_01: `count` GEN-001 cans (the rear GEN-002 if asked)."""
    cell = CONFIG.workcell
    pose = station_pose()
    # Lane frame -> world: the lane's centre line at x = -1.0, shelf origin (0, 0.55, 0.75).
    world = pose[:3, 3] + np.array(
        [-1.0 + cell.pose_world[0], cell.pose_world[1], cell.pose_world[2]]
    )
    depth = render([(CONFIG.products["can"], count)], pose)
    frames = root / "frames"
    frames.mkdir(parents=True)
    Image.fromarray((depth * 1000).astype(np.uint16)).save(frames / "f-001.depth.png")
    Image.fromarray(np.full((HEIGHT, WIDTH, 3), 100, dtype=np.uint8)).save(frames / "f-001.png")
    objects = [
        {
            "object_id": f"sim:lane_01_can_{i}",
            "lane_id": "lane_01",
            "lane_order": i,
            "sku": "GEN-002" if (misplaced_rear and i == count - 1) else "GEN-001",
            "misplaced": misplaced_rear and i == count - 1,
            "product_class": "can",
        }
        for i in range(count)
    ]
    row = {
        "frame_id": "f-001",
        "scenario_id": "s",
        "seed": 25,
        "seed_split": "train",
        "sensor": "wrist",
        "viewpoint": "lane_01",
        "camera": {
            "pose_world": list(world) + quaternion(pose[:3, :3]),
            "intrinsics": {
                "fx": 700.9,
                "fy": 700.9,
                "cx": 640.0,
                "cy": 360.0,
                "width": WIDTH,
                "height": HEIGHT,
            },
        },
        "lighting": {"condition": "nominal"},
        "objects": objects,
        "lanes": [
            {
                "lane_id": "lane_01",
                "expected_sku": "GEN-001",
                "count": count,
                "misplaced_count": int(misplaced_rear),
                "skus_in_order": [o["sku"] for o in objects],
                "in_view": True,
            }
        ],
    }
    (root / "manifest.json").write_text("{}", encoding="utf-8")
    (root / "labels.jsonl").write_text(json.dumps(row) + "\n", encoding="utf-8")
    return root


def test_abstain_arm_counts_and_never_flags(tmp_path):
    make_dataset(tmp_path / "d", 3, misplaced_rear=True)
    (row,) = benchmark.predict_datasets([tmp_path / "d"], CATALOGUE, arm="abstain")
    assert row["state"] == "estimated" and row["count"] == 3
    assert row["misplaced"] is False and row["named_count"] == 0
    assert row["measured_coverage"] == pytest.approx(1.0)


def test_oracle_arm_names_the_rear_product_and_flags_a_rear_twin(tmp_path):
    make_dataset(tmp_path / "d", 3, misplaced_rear=True)
    (row,) = benchmark.predict_datasets([tmp_path / "d"], CATALOGUE, arm="oracle")
    assert (
        row["named_count"] == 1
        and row["misplaced"] is True
        and row["misplaced_skus"] == ["GEN-002"]
    )


def test_channel_arm_applies_the_safe_policy(tmp_path):
    make_dataset(tmp_path / "d", 2, misplaced_rear=False)
    calls = []

    def channel(crop):
        calls.append(1)
        return {
            "sidecar_available": True,
            "status": 0,
            "has_sku": True,
            "sku": "GEN-002",
            "evidence_kinds": ["embedding"],
            "evidence_gtins": ["GEN-002"],
        }

    (row,) = benchmark.predict_datasets(
        [tmp_path / "d"], CATALOGUE, arm="channel", channel=channel
    )
    assert calls and row["misplaced"] is False and row["named_count"] == 0
    (loose,) = benchmark.predict_datasets(
        [tmp_path / "d"], CATALOGUE, arm="channel", channel=channel, identity_mode="embedding"
    )
    assert loose["misplaced"] is True  # what the policy prevents


def test_degradation_is_a_pure_function_of_the_frame_id_and_makes_the_evidence_worse():
    masks = [benchmark.degrade_mask(f"seed{n}-wrist-001", 720) for n in range(200)]
    hit = [m for m in masks if m is not None]
    assert 60 < len(hit) < 140
    assert benchmark.degrade_mask("seed5-wrist-001", 720) == benchmark.degrade_mask(
        "seed5-wrist-001", 720
    )
    assert all(
        0 <= top < bottom <= 720 and 0.25 < (bottom - top) / 720 < 0.75 for top, bottom in hit
    )


def test_degraded_frames_either_refuse_or_still_count_right(tmp_path):
    make_dataset(tmp_path / "d", 4, misplaced_rear=False)
    # Force a mask over the whole image by patching the draw, then check the refusal.
    original = benchmark.degrade_mask
    benchmark.degrade_mask = lambda frame_id, height: (0, height)
    try:
        (row,) = benchmark.predict_datasets(
            [tmp_path / "d"], CATALOGUE, arm="abstain", degrade=True
        )
    finally:
        benchmark.degrade_mask = original
    assert row["degraded"] and row["state"] == "cannot_tell" and row["measured_coverage"] == 0.0


def test_unknown_arm_is_refused(tmp_path):
    with pytest.raises(ValueError):
        benchmark.predict_datasets([tmp_path], CATALOGUE, arm="bogus")


def test_perturbation_is_seeded_and_the_count_survives_moderate_noise(tmp_path):
    depth = np.full((40, 60), 1.0)
    a = benchmark.perturb_depth(depth, "f", 0.01, 0.05)
    b = benchmark.perturb_depth(depth, "f", 0.01, 0.05)
    c = benchmark.perturb_depth(depth, "g", 0.01, 0.05)
    assert np.array_equal(a, b) and not np.array_equal(a, c)
    assert 0.02 < (a == 0).mean() < 0.09 and abs(a[a > 0].std() - 0.01) < 0.003
    make_dataset(tmp_path / "d", 4, misplaced_rear=False)
    (row,) = benchmark.predict_datasets(
        [tmp_path / "d"], CATALOGUE, arm="abstain", noise=0.005, dropout=0.02
    )
    assert row["noise"] == 0.005 and row["state"] == "estimated" and row["count"] == 4
    # At 1 % of range the lane depth producer's floor test reads bed speckle as product: the
    # estimator must refuse or be right, never answer a wrong count.
    (rough,) = benchmark.predict_datasets([tmp_path / "d"], CATALOGUE, arm="abstain", noise=0.01)
    assert rough["state"] == "cannot_tell" or rough["count"] == 4

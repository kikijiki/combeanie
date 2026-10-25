# Copyright 2026 Matteo Bernacchia <dev@kikijiki.com>
#
# SPDX-License-Identifier: MIT

"""Lane survey viewpoints for the benchmark recorder."""

import math

from restocker_benchmarks.perception_lane_views import lane_views


def rotate(q, v):
    x, y, z, w = q
    r = [
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ]
    return [sum(r[i][j] * v[j] for j in range(3)) for i in range(3)]


def test_one_view_per_lane_over_its_centre_line_looking_down_the_lane():
    views = lane_views()
    assert [v["name"] for v in views] == [f"lane_0{n}" for n in range(1, 7)]
    first = views[0]
    pose = first["camera_optical_pose"]
    assert pose[:3] == [-1.0, 0.0, 1.3]
    boresight = rotate(pose[3:], [0, 0, 1])
    right = rotate(pose[3:], [1, 0, 0])
    down = rotate(pose[3:], [0, 1, 0])
    assert boresight[1] > 0.8 and boresight[2] < 0 and abs(boresight[0]) < 1e-6
    assert right[0] > 0.999  # image-right along the shelf's +X, as the survey stations say
    assert down[2] < 0
    # Aimed at mid-lane: the boresight reaches the lane floor plane at depth 0.435.
    t = (0.75 - pose[2]) / boresight[2]
    assert math.isclose(pose[1] + t * boresight[1], 0.55 + 0.435, abs_tol=1e-4)


def test_target_is_the_rearmost_product_of_each_lane():
    truth = {
        "objects": [
            {"object_id": "a", "lane_id": "lane_01", "lane_order": 0},
            {"object_id": "b", "lane_id": "lane_01", "lane_order": 2},
            {"object_id": "c", "lane_id": "lane_01", "lane_order": 1},
            {"object_id": "t", "lane_id": None, "lane_order": None},
        ]
    }
    views = {v["lane_id"]: v for v in lane_views(truth)}
    assert views["lane_01"]["target_object_id"] == "b"
    assert views["lane_02"]["target_object_id"] is None

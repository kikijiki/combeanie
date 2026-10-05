#!/usr/bin/env python3
"""Scan a recorded run for links that penetrate the workcell collision geometry."""

import bisect
import contextlib
import csv
from pathlib import Path
import sys

import numpy as np

# fk.py is a sibling; this runs as a script, not a package.
sys.path.insert(0, str(Path(__file__).resolve().parent))

from fk import OBSTACLES, SHAPES, link_frames  # noqa: E402

J = [f"joint_{i}" for i in range(1, 7)]
GRID = np.linspace(-1.0, 1.0, 5)


def sample_points(shape, dims, centre):
    if shape == "box":
        hx, hy, hz = dims
    else:
        hx = hy = dims[0]
        hz = dims[1]
    pts = []
    for a in GRID:
        for b in GRID:
            for c in GRID:
                if shape == "cyl" and a * a + b * b > 1.0:
                    continue
                pts.append([centre[0] + a * hx, centre[1] + b * hy, centre[2] + c * hz, 1.0])
    return np.array(pts).T


SAMPLES = {n: sample_points(*v) for n, v in SHAPES.items()}


def read_rail(directory):
    """Read the rail sidecar as (stamp, position) pairs, tolerating a truncated trailing line."""
    pairs = []
    try:
        with open(directory + "/arm_state.csv.rail") as handle:
            for line in handle:
                fields = line.strip().split(",")
                if len(fields) < 3 or not fields[0] or not fields[2]:
                    continue
                with contextlib.suppress(ValueError):
                    pairs.append((float(fields[0]), float(fields[2])))
    except OSError:
        pass
    return pairs


def read_rows(directory):
    with open(directory + "/arm_state.csv") as handle:
        return list(csv.DictReader(handle))


def scan(d, step=5, t0=None, t1=None):
    rows = read_rows(d)
    rail = read_rail(d)
    rt = [r[0] for r in rail]

    def rail_at(t):
        if not rail:
            return 0.0
        i = min(max(bisect.bisect_left(rt, t), 0), len(rail) - 1)
        return rail[i][1]

    worst = {}
    for r in rows[::step]:
        t = float(r["stamp"])
        if t0 is not None and not (t0 <= t <= t1):
            continue
        q = [float(r[j + "_fb"]) for j in J]
        frames = link_frames(rail_at(t), q, 0.032, 0.032)
        for link, pts in SAMPLES.items():
            w = (frames[link] @ pts)[:3].T
            for name, lo, hi in OBSTACLES:
                # signed penetration depth: positive inside
                depth = np.minimum(w - lo, hi - w).min(axis=1)
                m = depth.max()
                if m > 0:
                    key = (link, name)
                    if key not in worst or m > worst[key][0]:
                        worst[key] = (m, t)
    return worst


if __name__ == "__main__":
    for d in sys.argv[1:]:
        print(f"### {d}")
        w = scan(d)
        for (link, obs), (depth, t) in sorted(w.items(), key=lambda kv: -kv[1][0]):
            print(f"  {link:20s} inside {obs:16s} depth={depth * 1000:7.1f} mm  t={t:.2f}")
        if not w:
            print("  no penetration found")


def product_proximity(d, t, radius=0.06, half_height=0.16):
    """Distance from each ground-truth product to the tool0 at time t."""
    rows = read_rows(d)
    rail = read_rail(d)
    rt = [r[0] for r in rail]
    i = min(max(bisect.bisect_left(rt, t), 0), len(rail) - 1)
    r = min(rows, key=lambda x: abs(float(x["stamp"]) - t))
    q = [float(r[j + "_fb"]) for j in J]
    frames = link_frames(rail[i][1], q, 0.032, 0.032)
    latest = {}
    with open(d + "/arm_state.csv.obj") as handle:
        for row in csv.DictReader(handle):
            if float(row["stamp"]) <= t:
                latest[row["source_object_id"]] = row
    out = []
    for oid, row in latest.items():
        p = np.array([float(row["x"]), float(row["y"]), float(row["z"])])
        for link in (
            "tool0",
            "gripper",
            "left_finger",
            "right_finger",
            "wrist_3_link",
            "wrist_2_link",
            "wrist_1_link",
            "forearm_link",
        ):
            lp = frames[link][:3, 3]
            out.append((float(np.linalg.norm(lp - p)), oid, link, row["frame"], tuple(p.round(3))))
    out.sort()
    return out[:8]

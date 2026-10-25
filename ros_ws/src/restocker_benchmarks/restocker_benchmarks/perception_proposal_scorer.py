# Copyright 2026 Matteo Bernacchia <dev@kikijiki.com>
#
# SPDX-License-Identifier: MIT

"""Score product proposals against analytic ground-truth masks (CMB-113)."""
# The renderer gives no instance masks, so the ground truth is the scene document ray-cast through
# the recorded camera: every labelled product is a cylinder (the collision catalogue's radius and
# height, any pose), each pixel belongs to the nearest one, and a pixel is kept only if the
# recorded depth agrees with it. The agreement test is what makes this a mask of the *rendered*
# product rather than of the model: a pixel where the sensor saw something clearly nearer than the
# cylinder is occluded by something that is not a labelled product (the arm, a divider) and is
# dropped; a pixel where the sensor saw something up to 3 cm behind it is a contour bottle's waist
# and is kept. ``depth_residual_mm`` reports how far the model is from the depth on kept pixels, so
# the mask error is a number and not an assumption.
#
# A proposal recalls a product when it matches it one to one at mask IoU >= 0.5, greedily from the
# best pair down, over products that are recallable (enough visible pixels and visible fraction).
# That is strict: a proposal that merges two touching products recalls at most one of them.
# ``loose`` recall (a proposal covers at least half of the product's visible mask, merges allowed)
# is reported next to it so a merge is distinguishable from a miss.

from __future__ import annotations

from collections import defaultdict
from collections.abc import Sequence
import math
from typing import Any

import numpy as np

from restocker_benchmarks.perception_scorer import rate, summary

SHAPES = {
    "can": (0.033, 0.122),
    "small_bottle": (0.034, 0.200),
    "large_bottle": (0.045, 0.290),
    "contour_bottle": (0.036, 0.230),
}
# Depth agreement window around the analytic front surface, metres.
NEARER_TOLERANCE_M = 0.020
DEEPER_TOLERANCE_M = 0.030
# A product is recallable when this many of its pixels are visible and this fraction of its
# in-image silhouette is visible; below that no proposer can be asked to find it.
MIN_VISIBLE_PIXELS = 250
MIN_VISIBLE_FRACTION = 0.30
MATCH_IOU = 0.5
LOOSE_COVER = 0.5
OCCLUSION_BINS = (("clear", 0.9), ("partial", 0.6), ("heavy", 0.0))
GAP_BINS = (("touching", 0.003), ("close", 0.015), ("separate", math.inf))


def quaternion_matrix(quaternion: Sequence[float]) -> np.ndarray:
    x, y, z, w = quaternion
    n = math.sqrt(x * x + y * y + z * z + w * w)
    x, y, z, w = x / n, y / n, z / n, w / n
    return np.array(
        [
            [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
            [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
            [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
        ]
    )


def shape_of(obj: dict[str, Any]) -> tuple[float, float]:
    """(radius, height) of a labelled product; for a box, the footprint's bounding radius."""
    if obj["product_class"] == "box":
        sx, sy, sz = obj["size_xyz_m"]
        return 0.5 * math.hypot(sx, sy), sz
    return SHAPES[obj["product_class"]]


def cylinder_of(obj: dict[str, Any]) -> dict[str, Any]:
    """Centre, unit axis, radius and half length of a labelled product.

    A box is returned in the same dict (``radius`` its bounding radius, ``half`` half its height,
    which bound the search window) plus ``box``, its size in the body frame.
    """
    radius, height = shape_of(obj)
    rotation = quaternion_matrix(obj["pose_world"][3:7])
    body = {
        "centre": np.array(obj["pose_world"][:3], dtype=float),
        "rotation": rotation,
        "axis": rotation[:, 2],
        "radius": radius,
        "half": height / 2.0,
    }
    if obj["product_class"] == "box":
        body["box"] = np.array(obj["size_xyz_m"], dtype=float)
    return body


def is_lying(obj: dict[str, Any]) -> bool:
    """True when the product's axis is more than 45 degrees from the vertical."""
    return abs(cylinder_of(obj)["axis"][2]) < math.cos(math.radians(45.0))


def cylinder_depth(
    rays: np.ndarray, origin: np.ndarray, cylinder: dict[str, Any], window: tuple[slice, slice]
) -> np.ndarray:
    """Optical z-depth of the first cylinder hit for every pixel of ``window`` (inf: miss)."""
    d = rays[window] @ cylinder["rotation"]  # direction in the cylinder frame (z = axis)
    o = (origin - cylinder["centre"]) @ cylinder["rotation"]
    if "box" in cylinder:
        # Slab test against the oriented box: first entry distance, inf on a miss.
        half_size = cylinder["box"] / 2.0
        with np.errstate(divide="ignore", invalid="ignore"):
            t1 = (-half_size - o) / d
            t2 = (half_size - o) / d
        enter = np.minimum(t1, t2).max(axis=-1)
        leave = np.maximum(t1, t2).min(axis=-1)
        return np.where((enter <= leave) & (leave > 0) & (enter > 0), enter, np.inf)
    r, half = cylinder["radius"], cylinder["half"]
    best = np.full(d.shape[:2], np.inf)
    with np.errstate(divide="ignore", invalid="ignore"):
        a = d[..., 0] ** 2 + d[..., 1] ** 2
        b = 2.0 * (o[0] * d[..., 0] + o[1] * d[..., 1])
        c = o[0] ** 2 + o[1] ** 2 - r * r
        disc = b * b - 4.0 * a * c
        root = np.sqrt(np.where(disc >= 0, disc, np.nan))
        for sign in (-1.0, 1.0):
            t = (-b + sign * root) / (2.0 * a)
            z = o[2] + t * d[..., 2]
            ok = (t > 0) & (np.abs(z) <= half)
            best = np.where(ok & (t < best), t, best)
        for cap in (-half, half):
            t = (cap - o[2]) / d[..., 2]
            x = o[0] + t * d[..., 0]
            y = o[1] + t * d[..., 1]
            ok = (t > 0) & (x * x + y * y <= r * r)
            best = np.where(ok & (t < best), t, best)
    return best


def _window(camera: Any, centre: np.ndarray, radius_m: float) -> tuple[slice, slice] | None:
    """Pixel window that contains a sphere of ``radius_m`` around a world point, or None."""
    rotation = camera.rotation()
    local = rotation.T @ (centre - np.array(camera.pose_world[:3]))
    if local[2] <= 0.02:
        return None
    u = camera.fx * local[0] / local[2] + camera.cx
    v = camera.fy * local[1] / local[2] + camera.cy
    side = camera.fx * radius_m * 1.4 / max(local[2] - radius_m, 0.02) + 4.0
    u0, u1 = int(max(0, math.floor(u - side))), int(min(camera.width, math.ceil(u + side)))
    v0, v1 = int(max(0, math.floor(v - side))), int(min(camera.height, math.ceil(v + side)))
    if u1 <= u0 or v1 <= v0:
        return None
    return slice(v0, v1), slice(u0, u1)


def ground_truth(
    objects: Sequence[dict[str, Any]], camera: Any, depth_m: np.ndarray
) -> list[dict[str, Any]]:
    """Per-object analytic masks and visibility for one frame.

    Each entry: ``object_id``, ``silhouette_px`` (alone, inside the image), ``visible_px``,
    ``visible_fraction``, ``in_image_fraction``, ``depth_residual_mm`` (median |sensor - model| on
    visible pixels), ``mask`` (bool image) and the raw ``label`` row.
    """
    rays, origin = camera.rays_world()
    cylinders = [cylinder_of(obj) for obj in objects]
    nearest = np.full(depth_m.shape, np.inf)
    owner = np.full(depth_m.shape, -1, dtype=np.int32)
    alone: list[tuple[tuple[slice, slice], np.ndarray] | None] = []
    for index, cylinder in enumerate(cylinders):
        reach = math.hypot(cylinder["radius"], cylinder["half"])
        window = _window(camera, cylinder["centre"], reach)
        if window is None:
            alone.append(None)
            continue
        t = cylinder_depth(rays, origin, cylinder, window)
        alone.append((window, t))
        view = nearest[window]
        closer = t < view
        view[closer] = t[closer]
        owner[window][closer] = index
    results = []
    for index, obj in enumerate(objects):
        entry: dict[str, Any] = {"object_id": obj["object_id"], "label": obj}
        if alone[index] is None:
            entry.update(
                silhouette_px=0,
                visible_px=0,
                visible_fraction=0.0,
                in_image_fraction=0.0,
                depth_residual_mm=None,
                mask=np.zeros(depth_m.shape, dtype=bool),
            )
            results.append(entry)
            continue
        window, t = alone[index]
        hit = np.isfinite(t)
        sensor = depth_m[window]
        valid = sensor > 0
        consistent = (
            hit & valid & (sensor >= t - NEARER_TOLERANCE_M) & (sensor <= t + DEEPER_TOLERANCE_M)
        )
        front = owner[window] == index
        mask_window = consistent & front
        mask = np.zeros(depth_m.shape, dtype=bool)
        mask[window] = mask_window
        # The silhouette that is in the image, over the window the product could occupy.
        silhouette = int(hit.sum())
        full = _full_silhouette_px(camera, cylinders[index], rays, origin)
        residual = np.abs(sensor[mask_window] - t[mask_window]) * 1000.0
        entry.update(
            silhouette_px=silhouette,
            visible_px=int(mask_window.sum()),
            visible_fraction=float(mask_window.sum()) / silhouette if silhouette else 0.0,
            in_image_fraction=silhouette / full if full else 0.0,
            depth_residual_mm=float(np.median(residual)) if residual.size else None,
            mask=mask,
        )
        results.append(entry)
    return results


def _full_silhouette_px(
    camera: Any, cylinder: dict[str, Any], rays: np.ndarray, origin: np.ndarray
) -> int:
    """Pixels the product would cover on an unbounded image (estimated at coarse resolution)."""
    # Project a dense sample of the cylinder surface; the count is scaled by pixel area. A box is
    # convex, so the hull of its eight corners is its silhouette exactly.
    if "box" in cylinder:
        corners = np.array(
            [[sx, sy, sz] for sx in (-1, 1) for sy in (-1, 1) for sz in (-1, 1)], dtype=float
        ) * (cylinder["box"] / 2.0)
        points = corners @ cylinder["rotation"].T + cylinder["centre"]
        return _hull_pixels(camera, points, origin)
    return _hull_pixels(camera, _cylinder_surface_points(cylinder), origin)


def _cylinder_surface_points(cylinder: dict[str, Any]) -> np.ndarray:
    r, half = cylinder["radius"], cylinder["half"]
    angles = np.linspace(0.0, 2.0 * math.pi, 48, endpoint=False)
    rows = np.linspace(-half, half, 24)
    side = np.stack(
        [
            np.repeat(r * np.cos(angles)[None, :], len(rows), axis=0),
            np.repeat(r * np.sin(angles)[None, :], len(rows), axis=0),
            np.repeat(rows[:, None], len(angles), axis=1),
        ],
        axis=-1,
    ).reshape(-1, 3)
    radii = np.linspace(0.0, r, 8)
    caps = np.concatenate(
        [
            np.stack(
                [
                    rr * np.cos(angles),
                    rr * np.sin(angles),
                    np.full_like(angles, z),
                ],
                axis=-1,
            )
            for rr in radii
            for z in (-half, half)
        ]
    )
    return np.concatenate([side, caps]) @ cylinder["rotation"].T + cylinder["centre"]


def _hull_pixels(camera: Any, points: np.ndarray, origin: np.ndarray) -> int:
    local = (points - origin) @ camera.rotation()
    ok = local[:, 2] > 0.02
    pu = camera.fx * local[ok, 0] / local[ok, 2] + camera.cx
    pv = camera.fy * local[ok, 1] / local[ok, 2] + camera.cy
    if pu.size == 0:
        return 0
    hull_u0, hull_u1 = pu.min(), pu.max()
    hull_v0, hull_v1 = pv.min(), pv.max()
    # Rasterise the convex projected outline into a small grid to count its pixels.
    import cv2  # noqa: PLC0415 - optional dependency, only the scorer needs it

    points2d = np.column_stack([pu, pv]).astype(np.float32)
    hull = cv2.convexHull(points2d)
    width = int(math.ceil(hull_u1 - hull_u0)) + 3
    height = int(math.ceil(hull_v1 - hull_v0)) + 3
    if width * height > 4_000_000:
        return 0
    canvas = np.zeros((height, width), dtype=np.uint8)
    shifted = (hull - np.array([hull_u0 - 1, hull_v0 - 1], dtype=np.float32)).astype(np.int32)
    cv2.fillConvexPoly(canvas, shifted, 1)
    return int(canvas.sum())


def recallable(entry: dict[str, Any]) -> bool:
    return (
        entry["visible_px"] >= MIN_VISIBLE_PIXELS
        and entry["visible_fraction"] >= MIN_VISIBLE_FRACTION
        and entry["in_image_fraction"] >= 0.6
    )


def occlusion_bin(fraction: float) -> str:
    for name, lower in OCCLUSION_BINS:
        if fraction >= lower:
            return name
    return OCCLUSION_BINS[-1][0]


def neighbour_gap(obj: dict[str, Any], others: Sequence[dict[str, Any]]) -> float:
    """Smallest surface-to-surface distance to another product, centre distance less radii."""
    ra = shape_of(obj)[0]
    best = math.inf
    for other in others:
        if other["object_id"] == obj["object_id"]:
            continue
        rb = shape_of(other)[0]
        d = math.dist(obj["pose_world"][:3], other["pose_world"][:3])
        # Lying products reach further than their radius: use the larger of radius, half length.
        ea = max(ra, shape_of(obj)[1] / 2.0 if is_lying(obj) else ra)
        eb = max(rb, shape_of(other)[1] / 2.0 if is_lying(other) else rb)
        best = min(best, d - ea - eb)
    return best


def gap_bin(gap: float) -> str:
    for name, upper in GAP_BINS:
        if gap < upper:
            return name
    return GAP_BINS[-1][0]


def _iou(a: np.ndarray, b: np.ndarray) -> float:
    union = np.logical_or(a, b).sum()
    return float(np.logical_and(a, b).sum()) / union if union else 0.0


def score_frame(
    row: dict[str, Any],
    truth: Sequence[dict[str, Any]],
    labels: np.ndarray,
    proposals: Sequence[Any],
) -> dict[str, Any]:
    """Match one frame's proposals to its ground truth; return object and proposal records."""
    ids = [p.proposal_id for p in proposals]
    masks = {pid: labels == pid for pid in ids}
    candidates = []
    for index, entry in enumerate(truth):
        if not recallable(entry):
            continue
        for pid in ids:
            iou = _iou(entry["mask"], masks[pid])
            if iou > 0.0:
                candidates.append((iou, index, pid))
    candidates.sort(reverse=True)
    taken_objects: dict[int, tuple[int, float]] = {}
    taken_proposals: dict[int, int] = {}
    for iou, index, pid in candidates:
        if iou < MATCH_IOU:
            break
        if index in taken_objects or pid in taken_proposals:
            continue
        taken_objects[index] = (pid, iou)
        taken_proposals[pid] = index

    objects_all = [entry["label"] for entry in truth]
    object_records = []
    for index, entry in enumerate(truth):
        if not recallable(entry):
            continue
        obj = entry["label"]
        matched = taken_objects.get(index)
        covered = 0.0
        for pid in ids:
            overlap = np.logical_and(entry["mask"], masks[pid]).sum()
            covered = max(covered, float(overlap) / max(entry["visible_px"], 1))
        object_records.append(
            {
                "frame_id": row["frame_id"],
                "scene": row["scenario_id"],
                "object_id": obj["object_id"],
                "sku": obj["sku"],
                "surface": "lane" if obj.get("lane_id") else "tray",
                "shape": obj["product_class"],
                "pose": "lying" if is_lying(obj) else "upright",
                "view": row.get("viewpoint") or "home",
                "sensor": row["sensor"],
                "lighting": row["lighting"]["condition"],
                "tray_kind": row.get("tray_kind"),
                "visible_fraction": entry["visible_fraction"],
                "occlusion": occlusion_bin(entry["visible_fraction"]),
                "gap": gap_bin(neighbour_gap(obj, objects_all)),
                "range_m": obj.get("range_m"),
                "recalled": matched is not None,
                "iou": None if matched is None else matched[1],
                "loose": covered >= LOOSE_COVER,
                "disc_class": None
                if matched is None
                else next(p.catalogue_class for p in proposals if p.proposal_id == matched[0]),
                "matched_kind": None
                if matched is None
                else next(p.kind for p in proposals if p.proposal_id == matched[0]),
            }
        )
    proposal_records = []
    for proposal in proposals:
        mask = masks[proposal.proposal_id]
        best_gt, best_share = None, 0.0
        for entry in truth:
            overlap = float(np.logical_and(mask, entry["mask"]).sum())
            share = overlap / max(int(mask.sum()), 1)
            if share > best_share:
                best_gt, best_share = entry["object_id"], share
        covered_objects = sum(
            1
            for entry in truth
            if np.logical_and(mask, entry["mask"]).sum() >= 0.25 * max(int(mask.sum()), 1)
        )
        proposal_records.append(
            {
                "frame_id": row["frame_id"],
                "scene": row["scenario_id"],
                "merged": covered_objects >= 2,
                "proposal_id": proposal.proposal_id,
                "surface": proposal.surface,
                "kind": proposal.kind,
                "view": row.get("viewpoint") or "home",
                "matched": proposal.proposal_id in taken_proposals,
                "best_object": best_gt,
                "overlap_share": best_share,
                "foreign": best_share < 0.3,
                "n_pixels": proposal.n_pixels,
                "tray_kind": row.get("tray_kind"),
            }
        )
    return {"objects": object_records, "proposals": proposal_records}


BOOTSTRAP_DRAWS = 4000
BOOTSTRAP_SEED = 0x43494E31  # pinned: the interval of a given record set never changes


def cluster_rate(
    records: Sequence[dict[str, Any]], flag: str, cluster: str = "scene"
) -> dict[str, Any]:
    """A proportion with a scene-clustered bootstrap 95 % interval (and the plain Wilson one).

    Products and views of one scene are not independent, so the interval resamples whole scenes.
    ``flag`` is a record key (bool); a record counts as a success when it is truthy.
    """
    n = len(records)
    out = rate(sum(1 for r in records if r[flag]), n)
    groups: dict[str, list[int]] = defaultdict(lambda: [0, 0])
    for record in records:
        entry = groups[str(record[cluster])]
        entry[0] += 1 if record[flag] else 0
        entry[1] += 1
    out["scenes"] = len(groups)
    out["wilson95"] = out.pop("ci95")
    if len(groups) < 2:
        out["cluster_ci95"] = None
        return out
    table = np.array(list(groups.values()), dtype=float)
    draws = np.random.default_rng(BOOTSTRAP_SEED + n).integers(
        0, len(table), size=(BOOTSTRAP_DRAWS, len(table))
    )
    k, total = table[draws, 0].sum(axis=1), table[draws, 1].sum(axis=1)
    values = k / np.maximum(total, 1.0)
    out["cluster_ci95"] = [float(np.quantile(values, 0.025)), float(np.quantile(values, 0.975))]
    return out


STRATA_KEYS = ("surface", "shape", "pose", "view", "lighting", "tray_kind", "occlusion", "gap")


def aggregate(
    object_records: Sequence[dict[str, Any]],
    proposal_records: Sequence[dict[str, Any]],
    keys: Sequence[str] = STRATA_KEYS,
) -> dict[str, Any]:
    """Overall and per-stratum strict and loose recall, proposal quality, with intervals."""
    with_loose = [{**r, "loose_or_strict": r["loose"] or r["recalled"]} for r in object_records]

    def block(records: Sequence[dict[str, Any]]) -> dict[str, Any]:
        return {
            "recall_strict": cluster_rate(records, "recalled"),
            "recall_loose": cluster_rate(records, "loose_or_strict"),
        }

    out: dict[str, Any] = {"overall": block(with_loose), "strata": {}}
    for key in keys:
        groups: dict[str, list] = defaultdict(list)
        for record in with_loose:
            groups[str(record[key])].append(record)
        out["strata"][key] = {name: block(rows) for name, rows in sorted(groups.items())}
    n_prop = len(proposal_records)
    out["proposals"] = {
        "n": n_prop,
        "precision_strict": rate(sum(p["matched"] for p in proposal_records), n_prop),
        "foreign": rate(sum(p["foreign"] for p in proposal_records), n_prop),
        "merged": rate(sum(p["merged"] for p in proposal_records), n_prop),
        "kinds": {
            kind: sum(1 for p in proposal_records if p["kind"] == kind)
            for kind in sorted({p["kind"] for p in proposal_records})
        },
    }
    matched = [r for r in object_records if r["iou"] is not None]
    out["matched_iou"] = summary([r["iou"] for r in matched])
    return out


def hard_stratum_records(
    records: Sequence[dict[str, Any]],
) -> dict[str, list[dict[str, Any]]]:
    """The six hard strata of the pre-declared bounds file, each a list of object records."""
    return {
        "pose = lying": [r for r in records if r["pose"] == "lying"],
        "shape = contour_bottle": [r for r in records if r["shape"] == "contour_bottle"],
        "gap = touching": [r for r in records if r["gap"] == "touching"],
        "occlusion = partial": [r for r in records if r["occlusion"] == "partial"],
        "surface = lane": [r for r in records if r["surface"] == "lane"],
        "tray_kind in {dense, rows, tall_front}": [
            r for r in records if r["tray_kind"] in ("dense", "rows", "tall_front")
        ],
    }


def decide(
    object_records: Sequence[dict[str, Any]],
    proposal_records: Sequence[dict[str, Any]],
    latencies_ms: Sequence[float],
    bounds: dict[str, Any],
) -> dict[str, Any]:
    """Apply the bounds file's pass rule; returns each bound's verdict and the overall one."""
    limits = bounds["bounds"]
    need = bounds["required_sample"]
    checks: dict[str, Any] = {}
    flagged = [dict(r) for r in object_records]
    overall = cluster_rate(flagged, "recalled")
    checks["overall_strict_recall"] = _recall_verdict(
        overall,
        limits["overall_strict_recall_min"],
        0.85,
        need["recallable_products_overall_min"],
        None,
    )
    for name, rows in hard_stratum_records(flagged).items():
        rate_ = cluster_rate(rows, "recalled")
        checks[f"hard: {name}"] = _recall_verdict(
            rate_,
            limits["hard_stratum_strict_recall_min"],
            0.80,
            need["per_hard_stratum_min"],
            need["scenes_per_hard_stratum_min"],
        )
    foreign = rate(sum(p["foreign"] for p in proposal_records), len(proposal_records))
    checks["foreign_proposal_rate"] = {
        **foreign,
        "bound": limits["foreign_proposal_rate_max"],
        "verdict": "pass"
        if foreign["value"] is not None and foreign["value"] <= limits["foreign_proposal_rate_max"]
        else "fail",
    }
    p95 = float(np.quantile(latencies_ms, 0.95)) if len(latencies_ms) else None
    checks["proposer_latency_p95_ms"] = {
        "value": p95,
        "bound": limits["proposer_latency_p95_ms_max"],
        "verdict": "pass"
        if p95 is not None and p95 <= limits["proposer_latency_p95_ms_max"]
        else "fail",
    }
    verdicts = [c["verdict"] for c in checks.values()]
    sufficient = all(v == "pass" for v in verdicts)
    return {
        "checks": checks,
        "geometry_sufficient": sufficient,
        "verdict": "SUFFICIENT" if sufficient else "INSUFFICIENT",
    }


def decide_boxes(
    object_records: Sequence[dict[str, Any]],
    proposal_records: Sequence[dict[str, Any]],
    latencies_ms: Sequence[float],
    bounds: dict[str, Any],
) -> dict[str, Any]:
    """Apply a CMB-120 box bounds file (``"kind": "box_proposals"``): one verdict per check.

    Every check names a metric, the records it is over (``where``: record field equals value,
    ``where_not`` the opposite), a bound ``value`` and the scene-clustered limit the interval must
    clear (``limit``: its lower end for a minimum, its upper end for a maximum), and the sample it
    needs (``required_n``, ``required_scenes``). A check passes when the point estimate meets the
    bound, the interval's limit does, and the sample is met; it fails when the point estimate
    misses the bound or, with the sample met, when the interval's limit misses; a sample that is
    too small is inconclusive. The overall verdict is PASS only if every check passes.
    """
    objects = [dict(r) for r in object_records]
    checks: dict[str, Any] = {}
    for spec in bounds["checks"]:
        metric = spec["metric"]
        if metric in ("strict_recall", "cylinder_class_share", "rect_share_of_cylinders"):
            rows = [
                r
                for r in objects
                if all(r.get(k) == v for k, v in spec.get("where", {}).items())
                and all(r.get(k) != v for k, v in spec.get("where_not", {}).items())
            ]
            if metric == "cylinder_class_share":
                rows = [{**r, "flag": r["recalled"] and r["disc_class"] is not None} for r in rows if r["recalled"]]
            elif metric == "rect_share_of_cylinders":
                rows = [{**r, "flag": r["matched_kind"] == "rect"} for r in rows if r["recalled"]]
            else:
                rows = [{**r, "flag": r["recalled"]} for r in rows]
            value = cluster_rate(rows, "flag")
        elif metric == "foreign_proposal_rate":
            value = rate(sum(p["foreign"] for p in proposal_records), len(proposal_records))
            value["scenes"] = len({p["scene"] for p in proposal_records})
            value["cluster_ci95"] = value["ci95"]
        elif metric == "latency_p95_ms":
            p95 = float(np.quantile(latencies_ms, 0.95)) if len(latencies_ms) else None
            checks[spec["id"]] = {
                "value": p95,
                "bound": spec["value"],
                "verdict": "pass" if p95 is not None and p95 <= spec["value"] else "fail",
            }
            continue
        else:
            raise ValueError(f"unknown metric {metric!r}")
        checks[spec["id"]] = _bound_verdict(value, spec)
    verdicts = [c["verdict"] for c in checks.values()]
    overall = (
        "PASS" if all(v == "pass" for v in verdicts) else "FAIL" if "fail" in verdicts else "INCONCLUSIVE"
    )
    return {"checks": checks, "verdict": overall}


def _bound_verdict(rate_: dict[str, Any], spec: dict[str, Any]) -> dict[str, Any]:
    interval = rate_.get("cluster_ci95")
    verdict = "pass"
    if rate_["n"] < spec["required_n"] or rate_.get("scenes", 0) < spec.get("required_scenes", 0):
        verdict = "inconclusive"
    elif spec["direction"] == "min":
        if rate_["value"] < spec["value"]:
            verdict = "fail"
        elif interval is None or interval[0] < spec["limit"]:
            verdict = "fail"
    else:
        if rate_["value"] > spec["value"]:
            verdict = "fail"
        elif interval is None or interval[1] > spec["limit"]:
            verdict = "fail"
    return {
        **rate_,
        "direction": spec["direction"],
        "bound": spec["value"],
        "interval_limit_required": spec["limit"],
        "verdict": verdict,
    }


def _recall_verdict(
    rate_: dict[str, Any],
    bound: float,
    lower_limit: float,
    n_min: int,
    scenes_min: int | None,
) -> dict[str, Any]:
    verdict = "pass"
    if rate_["n"] < n_min or (scenes_min is not None and rate_["scenes"] < scenes_min):
        verdict = "inconclusive"
    elif rate_["value"] < bound:
        verdict = "fail"
    elif rate_["cluster_ci95"] is None or rate_["cluster_ci95"][0] < lower_limit:
        verdict = "inconclusive"
    return {**rate_, "bound": bound, "lower_limit_required": lower_limit, "verdict": verdict}

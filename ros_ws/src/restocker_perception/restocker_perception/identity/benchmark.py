# Copyright 2026 Matteo Bernacchia <dev@kikijiki.com>
#
# SPDX-License-Identifier: MIT

"""Run the identity channel over perception-benchmark datasets: calibrate, then evaluate."""
# Consumes a benchmark dataset through its public format only (manifest.json, labels.jsonl, PNG
# frames; see the restocker_benchmarks README). Crops are cut from the label rows: the object's
# projected pixel and range give a square of the product's catalogue extent. Nothing here imports
# restocker_benchmarks, so this runs in the plain Python environment that has the ML packages.
#
# Two entry points, with the guards the pre-declared protocol needs:
#   calibrate  reads only seeds the split file lists as train or validation, refuses any other
#              (held-out or unlisted), builds leave-products-out indices so that unknown products
#              exist in calibration data, fits the whitening on unlabeled crops and the thresholds
#              on the observations, and writes the calibration file;
#   evaluate   reads only held-out seeds, uses the committed calibration file and bounds file
#              as they are, and refuses to overwrite an earlier result (one run, no re-tuning).

from __future__ import annotations

from collections import defaultdict
from collections.abc import Callable, Iterator, Sequence
from dataclasses import dataclass, field
import hashlib
import json
import math
from pathlib import Path
import time
from typing import Any

import numpy as np

from restocker_perception.identity.barcode import BarcodeBackend
from restocker_perception.identity.calibrate import (
    Metrics,
    Sample,
    choose_floor_margin,
    cross_validate,
    evaluate,
    fit_thresholds,
)
from restocker_perception.identity.config import (
    IdentityConfig,
    load_calibration,
    write_calibration,
)
from restocker_perception.identity.crops import crop_square, unwrap_cylinder
from restocker_perception.identity.engine import (
    ABSTAIN,
    IDENTIFIED,
    UNKNOWN,
    IdentityEngine,
    Observation,
    decide,
)
from restocker_perception.registry.backends import EmbeddingBackend, OcrBackend
from restocker_perception.registry.cache import DerivedCache
from restocker_perception.registry.imageio import read_image
from restocker_perception.registry.model import Registry
from restocker_perception.registry.retrieval import RegistryIndex, Whitener
from restocker_perception.registry.simulated import load_split_file

CALIBRATION_SPLITS = ("train", "validation")
RANGE_BANDS = ((0.35, "<=0.35m"), (0.45, "0.35-0.45m"), (math.inf, ">0.45m"))
Z_95 = 1.959963984540054


class ProtocolError(RuntimeError):
    """A run would break the pre-declared protocol (held-out leakage, a second held-out run)."""


# --- statistics ------------------------------------------------------------------------------


def wilson(successes: int, n: int) -> list[float] | None:
    if n <= 0:
        return None
    p = successes / n
    z2 = Z_95 * Z_95
    centre = (p + z2 / (2 * n)) / (1 + z2 / n)
    half = Z_95 * math.sqrt(p * (1 - p) / n + z2 / (4 * n * n)) / (1 + z2 / n)
    return [max(0.0, centre - half), min(1.0, centre + half)]


def percentile(values: Sequence[float], q: float) -> float | None:
    return float(np.percentile(np.asarray(values, dtype=np.float64), q)) if len(values) else None


# --- datasets ----------------------------------------------------------------------------------


@dataclass(frozen=True)
class ViewCrop:
    frame_id: str
    camera_key: str
    range_m: float
    lighting: str
    image: np.ndarray
    # Label-side facts for diagnosis (object and camera pose in world, crop side, geometry key).
    meta: dict[str, Any] = field(default_factory=dict)


@dataclass(frozen=True)
class ObjectGroup:
    """One object (in one scene) with every usable close view of it."""

    seed: int
    seed_split: str
    object_id: str
    sku: str  # catalogue id, e.g. GEN-003
    product_class: str
    views: tuple[ViewCrop, ...]
    # Crops from the aimed barcode viewpoints (also among ``views`` when they fit the frame).
    barcode_images: tuple[np.ndarray, ...] = ()


def load_dataset(directory: str | Path) -> tuple[dict[str, Any], list[dict[str, Any]]]:
    root = Path(directory)
    manifest = json.loads((root / "manifest.json").read_text(encoding="utf-8"))
    rows = [
        json.loads(line)
        for line in (root / "labels.jsonl").read_text(encoding="utf-8").splitlines()
        if line.strip()
    ]
    return manifest, rows


def _depth_agrees(
    depth: np.ndarray | None, u: float, v: float, range_m: float, tol: float
) -> bool:
    """The depth image at the object centre is the object, not something in front of it."""
    if depth is None:
        return True
    row, col = int(round(v)), int(round(u))
    patch = depth[max(row - 2, 0) : row + 3, max(col - 2, 0) : col + 3].astype(np.float64)
    patch = patch[patch > 0] / 1000.0
    if not len(patch):
        return False
    # range_m is along the ray, depth along the optical axis: the axial value is never larger.
    return bool(np.median(patch) <= range_m + tol and np.median(patch) >= 0.6 * range_m)


def object_groups(
    directory: str | Path,
    geometry: dict[str, dict[str, Any]],
    *,
    allowed_splits: Sequence[str],
    max_range_m: float = 0.6,
    min_crop_px: int = 48,
    margin: float = 1.2,
    depth_tolerance_m: float = 0.05,
) -> Iterator[ObjectGroup]:
    """Yield every object seen closely enough to read, grouped over the views that saw it.

    A view counts if the object centre is inside the image, its range is at most
    ``max_range_m`` (the overhead camera at about 3 m never qualifies), the cut square is at
    least ``min_crop_px`` wide and at least 60 percent inside the image, and the depth frame
    shows the object (not an occluder) at its centre. Identical camera poses (the recorder takes
    several frames per pose) count once.
    """
    root = Path(directory)
    _manifest, rows = load_dataset(root)
    splits_by_scene = {
        row["scenario_id"]: row.get("seed_split") for row in rows if "scenario_id" in row
    }
    collected: dict[tuple[str, str], dict[str, Any]] = {}
    for row in rows:
        split = row.get("seed_split")
        if split not in allowed_splits:
            raise ProtocolError(
                f"{root}: frame {row['frame_id']} is from seed split {split!r}, "
                f"this run may read only {list(allowed_splits)}"
            )
        intrinsics = row["camera"]["intrinsics"]
        pose_key = json.dumps([round(x, 4) for x in row["camera"]["pose_world"]])
        image = depth = None
        for obj in row["objects"]:
            if not obj.get("in_view") or obj["range_m"] > max_range_m:
                continue
            group = collected.setdefault(
                (row["scenario_id"], obj["object_id"]),
                {"row": row, "obj": obj, "views": {}},
            )
            if pose_key in group["views"]:
                continue
            if image is None:
                image = read_image(root / row["files"]["rgb"])
                depth = _read_depth(root / row["files"].get("depth_mm", ""))
            geo = geometry.get(obj["geometry_key"])
            extent = max(2 * geo["radius_m"], geo["height_m"]) if geo else None
            if extent is None:
                continue
            u, v = obj["pixel"]["u"], obj["pixel"]["v"]
            side = intrinsics["fx"] * extent * margin / obj["range_m"]
            if side < min_crop_px or not _depth_agrees(
                depth, u, v, obj["range_m"], depth_tolerance_m
            ):
                continue
            crop = crop_square(
                image, u, v, obj["range_m"], intrinsics["fx"], extent, margin=margin
            )
            if crop is None:
                continue
            variants: list[np.ndarray] = []
            if obj["product_class"] == "box" and str(row.get("viewpoint") or "").startswith(
                "barcode"
            ):
                # A box's back face is flat: unrolling it as a barrel would only bend the bars.
                variants = [crop]
            elif str(row.get("viewpoint") or "").startswith("barcode"):
                # The aimed close-up: the crop as it is, and the barrel unrolled (a code that
                # spans a large share of the circumference is foreshortened towards the edges).
                radius_px = intrinsics["fx"] * geo["radius_m"] / obj["range_m"]
                left = int(max(0, u - 1.6 * radius_px))
                right = int(min(image.shape[1], u + 1.6 * radius_px))
                variants = [
                    unwrap_cylinder(image[:, left:right], radius_px, centre_x=u - left),
                    crop,
                ]
            group["views"][pose_key] = ViewCrop(
                row["frame_id"],
                pose_key,
                float(obj["range_m"]),
                str(row.get("lighting", {}).get("condition", "unknown")),
                crop,
                {
                    "object_pose": obj.get("pose_world"),
                    "camera_pose": row["camera"]["pose_world"],
                    "geometry_key": obj["geometry_key"],
                    "side_px": side,
                    "viewpoint": row.get("viewpoint"),
                    "barcode_variants": variants,
                },
            )
    for (scenario_id, object_id), group in sorted(collected.items()):
        if not group["views"]:
            continue
        row, obj = group["row"], group["obj"]
        yield ObjectGroup(
            seed=int(row["seed"]),
            seed_split=str(splits_by_scene.get(scenario_id)),
            object_id=object_id,
            sku=obj["sku"],
            product_class=obj["product_class"],
            views=tuple(group["views"].values()),
            barcode_images=tuple(
                image
                for v in group["views"].values()
                if str(v.meta.get("viewpoint") or "").startswith("barcode")
                for image in v.meta["barcode_variants"]
            ),
        )


def _read_depth(path: Path) -> np.ndarray | None:
    if not path.is_file():
        return None
    try:
        from PIL import Image  # noqa: PLC0415 - 16-bit PNG needs a real decoder
    except ImportError:
        return None
    with Image.open(path) as image:
        return np.asarray(image, dtype=np.uint16)


# --- caching wrappers (calibration runs embed each crop once, not once per fold) ---------------


def _digest(image: np.ndarray) -> str:
    return hashlib.sha1(
        np.ascontiguousarray(image).tobytes() + str(image.shape).encode()
    ).hexdigest()


class CachedEmbedding:
    """Memoises another backend by crop content; same name and version, so indices accept it."""

    def __init__(self, backend: EmbeddingBackend) -> None:
        self.backend = backend
        self.name, self.version = backend.name, backend.version
        self._memo: dict[str, np.ndarray] = {}

    @property
    def dim(self) -> int:
        return self.backend.dim

    def embed(self, images: Sequence[np.ndarray]) -> np.ndarray:
        keys = [_digest(i) for i in images]
        missing = [(k, i) for k, i in zip(keys, images, strict=True) if k not in self._memo]
        if missing:
            vectors = self.backend.embed([i for _k, i in missing])
            for (key, _i), vector in zip(missing, vectors, strict=True):
                self._memo[key] = vector
        return np.stack([self._memo[k] for k in keys]) if keys else np.zeros((0, self.dim))


class CachedOcr:
    def __init__(self, backend: OcrBackend) -> None:
        self.backend = backend
        self.name, self.version = backend.name, backend.version
        self._memo: dict[str, list[str]] = {}

    def read(self, image: np.ndarray) -> list[str]:
        key = _digest(image)
        if key not in self._memo:
            self._memo[key] = self.backend.read(image)
        return self._memo[key]


# --- whitening artifact ----------------------------------------------------------------------


def save_whitener(path: str | Path, whitener: Whitener) -> None:
    np.savez(path, mean=whitener.mean, components=whitener.components, scale=whitener.scale)


def load_whitener(path: str | Path) -> Whitener:
    with np.load(path, allow_pickle=False) as data:
        return Whitener(data["mean"], data["components"], data["scale"])


# --- registry views ----------------------------------------------------------------------------


def ocr_strings(
    registry: Registry, product_ids: Sequence[str] | None = None
) -> dict[str, list[str]]:
    return {
        r.product_id: list(r.ocr_strings)
        for r in registry.records()
        if product_ids is None or r.product_id in product_ids
    }


def split_product_ids(splits: dict[str, Any], names: Sequence[str]) -> list[str]:
    return sorted({p for name in names for p in splits["products"][name]})


def twin_of(splits: dict[str, Any], sku: str) -> str | None:
    groups = splits["lookalike_groups"]
    mates = [s for s, g in groups.items() if g == groups.get(sku) and s != sku]
    return mates[0] if mates else None


# --- calibration -------------------------------------------------------------------------------


def fold_products(
    skus: Sequence[str], splits: dict[str, Any] | None = None, folds: int = 3
) -> list[list[str]]:
    """Leave-products-out folds that put look-alike twins in different folds.

    The dangerous unknown product is one whose twin is registered, so each look-alike group with
    two or more registered members spreads its members over the first folds (member i goes to fold
    i modulo ``folds - 1``); products with no registered twin form the last fold. Without a split
    document the products are simply dealt out in sorted order.
    """
    ordered = sorted(skus)
    if splits is None:
        return [ordered[i::folds] for i in range(folds)]
    groups = splits["lookalike_groups"]
    by_group: dict[str, list[str]] = defaultdict(list)
    for sku in ordered:
        by_group[groups.get(sku) or sku].append(sku)
    out: list[list[str]] = [[] for _ in range(folds)]
    for members in by_group.values():
        if len(members) == 1:
            out[folds - 1].append(members[0])
            continue
        for position, sku in enumerate(members):
            out[position % (folds - 1)].append(sku)
    return out


def build_samples(
    groups: Sequence[ObjectGroup],
    registry: Registry,
    backend: EmbeddingBackend,
    ocr_backend: OcrBackend | None,
    whitener: Whitener | None,
    splits: dict[str, Any],
    *,
    left_out: Sequence[str],
    config: IdentityConfig,
    cache: DerivedCache | None = None,
    barcode_backend: BarcodeBackend | None = None,
) -> list[Sample]:
    """Observations of every group against an index with ``left_out`` products removed."""
    index = RegistryIndex.from_registry(
        registry, backend, cache, whitener=whitener, exclude=left_out
    )
    engine = IdentityEngine(
        index,
        backend,
        config,
        ocr_backend=ocr_backend,
        ocr_strings=ocr_strings(registry, index.product_ids),
        barcode_backend=barcode_backend,
    )
    samples = []
    for group in groups:
        observation = engine.observe([v.image for v in group.views], group.barcode_images)
        product_id = registry.resolve(group.sku)
        samples.append(
            Sample(
                truth=product_id or group.sku,
                known=product_id in index.product_ids,
                observation=observation,
                seed=group.seed,
                twin=registry.resolve(twin_of(splits, group.sku) or "") or None,
                twin_registered=(
                    product_id not in index.product_ids
                    and (registry.resolve(twin_of(splits, group.sku) or "") or "")
                    in index.product_ids
                ),
            )
        )
    return samples


def calibrate(
    datasets: Sequence[str | Path],
    registry: Registry,
    backend: EmbeddingBackend,
    ocr_backend: OcrBackend | None,
    splits_path: str | Path,
    geometry: dict[str, dict[str, Any]],
    *,
    out: str | Path,
    whitener_out: str | Path,
    bounds_path: str | Path,
    base: IdentityConfig | None = None,
    barcode_backend: BarcodeBackend | None = None,
    whitening_components: int = 64,
    progress: Callable[[str], None] = print,
) -> dict[str, Any]:
    """Fit the numeric thresholds. Every option is fixed in ``base`` beforehand, not searched.

    The defaults of ``IdentityConfig`` are the a-priori options: max scoring, whitening fit on
    unlabeled registry exemplars plus calibration crops, up to five views, an independent channel
    may name a product the embedding ranks first or second.
    """
    base = base or IdentityConfig()
    splits = load_split_file(splits_path)
    allowed = CALIBRATION_SPLITS
    registered = {registry.resolve(s) for s in split_product_ids(splits, allowed)}
    if None in registered or set(registry.product_ids) != registered:
        raise ProtocolError(
            "calibration registry must hold exactly the train and validation products"
        )
    bounds = json.loads(Path(bounds_path).read_text(encoding="utf-8"))
    groups: list[ObjectGroup] = []
    for directory in datasets:
        groups += list(object_groups(directory, geometry, allowed_splits=allowed))
    groups = [
        ObjectGroup(
            g.seed,
            g.seed_split,
            g.object_id,
            g.sku,
            g.product_class,
            g.views[: base.max_views],
            g.barcode_images,
        )
        for g in groups
    ]
    unexpected = sorted({g.sku for g in groups if registry.resolve(g.sku) is None})
    if unexpected:
        raise ProtocolError(f"calibration seeds show unregistered products {unexpected}")
    progress(f"{len(groups)} object groups, {sum(len(g.views) for g in groups)} crops")

    cached = CachedEmbedding(backend)
    cached_ocr = CachedOcr(ocr_backend) if ocr_backend else None
    # Whitening: unlabeled vectors only (registry exemplars and calibration crops; no label read).
    exemplars = np.concatenate(
        [cached.embed(registry.view_images(g)) for g in registry.product_ids]
    )
    crops = cached.embed([v.image for g in groups for v in g.views])
    pool = np.concatenate([exemplars, crops])
    k = min(whitening_components, len(pool) - 1)
    whitener = Whitener.fit(pool, k)
    save_whitener(whitener_out, whitener)

    skus = [a for r in registry.records() for a in r.aliases]
    fold_sets = fold_products(skus, splits)
    samples: list[Sample] = []
    for fold, left_out in enumerate(fold_sets):
        progress(f"fold {fold}: unknown products {left_out}")
        samples += build_samples(
            groups,
            registry,
            cached,
            cached_ocr,
            whitener if base.whiten else None,
            splits,
            left_out=left_out,
            config=base,
            barcode_backend=barcode_backend,
        )
    ceilings = bounds["calibration_ceilings"]
    margin, margins_tried = choose_floor_margin(
        samples,
        max_false_identity_known=ceilings["false_identity_rate_known_max"],
        max_unknown_named=ceilings["unknown_named_as_sku_max"],
        base=base,
    )
    config, record = fit_thresholds(
        samples,
        max_false_identity_known=ceilings["false_identity_rate_known_max"],
        max_unknown_named=ceilings["unknown_named_as_sku_max"],
        base=base,
        floor_margin=margin,
    )
    record["floor_margin"] = margin
    record["floor_margins_tried"] = margins_tried
    record["cross_validation"] = cross_validate(
        samples,
        max_false_identity_known=ceilings["false_identity_rate_known_max"],
        max_unknown_named=ceilings["unknown_named_as_sku_max"],
        base=base,
        floor_margin=margin,
    )
    fit = {
        "tool": "restocker_perception.identity.benchmark.calibrate",
        "backend": f"{backend.name} {backend.version}",
        "ocr": f"{ocr_backend.name} {ocr_backend.version}" if ocr_backend else None,
        "registry_sha256": registry.content_hash(),
        "registry_products": sorted(registry.product_ids),
        "splits_sha256": splits["sha256"],
        "bounds_sha256": hashlib.sha256(Path(bounds_path).read_bytes()).hexdigest(),
        "seed_splits_read": list(allowed),
        "seeds": sorted({g.seed for g in groups}),
        "datasets": [Path(d).name for d in datasets],
        "folds": fold_sets,
        "options_fixed_a_priori": base.to_dict(),
        "whitening": {
            "fingerprint": whitener.fingerprint,
            "components": k,
            "pool_vectors": len(pool),
            "pool_registry_exemplars": len(exemplars),
            "pool_calibration_crops": len(crops),
        },
        "n_groups": len(groups),
        "n_crops": sum(len(g.views) for g in groups),
        "search": record,
    }
    digest = write_calibration(out, config, fit)
    progress(f"calibration {digest}: {config}")
    return {"config": config, "fit": fit, "sha256": digest, "samples": samples}


# --- evaluation (held-out, once) --------------------------------------------------------------


def _bucket(range_m: float) -> str:
    for limit, label in RANGE_BANDS:
        if range_m <= limit:
            return label
    return RANGE_BANDS[-1][1]


def _summarise(rows: Sequence[dict[str, Any]]) -> dict[str, Any]:
    known = [r for r in rows if r["known"]]
    unknown = [r for r in rows if not r["known"]]
    twin_unknown = [r for r in unknown if r["twin_registered"]]

    def count(items, **match):
        return sum(all(r[k] == v for k, v in match.items()) for r in items)

    wrong = sum(1 for r in known if r["status"] == IDENTIFIED and r["sku"] != r["truth"])
    twin = sum(1 for r in known if r["status"] == IDENTIFIED and r["sku"] == r["twin"])
    named_unknown = count(unknown, status=IDENTIFIED)
    named_twin_unknown = count(twin_unknown, status=IDENTIFIED)
    correct = sum(1 for r in known if r["status"] == IDENTIFIED and r["sku"] == r["truth"])
    return {
        "n_known": len(known),
        "n_unknown": len(unknown),
        "n_unknown_twin_registered": len(twin_unknown),
        "known_correct": correct,
        "known_false_identity": wrong,
        "known_twin_confusion": twin,
        "known_abstain": count(known, status=ABSTAIN),
        "known_called_unknown": count(known, status=UNKNOWN),
        "unknown_named_as_sku": named_unknown,
        "unknown_twin_named_as_sku": named_twin_unknown,
        "unknown_called_unknown": count(unknown, status=UNKNOWN),
        "unknown_abstain": count(unknown, status=ABSTAIN),
        "false_identity_rate_known": wrong / len(known) if known else None,
        "false_identity_rate_known_wilson95": wilson(wrong, len(known)),
        "unknown_named_rate": named_unknown / len(unknown) if unknown else None,
        "unknown_named_rate_wilson95": wilson(named_unknown, len(unknown)),
        "unknown_twin_named_rate": named_twin_unknown / len(twin_unknown)
        if twin_unknown
        else None,
        "unknown_twin_named_rate_wilson95": wilson(named_twin_unknown, len(twin_unknown)),
        "correct_rate_known": correct / len(known) if known else None,
        "abstention_rate_known": (
            (count(known, status=ABSTAIN) + count(known, status=UNKNOWN)) / len(known)
            if known
            else None
        ),
        "twin_confusion_rate_known": twin / len(known) if known else None,
        "identified_by": _by_channel(rows),
        "channels": _channels(rows),
    }


def _by_channel(rows: Sequence[dict[str, Any]]) -> dict[str, int]:
    out: dict[str, int] = defaultdict(int)
    for r in rows:
        if r["status"] == IDENTIFIED:
            out[r["reason"].split(" ")[0]] += 1
    return dict(sorted(out.items()))


def _channels(rows: Sequence[dict[str, Any]]) -> dict[str, Any]:
    """Per channel: how often it named a product at all, and how often that was right."""
    out: dict[str, Any] = {}
    for kind in ("embedding", "ocr", "barcode"):
        answered = right = 0
        for r in rows:
            product_ids = [g for k, g in r["evidence"] if k == kind and g]
            if not product_ids or not r["known"]:
                continue
            answered += 1
            right += product_ids[0] == r["truth"]
        known = sum(1 for r in rows if r["known"])
        out[kind] = {
            "answered_on_known": answered,
            "answer_rate_known": answered / known if known else None,
            "correct": right,
            "precision": right / answered if answered else None,
            "precision_wilson95": wilson(right, answered),
        }
    return out


def _upper_pass(
    summary: dict[str, Any], rate: str, bound: float, n_min: int, n: int
) -> dict[str, Any]:
    value, interval = summary[rate], summary[f"{rate}_wilson95"]
    upper = interval[1] if interval else None
    enough = n >= n_min
    if value is None or upper is None:
        status = "inconclusive"
    elif value > bound:
        status = "fail"
    elif upper <= bound and enough:
        status = "pass"
    else:
        status = "inconclusive"
    return {
        "value": value,
        "bound": bound,
        "wilson95": interval,
        "upper_limit": upper,
        "n": n,
        "n_required": n_min,
        "sample_size_met": enough,
        "status": status,
    }


def evaluate_held_out(
    datasets: Sequence[str | Path],
    registry: Registry,
    backend: EmbeddingBackend,
    ocr_backend: OcrBackend | None,
    splits_path: str | Path,
    geometry: dict[str, dict[str, Any]],
    *,
    calibration_path: str | Path,
    whitener_path: str | Path,
    bounds_path: str | Path,
    out: str | Path,
    barcode_backend: BarcodeBackend | None = None,
    progress: Callable[[str], None] = print,
) -> dict[str, Any]:
    """Run once on the held-out split with the calibration as committed; write the result.

    With a version-2 split file the decision is made on the fresh held-out seeds; the burned
    ones (the round-1 held-out) are summarised separately and decide nothing.
    """
    out = Path(out)
    if out.exists():
        raise ProtocolError(f"{out} exists: the held-out run happens once and is not re-tuned")
    splits = load_split_file(splits_path)
    config, fit, calibration_sha = load_calibration(calibration_path)
    whitener = load_whitener(whitener_path)
    if whitener.fingerprint != fit["whitening"]["fingerprint"]:
        raise ProtocolError("whitening file does not match the calibration file")
    if registry.content_hash() != fit["registry_sha256"]:
        raise ProtocolError("registry differs from the one the calibration was fitted against")
    if fit["splits_sha256"] != splits["sha256"]:
        raise ProtocolError("split file differs from the one the calibration was fitted against")
    bounds_bytes = Path(bounds_path).read_bytes()
    if fit["bounds_sha256"] != hashlib.sha256(bounds_bytes).hexdigest():
        raise ProtocolError("bounds file changed after calibration")
    if f"{backend.name} {backend.version}" != fit["backend"]:
        raise ProtocolError("embedding backend differs from calibration")
    bounds_doc = json.loads(bounds_bytes)
    bounds = bounds_doc["bounds"]
    required = bounds_doc.get(
        "required_sample", {"known_objects_min": 0, "unknown_objects_min": 0}
    )
    generations = splits.get("held_out_generations", {})
    burned_seeds = set(generations.get("burned", []))

    groups: list[ObjectGroup] = []
    for directory in datasets:
        groups += list(object_groups(directory, geometry, allowed_splits=("held_out",)))
    progress(f"{len(groups)} held-out object groups")
    index = RegistryIndex.from_registry(
        registry, backend, whitener=whitener if config.whiten else None
    )
    engine = IdentityEngine(
        index,
        backend,
        config,
        ocr_backend=ocr_backend,
        ocr_strings=ocr_strings(registry),
        barcode_backend=barcode_backend,
    )
    # Warm the models so the first decision does not carry model loading into the latency.
    if groups:
        engine.identify([groups[0].views[0].image])

    def run(group: ObjectGroup, views: Sequence[ViewCrop]) -> dict[str, Any]:
        start = time.perf_counter()
        result = engine.identify(
            [v.image for v in views[: config.max_views]],
            [
                image
                for v in views
                if str(v.meta.get("viewpoint") or "").startswith("barcode")
                for image in v.meta["barcode_variants"]
            ],
        )
        wall = (time.perf_counter() - start) * 1000.0
        product_id = registry.resolve(group.sku)
        twin = registry.resolve(twin_of(splits, group.sku) or "")
        return {
            "seed": group.seed,
            "object_id": group.object_id,
            "truth_sku": group.sku,
            "truth": product_id or group.sku,
            "known": product_id is not None,
            "twin": twin,
            "twin_registered": product_id is None and twin in index.product_ids,
            "product_class": group.product_class,
            "n_views": min(len(views), config.max_views),
            "range_band": _bucket(min(v.range_m for v in views)),
            "lighting": views[0].lighting,
            "status": result.status,
            "sku": result.sku,
            "reason": result.reason,
            "evidence": [(e.kind, e.product_id) for e in result.evidence],
            "latency_ms": wall,
        }

    fresh_groups = [g for g in groups if g.seed not in burned_seeds]
    burned_groups = [g for g in groups if g.seed in burned_seeds]
    report: dict[str, Any] = {
        "card": "CMB-112",
        "round": bounds_doc.get("round", 1),
        "calibration_sha256": calibration_sha,
        "bounds_sha256": hashlib.sha256(bounds_bytes).hexdigest(),
        "registry_sha256": registry.content_hash(),
        "splits_sha256": splits["sha256"],
        "backend": f"{backend.name} {backend.version}",
        "ocr": f"{ocr_backend.name} {ocr_backend.version}" if ocr_backend else None,
        "barcode": (
            f"{barcode_backend.name} {barcode_backend.version}" if barcode_backend else None
        ),
        "datasets": [Path(d).name for d in datasets],
        "config": config.to_dict(),
        "bounds": bounds,
        "required_sample": required,
        "decision_unit": "one object with all its close views in the scene (up to max_views)",
        "n_fresh_groups": len(fresh_groups),
        "n_burned_groups": len(burned_groups),
    }
    decisions: dict[str, list[dict[str, Any]]] = {}
    for label, subset in (("fresh", fresh_groups), ("burned", burned_groups)):
        if not subset:
            continue
        multi = [run(g, g.views) for g in subset]
        single = [run(g, (v,)) for g in subset for v in g.views]
        decisions[label] = multi
        block: dict[str, Any] = {}
        for name, rows in (("multi_view", multi), ("single_view", single)):
            latency = [r["latency_ms"] for r in rows]
            summary = _summarise(rows)
            summary["latency_ms_p50"] = percentile(latency, 50)
            summary["latency_ms_p95"] = percentile(latency, 95)
            for key, field_name in (
                ("by_range_band", "range_band"),
                ("by_lighting", "lighting"),
                ("by_product_class", "product_class"),
            ):
                summary[key] = {
                    value: _summarise([r for r in rows if r[field_name] == value])
                    for value in sorted({r[field_name] for r in rows})
                }
            block[name] = summary
        report[label] = block
    if "fresh" in report:
        primary = report["fresh"]["multi_view"]
        report["verdict"] = {
            "false_identity_rate_known": _upper_pass(
                primary,
                "false_identity_rate_known",
                bounds["false_identity_rate_known_max"],
                required["known_objects_min"],
                primary["n_known"],
            ),
            "unknown_named_as_sku": _upper_pass(
                primary,
                "unknown_named_rate",
                bounds["unknown_named_as_sku_max"],
                required["unknown_objects_min"],
                primary["n_unknown"],
            ),
            "latency_p95_ms": {
                "value": primary["latency_ms_p95"],
                "bound": bounds["latency_p95_ms_max"],
                "status": "pass"
                if primary["latency_ms_p95"] <= bounds["latency_p95_ms_max"]
                else "fail",
            },
        }
        report["decisions_fresh"] = decisions["fresh"]
    if "burned" in decisions:
        report["decisions_burned"] = decisions["burned"]
    out.write_text(
        json.dumps(report, indent=2, sort_keys=True, default=float) + "\n", encoding="utf-8"
    )
    progress(json.dumps(report.get("verdict", {}), indent=2, default=float))
    return report


__all__ = [
    "CachedEmbedding",
    "CachedOcr",
    "Metrics",
    "Observation",
    "ObjectGroup",
    "ProtocolError",
    "build_samples",
    "calibrate",
    "decide",
    "evaluate",
    "evaluate_held_out",
    "load_dataset",
    "object_groups",
]

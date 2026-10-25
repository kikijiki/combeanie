# Copyright 2026 Matteo Bernacchia <dev@kikijiki.com>
#
# SPDX-License-Identifier: MIT

"""Score perception predictions against the benchmark labels, per condition, with intervals."""
# Prediction file: JSON Lines, one JSON object per line, ``kind`` selects the row type.
#
#   {"kind": "object", "frame_id": "seed000001-wrist-000",
#    "object_id": "sim:lane_01_can_01",            # optional: omit it and give pose_world and the
#                                                  # row is matched to the nearest label (<= 5 cm)
#    "sku": "GEN-003",                             # a SKU, or null = abstain ("cannot tell"),
#                                                  # or "UNKNOWN" = "not a registered product"
#    "confidence": 0.83,                           # optional, recorded in nothing but your file
#    "pose_world": [x, y, z, qx, qy, qz, qw]}      # optional; scored for pose error when present
#
#   {"kind": "lane", "frame_id": "...", "lane_id": "lane_03",
#    "count": 4,                                   # null = abstain on the count
#    "skus_in_order": ["GEN-003", ...],            # optional, front first
#    "misplaced": false}                           # optional
#
# Outcomes per labelled object that is in view (label ``in_view``):
#   known object (its SKU is in the registry):    correct | false_identity | abstain | missed
#   unknown object (e.g. a held-out product):     rejected | false_identity | missed
# ``false_identity`` is the dangerous error: a confident answer naming a SKU that is not the true
# one. For an unknown object ANY registered SKU is a false identity. ``missed`` means the
# predictor produced no row for the object; it is not an abstention and not a false identity, and
# it is reported separately so a predictor cannot hide failures by saying nothing.
#
# Pose error follows the existing pose_error_evaluator: translation error in metres and the angle
# between the true and estimated symmetry (z) axes in degrees, in [0, 90]; yaw is unobservable on
# a cylinder and is not scored.
#
# Intervals: Wilson 95% for proportions; percentile bootstrap (1000 resamples, pinned generator)
# for means and quantiles. Small groups produce wide intervals on purpose; ``n`` is always given.

from __future__ import annotations

from collections.abc import Iterable, Sequence
import json
import math
from pathlib import Path
from typing import Any

from restocker_gazebo.scenario_random import SplitMix64  # noqa: I100,I101

UNKNOWN_SKU = "UNKNOWN"
MATCH_RADIUS_M = 0.05
BOOTSTRAP_RESAMPLES = 1000
BOOTSTRAP_SEED = 20261019
Z_95 = 1.959963984540054
RANGE_BANDS = ((0.5, "<0.5m"), (1.0, "0.5-1.0m"), (2.0, "1.0-2.0m"), (math.inf, ">2.0m"))
CONDITION_KEYS = (
    "sensor",
    "lighting",
    "known",
    "location",
    "product_class",
    "seed_split",
    "range",
)


class ScoreError(ValueError):
    """A predictions file is malformed (the message names the line)."""


# --------------------------------------------------------------------------------------------
# Statistics
# --------------------------------------------------------------------------------------------


def wilson_interval(successes: int, n: int) -> tuple[float, float] | None:
    """Wilson score interval for a proportion; None when there are no trials."""
    if n <= 0:
        return None
    p = successes / n
    z2 = Z_95 * Z_95
    centre = (p + z2 / (2 * n)) / (1 + z2 / n)
    half = Z_95 * math.sqrt(p * (1 - p) / n + z2 / (4 * n * n)) / (1 + z2 / n)
    return max(0.0, centre - half), min(1.0, centre + half)


def rate(successes: int, n: int) -> dict[str, Any]:
    """A proportion with its Wilson interval (``value`` is None when ``n`` is 0)."""
    interval = wilson_interval(successes, n)
    return {
        "value": None if n == 0 else successes / n,
        "ci95": None if interval is None else [interval[0], interval[1]],
        "k": successes,
        "n": n,
    }


def _quantile(sorted_values: Sequence[float], q: float) -> float:
    position = q * (len(sorted_values) - 1)
    low = math.floor(position)
    high = math.ceil(position)
    return sorted_values[low] + (sorted_values[high] - sorted_values[low]) * (position - low)


def summary(values: Sequence[float]) -> dict[str, Any]:
    """Mean, median and p95 of a sample, with bootstrap intervals on the mean and the median."""
    n = len(values)
    if n == 0:
        return {
            "n": 0,
            "mean": None,
            "mean_ci95": None,
            "median": None,
            "median_ci95": None,
            "p95": None,
        }
    ordered = sorted(values)
    mean = sum(values) / n
    stream = SplitMix64(BOOTSTRAP_SEED + n)
    means, medians = [], []
    for _ in range(BOOTSTRAP_RESAMPLES):
        draw = [values[stream.next_below(n)] for _ in range(n)]
        means.append(sum(draw) / n)
        medians.append(_quantile(sorted(draw), 0.5))
    means.sort()
    medians.sort()
    return {
        "n": n,
        "mean": mean,
        "mean_ci95": [_quantile(means, 0.025), _quantile(means, 0.975)],
        "median": _quantile(ordered, 0.5),
        "median_ci95": [_quantile(medians, 0.025), _quantile(medians, 0.975)],
        "p95": _quantile(ordered, 0.95),
    }


# --------------------------------------------------------------------------------------------
# Reading
# --------------------------------------------------------------------------------------------


def read_predictions(path: str | Path) -> list[dict[str, Any]]:
    """Read and validate a predictions JSONL file."""
    rows: list[dict[str, Any]] = []
    for number, line in enumerate(Path(path).read_text(encoding="utf-8").splitlines(), start=1):
        if not line.strip():
            continue
        try:
            row = json.loads(line)
        except ValueError as error:
            raise ScoreError(f"line {number}: not JSON ({error})") from error
        rows.append(validate_prediction(row, number))
    return rows


def validate_prediction(row: Any, number: int = 0) -> dict[str, Any]:
    """Check one prediction row against the schema; return it unchanged."""
    where = f"line {number}: "
    if not isinstance(row, dict) or row.get("kind") not in {"object", "lane"}:
        raise ScoreError(where + "kind must be 'object' or 'lane'")
    if not isinstance(row.get("frame_id"), str):
        raise ScoreError(where + "frame_id is required")
    if row["kind"] == "object":
        if row.get("object_id") is None and row.get("pose_world") is None:
            raise ScoreError(where + "an object row needs object_id or pose_world to be matched")
        if row.get("sku") is not None and not isinstance(row["sku"], str):
            raise ScoreError(where + "sku must be a string or null")
        pose = row.get("pose_world")
        if pose is not None and (len(pose) != 7 or not all(math.isfinite(v) for v in pose)):
            raise ScoreError(where + "pose_world must be 7 finite numbers [x y z qx qy qz qw]")
    else:
        if not isinstance(row.get("lane_id"), str):
            raise ScoreError(where + "a lane row needs lane_id")
        count = row.get("count")
        if count is not None and (
            not isinstance(count, int) or isinstance(count, bool) or count < 0
        ):
            raise ScoreError(where + "count must be a non-negative integer or null")
    return row


# --------------------------------------------------------------------------------------------
# Matching and per-item outcomes
# --------------------------------------------------------------------------------------------


def _range_band(range_m: float | None) -> str:
    if range_m is None:
        return "unknown"
    for limit, name in RANGE_BANDS:
        if range_m < limit:
            return name
    return RANGE_BANDS[-1][1]


def _distance(a: Sequence[float], b: Sequence[float]) -> float:
    return math.dist(a[:3], b[:3])


def _axis_error_deg(a: Sequence[float], b: Sequence[float]) -> float:
    """Acute angle between the z axes of two quaternions [x y z w], in degrees (0 to 90)."""

    def z_axis(q: Sequence[float]) -> list[float]:
        x, y, z, w = q
        norm = math.sqrt(x * x + y * y + z * z + w * w)
        x, y, z, w = x / norm, y / norm, z / norm, w / norm
        return [2 * (x * z + y * w), 2 * (y * z - x * w), 1 - 2 * (x * x + y * y)]

    za, zb = z_axis(a), z_axis(b)
    cosine = abs(sum(p * q for p, q in zip(za, zb, strict=True)))
    return math.degrees(math.acos(min(1.0, cosine)))


def _match_objects(
    label_objects: list[dict[str, Any]], predictions: list[dict[str, Any]]
) -> tuple[dict[str, dict[str, Any]], int]:
    """Return ({object_id: prediction}, spurious count) for one frame."""
    matched: dict[str, dict[str, Any]] = {}
    anonymous: list[dict[str, Any]] = []
    known_ids = {obj["object_id"] for obj in label_objects}
    spurious = 0
    for row in predictions:
        object_id = row.get("object_id")
        if object_id is None:
            anonymous.append(row)
        elif object_id in known_ids and object_id not in matched:
            matched[object_id] = row
        else:
            spurious += 1
    pairs = sorted(
        (
            (_distance(obj["pose_world"], row["pose_world"]), obj["object_id"], index)
            for obj in label_objects
            if obj["object_id"] not in matched
            for index, row in enumerate(anonymous)
            if row.get("pose_world") is not None
        ),
        key=lambda item: (item[0], item[1], item[2]),
    )
    used: set[int] = set()
    for distance, object_id, index in pairs:
        if distance > MATCH_RADIUS_M or object_id in matched or index in used:
            continue
        matched[object_id] = anonymous[index]
        used.add(index)
    spurious += len(anonymous) - len(used)
    return matched, spurious


def _identity_outcome(truth_sku: str, known: bool, prediction: dict[str, Any] | None) -> str:
    if prediction is None:
        return "missed"
    answer = prediction.get("sku")
    if answer is None or answer == UNKNOWN_SKU or prediction.get("abstain") is True:
        return "abstain" if known else "rejected"
    if known and answer == truth_sku:
        return "correct"
    return "false_identity"


def score_objects_and_lanes(
    labels: Sequence[dict[str, Any]],
    predictions: Sequence[dict[str, Any]],
    registry_skus: Iterable[str],
    lookalike_groups: dict[str, str | None] | None = None,
) -> dict[str, Any]:
    """Reduce labels and predictions to per-item records (no aggregation yet)."""
    registry = set(registry_skus)
    groups = lookalike_groups or {}
    by_frame: dict[str, list[dict[str, Any]]] = {}
    for row in predictions:
        by_frame.setdefault(row["frame_id"], []).append(row)
    label_frames = {label["frame_id"] for label in labels}
    unknown_frames = sorted(set(by_frame) - label_frames)

    object_records: list[dict[str, Any]] = []
    lane_records: list[dict[str, Any]] = []
    spurious = 0
    for label in labels:
        frame_predictions = by_frame.get(label["frame_id"], [])
        in_view = [obj for obj in label["objects"] if obj["in_view"]]
        matched, extra = _match_objects(
            in_view, [p for p in frame_predictions if p["kind"] == "object"]
        )
        spurious += extra
        lighting = (label.get("lighting") or {}).get("condition", "unknown")
        for obj in in_view:
            prediction = matched.get(obj["object_id"])
            known = obj["sku"] in registry
            outcome = _identity_outcome(obj["sku"], known, prediction)
            answer = None if prediction is None else prediction.get("sku")
            record = {
                "frame_id": label["frame_id"],
                "object_id": obj["object_id"],
                "outcome": outcome,
                "twin_confusion": bool(
                    outcome == "false_identity"
                    and answer
                    and groups.get(answer) is not None
                    and groups.get(answer) == groups.get(obj["sku"])
                ),
                "conditions": {
                    "sensor": label["sensor"],
                    "lighting": lighting,
                    "known": "known" if known else "unknown",
                    "location": "lane" if obj["lane_id"] else "tray",
                    "product_class": obj["product_class"],
                    "seed_split": label.get("seed_split") or "none",
                    "range": _range_band(obj.get("range_m")),
                },
                "translation_error_m": None,
                "axis_error_deg": None,
            }
            if prediction is not None and prediction.get("pose_world") is not None:
                record["translation_error_m"] = _distance(
                    obj["pose_world"], prediction["pose_world"]
                )
                record["axis_error_deg"] = _axis_error_deg(
                    obj["pose_world"][3:7], prediction["pose_world"][3:7]
                )
            object_records.append(record)
        lane_predictions = {p["lane_id"]: p for p in frame_predictions if p["kind"] == "lane"}
        for lane in label["lanes"]:
            if not lane["in_view"]:
                continue
            prediction = lane_predictions.get(lane["lane_id"])
            count = None if prediction is None else prediction.get("count")
            lane_records.append(
                {
                    "frame_id": label["frame_id"],
                    "lane_id": lane["lane_id"],
                    "answered": prediction is not None,
                    "abstained": prediction is not None and count is None,
                    "count_error": None if count is None else count - lane["count"],
                    "misplaced_correct": None
                    if prediction is None or prediction.get("misplaced") is None
                    else bool(prediction["misplaced"]) == (lane["misplaced_count"] > 0),
                    "conditions": {
                        "sensor": label["sensor"],
                        "lighting": lighting,
                        "seed_split": label.get("seed_split") or "none",
                    },
                }
            )
    return {
        "objects": object_records,
        "lanes": lane_records,
        "spurious_predictions": spurious,
        "prediction_frames_without_labels": unknown_frames,
    }


# --------------------------------------------------------------------------------------------
# Aggregation
# --------------------------------------------------------------------------------------------


def aggregate_objects(records: Sequence[dict[str, Any]]) -> dict[str, Any]:
    """Identity, abstention, false-identity and pose metrics for one group of objects."""
    n = len(records)
    count = {name: 0 for name in ("correct", "false_identity", "abstain", "rejected", "missed")}
    for record in records:
        count[record["outcome"]] += 1
    known = [r for r in records if r["conditions"]["known"] == "known"]
    unknown = [r for r in records if r["conditions"]["known"] == "unknown"]
    answered = count["correct"] + count["false_identity"]
    return {
        "n_objects": n,
        "n_known": len(known),
        "n_unknown": len(unknown),
        # Of the objects the registry can name, how many were named correctly.
        "identity_accuracy": rate(count["correct"], len(known)),
        # Abstention and unknown-rejection both mean "I did not name a SKU".
        "abstention_rate": rate(count["abstain"] + count["rejected"], n),
        # The dangerous error, over every in-view object (so silence and abstention lower it).
        "false_identity_rate": rate(count["false_identity"], n),
        "false_identity_among_answered": rate(count["false_identity"], answered),
        "false_identity_twin_share": rate(
            sum(1 for r in records if r["twin_confusion"]), count["false_identity"]
        ),
        # Held-out / unregistered objects that were correctly not named.
        "unknown_rejection_rate": rate(
            sum(1 for r in unknown if r["outcome"] == "rejected"), len(unknown)
        ),
        "miss_rate": rate(count["missed"], n),
        "pose": {
            "translation_error_m": summary(
                [r["translation_error_m"] for r in records if r["translation_error_m"] is not None]
            ),
            "axis_error_deg": summary(
                [r["axis_error_deg"] for r in records if r["axis_error_deg"] is not None]
            ),
        },
    }


def aggregate_lanes(records: Sequence[dict[str, Any]]) -> dict[str, Any]:
    """Count error and misplaced-flag accuracy for one group of lane observations."""
    counted = [r for r in records if r["count_error"] is not None]
    misplaced = [r for r in records if r["misplaced_correct"] is not None]
    return {
        "n_lanes": len(records),
        "abstention_rate": rate(sum(1 for r in records if r["abstained"]), len(records)),
        "no_answer_rate": rate(sum(1 for r in records if not r["answered"]), len(records)),
        "count_exact": rate(sum(1 for r in counted if r["count_error"] == 0), len(counted)),
        "count_abs_error": summary([abs(r["count_error"]) for r in counted]),
        "count_bias": summary([float(r["count_error"]) for r in counted]),
        "misplaced_accuracy": rate(
            sum(1 for r in misplaced if r["misplaced_correct"]), len(misplaced)
        ),
    }


def _groups(records: Sequence[dict[str, Any]], keys: Sequence[str]) -> dict[str, list[dict]]:
    grouped: dict[str, list[dict[str, Any]]] = {}
    for record in records:
        for key in keys:
            if key in record["conditions"]:
                grouped.setdefault(f"{key}={record['conditions'][key]}", []).append(record)
    return dict(sorted(grouped.items()))


def score(
    labels: Sequence[dict[str, Any]],
    predictions: Sequence[dict[str, Any]],
    registry_skus: Iterable[str],
    *,
    lookalike_groups: dict[str, str | None] | None = None,
    condition_keys: Sequence[str] = CONDITION_KEYS,
) -> dict[str, Any]:
    """Full report: overall plus one block per condition value (``sensor=wrist`` ...)."""
    items = score_objects_and_lanes(labels, predictions, registry_skus, lookalike_groups)
    return {
        "report_version": 1,
        "n_frames": len(labels),
        "registry_size": len(set(registry_skus)),
        "spurious_predictions": items["spurious_predictions"],
        "prediction_frames_without_labels": items["prediction_frames_without_labels"],
        "overall": {
            "objects": aggregate_objects(items["objects"]),
            "lanes": aggregate_lanes(items["lanes"]),
        },
        "by_condition": {
            name: {"objects": aggregate_objects(group)}
            for name, group in _groups(items["objects"], condition_keys).items()
        },
        "lanes_by_condition": {
            name: aggregate_lanes(group)
            for name, group in _groups(items["lanes"], condition_keys).items()
        },
    }


def format_report(report: dict[str, Any]) -> str:
    """Human-readable summary of :func:`score` output."""

    def pct(item: dict[str, Any]) -> str:
        if item["value"] is None:
            return "n/a (n=0)"
        low, high = item["ci95"]
        return f"{100 * item['value']:5.1f}% [{100 * low:.1f}, {100 * high:.1f}] n={item['n']}"

    def block(title: str, objects: dict[str, Any]) -> list[str]:
        pose = objects["pose"]["translation_error_m"]
        return [
            f"{title}  (objects={objects['n_objects']}, known={objects['n_known']}, "
            f"unknown={objects['n_unknown']})",
            f"  identity accuracy      {pct(objects['identity_accuracy'])}",
            f"  abstention rate        {pct(objects['abstention_rate'])}",
            f"  FALSE IDENTITY rate    {pct(objects['false_identity_rate'])}",
            f"  false id / answered    {pct(objects['false_identity_among_answered'])}",
            f"  unknown rejected       {pct(objects['unknown_rejection_rate'])}",
            f"  missed                 {pct(objects['miss_rate'])}",
            "  position error m       "
            + (
                "n/a"
                if pose["n"] == 0
                else f"mean {pose['mean']:.4f} "
                f"[{pose['mean_ci95'][0]:.4f}, {pose['mean_ci95'][1]:.4f}] n={pose['n']}"
            ),
        ]

    lines = [
        f"frames={report['n_frames']} registry={report['registry_size']} "
        f"spurious={report['spurious_predictions']}"
    ]
    lines += block("overall", report["overall"]["objects"])
    lane = report["overall"]["lanes"]
    count = lane["count_abs_error"]
    lines.append(
        f"lanes={lane['n_lanes']}  count exact {pct(lane['count_exact'])}  "
        + (
            "count |err| n/a"
            if count["n"] == 0
            else f"count |err| mean {count['mean']:.3f} "
            f"[{count['mean_ci95'][0]:.3f}, {count['mean_ci95'][1]:.3f}]"
        )
    )
    for name, entry in report["by_condition"].items():
        lines += block(name, entry["objects"])
    return "\n".join(lines) + "\n"


# --------------------------------------------------------------------------------------------
# Baseline predictors (to test the scorer, and as the floor any real predictor must beat)
# --------------------------------------------------------------------------------------------


def ground_truth_predictions(labels: Sequence[dict[str, Any]]) -> list[dict[str, Any]]:
    """Passthrough: predict exactly the labels (a perfect predictor that never abstains)."""
    rows: list[dict[str, Any]] = []
    for label in labels:
        for obj in label["objects"]:
            if obj["in_view"]:
                rows.append(
                    {
                        "kind": "object",
                        "frame_id": label["frame_id"],
                        "object_id": obj["object_id"],
                        "sku": obj["sku"],
                        "confidence": 1.0,
                        "pose_world": list(obj["pose_world"]),
                    }
                )
        for lane in label["lanes"]:
            if lane["in_view"]:
                rows.append(
                    {
                        "kind": "lane",
                        "frame_id": label["frame_id"],
                        "lane_id": lane["lane_id"],
                        "count": lane["count"],
                        "skus_in_order": list(lane["skus_in_order"]),
                        "misplaced": lane["misplaced_count"] > 0,
                    }
                )
    return rows


def _gauss(stream: SplitMix64) -> float:
    u1 = max(stream.next_unit(), 1e-12)
    return math.sqrt(-2.0 * math.log(u1)) * math.cos(2.0 * math.pi * stream.next_unit())


def noisy_predictions(
    labels: Sequence[dict[str, Any]],
    registry_skus: Sequence[str],
    *,
    seed: int = 1,
    abstain_probability: float = 0.1,
    wrong_probability: float = 0.15,
    drop_probability: float = 0.05,
    position_sigma_m: float = 0.01,
    tilt_sigma_rad: float = 0.05,
    count_error_probability: float = 0.2,
    lookalike_groups: dict[str, str | None] | None = None,
) -> list[dict[str, Any]]:
    """A deliberately imperfect predictor with known error rates, to exercise every metric.

    Wrong answers prefer the look-alike twin when there is one, because that is the confusion
    the benchmark exists to catch. Held-out objects are named as a registered SKU with
    ``wrong_probability``, otherwise rejected, so unknown handling is exercised too.
    """
    stream = SplitMix64(seed)
    registry = sorted(registry_skus)
    groups = lookalike_groups or {}
    rows: list[dict[str, Any]] = []
    for label in labels:
        for obj in label["objects"]:
            if not obj["in_view"]:
                continue
            if stream.next_unit() < drop_probability:
                continue
            pose = list(obj["pose_world"])
            for axis in range(3):
                pose[axis] += position_sigma_m * _gauss(stream)
            pose[3] += tilt_sigma_rad * _gauss(stream) / 2.0
            pose[4] += tilt_sigma_rad * _gauss(stream) / 2.0
            draw = stream.next_unit()
            known = obj["sku"] in registry
            if draw < abstain_probability:
                sku = None
            elif draw < abstain_probability + wrong_probability or not known:
                if known:
                    twins = [
                        s
                        for s in registry
                        if s != obj["sku"]
                        and groups.get(s) is not None
                        and groups.get(s) == groups.get(obj["sku"])
                    ]
                    pool = twins or [s for s in registry if s != obj["sku"]]
                    sku = pool[stream.next_below(len(pool))]
                else:
                    sku = (
                        registry[stream.next_below(len(registry))]
                        if stream.next_unit() < wrong_probability
                        else UNKNOWN_SKU
                    )
            else:
                sku = obj["sku"]
            rows.append(
                {
                    "kind": "object",
                    "frame_id": label["frame_id"],
                    "object_id": obj["object_id"],
                    "sku": sku,
                    "confidence": round(stream.next_unit(), 3),
                    "pose_world": pose,
                }
            )
        for lane in label["lanes"]:
            if not lane["in_view"]:
                continue
            count = lane["count"]
            if stream.next_unit() < count_error_probability:
                count = max(0, count + (1 if stream.next_unit() < 0.5 else -1))
            rows.append(
                {
                    "kind": "lane",
                    "frame_id": label["frame_id"],
                    "lane_id": lane["lane_id"],
                    "count": count,
                }
            )
    return rows


def write_predictions(rows: Iterable[dict[str, Any]], path: str | Path) -> None:
    """Write prediction rows as JSONL."""
    Path(path).write_text(
        "".join(json.dumps(row, sort_keys=True) + "\n" for row in rows), encoding="utf-8"
    )

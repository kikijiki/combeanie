# Copyright 2026 Matteo Bernacchia <dev@kikijiki.com>
#
# SPDX-License-Identifier: MIT

"""Run the identity ladder over perception-benchmark datasets with a rotation oracle."""
# The ladder asks to turn a product; no manipulation skill exists yet, so the simulator is the
# oracle. A *round* is one look at every object still unresolved:
#
#   round 0   the recorded scenes as they are (every tray object, close and barcode views);
#   round k   the same scenes re-recorded with each unresolved object turned in place by the
#             cumulative turn the ladder suggested (tools/record_perception_dataset.py
#             --yaw-offsets ... --relook k), looked at again from fresh camera poses.
#
# ``step`` consumes the datasets of one round, extends every object's state with that look, decides
# in both arms (barcode on, barcode off) from the same measurements, and writes the turns the next
# round needs. The barcode-off arm drives the rounds: whatever the barcode-on arm still has open is
# a subset of it. The decisions are stored per object and per look, so ``report`` can recompute
# every number (coverage by look, re-looks needed, errors) without a model. Nothing here reads a
# seed the split file does not allow for the run (``--allow``), and a report is written once.

from __future__ import annotations

from collections import Counter, defaultdict
from collections.abc import Sequence
import hashlib
import json
import math
from pathlib import Path
import pickle
from typing import Any

import numpy as np

from restocker_perception.identity.benchmark import (
    ObjectGroup,
    ProtocolError,
    object_groups,
    percentile,
    wilson,
)
from restocker_perception.identity.diagnose import relative_azimuth_deg, tilt_deg
from restocker_perception.identity.ladder import (
    IDENTIFIED,
    NEEDS_HUMAN,
    NOT_YET,
    RUNG_TEXT_ESCALATED,
    UNKNOWN,
    LadderEngine,
    LookCrop,
    LookRecord,
    ObjectState,
)
from restocker_perception.registry.model import Registry

# The arms. ``off`` is the headline: the barcode channel is disabled AND the aimed barcode close-up
# views are not shown to the ladder (a product with a barcode would otherwise get views a product
# without one never gets: round 3's arm, kept here as ``off_views``); ``on`` has the barcode
# channel and every view. ``off`` and ``off_views`` differ only in the views.
ARMS = ("off", "off_views", "on")
# The state of observations each arm decides from: ``strict`` has no aimed barcode view.
STATE_OF = {"off": "strict", "off_views": "all", "on": "all"}
TRAY_PREFIX = "sim:stock_"


def _look_input(group: ObjectGroup, geometry: dict[str, dict[str, Any]]) -> dict[str, Any]:
    """A group's crops as the engine takes them, plus the ground truth the report needs."""
    crops: list[LookCrop] = []
    facing: list[float] = []
    barcode_mask: list[bool] = []
    for view in group.views:
        meta = view.meta
        geo = geometry[meta["geometry_key"]]
        extent = max(2 * geo["radius_m"], geo["height_m"])
        half_width = meta["side_px"] / 1.2 * geo["radius_m"] / extent
        pose, cam = meta["object_pose"], meta["camera_pose"]
        azimuth = math.degrees(math.atan2(cam[1] - pose[1], cam[0] - pose[0])) % 360.0
        crops.append(LookCrop(view.image, half_width, azimuth))
        facing.append(relative_azimuth_deg(pose, cam))
        barcode_mask.append(str(meta.get("viewpoint") or "").startswith("barcode"))
    pose = group.views[0].meta["object_pose"]
    return {
        "crops": crops,
        # True for the crops that come from the aimed barcode viewpoints.
        "barcode_view_mask": barcode_mask,
        "barcode_images": list(group.barcode_images),
        "truth": {
            "sku": group.sku,
            "product_class": group.product_class,
            "lighting": group.views[0].lighting,
            "tilt_deg": tilt_deg(pose),
            "seed": group.seed,
            "object_id": group.object_id,
        },
        # Ground truth of the camera's direction in the product frame, per crop (for the report
        # only: the ladder never sees it).
        "true_relative_azimuth_deg": facing,
        "range_m": min(v.range_m for v in group.views),
    }


def round_inputs(
    datasets: Sequence[str | Path],
    geometry: dict[str, dict[str, Any]],
    *,
    allowed_splits: Sequence[str],
    only: set[tuple[int, str]] | None = None,
) -> dict[tuple[int, str], dict[str, Any]]:
    """One round's looks, by (seed, object id); tray objects only, optionally only ``only``."""
    out: dict[tuple[int, str], dict[str, Any]] = {}
    for directory in datasets:
        for group in object_groups(directory, geometry, allowed_splits=allowed_splits):
            key = (group.seed, group.object_id)
            if not group.object_id.startswith(TRAY_PREFIX):
                continue
            if only is not None and key not in only:
                continue
            if key in out:
                raise ProtocolError(f"{key} appears in two datasets of the same round")
            out[key] = _look_input(group, geometry)
    return out


class LadderRun:
    """The per-object state of a benchmark across rounds (pickled between rounds)."""

    def __init__(self, registry: Registry) -> None:
        self.registry_sha256 = registry.content_hash()
        self.objects: dict[tuple[int, str], dict[str, Any]] = {}
        self.rounds_done = -1
        self.meta: dict[str, Any] = {}

    def save(self, path: str | Path) -> None:
        with open(path, "wb") as stream:
            pickle.dump(self, stream)

    @staticmethod
    def load(path: str | Path) -> LadderRun:
        with open(path, "rb") as stream:
            return pickle.load(stream)  # noqa: S301 - our own file


def step(
    run: LadderRun,
    engine: LadderEngine,
    inputs: dict[tuple[int, str], dict[str, Any]],
    *,
    round_index: int,
    guided: bool = True,
    progress=print,
) -> dict[str, dict[str, float]]:
    """Add look ``round_index`` to every object in ``inputs``; return the turns for the next round.

    Round 0 creates the objects. A later round may only name objects some arm left open. The
    returned ``{seed: {object_id: cumulative degrees}}`` is the next round's ``--yaw-offsets``
    file; the turn is the guidance of the first arm still open (``off``, the headline, first).
    Every look is measured on all the crops (the ``off_views`` and ``on`` arms) and, when the look
    has aimed barcode views, once more on the others only (the ``off`` arm).
    """
    if round_index != run.rounds_done + 1:
        raise ProtocolError(f"round {round_index} after round {run.rounds_done}")
    offsets: dict[str, dict[str, float]] = {}
    # An open object that the round's datasets do not show (a view the planner refused) still
    # spends its look: the turn was made, nothing was seen, the ladder asks again.
    unseen = (
        sorted(k for k, e in run.objects.items() if not e["closed"] and k not in inputs)
        if round_index > 0
        else []
    )
    work = sorted(inputs.items()) + [(k, None) for k in unseen]
    for count, (key, data) in enumerate(work, start=1):
        if round_index == 0:
            entry = run.objects.setdefault(
                key,
                {
                    "truth": data["truth"],
                    "state": ObjectState(),
                    "state_strict": ObjectState(),
                    "decisions": {arm: [] for arm in ARMS},
                    "turn_deg": 0.0,
                    "closed": False,
                    "oracle": [],
                    # The looks each arm sent to the heavy reader (the escalation rung).
                    "escalated": {arm: set() for arm in ARMS},
                },
            )
        else:
            entry = run.objects.get(key)
            if entry is None or entry["closed"]:
                raise ProtocolError(f"{key} is not open for round {round_index}")
        if data is None:
            empty = LookRecord(
                np.zeros((0, engine.embedding.dim), dtype=np.float32), [], None, [], [], {}
            )
            data = {
                "crops": [],
                "barcode_view_mask": [],
                "range_m": entry.get("last_range_m"),
                "true_relative_azimuth_deg": [None],
            }
            look_all = look_strict = empty
        else:
            look_all = engine.observe(data["crops"], data["barcode_images"])
            mask = data.get("barcode_view_mask") or [False] * len(data["crops"])
            strict_crops = [c for c, aimed in zip(data["crops"], mask, strict=True) if not aimed]
            look_strict = (
                look_all
                if len(strict_crops) == len(data["crops"])
                else engine.observe(strict_crops, ())
            )
        mask = data.get("barcode_view_mask") or [False] * len(data["crops"])
        crops_of = {
            "off": [c for c, aimed in zip(data["crops"], mask, strict=True) if not aimed],
            "off_views": data["crops"],
            "on": data["crops"],
        }
        first_truth = {
            "off": next((i for i, aimed in enumerate(mask) if not aimed), 0),
            "off_views": 0,
            "on": 0,
        }
        wanted = {arm: round_index == 0 or _open(entry, arm) for arm in ARMS}
        if wanted["off_views"] or wanted["on"]:
            entry["state"].looks.append(look_all)
        if wanted["off"]:
            entry["state_strict"].looks.append(look_strict)
        entry["last_range_m"] = data["range_m"]
        for arm in ARMS:
            if not wanted[arm]:
                entry["decisions"][arm].append(entry["decisions"][arm][-1])
                continue
            state = entry["state_strict"] if arm == "off" else entry["state"]
            result = engine.decide_escalating(
                state,
                crops_of[arm],
                escalated=entry["escalated"][arm],
                barcode=arm == "on",
                guided=guided,
            )
            decision = result.to_dict() | {"round": round_index}
            guidance = decision.get("guidance")
            truths = data["true_relative_azimuth_deg"]
            if guidance is not None and guidance.get("true_relative_azimuth_deg") is None:
                guidance["true_relative_azimuth_deg"] = (
                    truths[first_truth[arm]] if first_truth[arm] < len(truths) else None
                )
            entry["decisions"][arm].append(decision)
        entry["oracle"].append(
            {
                "round": round_index,
                "turn_before_deg": entry["turn_deg"],
                "n_crops": len(data["crops"]),
                "true_relative_azimuth_deg": data["true_relative_azimuth_deg"],
            }
        )
        driving = next((a for a in ARMS if _open(entry, a)), None)
        if driving is not None:
            # A secondary arm can stay open with no guidance (nothing to aim by): no turn then.
            guidance = entry["decisions"][driving][-1]["guidance"] or {}
            turn = float(guidance.get("suggested_turn_deg") or 0.0)
            entry["turn_deg"] = (entry["turn_deg"] + turn) % 360.0
            offsets.setdefault(str(key[0]), {})[key[1]] = entry["turn_deg"]
        else:
            entry["closed"] = True
        if count % 25 == 0:
            progress(f"round {round_index}: {count}/{len(inputs)} objects")
    run.rounds_done = round_index
    return offsets


def _state(entry: dict[str, Any], arm: str) -> ObjectState:
    """The observations an arm decided from (older states had one for every arm)."""
    if STATE_OF[arm] == "strict":
        return entry.get("state_strict", entry["state"])
    return entry["state"]


def _open(entry: dict[str, Any], arm: str) -> bool:
    decisions = entry["decisions"][arm]
    return not decisions or decisions[-1]["status"] == NOT_YET


# --- report --------------------------------------------------------------------------------


def _final(entry: dict[str, Any], arm: str, budget: int) -> dict[str, Any]:
    """The decision the ladder ends on after at most ``budget`` re-looks."""
    decisions = entry["decisions"][arm]
    for decision in decisions[: budget + 1]:
        if decision["status"] in (IDENTIFIED, UNKNOWN):
            return decision
    last = decisions[min(budget, len(decisions) - 1)]
    if last["status"] == NOT_YET and len(decisions) - 1 >= budget:
        return last | {"status": NEEDS_HUMAN}
    return last


def summarise(
    run: LadderRun, registry: Registry, *, arm: str, budget: int, seeds: set[int] | None = None
) -> dict[str, Any]:
    """Every number of one arm after at most ``budget`` re-looks."""
    rows = []
    for (seed, _object_id), entry in sorted(run.objects.items()):
        if seeds is not None and seed not in seeds:
            continue
        truth = entry["truth"]
        product = registry.resolve(truth["sku"])
        final = _final(entry, arm, budget)
        rows.append((entry, truth, product, final))
    known = [r for r in rows if r[2] is not None]
    unknown = [r for r in rows if r[2] is None]
    named = [r for r in known if r[3]["status"] == IDENTIFIED]
    wrong = [r for r in named if r[3]["sku"] != r[2]]
    correct = [r for r in named if r[3]["sku"] == r[2]]
    unknown_named = [r for r in unknown if r[3]["status"] == IDENTIFIED]
    relooks = [r[3]["look"] for r in correct]
    wrong_late = [r for r in wrong if r[3]["look"] >= 1]
    status = Counter(r[3]["status"] for r in known)
    by_look = []
    for k in range(budget + 1):
        ids = [r for r in known if r[3]["status"] == IDENTIFIED and r[3]["look"] == k]
        by_look.append(
            {
                "look": k,
                "named": len(ids),
                "correct": sum(1 for r in ids if r[3]["sku"] == r[2]),
                "mean_text_score": _mean([r[3]["text"]["best_score"] for r in ids]),
                "mean_text_margin": _mean([r[3]["text"]["margin"] for r in ids]),
                "by_rung": dict(Counter(r[3]["rung"] for r in ids)),
            }
        )
    latency = [
        sum(v for k, v in _per_look_latency(entry, arm, final["look"]).items())
        for entry, _t, _p, final in rows
    ]
    twin = sum(1 for r in wrong if _twin(r[1]["sku"], r[3]["sku"], registry))
    heavy = [
        _state(entry, arm).looks[i].escalation_ms
        for entry, *_rest in rows
        for i in sorted(entry.get("escalated", {}).get(arm, ()))
        if i <= budget and i < len(_state(entry, arm).looks)
    ]
    named_heavy = [r for r in named if r[3].get("escalated")]
    return {
        "escalated_reads": len(heavy),
        "escalated_objects": sum(
            1
            for entry, *_rest in rows
            if any(i <= budget for i in entry.get("escalated", {}).get(arm, ()))
        ),
        "named_after_heavy_read": len(named_heavy),
        "named_by_heavy_read_only": sum(1 for r in named if r[3]["rung"] == RUNG_TEXT_ESCALATED),
        "wrong_after_heavy_read": sum(1 for r in named_heavy if r[3]["sku"] != r[2]),
        "latency_ms_per_escalated_read_mean": _mean(heavy),
        "latency_ms_per_escalated_read_p50": percentile(heavy, 50) if heavy else None,
        "latency_ms_per_escalated_read_p95": percentile(heavy, 95) if heavy else None,
        "arm": arm,
        "budget_relooks": budget,
        "n_objects": len(rows),
        "n_known": len(known),
        "n_unknown": len(unknown),
        "named": len(named),
        "correct": len(correct),
        "false_identity": len(wrong),
        "false_identity_rate_named": len(wrong) / len(named) if named else None,
        "false_identity_rate_named_wilson95": wilson(len(wrong), len(named)),
        "false_identity_rate_known": len(wrong) / len(known) if known else None,
        "coverage_known": len(named) / len(known) if known else None,
        "coverage_known_wilson95": wilson(len(named), len(known)),
        "correct_rate_known": len(correct) / len(known) if known else None,
        "status_known": dict(status),
        "needs_human_rate_known": status[NEEDS_HUMAN] / len(known) if known else None,
        "unknown_named": len(unknown_named),
        "unknown_named_rate": len(unknown_named) / len(unknown) if unknown else None,
        "unknown_named_wilson95": wilson(len(unknown_named), len(unknown)),
        "unknown_refused_by": dict(Counter(r[3]["status"] for r in unknown)),
        "relooks_mean_named": _mean(relooks),
        "relooks_p95_named": percentile(relooks, 95),
        "relooks_mean_all_known": _mean(
            [min(r[3]["look"], budget) for r in known if r[3]["status"] != UNKNOWN]
        ),
        "wrong_after_escalation": len(wrong_late),
        "wrong_after_escalation_of_named_after_relook": len(wrong_late)
        / max(1, sum(1 for r in named if r[3]["look"] >= 1)),
        "twin_confusions": twin,
        "by_look": by_look,
        "latency_ms_per_decision_p50": percentile(latency, 50),
        "latency_ms_per_decision_p95": percentile(latency, 95),
        "latency_ms_per_look_p95": percentile(_look_latencies(rows, arm), 95),
        "by_product_class": {
            cls: _small(
                [r for r in known if r[1]["product_class"] == cls],
                [r for r in unknown if r[1]["product_class"] == cls],
            )
            for cls in sorted({r[1]["product_class"] for r in rows})
        },
        "by_posture": {
            posture: _small(
                [r for r in known if _posture(r[1]) == posture],
                [r for r in unknown if _posture(r[1]) == posture],
            )
            for posture in ("upright", "lying")
        },
        "by_lighting": {
            light: _small([r for r in known if r[1]["lighting"] == light], [])
            for light in sorted({r[1]["lighting"] for r in known})
        },
        "wrong": [
            {
                "seed": r[1]["seed"],
                "object": r[1]["object_id"],
                "truth": r[2],
                "named": r[3]["sku"],
                "look": r[3]["look"],
                "rung": r[3]["rung"],
                "text": r[3]["text"],
            }
            for r in wrong
        ],
        "unknown_named_cases": [
            {
                "seed": r[1]["seed"],
                "object": r[1]["object_id"],
                "truth": r[1]["sku"],
                "named": r[3]["sku"],
                "look": r[3]["look"],
                "rung": r[3]["rung"],
                "text": r[3]["text"],
            }
            for r in unknown_named
        ],
    }


def _posture(truth: dict[str, Any]) -> str:
    """A product tilted more than 45 degrees from the vertical lies on a side."""
    return "lying" if truth.get("tilt_deg", 0.0) > 45.0 else "upright"


def _small(known: list, unknown: list) -> dict[str, Any]:
    named = [r for r in known if r[3]["status"] == IDENTIFIED]
    return {
        "n_known": len(known),
        "named": len(named),
        "false_identity": sum(1 for r in named if r[3]["sku"] != r[2]),
        "n_unknown": len(unknown),
        "unknown_named": sum(1 for r in unknown if r[3]["status"] == IDENTIFIED),
    }


def _mean(values: Sequence[float]) -> float | None:
    values = [v for v in values if v is not None]
    return float(np.mean(values)) if values else None


def _twin(truth_sku: str, named: str, registry: Registry) -> bool:
    a, b = registry.resolve(truth_sku), named
    if a is None or b is None:
        return False
    names = (registry.get(a).display_name.split()[0], registry.get(b).display_name.split()[0])
    return names[0] == names[1]


def _per_look_latency(entry: dict[str, Any], arm: str, look: int) -> dict[str, float]:
    """Wall time of the looks up to ``look`` (the decision at that look sums them): each look is
    measured once, embedded, read and (arm on) decoded."""
    out: dict[str, float] = defaultdict(float)
    for record in _state(entry, arm).looks[: look + 1]:
        for key, value in record.latency_ms.items():
            if key == "barcode" and arm != "on":
                continue
            out[key] += value
    return out


def _look_latencies(rows: list, arm: str) -> list[float]:
    values = []
    for entry, *_rest in rows:
        for record in _state(entry, arm).looks:
            values.append(
                sum(
                    v for k, v in record.latency_ms.items() if not (k == "barcode" and arm != "on")
                )
            )
    return values


def facing_report(run: LadderRun) -> dict[str, Any]:
    """How good the guidance is, against the simulator's ground truth (never seen by the ladder).

    The estimate is in the registry's azimuth frame and the truth in the world's: they differ by a
    fixed convention offset, estimated here as the circular mean of the difference and removed
    before the residuals are reported.
    """
    estimates, truths, boxes = [], [], []
    for entry in run.objects.values():
        for decision in entry["decisions"]["off"]:
            guidance = decision.get("guidance")
            if not guidance or guidance.get("estimated_facing_deg") is None:
                continue
            truth = guidance.get("true_relative_azimuth_deg")
            if truth is None:
                continue
            estimates.append(guidance["estimated_facing_deg"])
            truths.append(truth)
            boxes.append(guidance["shape"] == "box")
    out: dict[str, Any] = {"n": len(estimates)}
    for shape in ("round", "box"):
        idx = [i for i, b in enumerate(boxes) if (b if shape == "box" else not b)]
        if not idx:
            continue
        diff = np.radians(np.array(estimates)[idx] - np.array(truths)[idx])
        offset = math.degrees(math.atan2(np.sin(diff).mean(), np.cos(diff).mean()))
        resid = np.abs((np.degrees(diff) - offset + 180) % 360 - 180)
        out[shape] = {
            "n": len(idx),
            "convention_offset_deg": offset,
            "median_abs_error_deg": float(np.median(resid)),
            "within_30": float(np.mean(resid <= 30)),
            "within_45": float(np.mean(resid <= 45)),
        }
    return out


def verdict(summary: dict[str, Any], bounds: dict[str, Any]) -> dict[str, Any]:
    """Apply a pre-declared bounds file's pass rule to one arm's summary."""
    out: dict[str, Any] = {}
    rules = bounds["bounds"]

    def upper_rule(name: str, value, interval, n: int, n_min: int, bound: float) -> None:
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
        out[name] = {
            "value": value,
            "bound": bound,
            "upper_limit": upper,
            "n": n,
            "n_required": n_min,
            "status": status,
        }

    sample = bounds["required_sample"]
    upper_rule(
        "false_identity_rate_named",
        summary["false_identity_rate_named"],
        summary["false_identity_rate_named_wilson95"],
        summary["named"],
        sample["named_min"],
        rules["false_identity_rate_named_max"],
    )
    upper_rule(
        "unknown_named_rate",
        summary["unknown_named_rate"],
        summary["unknown_named_wilson95"],
        summary["n_unknown"],
        sample["unknown_objects_min"],
        rules["unknown_named_as_sku_max"],
    )
    # Lower-bounded rate: coverage must reach its floor with the lower Wilson limit.
    interval = summary["coverage_known_wilson95"]
    floor = rules["coverage_known_min"][summary["arm"]]
    value = summary["coverage_known"]
    enough = summary["n_known"] >= sample["known_objects_min"]
    out["coverage_known"] = {
        "value": value,
        "bound": floor,
        "lower_limit": interval[0] if interval else None,
        "n": summary["n_known"],
        "n_required": sample["known_objects_min"],
        "status": (
            "inconclusive"
            if value is None
            else "fail"
            if value < floor
            else "pass"
            if interval[0] >= floor and enough
            else "inconclusive"
        ),
    }
    for name, key, bound_key in (
        ("relooks_mean_named", "relooks_mean_named", "relooks_mean_max"),
        ("relooks_p95_named", "relooks_p95_named", "relooks_p95_max"),
        ("latency_p95_ms_per_look", "latency_ms_per_look_p95", "latency_p95_ms_per_look_max"),
        ("wrong_after_escalation", "wrong_after_escalation", "wrong_after_escalation_max"),
    ):
        value = summary[key]
        bound = rules[bound_key]
        out[name] = {
            "value": value,
            "bound": bound,
            "status": "pass" if value is not None and value <= bound else "fail",
        }
    # Round 4 bounds on the heavy reader, when the bounds file declares them.
    for name, key, bound_key in (
        (
            "latency_p95_ms_per_escalated_read",
            "latency_ms_per_escalated_read_p95",
            "latency_p95_ms_per_escalated_read_max",
        ),
        ("wrong_after_heavy_read", "wrong_after_heavy_read", "wrong_after_heavy_read_max"),
    ):
        if bound_key not in rules:
            continue
        value = summary.get(key)
        bound = rules[bound_key]
        out[name] = {
            "value": value,
            "bound": bound,
            "status": "pass" if value is not None and value <= bound else "fail",
        }
        if value is None:  # nothing was escalated: nothing to bound, the rung did not run
            out[name]["status"] = "not_exercised"
    return out


def file_sha256(path: str | Path) -> str:
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def dump_report(path: str | Path, report: dict[str, Any]) -> None:
    Path(path).write_text(
        json.dumps(report, indent=2, sort_keys=True, default=float) + "\n", encoding="utf-8"
    )

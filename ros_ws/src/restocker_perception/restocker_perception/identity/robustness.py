# Copyright 2026 Matteo Bernacchia <dev@kikijiki.com>
#
# SPDX-License-Identifier: MIT

"""Pre-declared bounds for the perception robustness campaign (CMB-116): tables and verdicts."""
# One *condition* is one ladder state (a set of seeds recorded and read under one stress, see the
# bounds file). ``object_table`` turns a state into one row per tray object of the scene, taking
# the denominators from the scene's ground truth, not from the objects a look happened to reach:
# an object with no usable crop at look 0 is a row with status ``unseen`` and counts as not named.
# ``evaluate`` applies the bounds file to the tables. Nothing here changes what the ladder
# decides; it only counts. The pass rule is the one of the earlier rounds: an upper bound passes
# only if the upper Wilson limit is at or under it and the sample is met, fails if the point
# estimate is over it, and is otherwise inconclusive; a lower bound mirrors that on the lower
# limit.

from __future__ import annotations

from collections.abc import Iterable, Mapping, Sequence
import json
from pathlib import Path
from typing import Any

from restocker_perception.identity import ladder_benchmark as lb
from restocker_perception.identity.benchmark import wilson
from restocker_perception.identity.ladder import IDENTIFIED
from restocker_perception.registry.model import Registry

UNSEEN = "unseen"
BOX = "box"


def shape_of(product_class: str) -> str:
    return "box" if product_class == BOX else "bottle" if "bottle" in product_class else "can"


def truth_objects(
    datasets: Iterable[str | Path], *, seeds: set[int] | None = None
) -> dict[tuple[int, str], dict[str, Any]]:
    """Every tray object of the scenes the datasets record: (seed, object id) -> truth."""
    out: dict[tuple[int, str], dict[str, Any]] = {}
    for directory in datasets:
        for line in (Path(directory) / "labels.jsonl").read_text(encoding="utf-8").splitlines():
            row = json.loads(line)
            seed = row.get("seed")
            if seed is None or (seeds is not None and seed not in seeds):
                continue
            for obj in row["objects"]:
                if obj["lane_id"] is None and obj["object_id"].startswith(lb.TRAY_PREFIX):
                    out.setdefault(
                        (seed, obj["object_id"]),
                        {"sku": obj["sku"], "product_class": obj["product_class"]},
                    )
    return out


def twin_registered(sku: str, registry: Registry, groups: Mapping[str, str | None]) -> bool:
    """True when another product of this SKU's look-alike group is in the registry."""
    group = groups.get(sku)
    if group is None:
        return False
    return any(
        other != sku and groups.get(other) == group and registry.resolve(other) is not None
        for other in groups
    )


def object_table(
    run: lb.LadderRun,
    registry: Registry,
    truth: Mapping[tuple[int, str], Mapping[str, Any]],
    groups: Mapping[str, str | None],
    *,
    arm: str,
    budget: int,
) -> list[dict[str, Any]]:
    """One row per tray object of ``truth``: what the ladder ended on after ``budget`` re-looks."""
    rows = []
    for key, info in sorted(truth.items()):
        product = registry.resolve(info["sku"])
        row: dict[str, Any] = {
            "seed": key[0],
            "object": key[1],
            "sku": info["sku"],
            "product_class": info["product_class"],
            "shape": shape_of(info["product_class"]),
            "known": product is not None,
            "twin_registered": twin_registered(info["sku"], registry, groups),
        }
        entry = run.objects.get(key)
        if entry is None:
            row.update(status=UNSEEN, named=None, look=None, rung=None)
        else:
            final = lb._final(entry, arm, budget)
            row.update(
                status=final["status"],
                named=final["sku"] if final["status"] == IDENTIFIED else None,
                look=final["look"],
                rung=final["rung"],
            )
        row["correct"] = bool(
            row["known"] and row["status"] == IDENTIFIED and row["named"] == product
        )
        row["wrong"] = bool(row["known"] and row["status"] == IDENTIFIED and not row["correct"])
        row["unknown_named"] = bool(not row["known"] and row["status"] == IDENTIFIED)
        rows.append(row)
    return rows


def _select(rows: Sequence[Mapping[str, Any]], group: Mapping[str, Any] | None):
    if not group:
        return list(rows)
    return [r for r in rows if all(r.get(k) == v for k, v in group.items())]


def metric(rows: Sequence[Mapping[str, Any]], name: str) -> tuple[int, int]:
    """(successes, n) of a named rate over table rows."""
    known = [r for r in rows if r["known"]]
    unknown = [r for r in rows if not r["known"]]
    named = [r for r in known if r["status"] == IDENTIFIED]
    if name == "false_identity_named":
        return sum(r["wrong"] for r in named), len(named)
    if name == "coverage_known":
        return sum(r["correct"] for r in known), len(known)
    if name == "seen_known":
        return sum(r["status"] != UNSEEN for r in known), len(known)
    if name == "unknown_named":
        return sum(r["unknown_named"] for r in unknown), len(unknown)
    raise ValueError(f"unknown metric {name!r}")


def judge(direction: str, k: int, n: int, bound: float, n_required: int) -> dict[str, Any]:
    interval = wilson(k, n)
    value = k / n if n else None
    out: dict[str, Any] = {
        "k": k,
        "n": n,
        "n_required": n_required,
        "value": value,
        "bound": bound,
        "direction": direction,
        "wilson95": interval,
    }
    if value is None:
        out["status"] = "inconclusive"
    elif direction == "max":
        out["status"] = (
            "fail"
            if value > bound
            else "pass"
            if interval[1] <= bound and n >= n_required
            else "inconclusive"
        )
    else:
        out["status"] = (
            "fail"
            if value < bound
            else "pass"
            if interval[0] >= bound and n >= n_required
            else "inconclusive"
        )
    return out


def evaluate(
    bounds: Mapping[str, Any], tables: Mapping[str, Mapping[int, list[dict[str, Any]]]]
) -> list[dict[str, Any]]:
    """Apply every bound of the file. ``tables[condition][budget]`` is an ``object_table``.

    A bound names ``conditions`` (a list; ``"all_stress"`` expands to the file's stress conditions
    and pools their rows), a ``budget``, a ``metric``, optionally a ``group`` (row filter), a
    ``direction`` (max or min), a ``value`` and ``n_required``.
    """
    stress = list(bounds.get("stress_conditions", []))
    out = []
    for bound in bounds["bounds"]:
        names = stress if bound["conditions"] == "all_stress" else list(bound["conditions"])
        missing = [n for n in names if n not in tables or bound["budget"] not in tables[n]]
        if missing:
            out.append({"id": bound["id"], "status": "inconclusive", "missing": missing})
            continue
        rows = [r for n in names for r in tables[n][bound["budget"]]]
        k, n = metric(_select(rows, bound.get("group")), bound["metric"])
        out.append(
            {"id": bound["id"], "conditions": names, "metric": bound["metric"]}
            | judge(bound["direction"], k, n, bound["value"], bound["n_required"])
        )
    return out

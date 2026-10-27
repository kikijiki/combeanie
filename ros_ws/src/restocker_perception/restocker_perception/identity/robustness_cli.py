# Copyright 2026 Matteo Bernacchia <dev@kikijiki.com>
#
# SPDX-License-Identifier: MIT

"""CMB-116 tables and verdict: ``tables`` from ladder states, ``verdict`` from a bounds file."""

from __future__ import annotations

import argparse
import json
from pathlib import Path
import sys

from restocker_perception.identity import ladder_benchmark as lb
from restocker_perception.identity import robustness
from restocker_perception.registry.model import Registry


def _tables(args: argparse.Namespace) -> int:
    from restocker_benchmarks import perception_splits as sp  # noqa: PLC0415

    registry = Registry.load(args.registry)
    document = sp.load_splits(args.splits)
    groups = document["lookalike_groups"]
    budgets = [int(b) for b in args.budgets.split(",")]
    tables: dict[str, dict[str, list]] = {}
    for item in args.condition:
        # NAME=STATE[,STATE...]@LISTFILE: the states of one condition and the file naming the
        # look-0 datasets whose scenes are its denominator.
        name, _, rest = item.partition("=")
        states, _, listing = rest.partition("@")
        run = None
        for path in states.split(","):
            part = lb.LadderRun.load(path)
            if run is None:
                run = part
            elif set(part.objects) & set(run.objects):
                raise lb.ProtocolError(f"{name}: two states hold the same object")
            else:
                run.objects.update(part.objects)
        if run.registry_sha256 != registry.content_hash():
            raise lb.ProtocolError(f"{name}: registry differs from the state's")
        datasets = [line for line in Path(listing).read_text().split() if line]
        seeds = {k[0] for k in run.objects}
        truth = robustness.truth_objects(datasets, seeds=None if args.all_scene_seeds else seeds)
        tables[name] = {
            str(b): robustness.object_table(run, registry, truth, groups, arm=args.arm, budget=b)
            for b in budgets
        }
    out = {"registry_sha256": registry.content_hash(), "arm": args.arm, "tables": tables}
    if args.out.exists():
        raise lb.ProtocolError(f"{args.out} exists; tables are written once")
    args.out.write_text(json.dumps(out, indent=1, sort_keys=True) + "\n", encoding="utf-8")
    sizes = ", ".join(f"{k} ({len(v[str(budgets[0])])} objects)" for k, v in tables.items())
    print(f"wrote {args.out}: {sizes}")
    return 0


def _verdict(args: argparse.Namespace) -> int:
    bounds = json.loads(args.bounds.read_text(encoding="utf-8"))
    document = json.loads(args.tables.read_text(encoding="utf-8"))
    tables = {
        name: {int(b): rows for b, rows in per_budget.items()}
        for name, per_budget in document["tables"].items()
    }
    rows = robustness.evaluate(bounds, tables)
    result = {
        "bounds_sha256": lb.file_sha256(args.bounds),
        "tables": args.tables.name,
        "verdicts": rows,
        "summary": {
            s: sum(1 for r in rows if r["status"] == s) for s in ("pass", "fail", "inconclusive")
        },
    }
    if args.out.exists():
        raise lb.ProtocolError(f"{args.out} exists; a verdict is written once")
    args.out.write_text(json.dumps(result, indent=1, sort_keys=True) + "\n", encoding="utf-8")
    for row in rows:
        value = row.get("value")
        shown = "" if value is None else f"{value:.3f}"
        print(f"{row['status']:13s} {row['id']:44s} {row.get('k')}/{row.get('n')} {shown}")
    print(result["summary"])
    return 0


def main(argv=None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    sub = parser.add_subparsers(dest="command", required=True)
    t = sub.add_parser("tables", help="one row per tray object, per condition and budget")
    t.add_argument("--registry", required=True, type=Path)
    t.add_argument("--splits", required=True, type=Path)
    t.add_argument(
        "--condition", action="append", required=True, help="NAME=STATE[,STATE]@LISTFILE"
    )
    t.add_argument("--budgets", default="0,1,2,3")
    t.add_argument("--arm", default="off", choices=lb.ARMS)
    t.add_argument(
        "--all-scene-seeds", action="store_true", help="denominator: every seed of the datasets"
    )
    t.add_argument("--out", required=True, type=Path)
    v = sub.add_parser("verdict", help="apply a bounds file to tables")
    v.add_argument("--bounds", required=True, type=Path)
    v.add_argument("--tables", required=True, type=Path)
    v.add_argument("--out", required=True, type=Path)
    args = parser.parse_args(argv)
    try:
        return _tables(args) if args.command == "tables" else _verdict(args)
    except (lb.ProtocolError, ValueError, OSError) as error:
        print(f"error: {error}", file=sys.stderr)
        return 2


if __name__ == "__main__":
    sys.exit(main())

# Copyright 2026 Matteo Bernacchia <dev@kikijiki.com>
#
# SPDX-License-Identifier: MIT

"""``inventory_tool``: "how many of SKU X are in stock", from the running node or a recording."""

from __future__ import annotations

import argparse
import gzip
import json
from pathlib import Path
import sys

from restocker_perception.inventory import model as m
from restocker_perception.inventory.report import format_inventory

CONFIDENCE = {
    "any": m.CONFIDENCE_ANY,
    "expected": m.CONFIDENCE_EXPECTED,
    "confirmed": m.CONFIDENCE_CONFIRMED,
}


def _filters(parser: argparse.ArgumentParser) -> None:
    parser.add_argument("--sku", default="", help="one SKU (how many are in stock?)")
    parser.add_argument("--lane", default="", help="one lane")
    parser.add_argument(
        "--min-confidence",
        choices=sorted(CONFIDENCE, key=CONFIDENCE.get),
        default="any",
        help="evidence floor: any (incl. world-state reports), expected (sensor-counted), "
        "confirmed (named by the identity channel only)",
    )
    parser.add_argument(
        "--max-age", type=float, default=0.0, help="stale after this many seconds (0: declared)"
    )


def _print_answer(answer, *, sku: str, lane: str) -> None:
    if not answer.ok:
        print(f"refused: {answer.message}", file=sys.stderr)
        return
    if sku:
        print(answer.explanation)
        if answer.count_known:
            span = (
                f"{answer.count_estimate}"
                if answer.count_min == answer.count_max
                else f"{answer.count_estimate} (between {answer.count_min} and {answer.count_max})"
            )
            print(
                f"\n{sku}: {span}; {answer.confirmed_count} confirmed; "
                f"{'complete' if answer.complete else 'INCOMPLETE'}"
            )
        else:
            print(f"\n{sku}: unknown, no lane that can hold it was readable")
        return
    print(format_inventory(answer.view))


def _ros(arguments: argparse.Namespace) -> int:
    import rclpy
    from restocker_interfaces.srv import QueryInventory

    rclpy.init()
    node = rclpy.create_node("inventory_tool")
    try:
        client = node.create_client(QueryInventory, arguments.service)
        if not client.wait_for_service(timeout_sec=arguments.timeout):
            print(
                f"{arguments.service} is not available (is the inventory node running?)",
                file=sys.stderr,
            )
            return 2
        request = QueryInventory.Request()
        request.sku, request.lane_id = arguments.sku, arguments.lane
        request.min_confidence = CONFIDENCE[arguments.min_confidence]
        request.max_age_s = float(arguments.max_age)
        future = client.call_async(request)
        rclpy.spin_until_future_complete(node, future, timeout_sec=arguments.timeout)
        response = future.result()
        if response is None:
            print("no answer from the inventory node", file=sys.stderr)
            return 2
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
    if not response.ok:
        print(f"refused: {response.message}", file=sys.stderr)
        return 1
    if arguments.sku:
        _print_answer(response, sku=arguments.sku, lane=arguments.lane)
    else:
        print(format_inventory(response.inventory))
    return 0


def read_predictions(path: str | Path) -> list[dict]:
    opener = gzip.open if str(path).endswith(".gz") else open
    with opener(path, "rt", encoding="utf-8") as handle:
        return [json.loads(line) for line in handle if line.strip()]


def _replay(arguments: argparse.Namespace) -> int:
    from restocker_perception.inventory import replay
    from restocker_perception.inventory.query import query_inventory
    from restocker_perception.stock.benchmark import load_dataset

    labels = [row for directory in arguments.datasets for row in load_dataset(directory)[1]]
    by_frame = {row["frame_id"]: row for row in labels}
    predictions = [p for path in arguments.predictions for p in read_predictions(path)]
    rows = [
        p
        for p in predictions
        if p["frame_id"] in by_frame
        and by_frame[p["frame_id"]]["seed"] == arguments.seed
        and "state" in p
    ]
    if not rows:
        print(f"no lane predictions for seed {arguments.seed}", file=sys.stderr)
        return 2
    truth = replay.scene_truth(by_frame[rows[0]["frame_id"]])
    config = m.InventoryConfig(stale_after_s=arguments.horizon)
    ages = replay.reading_ages(truth, config.stale_after_s, aged=arguments.aged)
    readings = replay.readings_for_scene(truth, rows, ages)
    perturb = replay.perturbation(truth) if arguments.perturb_world else None
    answer = query_inventory(
        readings,
        replay.world_view(truth, perturb),
        None,
        replay.QUERY_TIME_S,
        config,
        sku=arguments.sku,
        lane_id=arguments.lane,
        min_confidence=CONFIDENCE[arguments.min_confidence],
        max_age_s=arguments.max_age,
    )
    _print_answer(answer, sku=arguments.sku, lane=arguments.lane)
    if arguments.truth and answer.ok:
        print("\nlabelled truth (lanes + tray):", dict(sorted(truth.sku_totals().items())))
    return 0 if answer.ok else 1


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    sub = parser.add_subparsers(dest="command", required=True)
    for name, text in (("query", "ask the running node"), ("show", "print the whole inventory")):
        p = sub.add_parser(name, help=text)
        _filters(p)
        p.add_argument("--service", default="/query_inventory")
        p.add_argument("--timeout", type=float, default=5.0)
        p.set_defaults(func=_ros)
    p = sub.add_parser("replay", help="the inventory of one recorded benchmark scene")
    _filters(p)
    p.add_argument("--datasets", nargs="+", required=True, help="pbench-* directories")
    p.add_argument(
        "--predictions", nargs="+", required=True, help="stock_benchmark rows (jsonl[.gz])"
    )
    p.add_argument("--seed", type=int, required=True)
    p.add_argument("--horizon", type=float, default=600.0)
    p.add_argument(
        "--aged", action="store_true", help="spread the readings' ages past the horizon"
    )
    p.add_argument("--perturb-world", action="store_true", help="declared seeded ledger errors")
    p.add_argument("--truth", action="store_true", help="also print the labelled totals")
    p.set_defaults(func=_replay)
    arguments = parser.parse_args(argv)
    if arguments.command == "show":
        arguments.sku = arguments.lane = ""
    return arguments.func(arguments)


if __name__ == "__main__":
    sys.exit(main())

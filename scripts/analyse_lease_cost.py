#!/usr/bin/env python3
# Copyright 2026 Matteo Bernacchia <dev@kikijiki.com>
# SPDX-License-Identifier: MIT

"""Read a paired lease-cost measurement and say what it can and cannot support.

The pairing is the point. Runs of a simulator launch test on a shared machine are dominated by
whatever else the machine is doing, so the unpaired failure counts of two arms measured at
different times say almost nothing. Comparing the two arms *within* a pair removes the load that
both saw, and the discordant pairs -- the ones where the arms disagreed -- are the entire evidence
about the treatment. That is McNemar's test, and its exact form is a binomial test on the
discordant pairs against p = 0.5.

It also reports what the sample could have detected, because "no significant difference" from four
pairs is not a finding and should not be allowed to read like one.
"""

from __future__ import annotations

import csv
from math import comb
from pathlib import Path
import statistics
import sys


def exact_binomial_two_sided(successes: int, trials: int) -> float:
    """Return the two-sided exact binomial p-value against p = 0.5."""
    if trials == 0:
        return 1.0
    outcomes = [comb(trials, k) for k in range(trials + 1)]
    total = float(sum(outcomes))
    observed = outcomes[successes]
    return min(1.0, sum(count for count in outcomes if count <= observed) / total)


def majority_needed(trials: int, alpha: float = 0.05) -> int | None:
    """Return how many of ``trials`` discordant pairs must agree to reach significance.

    None when no split of that many pairs can, which is always the case below six.
    """
    for minority in range(trials // 2, -1, -1):
        if exact_binomial_two_sided(minority, trials) <= alpha:
            return trials - minority
    return None


def main(argv: list[str]) -> int:
    """Summarise the measurement directory named on the command line."""
    directory = Path(argv[0] if argv else "/tmp/restocker-lease-cost")
    records_path = directory / "records.tsv"
    if not records_path.is_file():
        print(f"no records at {records_path}", file=sys.stderr)
        return 2

    with records_path.open(encoding="utf-8") as handle:
        rows = list(csv.DictReader(handle, delimiter="\t"))
    if not rows:
        print("no runs recorded", file=sys.stderr)
        return 2

    arms: dict[str, dict[int, dict[str, str]]] = {"leased": {}, "pinned": {}}
    for row in rows:
        arms[row["arm"]][int(row["pair"])] = row
    pairs = sorted(set(arms["leased"]) & set(arms["pinned"]))

    print(f"pairs completed: {len(pairs)}")
    for arm in ("pinned", "leased"):
        failures = sum(1 for pair in pairs if arms[arm][pair]["exit"] != "0")
        loads = [float(arms[arm][pair]["load_at_start"]) for pair in pairs]
        seconds = [float(arms[arm][pair]["seconds"]) for pair in pairs]
        gz = [int(arms[arm][pair]["gz_at_start"]) for pair in pairs]
        # `bc` is absent from this environment, so a batch can record every duration as 0.0.
        # Printing a median of zero would read as a measurement rather than as a missing one.
        duration = (
            f"wall seconds median {statistics.median(seconds):.0f}  "
            if any(value > 0 for value in seconds)
            else "wall seconds not captured  "
        )
        print(
            f"  {arm:6s} failed {failures}/{len(pairs)}  "
            f"load at start median {statistics.median(loads):.1f} "
            f"(min {min(loads):.1f}, max {max(loads):.1f})  "
            f"{duration}"
            f"simulators already running at start: {sorted(gz)}"
        )

    both_fail = leased_only = pinned_only = neither = 0
    for pair in pairs:
        leased_failed = arms["leased"][pair]["exit"] != "0"
        pinned_failed = arms["pinned"][pair]["exit"] != "0"
        if leased_failed and pinned_failed:
            both_fail += 1
        elif leased_failed:
            leased_only += 1
        elif pinned_failed:
            pinned_only += 1
        else:
            neither += 1

    print()
    print("paired outcomes")
    print(f"  both passed          {neither}")
    print(f"  both failed          {both_fail}")
    print(f"  only leased failed   {leased_only}")
    print(f"  only pinned failed   {pinned_only}")

    discordant = leased_only + pinned_only
    p_value = exact_binomial_two_sided(leased_only, discordant)
    print()
    print(f"McNemar exact on {discordant} discordant pair(s): p = {p_value:.3f}")

    on_observed = majority_needed(discordant)
    if discordant == 0:
        print("  the arms never disagreed within a pair, so there is nothing for the test to see.")
    elif on_observed is None:
        print(
            f"  {discordant} discordant pair(s) cannot reach p <= 0.05 on any split, so this "
            "result excludes nothing but a total effect."
        )
    else:
        print(
            f"  on {discordant} discordant pair(s), p <= 0.05 needed {on_observed} of them to "
            "fall the same way."
        )

    best_case = majority_needed(len(pairs))
    if best_case is None:
        print(
            f"  ceiling: even if all {len(pairs)} pairs had been discordant, no split reaches "
            "p <= 0.05. This sample size cannot answer the question."
        )
    else:
        print(
            f"  ceiling: with all {len(pairs)} pairs discordant, {best_case} of {len(pairs)} "
            "falling the same way would have been the smallest significant result."
        )
    return 0


if __name__ == "__main__":
    raise SystemExit(main(sys.argv[1:]))

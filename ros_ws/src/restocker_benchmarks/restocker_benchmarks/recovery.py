# Copyright 2026 Matteo Bernacchia <dev@kikijiki.com>
#
# SPDX-License-Identifier: MIT

"""Measure whether a goal's recovery attempts behave as independent redraws."""
# A reliability estimate for a bounded-recovery fault is a product over attempts, valid only if
# the attempts are independent draws. This module measures the two assumptions of that product from
# the stored feedback trace alone:
#
#   Independence. Given that a goal entered recovery on fault family F, how often did F come
#   back on a later recovery slot? Under independent redraws that rate is F's base rate. A
#   rate measurably above it means the attempts are correlated and the product understates the
#   joint failure probability.
#
#   Budget sharing. task.max_recovery_attempts is spent across the whole RestockProduct goal,
#   not per operation, so a late fault can find the pool already emptied by an unrelated one.
#   The number of slots left when each family first appears is what a per-fault estimate must
#   condition on.
#
# The independence assumption alone accounted for a factor of 7.5 in an earlier estimate.
#
# Everything is recomputed from `trace` rather than read from a stored field, like the failure
# taxonomy, so a campaign recorded earlier re-aggregates correctly.
#
# Limit of the evidence: the trace records state transitions. task.max_operation_retries
# re-submits an operation without a transition, so a per-operation retry leaves no entry and its
# `attempt` counter is reset by the next transition. Each recovery slot counted here is therefore
# one recovery. For a Cartesian segment the invisible repeats rerun the same interpolation against
# the same retained pose target from an arm a planning failure never moved, so they carry little
# new information. Whether one ever succeeds is not observable from a stored record.

from __future__ import annotations

from collections import Counter, defaultdict
import math

from restocker_benchmarks import taxonomy

# Trace states that record the shared per-goal recovery pool being spent or found empty.
RECOVERY_STATES: tuple[str, ...] = ("RECOVER", "FAULT")

# A recovery slot's detail is a mid-goal fault, not a terminal outcome, so it has no status of its
# own. The taxonomy's family rules special-case only the statuses for "succeeded" and "the harness
# said this"; any other value routes the detail through the rules. Named so the argument to
# classify_family does not read as an arbitrary integer.
_FAULT_STATUS: int = 0


def wilson_interval(successes: int, total: int, z: float = 1.96) -> tuple[float, float] | None:
    """Return the Wilson score interval for successes of total, or None when there is no sample."""
    # Wilson rather than the normal approximation: rates here are measured on tens of goals and
    # several are near zero, where the normal interval is meaningless or negative. A zero
    # numerator still yields a usable upper bound ("0 of 39" is a bound, not proof of absence).
    if total <= 0:
        return None
    proportion = successes / total
    denominator = 1.0 + z * z / total
    centre = (proportion + z * z / (2.0 * total)) / denominator
    spread = z * math.sqrt(proportion * (1.0 - proportion) / total + z * z / (4.0 * total * total))
    spread /= denominator
    return (max(0.0, centre - spread), min(1.0, centre + spread))


def format_interval(interval: tuple[float, float] | None) -> str:
    """Render a Wilson interval as a percentage range, or ``n/a`` when there is no sample."""
    if interval is None:
        return "n/a"
    return f"{interval[0] * 100:.1f}-{interval[1] * 100:.1f}%"


def recovery_slots(goal: dict) -> list[dict]:
    """Return the ordered faults that spent this goal's shared recovery budget."""
    # Each entry carries the slot the fault occupied (`recovery_attempt` as the machine had it
    # when the transition was emitted), the taxonomy family of its detail, and whether the slot
    # was the terminal one.
    slots = []
    for entry in goal.get("trace", []):
        if entry.get("state_name") not in RECOVERY_STATES:
            continue
        detail = entry.get("detail") or ""
        slots.append(
            {
                "slot": int(entry.get("recovery_attempt", 0)),
                "family": taxonomy.classify_family(_FAULT_STATUS, detail),
                "terminal": entry.get("state_name") == "FAULT",
                "detail": detail,
            }
        )
    return slots


def _goals(runs: list[dict]) -> list[dict]:
    return [goal for run in runs for goal in run.get("goals", [])]


def independence(runs: list[dict]) -> list[dict]:
    """Measure recurrence of the same fault family on a later recovery slot."""
    # Returns P(next attempt fails | this one failed) per family. The unconditional side is the
    # family's share of goals, which the caller has from the taxonomy.
    entered: Counter[str] = Counter()
    recurred: Counter[str] = Counter()
    for goal in _goals(runs):
        slots = recovery_slots(goal)
        if not slots:
            continue
        first = slots[0]["family"]
        entered[first] += 1
        if any(slot["family"] == first for slot in slots[1:]):
            recurred[first] += 1

    rows = [
        {
            "family": family,
            "entered": entered[family],
            "recurred": recurred[family],
            "rate": recurred[family] / entered[family],
            "interval": wilson_interval(recurred[family], entered[family]),
        }
        for family in sorted(entered, key=lambda name: (-entered[name], name))
    ]
    total_entered = sum(entered.values())
    if total_entered:
        rows.append(
            {
                "family": "ALL",
                "entered": total_entered,
                "recurred": sum(recurred.values()),
                "rate": sum(recurred.values()) / total_entered,
                "interval": wilson_interval(sum(recurred.values()), total_entered),
            }
        )
    return rows


def budget_sharing(runs: list[dict], budget: int = 2) -> dict:
    """Measure how often the shared pool was already spent when a fault arrived."""
    # `budget` is task.max_recovery_attempts. A parameter because campaigns may have been run with
    # a different value and a stored record does not carry it.
    ladders = [recovery_slots(goal) for goal in _goals(runs)]
    spent_before: dict[str, Counter[int]] = defaultdict(Counter)
    starved = 0
    mixed = 0
    entering = 0
    for slots in ladders:
        if not slots:
            continue
        entering += 1
        seen: set[str] = set()
        for slot in slots:
            if slot["family"] in seen:
                continue
            seen.add(slot["family"])
            # Slots already spent when this family first appeared. A RECOVER entry reports the
            # counter after its own increment, so it consumed one slot; a FAULT entry is the guard
            # refusing to increment, so its counter is the number already gone.
            consumed = slot["slot"] if slot["terminal"] else max(0, slot["slot"] - 1)
            spent_before[slot["family"]][consumed] += 1
        terminal = next((slot for slot in reversed(slots) if slot["terminal"]), None)
        if terminal is None:
            continue
        # Shared-budget signal: the goal died on one fault after a different one had already
        # taken part of the pool.
        if any(slot["family"] != terminal["family"] for slot in slots if not slot["terminal"]):
            mixed += 1
        if terminal["slot"] >= budget:
            starved += 1

    families = [
        {
            "family": family,
            "first_seen_at_slot": dict(sorted(counts.items())),
            "arrived_with_pool_empty": sum(
                count for slot, count in counts.items() if slot >= budget
            ),
            # Counted once per goal, at first appearance: the measure is the pool state on arrival.
            "goals": sum(counts.values()),
        }
        for family, counts in sorted(spent_before.items(), key=lambda item: -sum(item[1].values()))
    ]
    return {
        "budget": budget,
        "goals_entering_recovery": entering,
        "died_with_the_pool_exhausted": starved,
        "died_after_another_fault_spent_a_slot": mixed,
        "families": families,
    }


def summarize(runs: list[dict], budget: int = 2) -> dict:
    """Reduce a campaign to the recovery-independence evidence."""
    return {
        "independence": independence(runs),
        "budget_sharing": budget_sharing(runs, budget),
    }

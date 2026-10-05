# Copyright 2026 Matteo Bernacchia <dev@kikijiki.com>
#
# SPDX-License-Identifier: MIT

"""The benchmark scenarios, as a table."""
# A Python table rather than a configuration format. A scenario is a stock file and a goal
# sequence; every other knob a run needs (headless, motion enabled, controller timeout) is fixed by
# the runner. If a scenario needs a value this table cannot express, add the field here.

from __future__ import annotations

from dataclasses import dataclass


@dataclass(frozen=True)
class Goal:
    """One RestockProduct goal."""

    # ``source_object_id`` is the scenario's stable simulator identity, resolved against the world
    # state at run time because the numeric object id is assigned as observations are admitted.
    # Both fields unset asks the coordinator to select the pair itself. Scenario goals stay
    # both-or-neither by this dataclass's own rule; ``select_task_pair`` additionally accepts an
    # object-only request since Card 037 (the destination is then chosen among the lanes
    # compatible with that object), but no shipped scenario uses that form.

    label: str
    source_object_id: str | None = None
    lane_id: str | None = None

    def __post_init__(self) -> None:
        if (self.source_object_id is None) != (self.lane_id is None):
            raise ValueError(f"goal {self.label!r} must address an object and a lane together")


@dataclass(frozen=True)
class Scenario:
    """A stock configuration and the goal sequence driven against it."""

    name: str
    description: str
    # Basename under restocker_gazebo/config, resolved through the package share directory so a
    # benchmark reads the same file the launch does.
    stock_config: str
    # ``None`` means one unaddressed goal per product the scenario stocked; the product count
    # varies with the seed.
    goals: tuple[Goal, ...] | None
    # Whether ``--seed`` is meaningful. The fixed-pose scenarios reproduce a named demo and a named
    # routing decision, so seeding them would measure something else.
    seedable: bool


SCENARIOS: tuple[Scenario, ...] = (
    Scenario(
        name="baseline_transfers",
        description=(
            "The baseline demo: one product of each category moved from the stock tray into an "
            "explicitly named lane. Every goal is addressed, so this measures the transfer and "
            "makes no selection decision."
        ),
        stock_config="baseline_products.yaml",
        goals=(
            Goal("can into lane_01", "sim:stock_can_01", "lane_01"),
            Goal("small bottle into lane_02", "sim:stock_small_bottle_01", "lane_02"),
            Goal("large bottle into lane_03", "sim:stock_large_bottle_01", "lane_03"),
        ),
        seedable=False,
    ),
    Scenario(
        name="dense_restock_transfers",
        description=(
            "Same addressed goals as baseline_transfers against dense_restock_products.yaml: "
            "partial columns in all six front lanes plus 25 products in three partial back-tray "
            "SKU grids. Use this to exercise a stocked shelf and sequential picks from a dense "
            "stock-side fixture."
        ),
        stock_config="dense_restock_products.yaml",
        goals=(
            Goal("can into lane_01", "sim:stock_can_01", "lane_01"),
            Goal("small bottle into lane_02", "sim:stock_small_bottle_01", "lane_02"),
            Goal("large bottle into lane_03", "sim:stock_large_bottle_01", "lane_03"),
        ),
        seedable=False,
    ),
    Scenario(
        name="lane_routing",
        description=(
            "A lane taking a column, then autonomous selection in front of it: two addressed "
            "goals put both small bottles into lane_02, one behind the other, and one unaddressed "
            "goal must clear the tray with a lane already holding two."
        ),
        stock_config="lane_routing_products.yaml",
        goals=(
            Goal("seed lane_02", "sim:stock_small_bottle_01", "lane_02"),
            Goal("column into lane_02", "sim:stock_small_bottle_02", "lane_02"),
            Goal("unaddressed goal"),
        ),
        seedable=False,
    ),
    Scenario(
        name="seeded_autonomous",
        description=(
            "One unaddressed goal per stocked product against a seeded stock draw. The seed fixes "
            "how many products exist and where they stand, so a failure found here replays "
            "exactly; where each product ends up is still the coordinator's decision."
        ),
        stock_config="baseline_products.yaml",
        goals=None,
        seedable=True,
    ),
)

SCENARIOS_BY_NAME = {scenario.name: scenario for scenario in SCENARIOS}


def scenario(name: str) -> Scenario:
    """Return the named scenario, or raise with the list of names that exist."""
    try:
        return SCENARIOS_BY_NAME[name]
    except KeyError:
        known = ", ".join(sorted(SCENARIOS_BY_NAME))
        raise SystemExit(
            f"unknown benchmark scenario {name!r}; known scenarios: {known}"
        ) from None

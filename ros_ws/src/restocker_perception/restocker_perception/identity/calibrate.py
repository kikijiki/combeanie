# Copyright 2026 Matteo Bernacchia <dev@kikijiki.com>
#
# SPDX-License-Identifier: MIT

"""Fit the abstention thresholds on labelled observations; report the same metrics everywhere."""
# This module never sees a split name. The caller hands it samples that it is entitled to use
# (the benchmark tool passes train and validation seeds only, never held-out) and the targets to
# fit against. A sample is one object (one or several views) with the observation the channels
# produced for it, the true product id, and whether it was registered when observed.
#
# Unknown products on calibration data come from leaving registered products out of the index in
# turn (cross-validation over products): a product that is absent from the index the sample was
# observed against is an unknown product for that sample. Held-out products are never involved.

from __future__ import annotations

from collections.abc import Sequence
from dataclasses import dataclass, field, replace
import itertools
import math
from typing import Any

import numpy as np

from restocker_perception.identity.config import IdentityConfig
from restocker_perception.identity.engine import IDENTIFIED, UNKNOWN, Observation, decide


@dataclass(frozen=True)
class Sample:
    truth: str
    known: bool
    observation: Observation
    # Optional look-alike partner of the truth, for twin-confusion reporting.
    twin: str | None = None
    # The truth is not registered but its look-alike twin is (the dangerous unknown).
    twin_registered: bool = False
    # Scene seed of the object: cross-validation keeps a scene's objects together.
    seed: int = 0


@dataclass
class Metrics:
    n_known: int = 0
    n_unknown: int = 0
    known_correct: int = 0
    known_false_identity: int = 0
    known_twin_confusion: int = 0
    known_abstain: int = 0
    known_unknown: int = 0
    unknown_named: int = 0
    n_unknown_twin: int = 0
    unknown_twin_named: int = 0
    unknown_rejected_unknown: int = 0
    unknown_abstain: int = 0
    # channel that named the SKU, among identified decisions
    by_channel: dict[str, int] = field(default_factory=dict)

    @staticmethod
    def _rate(numerator: int, denominator: int) -> float | None:
        return numerator / denominator if denominator else None

    @property
    def false_identity_known(self) -> float | None:
        return self._rate(self.known_false_identity, self.n_known)

    @property
    def unknown_named_rate(self) -> float | None:
        return self._rate(self.unknown_named, self.n_unknown)

    @property
    def coverage_known(self) -> float | None:
        """Share of known objects named correctly."""
        return self._rate(self.known_correct, self.n_known)

    @property
    def abstention_known(self) -> float | None:
        return self._rate(self.known_abstain + self.known_unknown, self.n_known)

    def to_dict(self) -> dict[str, Any]:
        return {
            "n_known": self.n_known,
            "n_unknown": self.n_unknown,
            "known_correct": self.known_correct,
            "known_false_identity": self.known_false_identity,
            "known_twin_confusion": self.known_twin_confusion,
            "known_abstain": self.known_abstain,
            "known_called_unknown": self.known_unknown,
            "unknown_named_as_sku": self.unknown_named,
            "n_unknown_twin_registered": self.n_unknown_twin,
            "unknown_twin_named_as_sku": self.unknown_twin_named,
            "unknown_called_unknown": self.unknown_rejected_unknown,
            "unknown_abstain": self.unknown_abstain,
            "false_identity_rate_known": self.false_identity_known,
            "unknown_named_rate": self.unknown_named_rate,
            "correct_rate_known": self.coverage_known,
            "abstention_rate_known": self.abstention_known,
            "twin_confusion_rate_known": self._rate(self.known_twin_confusion, self.n_known),
            "identified_by": dict(sorted(self.by_channel.items())),
        }


def evaluate(samples: Sequence[Sample], config: IdentityConfig) -> Metrics:
    metrics = Metrics()
    for sample in samples:
        result = decide(sample.observation, config, {})
        if result.status == IDENTIFIED:
            channel = result.reason.split(" ")[0]
            metrics.by_channel[channel] = metrics.by_channel.get(channel, 0) + 1
        if sample.known:
            metrics.n_known += 1
            if result.status == IDENTIFIED and result.sku == sample.truth:
                metrics.known_correct += 1
            elif result.status == IDENTIFIED:
                metrics.known_false_identity += 1
                if sample.twin is not None and result.sku == sample.twin:
                    metrics.known_twin_confusion += 1
            elif result.status == UNKNOWN:
                metrics.known_unknown += 1
            else:
                metrics.known_abstain += 1
        else:
            metrics.n_unknown += 1
            metrics.n_unknown_twin += sample.twin_registered
            if result.status == IDENTIFIED:
                metrics.unknown_named += 1
                metrics.unknown_twin_named += sample.twin_registered
            elif result.status == UNKNOWN:
                metrics.unknown_rejected_unknown += 1
            else:
                metrics.unknown_abstain += 1
    return metrics


def _grid(values: Sequence[float], count: int) -> list[float]:
    if not len(values):
        return []
    quantiles = np.quantile(np.asarray(values, dtype=np.float64), np.linspace(0.0, 1.0, count))
    return sorted({round(float(q), 6) for q in quantiles})


def wrong_wins(samples: Sequence[Sample], config: IdentityConfig) -> dict[str, float]:
    """Highest top-1 score with which each product was named wrongly (known or unknown object)."""
    worst: dict[str, float] = {}
    for sample in samples:
        result = decide(sample.observation, config, {})
        if result.status == IDENTIFIED and (not sample.known or result.sku != sample.truth):
            match = sample.observation.match
            if (
                match is not None
                and match.top1 is not None
                and result.sku == match.top1.product_id
            ):
                worst[result.sku] = max(worst.get(result.sku, -math.inf), match.top1.score)
    return worst


def naming_shares(samples: Sequence[Sample], config: IdentityConfig) -> dict[str, float]:
    """Share of all naming decisions that went to each product (a hub takes a large share)."""
    counts: dict[str, int] = {}
    total = 0
    for sample in samples:
        result = decide(sample.observation, config, {})
        if result.status == IDENTIFIED and result.sku is not None:
            counts[result.sku] = counts.get(result.sku, 0) + 1
            total += 1
    return {k: v / total for k, v in sorted(counts.items())} if total else {}


def fit_thresholds(
    samples: Sequence[Sample],
    *,
    max_false_identity_known: float,
    max_unknown_named: float,
    max_known_called_unknown: float = 0.05,
    grid_points: int = 9,
    base: IdentityConfig | None = None,
    floor_margin: float = 0.01,
) -> tuple[IdentityConfig, dict[str, Any]]:
    """Fit the numeric thresholds only; the options (scoring, whitening, views) are in ``base``.

    For each candidate (alone score, margin, vote) the per-product floors are set just above the
    highest score with which that product was ever named wrongly (the guard against a hub: a
    prototype that wins too much must clear a higher bar), then the candidate is scored. The one
    naming the most known objects within the ceilings wins; ties go to the stricter one. A hub
    flag is reported when one product takes more than twice its fair share of naming decisions.
    """
    base = base or IdentityConfig()
    scores = [s.observation.match.top1.score for s in samples if s.observation.match]
    margins = [
        s.observation.match.margin
        for s in samples
        if s.observation.match and s.observation.match.margin is not None
    ]
    votes = [0.5, 0.67, 0.8, 1.0]
    options: list[tuple[float | None, float | None, float | None]] = [(None, None, None)]
    options += list(
        itertools.product(_grid(scores, grid_points), _grid(margins, grid_points), votes)
    )
    best: tuple[tuple[float, float], IdentityConfig, Metrics] | None = None
    searched = 0
    for score, margin, vote in options:
        candidate = replace(
            base, alone_min_score=score, alone_min_margin=margin, alone_min_vote=vote
        )
        floors = wrong_wins(samples, candidate) if score is not None else {}
        candidate = replace(
            candidate, product_floors={k: round(v + floor_margin, 6) for k, v in floors.items()}
        )
        metrics = evaluate(samples, candidate)
        searched += 1
        fi = metrics.false_identity_known or 0.0
        un = metrics.unknown_named_rate or 0.0
        twin = (
            metrics.unknown_twin_named / metrics.n_unknown_twin if metrics.n_unknown_twin else 0.0
        )
        if fi > max_false_identity_known or un > max_unknown_named or twin > max_unknown_named:
            continue
        key = (metrics.known_correct, -(fi + un))
        if best is None or key > best[0]:
            best = (key, candidate, metrics)
    # Nothing feasible: name nothing at all (independent channels off too).
    config = best[1] if best else replace(base, supported_max_rank=0)
    floor = _fit_unknown_floor(samples, config, max_known_called_unknown)
    config = replace(config, unknown_below_score=floor)
    final = evaluate(samples, config)
    shares = naming_shares(samples, config)
    products = {s.truth for s in samples if s.known}
    fair = 1.0 / max(len(products), 1)
    record = {
        "configurations_searched": searched,
        "feasible": best is not None,
        "ceilings": {
            "false_identity_known": max_false_identity_known,
            "unknown_named": max_unknown_named,
            "unknown_twin_named": max_unknown_named,
            "known_called_unknown": max_known_called_unknown,
        },
        "calibration_metrics": final.to_dict(),
        "naming_shares": shares,
        "hub_flags": sorted(k for k, v in shares.items() if v > 2 * fair),
    }
    return config, record


def _fit_unknown_floor(
    samples: Sequence[Sample], config: IdentityConfig, max_known_called_unknown: float
) -> float | None:
    """Highest floor that sends at most max_known_called_unknown of known objects to unknown."""
    known_scores = sorted(
        s.observation.match.top1.score for s in samples if s.known and s.observation.match
    )
    unknown_scores = [
        s.observation.match.top1.score for s in samples if not s.known and s.observation.match
    ]
    if not known_scores or not unknown_scores:
        return None
    allowed = int(math.floor(max_known_called_unknown * len(known_scores)))
    # The floor must sit at or below the (allowed+1)-th smallest known score.
    ceiling = known_scores[min(allowed, len(known_scores) - 1)]
    floor = float(np.nextafter(ceiling, -np.inf))
    return floor if any(u < floor for u in unknown_scores) else None


def cross_validate(
    samples: Sequence[Sample],
    *,
    max_false_identity_known: float,
    max_unknown_named: float,
    base: IdentityConfig,
    folds: int = 5,
    floor_margin: float = 0.01,
) -> dict[str, Any]:
    """Out-of-fold error of the whole fitting procedure (thresholds and per-product floors).

    The calibration metrics of ``fit_thresholds`` are measured on the data that set the
    thresholds, and per-product floors make them look perfect by construction. This fits on
    the samples of all but one group of scenes (seeds) and scores the group left out, so what is
    reported is what the fitted channel does on scenes it did not see.
    """
    seeds = sorted({s.seed for s in samples})
    groups = [seeds[i::folds] for i in range(folds)]
    total = Metrics()
    per_fold = []
    for held in groups:
        held_set = set(held)
        train = [s for s in samples if s.seed not in held_set]
        test = [s for s in samples if s.seed in held_set]
        if not train or not test:
            continue
        config, _record = fit_thresholds(
            train,
            max_false_identity_known=max_false_identity_known,
            max_unknown_named=max_unknown_named,
            base=base,
            floor_margin=floor_margin,
        )
        metrics = evaluate(test, config)
        per_fold.append(metrics.to_dict())
        for name in (
            "n_known",
            "n_unknown",
            "known_correct",
            "known_false_identity",
            "known_twin_confusion",
            "known_abstain",
            "known_unknown",
            "unknown_named",
            "n_unknown_twin",
            "unknown_twin_named",
            "unknown_rejected_unknown",
            "unknown_abstain",
        ):
            setattr(total, name, getattr(total, name) + getattr(metrics, name))
        for channel, count in metrics.by_channel.items():
            total.by_channel[channel] = total.by_channel.get(channel, 0) + count
    return {"folds": folds, "pooled": total.to_dict(), "per_fold": per_fold}


def choose_floor_margin(
    samples: Sequence[Sample],
    *,
    max_false_identity_known: float,
    max_unknown_named: float,
    base: IdentityConfig,
    candidates: Sequence[float] = (0.01, 0.03, 0.05, 0.08),
) -> tuple[float, list[dict[str, Any]]]:
    """The one numeric knob of the hub guard, chosen on out-of-fold error (calibration data).

    A larger margin keeps a product's floor further above its worst wrong win, which costs
    coverage and buys robustness to scenes the fit did not see. Pick the margin with the most
    correct namings among those whose out-of-fold false-identity and unknown-named rates are
    within the calibration ceilings; if none is, the one with the fewest unknown products named.
    """
    tried = []
    for margin in candidates:
        report = cross_validate(
            samples,
            max_false_identity_known=max_false_identity_known,
            max_unknown_named=max_unknown_named,
            base=base,
            floor_margin=margin,
        )
        pooled = report["pooled"]
        tried.append(
            {
                "floor_margin": margin,
                "correct": pooled["known_correct"],
                "false_identity": pooled["known_false_identity"],
                "unknown_named": pooled["unknown_named_as_sku"],
                "false_identity_rate": pooled["false_identity_rate_known"],
                "unknown_named_rate": pooled["unknown_named_rate"],
            }
        )
    feasible = [
        t
        for t in tried
        if (t["false_identity_rate"] or 0.0) <= max_false_identity_known
        and (t["unknown_named_rate"] or 0.0) <= max_unknown_named
    ]
    if feasible:
        best = max(feasible, key=lambda t: (t["correct"], -t["floor_margin"]))
    else:
        best = min(tried, key=lambda t: (t["unknown_named"], t["false_identity"], -t["correct"]))
    return best["floor_margin"], tried

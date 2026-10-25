# Copyright 2026 Matteo Bernacchia <dev@kikijiki.com>
#
# SPDX-License-Identifier: MIT

"""Frozen train/validation/held-out splits over products and scenario seeds, and the run guard."""
# The split file is committed and carries its own hash. ``check_run`` is the gate: a run that
# registers a held-out product, trains or tunes on a held-out seed, or was configured against a
# split file other than the frozen one is refused. Nothing here imports ROS.
#
# Why the product split keeps look-alike twins apart. The held-out products exist to test the
# "unknown product" behaviour. A held-out product whose near-identical twin is also unseen tests
# nothing hard; one whose twin IS registered is the dangerous case (the system is tempted to name
# the twin). So at most one member of a look-alike group is held out, and the rest are registered.

from __future__ import annotations

from collections.abc import Iterable, Sequence
import json
from pathlib import Path
from typing import Any

from restocker_gazebo.scenario_random import SplitMix64  # noqa: I100,I101

from restocker_benchmarks.perception_dataset import canonical_json, sha256_bytes

SPLITS_SCHEMA_VERSION = 1
# Version 2 extends a version-1 file append-only: the old seeds keep their exact assignments.
SPLITS_SCHEMA_VERSION_EXTENDED = 2
# Version 3 (CMB-119) appends the box SKUs and a block of seeds whose scenes may contain them to a
# version-2 file. Nothing the base assigns changes; seeds up to the base's last keep the scenes
# they always drew (no boxes), and ``box_seed_start`` is the first seed that may include boxes.
SPLITS_SCHEMA_VERSION_BOXES = 3
# Version 4 (CMB-120 round 2) appends seeds only to a version-3 file or to a version-4 one: no
# product is added, the catalogue view and ``box_seed_start`` stay those of version 3.
SPLITS_SCHEMA_VERSION_BOX_SEEDS = 4
# Version 5 (CMB-112 round 3) appends seeds to a version-4 file and records the identity round's
# registry roles. In that round every product the cell handles is registered, held-out products
# included (the look-alike twin of a registered product must be registered too); a short list of
# products stays unregistered so refusal of unknown products can still be checked. The burned
# generation grows: every held-out seed an earlier identity round has read.
SPLITS_SCHEMA_VERSION_IDENTITY = 5
# Version 6 (CMB-121 round 4) appends seeds to a version-5 file (or a version-6 one), keeps the
# identity-round registry roles and burns every held-out seed the previous identity round read
# (those seeds decided a bound and can never decide another).
SPLITS_SCHEMA_VERSION_IDENTITY_SEEDS = 6
SPLIT_NAMES = ("train", "validation", "held_out")
# Product and seed fractions (train, validation, held-out). Held-out is never below one item.
PRODUCT_FRACTIONS = (0.5, 0.25, 0.25)
SEED_FRACTIONS = (0.5, 0.25, 0.25)
# Held-out seeds also contain registered products (so known and unknown are scored side by side),
# but train and validation seeds contain only registered ones.
SPLIT_SALT = 0xC0DEBEA7


def _shuffled(items: Sequence[Any], stream: SplitMix64) -> list[Any]:
    """Fisher-Yates over a sorted copy with the pinned generator (not ``random.shuffle``)."""
    result = sorted(items)
    for index in range(len(result) - 1, 0, -1):
        other = stream.next_below(index + 1)
        result[index], result[other] = result[other], result[index]
    return result


def _counts(total: int, fractions: tuple[float, float, float]) -> tuple[int, int, int]:
    held_out = max(1, round(total * fractions[2]))
    validation = max(1, round(total * fractions[1])) if total >= 3 else 0
    train = total - held_out - validation
    if train < 1:
        raise ValueError(f"cannot split {total} items into three non-empty splits")
    return train, validation, held_out


def split_products(products: Sequence[dict[str, Any]], split_seed: int) -> dict[str, list[str]]:
    """Assign every SKU to one split; at most one member of a look-alike group is held out."""
    stream = SplitMix64(split_seed ^ SPLIT_SALT)
    order = _shuffled([row["sku"] for row in products], stream)
    group_of = {row["sku"]: row.get("lookalike_group") for row in products}
    train_n, validation_n, held_n = _counts(len(order), PRODUCT_FRACTIONS)
    held_out: list[str] = []
    used_groups: set[str] = set()
    for sku in order:
        if len(held_out) == held_n:
            break
        group = group_of[sku]
        if group is not None and group in used_groups:
            continue
        held_out.append(sku)
        if group is not None:
            used_groups.add(group)
    if len(held_out) != held_n:
        raise ValueError("not enough look-alike groups to hold out the requested products")
    rest = [sku for sku in order if sku not in held_out]
    return {
        "train": sorted(rest[:train_n]),
        "validation": sorted(rest[train_n : train_n + validation_n]),
        "held_out": sorted(held_out),
    }


def split_seeds(seeds: Sequence[int], split_seed: int) -> dict[str, list[int]]:
    """Partition scenario seeds into the three splits."""
    stream = SplitMix64((split_seed ^ SPLIT_SALT) + 1)
    order = _shuffled(list(seeds), stream)
    train_n, validation_n, _held = _counts(len(order), SEED_FRACTIONS)
    return {
        "train": sorted(order[:train_n]),
        "validation": sorted(order[train_n : train_n + validation_n]),
        "held_out": sorted(order[train_n + validation_n :]),
    }


def body_hash(body: dict[str, Any]) -> str:
    """Hash of a split document's content (everything except the hash field itself)."""
    return sha256_bytes(canonical_json({k: v for k, v in body.items() if k != "sha256"}).encode())


def freeze_splits(
    products: Sequence[dict[str, Any]],
    seeds: Sequence[int],
    *,
    split_seed: int,
    catalogue_sha256: str,
) -> dict[str, Any]:
    """Build the frozen split document (deterministic in its arguments)."""
    document = {
        "schema_version": SPLITS_SCHEMA_VERSION,
        "split_seed": split_seed,
        "catalogue_sha256": catalogue_sha256,
        "products": split_products(products, split_seed),
        "seeds": split_seeds(seeds, split_seed),
        "lookalike_groups": {
            row["sku"]: row.get("lookalike_group")
            for row in sorted(products, key=lambda r: r["sku"])
        },
        "policy": (
            "held-out products are never registered; train and validation seeds contain only "
            "train and validation products; held-out seeds contain all products"
        ),
    }
    document["sha256"] = body_hash(document)
    return document


def extend_splits(
    base: dict[str, Any], *, last_seed: int, counts: tuple[int, int, int]
) -> dict[str, Any]:
    """Append seeds to a frozen file without touching anything it already assigns.

    ``base`` is the version-1 file or an earlier extension of it (the file written by this
    function). Seeds ``max(base seeds) + 1 .. last_seed`` are shuffled with their own pinned
    stream (``split_seed ^ SPLIT_SALT`` plus 2 for the first extension, plus 3 for the second,
    and so on; the product and base-seed streams use plus 0 and 1) and cut into train, validation
    and held-out by ``counts`` (zero is allowed, so an extension can add held-out seeds only).
    The product split, the look-alike groups and every base assignment are copied unchanged.
    The version-1 held-out seeds are recorded as ``burned``: they decided the first
    identity-channel round and must not be used for a new decision; every held-out seed added
    since is ``fresh``. The whole base document is embedded, so the extension chain can be
    re-derived and verified from the file alone.
    """
    problems = validate_splits(base)
    if problems:
        raise ValueError("extension needs a valid split file: " + "; ".join(problems))
    extended = base.get("schema_version") == SPLITS_SCHEMA_VERSION_EXTENDED
    if not extended and base.get("schema_version") != SPLITS_SCHEMA_VERSION:
        raise ValueError("extension needs a version-1 split file or an extension of one")
    generation = base["extension"]["generation"] + 1 if extended else 1
    first = max(seed for name in SPLIT_NAMES for seed in base["seeds"][name]) + 1
    new = list(range(first, last_seed + 1))
    if sum(counts) != len(new) or min(counts) < 0 or not new:
        raise ValueError(f"counts {counts} must be non-negative and sum to {len(new)} added seeds")
    stream = SplitMix64((base["split_seed"] ^ SPLIT_SALT) + 1 + generation)
    order = _shuffled(new, stream)
    train_n, validation_n, _held = counts
    added = {
        "train": sorted(order[:train_n]),
        "validation": sorted(order[train_n : train_n + validation_n]),
        "held_out": sorted(order[train_n + validation_n :]),
    }
    burned = base["held_out_generations"]["burned"] if extended else base["seeds"]["held_out"]
    fresh = base["held_out_generations"]["fresh"] if extended else []
    document = {
        "schema_version": SPLITS_SCHEMA_VERSION_EXTENDED,
        "split_seed": base["split_seed"],
        "catalogue_sha256": base["catalogue_sha256"],
        "products": base["products"],
        "seeds": {name: sorted(base["seeds"][name] + added[name]) for name in SPLIT_NAMES},
        "lookalike_groups": base["lookalike_groups"],
        "policy": base["policy"],
        "held_out_generations": {
            "burned": sorted(burned),
            "fresh": sorted(fresh + added["held_out"]),
        },
        "extension": {
            "generation": generation,
            "base_sha256": base["sha256"],
            "added_seed_range": [first, last_seed],
            "counts": list(counts),
            "base_document": base,
        },
    }
    document["sha256"] = body_hash(document)
    return document


def extend_with_products(
    base: dict[str, Any],
    new_products: Sequence[dict[str, Any]],
    *,
    last_seed: int,
    seed_counts: tuple[int, int, int],
    product_counts: tuple[int, int, int],
    catalogue_sha256: str,
) -> dict[str, Any]:
    """Append new SKUs and a block of seeds to a version-2 file (the box catalogue, CMB-119).

    The new SKUs are shuffled with their own pinned stream (``split_seed ^ SPLIT_SALT`` plus
    ``0x100 + generation``) and cut into train, validation and held-out by ``product_counts``,
    never holding out two members of one look-alike group. The new seeds
    ``max(base seeds) + 1 .. last_seed`` are shuffled with the stream ``extend_splits`` would use
    for this generation and cut by ``seed_counts``. Every base assignment is copied unchanged.
    The first new seed is recorded as ``box_seed_start``: only seeds from there on draw boxes.
    """
    problems = validate_splits(base)
    if problems or base.get("schema_version") != SPLITS_SCHEMA_VERSION_EXTENDED:
        raise ValueError(
            "box extension needs a valid version-2 split file: " + "; ".join(problems)
        )
    new_skus = sorted(row["sku"] for row in new_products)
    known = {sku for name in SPLIT_NAMES for sku in base["products"][name]}
    if not new_skus or known & set(new_skus):
        raise ValueError("new products must be non-empty and not already split")
    if sum(product_counts) != len(new_skus) or min(product_counts) < 0:
        raise ValueError(f"product_counts {product_counts} must sum to {len(new_skus)}")
    generation = base["extension"]["generation"] + 1
    stream = SplitMix64((base["split_seed"] ^ SPLIT_SALT) + 0x100 + generation)
    order = _shuffled(new_skus, stream)
    group_of = {row["sku"]: row.get("lookalike_group") for row in new_products}
    train_n, validation_n, held_n = product_counts
    held_out: list[str] = []
    used: set[str] = {
        group
        for sku in base["products"]["held_out"]
        if (group := base["lookalike_groups"].get(sku)) is not None
    }
    for sku in order:
        if len(held_out) == held_n:
            break
        group = group_of[sku]
        if group is not None and group in used:
            continue
        held_out.append(sku)
        if group is not None:
            used.add(group)
    if len(held_out) != held_n:
        raise ValueError("not enough look-alike groups to hold out the requested products")
    rest = [sku for sku in order if sku not in held_out]
    added_products = {
        "train": rest[:train_n],
        "validation": rest[train_n : train_n + validation_n],
        "held_out": held_out,
    }
    first = max(seed for name in SPLIT_NAMES for seed in base["seeds"][name]) + 1
    new_seeds = list(range(first, last_seed + 1))
    if sum(seed_counts) != len(new_seeds) or min(seed_counts) < 0 or not new_seeds:
        raise ValueError(f"seed_counts {seed_counts} must sum to {len(new_seeds)} added seeds")
    seed_stream = SplitMix64((base["split_seed"] ^ SPLIT_SALT) + 1 + generation)
    seed_order = _shuffled(new_seeds, seed_stream)
    train_s, validation_s, _ = seed_counts
    added_seeds = {
        "train": sorted(seed_order[:train_s]),
        "validation": sorted(seed_order[train_s : train_s + validation_s]),
        "held_out": sorted(seed_order[train_s + validation_s :]),
    }
    document = {
        "schema_version": SPLITS_SCHEMA_VERSION_BOXES,
        "split_seed": base["split_seed"],
        "catalogue_sha256": catalogue_sha256,
        "catalogue_includes_boxes": True,
        "box_seed_start": first,
        "products": {
            name: sorted(base["products"][name] + added_products[name]) for name in SPLIT_NAMES
        },
        "seeds": {name: sorted(base["seeds"][name] + added_seeds[name]) for name in SPLIT_NAMES},
        "lookalike_groups": {
            **base["lookalike_groups"],
            **{row["sku"]: row.get("lookalike_group") for row in new_products},
        },
        "policy": base["policy"]
        + "; seeds from box_seed_start on draw from the catalogue including boxes, earlier seeds "
        "draw as they always did",
        "held_out_generations": {
            "burned": sorted(base["held_out_generations"]["burned"]),
            "fresh": sorted(base["held_out_generations"]["fresh"] + added_seeds["held_out"]),
        },
        "extension": {
            "generation": generation,
            "base_sha256": base["sha256"],
            "added_seed_range": [first, last_seed],
            "seed_counts": list(seed_counts),
            "product_counts": list(product_counts),
            "new_products": sorted(new_skus),
            "base_document": base,
        },
    }
    document["sha256"] = body_hash(document)
    return document


def extend_box_seeds(
    base: dict[str, Any],
    *,
    last_seed: int,
    seed_counts: tuple[int, int, int],
    burn: Sequence[int] = (),
) -> dict[str, Any]:
    """Append a block of seeds to a version-3 or version-4 file (box scenes, no new product).

    The new seeds ``max(base seeds) + 1 .. last_seed`` are shuffled with the stream
    ``(split_seed ^ SPLIT_SALT) + 1 + generation`` (the one ``extend_splits`` would use for this
    generation) and cut into train, validation and held-out by ``seed_counts``. Every base
    assignment, the products and ``box_seed_start`` are copied unchanged; the new held-out seeds
    are ``fresh``. ``burn`` moves held-out seeds that earlier measurements already decided on from
    ``fresh`` to ``burned`` (it can only add to the burned list, never remove from it). Which fresh
    seeds a later measurement uses is recorded with that measurement, not in this file.
    """
    problems = validate_splits(base)
    if problems or base.get("schema_version") not in (
        SPLITS_SCHEMA_VERSION_BOXES,
        SPLITS_SCHEMA_VERSION_BOX_SEEDS,
    ):
        raise ValueError(
            "box seed extension needs a valid version-3 or version-4 split file: "
            + "; ".join(problems)
        )
    generation = base["extension"]["generation"] + 1
    first = max(seed for name in SPLIT_NAMES for seed in base["seeds"][name]) + 1
    new_seeds = list(range(first, last_seed + 1))
    if sum(seed_counts) != len(new_seeds) or min(seed_counts) < 0 or not new_seeds:
        raise ValueError(f"seed_counts {seed_counts} must sum to {len(new_seeds)} added seeds")
    burn = sorted(set(burn))
    if not set(burn) <= set(base["held_out_generations"]["fresh"]):
        raise ValueError("only fresh held-out seeds can be burned")
    order = _shuffled(new_seeds, SplitMix64((base["split_seed"] ^ SPLIT_SALT) + 1 + generation))
    train_n, validation_n, _ = seed_counts
    added = {
        "train": sorted(order[:train_n]),
        "validation": sorted(order[train_n : train_n + validation_n]),
        "held_out": sorted(order[train_n + validation_n :]),
    }
    document = {
        "schema_version": SPLITS_SCHEMA_VERSION_BOX_SEEDS,
        "split_seed": base["split_seed"],
        "catalogue_sha256": base["catalogue_sha256"],
        "catalogue_includes_boxes": True,
        "box_seed_start": base["box_seed_start"],
        "products": base["products"],
        "seeds": {name: sorted(base["seeds"][name] + added[name]) for name in SPLIT_NAMES},
        "lookalike_groups": base["lookalike_groups"],
        "policy": base["policy"],
        "held_out_generations": {
            "burned": sorted(base["held_out_generations"]["burned"] + burn),
            "fresh": sorted(
                [s for s in base["held_out_generations"]["fresh"] if s not in burn]
                + added["held_out"]
            ),
        },
        "extension": {
            "generation": generation,
            "base_sha256": base["sha256"],
            "added_seed_range": [first, last_seed],
            "seed_counts": list(seed_counts),
            "burned_additions": burn,
            "base_document": base,
        },
    }
    document["sha256"] = body_hash(document)
    return document


def extend_identity_round(
    base: dict[str, Any],
    *,
    last_seed: int,
    seed_counts: tuple[int, int, int],
    unregistered: Sequence[str],
    burn_held_out_through: int,
    round_name: str = "cmb112-r3",
) -> dict[str, Any]:
    """Append seeds and the identity round's registry roles to a version-4 file.

    The new seeds ``max(base seeds) + 1 .. last_seed`` are shuffled with the stream
    ``extend_splits`` would use for this generation and cut by ``seed_counts``. Every held-out
    seed up to ``burn_held_out_through`` joins ``burned`` (the earlier identity rounds read them:
    their outcome is known, so they cannot decide a new bound); the rest stay ``fresh``. The
    registry roles: every catalogue product is registered except ``unregistered``, which must be
    products of the catalogue; those appear in held-out scenes only (the base policy keeps
    train and validation scenes to train and validation products), where they are the unknown
    products the system must refuse. Nothing the base assigns changes.
    """
    problems = validate_splits(base)
    if problems or base.get("schema_version") != SPLITS_SCHEMA_VERSION_BOX_SEEDS:
        raise ValueError(
            "identity-round extension needs a valid version-4 split file: " + "; ".join(problems)
        )
    everything = sorted(base["lookalike_groups"])
    unregistered = sorted(set(unregistered))
    if not unregistered or not set(unregistered) <= set(everything):
        raise ValueError("unregistered must be a non-empty subset of the catalogue products")
    generation = base["extension"]["generation"] + 1
    first = max(seed for name in SPLIT_NAMES for seed in base["seeds"][name]) + 1
    new_seeds = list(range(first, last_seed + 1))
    if sum(seed_counts) != len(new_seeds) or min(seed_counts) < 0 or not new_seeds:
        raise ValueError(f"seed_counts {seed_counts} must sum to {len(new_seeds)} added seeds")
    order = _shuffled(new_seeds, SplitMix64((base["split_seed"] ^ SPLIT_SALT) + 1 + generation))
    train_s, validation_s, _ = seed_counts
    added = {
        "train": sorted(order[:train_s]),
        "validation": sorted(order[train_s : train_s + validation_s]),
        "held_out": sorted(order[train_s + validation_s :]),
    }
    generations = base["held_out_generations"]
    old_held = sorted(generations["burned"] + generations["fresh"])
    burned = sorted(
        set(generations["burned"]) | {x for x in old_held if x <= burn_held_out_through}
    )
    fresh = sorted((set(old_held) - set(burned)) | set(added["held_out"]))
    document = {
        "schema_version": SPLITS_SCHEMA_VERSION_IDENTITY,
        "split_seed": base["split_seed"],
        "catalogue_sha256": base["catalogue_sha256"],
        "catalogue_includes_boxes": True,
        "box_seed_start": base["box_seed_start"],
        "products": base["products"],
        "seeds": {name: sorted(base["seeds"][name] + added[name]) for name in SPLIT_NAMES},
        "lookalike_groups": base["lookalike_groups"],
        "policy": base["policy"]
        + "; identity round "
        + round_name
        + ": every product but the listed unregistered ones is registered (held-out products "
        "too); the unregistered products appear in held-out scenes only",
        "held_out_generations": {"burned": burned, "fresh": fresh},
        "identity_round": {
            "name": round_name,
            "registered_products": sorted(set(everything) - set(unregistered)),
            "unregistered_products": unregistered,
            "burned_through_seed": burn_held_out_through,
        },
        "extension": {
            "generation": generation,
            "base_sha256": base["sha256"],
            "added_seed_range": [first, last_seed],
            "seed_counts": list(seed_counts),
            "base_document": base,
        },
    }
    document["sha256"] = body_hash(document)
    return document


def extend_identity_seeds(
    base: dict[str, Any],
    *,
    last_seed: int,
    seed_counts: tuple[int, int, int],
    burn_held_out_through: int,
    round_name: str = "cmb121-r4",
) -> dict[str, Any]:
    """Append seeds to a version-5 or version-6 file and burn the held-out seeds already read.

    The new seeds ``max(base seeds) + 1 .. last_seed`` are shuffled with the stream
    ``extend_splits`` would use for this generation and cut by ``seed_counts``. Every held-out
    seed up to ``burn_held_out_through`` moves to ``burned`` (it can only add to the burned list).
    Products, look-alike groups and the identity-round registry roles are those of the base; only
    the round's name and its burn line change. The new held-out seeds are ``fresh``.
    """
    problems = validate_splits(base)
    if problems or base.get("schema_version") not in (
        SPLITS_SCHEMA_VERSION_IDENTITY,
        SPLITS_SCHEMA_VERSION_IDENTITY_SEEDS,
    ):
        raise ValueError(
            "identity seed extension needs a valid version-5 or version-6 split file: "
            + "; ".join(problems)
        )
    generation = base["extension"]["generation"] + 1
    first = max(seed for name in SPLIT_NAMES for seed in base["seeds"][name]) + 1
    new_seeds = list(range(first, last_seed + 1))
    if sum(seed_counts) != len(new_seeds) or min(seed_counts) < 0 or not new_seeds:
        raise ValueError(f"seed_counts {seed_counts} must sum to {len(new_seeds)} added seeds")
    if burn_held_out_through < base["identity_round"]["burned_through_seed"]:
        raise ValueError("the burn line cannot move down")
    order = _shuffled(new_seeds, SplitMix64((base["split_seed"] ^ SPLIT_SALT) + 1 + generation))
    train_s, validation_s, _ = seed_counts
    added = {
        "train": sorted(order[:train_s]),
        "validation": sorted(order[train_s : train_s + validation_s]),
        "held_out": sorted(order[train_s + validation_s :]),
    }
    generations = base["held_out_generations"]
    old_held = sorted(generations["burned"] + generations["fresh"])
    burned = sorted(
        set(generations["burned"]) | {x for x in old_held if x <= burn_held_out_through}
    )
    fresh = sorted((set(old_held) - set(burned)) | set(added["held_out"]))
    roles = base["identity_round"]
    document = {
        "schema_version": SPLITS_SCHEMA_VERSION_IDENTITY_SEEDS,
        "split_seed": base["split_seed"],
        "catalogue_sha256": base["catalogue_sha256"],
        "catalogue_includes_boxes": True,
        "box_seed_start": base["box_seed_start"],
        "products": base["products"],
        "seeds": {name: sorted(base["seeds"][name] + added[name]) for name in SPLIT_NAMES},
        "lookalike_groups": base["lookalike_groups"],
        "policy": base["policy"]
        + "; identity round "
        + round_name
        + ": seeds appended, held-out seeds through "
        + str(burn_held_out_through)
        + " burned, registry roles unchanged",
        "held_out_generations": {"burned": burned, "fresh": fresh},
        "identity_round": {
            "name": round_name,
            "registered_products": roles["registered_products"],
            "unregistered_products": roles["unregistered_products"],
            "burned_through_seed": burn_held_out_through,
        },
        "extension": {
            "generation": generation,
            "base_sha256": base["sha256"],
            "added_seed_range": [first, last_seed],
            "seed_counts": list(seed_counts),
            "base_document": base,
        },
    }
    document["sha256"] = body_hash(document)
    return document


def scene_includes_boxes(document: dict[str, Any], seed: int) -> bool:
    """True when a seed's scene may draw boxes (version 3 only, from ``box_seed_start`` on)."""
    start = document.get("box_seed_start")
    return start is not None and seed >= start


def reproduce_extension(document: dict[str, Any]) -> dict[str, Any]:
    """Re-derive an extension from its embedded base, recursively (the chain must reproduce)."""
    extension = document["extension"]
    base = extension["base_document"]
    if document.get("schema_version") == SPLITS_SCHEMA_VERSION_IDENTITY_SEEDS:
        if base.get("schema_version") == SPLITS_SCHEMA_VERSION_IDENTITY_SEEDS and (
            reproduce_extension(base) != base
        ):
            raise ValueError("embedded base extension does not reproduce")
        if base.get("schema_version") == SPLITS_SCHEMA_VERSION_IDENTITY and (
            reproduce_extension(base) != base
        ):
            raise ValueError("embedded base extension does not reproduce")
        return extend_identity_seeds(
            base,
            last_seed=extension["added_seed_range"][1],
            seed_counts=tuple(extension["seed_counts"]),
            burn_held_out_through=document["identity_round"]["burned_through_seed"],
            round_name=document["identity_round"]["name"],
        )
    if document.get("schema_version") == SPLITS_SCHEMA_VERSION_IDENTITY:
        if reproduce_extension(base) != base:
            raise ValueError("embedded base extension does not reproduce")
        return extend_identity_round(
            base,
            last_seed=extension["added_seed_range"][1],
            seed_counts=tuple(extension["seed_counts"]),
            unregistered=document["identity_round"]["unregistered_products"],
            burn_held_out_through=document["identity_round"]["burned_through_seed"],
            round_name=document["identity_round"]["name"],
        )
    if document.get("schema_version") == SPLITS_SCHEMA_VERSION_BOX_SEEDS:
        if base.get("schema_version") == SPLITS_SCHEMA_VERSION_BOX_SEEDS and (
            reproduce_extension(base) != base
        ):
            raise ValueError("embedded base extension does not reproduce")
        if base.get("schema_version") == SPLITS_SCHEMA_VERSION_BOXES and (
            reproduce_extension(base) != base
        ):
            raise ValueError("embedded base extension does not reproduce")
        return extend_box_seeds(
            base,
            last_seed=extension["added_seed_range"][1],
            seed_counts=tuple(extension["seed_counts"]),
            burn=extension["burned_additions"],
        )
    if document.get("schema_version") == SPLITS_SCHEMA_VERSION_BOXES:
        if reproduce_extension(base) != base:
            raise ValueError("embedded base extension does not reproduce")
        rows = [
            {"sku": sku, "lookalike_group": document["lookalike_groups"][sku]}
            for sku in extension["new_products"]
        ]
        return extend_with_products(
            base,
            rows,
            last_seed=extension["added_seed_range"][1],
            seed_counts=tuple(extension["seed_counts"]),
            product_counts=tuple(extension["product_counts"]),
            catalogue_sha256=document["catalogue_sha256"],
        )
    if (
        base.get("schema_version") == SPLITS_SCHEMA_VERSION_EXTENDED
        and reproduce_extension(base) != base
    ):
        raise ValueError("embedded base extension does not reproduce")
    return extend_splits(
        base,
        last_seed=extension["added_seed_range"][1],
        counts=tuple(extension["counts"]),
    )


def write_splits(document: dict[str, Any], path: str | Path) -> None:
    """Write the split file in a stable, diff-friendly form."""
    Path(path).write_text(json.dumps(document, indent=2, sort_keys=True) + "\n", encoding="utf-8")


def load_splits(path: str | Path) -> dict[str, Any]:
    """Read a split file and verify its embedded hash and internal consistency."""
    document = json.loads(Path(path).read_text(encoding="utf-8"))
    problems = validate_splits(document)
    if problems:
        raise ValueError("split file is invalid: " + "; ".join(problems))
    return document


def validate_splits(document: dict[str, Any]) -> list[str]:
    """Return what is wrong with a split document (empty list = frozen and consistent)."""
    problems: list[str] = []
    if document.get("sha256") != body_hash(document):
        problems.append("embedded sha256 does not match the content (file was edited)")
    for kind in ("products", "seeds"):
        parts = document.get(kind, {})
        if set(parts) != set(SPLIT_NAMES):
            problems.append(f"{kind} must have exactly {SPLIT_NAMES}")
            continue
        flat = [item for name in SPLIT_NAMES for item in parts[name]]
        if len(flat) != len(set(flat)):
            problems.append(f"{kind} splits overlap")
        if not parts["held_out"]:
            problems.append(f"{kind} held_out is empty")
    if document.get("schema_version") == SPLITS_SCHEMA_VERSION_EXTENDED:
        problems += _validate_extension(document)
    if document.get("schema_version") in (
        SPLITS_SCHEMA_VERSION_BOXES,
        SPLITS_SCHEMA_VERSION_BOX_SEEDS,
    ):
        problems += _validate_box_extension(document, products_added=(
            document.get("schema_version") == SPLITS_SCHEMA_VERSION_BOXES
        ))
    if document.get("schema_version") in (
        SPLITS_SCHEMA_VERSION_IDENTITY,
        SPLITS_SCHEMA_VERSION_IDENTITY_SEEDS,
    ):
        problems += _validate_identity_extension(document)
    groups = document.get("lookalike_groups", {})
    held = set(document.get("products", {}).get("held_out", []))
    seen: dict[str, int] = {}
    for sku in held:
        group = groups.get(sku)
        if group is not None:
            seen[group] = seen.get(group, 0) + 1
    if any(count > 1 for count in seen.values()):
        problems.append("a look-alike group has more than one held-out member")
    return problems


def _validate_extension(document: dict[str, Any]) -> list[str]:
    problems: list[str] = []
    extension = document.get("extension", {})
    base = extension.get("base_document", {})
    if validate_splits(base) or base.get("sha256") != extension.get("base_sha256"):
        problems.append("embedded base split file is invalid or does not match base_sha256")
        return problems
    for name in SPLIT_NAMES:
        if not set(base["seeds"][name]) <= set(document["seeds"][name]):
            problems.append(f"base {name} seeds are not preserved")
    if document["products"] != base["products"]:
        problems.append("product split differs from the base file")
    generations = document.get("held_out_generations", {})
    burned, fresh = set(generations.get("burned", [])), set(generations.get("fresh", []))
    base_burned = (
        set(base["held_out_generations"]["burned"])
        if base.get("schema_version") == SPLITS_SCHEMA_VERSION_EXTENDED
        else set(base["seeds"]["held_out"])
    )
    if burned != base_burned:
        problems.append("burned held-out seeds are not the first round's held-out seeds")
    if burned & fresh or (burned | fresh) != set(document["seeds"]["held_out"]):
        problems.append("held-out generations do not partition the held-out seeds")
    return problems


def _validate_box_extension(document: dict[str, Any], *, products_added: bool = True) -> list[str]:
    problems: list[str] = []
    extension = document.get("extension", {})
    base = extension.get("base_document", {})
    if validate_splits(base) or base.get("sha256") != extension.get("base_sha256"):
        return ["embedded base split file is invalid or does not match base_sha256"]
    for kind in ("products", "seeds"):
        for name in SPLIT_NAMES:
            if not set(base[kind][name]) <= set(document[kind][name]):
                problems.append(f"base {name} {kind} are not preserved")
    if products_added:
        expected_start = max(seed for name in SPLIT_NAMES for seed in base["seeds"][name]) + 1
        if document.get("box_seed_start") != expected_start:
            problems.append("box_seed_start is not the first seed added to the base")
    else:
        if document.get("box_seed_start") != base.get("box_seed_start"):
            problems.append("box_seed_start changed")
        if document["products"] != base["products"]:
            problems.append("a seed extension changed the products")
    generations = document.get("held_out_generations", {})
    burned, fresh = set(generations.get("burned", [])), set(generations.get("fresh", []))
    base_burned = set(base["held_out_generations"]["burned"])
    if products_added and burned != base_burned:
        problems.append("burned held-out seeds changed")
    if not products_added and not base_burned <= burned:
        problems.append("burned held-out seeds were removed")
    if burned & fresh or (burned | fresh) != set(document["seeds"]["held_out"]):
        problems.append("held-out generations do not partition the held-out seeds")
    return problems


def _validate_identity_extension(document: dict[str, Any]) -> list[str]:
    problems: list[str] = []
    extension = document.get("extension", {})
    base = extension.get("base_document", {})
    if validate_splits(base) or base.get("sha256") != extension.get("base_sha256"):
        return ["embedded base split file is invalid or does not match base_sha256"]
    for kind in ("products", "seeds"):
        for name in SPLIT_NAMES:
            if not set(base[kind][name]) <= set(document[kind][name]):
                problems.append(f"base {name} {kind} are not preserved")
    generations = document.get("held_out_generations", {})
    burned, fresh = set(generations.get("burned", [])), set(generations.get("fresh", []))
    if not set(base["held_out_generations"]["burned"]) <= burned:
        problems.append("a seed burned in the base is no longer burned")
    if burned & fresh or (burned | fresh) != set(document["seeds"]["held_out"]):
        problems.append("held-out generations do not partition the held-out seeds")
    roles = document.get("identity_round", {})
    registered = set(roles.get("registered_products", []))
    unregistered = set(roles.get("unregistered_products", []))
    if registered & unregistered or (registered | unregistered) != set(
        document["lookalike_groups"]
    ):
        problems.append("identity-round roles do not partition the catalogue products")
    if document.get("schema_version") == SPLITS_SCHEMA_VERSION_IDENTITY_SEEDS:
        line = roles.get("burned_through_seed", -1)
        if any(s <= line and s not in burned for s in document["seeds"]["held_out"]):
            problems.append("a held-out seed at or below the burn line is not burned")
        base_roles = base.get("identity_round", {})
        for key in ("registered_products", "unregistered_products"):
            if roles.get(key) != base_roles.get(key):
                problems.append(f"{key} changed from the base file")
        first, last = extension["added_seed_range"]
        added = set(range(first, last + 1))
        if any(s in burned for s in document["seeds"]["held_out"] if s in added):
            problems.append("a newly added held-out seed is already burned")
    return problems


def held_out_generation(document: dict[str, Any], seed: int) -> str | None:
    """``burned`` or ``fresh`` for a held-out seed of a version-2 file, else None."""
    for name, seeds in document.get("held_out_generations", {}).items():
        if seed in seeds:
            return name
    return None


def registered_skus(document: dict[str, Any]) -> list[str]:
    """The products a registry may hold under this split: train plus validation."""
    return sorted(document["products"]["train"] + document["products"]["validation"])


def allowed_skus_for_seed(document: dict[str, Any], seed: int) -> list[str] | None:
    """SKUs a scene of this seed may contain (None = unrestricted, i.e. a held-out seed)."""
    if seed in document["seeds"]["held_out"]:
        return None
    if seed in document["seeds"]["train"] or seed in document["seeds"]["validation"]:
        return registered_skus(document)
    raise ValueError(f"seed {seed} is not in the frozen split file")


def split_of_seed(document: dict[str, Any], seed: int) -> str | None:
    """Return the split a seed belongs to, or None if it is not in the file."""
    for name in SPLIT_NAMES:
        if seed in document["seeds"][name]:
            return name
    return None


class SplitViolation(Exception):
    """A run configuration crosses the held-out boundary."""


def check_run(document: dict[str, Any], run: dict[str, Any]) -> list[str]:
    """Return every reason this run configuration must be refused (empty list = allowed).

    ``run`` keys (all optional lists): ``registry_skus``, ``training_seeds``,
    ``calibration_seeds`` (threshold or margin tuning), ``registry_source_seeds`` (seeds the
    registry exemplar images were taken from), plus ``splits_sha256`` (the split file the run
    claims to have used).
    """
    problems = validate_splits(document)
    claimed = run.get("splits_sha256")
    if claimed is not None and claimed != document["sha256"]:
        problems.append(
            f"run was configured against split file {claimed[:12]}, frozen file is "
            f"{document['sha256'][:12]}"
        )
    held_products = set(document["products"]["held_out"])
    held_seeds = set(document["seeds"]["held_out"])
    roles = document.get("identity_round")
    if roles is None:
        for sku in sorted(set(run.get("registry_skus", [])) & held_products):
            problems.append(f"registry contains held-out product {sku}")
    else:
        # Identity round: held-out products may be registered; the listed ones may not.
        for sku in sorted(set(run.get("registry_skus", [])) & set(roles["unregistered_products"])):
            problems.append(f"registry contains product {sku}, which this round leaves unregistered")
    for key in ("training_seeds", "calibration_seeds", "registry_source_seeds"):
        for seed in sorted(set(run.get(key, [])) & held_seeds):
            problems.append(f"{key} contains held-out seed {seed}")
    known = (
        set(document["products"]["train"])
        | set(document["products"]["validation"])
        | held_products
    )
    for sku in sorted(set(run.get("registry_skus", [])) - known):
        problems.append(f"registry contains {sku}, which is not in the frozen split")
    return problems


def require_clean_run(document: dict[str, Any], run: dict[str, Any]) -> None:
    """Raise :class:`SplitViolation` unless ``check_run`` finds nothing."""
    problems = check_run(document, run)
    if problems:
        raise SplitViolation("; ".join(problems))


def seeds_range(count: int, start: int = 1) -> Iterable[int]:
    """The canonical seed list: ``start .. start + count - 1``."""
    return range(start, start + count)

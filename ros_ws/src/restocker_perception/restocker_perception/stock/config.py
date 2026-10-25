# Copyright 2026 Matteo Bernacchia <dev@kikijiki.com>
#
# SPDX-License-Identifier: MIT

"""The shared numbers of the lane stock estimator, loaded from ``config/lane_stock.yaml``."""

from __future__ import annotations

from dataclasses import dataclass, field
import math
from pathlib import Path
from typing import Any

import yaml

PRODUCT_CLASS_UNKNOWN = 0
# 4 (box, CMB-119): one class for every box shape, so a box's own dimensions come from the shape
# its SKU registers (``StockConfig.product_by_sku``), not from the class.
CLASS_NAMES = {1: "can", 2: "small_bottle", 3: "large_bottle", 4: "box"}
CLASS_ENUM = {name: value for value, name in CLASS_NAMES.items()}


@dataclass(frozen=True)
class ProductGeometry:
    """A product resting on the inclined bed, in one orientation.

    ``radius_m`` is half the product's extent along the lane and ``height_m`` its height above the
    bed: for an upright cylinder its radius and height, for a box the half of the side that faces
    along the lane and the side that points up. Every formula below (contact pitch, rear extent of
    a column tilted by the incline) is the same for both, which is why a box needs no formulas of
    its own: only its dimensions.
    """

    name: str
    product_class: int
    radius_m: float
    height_m: float
    # Box dimensions (across the lane, along it, up) of the standing orientation; None for a
    # cylinder.
    size_xyz_m: tuple[float, float, float] | None = None

    @property
    def depth_m(self) -> float:
        """Extent along the lane."""
        return 2.0 * self.radius_m

    def orientations(self) -> list[ProductGeometry]:
        """Every way the product can rest on the bed, as products of their own.

        A box rests on any of its six faces: the extent up is one of its sides and the extent along
        the lane is one of the other two. A cylinder is modelled upright only (it rolls when it
        lies in a lane, and the benchmark never stands one so).
        """
        if self.size_xyz_m is None:
            return [self]
        out = []
        for up in range(3):
            for along in range(3):
                if along == up:
                    continue
                out.append(
                    ProductGeometry(
                        f"{self.name}@{'xyz'[up]}up-{'xyz'[along]}along",
                        self.product_class,
                        self.size_xyz_m[along] / 2.0,
                        self.size_xyz_m[up],
                        self.size_xyz_m,
                    )
                )
        return out


@dataclass(frozen=True)
class Workcell:
    """The lane geometry the depth measurement and the count read."""

    pose_world: tuple[float, float, float]
    lane_center_x_m: dict[str, float]
    bed_datum_depth_m: float
    incline_rad: float
    rear_clearance_m: float
    usable_depth_m: float
    usable_height_m: float
    half_width_m: float

    def bed_height_at(self, lane_depth_m: Any) -> Any:
        """Height of the roller surface in the lane frame at a lane depth (scalar or array)."""
        return (self.bed_datum_depth_m - lane_depth_m) * math.tan(self.incline_rad)


@dataclass(frozen=True)
class Measurement:
    """Thresholds of the lane depth histogram (lane_survey.yaml)."""

    floor_margin_m: float
    front_inset_m: float
    obstruction_reach_m: float
    minimum_depth_m: float
    maximum_depth_m: float
    pixel_stride: int
    bin_width_m: float
    minimum_surface_bin_returns: int
    minimum_accepted_returns: int
    minimum_obstruction_returns: int
    # The floor margin grows to this many robust standard deviations of the bed's own height
    # residuals, measured in the frame (0 keeps the fixed margin). See depth_profile.measure_lane.
    floor_noise_sigma_factor: float = 0.0
    # The nearest surface is moved back by this many bed sigmas (the bias of a minimum of noisy
    # returns); only applies when a sigma was estimated.
    edge_bias_sigma_factor: float = 0.0


@dataclass(frozen=True)
class StockConfig:
    """Everything the estimator reads that is not evidence."""

    workcell: Workcell
    products: dict[str, ProductGeometry]
    measurement: Measurement
    minimum_coverage: float
    # SKU -> key of ``products``: the registered product's own shape (boxes share one class).
    sku_products: dict[str, str] = field(default_factory=dict)

    def product_by_enum(self, product_class: int) -> ProductGeometry | None:
        """The one product of a class, or None when the class names several shapes (boxes)."""
        found = [p for p in self.products.values() if p.product_class == product_class]
        return found[0] if len(found) == 1 else None

    def product_by_sku(self, sku: str | None) -> ProductGeometry | None:
        """The shape a SKU registers, if lane_stock.yaml names it."""
        key = self.sku_products.get(sku) if sku else None
        return self.products.get(key) if key else None

    def expected_product(self, product_class: int, sku: str | None) -> ProductGeometry | None:
        """What a lane policy expects: its SKU's registered shape, else its class's only shape."""
        return self.product_by_sku(sku) or self.product_by_enum(product_class)

    def pitch_m(self, product: ProductGeometry) -> float:
        """Centre-to-centre spacing of consecutive products along the bed (its contact pitch)."""
        return 2.0 * product.radius_m * math.cos(self.workcell.incline_rad)

    def rear_extent_m(self, product: ProductGeometry) -> float:
        """How far the product's depth extent reaches behind its centre (the axis-aligned bound).

        The same bound restocker_gazebo's lane evidence takes of a tilted cylinder, so a column's
        expected free depth agrees with the ground-truth producer's.
        """
        axial = abs(math.sin(self.workcell.incline_rad))
        return (
            product.radius_m * math.sqrt(max(0.0, 1.0 - axial * axial))
            + (product.height_m / 2.0) * axial
        )

    def free_depth_of_column(self, product: ProductGeometry, count: int) -> float:
        """Free depth, rear entrance to the nearest surface, of ``count`` packed products."""
        if count <= 0:
            return self.workcell.usable_depth_m
        centre = (
            self.workcell.bed_datum_depth_m
            - product.radius_m
            - (count - 1) * self.pitch_m(product)
        )
        free = centre - self.rear_extent_m(product) - self.workcell.rear_clearance_m
        return min(max(free, 0.0), self.workcell.usable_depth_m)


DEFAULT_PATH = Path(__file__).resolve().parents[2] / "config" / "lane_stock.yaml"


def _product(name: str, entry: dict[str, Any]) -> ProductGeometry:
    """A cylinder (radius, height) or a box (size_xyz_m: across the lane, along it, up)."""
    if "size_xyz_m" in entry:
        size = tuple(float(v) for v in entry["size_xyz_m"])
        return ProductGeometry(name, int(entry["product_class"]), size[1] / 2.0, size[2], size)
    return ProductGeometry(
        name, int(entry["product_class"]), float(entry["radius_m"]), float(entry["height_m"])
    )


def load_stock_config(path: str | Path | None = None) -> StockConfig:
    """Load the shared numbers; ``path`` defaults to the package's config file."""
    document = yaml.safe_load(Path(path or DEFAULT_PATH).read_text(encoding="utf-8"))
    cell = document["workcell"]
    workcell = Workcell(
        pose_world=tuple(float(v) for v in cell["pose_world"]),
        lane_center_x_m={k: float(v) for k, v in cell["lane_center_x_m"].items()},
        bed_datum_depth_m=float(cell["bed_datum_depth_m"]),
        incline_rad=math.radians(float(cell["incline_deg"])),
        rear_clearance_m=float(cell["rear_clearance_m"]),
        usable_depth_m=float(cell["usable_depth_m"]),
        usable_height_m=float(cell["usable_height_m"]),
        half_width_m=float(cell["usable_width_m"]) / 2.0 - float(cell["side_clearance_m"]),
    )
    products = {name: _product(name, entry) for name, entry in document["products"].items()}
    measurement = Measurement(**{k: v for k, v in document["measurement"].items()})
    return StockConfig(
        workcell=workcell,
        products=products,
        measurement=measurement,
        minimum_coverage=float(document["estimator"]["minimum_coverage"]),
        sku_products={str(k): str(v) for k, v in (document.get("sku_products") or {}).items()},
    )

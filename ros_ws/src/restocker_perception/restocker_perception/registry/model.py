# Copyright 2026 Matteo Bernacchia <dev@kikijiki.com>
#
# SPDX-License-Identifier: MIT

"""Registry data model and on-disk format."""
# Layout (one directory, so the whole registry is content-addressable as a unit):
#
#   <root>/registry.json                    manifest: schema, sorted product ids, hash, notes
#   <root>/products/<id>/product.json       one record (product.yaml is accepted when reading)
#   <root>/products/<id>/views/az000.png    eight exemplar views, az000 .. az315
#
# <id> is the internal product id (CMB-118): the key of the record, of its directory and of every
# derived-cache entry. A product that has a GTIN-13 uses it as its id, so a registry written
# before ids existed (schema 1, manifest key "gtins", no "product_id" in the records) loads
# unchanged and every existing id is the GTIN it always was. A product without a barcode gets
# ``prod-<12 hex>``, derived once at registration from its name and view hashes and then stored
# in the record (write-once: editing a view later does not rename the product). The GTIN is an
# optional, validated attribute of the record, unique across the registry when present.
#
# Only things a person can audit are stored. Embeddings, OCR output and every other derivation live
# in the cache (see ``cache.py``), keyed by backend name and version, and are regenerable from
# this directory alone. The content hash covers every record field and every view's SHA-256, and
# nothing else, so it changes exactly when the registry's meaning changes.

from __future__ import annotations

from collections.abc import Sequence
from dataclasses import dataclass, field
import hashlib
import json
from pathlib import Path
import re
import shutil
from typing import Any

import numpy as np

from restocker_perception.registry.gtin import validate_gtin13
from restocker_perception.registry.imageio import read_image, write_png

SCHEMA_VERSION = 2
READABLE_SCHEMAS = (1, 2)  # 1: GTIN-keyed, no product_id, manifest key "gtins"
VIEW_COUNT = 8
VIEW_STEP_DEG = 360 // VIEW_COUNT
MANIFEST_NAME = "registry.json"
PRODUCT_NAME = "product.json"
_CLASS_PATTERN = re.compile(r"^[a-z][a-z0-9_]*$")
_PRODUCT_ID_PATTERN = re.compile(r"^[A-Za-z0-9][A-Za-z0-9_-]{0,63}$")
GENERATED_ID_PREFIX = "prod-"


class RegistryError(ValueError):
    """A registry operation would break an invariant (duplicate, bad record, bad hash)."""


def canonical_json(value: Any) -> str:
    """Serialise identically on every machine (sorted keys, no spaces)."""
    return json.dumps(value, sort_keys=True, separators=(",", ":"), allow_nan=False)


def view_file_name(azimuth_deg: int) -> str:
    """File name of the view at ``azimuth_deg`` (``views/az045.png``)."""
    return f"views/az{azimuth_deg:03d}.png"


def derive_product_id(display_name: str, view_hashes: Sequence[str]) -> str:
    """Id for a product without a GTIN: ``prod-`` and 12 hex digits of its name and view hashes."""
    payload = canonical_json({"display_name": display_name, "views": list(view_hashes)})
    return GENERATED_ID_PREFIX + hashlib.sha256(payload.encode()).hexdigest()[:12]


@dataclass(frozen=True)
class ViewRef:
    """One exemplar photograph. The azimuth is nominal: an ordering, not a pose measurement."""

    file: str
    nominal_azimuth_deg: int
    sha256: str

    def to_dict(self) -> dict[str, Any]:
        return {
            "file": self.file,
            "nominal_azimuth_deg": self.nominal_azimuth_deg,
            "sha256": self.sha256,
        }


@dataclass(frozen=True)
class ProductRecord:
    """Everything stored about one product. ``product_id`` is the SKU id; the GTIN is optional."""

    product_id: str
    display_name: str
    product_class: str
    geometry_key: str
    radius_m: float
    height_m: float
    views: tuple[ViewRef, ...]
    # EAN-13 / GTIN-13 when the product has a barcode; None when it has none (never ''). A
    # product with a GTIN has ``product_id == gtin``.
    gtin: str | None = None
    ocr_strings: tuple[str, ...] = ()
    # Other identifiers the same product is known by (the simulator's ``GEN-001``), so the
    # benchmark's split file, which is keyed by those, can be applied to a GTIN-keyed registry.
    aliases: tuple[str, ...] = ()
    # Free provenance (``{"kind": "simulated", "texture_sha256": ...}`` or ``{"kind": "images"}``).
    # Part of the hash: the same views from a different source are a different registration.
    source: dict[str, Any] = field(default_factory=dict)
    # Boxes only (width, depth, height; the front face looks along -y); then ``radius_m`` is the
    # footprint's bounding radius. Left out of the serialised record for a body of revolution, so
    # every earlier record hashes as before.
    size_xyz_m: tuple[float, float, float] | None = None

    def __post_init__(self) -> None:
        if not isinstance(self.product_id, str) or not _PRODUCT_ID_PATTERN.match(self.product_id):
            raise RegistryError(f"bad product_id {self.product_id!r}")
        if self.gtin is not None:
            validate_gtin13(self.gtin)
            if self.product_id != self.gtin:
                raise RegistryError(f"{self.product_id}: a product with a GTIN uses it as its id")
        elif self.product_id.isdigit():
            raise RegistryError(f"{self.product_id}: a numeric product_id must be the GTIN")
        if not self.display_name.strip():
            raise RegistryError(f"{self.product_id}: display_name is empty")
        if not _CLASS_PATTERN.match(self.product_class):
            raise RegistryError(f"{self.product_id}: bad product_class {self.product_class!r}")
        if not self.geometry_key:
            raise RegistryError(f"{self.product_id}: geometry_key is empty")
        if not (self.radius_m > 0 and self.height_m > 0):
            raise RegistryError(f"{self.product_id}: radius_m and height_m must be positive")
        azimuths = [v.nominal_azimuth_deg for v in self.views]
        if azimuths != [i * VIEW_STEP_DEG for i in range(VIEW_COUNT)]:
            raise RegistryError(
                f"{self.product_id}: need {VIEW_COUNT} views at {VIEW_STEP_DEG} degrees "
                f"({[i * VIEW_STEP_DEG for i in range(VIEW_COUNT)]}), got {azimuths}"
            )

    def to_dict(self) -> dict[str, Any]:
        # ``product_id`` is written only when it is not the GTIN, so a record that predates ids
        # serialises (and hashes) exactly as it always did.
        extra = {} if self.product_id == self.gtin else {"product_id": self.product_id}
        return {
            **extra,
            "gtin": self.gtin,
            "display_name": self.display_name,
            "product_class": self.product_class,
            "geometry_key": self.geometry_key,
            "radius_m": self.radius_m,
            "height_m": self.height_m,
            "views": [v.to_dict() for v in self.views],
            "ocr_strings": list(self.ocr_strings),
            "aliases": list(self.aliases),
            "source": self.source,
            **({"size_xyz_m": list(self.size_xyz_m)} if self.size_xyz_m is not None else {}),
        }

    @staticmethod
    def from_dict(data: dict[str, Any]) -> ProductRecord:
        try:
            gtin = data.get("gtin")
            product_id = data.get("product_id", gtin)
            if product_id is None:
                raise KeyError("product_id (or gtin)")
            return ProductRecord(
                product_id=product_id,
                gtin=gtin,
                display_name=data["display_name"],
                product_class=data["product_class"],
                geometry_key=data["geometry_key"],
                radius_m=float(data["radius_m"]),
                height_m=float(data["height_m"]),
                views=tuple(ViewRef(**v) for v in data["views"]),
                ocr_strings=tuple(data.get("ocr_strings", ())),
                aliases=tuple(data.get("aliases", ())),
                source=dict(data.get("source", {})),
                size_xyz_m=(
                    tuple(float(v) for v in data["size_xyz_m"]) if "size_xyz_m" in data else None
                ),
            )
        except (KeyError, TypeError) as error:
            raise RegistryError(f"malformed product record: {error!r}") from error

    def view_hash(self) -> str:
        """Hash of this record's views alone: the key that ties derived data to its inputs."""
        return hashlib.sha256(canonical_json([v.sha256 for v in self.views]).encode()).hexdigest()


class Registry:
    """A registry directory: records plus their view images."""

    def __init__(self, root: str | Path) -> None:
        self.root = Path(root)
        self._products: dict[str, ProductRecord] = {}
        # Registration provenance not covered by the content hash (which splits were applied).
        self.notes: dict[str, Any] = {}

    # --- construction ------------------------------------------------------------------------

    @classmethod
    def create(cls, root: str | Path) -> Registry:
        """Create an empty registry; refuses a directory that already holds one."""
        root = Path(root)
        if (root / MANIFEST_NAME).exists():
            raise RegistryError(f"{root} already holds a registry")
        registry = cls(root)
        (root / "products").mkdir(parents=True, exist_ok=True)
        registry._write_manifest()
        return registry

    @classmethod
    def open_or_create(cls, root: str | Path) -> Registry:
        root = Path(root)
        return cls.load(root) if (root / MANIFEST_NAME).exists() else cls.create(root)

    @classmethod
    def load(cls, root: str | Path, *, verify: bool = True) -> Registry:
        """Read a registry; with ``verify`` also recheck every hash and refuse a damaged one.

        Schema 1 registries (GTIN-keyed) load as they are: each record's id is its GTIN. The
        manifest is rewritten as schema 2 the next time the registry is changed.
        """
        root = Path(root)
        manifest_path = root / MANIFEST_NAME
        if not manifest_path.exists():
            raise RegistryError(f"{root} holds no {MANIFEST_NAME}")
        manifest = json.loads(manifest_path.read_text(encoding="utf-8"))
        if manifest.get("schema_version") not in READABLE_SCHEMAS:
            raise RegistryError(f"unsupported registry schema {manifest.get('schema_version')}")
        registry = cls(root)
        registry.notes = dict(manifest.get("notes", {}))
        listed = manifest["gtins"] if manifest["schema_version"] == 1 else manifest["product_ids"]
        for product_id in listed:
            registry._products[product_id] = registry._read_record(product_id)
        if verify:
            problems = registry.verify(expected_hash=manifest.get("content_sha256"))
            if problems:
                raise RegistryError("registry is damaged: " + "; ".join(problems))
        return registry

    def _read_record(self, product_id: str) -> ProductRecord:
        directory = self.root / "products" / product_id
        path = directory / PRODUCT_NAME
        if path.exists():
            data = json.loads(path.read_text(encoding="utf-8"))
        elif (directory / "product.yaml").exists():
            import yaml  # noqa: PLC0415 - only hand-written manifests need it

            data = yaml.safe_load((directory / "product.yaml").read_text(encoding="utf-8"))
        else:
            raise RegistryError(f"{product_id}: no {PRODUCT_NAME} in {directory}")
        record = ProductRecord.from_dict(data)
        if record.product_id != product_id:
            raise RegistryError(f"{directory} holds a record for {record.product_id}")
        return record

    # --- queries -----------------------------------------------------------------------------

    @property
    def product_ids(self) -> list[str]:
        return sorted(self._products)

    @property
    def gtins(self) -> list[str]:
        """GTINs of the products that have one (barcode-less products are not listed)."""
        return sorted(r.gtin for r in self._products.values() if r.gtin is not None)

    def __contains__(self, product_id: object) -> bool:
        return product_id in self._products

    def __len__(self) -> int:
        return len(self._products)

    def get(self, product_id: str) -> ProductRecord:
        try:
            return self._products[product_id]
        except KeyError:
            raise RegistryError(f"{product_id} is not registered") from None

    def records(self) -> list[ProductRecord]:
        return [self._products[i] for i in self.product_ids]

    def by_gtin(self, gtin: str) -> str | None:
        """Product id of the product carrying ``gtin``, or None."""
        for record in self._products.values():
            if record.gtin is not None and record.gtin == gtin:
                return record.product_id
        return None

    def gtin_map(self) -> dict[str, str]:
        """GTIN -> product id for every product with a GTIN (what the barcode channel looks up)."""
        return {r.gtin: r.product_id for r in self.records() if r.gtin is not None}

    def resolve(self, identifier: str) -> str | None:
        """Product id for a product id, a GTIN or an alias (``GEN-001``), or None."""
        if identifier in self._products:
            return identifier
        by_gtin = self.by_gtin(identifier)
        if by_gtin is not None:
            return by_gtin
        for product_id in self.product_ids:
            if identifier in self._products[product_id].aliases:
                return product_id
        return None

    def view_path(self, product_id: str, view: ViewRef) -> Path:
        return self.root / "products" / product_id / view.file

    def view_images(self, product_id: str) -> list[np.ndarray]:
        """The eight exemplar views of ``product_id`` as HxWx3 uint8 arrays, azimuth order."""
        record = self.get(product_id)
        return [read_image(self.view_path(product_id, v)) for v in record.views]

    def content_hash(self) -> str:
        """SHA-256 over every record (including each view's hash), independent of file layout."""
        payload = canonical_json([self._products[i].to_dict() for i in self.product_ids])
        return hashlib.sha256(payload.encode()).hexdigest()

    # --- mutation ----------------------------------------------------------------------------

    def register(
        self,
        *,
        display_name: str,
        product_class: str,
        geometry_key: str,
        radius_m: float,
        height_m: float,
        images: Sequence[np.ndarray],
        gtin: str | None = None,
        product_id: str | None = None,
        ocr_strings: Sequence[str] = (),
        aliases: Sequence[str] = (),
        source: dict[str, Any] | None = None,
        size_xyz_m: Sequence[float] | None = None,
    ) -> ProductRecord:
        """Add one product from eight HxWx3 uint8 views (azimuth 0, 45, ... 315).

        With a ``gtin`` the product id is the GTIN. Without one the id is ``product_id`` if
        given, else derived from the name and the views (``prod-<12 hex>``).
        """
        if gtin is not None:
            validate_gtin13(gtin)
            if product_id not in (None, gtin):
                raise RegistryError(f"{gtin}: a product with a GTIN uses it as its id")
            if self.by_gtin(gtin) is not None:
                raise RegistryError(f"duplicate GTIN {gtin}: already registered")
        if len(images) != VIEW_COUNT:
            raise RegistryError(f"{gtin or product_id or display_name}: need {VIEW_COUNT} views")
        staging = self.root / "products" / ".staging"
        if staging.exists():
            shutil.rmtree(staging)
        (staging / "views").mkdir(parents=True)
        try:
            views = []
            for index, image in enumerate(images):
                azimuth = index * VIEW_STEP_DEG
                name = view_file_name(azimuth)
                digest = write_png(staging / name, np.asarray(image, dtype=np.uint8))
                views.append(ViewRef(name, azimuth, digest))
            if gtin is not None:
                product_id = gtin
            elif product_id is None:
                product_id = derive_product_id(display_name, [v.sha256 for v in views])
            if product_id in self._products:
                raise RegistryError(f"duplicate product id {product_id}: already registered")
            for alias in aliases:
                owner = self.resolve(alias)
                if owner is not None:
                    raise RegistryError(f"alias {alias} already belongs to {owner}")
            record = ProductRecord(
                product_id=product_id,
                gtin=gtin,
                display_name=display_name,
                product_class=product_class,
                geometry_key=geometry_key,
                radius_m=float(radius_m),
                height_m=float(height_m),
                views=tuple(views),
                ocr_strings=tuple(ocr_strings),
                aliases=tuple(aliases),
                source=dict(source or {}),
                size_xyz_m=tuple(float(v) for v in size_xyz_m) if size_xyz_m is not None else None,
            )
            (staging / PRODUCT_NAME).write_text(
                json.dumps(record.to_dict(), indent=2, sort_keys=True) + "\n", encoding="utf-8"
            )
            directory = self.root / "products" / product_id
            if directory.exists():
                raise RegistryError(f"{directory} exists but {product_id} is not in the manifest")
            staging.rename(directory)
        except BaseException:
            shutil.rmtree(staging, ignore_errors=True)
            raise
        self._products[product_id] = record
        self._write_manifest()
        return record

    def remove(self, product_id: str) -> None:
        self.get(product_id)
        shutil.rmtree(self.root / "products" / product_id)
        del self._products[product_id]
        self._write_manifest()

    def set_notes(self, **notes: Any) -> None:
        self.notes.update(notes)
        self._write_manifest()

    def _write_manifest(self) -> None:
        manifest = {
            "schema_version": SCHEMA_VERSION,
            "view_count": VIEW_COUNT,
            "product_ids": self.product_ids,
            "content_sha256": self.content_hash(),
            "notes": self.notes,
        }
        (self.root / MANIFEST_NAME).write_text(
            json.dumps(manifest, indent=2, sort_keys=True) + "\n", encoding="utf-8"
        )

    # --- integrity ---------------------------------------------------------------------------

    def verify(self, *, expected_hash: str | None = None) -> list[str]:
        """Return everything wrong with the directory (empty list = intact)."""
        problems: list[str] = []
        seen_aliases: dict[str, str] = {}
        seen_gtins: dict[str, str] = {}
        for product_id in self.product_ids:
            record = self._products[product_id]
            if record.gtin is not None:
                if record.gtin in seen_gtins:
                    problems.append(f"GTIN {record.gtin} of {product_id} is not unique")
                seen_gtins[record.gtin] = product_id
            for alias in record.aliases:
                if alias in seen_aliases or alias in self._products:
                    problems.append(f"alias {alias} of {product_id} is not unique")
                seen_aliases[alias] = product_id
            for view in record.views:
                path = self.view_path(product_id, view)
                if not path.exists():
                    problems.append(f"{product_id}: missing {view.file}")
                elif hashlib.sha256(path.read_bytes()).hexdigest() != view.sha256:
                    problems.append(f"{product_id}: {view.file} does not match its recorded hash")
        if expected_hash is not None and expected_hash != self.content_hash():
            problems.append("manifest content_sha256 does not match the records")
        return problems

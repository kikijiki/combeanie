# Copyright 2026 Matteo Bernacchia <dev@kikijiki.com>
#
# SPDX-License-Identifier: MIT

"""Products without a barcode (CMB-118): internal ids, legacy registries, identity end to end."""
# Offline and deterministic: the fake embedding backend and scripted OCR / barcode readers, as in
# test_identity_channel.py. The end-to-end test registers a product with no GTIN, builds the
# calibration files, loads the service and identifies it, with the barcode channel on and off.

import json
from pathlib import Path
import sys

import numpy as np
import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from restocker_perception.identity import (  # noqa: E402,I100,I101 - ament and Ruff disagree
    ABSTAIN,
    IDENTIFIED,
    UNKNOWN,
    IdentityConfig,
    IdentityEngine,
    write_calibration,
)
from restocker_perception.identity.benchmark import save_whitener  # noqa: E402
from restocker_perception.identity.service import IdentifyService  # noqa: E402
from restocker_perception.registry import (  # noqa: E402
    DerivedCache,
    FakeEmbeddingBackend,
    Registry,
    RegistryError,
    RegistryIndex,
    Whitener,
)
from restocker_perception.registry import cli as registry_cli  # noqa: E402,I100
from restocker_perception.registry import render as rd  # noqa: E402
from restocker_perception.registry.gtin import InvalidGtin  # noqa: E402

GTIN = "2000000000015"
OTHER_GTIN = "2000000000022"


def views(seed: int) -> list[np.ndarray]:
    rng = np.random.default_rng(seed)
    image = np.tile(rng.uniform(0.1, 0.9, 3), (96, 192, 1)).astype(np.float32)
    for _ in range(8):
        x, y = int(rng.integers(0, 160)), int(rng.integers(0, 80))
        image[y : y + 12, x : x + 24] = rng.uniform(0, 1, 3)
    return rd.render_views(image, 0.033, 0.122)


def add(registry: Registry, seed: int, name: str, **extra):
    fields = {
        "display_name": name,
        "product_class": "can",
        "geometry_key": f"can.t{seed}",
        "radius_m": 0.033,
        "height_m": 0.122,
        "images": views(seed),
    }
    fields.update(extra)
    return registry.register(**fields)


@pytest.fixture
def mixed(tmp_path):
    """One product with a GTIN, two without."""
    registry = Registry.create(tmp_path / "reg")
    add(registry, 0, "Fennlo Mint Soda", gtin=GTIN, ocr_strings=["Fennlo", "Mint", "Soda"])
    add(registry, 1, "Loose Pear", ocr_strings=["Pear", "Conference"], aliases=["LOOSE-1"])
    add(registry, 2, "Rye Loaf", ocr_strings=["Rye", "Bakery"])
    return registry


# --- model -------------------------------------------------------------------------------------


def test_a_product_without_a_gtin_gets_a_stable_internal_id(mixed, tmp_path):
    pear = mixed.get(mixed.resolve("LOOSE-1"))
    assert pear.gtin is None
    assert pear.product_id.startswith("prod-") and len(pear.product_id) == len("prod-") + 12
    assert mixed.get(GTIN).product_id == GTIN
    # Deterministic from name and views, so a rebuilt registry gets the same id...
    again = Registry.create(tmp_path / "again")
    assert add(again, 1, "Loose Pear").product_id == pear.product_id
    # ...and write-once: it is stored, not recomputed, so it survives a reload.
    reloaded = Registry.load(mixed.root)
    assert reloaded.product_ids == mixed.product_ids
    stored = json.loads((mixed.root / "products" / pear.product_id / "product.json").read_text())
    assert stored["product_id"] == pear.product_id and stored["gtin"] is None
    assert reloaded.content_hash() == mixed.content_hash()


def test_gtin_is_optional_unique_and_validated(mixed):
    assert mixed.gtins == [GTIN]  # barcode-less products are not listed as GTINs
    assert mixed.gtin_map() == {GTIN: GTIN}
    assert mixed.by_gtin(GTIN) == GTIN and mixed.by_gtin(OTHER_GTIN) is None
    with pytest.raises(RegistryError, match="duplicate GTIN"):
        add(mixed, 5, "Clone", gtin=GTIN)
    with pytest.raises(InvalidGtin):
        add(mixed, 5, "Empty", gtin="")  # an empty string is never "no GTIN"
    with pytest.raises(InvalidGtin):
        add(mixed, 5, "Bad", gtin="2000000000016")
    with pytest.raises(RegistryError, match="uses it as its id"):
        add(mixed, 5, "Mismatch", gtin=OTHER_GTIN, product_id="prod-custom")
    with pytest.raises(RegistryError, match="numeric"):
        add(mixed, 5, "Looks like a GTIN", product_id=OTHER_GTIN)
    with pytest.raises(RegistryError, match="already registered"):
        add(mixed, 1, "Loose Pear")  # same name and views: same derived id


def test_an_explicit_product_id_is_kept(mixed):
    record = add(mixed, 6, "Sourdough", product_id="bakery-sourdough")
    assert record.product_id == "bakery-sourdough" and record.gtin is None
    assert Registry.load(mixed.root).get("bakery-sourdough").display_name == "Sourdough"
    with pytest.raises(RegistryError, match="bad product_id"):
        add(mixed, 7, "Escape", product_id="../x")


def test_resolve_accepts_id_gtin_and_alias(mixed):
    pear = mixed.resolve("LOOSE-1")
    assert pear is not None and pear.startswith("prod-")
    assert mixed.resolve(pear) == pear and mixed.resolve(GTIN) == GTIN
    assert mixed.resolve("nope") is None


def test_the_cache_is_keyed_by_the_internal_id(mixed):
    backend = FakeEmbeddingBackend()
    cache = DerivedCache(mixed.root / ".cache")
    pear = mixed.resolve("LOOSE-1")
    cache.embeddings(mixed, backend, pear)
    cache.embeddings(mixed, backend, GTIN)
    names = {p.name for p in (mixed.root / ".cache" / "embeddings").rglob("*.npz")}
    assert names == {f"{pear}.npz", f"{GTIN}.npz"}
    cache.embeddings(mixed, backend, pear)
    assert cache.hits == 1


# --- legacy (schema 1, GTIN-keyed) registries ----------------------------------------------------


def _as_schema_1(registry: Registry) -> None:
    """Rewrite the manifest the way a registry written before product ids had it."""
    path = registry.root / "registry.json"
    manifest = json.loads(path.read_text())
    manifest["schema_version"] = 1
    manifest["gtins"] = manifest.pop("product_ids")
    path.write_text(json.dumps(manifest))


def test_a_gtin_keyed_registry_loads_unchanged_and_keeps_its_ids(tmp_path):
    registry = Registry.create(tmp_path / "old")
    add(registry, 0, "A", gtin=GTIN, aliases=["GEN-001"])
    add(registry, 3, "B", gtin=OTHER_GTIN, aliases=["GEN-002"])
    digest = registry.content_hash()
    record = json.loads((registry.root / "products" / GTIN / "product.json").read_text())
    assert "product_id" not in record  # byte-compatible with what schema 1 wrote
    _as_schema_1(registry)
    loaded = Registry.load(tmp_path / "old")
    assert loaded.product_ids == [GTIN, OTHER_GTIN] == loaded.gtins
    assert loaded.content_hash() == digest  # same hash, so calibrations stay valid
    assert loaded.resolve("GEN-002") == OTHER_GTIN
    # Caches written under the GTIN are the ones the product id asks for.
    backend = FakeEmbeddingBackend()
    cache = DerivedCache(loaded.root / ".cache")
    cache.embeddings(loaded, backend, GTIN)
    assert cache.misses == 1
    cache.embeddings(Registry.load(tmp_path / "old"), backend, GTIN)
    assert cache.hits == 1


def test_adding_a_barcode_less_product_upgrades_a_legacy_registry(tmp_path):
    registry = Registry.create(tmp_path / "old")
    add(registry, 0, "A", gtin=GTIN)
    _as_schema_1(registry)
    loaded = Registry.load(tmp_path / "old")
    pear = add(loaded, 1, "Loose Pear")
    manifest = json.loads((tmp_path / "old" / "registry.json").read_text())
    assert manifest["schema_version"] == 2
    assert manifest["product_ids"] == sorted([GTIN, pear.product_id])
    assert Registry.load(tmp_path / "old").gtins == [GTIN]


def test_a_damaged_legacy_registry_is_still_refused(tmp_path):
    registry = Registry.create(tmp_path / "old")
    add(registry, 0, "A", gtin=GTIN)
    _as_schema_1(registry)
    (tmp_path / "old" / "products" / GTIN / "views" / "az000.png").write_bytes(b"x")
    with pytest.raises(RegistryError, match="damaged"):
        Registry.load(tmp_path / "old")


def test_a_record_with_neither_id_nor_gtin_is_refused(tmp_path):
    registry = Registry.create(tmp_path / "bad")
    pear = add(registry, 1, "Loose Pear")
    path = tmp_path / "bad" / "products" / pear.product_id / "product.json"
    data = json.loads(path.read_text())
    del data["product_id"]
    path.write_text(json.dumps(data))
    with pytest.raises(RegistryError, match="malformed"):
        Registry.load(tmp_path / "bad", verify=False)


def test_the_cli_registers_and_lists_a_product_without_a_gtin(tmp_path, capsys):
    from restocker_perception.registry.imageio import write_png  # noqa: PLC0415

    directory = tmp_path / "pear"
    directory.mkdir()
    for index, image in enumerate(views(1)):
        write_png(directory / f"az{index * 45:03d}.png", image)
    common = ["--registry", str(tmp_path / "reg")]
    assert (
        registry_cli.main(
            ["add-images", *common, "--images", str(directory), "--name", "Loose Pear"]
            + ["--product-class", "can", "--geometry-key", "can.t1", "--radius", "0.03"]
            + ["--height", "0.1"]
        )
        == 0
    )
    assert registry_cli.main(["info", *common]) == 0
    out = capsys.readouterr().out
    assert "prod-" in out and "gtin -" in out
    assert registry_cli.main(["verify", *common]) == 0


# --- identity ------------------------------------------------------------------------------------


class ScriptedOcr:
    name = "scripted-ocr"
    version = "1"

    def read(self, image: np.ndarray) -> list[str]:
        return {7: ["Loose", "Pear", "Conference"]}.get(int(image[0, 0, 0]), [])


class ScriptedBarcode:
    name = "scripted-barcode"
    version = "1"

    def __init__(self, text: str) -> None:
        self.text = text

    def decode(self, image: np.ndarray) -> list[str]:
        return [self.text]


def tagged(image: np.ndarray, tag: int) -> np.ndarray:
    out = image.copy()
    out[0, 0, 0] = tag
    return out


def engine_for(registry, *, barcode=None, barcode_enabled=True, ocr=True, config=None):
    backend = FakeEmbeddingBackend()
    index = RegistryIndex.from_registry(registry, backend)
    return IdentityEngine(
        index,
        backend,
        config or IdentityConfig(supported_max_rank=1),
        ocr_backend=ScriptedOcr() if ocr else None,
        ocr_strings={r.product_id: list(r.ocr_strings) for r in registry.records()},
        barcode_backend=ScriptedBarcode(barcode) if barcode else None,
        gtin_to_product=registry.gtin_map(),
        barcode_enabled=barcode_enabled,
    )


def test_a_barcode_less_product_is_named_by_ocr_and_embedding(mixed):
    pear = mixed.resolve("LOOSE-1")
    crop = tagged(mixed.view_images(pear)[2], 7)
    result = engine_for(mixed).identify([crop])
    assert (result.status, result.sku) == (IDENTIFIED, pear)
    assert {e.kind for e in result.evidence} == {"embedding", "ocr"}
    assert result.channels == ("embedding", "ocr")
    assert all(e.product_id == pear for e in result.evidence)


def test_a_barcode_less_product_is_named_by_the_embedding_alone_when_calibrated(mixed):
    config = IdentityConfig(alone_min_score=0.0, alone_min_margin=0.0, alone_min_vote=1.0)
    rye = next(r.product_id for r in mixed.records() if r.display_name == "Rye Loaf")
    result = engine_for(mixed, ocr=False, config=config).identify([mixed.view_images(rye)[1]])
    assert (result.status, result.sku) == (IDENTIFIED, rye)
    assert result.channels == ("embedding",)


def test_the_barcode_channel_names_only_products_that_have_a_gtin(mixed):
    pear = mixed.resolve("LOOSE-1")
    images = [np.zeros((16, 16, 3), np.uint8)]
    eng = engine_for(mixed, barcode=GTIN, ocr=False)
    named = eng.identify([mixed.view_images(GTIN)[0]], barcode_images=images)
    assert (named.status, named.sku) == (IDENTIFIED, GTIN)
    assert named.channels == ("embedding", "barcode")
    # A barcode cannot name the barcode-less product the embedding is looking at.
    other = eng.identify([mixed.view_images(pear)[0]], barcode_images=images)
    assert other.status == ABSTAIN and other.sku is None


def test_a_valid_but_unregistered_gtin_is_unknown_only_while_the_barcode_channel_is_on(mixed):
    pear = mixed.resolve("LOOSE-1")
    crop = [tagged(mixed.view_images(pear)[2], 7)]
    images = [np.zeros((16, 16, 3), np.uint8)]
    on = engine_for(mixed, barcode=OTHER_GTIN).identify(crop, barcode_images=images)
    assert on.status == UNKNOWN
    off = engine_for(mixed, barcode=OTHER_GTIN, barcode_enabled=False)
    result = off.identify(crop, barcode_images=images)
    assert (result.status, result.sku) == (IDENTIFIED, pear)
    assert "barcode" not in result.channels
    assert result.provenance["barcode_enabled"] is False and result.provenance["barcode"] is None


def test_barcode_off_names_a_gtin_product_without_looking_at_the_barcode(mixed):
    eng = engine_for(mixed, barcode=GTIN, barcode_enabled=False, ocr=False)
    images = [np.zeros((8, 8, 3), np.uint8)]
    result = eng.identify([mixed.view_images(GTIN)[0]], barcode_images=images)
    assert result.status == ABSTAIN  # nothing but the embedding, and it never names alone
    assert result.channels == ("embedding",)


def test_a_registry_without_gtins_never_runs_the_barcode_channel(tmp_path):
    registry = Registry.create(tmp_path / "reg")
    pear = add(registry, 1, "Loose Pear", ocr_strings=["Pear", "Conference"]).product_id
    eng = engine_for(registry, barcode=GTIN)
    result = eng.identify(
        [tagged(registry.view_images(pear)[0], 7)], barcode_images=[np.zeros((8, 8, 3), np.uint8)]
    )
    assert "barcode" not in result.channels


def test_the_default_gtin_map_covers_exactly_the_gtin_ids(mixed):
    backend = FakeEmbeddingBackend()
    eng = IdentityEngine(RegistryIndex.from_registry(mixed, backend), backend, IdentityConfig())
    assert eng.gtin_to_product == {GTIN: GTIN}
    assert eng.barcode_enabled is True


# --- the service, end to end ---------------------------------------------------------------------


@pytest.fixture
def deployment(mixed, tmp_path):
    backend = FakeEmbeddingBackend()
    pool = np.concatenate([backend.embed(mixed.view_images(p)) for p in mixed.product_ids])
    whitener = Whitener.fit(pool, 8)
    save_whitener(tmp_path / "w.npz", whitener)
    fit = {
        "backend": f"{backend.name} {backend.version}",
        "registry_sha256": mixed.content_hash(),
        "whitening": {"fingerprint": whitener.fingerprint},
    }
    config = IdentityConfig(alone_min_score=0.0, alone_min_margin=0.0, alone_min_vote=1.0)
    write_calibration(tmp_path / "cal.json", config, fit)
    return backend


def load_service(mixed, tmp_path, backend, **options):
    return IdentifyService.load(
        str(mixed.root), str(tmp_path / "cal.json"), str(tmp_path / "w.npz"), backend, **options
    )


def test_register_and_identify_a_product_with_no_gtin_end_to_end(mixed, tmp_path, deployment):
    options = {"ocr_backend": ScriptedOcr(), "barcode_backend": ScriptedBarcode(GTIN)}
    service = load_service(mixed, tmp_path, deployment, **options)
    assert service.available
    pear = mixed.resolve("LOOSE-1")
    reply = service.identify([tagged(mixed.view_images(pear)[4], 7)])
    assert reply["status"] == 0 and reply["has_sku"] and reply["sku"] == pear
    assert reply["has_gtin"] is False and reply["gtin"] == ""
    assert reply["evidence_kinds"] == ["embedding", "ocr"]
    assert reply["evidence_product_ids"] == [pear, pear]
    assert reply["evidence_gtins"] == ["", ""]  # no GTIN: empty, never a made-up value
    assert reply["channels_used"] == ["embedding", "ocr"] and reply["barcode_enabled"] is True
    json.dumps(reply)


def test_the_service_still_exposes_the_gtin_as_sku_for_existing_products(
    mixed, tmp_path, deployment
):
    service = load_service(mixed, tmp_path, deployment)
    reply = service.identify([mixed.view_images(GTIN)[3]])
    assert reply["sku"] == GTIN and reply["has_gtin"] and reply["gtin"] == GTIN
    assert reply["evidence_product_ids"] == [GTIN] and reply["evidence_gtins"] == [GTIN]


def test_the_service_barcode_switch_is_reported_and_honoured(mixed, tmp_path, deployment):
    images = [np.zeros((16, 16, 3), np.uint8)]
    pear = mixed.resolve("LOOSE-1")
    crop = [tagged(mixed.view_images(pear)[4], 7)]
    options = {"ocr_backend": ScriptedOcr(), "barcode_backend": ScriptedBarcode(OTHER_GTIN)}
    on = load_service(mixed, tmp_path, deployment, **options).identify(crop, images)
    assert on["status"] == 2 and "barcode" in on["channels_used"]
    off_service = load_service(mixed, tmp_path, deployment, barcode_enabled=False, **options)
    off = off_service.identify(crop, images)
    assert off["status"] == 0 and off["sku"] == pear
    assert off["barcode_enabled"] is False and "barcode" not in off["channels_used"]

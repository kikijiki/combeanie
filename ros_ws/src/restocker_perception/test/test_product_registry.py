# Copyright 2026 Matteo Bernacchia <dev@kikijiki.com>
#
# SPDX-License-Identifier: MIT

"""Product registry: format, integrity, registration, rendering, cache and retrieval."""
# Pure Python (numpy, PyYAML), no simulator, no ML dependency: embeddings come from the
# deterministic fake backend. The shipped catalogue, geometry catalogue, textures, mesh and the
# frozen benchmark split file are read from the paths the build passes in RESTOCKER_TEST_*.

import hashlib
import json
import os
from pathlib import Path
import struct
import sys
import zlib

import numpy as np
import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from restocker_perception.registry import (  # noqa: E402,I100,I101 - ament and Ruff disagree
    DerivedCache,
    FakeEmbeddingBackend,
    InvalidGtin,
    Registry,
    RegistryError,
    RegistryIndex,
    SigLip2Backend,
    Whitener,
    cli,  # noqa: E402
    validate_gtin13,
)
from restocker_perception.registry import imageio as io  # noqa: E402
from restocker_perception.registry import render as rd  # noqa: E402
from restocker_perception.registry import simulated as sim  # noqa: E402

SRC = Path(__file__).resolve().parents[2]


def _path(variable: str, default: Path) -> Path:
    return Path(os.environ.get(variable, default))


CATALOGUE = _path(
    "RESTOCKER_TEST_GENERATED_LABELS",
    SRC / "restocker_gazebo/config/generated_product_labels.yaml",
)
GEOMETRY = _path(
    "RESTOCKER_TEST_PRODUCT_CATALOG",
    SRC / "restocker_description/config/product_collision_catalog.yaml",
)
TEXTURES = _path("RESTOCKER_TEST_TEXTURES", SRC / "restocker_gazebo/materials/textures")
MESH_DIR = _path("RESTOCKER_TEST_MESHES", SRC / "restocker_gazebo/meshes")
SPLITS = _path(
    "RESTOCKER_TEST_PERCEPTION_SPLITS",
    SRC / "restocker_benchmarks/config/perception_splits_v3.json",
)

GTINS = ["2000000000015", "2000000000022", "2000000000039", "2000000000046"]


def synthetic_texture(seed: int, width: int = 192, height: int = 96) -> np.ndarray:
    """A float texture with a distinct colour scheme and strokes per seed, periodic in u."""
    rng = np.random.default_rng(seed)
    base = rng.uniform(0.15, 0.85, 3)
    image = np.tile(base, (height, width, 1)).astype(np.float32)
    for _ in range(8):
        x0 = int(rng.integers(0, width - 24))
        y0 = int(rng.integers(0, height - 16))
        image[y0 : y0 + int(rng.integers(6, 16)), x0 : x0 + int(rng.integers(8, 24))] = (
            rng.uniform(0, 1, 3)
        )
    return image


def synthetic_views(seed: int) -> list[np.ndarray]:
    return rd.render_views(synthetic_texture(seed), 0.033, 0.122)


def register(registry: Registry, gtin: str, seed: int, **extra) -> None:
    fields = {
        "gtin": gtin,
        "display_name": f"Product {seed}",
        "product_class": "can",
        "geometry_key": f"can.test{seed}",
        "radius_m": 0.033,
        "height_m": 0.122,
        "images": synthetic_views(seed),
    }
    fields.update(extra)
    registry.register(**fields)


@pytest.fixture(scope="module")
def views_by_seed():
    return {seed: synthetic_views(seed) for seed in range(4)}


@pytest.fixture
def registry(tmp_path, views_by_seed):
    reg = Registry.create(tmp_path / "reg")
    for seed, gtin in enumerate(GTINS):
        reg.register(
            gtin=gtin,
            display_name=f"Product {seed}",
            product_class="can",
            geometry_key=f"can.test{seed}",
            radius_m=0.033,
            height_m=0.122,
            images=views_by_seed[seed],
            aliases=[f"T-{seed}"],
        )
    return reg


# --- GTIN ------------------------------------------------------------------------------------


def test_every_catalogue_gtin_is_valid():
    rows = sim.load_catalogue_rows(CATALOGUE)
    assert len(rows) == 20
    for row in rows:
        if row["shape"] == "box" and not row["barcode"]:
            assert "gtin" not in row  # a product with no barcode panel has no GTIN at all
            continue
        assert validate_gtin13(row["gtin"]) == row["gtin"]


@pytest.mark.parametrize(
    "bad", ["2000000000016", "200000000001", "20000000000150", "abcdefghijklm", "", None]
)
def test_bad_gtin_is_refused(bad):
    with pytest.raises(InvalidGtin):
        validate_gtin13(bad)


# --- format, hash, integrity -------------------------------------------------------------------


def test_register_and_reload_round_trip(registry):
    loaded = Registry.load(registry.root)
    assert loaded.product_ids == sorted(GTINS)
    assert loaded.content_hash() == registry.content_hash()
    record = loaded.get(GTINS[0])
    assert [v.nominal_azimuth_deg for v in record.views] == [0, 45, 90, 135, 180, 225, 270, 315]
    assert (loaded.root / "products" / GTINS[0] / "views" / "az045.png").exists()
    assert loaded.resolve("T-2") == GTINS[2]
    images = loaded.view_images(GTINS[1])
    assert len(images) == 8 and images[0].dtype == np.uint8


def test_duplicate_gtin_is_refused(registry, views_by_seed):
    before = registry.content_hash()
    with pytest.raises(RegistryError, match="duplicate GTIN"):
        register(registry, GTINS[0], 9)
    assert registry.content_hash() == before
    assert not list((registry.root / "products").glob(".*staging"))


def test_bad_check_digit_is_refused_on_register(registry):
    with pytest.raises(InvalidGtin):
        register(registry, "2000000000016", 9)
    assert len(registry) == 4


def test_duplicate_alias_wrong_view_count_and_bad_fields_are_refused(registry):
    with pytest.raises(RegistryError, match="alias"):
        register(registry, "2000000000053", 5, aliases=["T-1"])
    with pytest.raises(RegistryError, match="need 8 views"):
        register(registry, "2000000000053", 5, images=synthetic_views(5)[:7])
    with pytest.raises(RegistryError, match="radius_m"):
        register(registry, "2000000000053", 5, radius_m=0.0)
    with pytest.raises(RegistryError, match="display_name"):
        register(registry, "2000000000053", 5, display_name=" ")
    assert len(registry) == 4


def test_content_hash_tracks_content_not_order(tmp_path, views_by_seed):
    a, b = Registry.create(tmp_path / "a"), Registry.create(tmp_path / "b")
    for gtin, seed in zip(GTINS[:2], (0, 1), strict=True):
        register(a, gtin, seed)
    for gtin, seed in reversed(list(zip(GTINS[:2], (0, 1), strict=True))):
        register(b, gtin, seed)
    assert a.content_hash() == b.content_hash()
    register(a, GTINS[2], 2)
    assert a.content_hash() != b.content_hash()
    register(b, GTINS[2], 2, display_name="Renamed")
    assert a.content_hash() != b.content_hash()


def test_damaged_registries_are_refused(registry):
    view = registry.root / "products" / GTINS[0] / "views" / "az090.png"
    original = view.read_bytes()
    view.write_bytes(original + b"x")
    with pytest.raises(RegistryError, match="does not match its recorded hash"):
        Registry.load(registry.root)
    view.write_bytes(original)
    Registry.load(registry.root)
    view.unlink()
    with pytest.raises(RegistryError, match="missing"):
        Registry.load(registry.root)
    view.write_bytes(original)
    manifest = registry.root / "registry.json"
    document = json.loads(manifest.read_text())
    document["content_sha256"] = "0" * 64
    manifest.write_text(json.dumps(document))
    with pytest.raises(RegistryError, match="content_sha256"):
        Registry.load(registry.root)


def test_yaml_product_manifest_is_read(registry):
    directory = registry.root / "products" / GTINS[3]
    data = json.loads((directory / "product.json").read_text())
    (directory / "product.json").unlink()
    import yaml

    (directory / "product.yaml").write_text(yaml.safe_dump(data))
    assert Registry.load(registry.root).content_hash() == registry.content_hash()


def test_remove_and_create_twice(registry):
    with pytest.raises(RegistryError, match="already holds"):
        Registry.create(registry.root)
    registry.remove(GTINS[0])
    assert GTINS[0] not in Registry.load(registry.root)


# --- images ----------------------------------------------------------------------------------


def _png_with_filters(pixels: np.ndarray) -> bytes:
    """A PNG that uses every filter type, built independently of the decoder under test."""
    height, width, channels = pixels.shape
    stride = width * channels
    raw, previous = b"", bytes(stride)
    for row in range(height):
        line = pixels[row].reshape(-1).tobytes()
        mode = row % 5
        out = bytearray(stride)
        for i in range(stride):
            a = line[i - channels] if i >= channels else 0
            b = previous[i]
            c = previous[i - channels] if i >= channels else 0
            if mode == 0:
                predictor = 0
            elif mode == 1:
                predictor = a
            elif mode == 2:
                predictor = b
            elif mode == 3:
                predictor = (a + b) // 2
            else:
                p = a + b - c
                pa, pb, pc = abs(p - a), abs(p - b), abs(p - c)
                predictor = a if pa <= pb and pa <= pc else (b if pb <= pc else c)
            out[i] = (line[i] - predictor) & 0xFF
        raw += bytes([mode]) + bytes(out)
        previous = line

    def chunk(kind: bytes, payload: bytes) -> bytes:
        body = kind + payload
        return struct.pack(">I", len(payload)) + body + struct.pack(">I", zlib.crc32(body))

    colour = {1: 0, 3: 2, 4: 6}[channels]
    return (
        b"\x89PNG\r\n\x1a\n"
        + chunk(b"IHDR", struct.pack(">IIBBBBB", width, height, 8, colour, 0, 0, 0))
        + chunk(b"IDAT", zlib.compress(raw))
        + chunk(b"IEND", b"")
    )


@pytest.mark.parametrize("channels", [1, 3, 4])
def test_png_decoder_handles_every_filter(channels):
    rng = np.random.default_rng(3)
    pixels = rng.integers(0, 256, (11, 13, channels), dtype=np.uint8)
    decoded = io.decode_png(_png_with_filters(pixels))
    expected = np.repeat(pixels, 3, axis=2) if channels == 1 else pixels[:, :, :3]
    assert np.array_equal(decoded, expected)


def test_png_writer_round_trips_and_is_deterministic():
    pixels = np.random.default_rng(1).integers(0, 256, (9, 7, 3), dtype=np.uint8)
    data = io.encode_png(pixels)
    assert data == io.encode_png(pixels)
    assert np.array_equal(io.decode_png(data), pixels)


def test_area_resize_preserves_mean_and_shape():
    image = np.random.default_rng(2).integers(0, 256, (40, 30, 3), dtype=np.uint8)
    small = io.resize_area(image, 10, 8)
    assert small.shape == (8, 10, 3)
    assert abs(float(small.mean()) - float(image.mean())) < 1.0


# --- rendering ---------------------------------------------------------------------------------


def test_render_is_deterministic_and_views_differ():
    texture = synthetic_texture(0)
    first = rd.render_views(texture, 0.033, 0.122)
    again = rd.render_views(texture, 0.033, 0.122)
    assert len(first) == 8
    for a, b in zip(first, again, strict=True):
        assert np.array_equal(a, b)
    hashes = {hashlib.sha256(v.tobytes()).hexdigest() for v in first}
    assert len(hashes) == 8
    # A tight crop: the body fills the frame width, the corners are background.
    assert first[0].shape[0] > first[0].shape[1]
    assert abs(float(first[0][0, 0, 0]) - 127.5) < 1.0


def test_render_turns_the_barrel_by_yaw():
    texture = synthetic_texture(1)
    full_turn = rd.render_view(texture, 0.033, 0.122, 360.0)
    assert np.array_equal(full_turn, rd.render_view(texture, 0.033, 0.122, 0.0))
    # 180 degrees shows what the camera did not see at 0.
    assert not np.array_equal(
        rd.render_view(texture, 0.033, 0.122, 180.0), rd.render_view(texture, 0.033, 0.122, 0.0)
    )


def test_profile_render_follows_the_waist():
    heights = np.linspace(-0.1, 0.1, 21)
    radii = 0.035 - 0.005 * np.cos(heights / 0.1 * np.pi)  # fat ends, thin middle
    profile = rd.Profile(heights, radii)
    image = rd.render_view(synthetic_texture(2), 0.04, 0.2, 0.0, profile=profile)
    body = np.abs(image.astype(int) - 127).sum(axis=2) > 6
    widths = body.sum(axis=1)
    middle, top = widths[len(widths) // 2], widths[len(widths) // 8]
    assert top > middle * 1.15


@pytest.mark.skipif(not (TEXTURES / "gen-001.png").exists(), reason="shipped textures not found")
def test_shipped_texture_and_mesh_render():
    rows = {r["sku"]: r for r in sim.load_catalogue_rows(CATALOGUE)}
    geometry = sim.load_geometry_catalogue(GEOMETRY)
    can = geometry[rows["GEN-001"]["geometry_key"]]
    view = rd.render_view(
        rd.load_texture(TEXTURES / "gen-001.png"), can["radius_m"], can["height_m"], 0.0
    )
    assert view.shape[2] == 3 and view.std() > 10
    obj = MESH_DIR / "contour_bottle.obj"
    profile = rd.profile_from_obj(obj)
    assert profile.heights[0] < 0 < profile.heights[-1]
    assert 0.03 < profile.max_radius < 0.04
    bottle = geometry[rows["GEN-013"]["geometry_key"]]
    mesh_view = rd.render_view(
        rd.load_texture(TEXTURES / "gen-013.png"),
        bottle["radius_m"],
        bottle["height_m"],
        0.0,
        profile=profile,
    )
    assert mesh_view.std() > 10


# --- derived cache ---------------------------------------------------------------------------


def test_embedding_cache_hits_and_keys_on_backend(registry):
    backend = FakeEmbeddingBackend(dim=24, seed=1)
    cache = DerivedCache(registry.root / ".cache")
    first = cache.embeddings(registry, backend, GTINS[0])
    assert (cache.hits, cache.misses, backend.calls) == (0, 1, 8)
    again = DerivedCache(registry.root / ".cache")
    assert np.array_equal(again.embeddings(registry, backend, GTINS[0]), first)
    assert (again.hits, again.misses, backend.calls) == (1, 0, 8)
    assert first.shape == (8, 24)
    assert np.allclose(np.linalg.norm(first, axis=1), 1.0, atol=1e-5)


def test_cache_invalidates_on_backend_version_and_name(registry):
    cache = DerivedCache(registry.root / ".cache")
    base = FakeEmbeddingBackend(dim=24, seed=1)
    cache.embeddings(registry, base, GTINS[0])
    newer = FakeEmbeddingBackend(dim=24, seed=2)  # same name, new version
    assert newer.name == base.name and newer.version != base.version
    cache.embeddings(registry, newer, GTINS[0])
    assert (cache.hits, cache.misses) == (0, 2)
    cache.embeddings(registry, base, GTINS[0])  # old version is still its own valid entry
    assert cache.hits == 1

    class Renamed(FakeEmbeddingBackend):
        name = "another-encoder"

    cache.embeddings(registry, Renamed(dim=24, seed=1), GTINS[0])
    assert cache.misses == 3
    assert len(list((registry.root / ".cache" / "embeddings").iterdir())) == 2


def test_cache_invalidates_when_the_views_change(registry, views_by_seed):
    backend = FakeEmbeddingBackend(dim=24)
    cache = DerivedCache(registry.root / ".cache")
    old = cache.embeddings(registry, backend, GTINS[0])
    registry.remove(GTINS[0])
    register(registry, GTINS[0], 3)  # same GTIN, different pictures
    new = cache.embeddings(registry, backend, GTINS[0])
    assert cache.misses == 2 and cache.hits == 0
    assert not np.allclose(old, new)


def test_corrupt_cache_entry_is_recomputed(registry):
    backend = FakeEmbeddingBackend(dim=24)
    cache = DerivedCache(registry.root / ".cache")
    first = cache.embeddings(registry, backend, GTINS[0])
    entry = next((registry.root / ".cache" / "embeddings").rglob("*.npz"))
    entry.write_bytes(b"garbage")
    assert np.array_equal(cache.embeddings(registry, backend, GTINS[0]), first)
    assert cache.misses == 2


def test_ocr_cache_is_keyed_like_embeddings(registry):
    class Reader:
        name, version = "fake-ocr", "1"
        calls = 0

        def read(self, image):
            Reader.calls += 1
            return ["WORD"]

    cache = DerivedCache(registry.root / ".cache")
    assert cache.ocr_reads(registry, Reader(), GTINS[0]) == [["WORD"]] * 8
    cache.ocr_reads(registry, Reader(), GTINS[0])
    assert Reader.calls == 8 and cache.hits == 1
    Reader.version = "2"
    cache.ocr_reads(registry, Reader(), GTINS[0])
    assert Reader.calls == 16


# --- retrieval ---------------------------------------------------------------------------------


def _query(seed: int, yaw: float) -> np.ndarray:
    return rd.render_view(synthetic_texture(seed), 0.033, 0.122, yaw)


def test_retrieval_names_the_product_from_an_off_grid_view(registry):
    backend = FakeEmbeddingBackend()
    index = RegistryIndex.from_registry(registry, backend)
    for seed, gtin in enumerate(GTINS):
        result = index.identify([_query(seed, 20.0)], backend)
        assert result.top1.product_id == gtin
        assert result.top1.best_view_deg in range(0, 360, 45)
        assert result.margin > 0
        assert result.top2.product_id != gtin
        assert [m.product_id for m in result.top_k(2)] == [
            result.top1.product_id,
            result.top2.product_id,
        ]
        assert result.registry_sha256 == registry.content_hash()
        assert (result.backend_name, result.backend_version) == (backend.name, backend.version)
        scores = [m.score for m in result.ranked]
        assert scores == sorted(scores, reverse=True) and len(scores) == 4


def test_exact_exemplar_scores_one_and_reports_its_view(registry):
    backend = FakeEmbeddingBackend()
    index = RegistryIndex.from_registry(registry, backend)
    result = index.identify([_query(1, 135.0)], backend)
    assert result.top1.product_id == GTINS[1]
    assert result.top1.score == pytest.approx(1.0, abs=1e-4)
    assert result.top1.best_view_deg == 135


def test_voting_averages_views_and_reports_agreement(registry):
    backend = FakeEmbeddingBackend()
    index = RegistryIndex.from_registry(registry, backend)
    queries = [_query(2, 10.0), _query(2, 100.0), _query(2, 200.0)]
    result = index.identify(queries, backend)
    assert result.n_queries == 3 and result.top1.product_id == GTINS[2]
    assert result.vote_agreement >= 2 / 3
    mixed = index.identify([_query(2, 10.0), _query(0, 10.0)], backend)
    assert mixed.n_queries == 2 and mixed.vote_agreement == 0.5


@pytest.mark.parametrize("scoring", ["max", "mean", "top2"])
def test_scoring_modes_rank_the_right_product(registry, scoring):
    backend = FakeEmbeddingBackend()
    index = RegistryIndex.from_registry(registry, backend)
    assert index.identify([_query(3, 0.0)], backend, scoring=scoring).top1.product_id == GTINS[3]
    with pytest.raises(ValueError, match="scoring"):
        index.identify([_query(3, 0.0)], backend, scoring="bogus")


def test_margin_is_none_for_a_single_product_and_exclude_works(registry):
    backend = FakeEmbeddingBackend()
    index = RegistryIndex.from_registry(registry, backend, exclude=[GTINS[0], "T-1", GTINS[2]])
    assert index.product_ids == [GTINS[3]]
    result = index.identify([_query(3, 30.0)], backend)
    assert result.margin is None and result.top2 is None
    with pytest.raises(ValueError, match="no products"):
        RegistryIndex.from_registry(registry, backend, exclude=GTINS)


def test_a_query_of_an_excluded_product_does_not_match_it(registry):
    backend = FakeEmbeddingBackend()
    index = RegistryIndex.from_registry(registry, backend, exclude=[GTINS[0]])
    result = index.identify([_query(0, 0.0)], backend)
    assert GTINS[0] not in result.scores()


def test_index_refuses_a_different_backend(registry):
    index = RegistryIndex.from_registry(registry, FakeEmbeddingBackend(seed=1))
    with pytest.raises(ValueError, match="query backend"):
        index.identify([_query(0, 0.0)], FakeEmbeddingBackend(seed=2))


def test_whitening_is_applied_to_prototypes_and_queries(registry):
    backend = FakeEmbeddingBackend()
    pool = np.concatenate(
        [backend.embed([_query(s, y) for y in range(0, 360, 30)]) for s in range(4)]
    )
    whitener = Whitener.fit(pool, k=16)
    index = RegistryIndex.from_registry(registry, backend, whitener=whitener)
    result = index.identify([_query(2, 20.0)], backend)
    assert result.top1.product_id == GTINS[2]
    assert result.whitening == whitener.fingerprint
    plain = RegistryIndex.from_registry(registry, backend).identify([_query(2, 20.0)], backend)
    assert plain.whitening is None
    with pytest.raises(ValueError):
        Whitener.fit(pool, k=pool.shape[0])


def test_fake_backend_is_deterministic_across_instances():
    image = _query(0, 0.0)
    a = FakeEmbeddingBackend(seed=4).embed([image])
    b = FakeEmbeddingBackend(seed=4).embed([image])
    assert np.array_equal(a, b)


def test_siglip2_adapter_is_lazy():
    adapter = SigLip2Backend(revision="abc123")
    assert adapter.name == "siglip2" and adapter.version.endswith("@abc123")
    try:
        import torch  # noqa: F401
        import transformers  # noqa: F401
    except ImportError:
        with pytest.raises(RuntimeError, match="not part of the flake"):
            adapter.embed([_query(0, 0.0)])
    else:
        pytest.skip("torch and transformers installed: the adapter would download weights")


# --- simulated registration and held-out exclusion ------------------------------------------


@pytest.fixture(scope="module")
def synthetic_textures(tmp_path_factory):
    directory = tmp_path_factory.mktemp("textures")
    for index, row in enumerate(sim.load_catalogue_rows(CATALOGUE)):
        texture = (synthetic_texture(100 + index) * 255).astype(np.uint8)
        (directory / row["texture"]).write_bytes(io.encode_png(texture))
    return directory


def test_held_out_products_are_never_registered(tmp_path, synthetic_textures):
    splits = sim.load_split_file(SPLITS)
    held = sim.held_out_products(splits)
    assert held
    reg = Registry.create(tmp_path / "reg")
    exclude = set(held)
    added = sim.register_simulated(
        reg,
        sim.load_catalogue_rows(CATALOGUE),
        sim.load_geometry_catalogue(GEOMETRY),
        synthetic_textures,
        exclude=exclude,
    )
    assert len(added) == len(sim.load_catalogue_rows(CATALOGUE)) - len(held)
    assert sim.check_registry_against_splits(reg, splits) == []
    aliases = {a for r in reg.records() for a in r.aliases}
    assert aliases.isdisjoint(held)
    # Without exclusion the guard catches the leak.
    leaky = Registry.create(tmp_path / "leaky")
    sim.register_simulated(
        leaky,
        sim.load_catalogue_rows(CATALOGUE),
        sim.load_geometry_catalogue(GEOMETRY),
        synthetic_textures,
        only=held[:1],
    )
    problems = sim.check_registry_against_splits(leaky, splits)
    assert len(problems) == 1 and held[0] in problems[0]


def test_tampered_split_file_is_refused(tmp_path):
    document = json.loads(SPLITS.read_text())
    document["products"]["held_out"] = []
    path = tmp_path / "splits.json"
    path.write_text(json.dumps(document))
    with pytest.raises(RegistryError, match="sha256"):
        sim.load_split_file(path)


def test_registration_cli_simulated_with_exclusion_and_verify(
    tmp_path, synthetic_textures, capsys
):
    root = tmp_path / "reg"
    argv = [
        "add-sim",
        "--registry",
        str(root),
        "--catalogue",
        str(CATALOGUE),
        "--geometry",
        str(GEOMETRY),
        "--textures",
        str(synthetic_textures),
        "--exclude-held-out",
        str(SPLITS),
    ]
    assert cli.main(argv) == 0
    registry = Registry.load(root)
    splits = sim.load_split_file(SPLITS)
    assert len(registry) == len(sim.load_catalogue_rows(CATALOGUE)) - len(
        sim.held_out_products(splits)
    )
    assert registry.notes["splits_sha256"] == splits["sha256"]
    assert registry.notes["excluded_held_out"] == sim.held_out_products(splits)
    assert cli.main(["verify", "--registry", str(root), "--splits", str(SPLITS)]) == 0
    assert cli.main(["info", "--registry", str(root)]) == 0
    # Rebuilding gives the same registry (the texture is the source of truth).
    other = tmp_path / "again"
    assert cli.main([*argv[:2], str(other), *argv[3:]]) == 0
    assert Registry.load(other).content_hash() == registry.content_hash()
    # Registering a held-out product by hand is caught by verify.
    held_row = next(
        r
        for r in sim.load_catalogue_rows(CATALOGUE)
        if r["sku"] == splits["products"]["held_out"][0]
    )
    sim.register_simulated(
        registry,
        [held_row],
        sim.load_geometry_catalogue(GEOMETRY),
        synthetic_textures,
    )
    assert cli.main(["verify", "--registry", str(root), "--splits", str(SPLITS)]) == 1
    capsys.readouterr()


def test_registration_cli_from_images_and_query(tmp_path, capsys):
    folder = tmp_path / "photos"
    folder.mkdir()
    for k, image in enumerate(synthetic_views(7)):
        (folder / f"az{k * 45:03d}.png").write_bytes(io.encode_png(image))
    root = tmp_path / "reg"
    base = [
        "add-images",
        "--registry",
        str(root),
        "--images",
        str(folder),
        "--name",
        "Real Cola",
        "--product-class",
        "can",
        "--geometry-key",
        "can.real",
        "--radius",
        "0.033",
        "--height",
        "0.122",
        "--ocr",
        "COLA",
    ]
    assert cli.main([*base, "--gtin", "2000000000015"]) == 0
    assert cli.main([*base, "--gtin", "2000000000015"]) == 2  # duplicate
    assert cli.main([*base, "--gtin", "2000000000016"]) == 2  # bad check digit
    query = tmp_path / "q.png"
    query.write_bytes(io.encode_png(_query(7, 15.0)))
    assert cli.main(["embed", "--registry", str(root)]) == 0
    assert cli.main(["query", "--registry", str(root), str(query)]) == 0
    out = capsys.readouterr().out
    assert "2000000000015" in out and "1 computed" in out
    assert Registry.load(root).get("2000000000015").ocr_strings == ("COLA",)


def test_image_directory_must_hold_eight_views(tmp_path):
    folder = tmp_path / "photos"
    folder.mkdir()
    for k in range(7):
        (folder / f"{k}.png").write_bytes(io.encode_png(synthetic_views(0)[0]))
    with pytest.raises(RegistryError, match="exactly 8"):
        cli.ordered_view_files(folder)


def test_every_shape_is_rendered_from_the_upright_strip_as_stored(tmp_path):
    """Strips are upright on disk, so cylinder and contour views both follow the file as stored.

    A strip that is bright on its top half only: the view must be bright at the *top* for a
    cylinder product and for the contour mesh alike (the cylinder half-turn belongs to the
    Gazebo install step, not to the registry).
    """
    strip = np.zeros((96, 192, 3), np.uint8)
    strip[:48] = 255
    textures = tmp_path / "textures"
    textures.mkdir()
    rows = [r for r in sim.load_catalogue_rows(CATALOGUE) if r["sku"] in ("GEN-001", "GEN-013")]
    for row in rows:
        (textures / row["texture"]).write_bytes(io.encode_png(strip))
    reg = Registry.create(tmp_path / "reg")
    sim.register_simulated(
        reg, rows, sim.load_geometry_catalogue(GEOMETRY), textures, mesh_dir=MESH_DIR
    )

    def halves(gtin):
        view = reg.view_images(gtin)[0].astype(float)
        occupied = np.where(view.std(axis=(1, 2)) > 1)[0]
        middle = (occupied.min() + occupied.max()) // 2
        return view[occupied.min() : middle].mean(), view[middle : occupied.max()].mean()

    cylinder = next(r["gtin"] for r in rows if r["sku"] == "GEN-001")
    top, bottom = halves(cylinder)
    assert top > bottom + 50
    contour = next(r["gtin"] for r in rows if r["sku"] == "GEN-013")
    top, bottom = halves(contour)
    assert top > bottom + 50
    assert "rendered_as_gazebo_shows_it" not in reg.get(cylinder).source


# --- boxes (CMB-119) ---------------------------------------------------------------------------


def _box_row(sku):
    return next(r for r in sim.load_catalogue_rows(CATALOGUE) if r["sku"] == sku)


def _panel_atlas(row):
    """An atlas whose every face is a distinct, asymmetric pattern (brighter to the right/up)."""
    width, height = row["atlas_px"]
    atlas = np.zeros((height, width, 3), np.uint8)
    for index, face in enumerate(row["faces"].values()):
        x, y, w, h = face["rect_px"]
        gradient_x = np.linspace(0, 255, w, dtype=np.float32)[None, :]
        gradient_y = np.linspace(255, 0, h, dtype=np.float32)[:, None]  # image top is bright
        patch = np.zeros((h, w, 3), np.float32)
        patch[..., 0] = gradient_x
        patch[..., 1] = gradient_y
        patch[..., 2] = 40 + 35 * index
        atlas[y : y + h, x : x + w] = patch.astype(np.uint8)
    return atlas


@pytest.mark.parametrize("sku", ["GEN-017", "GEN-019", "GEN-020"])
def test_box_front_view_shows_the_front_panel_upright_and_unmirrored(sku):
    row = _box_row(sku)
    layout = rd.BoxLayout.from_manifest_row(row)
    atlas = _panel_atlas(row).astype(np.float32) / 255.0
    view = rd.render_box_view(atlas, layout, 0.0, 0.0).astype(np.float32)
    body = np.abs(view - 127.5).sum(axis=2) > 6
    ys, xs = np.where(body)
    crop = view[ys.min() : ys.max() + 1, xs.min() : xs.max() + 1]
    # Red rises to the right, green falls downward: red mean right > left, green mean top > bottom.
    half_w, half_h = crop.shape[1] // 2, crop.shape[0] // 2
    assert crop[:, half_w:, 0].mean() > crop[:, :half_w, 0].mean() + 40
    assert crop[:half_h, :, 1].mean() > crop[half_h:, :, 1].mean() + 40


def test_box_views_turn_through_the_four_sides_and_the_top_is_visible_from_above():
    row = _box_row("GEN-017")
    layout = rd.BoxLayout.from_manifest_row(row)
    atlas = _panel_atlas(row).astype(np.float32) / 255.0
    blue = {}
    for yaw in (0, 90, 180, 270):
        view = rd.render_box_view(atlas, layout, float(yaw), 0.0)
        body = np.abs(view.astype(int) - 127).sum(axis=2) > 6
        blue[yaw] = round(float(view[..., 2][body].mean()), 1)
    # Each face has its own blue level (40 + 35 * face index): front 0, right 1, back 2, left 3.
    # Turning the box counter-clockwise shows the left side at 90 and the right at 270.
    expected = {0: 40, 90: 40 + 35 * 3, 180: 40 + 35 * 2, 270: 40 + 35 * 1}
    for yaw, level in expected.items():
        assert abs(blue[yaw] - level) < 6, (yaw, blue)
    top = rd.render_box_view(atlas, layout, 0.0, 90.0)
    body = np.abs(top.astype(int) - 127).sum(axis=2) > 6
    assert abs(float(top[..., 2][body].mean()) - (40 + 35 * 4)) < 6


def test_box_registration_is_deterministic_and_the_box_has_a_product_id(tmp_path):
    rows = [_box_row("GEN-019"), _box_row("GEN-020")]
    textures = tmp_path / "textures"
    textures.mkdir()
    for row in rows:
        (textures / row["texture"]).write_bytes(io.encode_png(_panel_atlas(row)))
    hashes = []
    for name in ("a", "b"):
        reg = Registry.create(tmp_path / name)
        sim.register_simulated(reg, rows, sim.load_geometry_catalogue(GEOMETRY), textures)
        hashes.append(reg.content_hash())
        assert len(reg) == 2
        with_barcode = reg.get(rows[0]["gtin"])
        assert with_barcode.size_xyz_m == (0.095, 0.06, 0.165)
        assert with_barcode.product_class == "box"
        barcode_less = next(r for r in reg.records() if r.gtin is None)
        assert barcode_less.aliases == ("GEN-020",) and barcode_less.size_xyz_m is not None
        assert barcode_less.product_id.startswith("prod-")
    assert hashes[0] == hashes[1]

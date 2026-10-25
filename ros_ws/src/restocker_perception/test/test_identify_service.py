# Copyright 2026 Matteo Bernacchia <dev@kikijiki.com>
#
# SPDX-License-Identifier: MIT

"""identify_product service logic: reply contract, sidecar-absent answer, image conversion."""
# No ROS here: the node only converts messages (see test_identify_product_node.py).

import json
from pathlib import Path
import sys

import numpy as np
import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from restocker_perception.identity import (  # noqa: E402,I100,I101 - ament and Ruff disagree
    IdentityConfig,
    IdentityEngine,
    write_calibration,
)
from restocker_perception.identity.benchmark import save_whitener  # noqa: E402
from restocker_perception.identity.service import (  # noqa: E402
    STATUS_CODES,
    IdentifyService,
    image_to_rgb,
)
from restocker_perception.registry import (  # noqa: E402
    FakeEmbeddingBackend,
    Registry,
    RegistryIndex,
    Whitener,
)
from restocker_perception.registry import render as rd  # noqa: E402

GTIN = "2000000000015"
OTHER = "2000000000022"


def texture(seed: int) -> np.ndarray:
    rng = np.random.default_rng(seed)
    image = np.tile(rng.uniform(0.1, 0.9, 3), (96, 192, 1)).astype(np.float32)
    for _ in range(8):
        x, y = int(rng.integers(0, 160)), int(rng.integers(0, 80))
        image[y : y + 12, x : x + 24] = rng.uniform(0, 1, 3)
    return image


@pytest.fixture(scope="module")
def deployment(tmp_path_factory):
    root = tmp_path_factory.mktemp("service")
    registry = Registry.create(root / "reg")
    for seed, gtin in enumerate((GTIN, OTHER)):
        registry.register(
            gtin=gtin,
            display_name=f"P{seed}",
            product_class="can",
            geometry_key=f"can.t{seed}",
            radius_m=0.033,
            height_m=0.122,
            images=rd.render_views(texture(seed), 0.033, 0.122),
            ocr_strings=["Brand", f"Flavour{seed}"],
        )
    backend = FakeEmbeddingBackend()
    pool = np.concatenate([backend.embed(registry.view_images(g)) for g in registry.product_ids])
    whitener = Whitener.fit(pool, 8)
    save_whitener(root / "w.npz", whitener)
    fit = {
        "backend": f"{backend.name} {backend.version}",
        "registry_sha256": registry.content_hash(),
        "whitening": {"fingerprint": whitener.fingerprint},
    }
    config = IdentityConfig(alone_min_score=0.0, alone_min_margin=0.0, alone_min_vote=1.0)
    digest = write_calibration(root / "cal.json", config, fit)
    return root, registry, backend, digest


def service_for(deployment) -> IdentifyService:
    root, registry, backend, _digest = deployment
    return IdentifyService.load(
        str(root / "reg"), str(root / "cal.json"), str(root / "w.npz"), backend
    )


def test_loaded_service_names_a_registered_view(deployment):
    root, registry, backend, digest = deployment
    service = service_for(deployment)
    assert service.available
    reply = service.identify([registry.view_images(GTIN)[3]])
    assert reply["status"] == STATUS_CODES["identified"]
    assert reply["has_sku"] and reply["sku"] == GTIN and reply["sidecar_available"]
    assert reply["calibration_sha256"] == digest
    assert reply["registry_sha256"] == registry.content_hash()
    assert reply["evidence_kinds"] == ["embedding"]
    assert len(reply["evidence_gtins"]) == len(reply["evidence_scores"]) == 1
    json.dumps(reply)  # plain types only: it goes straight into a ROS response


def test_missing_files_mean_unknown_not_a_crash(tmp_path):
    service = IdentifyService.load(
        str(tmp_path / "nope"),
        str(tmp_path / "c.json"),
        str(tmp_path / "w.npz"),
        FakeEmbeddingBackend(),
    )
    assert not service.available
    reply = service.identify([np.zeros((32, 32, 3), np.uint8)])
    assert reply["status"] == STATUS_CODES["unknown"]
    assert not reply["has_sku"] and reply["sku"] == "" and not reply["sidecar_available"]
    assert "unavailable" in reply["reason"]


def test_unconfigured_service_answers_unknown():
    reply = IdentifyService(None).identify([])
    assert reply["status"] == STATUS_CODES["unknown"] and not reply["sidecar_available"]


def test_a_changed_registry_or_edited_calibration_disables_the_sidecar(deployment, tmp_path):
    root, registry, backend, _digest = deployment
    edited = tmp_path / "cal.json"
    document = json.loads((root / "cal.json").read_text())
    document["config"]["max_views"] = 1
    edited.write_text(json.dumps(document))
    service = IdentifyService.load(str(root / "reg"), str(edited), str(root / "w.npz"), backend)
    assert not service.available and "edited" in service.unavailable_reason
    other = Registry.create(tmp_path / "other")
    other.register(
        gtin=GTIN,
        display_name="X",
        product_class="can",
        geometry_key="can.x",
        radius_m=0.033,
        height_m=0.122,
        images=rd.render_views(texture(9), 0.033, 0.122),
    )
    service = IdentifyService.load(
        str(tmp_path / "other"), str(root / "cal.json"), str(root / "w.npz"), backend
    )
    assert not service.available and "registry differs" in service.unavailable_reason


def test_a_failing_engine_degrades_to_unknown(deployment):
    class Broken(FakeEmbeddingBackend):
        def embed(self, images):
            raise RuntimeError("model fell over")

    root, registry, _backend, _digest = deployment
    index = RegistryIndex.from_registry(registry, FakeEmbeddingBackend())
    engine = IdentityEngine(index, Broken(), IdentityConfig())
    engine.backend = Broken()
    reply = IdentifyService(engine).identify([np.zeros((32, 32, 3), np.uint8)])
    assert reply["status"] == STATUS_CODES["unknown"] and not reply["sidecar_available"]
    assert "model fell over" in reply["reason"]


def test_reject_abstains_with_the_reason(deployment):
    reply = service_for(deployment).reject("bad request: nope")
    assert reply["status"] == STATUS_CODES["abstain"] and reply["sidecar_available"]


def test_image_conversion_handles_encodings_and_row_padding():
    pixels = np.arange(2 * 3 * 3, dtype=np.uint8).reshape(2, 3, 3)
    padded = np.zeros((2, 12), np.uint8)
    padded[:, :9] = pixels.reshape(2, 9)
    rgb = image_to_rgb(2, 3, "rgb8", 12, padded.tobytes())
    assert (rgb == pixels).all()
    bgr = image_to_rgb(2, 3, "bgr8", 12, padded.tobytes())
    assert (bgr == pixels[:, :, ::-1]).all()
    with pytest.raises(ValueError, match="unsupported"):
        image_to_rgb(2, 3, "mono8", 3, bytes(6))

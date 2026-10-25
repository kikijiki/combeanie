# Copyright 2026 Matteo Bernacchia <dev@kikijiki.com>
#
# SPDX-License-Identifier: MIT

"""/identify_object: the service logic without ROS."""
# The ladder itself is covered in test_identity_ladder.py; this pins the per-object state across
# calls, the reply contract (guidance fields, sidecar-absent answer) and the bounded state.

import math
from pathlib import Path
import sys

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from restocker_perception.identity.ladder_service import (  # noqa: E402,I100,I101
    STATUS_CODES,
    IdentifyObjectService,
)
from ladder_support import (  # noqa: E402,I100
    MELON,
    MINT,
    crop,
    engine,
    make_registry,
)


@pytest.fixture
def registry(tmp_path):
    return make_registry(tmp_path)


def service(registry, lines, **kw):
    return IdentifyObjectService(engine(registry, lines, shortlist_rank=4, **kw))


def test_an_absent_sidecar_answers_unknown_and_says_why():
    reply = IdentifyObjectService(None).identify("o1", [crop(1)])
    assert reply["status"] == STATUS_CODES["unknown"] and not reply["sidecar_available"]
    assert "not configured" in reply["reason"] and reply["sku"] == ""


def test_a_look_that_does_not_name_the_product_returns_guidance_and_the_next_look_adds_to_it(
    registry,
):
    svc = service(registry, {1: ["Fennlo", "Sparkling"], 2: ["Mint"]})
    first = svc.identify("o1", [crop(1, 1)])
    assert first["status"] == STATUS_CODES["not_identified_yet"] and first["look_index"] == 0
    assert first["guidance_kind"] == "turn" and first["has_turn"] and first["has_facing"]
    assert -180 < first["suggested_turn_deg"] <= 180 and first["guidance_candidate_sku"]
    assert first["text_best_sku"] in (MINT, MELON) and first["text_matched"]
    second = svc.identify("o1", [crop(11, 2)])
    assert second["status"] == STATUS_CODES["identified"] and second["sku"] == MINT
    assert second["look_index"] == 1 and second["rung"] == "text" and second["has_gtin"]
    assert math.isnan(second["suggested_turn_deg"]) and second["guidance_kind"] == ""


def test_the_budget_is_the_callers_and_ends_in_needs_human(registry):
    svc = service(registry, {1: ["Fennlo", "Sparkling"]})
    one = svc.identify("a", [crop(1, 1)], max_relooks=0)
    assert one["status"] == STATUS_CODES["needs_human"] and one["sku"] == ""
    first = svc.identify("b", [crop(1, 1)], max_relooks=1)
    second = svc.identify("b", [crop(2, 1)], max_relooks=1)
    assert [first["status"], second["status"]] == [
        STATUS_CODES["not_identified_yet"],
        STATUS_CODES["needs_human"],
    ]


def test_keys_are_separate_and_reset_forgets(registry):
    svc = service(registry, {1: ["Fennlo", "Sparkling"], 2: ["Mint"]})
    svc.identify("a", [crop(1, 1)])
    assert svc.identify("b", [crop(2, 2)])["look_index"] == 0
    assert svc.identify("a", [crop(3, 1)])["look_index"] == 1
    assert svc.identify("a", [crop(3, 1)], reset=True)["look_index"] == 0


def test_a_finished_object_is_forgotten_and_state_is_bounded(registry):
    svc = service(registry, {1: ["Fennlo", "Mint", "Sparkling"]})
    assert svc.identify("a", [crop(1, 1)])["status"] == STATUS_CODES["identified"]
    assert "a" not in svc.states
    from restocker_perception.identity import ladder_service as ls  # noqa: PLC0415

    svc = service(registry, {})
    for i in range(ls.MAX_OBJECTS + 5):
        svc.identify(f"o{i}", [crop(i)])
    assert len(svc.states) == ls.MAX_OBJECTS and "o0" not in svc.states


def test_unknown_half_widths_and_azimuths_are_accepted(registry):
    svc = service(registry, {})
    reply = svc.identify("a", [crop(1), crop(2)], [0.0, 12.0], [math.nan, 30.0])
    assert reply["status"] == STATUS_CODES["not_identified_yet"]


def test_a_failing_engine_answers_unknown_and_stays_up(registry):
    class Broken:
        def read(self, image):
            raise RuntimeError("no model")

    eng = engine(registry, {}, shortlist_rank=4)
    eng.reader.backend = Broken()
    reply = IdentifyObjectService(eng).identify("a", [crop(1)])
    assert reply["status"] == STATUS_CODES["unknown"] and "failed" in reply["reason"]


def test_the_service_loads_from_files_and_reports_the_config_it_ran_with(registry, tmp_path):
    import json  # noqa: PLC0415

    import numpy as np  # noqa: PLC0415
    from ladder_support import ScriptedOcr  # noqa: PLC0415
    from restocker_perception.identity.benchmark import save_whitener  # noqa: PLC0415
    from restocker_perception.identity.ladder import LadderConfig  # noqa: PLC0415
    from restocker_perception.registry import FakeEmbeddingBackend, Whitener  # noqa: PLC0415

    backend = FakeEmbeddingBackend()
    pool = np.concatenate([backend.embed(registry.view_images(p)) for p in registry.product_ids])
    save_whitener(tmp_path / "w.npz", Whitener.fit(pool, 8))
    config = LadderConfig(max_relooks=1, shortlist_rank=4)
    assert LadderConfig.from_dict(config.to_dict()) == config
    (tmp_path / "config.json").write_text(json.dumps(config.to_dict()))
    svc = IdentifyObjectService.load(
        str(registry.root),
        str(tmp_path / "w.npz"),
        str(tmp_path / "config.json"),
        backend,
        ScriptedOcr({1: ["Fennlo", "Mint", "Sparkling"]}),
    )
    assert svc.available and svc.engine.config.max_relooks == 1
    reply = svc.identify("a", [crop(1, 1)])
    assert reply["sku"] == MINT and len(reply["config_sha256"]) == 64
    missing = IdentifyObjectService.load(
        str(registry.root), str(tmp_path / "nope.npz"), "", backend, ScriptedOcr({})
    )
    assert not missing.available and "unavailable" in missing.unavailable_reason

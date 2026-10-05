# Copyright 2026 Matteo Bernacchia <dev@kikijiki.com>
#
# SPDX-License-Identifier: MIT

"""The paired driver's write-side policy: fresh directories, retained attempts, aggregation."""

import json
from pathlib import Path
import sys

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from restocker_benchmarks import scenarios, transfer_reliability  # noqa: E402,I100,I101

# The driver chain imports runner, which needs rclpy and the built interfaces package. Under
# colcon both exist; in a bare interpreter the import fails and the driver tests skip at setup.
# The failure is captured rather than raised here on purpose: a module-level pytest.skip breaks
# collection under this environment's launch_testing pytest plugin, so skipping happens per test.
try:
    from restocker_benchmarks import paired_campaign  # noqa: E402,I100,I101

    _PAIRED_IMPORT_ERROR: Exception | None = None
except Exception as error:  # noqa: BLE001 - any driver import failure is an environment skip
    paired_campaign = None
    _PAIRED_IMPORT_ERROR = error


@pytest.fixture()
def campaign():
    """Return the paired driver module, or skip when this interpreter cannot import it."""
    if paired_campaign is None:
        pytest.skip(f"paired driver unavailable: {_PAIRED_IMPORT_ERROR}")
    return paired_campaign


def _scenario(name):
    return scenarios.scenario(name)


def test_refuse_existing_campaign_rejects_a_directory_that_holds_records(campaign, tmp_path):
    run_dir = tmp_path / "baseline_transfers" / "old-run"
    run_dir.mkdir(parents=True)
    (run_dir / "run.json").write_text("{}", encoding="utf-8")
    with pytest.raises(SystemExit, match="no attempt can replace another"):
        campaign._refuse_existing_campaign(tmp_path)


def test_a_fresh_or_recordless_directory_is_accepted(campaign, tmp_path):
    campaign._refuse_existing_campaign(tmp_path)
    empty = tmp_path / "empty"
    empty.mkdir()
    campaign._refuse_existing_campaign(empty)


def test_declared_pairs_round_trip_through_campaign_json(campaign, tmp_path):
    assert campaign._read_declared_pairs(tmp_path) is None
    (tmp_path / "campaign.json").write_text(
        json.dumps({"campaign": transfer_reliability.CAMPAIGN_NAME, "declared_pairs": 3}),
        encoding="utf-8",
    )
    assert campaign._read_declared_pairs(tmp_path) == 3
    (tmp_path / "campaign.json").write_text("{not json", encoding="utf-8")
    assert campaign._read_declared_pairs(tmp_path) is None
    (tmp_path / "campaign.json").write_text(json.dumps({"declared_pairs": 0}), encoding="utf-8")
    assert campaign._read_declared_pairs(tmp_path) is None


def test_harness_record_is_classifiable_and_carries_the_arm_gate(campaign, tmp_path):
    record = campaign._harness_record(
        _scenario("baseline_transfers"),
        "fixed",
        tmp_path / "run",
        detail="RuntimeError: boom",
        traceback_text="Traceback: boom",
    )
    assert record["outcome"] == "harness_exception"
    assert record["schema_version"] == 3  # RECORD_SCHEMA_VERSION: + provenance.obstacle_perception
    assert transfer_reliability.classify_run(record) == "harness_exception"
    # The arm is whatever this tree spawns; the gate reads it rather than assuming ur10e.
    assert "label" in record["provenance"]["arm"]
    assert "load_average_at_start" in record["provenance"]["hardware"]
    # Card 023: every run record names the depth-obstacle pipeline it exercised (value or None
    # when the launch file could not be read).
    assert "obstacle_perception" in record["provenance"]


def test_pair_order_alternates_so_neither_fixture_goes_first_always():
    """The driver's interleaving rule: AB on even pair indices, BA on odd ones."""
    configurations = transfer_reliability.CONFIGURATIONS
    orders = [
        configurations if pair % 2 == 0 else tuple(reversed(configurations)) for pair in range(4)
    ]
    assert orders[0] == ("baseline_transfers", "dense_restock_transfers")
    assert orders[1] == ("dense_restock_transfers", "baseline_transfers")
    assert orders[2] == orders[0]
    assert orders[3] == orders[1]
    # Each configuration appears exactly once per pair.
    for order in orders:
        assert sorted(order) == sorted(configurations)


def test_summarise_only_refuses_to_rescale_a_stored_campaign(campaign, tmp_path):
    (tmp_path / "campaign.json").write_text(json.dumps({"declared_pairs": 3}), encoding="utf-8")
    with pytest.raises(SystemExit, match="declared count"):
        campaign.main(["--summarise-only", "--pairs", "5", "--results-dir", str(tmp_path)])


def test_summarise_only_aggregates_a_stored_directory(campaign, tmp_path):
    run_dir = tmp_path / "baseline_transfers" / "run-1"
    run_dir.mkdir(parents=True)
    record = {
        "schema_version": 2,
        "run_id": "run-1",
        "scenario": "baseline_transfers",
        "configuration": "baseline_transfers",
        "pair_index": 0,
        "attempt_index": 1,
        "seed": "fixed",
        "outcome": "completed",
        "started_utc": "2026-09-23T00:00:00+00:00",
        "provenance": {
            "arm": {"label": "ur10e"},
            "hardware": {"load_average_at_start": [1.0, 1.0, 1.0], "cpu_count": 8},
        },
        "goals": [],
    }
    (run_dir / "run.json").write_text(json.dumps(record), encoding="utf-8")
    assert campaign.main(["--summarise-only", "--pairs", "1", "--results-dir", str(tmp_path)]) == 0
    assert (tmp_path / "summary.json").is_file()
    assert (tmp_path / "summary.txt").is_file()
    assert (tmp_path / "report.md").is_file()
    summary = json.loads((tmp_path / "summary.json").read_text(encoding="utf-8"))
    assert summary["declared_pairs"] == 1
    # The single completed run recorded no goals, so it is retained but not clean.
    assert summary["by_configuration"]["baseline_transfers"]["run_classes"] == {
        "completed_with_failure": 1
    }
    assert summary["population_complete"] is False  # dense slot never ran


def _addendum_declaration(pairs=1):
    return {
        "campaign": transfer_reliability.ADDENDUM_CAMPAIGN_NAME,
        "addendum": transfer_reliability.ADDENDUM_ID,
        "declared_pairs": pairs,
        "configurations": list(transfer_reliability.ADDENDUM_CONFIGURATIONS),
        "arm_label_required": transfer_reliability.ARM_LABEL,
        "seed_policy": {"dense_restock_transfers": "fixed"},
        "started_utc": "2026-09-24T00:00:00+00:00",
    }


def _dense_record(run_id, attempt_index, *, started=None):
    return {
        "schema_version": 2,
        "run_id": run_id,
        "scenario": "dense_restock_transfers",
        "configuration": "dense_restock_transfers",
        "pair_index": attempt_index - 1,
        "attempt_index": attempt_index,
        "seed": "fixed",
        "outcome": "completed",
        "started_utc": started or f"2026-09-24T01:{attempt_index:02d}:00+00:00",
        "provenance": {
            "arm": {"label": "ur10e"},
            "hardware": {"load_average_at_start": [2.0, 2.0, 2.0], "cpu_count": 32},
        },
        "goals": [],
    }


def test_declaration_payload_stamps_the_addendum_id_only_on_the_addendum(campaign):
    addendum = campaign._declaration_payload(
        campaign=transfer_reliability.ADDENDUM_CAMPAIGN_NAME,
        declared_pairs=transfer_reliability.ADDENDUM_ATTEMPTS,
        configurations=transfer_reliability.ADDENDUM_CONFIGURATIONS,
        started_utc="2026-09-24T00:00:00+00:00",
    )
    assert addendum["addendum"] == transfer_reliability.ADDENDUM_ID
    assert addendum["configurations"] == ["dense_restock_transfers"]
    assert addendum["seed_policy"] == {"dense_restock_transfers": "fixed"}
    paired = campaign._declaration_payload(
        campaign=transfer_reliability.CAMPAIGN_NAME,
        declared_pairs=transfer_reliability.PAIRS_PER_CONFIGURATION,
        configurations=transfer_reliability.CONFIGURATIONS,
        started_utc="2026-09-23T00:00:00+00:00",
    )
    assert "addendum" not in paired
    assert paired["configurations"] == list(transfer_reliability.CONFIGURATIONS)


def test_declaration_scope_prefers_the_stored_declaration(campaign):
    scope = campaign._declaration_scope(_addendum_declaration(pairs=3), None)
    assert scope == {
        "declared_pairs": 3,
        "configurations": ("dense_restock_transfers",),
        "campaign": transfer_reliability.ADDENDUM_CAMPAIGN_NAME,
    }
    # A stored declaration refuses a conflicting --pairs rather than rescaling the population.
    with pytest.raises(SystemExit, match="declared count"):
        campaign._declaration_scope(_addendum_declaration(pairs=3), 5)
    # With no declaration the invocation's own defaults describe its population.
    scope = campaign._declaration_scope(
        None,
        None,
        default_configurations=transfer_reliability.ADDENDUM_CONFIGURATIONS,
        default_campaign=transfer_reliability.ADDENDUM_CAMPAIGN_NAME,
        default_pairs=transfer_reliability.ADDENDUM_ATTEMPTS,
    )
    assert scope == {
        "declared_pairs": transfer_reliability.ADDENDUM_ATTEMPTS,
        "configurations": transfer_reliability.ADDENDUM_CONFIGURATIONS,
        "campaign": transfer_reliability.ADDENDUM_CAMPAIGN_NAME,
    }
    # An unusable stored scope is a clean exit, not a traceback.
    broken = {"configurations": ["lane_routing"]}
    with pytest.raises(SystemExit, match="unusable population scope"):
        campaign._declaration_scope(broken, None)


def test_summarise_only_aggregates_the_stored_dense_addendum(campaign, tmp_path):
    """The addendum directory re-aggregates against its own declaration, not the paired one."""
    run_dir = tmp_path / "dense_restock_transfers" / "dense-01"
    run_dir.mkdir(parents=True)
    (run_dir / "run.json").write_text(json.dumps(_dense_record("dense-01", 1)), encoding="utf-8")
    (tmp_path / "campaign.json").write_text(
        json.dumps(_addendum_declaration(pairs=1)), encoding="utf-8"
    )
    assert campaign.main(["--summarise-only", "--results-dir", str(tmp_path)]) == 0
    summary = json.loads((tmp_path / "summary.json").read_text(encoding="utf-8"))
    assert summary["campaign"] == transfer_reliability.ADDENDUM_CAMPAIGN_NAME
    assert summary["configurations"] == ["dense_restock_transfers"]
    # One declared dense attempt, one clean record: complete on the addendum's own terms even
    # though no baseline row exists (the addendum never declares baseline slots).
    assert summary["population_complete"] is True
    assert summary["coverage"] == {
        "dense_restock_transfers": {"slots": 1, "filled": 1, "missing": []}
    }
    assert "baseline_transfers" not in summary["by_configuration"]
    # The run recorded no goals, so it is completed_with_failure, not clean.
    assert summary["by_configuration"]["dense_restock_transfers"]["run_classes"] == {
        "completed_with_failure": 1
    }

# Copyright 2026 Matteo Bernacchia <dev@kikijiki.com>
#
# SPDX-License-Identifier: MIT

"""The runner's readiness gate: dispatch only when observed == stocked, else fail closed."""
# The 2026-09-23 dense campaign dispatched 7-9 s into a still-spawning 62-product fixture on all
# 24 runs, so every goal was a product_unobserved harness terminal. These tests pin the fix: a
# partial spawn must never reach dispatch, and a fixture that never completes must end as a
# classified startup failure with the partial observation archived — not a hang, not a rate.

from pathlib import Path
import sys
from types import MethodType, SimpleNamespace

import pytest
import yaml

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from restocker_benchmarks import scenarios, transfer_reliability  # noqa: E402,I100,I101

# The runner imports rclpy and the built interfaces package. Under colcon both exist; in a bare
# interpreter the import fails and these tests skip at setup (a module-level pytest.skip breaks
# collection under this environment's launch_testing plugin — see the field notes).
try:
    from restocker_benchmarks import runner  # noqa: E402,I100,I101

    _RUNNER_IMPORT_ERROR: Exception | None = None
except Exception as error:  # noqa: BLE001 - any runner import failure is an environment skip
    runner = None
    _RUNNER_IMPORT_ERROR = error


@pytest.fixture()
def bench():
    """Return the runner module, or skip when this interpreter cannot import it."""
    if runner is None:
        pytest.skip(f"runner unavailable: {_RUNNER_IMPORT_ERROR}")
    return runner


# A four-product slice of the dense fixture's shape: front-lane products spawn first, the
# addressed back-stock sources last. The campaign's race left every sim:stock_* unobserved.
STOCKED_IDS = [
    "sim:lane_01_can_01",
    "sim:lane_02_small_bottle_01",
    "sim:stock_can_01",
    "sim:stock_small_bottle_01",
    "sim:stock_large_bottle_01",
]
FRONT_ONLY_IDS = [name for name in STOCKED_IDS if not name.startswith("sim:stock_")]


def _tracked(source_object_id, object_id):
    position = SimpleNamespace(x=0.0, y=0.0, z=0.0)
    return SimpleNamespace(
        source_object_id=source_object_id,
        id=object_id,
        pose=SimpleNamespace(pose=SimpleNamespace(position=position)),
    )


def _snapshot(source_ids, *, telemetry=True):
    return SimpleNamespace(
        objects=[_tracked(name, index + 1) for index, name in enumerate(source_ids)],
        lanes=[],
        robot=SimpleNamespace(
            telemetry_source_id="ground_truth" if telemetry else "",
            telemetry_revision=1 if telemetry else 0,
            has_held_object=False,
        ),
    )


def _stock_document():
    return {
        "products": [
            {
                "source_object_id": source_id,
                "product_class": "can",
                "spawn_pose": [0.0, 0.0, 0.0],
            }
            for source_id in STOCKED_IDS
        ]
    }


def _write_stock(tmp_path):
    path = tmp_path / "stock.yaml"
    path.write_text(yaml.safe_dump(_stock_document()), encoding="utf-8")
    return path


class _FakeClient:
    """
    A client with scripted snapshots and the production readiness predicates bound on.

    ``snapshot()`` walks the script and repeats the last entry, so a one-element script is a
    fixture that never finishes spawning. ``spin_until`` polls a bounded number of times instead
    of a wall clock: the fake has no clock to time out on, and the bound exercises the same
    control flow (poll, give up, classify) the real gate uses when SCENARIO_READY_TIMEOUT_S
    expires.
    """

    POLL_BUDGET = 8

    def __init__(self, bench_module, snapshots):
        self.latest_status = SimpleNamespace(admission_ready=True)
        self._snapshots = list(snapshots)
        self._index = 0
        self.action_client = SimpleNamespace(wait_for_server=lambda timeout_sec: True)
        # The real gate methods, so the test exercises the production predicates rather than a
        # copy of them.
        self.admitted_snapshot = MethodType(bench_module.BenchmarkClient.admitted_snapshot, self)
        self.fixture_snapshot = MethodType(bench_module.BenchmarkClient.fixture_snapshot, self)

    def spin_until(self, predicate, timeout_s, alive):
        for _ in range(self.POLL_BUDGET):
            if not alive():
                return None
            found = predicate()
            if found:
                return found
        return None

    def snapshot(self, alive, timeout_s=30.0):
        if not alive() or not self._snapshots:
            return None
        current = self._snapshots[self._index]
        if self._index < len(self._snapshots) - 1:
            self._index += 1
        return current


def _run_directory(tmp_path):
    run_directory = tmp_path / "run"
    run_directory.mkdir()  # execute_run creates it before _drive; the tests stand in for it
    return run_directory


def test_stocked_ids_of_reads_the_scenario_document(bench):
    assert bench.stocked_ids_of(_stock_document()) == set(STOCKED_IDS)
    assert bench.stocked_ids_of({}) == set()


def test_fixture_fully_observed_rejects_a_partial_spawn(bench):
    """The regression case: 2-of-N front stock with the addressed back stock still missing."""
    stocked = set(STOCKED_IDS)
    assert bench.fixture_fully_observed(_snapshot(FRONT_ONLY_IDS), stocked) is False
    assert bench.fixture_fully_observed(_snapshot(STOCKED_IDS), stocked) is True
    assert bench.fixture_fully_observed(None, stocked) is False
    # Extra products beyond the scenario's own stocking do not block readiness.
    extra = [*STOCKED_IDS, "sim:something_else"]
    assert bench.fixture_fully_observed(_snapshot(extra), stocked) is True
    assert bench.fixture_fully_observed(_snapshot([]), set()) is True


def test_scenario_completeness_wait_is_bounded(bench):
    assert bench.SCENARIO_READY_TIMEOUT_S > 0.0
    assert bench.SCENARIO_READY_TIMEOUT_S <= bench.STARTUP_TIMEOUT_S


def test_drive_waits_out_a_partial_spawn_then_dispatches(bench, tmp_path, monkeypatch):
    """Partial first, complete second: the gate must keep polling, then dispatch every goal."""
    stock = _write_stock(tmp_path)
    scenario = scenarios.scenario("dense_restock_transfers")
    client = _FakeClient(bench, [_snapshot(FRONT_ONLY_IDS), _snapshot(STOCKED_IDS)])
    record: dict = {"provenance": {"stock_config": str(stock)}, "goals": []}
    dispatched = []

    def fake_run_goal(_client, goal_spec, object_id, alive):
        dispatched.append((goal_spec.label, object_id))
        return {
            "label": goal_spec.label,
            "outcome": {
                "status": bench.taxonomy.STATUS_SUCCEEDED,
                "status_name": "SUCCEEDED",
                "detail": "delivered",
                "detail_full": "delivered",
                "family": "succeeded",
                "signature": "SUCCEEDED: delivered",
            },
        }

    monkeypatch.setattr(bench, "_run_goal", fake_run_goal)
    run_directory = _run_directory(tmp_path)
    bench._drive(client, scenario, run_directory, "fixed", record, alive=lambda: True)

    assert record["outcome"] == "completed"
    assert record["ground_truth_observation"]["stocked"] == len(STOCKED_IDS)
    assert record["ground_truth_observation"]["observed"] == len(STOCKED_IDS)
    # All three addressed goals dispatched, each resolved to a numeric object id.
    assert len(dispatched) == 3
    assert {label for label, _ in dispatched} == {goal.label for goal in scenario.goals}
    assert all(object_id is not None for _, object_id in dispatched)


def test_drive_never_dispatches_into_a_partial_spawn(bench, tmp_path, monkeypatch):
    """The campaign's race, fail-closed: fixture never completes -> classified startup failure."""
    stock = _write_stock(tmp_path)
    scenario = scenarios.scenario("dense_restock_transfers")
    client = _FakeClient(bench, [_snapshot(FRONT_ONLY_IDS)])
    record: dict = {"provenance": {"stock_config": str(stock)}, "goals": []}

    def fail_if_dispatched(*_args, **_kwargs):
        pytest.fail("a goal must not be dispatched while the fixture is partially spawned")

    monkeypatch.setattr(bench, "_run_goal", fail_if_dispatched)
    run_directory = _run_directory(tmp_path)
    bench._drive(client, scenario, run_directory, "fixed", record, alive=lambda: True)

    # Classified, bounded, and informative — not a hang and not an unclassified outcome.
    assert record["outcome"] == "startup_failed"
    assert transfer_reliability.classify_run(record) == "startup_failed"
    assert "never finished spawning" in record["outcome_detail"]
    assert f"observed {len(FRONT_ONLY_IDS)} of {len(STOCKED_IDS)}" in record["outcome_detail"]
    assert record["goals"] == []
    # The partial observation and the fixture document are archived beside the record.
    observation = record["ground_truth_observation"]
    assert observation["stocked"] == len(STOCKED_IDS)
    assert observation["observed"] == len(FRONT_ONLY_IDS)
    assert (tmp_path / "run" / "scenario.yaml").is_file()
    assert len(record["stocked_products"]) == len(STOCKED_IDS)


def test_drive_keeps_the_telemetry_failure_classification(bench, tmp_path):
    """No admitted telemetry at all is still the original startup failure, not the spawn one."""
    stock = _write_stock(tmp_path)
    scenario = scenarios.scenario("dense_restock_transfers")
    client = _FakeClient(bench, [_snapshot(STOCKED_IDS, telemetry=False)])
    record: dict = {"provenance": {"stock_config": str(stock)}, "goals": []}

    run_directory = _run_directory(tmp_path)
    bench._drive(client, scenario, run_directory, "fixed", record, alive=lambda: True)

    assert record["outcome"] == "startup_failed"
    assert record["outcome_detail"] == "the world state never admitted robot telemetry"
    assert record["goals"] == []
    assert record["ground_truth_observation"]["observed"] == 0

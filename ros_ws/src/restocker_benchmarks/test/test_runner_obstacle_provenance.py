# Copyright 2026 Matteo Bernacchia <dev@kikijiki.com>
#
# SPDX-License-Identifier: MIT

"""provenance.obstacle_perception: each run records the pipeline it launched (Card 023)."""
# The benchmark harness launches baseline.launch.py without pinning obstacle_perception, so the
# default flip would otherwise be invisible in stored campaigns. These tests pin the resolver:
# an explicit argument wins, the shipped default is read from the launch file's own text, and an
# unreadable launch file records None rather than failing the run.

from pathlib import Path
import sys

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

# The runner imports rclpy and the built interfaces package. Under colcon both exist; in a bare
# interpreter the import fails and these tests skip at setup (a module-level pytest.skip breaks
# collection under this environment's launch_testing plugin — see the field notes).
try:
    from restocker_benchmarks import runner  # noqa: E402,I100,I101

    _RUNNER_IMPORT_ERROR: Exception | None = None
except Exception as error:  # noqa: BLE001 - any runner import failure is an environment skip
    runner = None
    _RUNNER_IMPORT_ERROR = error

SAMPLE_LAUNCH = """
    DeclareLaunchArgument(
        "obstacle_perception",
        default_value="true",
        description=("Recover unmodeled obstacles from the overhead depth stream"),
    ),
"""


@pytest.fixture()
def bench():
    """Return the runner module, or skip when this interpreter cannot import it."""
    if runner is None:
        pytest.skip(f"runner unavailable: {_RUNNER_IMPORT_ERROR}")
    return runner


def test_default_parses_from_the_launch_declarations(bench):
    assert bench.obstacle_perception_default_from(SAMPLE_LAUNCH) == "true"


def test_a_launch_file_without_the_declaration_is_unknown(bench):
    assert bench.obstacle_perception_default_from('DeclareLaunchArgument("gui")') is None


def test_an_explicit_argument_wins_over_the_shipped_default(bench):
    assert bench.effective_obstacle_perception({}, SAMPLE_LAUNCH) == "true"
    explicit_false = {"obstacle_perception": "false"}
    assert bench.effective_obstacle_perception(explicit_false, SAMPLE_LAUNCH) == "false"
    explicit_bool = {"obstacle_perception": True}
    assert bench.effective_obstacle_perception(explicit_bool, SAMPLE_LAUNCH) == "True"


def test_an_unreadable_launch_file_records_none_not_a_crash(bench, monkeypatch):
    def explode(_share):
        raise RuntimeError("share missing")

    monkeypatch.setattr(bench, "get_package_share_directory", explode)
    assert bench.effective_obstacle_perception({}) is None


def test_the_shipped_baseline_default_is_the_value_campaigns_inherit(bench):
    """Integration: the installed launch file the harness invokes parses to the live default."""
    from ament_index_python.packages import PackageNotFoundError, get_package_share_directory

    try:
        launch_path = (
            Path(get_package_share_directory("restocker_bringup"))
            / "launch"
            / "baseline.launch.py"
        )
        text = launch_path.read_text(encoding="utf-8")
    except (PackageNotFoundError, OSError) as error:
        pytest.skip(f"restocker_bringup share unavailable: {error}")
    assert bench.effective_obstacle_perception({}, text) in {"true", "false"}
    # Card 023 flipped this on; if it flips again the recorded value changes with it, which is
    # the point — this assertion documents the current shipped value for a campaign reader.
    assert bench.effective_obstacle_perception({}, text) == "true"

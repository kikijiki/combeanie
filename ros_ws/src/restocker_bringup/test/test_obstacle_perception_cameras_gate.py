# Copyright 2026 Matteo Bernacchia <dev@kikijiki.com>
#
# SPDX-License-Identifier: MIT

"""Static contract for the obstacle-evidence gate (Card 023 review follow-up)."""
# `cameras:=false` with `obstacle_perception` defaulting true used to compose a projector that
# *requires* obstacle evidence while `depth_obstacle_node` never starts: no producer can exist,
# so the scene stays uncertified forever (a fail-closed deadlock). Both consumers are now gated
# on `cameras AND obstacle_perception`, the conjunction pattern simulation.launch.py already
# uses for `perception`. Everything here is read from the source tree: no GUI, no simulator,
# no lease.

import importlib.util
import os
from pathlib import Path

from launch import LaunchContext
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch_ros.actions import Node
import pytest

BRINGUP = Path(os.environ["RESTOCKER_BRINGUP_SOURCE_DIR"])
BASELINE_LAUNCH = BRINGUP / "launch" / "baseline.launch.py"

# Four combinations of the gate inputs: depth pipeline on iff cameras AND obstacle_perception.
GATE_COMBINATIONS = [
    {"cameras": "true", "obstacle_perception": "true"},
    {"cameras": "true", "obstacle_perception": "false"},
    {"cameras": "false", "obstacle_perception": "true"},
    {"cameras": "false", "obstacle_perception": "false"},
]


@pytest.fixture(scope="module")
def baseline_description():
    spec = importlib.util.spec_from_file_location(
        "restocker_baseline_launch_gate", BASELINE_LAUNCH
    )
    assert spec is not None and spec.loader is not None
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module.generate_launch_description()


def _node(description, executable):
    matches = [
        entity
        for entity in description.entities
        if isinstance(entity, Node) and entity.node_executable == executable
    ]
    assert len(matches) == 1, f"expected exactly one {executable} node, got {len(matches)}"
    return matches[0]


def _context(**configurations):
    context = LaunchContext()
    context.launch_configurations.update(configurations)
    return context


def test_cameras_is_declared_and_forwarded_to_the_simulation(baseline_description) -> None:
    declarations = [
        entity
        for entity in baseline_description.entities
        if isinstance(entity, DeclareLaunchArgument) and entity.name == "cameras"
    ]
    assert len(declarations) == 1, "baseline must declare cameras itself"
    context = LaunchContext()
    declarations[0].execute(context)
    assert context.launch_configurations["cameras"] == "true"

    includes = [
        entity
        for entity in baseline_description.entities
        if isinstance(entity, IncludeLaunchDescription)
    ]
    assert any(
        key == "cameras" for include in includes for key, _value in include.launch_arguments
    ), "baseline must forward cameras to the simulation composition"


def test_depth_pipeline_gates_on_cameras_and_obstacle_perception(baseline_description) -> None:
    """The detector must be off whenever either half of the gate is off."""
    depth = _node(baseline_description, "depth_obstacle_node")
    for configurations in GATE_COMBINATIONS:
        expected = (
            configurations["cameras"] == "true" and configurations["obstacle_perception"] == "true"
        )
        assert depth.condition.evaluate(_context(**configurations)) is expected, (
            f"depth_obstacle_node condition wrong for {configurations}"
        )


def test_projector_requires_evidence_through_the_same_gate(baseline_description) -> None:
    """require_obstacle_evidence must share the conjunction, never raw obstacle_perception."""
    source = BASELINE_LAUNCH.read_text(encoding="utf-8")
    # The projector parameter and the detector condition are pinned to one shared substitution;
    # evaluating the condition above is what proves that substitution's behaviour.
    assert "obstacle_evidence_required = AndSubstitution(cameras, obstacle_perception)" in source
    collapsed = " ".join(source.split())
    assert "require_obstacle_evidence" in collapsed
    assert "obstacle_evidence_required, value_type=bool" in collapsed
    projector = _node(baseline_description, "planning_scene_projector_node")
    assert projector.condition is not None, "the projector keeps its own projection switch"


def test_obstacle_perception_still_defaults_on(baseline_description) -> None:
    declarations = {
        entity.name: entity
        for entity in baseline_description.entities
        if isinstance(entity, DeclareLaunchArgument)
    }
    context = LaunchContext()
    declarations["obstacle_perception"].execute(context)
    assert context.launch_configurations["obstacle_perception"] == "true"

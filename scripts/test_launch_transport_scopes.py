# Copyright 2026 Matteo Bernacchia <dev@kikijiki.com>
#
# SPDX-License-Identifier: MIT

"""Exactly the /clock-hop participants opt into UDP-only Fast DDS; camera bridges do not."""

import ast
from pathlib import Path

REPOSITORY_ROOT = Path(__file__).resolve().parents[1]
GAZEBO_SIMULATION = REPOSITORY_ROOT / "ros_ws/src/restocker_gazebo/launch/simulation.launch.py"
BRINGUP_SIMULATION = REPOSITORY_ROOT / "ros_ws/src/restocker_bringup/launch/simulation.launch.py"
TRANSPORT_ENV = "FASTDDS_BUILTIN_TRANSPORTS"


def _call_name(node: ast.AST) -> str | None:
    """Return the callee's plain name for Name and Attribute calls."""
    if isinstance(node, ast.Call):
        if isinstance(node.func, ast.Name):
            return node.func.id
        if isinstance(node.func, ast.Attribute):
            return node.func.attr
    return None


def _udp_scoped_nodes(path: Path) -> tuple[set[str], int]:
    """Return (node names inside UDP-only groups, count of UDP-only env sets) for one file.

    Nodes may sit inside a group inline or be referenced by name from an earlier assignment
    (the production form for `world_state`, which is built first and then wrapped), so names
    resolve through every call assigned to that identifier.
    """
    tree = ast.parse(path.read_text(encoding="utf-8"))
    parents: dict[ast.AST, ast.AST] = {}
    for parent in ast.walk(tree):
        for child in ast.iter_child_nodes(parent):
            parents[child] = parent
    assignments: dict[str, list[ast.Call]] = {}
    for node in ast.walk(tree):
        if isinstance(node, ast.Assign) and isinstance(node.value, ast.Call):
            for target in node.targets:
                if isinstance(target, ast.Name):
                    assignments.setdefault(target.id, []).append(node.value)
    scoped: set[str] = set()
    env_sets = 0

    def add_node_name(call: ast.Call) -> None:
        for keyword in call.keywords:
            if keyword.arg == "name" and isinstance(keyword.value, ast.Constant):
                scoped.add(str(keyword.value.value))

    for node in ast.walk(tree):
        if _call_name(node) != "SetEnvironmentVariable":
            continue
        call = node
        if not call.args or not isinstance(call.args[0], ast.Constant):
            continue
        if call.args[0].value != TRANSPORT_ENV:
            continue
        env_sets += 1
        group = parents.get(call)
        while group is not None and _call_name(group) != "GroupAction":
            group = parents.get(group)
        assert group is not None, (
            f"{path.name}: {TRANSPORT_ENV} must be scoped inside a GroupAction"
        )
        for inner in ast.walk(group):
            if _call_name(inner) == "Node":
                add_node_name(inner)
            elif isinstance(inner, ast.Name):
                for assigned in assignments.get(inner.id, []):
                    if _call_name(assigned) == "Node":
                        add_node_name(assigned)
    return scoped, env_sets


def test_clock_bridge_only_is_udp_scoped_in_the_gazebo_composition():
    """The clock publisher opts in; the megabyte camera bridge stays on shared memory."""
    scoped, env_sets = _udp_scoped_nodes(GAZEBO_SIMULATION)
    assert env_sets == 1, f"expected one {TRANSPORT_ENV} opt-in, found {env_sets}"
    assert scoped == {"clock_bridge"}, scoped


def test_world_state_only_is_udp_scoped_in_the_bringup_composition():
    """The /clock subscriber (world state) opts in; camera-facing nodes do not."""
    scoped, env_sets = _udp_scoped_nodes(BRINGUP_SIMULATION)
    assert env_sets == 1, f"expected one {TRANSPORT_ENV} opt-in, found {env_sets}"
    assert scoped == {"world_state"}, scoped


def test_camera_bridge_exists_and_is_not_udp_scoped():
    """The camera bridge is present in the composition and never appears in a UDP-only group."""
    scoped, _ = _udp_scoped_nodes(GAZEBO_SIMULATION)
    source = GAZEBO_SIMULATION.read_text(encoding="utf-8")
    assert 'name="camera_bridge"' in source
    assert "camera_bridge" not in scoped


if __name__ == "__main__":
    test_clock_bridge_only_is_udp_scoped_in_the_gazebo_composition()
    test_world_state_only_is_udp_scoped_in_the_bringup_composition()
    test_camera_bridge_exists_and_is_not_udp_scoped()

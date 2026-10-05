# Copyright 2026 Matteo Bernacchia <dev@kikijiki.com>
#
# SPDX-License-Identifier: MIT

"""Static contract for the demo camera-view RViz layout and its opt-in wiring (Card 028)."""
# Everything here is read from the source tree: no GUI, no simulator, no ROS graph, no lease.
# The live proof that the panes render is SC-002's screenshot, still outstanding; this test pins
# the wiring that makes that layout reachable and keeps it out of every other launch.

import importlib.util
import os
from pathlib import Path
import re

from launch import LaunchContext
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
import pytest
import yaml


def _environment_path(name: str) -> Path:
    """Read a source-tree path from the environment ctest sets for this test."""
    try:
        return Path(os.environ[name])
    except KeyError as error:
        raise RuntimeError(
            f"{name} is not set. ament_add_pytest_test provides it under ctest; run this "
            "file as `ctest -R test_demo_camera_view_config` (or pass the four "
            "RESTOCKER_*_DIR variables by hand), not as bare pytest"
        ) from error


BRINGUP = _environment_path("RESTOCKER_BRINGUP_SOURCE_DIR")
GAZEBO = _environment_path("RESTOCKER_GAZEBO_SOURCE_DIR")
MOVEIT = _environment_path("RESTOCKER_MOVEIT_CONFIG_SOURCE_DIR")
REPO_ROOT = _environment_path("RESTOCKER_REPO_ROOT")

DEMO_CONFIG = BRINGUP / "rviz" / "demo_camera_view.rviz"
BASELINE_LAUNCH = BRINGUP / "launch" / "baseline.launch.py"
DEMO_SCRIPT = REPO_ROOT / "scripts" / "demo.bash"
GAZEBO_SIMULATION_LAUNCH = GAZEBO / "launch" / "simulation.launch.py"
MOVE_GROUP_LAUNCH = MOVEIT / "launch" / "move_group.launch.py"

# The colour image topics the camera_bridge carries in restocker_gazebo's simulation.launch.py.
BRIDGED_COLOUR_TOPICS = {"/overhead_camera/image", "/wrist_camera/image"}


def _generate(launch_path: Path, module_name: str):
    """Load a launch file as a module and build its LaunchDescription."""
    spec = importlib.util.spec_from_file_location(module_name, launch_path)
    assert spec is not None and spec.loader is not None
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module.generate_launch_description()


def _declared_default(description, name: str) -> str:
    """Resolve the default value of one declared launch argument."""
    context = LaunchContext()
    declarations = [
        entity
        for entity in description.entities
        if isinstance(entity, DeclareLaunchArgument) and entity.name == name
    ]
    assert len(declarations) == 1, f"expected exactly one DeclareLaunchArgument({name!r})"
    declarations[0].execute(context)
    return context.launch_configurations[name]


@pytest.fixture(scope="module")
def config() -> dict:
    return yaml.safe_load(DEMO_CONFIG.read_text(encoding="utf-8"))


@pytest.fixture(scope="module")
def baseline_description():
    return _generate(BASELINE_LAUNCH, "restocker_baseline_launch")


def test_demo_config_parses_and_keeps_the_planning_view(config: dict) -> None:
    manager = config["Visualization Manager"]
    assert manager["Global Options"]["Fixed Frame"] == "world"
    display_classes = {display["Class"] for display in manager["Displays"]}
    assert "moveit_rviz_plugin/MotionPlanning" in display_classes
    assert "rviz_default_plugins/Grid" in display_classes
    assert "rviz_default_plugins/TF" in display_classes
    window = config["Window Geometry"]
    # Both camera panes need room beside the 3D view; the state below docks them into it.
    assert int(window["Width"]) >= 1600
    assert int(window["Height"]) >= 800


def test_image_displays_show_the_bridged_colour_streams_reliably(config: dict) -> None:
    images = [
        display
        for display in config["Visualization Manager"]["Displays"]
        if display["Class"] == "rviz_default_plugins/Image"
    ]
    assert len(images) == 2, "exactly the wrist and overhead colour displays"
    topics = set()
    for display in images:
        assert display["Enabled"] is True
        topic = display["Topic"]
        # The bridge publishes reliable; a best-effort subscription was measured losing most
        # large frames (field notes / depth_obstacle_node), so the panes must not be best-effort.
        assert topic["Reliability Policy"] == "Reliable"
        assert topic["Durability Policy"] == "Volatile"
        assert int(topic["Depth"]) >= 1
        topics.add(topic["Value"])
    assert topics == BRIDGED_COLOUR_TOPICS


def test_referenced_topics_come_from_the_camera_bridge(config: dict) -> None:
    bridge_source = GAZEBO_SIMULATION_LAUNCH.read_text(encoding="utf-8")
    for topic in BRIDGED_COLOUR_TOPICS:
        entry = f'"{topic}@sensor_msgs/msg/Image[gz.msgs.Image"'
        assert entry in bridge_source, f"{topic} is not bridged Gazebo-to-ROS"


def test_window_geometry_docks_both_camera_panes(config: dict) -> None:
    window = config["Window Geometry"]
    state = window["QMainWindow State"]
    assert re.fullmatch(r"[0-9a-f]+", state), "state must be a hex QByteArray"
    assert bytes.fromhex(state)[:4] == bytes.fromhex("000000ff"), "QMainWindow state magic"
    for pane in ("Overhead Camera", "Wrist Camera", "Displays", "Views"):
        # Dock object names are encoded as UTF-16BE inside the state blob.
        encoded = "".join(f"{ord(character):04x}" for character in pane)
        assert encoded in state, f"state does not reference the {pane!r} pane"
        assert window[pane]["collapsed"] is False


def test_demo_passes_the_demo_rviz_config_argument() -> None:
    script = DEMO_SCRIPT.read_text(encoding="utf-8")
    assert "demo_camera_view.rviz" in script
    assert 'rviz_config:="$demo_rviz_config"' in script


def test_baseline_rviz_config_defaults_to_moveit_and_is_forwarded(
    baseline_description,
) -> None:
    default_path = _declared_default(baseline_description, "rviz_config")
    assert default_path.endswith("restocker_moveit_config/rviz/moveit.rviz"), default_path
    assert "demo_camera_view" not in default_path, "the demo layout must stay opt-in"
    forwarded = any(
        isinstance(entity, IncludeLaunchDescription)
        and any(key == "rviz_config" for key, _value in entity.launch_arguments)
        for entity in baseline_description.entities
    )
    assert forwarded, "baseline must forward rviz_config to the MoveIt include"


def test_other_launches_stay_on_the_default_layout() -> None:
    move_group_source = MOVE_GROUP_LAUNCH.read_text(encoding="utf-8")
    assert '"rviz_config"' in move_group_source
    assert "moveit.rviz" in move_group_source
    assert "demo_camera_view" not in move_group_source
    moveit_rviz = (MOVEIT / "rviz" / "moveit.rviz").read_text(encoding="utf-8")
    assert "rviz_default_plugins/Image" not in moveit_rviz
    # The chosen mechanism is RViz, not a Gazebo GUI config: gz_args stays as it is.
    gazebo_source = GAZEBO_SIMULATION_LAUNCH.read_text(encoding="utf-8")
    assert "--gui-config" not in gazebo_source
    assert "demo_camera_view" not in gazebo_source

# Copyright 2026 Matteo Bernacchia <dev@kikijiki.com>
#
# SPDX-License-Identifier: MIT

"""Single source of truth for MoveIt node parameters used by launch compositions."""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from moveit_configs_utils import MoveItConfigsBuilder
import yaml


def _flatten(data, prefix=""):
    """Flatten nested YAML into ROS parameter names (``a.b.c``), lists kept whole."""
    flat = {}
    for key, value in data.items():
        name = f"{prefix}{key}"
        if isinstance(value, dict):
            flat.update(_flatten(value, name + "."))
        else:
            flat[name] = value
    return flat


def parallel_planning_parameters():
    """
    OMPL plus PRM-fallback pipeline parameters for the free-space race (Card 053).

    Both namespaces load the same ompl_planning.yaml; the fallback namespace names PRM as its
    group planner AND as ``default_planner_config``, because upstream ompl_interface hardcodes
    the group-default entry to geometric::RRTConnect when that parameter is absent.
    """
    config_share = Path(get_package_share_directory("restocker_moveit_config"))
    with (config_share / "config" / "ompl_planning.yaml").open() as stream:
        ompl = yaml.safe_load(stream)
    parameters = _flatten(ompl, "ompl.")
    for key, value in list(parameters.items()):
        parameters[key.replace("ompl.", "ompl_fallback.", 1)] = value
    for group in ("manipulator", "arm"):
        parameters[f"ompl_fallback.{group}.planner_configs"] = ["PRMkConfigDefault"]
        parameters[f"ompl_fallback.{group}.default_planner_config"] = "PRMkConfigDefault"
    parameters["ompl_fallback.planner_configs.PRMkConfigDefault.type"] = "geometric::PRM"
    return parameters


def build_moveit_config():
    """Build the MoveIt configuration from the package's own files."""
    description_share = Path(get_package_share_directory("restocker_description"))
    return (
        MoveItConfigsBuilder("restocker", package_name="restocker_moveit_config")
        .robot_description(
            file_path=str(description_share / "urdf" / "restocker_planning.urdf.xacro")
        )
        .robot_description_semantic(file_path="config/restocker.srdf")
        .robot_description_kinematics(file_path="config/kinematics.yaml")
        .joint_limits(file_path="config/joint_limits.yaml")
        .planning_pipelines(default_planning_pipeline="ompl", pipelines=["ompl"])
        .trajectory_execution(
            file_path="config/moveit_controllers.yaml", moveit_manage_controllers=False
        )
        .planning_scene_monitor(
            publish_planning_scene=True,
            publish_geometry_updates=True,
            publish_state_updates=True,
            publish_transforms_updates=True,
            # robot_state_publisher owns the global URDF topic. Publishing MoveIt's bare planning
            # URDF there can make controller_manager consume a description without ros2_control
            # interfaces.
            publish_robot_description=False,
            publish_robot_description_semantic=False,
        )
        .to_moveit_configs()
    )

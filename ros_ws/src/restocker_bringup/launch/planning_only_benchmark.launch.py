# Copyright 2026 Matteo Bernacchia <dev@kikijiki.com>
#
# SPDX-License-Identifier: MIT

"""
Headless planning-only benchmark stack (Card 053 Phase 2).

No Gazebo, no controllers, no move_group: the ``planning_only_benchmark`` node loads the robot
model, builds an acceptance-like planning scene itself, and replays recorded pose goals against
the production OMPL configuration (plus a PRM fallback namespace raced in parallel mode). All
MoveIt parameter plumbing is done here so the node sees exactly what ``move_group`` would.
"""

from pathlib import Path
import re

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, FindExecutable, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from restocker_moveit_config import parallel_planning_parameters
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


def _move_group_robot_padding(config_share):
    """move_group's robot padding, read from its launch file (Card 068 padding audit)."""
    source = (config_share / "launch" / "move_group.launch.py").read_text(encoding="utf-8")
    declared = re.findall(
        r'"robot_description_planning\.default_robot_padding"\s*:\s*([0-9.eE+-]+)', source
    )
    if len(declared) != 1:
        message = f"expected one default_robot_padding in move_group.launch.py: {declared}"
        raise RuntimeError(message)
    return declared[0]


def generate_launch_description():
    description_share = Path(get_package_share_directory("restocker_description"))
    config_share = Path(get_package_share_directory("restocker_moveit_config"))
    gazebo_share = Path(get_package_share_directory("restocker_gazebo"))

    scenario_path = gazebo_share / "config" / "sensor_acceptance_products.yaml"
    workcell_geometry = description_share / "config" / "workcell_geometry.yaml"
    product_catalog = description_share / "config" / "product_collision_catalog.yaml"
    with scenario_path.open() as stream:
        scenario = yaml.safe_load(stream)
    workcell_pose = [float(value) for value in scenario["workcell_pose"]]

    # Same ompl + ompl_fallback (PRM) namespaces the production port nodes receive
    # (Card 053's shared helper keeps the benchmark and the race on one configuration).
    parameters = parallel_planning_parameters()

    with (config_share / "config" / "kinematics.yaml").open() as stream:
        kinematics = yaml.safe_load(stream)
    parameters.update(_flatten(kinematics, "robot_description_kinematics."))
    # Acceleration/velocity overrides (rail_joint has no acceleration in the URDF); the
    # response adapter's TOTG refuses a joint without one, which fails every otherwise valid
    # plan. RobotModelLoader applies these onto the model's bounds.
    with (config_share / "config" / "joint_limits.yaml").open() as stream:
        limits = yaml.safe_load(stream)
    parameters.update(_flatten(limits, "robot_description_planning."))
    with (config_share / "config" / "restocker.srdf").open() as stream:
        parameters["robot_description_semantic"] = stream.read()

    robot_description = Command(
        [
            FindExecutable(name="xacro"),
            " ",
            str(description_share / "urdf" / "restocker_planning.urdf.xacro"),
        ]
    )
    workcell_description = Command(
        [
            FindExecutable(name="xacro"),
            " ",
            str(description_share / "urdf" / "workcell.urdf.xacro"),
            " geometry_config:=",
            str(workcell_geometry),
        ]
    )
    parameters.update(
        {
            "robot_description": ParameterValue(robot_description, value_type=str),
            "workcell_description": ParameterValue(workcell_description, value_type=str),
            "scenario_path": str(scenario_path),
            "product_catalog_path": str(product_catalog),
            "workcell_geometry_path": str(workcell_geometry),
            "workcell_pose": workcell_pose,
            "use_sim_time": False,
            "output_csv": LaunchConfiguration("output_csv"),
            "band_label": LaunchConfiguration("band"),
            "repeats": ParameterValue(LaunchConfiguration("repeats"), value_type=int),
            "padding_audit": ParameterValue(LaunchConfiguration("padding_audit"), value_type=bool),
            "robot_padding_m": ParameterValue(
                LaunchConfiguration("robot_padding_m"), value_type=float
            ),
        }
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("output_csv", default_value="/tmp/planning_only_benchmark.csv"),
            DeclareLaunchArgument("band", default_value="quiet"),
            DeclareLaunchArgument("repeats", default_value="4"),
            # Card 068: race each goal on the unpadded (pre-068 race) and the move_group-padded
            # scene and check the unpadded plans under the padding.
            DeclareLaunchArgument("padding_audit", default_value="false"),
            DeclareLaunchArgument(
                "robot_padding_m", default_value=_move_group_robot_padding(config_share)
            ),
            Node(
                package="restocker_task_executor",
                executable="planning_only_benchmark",
                name="planning_only_benchmark",
                output="screen",
                parameters=[parameters],
            ),
        ]
    )

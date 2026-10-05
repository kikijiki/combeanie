# Copyright 2026 Matteo Bernacchia <dev@kikijiki.com>
#
# SPDX-License-Identifier: MIT

"""Launch MoveIt planning and optional RViz against an externally controlled robot."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, Shutdown
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare
from restocker_moveit_config import build_moveit_config


def _moveit_configuration():
    return build_moveit_config()


def generate_launch_description() -> LaunchDescription:
    """Create the MoveIt-only composition; simulation remains owned by bringup."""
    rviz = LaunchConfiguration("rviz")
    rviz_config = LaunchConfiguration("rviz_config")
    use_sim_time = LaunchConfiguration("use_sim_time")
    allow_trajectory_execution = LaunchConfiguration("allow_trajectory_execution")
    moveit_config = _moveit_configuration()

    move_group = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        name="move_group",
        output="screen",
        parameters=[
            moveit_config.to_dict(),
            {
                "use_sim_time": ParameterValue(use_sim_time, value_type=bool),
                "allow_trajectory_execution": ParameterValue(
                    allow_trajectory_execution, value_type=bool
                ),
                "monitor_dynamics": False,
                # Collision padding on every robot link, applied by this node's planning scene
                # monitor. A state is valid at any positive clearance and OMPL's path
                # simplification pulls the solution taut against obstacles, so an unpadded planner
                # produces plans that graze. Observed on the acceptance test: the gripper body
                # passed 0.2 mm over a stock bottle and tipped it, and a closed finger passed 0.4
                # mm from the target can and shifted it 24 mm, truncating the following Cartesian
                # approach. The planning scene was exact, so this is a margin problem;
                # longest_valid_segment_fraction cannot fix it because the checked states
                # themselves have no clearance.
                #
                # Bounds on the value, measured with independent forward kinematics against Gazebo
                # ground-truth product poses:
                #   * The fingers pass the target product at 2.63 mm at the tightest sample with
                # the     jaws at the approach aperture (design 5 mm per side, less the product's
                #     settled offset and fk's own 1.1 mm). Padding at or above that makes the
                #     Cartesian approach unplannable.
                #   * Every other link stays clear of the products: 25.00 mm for the gripper body
                #     against the large bottle, 9.99 mm for the wrist camera against it.
                #   * The carried product is an attached body, and MoveIt pads those by their
                # parent     link's padding (default_attached_padding is read but never applied, as
                #     planning_scene_projector_node.cpp notes). The projector works around that by
                #     shrinking the attached cylinder by attachment_settle_clearance_m = 2 mm at
                #     each end, so a product resting on its support does not read as a start-state
                #     collision. Padding gives that allowance back, and products were measured
                #     sitting on the tray within 0.01 mm of nominal, so there is no slack. The
                # third     bound is binding, so 2 mm is not available. At 0.0025 the first retract
                # off     the stock tray stopped at 0% on
                # restocker/workcell/shelf/stock_tray_collision     against restocker/object/1 (the
                # carried can, inflated past its settle     allowance, against the tray it was
                # still over). 0.0015 keeps 0.5 mm of that     allowance and 1.13 mm of the
                # approach, and is four to seven times the grazes     it exists to stop. Raise
                # attachment_settle_clearance_m in step if this needs     to go higher.
                #
                # The seeded scenario generator rejects a product pair the fingers cannot pass
                # between, measured with the padded finger. It cannot read this file
                # (restocker_gazebo does not depend on the MoveIt configuration), so it carries the
                # number as restocker_gazebo.scenario_config.PLANNER_ROBOT_PADDING_M. Change both
                # together and bump SUPPORTED_GENERATOR_VERSION: every recorded seed shifts.
                "robot_description_planning.default_robot_padding": 0.0015,
            },
        ],
        on_exit=Shutdown(reason="move_group exited"),
    )

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="moveit_rviz",
        output="screen",
        arguments=["-d", rviz_config],
        parameters=[
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.robot_description_kinematics,
            moveit_config.planning_pipelines,
            moveit_config.joint_limits,
            {"use_sim_time": ParameterValue(use_sim_time, value_type=bool)},
        ],
        condition=IfCondition(rviz),
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("rviz", default_value="true", description="Start MoveIt RViz"),
            DeclareLaunchArgument(
                "rviz_config",
                default_value=PathJoinSubstitution(
                    [FindPackageShare("restocker_moveit_config"), "rviz", "moveit.rviz"]
                ),
                description="Absolute RViz configuration path",
            ),
            DeclareLaunchArgument(
                "use_sim_time", default_value="true", description="Use the Gazebo clock"
            ),
            DeclareLaunchArgument(
                "allow_trajectory_execution",
                default_value="true",
                description="Allow MoveIt to execute validated trajectories",
            ),
            move_group,
            rviz_node,
        ]
    )

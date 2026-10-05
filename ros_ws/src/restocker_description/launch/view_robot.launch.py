# Copyright 2026 Matteo Bernacchia <dev@kikijiki.com>
#
# SPDX-License-Identifier: MIT

"""Launch the robot description and RViz visualization."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description() -> LaunchDescription:
    """Build the visualization launch description without writing generated URDF files."""
    gui = LaunchConfiguration("gui")
    prefix = LaunchConfiguration("prefix")
    rviz = LaunchConfiguration("rviz")
    use_sim_time = LaunchConfiguration("use_sim_time")

    xacro_file = PathJoinSubstitution(
        [FindPackageShare("restocker_description"), "urdf", "restocker.urdf.xacro"]
    )
    rviz_config = PathJoinSubstitution(
        [FindPackageShare("restocker_description"), "rviz", "restocker.rviz"]
    )
    robot_description = ParameterValue(
        Command([FindExecutable(name="xacro"), " ", xacro_file, " prefix:=", prefix]),
        value_type=str,
    )

    common_parameters = [
        {"robot_description": robot_description},
        {"use_sim_time": use_sim_time},
    ]

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "gui", default_value="true", description="Use the graphical joint-state controls"
            ),
            DeclareLaunchArgument(
                "rviz", default_value="true", description="Start RViz with the checked-in view"
            ),
            DeclareLaunchArgument(
                "prefix", default_value="", description="Optional frame and joint name prefix"
            ),
            DeclareLaunchArgument(
                "use_sim_time", default_value="false", description="Read time from /clock"
            ),
            Node(
                package="robot_state_publisher",
                executable="robot_state_publisher",
                name="robot_state_publisher",
                output="screen",
                parameters=common_parameters,
            ),
            Node(
                package="joint_state_publisher_gui",
                executable="joint_state_publisher_gui",
                name="joint_state_publisher",
                output="screen",
                condition=IfCondition(gui),
                parameters=common_parameters,
            ),
            Node(
                package="joint_state_publisher",
                executable="joint_state_publisher",
                name="joint_state_publisher",
                output="screen",
                condition=UnlessCondition(gui),
                parameters=common_parameters,
            ),
            Node(
                package="rviz2",
                executable="rviz2",
                name="rviz2",
                output="screen",
                arguments=["-d", rviz_config],
                condition=IfCondition(rviz),
                parameters=[{"use_sim_time": use_sim_time}],
            ),
        ]
    )

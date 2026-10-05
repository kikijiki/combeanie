# Copyright 2026 Matteo Bernacchia <dev@kikijiki.com>
#
# SPDX-License-Identifier: MIT

"""Require one product to be solid to another in the shipped world file."""

# Smallest reproduction of the defect: four products spawned by the same `ros_gz_sim create`
# call `simulation.launch.py` uses, into the same `worlds/restocking.sdf`, expanded from the same
# `urdf/product.urdf.xacro`. No arm, gripper, attachment or controller; the products
# interpenetrated on their own.
#
# The real world file is used rather than a stripped fixture because the fault was in the
# world's physics configuration.

import os
from pathlib import Path
import unittest

from ament_index_python.packages import get_package_share_directory
import launch
from launch.actions import (
    EmitEvent,
    ExecuteProcess,
    RegisterEventHandler,
    SetEnvironmentVariable,
)
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.substitutions import Command, EnvironmentVariable, FindExecutable
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
import launch_testing
import launch_testing.actions
import launch_testing.asserts
import pytest

# The large bottle of restocker_description/config/product_collision_catalog.yaml: the widest
# catalogued product, so the pair separations are the largest the shelf has to resolve.
# product_contact_probe.cpp restates the derived height and diameter it checks against.
PRODUCT_RADIUS_M = 0.045
PRODUCT_HEIGHT_M = 0.290
PRODUCT_MASS_KG = 1.100
PRODUCT_FRICTION = 0.76

# Two independent pairs, on bare floor well clear of each other and of the origin.
#
# `stack` releases one product 0.455 m above another standing on the floor. `column` puts two
# products 0.05 m apart in y, closer than their 0.09 m diameter: the state a placement creates
# when the arm opens its jaws over a lane that already holds a product, in which a two-product
# column was measured consuming one product's worth of lane depth.
#
# `pinned` is `column` with the first product spawned `static`, as every stocked front product in
# the dense demo fixture is (restocker_gazebo/lane_columns.py). The demo turns its column-growth
# placement proof off on the claim that a placed product can pass through a pinned one; this
# measures that claim.
SPAWNS = (
    ("contact_stack_lower", 1.50, 1.50, PRODUCT_HEIGHT_M / 2.0, False),
    ("contact_stack_upper", 1.50, 1.50, 0.600, False),
    ("contact_column_near", 1.50, -1.50, PRODUCT_HEIGHT_M / 2.0, False),
    ("contact_column_far", 1.50, -1.45, PRODUCT_HEIGHT_M / 2.0, False),
    ("contact_pinned_static", -1.50, 1.50, PRODUCT_HEIGHT_M / 2.0, True),
    ("contact_pinned_placed", -1.50, 1.55, PRODUCT_HEIGHT_M / 2.0, False),
)


@pytest.mark.launch_test
def generate_test_description():
    """Start the shipped world, spawn six real products into it, and run the owned probe."""
    world = Path(get_package_share_directory("restocker_gazebo")) / "worlds" / "restocking.sdf"
    product_xacro = str(
        Path(get_package_share_directory("restocker_gazebo")) / "urdf" / "product.urdf.xacro"
    )

    def description(static: bool) -> ParameterValue:
        """Emit the shipped product description, optionally pinned in place like a stocked lane."""
        return ParameterValue(
            Command(
                [
                    FindExecutable(name="xacro"),
                    " ",
                    product_xacro,
                    " radius:=",
                    str(PRODUCT_RADIUS_M),
                    " height:=",
                    str(PRODUCT_HEIGHT_M),
                    " mass:=",
                    str(PRODUCT_MASS_KG),
                    " friction:=",
                    str(PRODUCT_FRICTION),
                    " static:=",
                    "true" if static else "false",
                ]
            ),
            value_type=str,
        )

    gazebo = ExecuteProcess(
        cmd=["gz", "sim", "-s", "-r", str(world)],
        name="product_contact_gazebo",
        output="screen",
    )
    spawns = [
        Node(
            package="ros_gz_sim",
            executable="create",
            name=f"spawn_{name}",
            output="screen",
            parameters=[
                {
                    "world": "restocking",
                    "string": description(static),
                    "name": name,
                    "allow_renaming": False,
                    "x": x,
                    "y": y,
                    "z": z,
                }
            ],
        )
        for name, x, y, z, static in SPAWNS
    ]
    probe = ExecuteProcess(
        cmd=[os.environ["RESTOCKER_PRODUCT_CONTACT_PROBE"]],
        name="product_contact_probe",
        output="screen",
    )
    stop_after_probe = RegisterEventHandler(
        OnProcessExit(
            target_action=probe,
            on_exit=[EmitEvent(event=Shutdown(reason="product contact probe completed"))],
        )
    )
    return (
        launch.LaunchDescription(
            [
                # The world names the attachment system unconditionally. Nothing here attaches
                # anything, but a half-loaded world is not the world the simulation runs.
                SetEnvironmentVariable(
                    "GZ_SIM_SYSTEM_PLUGIN_PATH",
                    [
                        os.environ["RESTOCKER_ATTACHMENT_PLUGIN_DIR"],
                        ":",
                        EnvironmentVariable("GZ_SIM_SYSTEM_PLUGIN_PATH", default_value=""),
                    ],
                ),
                gazebo,
                *spawns,
                probe,
                stop_after_probe,
                launch_testing.actions.ReadyToTest(),
            ]
        ),
        {"probe": probe},
    )


class TestProductContactRuntime(unittest.TestCase):
    """Require the probe's contact measurements to hold."""

    def test_products_are_solid_to_each_other(self, proc_info, probe):
        """Fail with the probe's measured separations when a product is not solid."""
        proc_info.assertWaitForShutdown(process=probe, timeout=120)
        launch_testing.asserts.assertExitCodes(proc_info, process=probe)

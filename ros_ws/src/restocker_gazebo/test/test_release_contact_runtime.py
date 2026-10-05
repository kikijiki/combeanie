# Copyright 2026 Matteo Bernacchia <dev@kikijiki.com>
#
# SPDX-License-Identifier: MIT

"""Run the attach/carry/release/settle path against a pinned product column."""

# The renderer-independent half of the product-contact claim: `test_product_contact_runtime`
# proves spawned products are solid to each other in the shipped world (and carries the
# `renderer` label because that world loads the sensors system). This one loads a fixture world
# with physics, the scene broadcaster and the attachment plugin only -- no sensors system, no
# OpenGL -- so it runs under `nix flake check`, and it exercises the path a real placement takes:
# the production attachment plugin welds the product into the gripper, a prismatic insert carries
# it to the release depth, the plugin detaches it 8 mm above an inclined roller bed, and the bed
# delivers it into a pinned column. The probe fails when the released product passes through
# that column or stays interpenetrating with it, and prints the separation, penetration, column
# growth and terminal placement-evidence verdict as a receipt.

import os
import unittest

import launch
from launch.actions import EmitEvent, ExecuteProcess, RegisterEventHandler, SetEnvironmentVariable
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.substitutions import EnvironmentVariable
import launch_testing
import launch_testing.actions
import launch_testing.asserts
import pytest


@pytest.mark.launch_test
def generate_test_description():
    """Start the release-contact fixture and its measurement probe."""
    gazebo = ExecuteProcess(
        cmd=["gz", "sim", "-s", "-r", os.environ["RESTOCKER_RELEASE_CONTACT_TEST_WORLD"]],
        name="release_contact_gazebo",
        output="screen",
    )
    probe = ExecuteProcess(
        cmd=[os.environ["RESTOCKER_RELEASE_CONTACT_PROBE"]],
        name="release_contact_probe",
        output="screen",
    )
    stop_after_probe = RegisterEventHandler(
        OnProcessExit(
            target_action=probe,
            on_exit=[EmitEvent(event=Shutdown(reason="release contact probe completed"))],
        )
    )
    return (
        launch.LaunchDescription(
            [
                SetEnvironmentVariable(
                    "GZ_SIM_SYSTEM_PLUGIN_PATH",
                    [
                        os.environ["RESTOCKER_ATTACHMENT_PLUGIN_DIR"],
                        ":",
                        EnvironmentVariable("GZ_SIM_SYSTEM_PLUGIN_PATH", default_value=""),
                    ],
                ),
                SetEnvironmentVariable(
                    "RESTOCKER_ATTACHMENT_BOUNDARY_CONFIG",
                    os.environ["RESTOCKER_TEST_ATTACHMENT_BOUNDARY"],
                ),
                SetEnvironmentVariable(
                    "RESTOCKER_GRIPPER_GEOMETRY_CONFIG",
                    os.environ["RESTOCKER_TEST_GRIPPER_GEOMETRY"],
                ),
                SetEnvironmentVariable(
                    "RESTOCKER_PRODUCT_CATALOG_CONFIG",
                    os.environ["RESTOCKER_TEST_PRODUCT_CATALOG"],
                ),
                SetEnvironmentVariable(
                    "RESTOCKER_SCENARIO_CONFIG", os.environ["RESTOCKER_TEST_SCENARIO"]
                ),
                gazebo,
                probe,
                stop_after_probe,
                launch_testing.actions.ReadyToTest(),
            ]
        ),
        {"probe": probe},
    )


class TestReleaseContactRuntime(unittest.TestCase):
    """Require the probe's release-contact measurements to hold."""

    def test_released_product_is_solid_to_the_pinned_column(self, proc_info, probe):
        """Fail with the probe's receipt when release contact does not resolve."""
        proc_info.assertWaitForShutdown(process=probe, timeout=120)
        launch_testing.asserts.assertExitCodes(proc_info, process=probe)

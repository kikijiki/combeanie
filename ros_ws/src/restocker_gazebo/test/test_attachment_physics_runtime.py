# Copyright 2026 Matteo Bernacchia <dev@kikijiki.com>
#
# SPDX-License-Identifier: MIT

"""Exercise Gazebo fixed-joint attachment and detachment acknowledgement."""

import os
from pathlib import Path
import tempfile
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


def _physical_gripper_geometry() -> str:
    """Opt this fixture into hold_mode: physical, which is no longer the shipped mode."""
    source = Path(os.environ["RESTOCKER_TEST_GRIPPER_GEOMETRY"]).read_text(encoding="utf-8")
    if "hold_mode: weld" not in source and "hold_mode: physical" not in source:
        raise AssertionError("gripper_geometry.yaml carries no hold_mode")
    target = Path(tempfile.gettempdir()) / f"restocker_{Path(__file__).stem}_gripper_physical.yaml"
    target.write_text(source.replace("hold_mode: weld", "hold_mode: physical"), encoding="utf-8")
    return str(target)


@pytest.mark.launch_test
def generate_test_description():
    """Start the isolated physics fixture and its typed transport probe."""
    gazebo = ExecuteProcess(
        cmd=["gz", "sim", "-s", "-r", os.environ["RESTOCKER_ATTACHMENT_TEST_WORLD"]],
        name="attachment_physics_gazebo",
        output="screen",
    )
    probe = ExecuteProcess(
        cmd=[os.environ["RESTOCKER_ATTACHMENT_PROBE"]],
        name="attachment_physics_probe",
        output="screen",
    )
    stop_after_probe = RegisterEventHandler(
        OnProcessExit(
            target_action=probe,
            on_exit=[EmitEvent(event=Shutdown(reason="attachment physics probe completed"))],
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
                    _physical_gripper_geometry(),
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


class TestAttachmentPhysicsRuntime(unittest.TestCase):
    """Require the complete physical mutation sequence to terminate successfully."""

    def test_probe_succeeds(self, proc_info, probe):
        proc_info.assertWaitForShutdown(process=probe, timeout=20)
        launch_testing.asserts.assertExitCodes(proc_info, process=probe)

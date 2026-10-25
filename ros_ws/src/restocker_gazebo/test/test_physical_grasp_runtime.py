# Copyright 2026 Matteo Bernacchia <dev@kikijiki.com>
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""Carry a product on friction alone: the physical hold through insert and release."""

# SC-002 of Card 057 on `fixtures/physical_grasp.sdf`: production's side grasp (jaws closing
# across the insert, so the carry and gravity load the pads tangentially) holds the product by
# contact and friction alone -- no weld in the world -- through close, verified attach, the 0.12 m
# insert and release onto the roller bed. The probe fails on a refused attach, on held drift over
# the 1 mm bound, or on any release-contact failure, and the fixture's GripForceProbe logs the
# measured squeeze. No sensors system, so it runs under `nix flake check`.

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
    """Start the physical-grasp fixture and its measurement probe."""
    gazebo = ExecuteProcess(
        # -v 2: the boundary's held-object watchdog reports a latch, with its measurement, only
        # as a gz warning, and SC-002 must be able to say why a grip was declared lost.
        cmd=[
            "gz",
            "sim",
            "-s",
            "-r",
            "-v",
            "2",
            os.environ["RESTOCKER_RELEASE_CONTACT_TEST_WORLD"],
        ],
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
                    _physical_gripper_geometry(),
                ),
                SetEnvironmentVariable("RESTOCKER_RELEASE_CONTACT_HOLD_MODE", "physical"),
                SetEnvironmentVariable("RESTOCKER_RELEASE_CONTACT_HOLD_TARGET", "0.0093"),
                SetEnvironmentVariable("RESTOCKER_RELEASE_CONTACT_SETTLE_MS", "3000"),
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


class TestPhysicalGraspRuntime(unittest.TestCase):
    """Require the probe's release-contact measurements to hold."""

    def test_product_is_carried_on_friction_alone(self, proc_info, probe):
        """Fail with the probe's receipt when release contact does not resolve."""
        proc_info.assertWaitForShutdown(process=probe, timeout=120)
        launch_testing.asserts.assertExitCodes(proc_info, process=probe)

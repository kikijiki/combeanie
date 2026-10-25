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

"""Press one pad row against a fixed block and check the rubber this design claims."""

# SC-002's grip force read 8.7 N from the contact wrenches while the pad compression in the same
# run implied 213 N through the springs spec M10 section 11 D10 item 3 specifies, and the two
# cannot both be right. This bench takes the full grasp out of the loop: one finger, one loaded
# pad row, a fixed block, and three KNOWN constant efforts (10, 20 and 40 N). At 111 kN/m the
# deflection is effort/111000 and the contact wrench should read the applied effort, so a factor
# of ~25 in either direction names which path was lying. The verdict is computed by the bench's
# own gz system, which prints PAD_SPRING_PASS or PAD_SPRING_FAIL.

import os
import unittest

import launch
from launch.actions import ExecuteProcess, SetEnvironmentVariable
from launch.substitutions import EnvironmentVariable
import launch_testing
import launch_testing.actions
import launch_testing.asserts
import pytest


@pytest.mark.launch_test
def generate_test_description():
    """Run the pad-spring bench headless and stop when it finishes."""
    gazebo = ExecuteProcess(
        cmd=[
            "gz",
            "sim",
            "-s",
            "-r",
            os.environ["RESTOCKER_PAD_SPRING_TEST_WORLD"],
        ],
        name="pad_spring_gazebo",
        output="screen",
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
                gazebo,
                launch_testing.actions.ReadyToTest(),
            ]
        ),
        {"gazebo": gazebo},
    )


class TestPadSpringBench(unittest.TestCase):
    """Require the measured stiffness and wrench to match the specified pad rubber."""

    def test_pad_spring_deflection_and_wrench(self, proc_output, gazebo):
        """Fail with the bench's own receipt when the springs or the wrench disagree."""
        proc_output.assertWaitFor("PAD_SPRING_PASS", process=gazebo, timeout=45)

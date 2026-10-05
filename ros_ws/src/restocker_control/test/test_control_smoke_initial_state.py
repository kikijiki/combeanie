# Copyright 2026 Matteo Bernacchia <dev@kikijiki.com>
#
# SPDX-License-Identifier: MIT

"""Runtime failure contracts for fresh simulated finger-state validation."""

from __future__ import annotations

import math
import threading
import time
import unittest

from controller_manager_msgs.msg import ControllerState
from controller_manager_msgs.srv import ListControllers
import launch
import launch_ros.actions
import launch_testing
import launch_testing.actions
import launch_testing.asserts
import launch_testing.util
import rclpy
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import JointState


def _smoke_node(name: str, action_timeout_sec: float, joint_state_topic: str):
    return launch_ros.actions.Node(
        package="restocker_control",
        executable="control_smoke_test",
        name=name,
        parameters=[
            {
                "startup_timeout_sec": 5.0,
                "controller_response_attempt_timeout_sec": 0.5,
                "action_timeout_sec": action_timeout_sec,
            }
        ],
        remappings=[("/joint_states", joint_state_topic)],
        output="screen",
    )


def generate_test_description():
    no_state = _smoke_node("smoke_no_initial_state", 0.35, "/test/no_state")
    invalid_state = _smoke_node("smoke_invalid_initial_state", 1.0, "/test/invalid_state")
    out_of_range = _smoke_node("smoke_out_of_range_initial_state", 1.0, "/test/out_of_range")
    recovering = _smoke_node("smoke_recovering_initial_state", 1.5, "/test/recovering")
    return (
        launch.LaunchDescription(
            [
                no_state,
                invalid_state,
                out_of_range,
                recovering,
                launch_testing.util.KeepAliveProc(),
                launch_testing.actions.ReadyToTest(),
            ]
        ),
        {
            "no_state": no_state,
            "invalid_state": invalid_state,
            "out_of_range": out_of_range,
            "recovering": recovering,
        },
    )


def _controller(name: str, plugin_type: str, claimed_interfaces: list[str]) -> ControllerState:
    controller = ControllerState()
    controller.name = name
    controller.type = plugin_type
    controller.state = "active"
    controller.claimed_interfaces = claimed_interfaces
    return controller


class TestControlSmokeInitialState(unittest.TestCase):
    @classmethod
    def setUpClass(cls) -> None:
        rclpy.init()
        cls.node = rclpy.create_node("control_smoke_initial_state_fixture")
        cls.request_count = 0
        cls.request_count_lock = threading.Lock()
        cls.release_controller_responses = threading.Event()
        cls.executor = None
        cls.executor_thread = None
        cls.service = None
        cls.publishers = {
            topic: cls.node.create_publisher(JointState, topic, qos_profile_sensor_data)
            for topic in (
                "/test/no_state",
                "/test/invalid_state",
                "/test/out_of_range",
                "/test/recovering",
            )
        }

    @classmethod
    def tearDownClass(cls) -> None:
        cls.release_controller_responses.set()
        if cls.executor is not None:
            cls.executor.shutdown(timeout_sec=2.0)
        if cls.executor_thread is not None:
            cls.executor_thread.join(timeout=2.0)
        for publisher in cls.publishers.values():
            cls.node.destroy_publisher(publisher)
        if cls.service is not None:
            cls.node.destroy_service(cls.service)
        cls.node.destroy_node()
        rclpy.shutdown()

    @classmethod
    def _wait_for_controller_requests(cls) -> None:
        deadline = time.monotonic() + 4.0
        observed = 0
        while time.monotonic() < deadline:
            with cls.request_count_lock:
                observed = cls.request_count
            if observed >= 4:
                return
            time.sleep(0.02)
        if observed < 4:
            raise AssertionError("all smoke clients did not validate controllers")

    @classmethod
    def _wait_for_joint_state_subscriptions(cls) -> None:
        deadline = time.monotonic() + 4.0
        while time.monotonic() < deadline:
            if all(
                publisher.get_subscription_count() == 1 for publisher in cls.publishers.values()
            ):
                return
            time.sleep(0.02)
        raise AssertionError("smoke-client joint-state subscriptions were not discovered")

    @classmethod
    def _publish_for(cls, samples: dict[str, JointState], duration_sec: float) -> None:
        deadline = time.monotonic() + duration_sec
        while time.monotonic() < deadline:
            for topic, sample in samples.items():
                cls.publishers[topic].publish(sample)
            time.sleep(0.02)

    @classmethod
    def _create_controller_service(cls) -> None:
        def respond(_request, response):
            with cls.request_count_lock:
                cls.request_count += 1
            if not cls.release_controller_responses.wait(timeout=4.0):
                raise RuntimeError("test did not release controller responses")
            trajectory_type = "joint_trajectory_controller/JointTrajectoryController"
            response.controller = [
                _controller(
                    "joint_state_broadcaster",
                    "joint_state_broadcaster/JointStateBroadcaster",
                    [],
                ),
                _controller(
                    "arm_controller",
                    trajectory_type,
                    [
                        f"{joint}/velocity"
                        for joint in (
                            "shoulder_pan_joint",
                            "shoulder_lift_joint",
                            "elbow_joint",
                            "wrist_1_joint",
                            "wrist_2_joint",
                            "wrist_3_joint",
                        )
                    ],
                ),
                _controller("rail_controller", trajectory_type, ["rail_joint/velocity"]),
                _controller(
                    "gripper_controller",
                    trajectory_type,
                    ["left_finger_joint/position", "right_finger_joint/position"],
                ),
            ]
            return response

        cls.service = cls.node.create_service(
            ListControllers,
            "/controller_manager/list_controllers",
            respond,
            callback_group=ReentrantCallbackGroup(),
        )
        cls.executor = MultiThreadedExecutor(num_threads=5)
        cls.executor.add_node(cls.node)
        cls.executor_thread = threading.Thread(target=cls.executor.spin, daemon=True)
        cls.executor_thread.start()

    @staticmethod
    def _finger_state(left: float, right: float | None = None) -> JointState:
        sample = JointState()
        sample.name = ["left_finger_joint"]
        sample.position = [left]
        if right is not None:
            sample.name.append("right_finger_joint")
            sample.position.append(right)
        return sample

    def test_timeout_and_invalid_state_fail_before_action_discovery(
        self,
        no_state,
        invalid_state,
        out_of_range,
        recovering,
        proc_info,
        proc_output,
    ) -> None:
        self._wait_for_joint_state_subscriptions()
        self._create_controller_service()
        self._wait_for_controller_requests()

        valid = self._finger_state(0.005, 0.005)
        self._publish_for({topic: valid for topic in self.publishers}, duration_sec=0.3)
        self.release_controller_responses.set()
        self._publish_for({}, duration_sec=0.1)

        proc_info.assertWaitForShutdown(process=no_state, timeout=2.0)
        launch_testing.asserts.assertExitCodes(
            proc_info, allowable_exit_codes=[32], process=no_state
        )
        proc_output.assertWaitFor(
            "no fresh joint state was received after controller readiness",
            process=no_state,
            timeout=1.0,
        )

        invalid = self._finger_state(math.nan)
        finite_error = self._finger_state(0.0, 0.0)
        self._publish_for(
            {
                "/test/invalid_state": invalid,
                "/test/out_of_range": finite_error,
                "/test/recovering": finite_error,
            },
            duration_sec=0.2,
        )
        self._publish_for(
            {
                "/test/invalid_state": invalid,
                "/test/out_of_range": finite_error,
                "/test/recovering": valid,
            },
            duration_sec=1.0,
        )

        proc_info.assertWaitForShutdown(process=invalid_state, timeout=2.0)
        launch_testing.asserts.assertExitCodes(
            proc_info, allowable_exit_codes=[33], process=invalid_state
        )
        proc_output.assertWaitFor(
            "joint position is not finite: left_finger_joint",
            process=invalid_state,
            timeout=1.0,
        )
        proc_output.assertWaitFor(
            "joint state is missing right_finger_joint",
            process=invalid_state,
            timeout=1.0,
        )

        proc_info.assertWaitForShutdown(process=out_of_range, timeout=2.0)
        launch_testing.asserts.assertExitCodes(
            proc_info, allowable_exit_codes=[33], process=out_of_range
        )
        proc_output.assertWaitFor(
            "left_finger_joint position error 0.005 exceeds tolerance 0.003",
            process=out_of_range,
            timeout=1.0,
        )

        proc_info.assertWaitForShutdown(process=recovering, timeout=6.0)
        launch_testing.asserts.assertExitCodes(
            proc_info, allowable_exit_codes=[20], process=recovering
        )
        proc_output.assertWaitFor(
            "fresh joint state confirms interior simulated finger positions",
            process=recovering,
            timeout=1.0,
        )
        proc_output.assertWaitFor(
            "rail_controller action server timed out",
            process=recovering,
            timeout=1.0,
        )

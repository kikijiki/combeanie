# Copyright 2026 Matteo Bernacchia <dev@kikijiki.com>
#
# SPDX-License-Identifier: MIT

"""Interface checks for the telemetry-only robot input DTO."""

from restocker_interfaces.msg import RobotTelemetry


def test_robot_telemetry_has_fixed_canonical_shape() -> None:
    message = RobotTelemetry()

    assert list(message.joint_names) == ["", "", "", "", "", ""]
    assert list(message.joint_positions) == [0.0] * 6
    assert list(message.joint_velocities) == [0.0] * 6
    assert message.rail_velocity == 0.0
    assert list(message.gripper_joint_names) == ["", ""]
    assert list(message.gripper_joint_positions) == [0.0, 0.0]
    assert list(message.gripper_joint_velocities) == [0.0, 0.0]
    assert [
        message.JOINT_1_NAME,
        message.JOINT_2_NAME,
        message.JOINT_3_NAME,
        message.JOINT_4_NAME,
        message.JOINT_5_NAME,
        message.JOINT_6_NAME,
    ] == [
        "shoulder_pan_joint",
        "shoulder_lift_joint",
        "elbow_joint",
        "wrist_1_joint",
        "wrist_2_joint",
        "wrist_3_joint",
    ]
    assert [
        message.LEFT_FINGER_JOINT_NAME,
        message.RIGHT_FINGER_JOINT_NAME,
    ] == ["left_finger_joint", "right_finger_joint"]


def test_robot_telemetry_cannot_encode_authoritative_semantics() -> None:
    fields = RobotTelemetry.get_fields_and_field_types()

    assert "task_phase" not in fields
    assert "fault_state" not in fields
    assert "held_object" not in fields
    assert "revision" not in fields

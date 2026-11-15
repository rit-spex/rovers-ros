"""Tests for the shared arm constants and the constants-package bridge."""

import math

from constants import (
    ARM_ENABLED_TOPIC,
    ARM_JOINT_NAMES,
    ARM_JOINTS,
    ARM_MOTOR_IDX,
    ARM_OPERATION_MODE_LABELS,
    ARM_OPERATION_MODES,
    ARM_STATE_TOPICS,
    CURRENT_ANGLE_TOPICS,
    GRIPPER_MAX_ANGLE,
    GRIPPER_MIN_ANGLE,
    HOMING_TARGET,
    MECHANICAL_LIMITS,
    MOTOR_JOINT_NAMES,
    TARGET_ANGLE_TOPICS,
    angle_limits,
)


def test_shared_constants_package_still_imports():
    """The arm module shadows the workspace package; __path__ must bridge."""
    from constants.CAN_constants import CAN_MESSAGE_IDS

    assert CAN_MESSAGE_IDS.SEND_BASE is not None


def test_motor_order_matches_the_joint_order():
    """Motor indices and joint names must stay in the same order."""
    assert [MOTOR_JOINT_NAMES[member] for member in ARM_MOTOR_IDX] == (
        list(ARM_JOINTS)
    )


def test_motor_topics_follow_the_canonical_joint_order():
    """Motor index topic maps must use the shared joint names."""
    assert TARGET_ANGLE_TOPICS[ARM_MOTOR_IDX.BASE] == "/ARM/JOINT/TARGET_BASE"
    assert CURRENT_ANGLE_TOPICS[ARM_MOTOR_IDX.GRIPPER] == (
        "/ARM/JOINT/CURRENT_GRIPPER"
    )


def test_enabled_topic_matches_the_state_map():
    """The enabled topic is part of the state topic map."""
    assert ARM_ENABLED_TOPIC == ARM_STATE_TOPICS["enabled"]


def test_operation_modes_match_their_labels():
    """Every wire code must have exactly one label."""
    assert set(ARM_OPERATION_MODES.values()) == set(ARM_OPERATION_MODE_LABELS)
    assert len(ARM_OPERATION_MODES) == len(ARM_OPERATION_MODE_LABELS)


def test_gripper_travel_is_zero_to_ninety_degrees():
    """The hardware calibration and the controller agree on the gripper."""
    lower, upper = angle_limits(ARM_MOTOR_IDX.GRIPPER)
    assert (lower, upper) == (GRIPPER_MIN_ANGLE, GRIPPER_MAX_ANGLE)
    assert lower == 0.0
    assert upper == math.pi / 2


def test_home_pose_is_five_dof():
    """Home pose and mechanical limits cover the five model joints."""
    assert len(ARM_JOINT_NAMES) == 5
    assert len(HOMING_TARGET) == 5
    assert len(MECHANICAL_LIMITS) == 5


def test_controller_tuning_lives_in_the_shared_module():
    """Controller tuning is part of the one constants module."""
    import constants

    assert constants.A0 == 6.5
    assert constants.DT == 0.05
    assert constants.HOMING_TARGET is HOMING_TARGET
    assert constants.MAX_ANG_VEL > 0
    assert constants.PLANAR_BASE == math.radians(5)

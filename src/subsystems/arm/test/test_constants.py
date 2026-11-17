"""Tests for the arm constants and the shared workspace constants package."""

import math

from arm_constants import (
    ARM_ENABLED_TOPIC,
    ARM_JOINT_NAMES,
    ARM_JOINTS,
    ARM_MOTOR_IDX,
    ARM_OPERATION_MODE_LABELS,
    ARM_OPERATION_MODES,
    ARM_STATE_TOPICS,
    CURRENT_ANGLE_TOPICS,
    END_EFFECTOR_COMMANDS,
    FULL_RANGE_LIMITS,
    GRIPPER_MAX_ANGLE,
    GRIPPER_MIN_ANGLE,
    HOMING_LIMITS,
    HOMING_TARGET,
    MECHANICAL_LIMITS,
    MOTOR_JOINT_NAMES,
    PICK_AND_PLACE_FLIPPED_LIMITS,
    PICK_AND_PLACE_LIMITS,
    PLANAR_LIMITS,
    PRE_READY_END_EFFECTOR_COMMANDS,
    TARGET_ANGLE_TOPICS,
    angle_limits,
)


def test_shared_constants_package_still_imports():
    """The shared workspace constants package resolves independently."""
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


def test_wrist_twist_travel_is_plus_minus_45_degrees():
    """The operator contract: wrist twist travels +/-45 degrees everywhere."""
    lower, upper = MECHANICAL_LIMITS[4]
    assert math.isclose(math.degrees(lower), -45.0, abs_tol=1e-9)
    assert math.isclose(math.degrees(upper), 45.0, abs_tol=1e-9)
    for limits in (
        FULL_RANGE_LIMITS,
        PICK_AND_PLACE_LIMITS,
        PICK_AND_PLACE_FLIPPED_LIMITS,
        PLANAR_LIMITS,
        HOMING_LIMITS,
    ):
        assert limits[4] == (lower, upper)


def test_end_effector_commands_are_owned_outside_the_planner():
    """Gripper and solenoid commands stay with the end effector."""
    assert END_EFFECTOR_COMMANDS == {
        "gripper_up",
        "gripper_down",
        "fire_solenoid",
        "toggle_solenoid",
    }
    assert PRE_READY_END_EFFECTOR_COMMANDS <= END_EFFECTOR_COMMANDS


def test_master_mux_owns_the_velocity_boundary():
    """Manual teleop feeds the master, which publishes the arbitrated topic."""
    import arm_constants

    assert arm_constants.MANUAL_VELOCITY_TOPIC == "/ARM/MANUAL/CMD_VELOCITY"
    assert arm_constants.VELOCITY_TOPIC == "/ARM/CMD_VELOCITY"
    assert arm_constants.MANUAL_VELOCITY_TOPIC != arm_constants.VELOCITY_TOPIC
    # The internal handover command must not be operator-sendable.
    assert (arm_constants.END_EFFECTOR_STOP_COMMAND
            not in arm_constants.END_EFFECTOR_COMMANDS)


def test_node_defaults_live_in_the_arm_constants_module():
    """Node cadence and admission defaults have one source."""
    import arm_constants

    assert arm_constants.TELEOP_PUBLISH_RATE_HZ > 0
    assert arm_constants.SNIFFER_PERIOD_SEC > 0
    assert arm_constants.JOG_REJECT_INTERVAL_SEC > 0
    assert arm_constants.VELOCITY_NEUTRAL_EPSILON > 0
    assert arm_constants.WAYPOINT_TOL == arm_constants.HOMING_THRESHOLD
    assert arm_constants.WAYPOINT_STALL_LIMIT > 0
    assert (arm_constants.PATH_PLANAR_BASE
            <= arm_constants.PLANAR_BASE)


def test_spacemouse_axis_map_lives_in_the_arm_constants_module():
    """The teleop node does not redefine the device topic vocabulary."""
    import arm_constants

    assert arm_constants.SPACEMOUSE_AXIS_NAMES == (
        "x", "y", "z", "rx", "ry", "rz")
    assert arm_constants.SPACEMOUSE_AXIS_TOPICS["rz"] == (
        "/BASESTATION/spacemouse/rz")
    assert set(arm_constants.SPACEMOUSE_AXIS_TOPICS) == set(
        arm_constants.SPACEMOUSE_AXIS_NAMES)
    assert set(arm_constants.AXIS_SIGNS) == set(
        arm_constants.SPACEMOUSE_AXIS_NAMES)


def test_gripper_drive_flag_is_gone():
    """The SpaceMouse no longer drives the gripper, so the flag is gone."""
    assert "GRIPPER_DRIVE" not in ARM_STATE_TOPICS["flags"]


def test_controller_tuning_lives_in_the_arm_constants_module():
    """Controller tuning is part of the one arm constants module."""
    import arm_constants

    assert arm_constants.A0 == 6.32
    assert arm_constants.DT == 0.05
    assert arm_constants.HOMING_TARGET is HOMING_TARGET
    assert arm_constants.MAX_ANG_VEL > 0
    assert arm_constants.PLANAR_BASE == math.radians(5)

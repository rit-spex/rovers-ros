"""Tests for the absolute pose target path used by the typing node."""

import math

import pytest

from arm_constants import HOMING_TARGET
from arm_route_planner.kinematics.simulated_arm import RoverArmController


def reachable_pose():
    """Return a position and rotation that the controller can reach."""
    reference = RoverArmController(HOMING_TARGET)
    reference.sync_angles(
        [math.radians(-90), math.radians(60), math.radians(-60), 0.0, 0.0],
        reset_target=True,
    )
    return reference.target_pos, reference.target_frame.rotation_matrix()


def test_command_pose_reaches_a_known_configuration():
    """A full pose command updates the joint target through IK."""
    controller = RoverArmController(HOMING_TARGET)
    position, rotation = reachable_pose()
    assert controller.command_pose(
        position, rotation.tolist(), drive=False)
    expected = [
        math.radians(-90), math.radians(60), math.radians(-60), 0.0, 0.0]
    for actual, wanted in zip(controller.target_angles, expected):
        assert actual == pytest.approx(wanted, abs=1e-3)


def test_command_pose_rejects_a_malformed_position():
    """A bad position is refused instead of moving the arm."""
    controller = RoverArmController(HOMING_TARGET)
    _, rotation = reachable_pose()
    assert not controller.command_pose([1.0, 2.0], rotation.tolist())


def test_hold_syncs_the_target_to_the_measured_pose():
    """Holding drops the old target and tracks the current angles."""
    controller = RoverArmController(HOMING_TARGET)
    position, rotation = reachable_pose()
    controller.command_pose(position, rotation.tolist(), drive=False)
    assert controller.hold()
    assert controller.target_angles == controller.angles


def test_typing_mode_unlocks_and_restores_the_wrists():
    """Typing releases the wrist locks and restores them on exit."""
    controller = RoverArmController(HOMING_TARGET)
    assert controller.locked_joints[3]
    assert controller.locked_joints[4]
    assert controller.set_typing_mode(True)
    assert controller.limit_mode.name == "Typing"
    assert not controller.locked_joints[3]
    assert not controller.locked_joints[4]
    assert controller.set_typing_mode(False)
    assert controller.locked_joints[3]
    assert controller.locked_joints[4]

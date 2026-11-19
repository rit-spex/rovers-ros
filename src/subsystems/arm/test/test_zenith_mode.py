"""Tests for the zenith test pose and startup mode inference."""

import math

import pytest

from arm_constants import (
    ELBOW_DEADBAND,
    HOMING_TARGET,
    ZENITH_JOINT_TARGET,
)
from arm_route_planner.kinematics.arm_controller import RoverArmController


def run_frames(arm, frames):
    """Advance the controller for a fixed number of control cycles."""
    for _ in range(frames):
        arm.update(None)


def run_until_idle(arm, max_frames=2000):
    """Advance until a zenith exit or other action has finished."""
    for _ in range(max_frames):
        arm.update(None)
        if not arm.zenith and not arm.is_homing and not arm.is_flipping:
            return
    raise AssertionError("arm did not settle")


def test_zenith_drives_to_the_straight_up_pose():
    arm = RoverArmController.from_measured(HOMING_TARGET)
    base = arm.angles[0]

    assert arm.enter_zenith()
    assert arm.zenith
    run_frames(arm, 300)

    expected = [base, *ZENITH_JOINT_TARGET]
    assert arm.angles == pytest.approx(expected, abs=1e-2)
    # The elbow must reach zero, not stop at the singularity deadband.
    assert abs(arm.angles[2]) < 1e-2


def test_zenith_exit_returns_to_the_normal_side():
    arm = RoverArmController.from_measured(HOMING_TARGET)
    assert not arm.flipped

    arm.enter_zenith()
    run_frames(arm, 300)
    assert arm.exit_zenith()
    run_until_idle(arm)

    assert not arm.zenith
    assert not arm.flipped
    assert arm.angles[2] <= -ELBOW_DEADBAND + 1e-3
    # The default wrist locks return when zenith is left.
    assert arm.locked_joints[3]
    assert arm.locked_joints[4]


def test_zenith_exit_returns_to_the_flipped_side():
    flipped_pose = [0.0, math.radians(120), math.radians(45), 0.0, 0.0]
    arm = RoverArmController.from_measured(flipped_pose)
    assert arm.flipped

    arm.enter_zenith()
    run_frames(arm, 300)
    arm.exit_zenith()
    run_until_idle(arm)

    assert arm.flipped
    assert arm.angles[2] >= ELBOW_DEADBAND - 1e-3


def test_initial_straight_up_pose_infers_zenith():
    arm = RoverArmController.from_measured(
        [math.radians(30), math.pi / 2, 0.0, 0.0, 0.0])

    assert arm.zenith
    assert arm.limit_mode.name == "Zenith"


def test_initial_top_back_pose_infers_planar():
    arm = RoverArmController.from_measured(
        [0.0, 0.0, math.radians(135), 0.0, 0.0])

    assert arm.planar
    assert arm.flipped


def test_initial_pose_outside_both_sides_keeps_the_nearest_side():
    # The base is outside the planar band and neither side fits exactly;
    # the flipped side needs the smaller correction.
    arm = RoverArmController.from_measured(
        [math.radians(30), math.radians(160), math.radians(-30), 0.0, 0.0])

    assert not arm.planar
    assert arm.flipped


def test_mode_changes_are_blocked_while_zenith_is_active():
    arm = RoverArmController.from_measured(HOMING_TARGET)
    arm.enter_zenith()

    assert not arm.start_homing()
    assert not arm.set_pnp_normal()
    assert not arm.toggle_flipped()
    assert not arm.toggle_planar()
    assert arm.zenith

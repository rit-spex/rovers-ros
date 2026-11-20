"""Tests for zenith inference and mode gating on the reporting mirror."""

import math

from arm_constants import HOMING_TARGET
from arm_route_planner.kinematics.arm_controller import RoverArmController


def test_initial_straight_up_pose_infers_zenith():
    arm = RoverArmController.from_measured(
        [math.radians(30), math.pi / 2, 0.0, 0.0, 0.0])

    assert arm.zenith
    assert arm.limit_mode.name == "Zenith"


def test_zenith_inference_ignores_wrist_twist():
    arm = RoverArmController.from_measured(
        [math.radians(30), math.pi / 2, 0.0, 0.0, math.radians(35)])

    assert arm.zenith


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
    arm.zenith = True

    assert not arm.set_pnp_normal()
    assert not arm.toggle_flipped()
    assert not arm.toggle_planar()
    assert arm.zenith

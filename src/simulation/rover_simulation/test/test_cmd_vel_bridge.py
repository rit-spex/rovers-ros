"""Tests for the simulated drive motor deadzone."""

import pytest
from geometry_msgs.msg import Twist

from rover_simulation.drivebase_sim_node import apply_motor_deadzone

SEPARATION = 0.58
DEADZONE = 0.02


def make_twist(linear: float = 0.0, angular: float = 0.0) -> Twist:
    twist = Twist()
    twist.linear.x = linear
    twist.angular.z = angular
    return twist


def test_zero_command_stays_zero():
    result = apply_motor_deadzone(
        make_twist(), SEPARATION, DEADZONE)
    assert result.linear.x == 0.0
    assert result.angular.z == 0.0


def test_small_linear_command_is_stopped():
    result = apply_motor_deadzone(
        make_twist(linear=DEADZONE / 2.0), SEPARATION, DEADZONE)
    assert result.linear.x == 0.0
    assert result.angular.z == 0.0


def test_linear_command_above_deadzone_passes():
    result = apply_motor_deadzone(
        make_twist(linear=4 * DEADZONE), SEPARATION, DEADZONE)
    assert result.linear.x == pytest.approx(4 * DEADZONE)
    assert result.angular.z == pytest.approx(0.0)


def test_small_rotation_is_stopped():
    # Pure rotation: the wheel speeds are +/- angular * separation / 2.
    angular = DEADZONE / SEPARATION
    result = apply_motor_deadzone(
        make_twist(angular=angular), SEPARATION, DEADZONE)
    assert result.linear.x == 0.0
    assert result.angular.z == 0.0


def test_one_wheel_below_deadzone_only_clamps_that_wheel():
    # Left wheel: 0.02 - 0.05 * 0.29 = 0.0055 (deadzone),
    # right wheel: 0.02 + 0.0145 = 0.0345 (moving).
    result = apply_motor_deadzone(
        make_twist(linear=0.02, angular=0.05), SEPARATION, DEADZONE)
    assert result.linear.x == pytest.approx(0.0345 / 2.0)
    assert result.angular.z == pytest.approx(0.0345 / SEPARATION)


def test_disabled_deadzone_leaves_the_command_alone():
    twist = make_twist(linear=0.001, angular=0.001)
    assert apply_motor_deadzone(twist, SEPARATION, 0.0) is twist

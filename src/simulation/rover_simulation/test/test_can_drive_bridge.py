"""Tests for the Drive_Power to diff-drive twist conversion."""

import pytest
from geometry_msgs.msg import Twist

from rover_simulation.drivebase_sim_node import drive_power_to_twist

SEPARATION = 0.58
RADIUS = 0.081674
MAX_WHEEL_ANGULAR_VELOCITY = 10.0
FULL_SPEED = MAX_WHEEL_ANGULAR_VELOCITY * RADIUS


def convert(left: float, right: float) -> Twist:
    return drive_power_to_twist(
        left, right, SEPARATION, RADIUS, MAX_WHEEL_ANGULAR_VELOCITY)


def test_neutral_power_is_stopped():
    result = convert(0.0, 0.0)
    assert result.linear.x == 0.0
    assert result.angular.z == 0.0


def test_full_power_drives_straight_at_max_speed():
    result = convert(1.0, 1.0)
    assert result.linear.x == pytest.approx(FULL_SPEED)
    assert result.angular.z == pytest.approx(0.0)


def test_reverse_power_drives_backwards():
    result = convert(-1.0, -1.0)
    assert result.linear.x == pytest.approx(-FULL_SPEED)


def test_opposite_power_spins_in_place():
    result = convert(-1.0, 1.0)
    assert result.linear.x == pytest.approx(0.0)
    assert result.angular.z == pytest.approx(2.0 * FULL_SPEED / SEPARATION)


def test_power_is_clamped_to_full_scale():
    result = convert(4.0, 4.0)
    assert result.linear.x == pytest.approx(FULL_SPEED)


def test_partial_power_scales_linearly():
    result = convert(0.25, 0.25)
    assert result.linear.x == pytest.approx(0.25 * FULL_SPEED)


def test_non_finite_power_is_treated_as_zero():
    result = convert(float("nan"), 1.0)
    assert result.linear.x == pytest.approx(FULL_SPEED / 2.0)


def test_invalid_separation_stops_the_rover():
    result = drive_power_to_twist(1.0, 1.0, 0.0, RADIUS, 10.0)
    assert result.linear.x == 0.0
    assert result.angular.z == 0.0

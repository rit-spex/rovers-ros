"""Tests for the manual teleop velocity gate in the arm master mux."""

from arm_master.arm_master_node import ManualVelocityGate, velocity_is_neutral
from arm_constants import PLANNER_ALWAYS_COMMANDS

NEUTRAL = (0.0, 0.0, 0.0, 0.0, 0.0, 0.0)
FORWARD = (0.2, 0.0, 0.0, 0.0, 0.0, 0.0)


def test_always_forward_set_covers_only_mode_toggles():
    # The resting/zenith toggles bypass the MANUAL-only gate so an
    # autonomous source cannot swallow them; motion commands do not.
    assert "toggle_resting" in PLANNER_ALWAYS_COMMANDS
    assert "rest" in PLANNER_ALWAYS_COMMANDS
    assert "zenith" in PLANNER_ALWAYS_COMMANDS
    assert "toggle_zenith" in PLANNER_ALWAYS_COMMANDS
    for motion in ("flip", "home", "pnp_normal", "toggle_planar",
                   "joint_drive", "position_drive"):
        assert motion not in PLANNER_ALWAYS_COMMANDS


def test_neutral_detection():
    assert velocity_is_neutral(NEUTRAL)
    assert velocity_is_neutral((1e-9, -1e-9, 0.0, 0.0, 0.0, 0.0))
    assert not velocity_is_neutral(FORWARD)
    assert not velocity_is_neutral((0.0, 0.0, 0.0, 0.0, 0.0, -0.001))


def test_gate_requires_neutral_before_arming():
    gate = ManualVelocityGate()
    assert not gate.armed
    assert not gate.admit(FORWARD)
    assert not gate.armed
    assert gate.admit(NEUTRAL)
    assert gate.armed
    # Once armed, motion passes without another neutral sample.
    assert gate.admit(FORWARD)


def test_reset_forces_neutral_again():
    gate = ManualVelocityGate()
    assert gate.admit(NEUTRAL)
    assert gate.admit(FORWARD)
    gate.reset()
    assert not gate.armed
    assert not gate.admit(FORWARD)
    assert gate.admit(NEUTRAL)
    assert gate.armed


def test_epsilon_is_configurable():
    gate = ManualVelocityGate(epsilon=0.5)
    # A small deflection is treated as neutral when the epsilon is large.
    assert gate.admit((0.4, 0.0, 0.0, 0.0, 0.0, 0.0))
    assert gate.armed

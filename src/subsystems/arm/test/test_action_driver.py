"""Tests for the action-driver stepping profile and observed regions."""

import math

import pytest

from arm_action_driver.actions import ACTION_CONFIG_CLASSES
from arm_action_driver.actions.base import ActionConfiguration
from arm_action_driver.stepping import JointRamp, LimitedRamp, remaining_delta
from arm_constants import (
    ACTION_ACCEL_RAD_S2,
    ACTION_ACTIVE_IDLE,
    ACTION_COMMANDS,
    ACTION_DECEL_RAD_S2,
    DT,
)
from arm_route_planner.kinematics.regions import (
    RegionTracker,
    is_resting_pose,
    is_zenith_pose,
)


def make_ramp():
    return JointRamp(1, ACTION_ACCEL_RAD_S2, ACTION_DECEL_RAD_S2, DT)


def run_to(ramp, start, goal, cap, cycles=4000):
    """Advance a one-joint ramp and return (position, history)."""
    position = float(start)
    history = [position]
    for _ in range(cycles):
        steps, done = ramp.step([goal - position], [cap])
        position += steps[0]
        history.append(position)
        if done:
            return position, history
    raise AssertionError("ramp did not reach its goal")


def test_ramp_reaches_goal_without_overshoot():
    ramp = make_ramp()
    goal, history = run_to(ramp, 0.0, 1.0, 1.0)

    assert goal == pytest.approx(1.0, abs=1e-3)
    assert max(history) <= 1.0 + 1e-9
    assert all(b >= a - 1e-12 for a, b in zip(history, history[1:]))


def test_ramp_respects_the_cruise_cap():
    ramp = make_ramp()
    cap = 0.2
    goal, history = run_to(ramp, 0.0, 2.0, cap)

    assert goal == pytest.approx(2.0, abs=1e-3)
    max_step = max(history[i + 1] - history[i]
                   for i in range(len(history) - 1))
    assert max_step <= cap * DT + 1e-9


def test_ramp_accelerates_then_decelerates():
    ramp = make_ramp()
    _, history = run_to(ramp, 0.0, 2.0, 0.5)
    velocities = [history[i + 1] - history[i]
                  for i in range(len(history) - 1)]

    peak = velocities.index(max(velocities))
    assert peak > 2 and peak < len(velocities) - 1
    assert velocities[0] < velocities[peak]
    assert velocities[-1] < velocities[peak]


def test_pause_decelerates_and_resume_finishes():
    ramp = make_ramp()
    position = 0.0
    goal = 2.0
    for _ in range(20):
        steps, _ = ramp.step([goal - position], [0.5])
        position += steps[0]
    assert not ramp.is_stopped()

    for _ in range(400):
        steps, _ = ramp.step([goal - position], [0.0])
        position += steps[0]
        if ramp.is_stopped():
            break
    assert ramp.is_stopped()
    paused_at = position

    final, _ = run_to(ramp, position, goal, 0.5)
    assert final == pytest.approx(goal, abs=1e-3)
    assert paused_at < final


def test_small_goal_is_reached_in_one_step():
    ramp = make_ramp()
    position, _ = run_to(ramp, 0.0, 1e-5, 1.0)
    assert position == pytest.approx(1e-5, abs=1e-9)


def test_limited_ramp_clamps_to_joint_limits():
    ramp = JointRamp(2, ACTION_ACCEL_RAD_S2, ACTION_DECEL_RAD_S2, DT)
    limited = LimitedRamp(ramp, [(0.0, 0.1), (-1.0, 1.0)])

    targets, _ = limited.step([0.05, 0.0], [1.0, 1.0], [1.0, 1.0])
    for _ in range(50):
        targets, _ = limited.step(targets, [1.0, 1.0], [1.0, 1.0])
    assert targets[0] == pytest.approx(0.1)
    assert targets[1] <= 1.0


def test_remaining_delta_wraps_the_short_way():
    assert remaining_delta(math.radians(170), math.radians(-170)) \
        == pytest.approx(math.radians(20), abs=1e-9)
    assert abs(remaining_delta(0.0, math.pi)) == pytest.approx(math.pi)


def test_zenith_pose_ignores_base_and_twist():
    pose = [math.radians(35), math.pi / 2, 0.0, 0.0, math.radians(40)]
    assert is_zenith_pose(pose)
    assert not is_zenith_pose([0.0, math.radians(45), 0.0, 0.0, 0.0])


def test_resting_pose_detects_the_collapsed_region():
    assert is_resting_pose([0.0, math.radians(-9.5), 0.0, 0.0, 0.0])
    assert is_resting_pose([0.0, 0.0, math.radians(170), 0.0, 0.0])
    assert not is_resting_pose([0.0, math.radians(45), 0.0, 0.0, 0.0])


def test_region_tracker_tracks_the_observed_side():
    tracker = RegionTracker()
    tracker.update([0.0, math.radians(120), math.radians(45), 0.0, 0.0])
    assert tracker.flipped

    tracker.update([0.0, math.radians(45), math.radians(-45), 0.0, 0.0])
    assert not tracker.flipped


def test_registry_declares_unique_active_values():
    values = [cls.active_value for cls in ACTION_CONFIG_CLASSES]
    assert len(values) == len(set(values))
    assert ACTION_ACTIVE_IDLE not in values


def test_registry_covers_the_action_command_table():
    covered = set()
    for cls in ACTION_CONFIG_CLASSES:
        assert cls.trigger_commands, \
            f"{cls.__name__} declares no trigger commands"
        covered.update(cls.trigger_commands)
        covered.update(cls.release_commands)
    assert ACTION_COMMANDS <= covered


def test_homing_matches_the_single_joint_prefix():
    homing = next(cls for cls in ACTION_CONFIG_CLASSES
                  if "homing" in cls.trigger_commands)
    config = homing()
    assert config.matches("home_joint:2")
    assert config.matches("homing")
    assert config.matches("home")
    assert not config.matches("flip")


def test_hold_to_run_actions_stop_on_release():
    configs = {cls.__name__: cls() for cls in ACTION_CONFIG_CLASSES}
    homing = configs["HomingConfiguration"]
    assert "home_release" in homing.release_commands
    assert homing.stop_on_release

    zenith = configs["ZenithConfiguration"]
    assert "zenith_release" in zenith.release_commands
    assert zenith.stop_on_release

    flip = configs["FlipConfiguration"]
    assert "flip_release" in flip.release_commands
    assert flip.pause_on_release


def test_resting_has_no_release_command():
    configs = {cls.__name__: cls() for cls in ACTION_CONFIG_CLASSES}
    resting = configs["RestingConfiguration"]
    assert not resting.release_commands
    assert not resting.stop_on_release
    assert not resting.pause_on_release


class _HomingStub:
    """Driver stub for the homing target helper."""

    def __init__(self, flipped, in_zenith):
        self.flipped = flipped
        self._in_zenith = in_zenith

    def in_zenith(self):
        return self._in_zenith


def _homing_config():
    return next(cls for cls in ACTION_CONFIG_CLASSES
                if "homing" in cls.trigger_commands)()


def test_homing_target_is_the_standard_home_when_not_flipped():
    target = _homing_config().target(_HomingStub(False, False))
    assert target[1] == pytest.approx(math.pi / 4)
    assert target[2] == pytest.approx(-math.pi / 4)


def test_homing_target_mirrors_on_the_flipped_side():
    target = _homing_config().target(_HomingStub(True, False))
    assert target[1] == pytest.approx(math.pi - math.pi / 4)
    assert target[2] == pytest.approx(math.pi / 4)


def test_homing_target_from_zenith_is_always_the_non_flipped_home():
    # Homing preempting a zenith hold returns to the standard home even
    # when the zenith was entered from the flipped side.
    target = _homing_config().target(_HomingStub(True, True))
    assert target[1] == pytest.approx(math.pi / 4)
    assert target[2] == pytest.approx(-math.pi / 4)


def test_every_config_declares_an_engine_and_a_target():
    for cls in ACTION_CONFIG_CLASSES:
        config = cls()
        assert config.engine in ("move", "resting")
        if config.engine == "move":
            assert config.target_angles is not None \
                or cls.target is not ActionConfiguration.target
        else:
            assert config.controller_class is not None


def test_target_helpers_are_pure_parameter_consumers():
    # The per-action classes are parameter blocks: they must not grow
    # lifecycle methods again.
    lifecycle = ("trigger", "repeat_trigger", "release", "update",
                 "abort", "adopt", "on_manual_motion")
    for cls in ACTION_CONFIG_CLASSES:
        for name in lifecycle:
            assert not hasattr(cls, name), \
                f"{cls.__name__} re-implements {name}; behaviour belongs " \
                "in the driver node"

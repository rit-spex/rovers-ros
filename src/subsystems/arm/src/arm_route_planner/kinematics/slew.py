"""Joint slew model used by the arm controller.

The controller never writes joint angles directly: it commands a target and
this class moves every joint toward it at a bounded speed.  It is used on
hardware as well as in simulation.
"""

import math

from arm_constants import (
    DT, MAX_ANG_VEL, ANGULAR_SPEED_RAMP_TIME, ARM_MOTOR_MAX_ANG_VEL,
)
from .linkage_models import wrap2pi, limit_respecting_diff


class SimulatedMechanicalArm:
    """Source-of-truth joint state for the simulated arm.

    The controller never writes joint angles directly.  Instead it
    commands a target each cycle and this class moves every joint toward
    it by at most ``max_ang_vel * dt``.  Joint speed ramps up from zero
    over ``ramp_time`` seconds.  Both the commanded target and the
    resulting angles are clamped to the caller-supplied joint limits, so
    an invalid target can never produce an out-of-range pose.

    Joints travel along the shortest angular path.  If that path would
    leave the joint's limits, the opposite (limit-respecting) direction
    is used, matching the homing controller's behaviour.
    """

    def __init__(self, angles, max_ang_vel=None,
                 ramp_time=ANGULAR_SPEED_RAMP_TIME):
        """Create the mechanical arm from an initial angle list.

        Args:
            angles: Initial joint angles [rad].
            max_ang_vel: Maximum joint speed [rad/s], either one scalar for
                every joint or one value per joint.  Defaults to the
                per-motor speeds derived from ``ARM_MOTOR_PARAMS``.
            ramp_time: Time to reach full speed from standstill [s].
        """
        self.angles = [float(a) for a in angles]
        self.max_ang_vel = self._velocity_limits(max_ang_vel, len(angles))
        self.ramp_time = max(0.0, float(ramp_time))
        self.joint_speeds = [0.0] * len(self.angles)

    @staticmethod
    def _velocity_limits(max_ang_vel, count):
        """Normalise a scalar or per-joint speed limit to one per joint."""
        if max_ang_vel is None:
            limits = [float(v) for v in ARM_MOTOR_MAX_ANG_VEL[:count]]
            if len(limits) < count:
                limits.extend([MAX_ANG_VEL] * (count - len(limits)))
            return limits
        if isinstance(max_ang_vel, (int, float)):
            return [float(max_ang_vel)] * count
        limits = [float(v) for v in max_ang_vel]
        if len(limits) < count:
            limits.extend([MAX_ANG_VEL] * (count - len(limits)))
        return limits[:count]

    def step(self, target_angles, joint_limits, dt=DT, max_step=None):
        """Advance one control cycle toward ``target_angles``.

        Args:
            target_angles: Desired joint angles [rad].
            joint_limits: Per-joint (min, max) pairs, or None.
            dt: Control period [s].
            max_step: Optional per-joint or scalar maximum angular step
                for this cycle [rad].  Defaults to the per-joint speed
                limit times *dt*.

        Returns:
            list[float]: The new (clamped, velocity-limited) angles.

        Raises:
            ValueError: If the target has a different number of joints.
        """
        if len(target_angles) != len(self.angles):
            raise ValueError(
                f"Expected {len(self.angles)} target angles, "
                f"got {len(target_angles)}")

        default_steps = [limit * dt for limit in self.max_ang_vel]
        if max_step is None:
            step_limits = default_steps
        elif isinstance(max_step, (int, float)):
            step_limits = [float(max_step)] * len(self.angles)
        else:
            step_limits = [float(v) for v in max_step]

        result = list(self.angles)

        for i, target in enumerate(target_angles):
            lo, hi = self._limits_for(joint_limits, i)
            target = max(lo, min(hi, float(target)))

            diff = wrap2pi(target - result[i])
            if abs(diff) <= 1e-12:
                result[i] = target
                self.joint_speeds[i] = 0.0
                continue

            # Commit to the direction whose whole path stays inside the
            # limits.  Re-picking the shortest direction every cycle makes
            # a joint that cannot take the short way oscillate forever on
            # the bound (the original flip-jitter bug).
            diff = limit_respecting_diff(result[i], diff, lo, hi)
            step_dir = 1.0 if diff > 0 else -1.0
            limit_step = (step_limits[i] if i < len(step_limits)
                          else default_steps[i])
            limit_speed = limit_step / dt if dt > 1e-12 else 0.0
            max_speed = self.max_ang_vel[i]
            target_speed = min(max_speed, limit_speed)
            if self.ramp_time <= 1e-9:
                speed = target_speed
            else:
                accel = max_speed / self.ramp_time
                speed = min(target_speed,
                            self.joint_speeds[i] + accel * dt)
            self.joint_speeds[i] = speed
            step_mag = min(speed * dt, limit_step, abs(diff))
            next_angle = result[i] + step_dir * step_mag
            if math.isinf(lo) and math.isinf(hi):
                result[i] = wrap2pi(next_angle)
            else:
                result[i] = max(lo, min(hi, next_angle))

        self.angles = result
        return result

    @staticmethod
    def _limits_for(joint_limits, index):
        """Return (lo, hi) for one joint, defaulting to unbounded."""
        if joint_limits is None or index >= len(joint_limits):
            return -float("inf"), float("inf")
        pair = joint_limits[index]
        if pair is None:
            return -float("inf"), float("inf")
        lo, hi = pair
        return float(lo), float(hi)

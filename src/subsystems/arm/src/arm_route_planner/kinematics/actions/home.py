"""Homing action controller."""

from ..linkage_models import wrap2pi, limit_respecting_diff


class HomingController:
    """Drive joint angles toward a fixed home configuration.

    Standalone action: it owns its own lifecycle and does not share a
    base class with any other controller.
    """

    name = "Homing"

    def __init__(self, target, speed, threshold):
        self.target = list(target)
        self.speed = speed
        self.threshold = threshold
        self.active = False

    @property
    def is_active(self):
        """True while homing is running."""
        return self.active

    def activate(self):
        """Begin homing.  Returns False if it is already running."""
        if self.active:
            return False
        self.active = True
        return True

    def deactivate(self):
        """Mark homing as finished."""
        self.active = False

    def step(self, angles, limits=None, mask=None):
        """Step each allowed joint toward *target*. Returns (new_angles, done).

        Steps along the shortest angular path.  When *limits* is given,
        a candidate that would leave the joint's bounds falls back to
        the opposite direction (the limit-respecting path) instead.
        Joints whose *mask* entry is False are held in place and ignored
        by the completion check.  Returns (angles, False) unchanged if
        the action is not running.
        """
        if not self.active:
            return list(angles), False

        def allowed(i):
            return mask is None or i >= len(mask) or mask[i]

        new_angles = list(angles)
        for i, tgt in enumerate(self.target):
            if not allowed(i):
                continue

            diff = wrap2pi(tgt - new_angles[i])
            if abs(diff) <= self.threshold:
                continue

            if limits is not None and i < len(limits):
                lo, hi = limits[i]
                # Pick the direction whose whole path stays inside the
                # limits; flipping direction on every step would chatter
                # on the bound when the shortest path is blocked.
                diff = limit_respecting_diff(
                    new_angles[i], diff, lo, hi)
                step_dir = 1.0 if diff > 0 else -1.0
                new_angles[i] = max(
                    lo, min(hi, new_angles[i] + step_dir * self.speed))
            else:
                step_dir = 1.0 if diff > 0 else -1.0
                new_angles[i] = wrap2pi(
                    new_angles[i] + step_dir * self.speed)

        done = all(
            not allowed(i)
            or abs(wrap2pi(tgt - new_angles[i])) <= self.threshold
            for i, tgt in enumerate(self.target)
        )
        if done:
            self.deactivate()
        return new_angles, done

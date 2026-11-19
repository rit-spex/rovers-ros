"""Flip actions and reflection helpers."""

import math

import numpy as np

from ..linkage_models import wrap2pi


class FlippingManager:
    """Reflection helpers used by the flip actions."""

    @staticmethod
    def side_flipped_angles(angles):
        """Mirror shoulder about 90 deg and elbow about 0 deg."""
        return [
            angles[0],
            math.pi - angles[1],
            -angles[2],
            angles[3],
            angles[4],
        ]

    @staticmethod
    def flipped_angles(th1, th2, l0, l1, l2):
        """Mirror the elbow while keeping the wrist center fixed."""
        v_arm = np.array(l1, dtype=float) - np.array(l0, dtype=float)
        v_axis = np.array(l2, dtype=float) - np.array(l0, dtype=float)

        norm_arm = np.linalg.norm(v_arm)
        norm_axis = np.linalg.norm(v_axis)
        if norm_arm < 1e-9 or norm_axis < 1e-9:
            return th1, -th2

        unit_arm = v_arm / norm_arm
        unit_axis = v_axis / norm_axis

        dot_product = np.dot(unit_arm, unit_axis)
        alpha = np.arccos(np.clip(dot_product, -1.0, 1.0))

        shift = 2 * alpha
        if th2 < 0:
            shift = -shift

        return wrap2pi(th1 + shift), -th2


# Compatibility aliases for callers and tests.
_get_side_flipped_angles = FlippingManager.side_flipped_angles
_get_flipped_angles = FlippingManager.flipped_angles


class FlipController:
    """Single flip action for both geometric and side flips.

    By default it uses the geometric flip that mirrors the elbow while
    keeping the wrist center position fixed.  With ``side=True`` it uses
    the Pick-and-Place side flip (shoulder about 90°, elbow about 0°).

    All target angles are clamped to the joint limits so the flip never
    violates mechanical constraints.  Standalone action: it owns its own
    lifecycle and does not share a base class with any other controller.
    """

    name = "Flip"

    def __init__(self, duration_frames=30):
        self.duration = duration_frames
        self.frame = 0
        self.start_angles = None
        self.target_angles = None
        self.active = False

    @property
    def is_active(self):
        """True while the flip is running."""
        return self.active

    def deactivate(self):
        """Mark the flip as finished."""
        self.active = False

    def activate(self, current_angles, l0, l1, l2, joint_limits=None,
                 side=False):
        """Start a flip from the given angles and joint positions.

        Args:
            current_angles: list of 5 joint angles [rad].
            l0: shoulder position [x, y, z].
            l1: elbow position [x, y, z].
            l2: wrist position [x, y, z].
            joint_limits: optional list of (min, max) per joint for clamping.
            side: use the side-flip target (shoulder about 90 deg,
                elbow about 0 deg) instead of the geometric elbow mirror.

        Returns:
            bool: True if the flip started, False if one was already running.
        """
        if self.active:
            return False
        self.active = True

        self.start_angles = list(current_angles)

        if side:
            self.target_angles = _get_side_flipped_angles(current_angles)
        else:
            th1_f, th2_f = _get_flipped_angles(
                current_angles[1], current_angles[2], l0, l1, l2)
            th3_f = -current_angles[3]
            self.target_angles = [current_angles[0], th1_f, th2_f, th3_f,
                                  current_angles[4]]

        if joint_limits is not None:
            for i, pair in enumerate(joint_limits):
                if i >= len(self.target_angles):
                    break
                lo, hi = pair
                self.target_angles[i] = max(lo, min(hi, self.target_angles[i]))

        self.frame = 0
        return True

    def step(self, joint_limits=None):
        """Advance one frame. Returns interpolated angles, or None if done.

        Args:
            joint_limits: optional list of (min, max) per joint for clamping.
        """
        if not self.active:
            return None

        self.frame += 1
        t = min(self.frame / self.duration, 1.0)
        t = t * t * (3 - 2 * t)  # smoothstep

        angles = []
        for i in range(5):
            val = (self.start_angles[i]
                   + t * (self.target_angles[i] - self.start_angles[i]))
            if joint_limits is not None and i < len(joint_limits):
                lo, hi = joint_limits[i]
                val = max(lo, min(hi, val))
            angles.append(val)

        if self.frame >= self.duration:
            self.deactivate()
            return list(self.target_angles)

        return angles


class FlipSideController:
    """Flips the arm between Pick-and-Place and Pick-and-Place-Flipped.

    Shoulder is mirrored about 90° (π − θ) and elbow is mirrored about
    0° (−θ).  Base, wrist bend, and wrist twist stay fixed.

    This transitions the arm between the two pick-and-place workspaces:
    shoulder 0–90° ↔ 90–180°, elbow −135°–0° ↔ 0–135°.

    Standalone action: it owns its own lifecycle and does not share a
    base class with any other controller.
    """

    name = "Flip Side"

    def __init__(self, duration_frames=30):
        self.duration = duration_frames
        self.frame = 0
        self.start_angles = None
        self.target_angles = None
        self.active = False

    @property
    def is_active(self):
        """True while the side flip is running."""
        return self.active

    def deactivate(self):
        """Mark the side flip as finished."""
        self.active = False

    @staticmethod
    def compute_flipped(angles):
        """Return the side-flipped angle list.

        Shoulder mirrored about 90°: π − θ
        Elbow mirrored about 0°:     −θ
        All other joints unchanged.
        """
        return _get_side_flipped_angles(angles)

    def activate(self, current_angles, joint_limits=None):
        """Start a side flip from the given angles.

        Args:
            current_angles: list of 5 joint angles [rad].
            joint_limits: optional list of (min, max) per joint for clamping.

        Returns:
            bool: True if the flip started, False if one was already running.
        """
        if self.active:
            return False
        self.active = True

        self.start_angles = list(current_angles)
        self.target_angles = self.compute_flipped(current_angles)

        if joint_limits is not None:
            for i, pair in enumerate(joint_limits):
                if i >= len(self.target_angles):
                    break
                lo, hi = pair
                self.target_angles[i] = max(lo, min(hi, self.target_angles[i]))

        self.frame = 0
        return True

    def step(self, joint_limits=None):
        """Advance one frame. Returns interpolated angles, or None if done.

        Args:
            joint_limits: optional list of (min, max) per joint for clamping.
        """
        if not self.active:
            return None

        self.frame += 1
        t = min(self.frame / self.duration, 1.0)
        t = t * t * (3 - 2 * t)  # smoothstep

        angles = []
        for i in range(5):
            val = (self.start_angles[i]
                   + t * (self.target_angles[i] - self.start_angles[i]))
            if joint_limits is not None and i < len(joint_limits):
                lo, hi = joint_limits[i]
                val = max(lo, min(hi, val))
            angles.append(val)

        if self.frame >= self.duration:
            self.deactivate()
            return list(self.target_angles)

        return angles

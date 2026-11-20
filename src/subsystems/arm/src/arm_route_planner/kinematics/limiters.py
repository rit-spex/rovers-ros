import math

import numpy as np

from arm_constants import (
    MECHANICAL_LIMITS, FULL_RANGE_LIMITS, PLANAR_LIMITS,
    PICK_AND_PLACE_LIMITS, PICK_AND_PLACE_FLIPPED_LIMITS, HOMING_LIMITS,
    TYPING_LIMITS, RESTING_LIMITS,
    ELBOW_DEADBAND, O3_RELAX_MAX, EXCLUSION_RADIUS,
    BASE_ZERO_RANGE, HOMING_TARGET, END_EFFECTOR_MIN_Z,
    FRONT_HIGH, RIGHT_HIGH, BACK_HIGH, LEFT_HIGH,
    FRONT_LOW, RIGHT_LOW, BACK_LOW, LEFT_LOW,
    OCTANT_NAMES, OCTANT_RELIEF, OCTANT_CURL,
    OCTANT_PLANAR_EXIT_LOCK, OCTANT_DIRECT_FLIP,
    OCTANT_PLANAR_ALLOWED, OCTANT_PLANAR_FLIPPED_ALLOWED,
    OCTANT_NORMAL_ALLOWED, OCTANT_POSITION_LIMITS, OCTANT_RULES,
)
from .linkage_models import OrientationMatrix


# =============================================================================
# Shared geometry helpers used by limiters
# =============================================================================

class LimiterManager:
    """Shared joint-constraint and octant helpers."""

    @staticmethod
    def arm_constraints(angles):
        """Enforce deadband around t2=0 to prevent elbow singularity."""
        result = list(angles)
        if len(result) > 2:
            if 0.0 < result[2] < ELBOW_DEADBAND:
                result[2] = ELBOW_DEADBAND
            elif -ELBOW_DEADBAND < result[2] <= 0.0:
                result[2] = -ELBOW_DEADBAND
        return result


    @staticmethod
    def _get_octant(pos):
        """Compute the octant number (1-8) from a world position.

        Front is +Y, right is +X.  z >= 0 is high (1-4), z < 0 is low
        (5-8).  The z = 0 boundary belongs to the high octants, matching
        OCTANT_POSITION_LIMITS, which include z = 0 in the high boxes.
        """
        px, py, pz = pos[0], pos[1], pos[2]
        high = pz >= 0

        if py >= px and py >= -px:          # front (+Y)
            return FRONT_HIGH if high else FRONT_LOW
        if px >= py and px >= -py:          # right (+X)
            return RIGHT_HIGH if high else RIGHT_LOW
        if py <= px and py <= -px:          # back (-Y)
            return BACK_HIGH if high else BACK_LOW
        return LEFT_HIGH if high else LEFT_LOW  # left (-X)

# Compatibility aliases for callers and tests.
arm_constraints = LimiterManager.arm_constraints
_get_octant = LimiterManager._get_octant


# =============================================================================
# Limiter hierarchy
# =============================================================================
class BaseLimiter:
    """Defines the structure of a drive mode's limits.

    Subclasses provide the concrete per-joint bounds and override the
    hooks below to add mode-specific behaviour:

        can_set(angles)              — may this mode be activated?
        get_limits(angles, ee_pos)   — effective limits for this state
        clamp_target_position(...)   — workspace bounds on the target
        clamp_angles(angles, ee)     — convenience: clamp a full angle list
        geometry_constraint(angles)  — post-IK safety cleanup
    """

    name = "Base"
    limits: list = []

    def can_set(self, angles):
        """Return True if this mode may be activated from the given angles."""
        return True

    def get_limits(self, angles, end_effector_pos=None):
        """Effective per-joint (min, max) limits for the current state."""
        return [tuple(pair) for pair in self.limits]

    def clamp_angles(self, angles, end_effector_pos=None):
        """Clamp a full angle list to this mode's effective limits."""
        limits = self.get_limits(angles, end_effector_pos)
        return [max(lo, min(hi, a)) for a, (lo, hi) in zip(angles, limits)]

    def check_cylindrical(self, end_effector_pos):
        """Return True if the end effector is inside the exclusion zone."""
        return False

    def clamp_target_position(self, target_frame, angles=None):
        """Clamp a desired target frame to the global height floor."""
        pos = list(target_frame.position())
        if pos[2] >= END_EFFECTOR_MIN_Z:
            return target_frame
        pos[2] = END_EFFECTOR_MIN_Z
        R = np.array(target_frame.tolist(), dtype=float)[:3, :3]
        return OrientationMatrix([
            [R[0][0], R[0][1], R[0][2], pos[0]],
            [R[1][0], R[1][1], R[1][2], pos[1]],
            [R[2][0], R[2][1], R[2][2], pos[2]],
            [0.0, 0.0, 0.0, 1.0],
        ])

    def geometry_constraint(self, angles):
        """Post-IK angle cleanup (default: elbow singularity deadband)."""
        return arm_constraints(angles)


class FullRangeMode(BaseLimiter):
    """Full-range pick-and-place mode.

    Mechanical joint limits plus dynamic behaviours:
        - Shoulder expansion: base near zero opens the shoulder to 180 deg.
        - Octant relief: in OCTANT_RELIEF (Back High, Front Low, Back
          Low), a current upper-limit overshoot of at most O3_RELAX_MAX
          is excused so the arm can reach into the octant.
        - Octant curl: in OCTANT_CURL (Right Low, Left Low) the elbow
          upper limit is forced to 0 so the arm curls instead of
          extending downward.
        - Cylindrical avoidance: a keep-out cylinder around the base.
        - Octant position limits: Cartesian bounds on the target in all
          eight octants, from OCTANT_POSITION_LIMITS.
    """

    name = "Full Range"
    limits = FULL_RANGE_LIMITS

    octant_relief_octants = OCTANT_RELIEF
    octant_curl_octants = OCTANT_CURL
    position_limits = OCTANT_POSITION_LIMITS

    @staticmethod
    def _base_near_zero(angles):
        bmin, bmax = BASE_ZERO_RANGE
        return bmin <= angles[0] <= bmax

    def get_limits(self, angles, end_effector_pos=None):
        limits = [list(pair) for pair in self.limits]

        if self._base_near_zero(angles):
            limits[1] = (limits[1][0], math.pi)

        if end_effector_pos is not None:
            octant = _get_octant(end_effector_pos)

            if octant in self.octant_relief_octants:
                for i in range(min(len(angles), len(limits))):
                    overshoot = angles[i] - limits[i][1]
                    if 0 <= overshoot <= O3_RELAX_MAX:
                        limits[i] = (limits[i][0], angles[i])

            elif octant in self.octant_curl_octants:
                limits[2] = (limits[2][0], 0.0)

        return limits

    def check_cylindrical(self, end_effector_pos):
        if end_effector_pos is None:
            return False
        ex, ey = end_effector_pos[0], end_effector_pos[1]
        return math.sqrt(ex * ex + ey * ey) < EXCLUSION_RADIUS

    def clamp_target_position(self, target_frame, angles=None):
        pos = list(target_frame.position())

        if angles is not None and not self._base_near_zero(angles):
            if self.check_cylindrical(pos):
                ex, ey = pos[0], pos[1]
                radial = math.sqrt(ex * ex + ey * ey)
                if radial < 1e-9:
                    pos[0] = EXCLUSION_RADIUS
                else:
                    scale = EXCLUSION_RADIUS / radial
                    pos[0] *= scale
                    pos[1] *= scale

        if self.position_limits:
            for _ in range(4):
                before = list(pos)
                octant = _get_octant(pos)
                ranges = self.position_limits.get(octant)
                if ranges is None:
                    break
                for axis, axis_range in enumerate(ranges):
                    if axis_range is not None:
                        lo, hi = axis_range
                        pos[axis] = max(lo, min(hi, pos[axis]))
                if pos == before:
                    break

        if list(target_frame.position()) == pos:
            return target_frame

        R = np.array(target_frame.tolist(), dtype=float)[:3, :3]
        new_vals = [
            R[0][0], R[0][1], R[0][2], pos[0],
            R[1][0], R[1][1], R[1][2], pos[1],
            R[2][0], R[2][1], R[2][2], pos[2],
            0, 0, 0, 1,
        ]
        return OrientationMatrix(new_vals)


class PlanarMode(BaseLimiter):
    """Base-locked Pick-and-Place subset with extra vertical freedom.

    The base is restricted to +/-5 degrees, so the mode can only be
    activated when the base is already inside that band.  Shoulder and
    elbow use the full mechanical travel, giving the arm more vertical
    reach than either Pick-and-Place side.
    """

    name = "Planar"
    limits = PLANAR_LIMITS

    def can_set(self, angles):
        bmin, bmax = BASE_ZERO_RANGE
        return bmin <= angles[0] <= bmax


class PickAndPlaceMode(BaseLimiter):
    """Pick-and-place mode: reach-down workspace.

    Base uses mechanical limits.  Shoulder is restricted to 0–90°
    (cannot go past horizontal).  Elbow is restricted to -135°–0°
    (can only curl downward).  Wrists use mechanical limits.
    """

    name = "Pick and Place"
    limits = PICK_AND_PLACE_LIMITS

    def can_set(self, angles):
        """Only activatable when the arm is already in this workspace."""
        return all(lo - 1e-9 <= a <= hi + 1e-9
                   for a, (lo, hi) in zip(angles, self.limits))


class PickAndPlaceFlippedMode(BaseLimiter):
    """Pick-and-place flipped: mirror of PickAndPlaceMode.

    Shoulder is restricted to 90–180° (past horizontal on the
    opposite side).  Elbow is restricted to 0–135° (curls upward
    on the opposite side).  Base and wrists use mechanical limits.
    """

    name = "Pick and Place Flipped"
    limits = PICK_AND_PLACE_FLIPPED_LIMITS

    def can_set(self, angles):
        """Only activatable when the arm is already in this workspace."""
        return all(lo - 1e-9 <= a <= hi + 1e-9
                   for a, (lo, hi) in zip(angles, self.limits))


class TypingMode(BaseLimiter):
    """Keyboard-typing mode with the base limited to the rover's sides."""

    name = "Typing"
    limits = TYPING_LIMITS


class HomingMode(BaseLimiter):
    """Limits used while the homing controller is driving the arm.

    Uses the same mechanical bounds as FullRangeMode (no dynamic
    overrides) so homing can never push a joint outside hardware
    limits, no matter which mode was active before homing started.
    """

    name = "Homing"
    limits = HOMING_LIMITS

    home_target = list(HOMING_TARGET)


class ZenithMode(BaseLimiter):
    """Straight-up test pose with mechanical travel only.

    The elbow is deliberately allowed to pass through zero: the normal
    singularity deadband would stop the arm a few degrees short of vertical
    and defeat the point of a depower test.
    """

    name = "Zenith"
    limits = FULL_RANGE_LIMITS

    def geometry_constraint(self, angles):
        """Leave the elbow at zero instead of applying the deadband."""
        return list(angles)


class RestingMode(BaseLimiter):
    """Collapse/de-torque operation with padded shoulder/elbow travel.

    The shoulder lower and elbow upper bounds are widened by the resting
    padding so the limit pushes can reach the physical stops.  Like Zenith,
    the elbow may pass through the singularity deadband: the crouch pose
    (+135 deg) is on the opposite side of the band from a normal-side start,
    and the crossing is a controlled slew along a planned path, not an IK
    solve.
    """

    name = "Resting"
    limits = RESTING_LIMITS

    def geometry_constraint(self, angles):
        """Leave angles untouched; the crouch move crosses the deadband."""
        return list(angles)


FULL_RANGE_MODE = FullRangeMode()
PLANAR_MODE = PlanarMode()
PICK_AND_PLACE_MODE = PickAndPlaceMode()
PICK_AND_PLACE_FLIPPED_MODE = PickAndPlaceFlippedMode()
TYPING_MODE = TypingMode()
HOMING_MODE = HomingMode()
ZENITH_MODE = ZenithMode()
RESTING_MODE = RestingMode()

LIMIT_MODES = [PICK_AND_PLACE_MODE]

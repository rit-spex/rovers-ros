"""Arm controller: kinematic state machine for the 5-DOF arm.

Owns drive modes, limit modifiers, joint locks, homing, flips, and target
generation.  Pure library: no ROS imports.
"""

import math
from typing import cast

import numpy as np

from arm_constants import (
    HOMING_TARGET, HOMING_SPEED, HOMING_THRESHOLD,
    MAX_TRANSLATION_DELTA,
    ELBOW_DEADBAND, MECHANICAL_LIMITS, PLANAR_BASE,
    ORIENTATION_SNAP_THRESHOLD, ORIENTATION_SNAP_OFF_AXIS,
    END_EFFECTOR_MIN_Z, TRACK_POS_TOL, TRACK_ORI_TOL, DT,
    WAYPOINT_TOL, WAYPOINT_STALL_LIMIT,
    ZENITH_ANGLE_TOL, ZENITH_JOINT_TARGET,
    RESTING_CROUCH_TARGET, RESTING_INFERENCE_TOL,
    RESTING_MOVE_ANG_VEL,
    FRONT_LOW, BACK_HIGH, BACK_LOW,
    TELEOP_ACCEL_IN_S2, TELEOP_DECEL_IN_S2, TELEOP_ANG_ACCEL_RAD_S2,
    TELEOP_JOINT_ACCEL_TIME_S, TELEOP_LEAD_LIMIT_DEG,
    TELEOP_LEAD_STALL_CYCLES, TELEOP_STOP_EPSILON,
)
from .linkage_models import (
    OrientationMatrix, wrap2pi, nearest_axis_aligned_rotation,
)
from .limiters import (
    PLANAR_MODE, HOMING_MODE, ZENITH_MODE, RESTING_MODE,
    PICK_AND_PLACE_MODE, PICK_AND_PLACE_FLIPPED_MODE, TYPING_MODE,
    OCTANT_DIRECT_FLIP, OCTANT_PLANAR_EXIT_LOCK,
    OCTANT_NORMAL_ALLOWED, OCTANT_PLANAR_ALLOWED,
    OCTANT_PLANAR_FLIPPED_ALLOWED,
    _get_octant,
)
from .actions.home import HomingController
from .actions.flip import FlipController
from .actions.resting import RestingController
from .path_planner import (
    build_arm_solver_object, extract_key_joints, plan_safe_path,
)
from .slew import SimulatedMechanicalArm


class RoverArmController:
    """Central kinematic state container for the 5-DOF rover arm.

    State variables:
        angles: list[float] | None — latest achieved joint angles [rad],
            or None before the first measured sync
        target_angles: list[float] | None — desired joint angles [rad]
            (updated by IK in position drive; equals angles in
            direct-drive modes)
        target_frame: OrientationMatrix — FK of target_angles.
            Always a reachable pose — recomputed from target_angles
            after every IK solve so the target marker never decouples
            from what the arm can actually achieve.
        target_pos: list[float] — [x, y, z] of target_frame
        pos: list — 4 key joint positions (shoulder, elbow,
            wrist, end-effector), from FK of the current angles
        frames: list[OrientationMatrix] — 4 key joint frames
        l0, l1, l2, l3 — shorthand aliases for pos[0..3]
        solver_history: list[IterationResults] — last IK solve trace
        locked_joints: list[bool] — per-joint motor lockout flags
        home_joint_idx: int | None — single joint being homed, or None
    """

    def __init__(self):
        """Build the FK model, IK solver, and initialize all controllers.

        The controller starts with no joint state.  Callers provide measured
        angles with :meth:`from_measured` or ``sync_angles()`` before issuing
        any motion command; until then the controller reports itself as
        uninitialized and refuses commands that need a pose.
        """
        # Core robotics
        self.model, self.solver = build_arm_solver_object()

        # Sub-controllers
        self.homing = HomingController(HOMING_TARGET, HOMING_SPEED, HOMING_THRESHOLD)
        self.flipper = FlipController(duration_frames=30)

        # Operating modes
        self.joint_drive = False       # False=position drive, True=joint drive
        self.locked_joints = [False, False, False, True, True]
        self.position_locked = False   # True = target translation disabled
        self.orientation_locked = False  # True = target rotation disabled
        self._orientation_lock_rotation: np.ndarray | None = None
        self.home_joint_idx: int | None = None  # None=full homing, int=single
        self._homing_flipped = False   # side whose home pose is targeted
        self._homing_cross_dir = 0     # elbow deadband crossing direction
        self.limit_mode = PICK_AND_PLACE_MODE
        self.flipped = False           # Pick-and-Place side modifier
        self.planar = False            # base-locked vertical modifier
        self.typing = False            # keyboard-typing limits/modifier
        self.zenith = False            # straight-up depower/test pose
        self._zenith_side = False      # side to restore when leaving zenith
        self._zenith_exit_target: list[float] | None = None
        self._zenith_saved_locks: list[bool] | None = None
        self.resting = False           # collapse/de-torque operation active
        self._resting_saved_locks: list[bool] | None = None
        self.resting_action = RestingController()
        self._resting_path_phase: str | None = None
        self._resting_events: list[str] = []
        self._typing_saved_locks: list[bool] | None = None
        self.solver_history = []       # debug: last IK iteration results
        self.last_failure: str | None = None

        # Late-initialized state.  These are None at runtime until the first
        # measured sync; the casts stop static checkers from treating every
        # later access as optional.  Check ``initialized`` before relying on
        # them outside this class.
        self._initialized = False
        self.mechanical: SimulatedMechanicalArm = cast(
            SimulatedMechanicalArm, None)
        self.target_angles: list[float] = cast(list[float], None)
        # Last angle sent to the hardware bridge, per joint.  Locked joints
        # keep this value so encoder noise cannot make a motor chase itself.
        self._last_commanded: list[float] = cast(list[float], None)

        # Teleop output stage.  The reference is an internal, rate-limited
        # joint trajectory.  It is separate from the measured angles so the
        # command does not inherit encoder noise or feedback timing, and it
        # is anchored each cycle to keep the IK target from running away.
        self.teleop_reference: list[float] | None = None
        self.teleop_velocity = [0.0] * 6       # [in/s, in/s, in/s, rad/s]
        self.teleop_joint_velocity = [0.0] * 5  # [rad/s] per joint
        self.teleop_lead_cycles = 0
        self._teleop_needs_reset = True
        self._teleop_stalled = False

        # Safe homing path and flip waypoint follow state
        self._homing_path: list[list[float]] | None = None
        self._homing_path_idx = 0
        self._homing_seg_steps: list[float] | None = None
        self._follow_target: list[float] | None = None
        self._follow_path: list[list[float]] | None = None
        self._follow_path_idx = 0
        self._follow_limits: list[tuple[float, float]] | None = None
        self._follow_switch_mode = False
        self._follow_max_step: list[float] | None = None
        self._follow_stall = 0
        self._homing_stall = 0

        # Target frame is FK(target_angles) once a measured pose arrives.
        self.target_frame: OrientationMatrix = cast(OrientationMatrix, None)
        self.target_pos: list[float] = cast(list[float], None)

        # Achieved state — computed by refresh_fk()
        self.pos: list[list[float]] | None = None
        self.frames: list[OrientationMatrix] | None = None
        self.l0: list[float] | None = None
        self.l1: list[float] | None = None
        self.l2: list[float] | None = None
        self.l3: list[float] | None = None
        self.refresh_fk()

    @classmethod
    def from_measured(cls, angles):
        """Build a controller initialized from measured joint angles.

        The resulting controller has its target frame snapped to the
        measured pose and adopts the mode that fits that pose.
        """
        controller = cls()
        controller.sync_angles(angles, reset_target=True)
        controller.infer_initial_mode()
        return controller

    # ------------------------------------------------------------------
    # Joint state / mechanical arm
    # ------------------------------------------------------------------

    @property
    def angles(self) -> list[float]:
        """Latest joint angles.

        The value is ``None`` at runtime until the first measured sync; call
        ``initialized`` first if you need to distinguish that case.  The
        return type is non-optional so callers that have already checked
        ``initialized`` do not need a redundant guard.
        """
        if not self._initialized:
            return cast(list[float], None)
        return self.mechanical.angles

    @property
    def _angles(self) -> list[float]:
        """Non-optional joint angles for internal use after initialization."""
        return self.mechanical.angles

    @property
    def initialized(self) -> bool:
        """True once measured joint angles have been adopted."""
        return self._initialized

    def _require_initialized(self) -> bool:
        """Record a rejection and return False when no pose is known."""
        if self.initialized:
            return True
        return self._fail("Arm not initialized")

    def commanded_angles(self) -> list[float] | None:
        """Return the joint angles to send to the hardware bridge.

        Unlocked joints follow the achieved angles.  Locked joints keep the
        last commanded target, so encoder noise and tracking lag cannot make
        a motor chase its own feedback.  Returns None before the first sync.
        """
        if not self.initialized:
            return None
        result = list(self._angles)
        for index, locked in enumerate(self.locked_joints):
            if locked:
                result[index] = self._last_commanded[index]
            else:
                self._last_commanded[index] = result[index]
        return result

    def sync_angles(self, angles, reset_target=False):
        """Adopt measured joint angles from real hardware.

        The controller normally owns ``angles`` through its simulated
        mechanical arm.  A ROS node driving a physical arm calls this
        once per control cycle so homing, flipping, joint locks, and the
        IK start pose all track the real arm instead of a simulated one.

        Args:
            angles: Measured joint angles [rad], one per joint.
            reset_target: When True, also snap ``target_angles`` and the
                target frame to the measured pose.  Use this only on the
                first sync (before any input), otherwise position-drive
                targets would be thrown away every cycle.
        """
        angles = [float(a) for a in angles]
        expected = len(PICK_AND_PLACE_MODE.limits)
        if len(angles) != expected:
            raise ValueError(
                f"Expected {expected} angles, got {len(angles)}")
        # Every non-teleop path syncs measured angles.  Mark the teleop
        # reference stale so the next teleop cycle restarts from the arm
        # instead of resuming a trajectory from before the action.
        self._teleop_needs_reset = True
        first_sync = not self._initialized
        if first_sync:
            self.mechanical = SimulatedMechanicalArm(angles)
            self.last_failure = None
        else:
            self.mechanical.angles = angles
        self._initialized = True
        if reset_target or first_sync:
            self.mechanical.joint_speeds = [0.0] * len(angles)
            self.target_angles = list(angles)
            self._last_commanded = list(angles)
            frame = self.model.solve_forward_kinematics(self.target_angles)
            self.target_frame = self._frame_with_orientation_lock(frame)
            self.target_pos = list(self.target_frame.position())
        self.refresh_fk()

    def _command_mechanical(self, target_angles, joint_limits,
                            max_step=None):
        """Slew the mechanical arm toward a target for one cycle."""
        self.mechanical.step(target_angles, joint_limits, DT,
                             max_step=max_step)

    def _locked_limits(self, joint_limits):
        """Pin locked joints to their actual angles in a limit list."""
        if joint_limits is None:
            result = [None] * len(self._angles)
        else:
            result = [tuple(pair) if pair is not None else None
                      for pair in joint_limits]
        for i, locked in enumerate(self.locked_joints):
            if locked and i < len(result):
                a = self._angles[i]
                result[i] = (a, a)
        return result

    def _all_at(self, target, tol=HOMING_THRESHOLD, mask=None):
        """True when every allowed joint is within ``tol`` of target."""
        for i, tgt in enumerate(target):
            if mask is not None and i < len(mask) and not mask[i]:
                continue
            if abs(wrap2pi(tgt - self._angles[i])) > tol:
                return False
        return True

    def _target_error(self, target, mask=None):
        """Largest remaining error to ``target`` over the allowed joints."""
        error = 0.0
        for i, tgt in enumerate(target):
            if mask is not None and i < len(mask) and not mask[i]:
                continue
            error = max(error, abs(wrap2pi(tgt - self._angles[i])))
        return error

    def _drive_mechanical(self, target_angles, joint_limits, mask=None,
                          max_step=None, tol=HOMING_THRESHOLD):
        """Step the mechanical arm and report whether it has arrived."""
        self.mechanical.step(target_angles, joint_limits, DT,
                             max_step=max_step)
        if mask is None:
            mask = [not locked for locked in self.locked_joints]
        return self._all_at(target_angles, tol=tol, mask=mask)

    def _synchronized_steps(self, start, target):
        """Per-joint step limits so every moving joint finishes together.

        The largest remaining joint move sets the duration at that joint's
        hardware speed; every other joint gets the step that matches the
        same completion time, capped by its own speed.
        """
        deltas = [abs(wrap2pi(t - s)) for s, t in zip(start, target)]
        speeds = self.mechanical.max_ang_vel
        duration = max(
            (d / v for d, v in zip(deltas, speeds) if v > 1e-12),
            default=0.0,
        )
        if duration < 1e-12:
            return [0.0] * len(deltas)
        return [d * DT / duration for d in deltas]

    # ------------------------------------------------------------------
    # FK refresh
    # ------------------------------------------------------------------

    def refresh_fk(self):
        """Recompute achieved FK joint data from current angles.

        Evaluates the forward kinematics chain at the current joint
        angles and extracts the 4 key joint positions (after each
        linkage translation: shoulder, elbow, wrist, end-effector)
        and their corresponding 4x4 frames.  Does not touch the
        desired target frame.  Clears the cached pose before the first
        measured sync.
        """
        if not self.initialized:
            self.pos = None
            self.frames = None
            self.l0 = self.l1 = self.l2 = self.l3 = None
            return
        positions = self.model.joint_positions(self._angles, frame=False)
        frames = self.model.joint_positions(self._angles, frame=True)
        self.pos, self.frames = extract_key_joints(positions, frames)
        self.l0, self.l1, self.l2, self.l3 = self.pos

    def _sync_target_to_fk(self):
        """Snap target_angles and target_frame to the achieved FK pose.

        Used when the arm is driven directly (homing, flip, joint
        drive) so the target marker tracks the arm instead of leaving
        a stale desired pose behind.
        """
        self.target_angles = list(self._angles)
        frame = self.model.solve_forward_kinematics(self.target_angles)
        self.target_frame = self._frame_with_orientation_lock(frame)
        self.target_pos = list(self.target_frame.position())

    # ------------------------------------------------------------------
    # Mode queries
    # ------------------------------------------------------------------

    planar_exit_lock_octants = OCTANT_PLANAR_EXIT_LOCK

    def _ee_octant(self) -> int | None:
        """Octant containing the current end-effector position."""
        if not self.initialized:
            return None
        pos = self.model.solve_forward_kinematics(self._angles).position()
        return _get_octant(pos)

    def _workspace_blocked(self, angles):
        """True when a candidate pose violates the height or octant gates."""
        if self.is_homing or self.is_flipping or self.zenith \
                or self.resting:
            return False
        pos = self.model.solve_forward_kinematics(angles).position()
        if pos[2] < END_EFFECTOR_MIN_Z - 1e-9:
            self._fail(
                f"Height limit: end effector must stay above "
                f"{END_EFFECTOR_MIN_Z:.0f} in")
            return True
        return self._octant_blocked(angles, pos)

    def _octant_blocked(self, angles, pos=None):
        """True when a candidate pose is outside this mode's octant set."""
        if self.is_homing or self.is_flipping or self.zenith \
                or self.resting:
            return False
        if pos is None:
            pos = self.model.solve_forward_kinematics(angles).position()
        octant = _get_octant(pos)
        if self.planar:
            allowed = (OCTANT_PLANAR_FLIPPED_ALLOWED if self.flipped
                       else OCTANT_PLANAR_ALLOWED)
        else:
            allowed = OCTANT_NORMAL_ALLOWED
        if octant in allowed:
            return False
        if self.planar:
            if self.flipped:
                self._fail("Planar flipped limited to octants 3, 7")
            else:
                self._fail("Planar limited to octants 1, 5")
        else:
            side = "Flipped" if self.flipped else "Normal"
            self._fail(f"{side} blocked: octant {octant} requires Planar")
        return True

    def _planar_exit_allowed(self):
        """False while the end effector sits in an octant that needs planar."""
        return self._ee_octant() not in self.planar_exit_lock_octants

    @property
    def ee_octant(self):
        """Current end-effector octant (1-8)."""
        return self._ee_octant()

    def _active_limits(self, angles=None):
        """Limits for Pick and Place with its planar/flipped/typing modifiers."""
        if self.zenith:
            source = ZENITH_MODE.limits
        elif self.typing:
            source = TYPING_MODE.limits
        elif self.planar:
            source = PLANAR_MODE.limits
        elif self.flipped:
            source = PICK_AND_PLACE_FLIPPED_MODE.limits
        else:
            source = PICK_AND_PLACE_MODE.limits
        return [tuple(pair) for pair in source]

    @staticmethod
    def _flipped_home():
        """Home pose mirrored to the flipped Pick-and-Place side."""
        return [
            HOMING_TARGET[0],
            math.pi - HOMING_TARGET[1],
            -HOMING_TARGET[2],
            HOMING_TARGET[3],
            HOMING_TARGET[4],
        ]

    def _home_target(self):
        """Home pose for the currently selected side."""
        return self._flipped_home() if self.flipped else list(HOMING_TARGET)

    def settle_side(self):
        """Update the flipped flag from the achieved pose, if possible."""
        if (not self.initialized or self.planar or self.typing
                or self.zenith or self.resting):
            return
        if PICK_AND_PLACE_FLIPPED_MODE.can_set(self._angles):
            self.flipped = True
        elif PICK_AND_PLACE_MODE.can_set(self._angles):
            self.flipped = False

    @staticmethod
    def _limit_excursion(angles, limits):
        """Squared distance from *angles* to the nearest point inside limits."""
        return sum(
            max(lo - angle, 0.0, angle - hi) ** 2
            for angle, (lo, hi) in zip(angles, limits)
        )

    def _is_zenith_pose(self, tol=ZENITH_ANGLE_TOL):
        """True when the measured joints are already the straight-up pose."""
        if not self.initialized:
            return False
        angles = self._angles
        if abs(wrap2pi(angles[1] - ZENITH_JOINT_TARGET[0])) > tol:
            return False
        if abs(wrap2pi(angles[2])) > max(tol, ELBOW_DEADBAND):
            return False
        if abs(wrap2pi(angles[3])) > tol:
            return False
        if abs(wrap2pi(angles[4])) > tol:
            return False
        return True

    def _is_resting_pose(self):
        """True when the measured pose sits in the resting region.

        The resting region is the shoulder below the planar lower bound
        (0 deg: a planar top-back pose may legitimately sit at 0-2 deg) or
        the elbow above its normal upper limit, by more than the inference
        tolerance.  The collapsed pose (shoulder ~-9.5 deg after the
        backoff) and mid-push crashes past the elbow limit both match.
        Checked after zenith, so the straight-up pose never matches.
        """
        if not self.initialized:
            return False
        angles = self._angles
        tol = RESTING_INFERENCE_TOL
        shoulder_below = angles[1] < 0.0 - tol
        elbow_above = (
            angles[2] > MECHANICAL_LIMITS[2][1] + tol)
        return shoulder_below or elbow_above

    def infer_initial_mode(self):
        """Adopt the mode that best describes the measured starting pose.

        Startup can happen with the arm already outside the boot-default
        Pick-and-Place normal limits.  This runs once, before any operator
        command, so the operation mode and flags published to the
        basestation describe the pose the arm is actually in rather than
        the compile-time default.

        The straight-up pose selects Zenith.  The restricted octants
        (notably top-back) select Planar when the base is inside the planar
        band.  Otherwise the Pick-and-Place side whose limits fit wins; if
        neither fits, the side needing the smaller correction is kept so
        the first motion pulls the arm back into bounds.

        Returns:
            str | None: Selected mode name, or None before initialization.
        """
        if not self.initialized:
            return None

        if self._is_zenith_pose():
            self.enter_zenith()
            return "ZENITH"

        # A pose in the resting region (collapsed against the stops, or a
        # mid-crouch pose past the normal limits) means the arm was resting
        # when it lost power: stay de-torqued and wait for the restore.
        if self._is_resting_pose():
            self.infer_resting_mode()
            return "RESTING"

        octant = self._ee_octant()
        normal_fits = PICK_AND_PLACE_MODE.can_set(self._angles)
        flipped_fits = PICK_AND_PLACE_FLIPPED_MODE.can_set(self._angles)
        planar_possible = PLANAR_MODE.can_set(self._angles)

        if planar_possible and octant in OCTANT_PLANAR_EXIT_LOCK:
            self.set_typing_mode(False)
            self.planar = True
            self.flipped = octant in OCTANT_PLANAR_FLIPPED_ALLOWED
            self.limit_mode = PICK_AND_PLACE_MODE
            return "PLANAR"

        # Without planar, use the side that matches the octant region when
        # its joint limits fit: front-low is a normal-side reach, top/back is
        # the flipped side.
        if octant == FRONT_LOW and normal_fits:
            self.set_typing_mode(False)
            self.planar = False
            self.flipped = False
            self.limit_mode = PICK_AND_PLACE_MODE
            return "PNP"

        if octant in (BACK_HIGH, BACK_LOW) and flipped_fits:
            self.set_typing_mode(False)
            self.planar = False
            self.flipped = True
            self.limit_mode = PICK_AND_PLACE_MODE
            return "PNP_FLIPPED"

        if normal_fits:
            self.set_typing_mode(False)
            self.planar = False
            self.flipped = False
            self.limit_mode = PICK_AND_PLACE_MODE
            return "PNP"

        if flipped_fits:
            self.set_typing_mode(False)
            self.planar = False
            self.flipped = True
            self.limit_mode = PICK_AND_PLACE_MODE
            return "PNP_FLIPPED"

        if planar_possible and (octant in OCTANT_PLANAR_ALLOWED
                                or octant in OCTANT_PLANAR_FLIPPED_ALLOWED):
            self.set_typing_mode(False)
            self.planar = True
            self.flipped = octant in OCTANT_PLANAR_FLIPPED_ALLOWED
            self.limit_mode = PICK_AND_PLACE_MODE
            return "PLANAR"

        normal_error = self._limit_excursion(
            self._angles, PICK_AND_PLACE_MODE.limits)
        flipped_error = self._limit_excursion(
            self._angles, PICK_AND_PLACE_FLIPPED_MODE.limits)
        self.set_typing_mode(False)
        self.planar = False
        self.flipped = flipped_error < normal_error
        self.limit_mode = PICK_AND_PLACE_MODE
        return "PNP_FLIPPED" if self.flipped else "PNP"

    def _zenith_target_angles(self):
        """Full joint target for the straight-up pose, base included."""
        return [self._angles[0], *[float(a) for a in ZENITH_JOINT_TARGET]]

    def enter_zenith(self):
        """Drive to the straight-up test pose and select Zenith mode.

        The base stays where it is.  Joint locks are released while the
        mode is active so every joint can reach its target, then restored
        when the arm leaves zenith.  The Pick-and-Place side selected before
        entry is remembered for :meth:`exit_zenith`.
        """
        if not self._require_initialized():
            return False
        if self.homing.active or self.is_flipping:
            return self._fail("Zenith blocked: an action is running")
        if self.zenith:
            return True

        self.set_typing_mode(False)
        self._zenith_side = self.flipped
        self._zenith_saved_locks = list(self.locked_joints)
        self.locked_joints = [False] * len(self.locked_joints)
        self.planar = False
        self.flipped = False
        self.joint_drive = False
        self.zenith = True
        self._zenith_exit_target = None
        self.limit_mode = ZENITH_MODE
        return True

    def exit_zenith(self):
        """Move to the nearest valid pose on the last Pick-and-Place side.

        Zenith sits on the boundary between the normal and flipped sides.
        Leaving it nudges the shoulder/elbow just inside the remembered
        side's limits instead of guessing which side to flip to.
        """
        if not self.zenith:
            return True
        if self._zenith_exit_target is not None:
            return True

        angles = list(self._angles)
        if self._zenith_side:
            shoulder_lo, shoulder_hi = (
                PICK_AND_PLACE_FLIPPED_MODE.limits[1])
            elbow = max(angles[2], ELBOW_DEADBAND)
        else:
            shoulder_lo, shoulder_hi = PICK_AND_PLACE_MODE.limits[1]
            elbow = min(angles[2], -ELBOW_DEADBAND)

        wrist_lo, wrist_hi = MECHANICAL_LIMITS[3]
        twist_lo, twist_hi = MECHANICAL_LIMITS[4]
        self._zenith_exit_target = [
            angles[0],
            max(shoulder_lo, min(shoulder_hi, angles[1])),
            elbow,
            max(wrist_lo, min(wrist_hi, angles[3])),
            max(twist_lo, min(twist_hi, angles[4])),
        ]
        return True

    def toggle_zenith(self):
        """Enter Zenith, or leave it for the nearest valid side pose."""
        if self.zenith:
            return self.exit_zenith()
        return self.enter_zenith()

    def _update_zenith(self):
        """Slew to the straight-up target or to the pending exit pose."""
        target = (
            list(self._zenith_exit_target)
            if self._zenith_exit_target is not None
            else self._zenith_target_angles()
        )
        limits = [tuple(pair) for pair in ZENITH_MODE.limits]
        arrived = self._drive_mechanical(target, limits, tol=1e-3)
        self.target_angles = list(self._angles)
        self._sync_target_to_fk()
        if arrived and self._zenith_exit_target is not None:
            self._finish_zenith_exit()

    def _finish_zenith_exit(self):
        """Restore the remembered side and locks after leaving zenith."""
        self.zenith = False
        self._zenith_exit_target = None
        self.planar = False
        self.typing = False
        self.flipped = self._zenith_side
        if self._zenith_saved_locks is not None:
            self.locked_joints = list(self._zenith_saved_locks)
            self._zenith_saved_locks = None
        self.limit_mode = PICK_AND_PLACE_MODE
        self._sync_target_to_fk()

    # ------------------------------------------------------------------
    # Resting (collapse / de-torque / restore)
    # ------------------------------------------------------------------

    def enter_resting(self):
        """Start the RESTING collapse sequence.

        Mirrors zenith's entry reset: typing is left first, the operator's
        joint locks are saved and released, and the planar/flipped/
        joint-drive modifiers are cleared so the resting action owns every
        joint.  The saved locks are restored when the sequence completes.
        """
        if not self._require_initialized():
            return False
        if self.zenith:
            return self._fail("Resting blocked: exit zenith mode first")
        if self.homing.active or self.is_flipping:
            return self._fail("Resting blocked: an action is running")
        if self.resting:
            return True

        self.set_typing_mode(False)
        self._resting_saved_locks = list(self.locked_joints)
        self.locked_joints = [False] * len(self.locked_joints)
        self.planar = False
        self.flipped = False
        self.joint_drive = False
        self.resting = True
        self.limit_mode = RESTING_MODE
        self._resting_path_phase = None
        self.invalidate_teleop_reference()
        return self.resting_action.begin_collapse()

    def infer_resting_mode(self):
        """Enter RESTING from a startup pose, holding instead of collapsing.

        A resting-region startup pose means the drives should stay
        de-torqued; the first R restores torque instead of collapsing.
        """
        if not self.enter_resting():
            return False
        self.resting_action.abort("Resting mode inferred from startup pose")
        return True

    def toggle_resting(self):
        """R key: enter, abort-and-hold, or begin the restore sequence.

        While a sequence is running, R aborts it and holds the measured pose
        with torque on.  From COLLAPSED or HOLD, R starts the restore.
        """
        if self.resting:
            if self.resting_action.collapsed \
                    or self.resting_action.holding:
                return self.resting_action.request_restore()
            self.resting_action.abort(
                "Resting sequence aborted by operator")
            return True
        return self.enter_resting()

    def resting_torque_confirmed(self):
        """Advance TORQUE_ON -> MOVE_OUT after hardware feedback."""
        return self.resting_action.torque_confirmed()

    def resting_homing_finished(self):
        """Advance AUTO_HOME -> DONE once homing completes."""
        if not self.resting_action.homing_finished():
            return False
        self._finish_resting()
        return True

    def _finish_resting(self):
        """Restore the saved locks and normal mode after the restore ends."""
        self.resting = False
        self.planar = False
        self.typing = False
        self.joint_drive = False
        if self._resting_saved_locks is not None:
            self.locked_joints = list(self._resting_saved_locks)
            self._resting_saved_locks = None
        self.limit_mode = PICK_AND_PLACE_MODE
        self.settle_side()
        self._sync_target_to_fk()

    def take_resting_events(self):
        """Return and clear the resting phase-transition events."""
        events = self._resting_events
        self._resting_events = []
        return events

    def _plan_resting_path(self):
        """Plan a safe joint path for the crouch/move-out phase, if possible.

        The elbow must cross the singularity deadband to reach the crouch
        (+135 deg); RESTING_MODE deliberately leaves the deadband unenforced
        so the slew can cross it, and the planned path keeps the move out of
        restricted octants.  When no safe path exists, the resting controller
        falls back to a direct slow step.
        """
        phase = self.resting_action.phase
        if phase not in (self.resting_action.PHASE_CROUCH,
                         self.resting_action.PHASE_MOVE_OUT):
            self._resting_path_phase = None
            return
        if self._resting_path_phase == phase:
            return
        self._resting_path_phase = phase
        if self.resting_action._path is not None:
            return
        held = list(self._angles)
        target = self._hold_joints(RESTING_CROUCH_TARGET, held)
        path = plan_safe_path(self.model, held, target)
        if path and len(path) > 1:
            self.resting_action.set_path(
                path[1:],
                [RESTING_MOVE_ANG_VEL * DT] * len(held))

    def _update_resting(self, feedback_fresh=True, enabled=True):
        """Advance the resting state machine and drive the mechanical arm."""
        self._plan_resting_path()
        target, max_step, events = self.resting_action.update(
            self._angles, DT, feedback_fresh=feedback_fresh,
            enabled=enabled)
        self._resting_events.extend(events)

        for event in events:
            if event == "home":
                # Hand the arm to the normal homing action at the crouch.
                self.resting = False
                self.limit_mode = PICK_AND_PLACE_MODE
                self.start_homing()
            elif event.startswith("abort:"):
                self._fail(event[len("abort:"):])
                self._sync_target_to_fk()

        if target is not None:
            limits = self._locked_limits(RESTING_MODE.limits)
            self._command_mechanical(target, limits, max_step=max_step)
            self._sync_target_to_fk()
        return events

    def _fail(self, message):
        """Record a user-facing rejection and return False."""
        self.last_failure = message
        return False

    def take_failure(self):
        """Return and clear the last rejection message."""
        message = self.last_failure
        self.last_failure = None
        return message

    @property
    def base_in_range(self):
        """True when the base lies inside the planar modifier's band."""
        return self.initialized and PLANAR_MODE.can_set(self._angles)

    @property
    def effective_limits(self):
        """Per-joint limits currently enforced by the arm.

        Flip and homing actions run under full mechanical travel; the
        normal Pick-and-Place limits return when they finish.
        """
        if self.is_flipping or self.is_homing:
            return [tuple(pair) for pair in MECHANICAL_LIMITS]
        if self.resting:
            return [tuple(pair) for pair in RESTING_MODE.limits]
        if self.zenith:
            return [tuple(pair) for pair in ZENITH_MODE.limits]
        if self.typing:
            return [tuple(pair) for pair in TYPING_MODE.limits]
        return self._active_limits(self._angles)

    @property
    def is_flipping(self):
        """True while the flip action or its follow-through is running."""
        return self.flipper.active or self._follow_target is not None

    @property
    def is_homing(self):
        """True while the homing action is running."""
        return self.homing.active

    @property
    def homing_joints(self):
        """Per-joint flags for joints the homing action is moving."""
        if not self.is_homing:
            return [False] * len(self.locked_joints)
        return self._homing_mask()

    # ------------------------------------------------------------------
    # Modifier transitions (called by the simulator via keyboard actions)
    # ------------------------------------------------------------------

    def set_typing_mode(self, enabled):
        """Select or leave the keyboard-typing limits.

        Typing widens the shoulder/elbow travel beyond Pick and Place and
        releases the wrist locks, because a key press needs a level tool
        orientation.  The previous wrist lock state is restored on exit.
        """
        if not self._require_initialized():
            return False
        if enabled and self.zenith:
            return self._fail("Typing blocked: exit zenith mode first")
        if enabled and self.resting:
            return self._fail("Typing blocked: resting is active")
        if enabled:
            if not self.typing:
                self._typing_saved_locks = list(self.locked_joints)
            self.typing = True
            self.planar = False
            self.flipped = False
            self.joint_drive = False
            self.limit_mode = TYPING_MODE
            self.locked_joints[3] = False
            self.locked_joints[4] = False
            return True

        if not self.typing:
            return True
        self.typing = False
        self.limit_mode = PICK_AND_PLACE_MODE
        if self._typing_saved_locks is not None:
            self.locked_joints = list(self._typing_saved_locks)
            self._typing_saved_locks = None
        self.settle_side()
        return True

    def set_pnp_normal(self):
        """Select normal Pick-and-Place limits when the arm fits them."""
        if not self._require_initialized():
            return False
        if self.zenith:
            return self._fail("Normal unavailable: exit zenith mode first")
        if self.resting:
            return self._fail("Normal unavailable: resting is active")
        if self.planar and not self._planar_exit_allowed():
            return self._fail(
                "Normal unavailable: restricted octant requires planar")
        if not PICK_AND_PLACE_MODE.can_set(self._angles):
            return self._fail(
                "Normal unavailable: pose is outside normal limits")
        self.set_typing_mode(False)
        self.planar = False
        self.flipped = False
        return True

    def toggle_flipped(self):
        """Select flipped-side limits when the arm fits them.

        This only changes the limit modifier; it never moves the arm.
        Use activate_flip() to physically change sides.
        """
        if not self._require_initialized():
            return False
        if self.zenith:
            return self._fail("Flipped unavailable: exit zenith mode first")
        if self.resting:
            return self._fail("Flipped unavailable: resting is active")
        if self.planar and not self._planar_exit_allowed():
            return self._fail(
                "Flipped unavailable: restricted octant requires planar")
        if self.flipped:
            if not PICK_AND_PLACE_MODE.can_set(self._angles):
                return self._fail(
                    "Normal unavailable: pose is outside normal limits")
            self.set_typing_mode(False)
            self.flipped = False
            return True

        if not PICK_AND_PLACE_FLIPPED_MODE.can_set(self._angles):
            return self._fail(
                "Flipped unavailable: pose is outside flipped limits")
        self.set_typing_mode(False)
        self.planar = False
        self.flipped = True
        return True

    def toggle_planar(self):
        """Toggle the base-locked planar modifier.

        Planar cannot be exited while the end effector is in octants
        3, 5, or 7; those regions require the base lock.
        """
        if not self._require_initialized():
            return False
        if self.zenith:
            return self._fail("Planar unavailable: exit zenith mode first")
        if self.resting:
            return self._fail("Planar unavailable: resting is active")
        if self.planar:
            if not self._planar_exit_allowed():
                return self._fail(
                    "Planar locked: restricted octant requires planar")
            if PICK_AND_PLACE_FLIPPED_MODE.can_set(self._angles):
                self.set_typing_mode(False)
                self.planar = False
                self.flipped = True
                return True
            if PICK_AND_PLACE_MODE.can_set(self._angles):
                self.set_typing_mode(False)
                self.planar = False
                self.flipped = False
                return True
            return self._fail(
                "Planar cannot be exited at this pose")

        if not PLANAR_MODE.can_set(self._angles):
            return self._fail("Planar needs base within +/-5 deg")
        self.set_typing_mode(False)
        self.planar = True
        return True

    def start_homing(self):
        """Start full-arm homing, or keep an active full-home request alive.

        Repeated calls while full homing is already active are idempotent, so
        a held homing key can refresh the planner's release watchdog without
        restarting or cancelling the action.
        """
        if not self._require_initialized():
            return False
        if self.zenith:
            return self._fail("Homing blocked: exit zenith mode first")
        if self.resting:
            return self._fail("Homing blocked: resting is active")
        if self.homing.active and self.home_joint_idx is None:
            return True
        if self.homing.active:
            self.stop_homing(settle=False)
        self.home_joint_idx = None
        self.homing.target = self._home_target()
        self._homing_flipped = self.flipped
        self.homing.activate()
        self._homing_path = None
        self._homing_seg_steps = None
        self._homing_stall = 0
        if not any(self.locked_joints[:3]):
            effective = self._hold_joints(
                self.homing.target, self._angles)
            path = plan_safe_path(
                self.model, self._angles, effective)
            if path and len(path) > 1:
                self._homing_path = path[1:]
                self._homing_path_idx = 0
                self._homing_seg_steps = self._synchronized_steps(
                    self._angles, self._homing_path[0])
        self._homing_cross_dir = 0
        return True

    def stop_homing(self, settle=True):
        """Stop an active homing action and discard its planned path."""
        self.homing.deactivate()
        self.home_joint_idx = None
        self._homing_path = None
        self._homing_seg_steps = None
        self._homing_stall = 0
        self._homing_cross_dir = 0
        if settle:
            self.settle_side()
        return True

    def toggle_homing(self):
        """Toggle full-arm homing on/off.

        Kept for direct callers and tests.  The ROS command path uses
        start_homing()/stop_homing() so a held key can keep the action alive.
        """
        if self.homing.active:
            return self.stop_homing()
        return self.start_homing()

    def toggle_joint_lock(self, idx):
        """Toggle the motor lockout for one joint.

        Locked joints are frozen in every drive mode until unlocked.

        Args:
            idx: Joint index 0-4.

        Returns:
            bool: The new lock state, or False if the index is invalid.
        """
        if not (0 <= idx < len(self.locked_joints)):
            return False
        if self.resting:
            return self._fail("Joint lock blocked: resting is active")
        self.locked_joints[idx] = not self.locked_joints[idx]
        return self.locked_joints[idx]

    def toggle_position_lock(self):
        """Toggle locking the target's translation."""
        self.position_locked = not self.position_locked
        return self.position_locked

    def toggle_orientation_lock(self):
        """Toggle locking the target's rotation."""
        if not self._require_initialized():
            return False
        self.orientation_locked = not self.orientation_locked
        if self.orientation_locked:
            self._orientation_lock_rotation = (
                self.target_frame.rotation_matrix().copy())
        else:
            self._orientation_lock_rotation = None
        return self.orientation_locked

    def _locked_orientation(self):
        """Rotation matrix to hold while orientation lock is active."""
        if not self.orientation_locked:
            return None
        if self._orientation_lock_rotation is None:
            self._orientation_lock_rotation = (
                self.target_frame.rotation_matrix().copy())
        return self._orientation_lock_rotation

    def _frame_with_orientation_lock(self, frame):
        """Replace a frame's rotation with the locked orientation, if any."""
        locked_rotation = self._locked_orientation()
        if locked_rotation is None:
            return frame
        return frame.with_rotation(locked_rotation)

    def home_joint(self, idx):
        """Drive a single joint to its home angle.

        The joint is unlocked first so the command always takes effect.

        Args:
            idx: Joint index 0-4.

        Returns:
            bool: True if single-joint homing started.
        """
        if not self._require_initialized():
            return False
        if self.zenith:
            return self._fail("Joint home blocked: exit zenith mode first")
        if self.resting:
            return self._fail("Joint home blocked: resting is active")
        if not (0 <= idx < len(self.locked_joints)):
            return False
        if self.homing.active and self.home_joint_idx == idx:
            return True
        if self.homing.active:
            self.stop_homing(settle=False)
        self.locked_joints[idx] = False
        self.homing.target = self._home_target()
        self._homing_flipped = self.flipped
        self.homing.activate()
        self.home_joint_idx = idx
        self._homing_path = None
        self._homing_stall = 0
        self._homing_cross_dir = 0
        return True

    def _homing_mask(self):
        """Per-joint mask of which joints the homing action may move."""
        mask = [not locked for locked in self.locked_joints]
        if self.home_joint_idx is not None:
            mask = [allowed and i == self.home_joint_idx
                    for i, allowed in enumerate(mask)]
        return mask

    def _hold_joints(self, angles, held, mask=None):
        """Replace joints that are not allowed to move with held values.

        Args:
            angles: Candidate angle list.
            held: Angle list captured before the update.
            mask: Optional per-joint allowed-to-move flags.  Defaults to
                the inverse of ``locked_joints``.

        Returns:
            list[float]: Angles with disallowed joints restored.
        """
        result = list(angles)
        for i in range(len(result)):
            allowed = mask[i] if mask is not None else not self.locked_joints[i]
            if not allowed and i < len(held):
                result[i] = held[i]
        return result

    def set_joint_drive(self, enabled):
        """Select joint drive (True) or position drive (False)."""
        self.joint_drive = bool(enabled)

    def toggle_joint_drive(self):
        """Toggle between joint drive and position drive."""
        self.set_joint_drive(not self.joint_drive)

    def command_joint_angle(self, index, angle):
        """Command one joint toward an absolute target angle.

        The command is clamped to the active limits, respects locked
        joints, and drives the mechanical arm at its normal velocity.
        """
        if not self._require_initialized():
            return False
        if self.homing.active or self.is_flipping or self.zenith \
                or self.resting:
            return self._fail("Command blocked: action is running")
        if not 0 <= index < min(len(self._angles), len(self.target_angles)):
            return self._fail("Command blocked: invalid joint")

        held = list(self._angles)
        target = list(self.target_angles)
        limits = self._active_limits(target)
        lo, hi = limits[index]
        target[index] = max(lo, min(hi, float(angle)))
        target = self.limit_mode.geometry_constraint(target)
        target = self._hold_joints(target, held)
        if self._workspace_blocked(target):
            return False

        self.target_angles = target
        frame = self.model.solve_forward_kinematics(target)
        self.target_frame = self._frame_with_orientation_lock(frame)
        self.target_pos = list(self.target_frame.position())
        limits = self._active_limits(target)
        self._command_mechanical(target, self._locked_limits(limits))
        return True

    def command_target_axis(self, axis, value):
        """Command the target position on one world axis."""
        if not self._require_initialized():
            return False
        if self.homing.active or self.is_flipping or self.zenith \
                or self.resting:
            return self._fail("Command blocked: action is running")
        if axis not in (0, 1, 2):
            return self._fail("Command blocked: invalid axis")

        pos = list(self.target_pos)
        pos[axis] = float(value)
        desired = OrientationMatrix(self.target_frame.tolist())
        desired.set_position(pos)
        locked_rotation = self._locked_orientation()
        if locked_rotation is not None:
            desired = desired.with_rotation(locked_rotation)
        desired = self.limit_mode.clamp_target_position(
            desired, self._angles)
        return self._solve_desired_frame(desired)

    def command_pose(self, position, rotation, drive=True):
        """Command a full Cartesian target pose.

        Args:
            position: Target position ``[x, y, z]`` in inches.
            rotation: 3x3 target rotation, rows in world coordinates.
            drive: When True, step the mechanical arm immediately.  The
                planner passes False because its control cycle drives the
                target right after this call.

        Returns:
            bool: True when the target was accepted.
        """
        if not self._require_initialized():
            return False
        if self.homing.active or self.is_flipping or self.zenith \
                or self.resting:
            return self._fail("Command blocked: action is running")
        if len(position) != 3:
            return self._fail("Command blocked: invalid position")

        desired = OrientationMatrix([
            [rotation[0][0], rotation[0][1], rotation[0][2], position[0]],
            [rotation[1][0], rotation[1][1], rotation[1][2], position[1]],
            [rotation[2][0], rotation[2][1], rotation[2][2], position[2]],
            [0.0, 0.0, 0.0, 1.0],
        ])
        desired = self.limit_mode.clamp_target_position(
            desired, self.target_angles)
        return self._solve_desired_frame(desired, drive=drive)

    def hold(self):
        """Drop the current target and hold the measured pose."""
        if not self.initialized:
            return False
        if self.zenith:
            # Zenith is a mode, not a target; a handover hold leaves it be.
            return True
        if self.resting:
            # Resting owns its target; a handover hold leaves it be.
            return True
        if self.homing.active or self.is_flipping:
            return self._fail("Hold blocked: action is running")
        # The teleop stage restarts from the measured pose on the next stick
        # input rather than resuming a trajectory from before the hold.
        self._teleop_needs_reset = True
        self._sync_target_to_fk()
        return True

    def activate_flip(self):
        """Start the single side-flip action.

        The shoulder mirrors about 90° and the elbow about 0°, so the
        arm flips to the opposite side instead of inverting the elbow.
        The mechanical arm then slews to that target under its normal
        velocity and limit checks.

        Flips are allowed directly in OCTANT_DIRECT_FLIP (the even
        octants).  In the other octants the planar modifier must be
        active.

        Returns:
            bool: True if a flip started, False if refused or already running.
        """
        if not self._require_initialized():
            return False
        if self.zenith:
            return self._fail("Flip blocked: exit zenith mode first")
        if self.resting:
            return self._fail("Flip blocked: resting is active")
        octant = self._ee_octant()
        if octant not in OCTANT_DIRECT_FLIP and not self.planar:
            return self._fail(
                f"Flip blocked: octant {octant} requires planar")

        limits = self.flip_side_limits()
        assert self.l0 is not None
        assert self.l1 is not None
        assert self.l2 is not None
        started = self.flipper.activate(
            self._angles, self.l0, self.l1, self.l2,
            joint_limits=limits, side=True)
        if not started:
            return self._fail("Flip blocked: another flip is running")
        if started:
            held = list(self._angles)
            target = self._hold_joints(self.flipper.target_angles, held)
            if any(self.locked_joints[:3]):
                path = [held, target]
            else:
                path = plan_safe_path(self.model, held, target)
            if path is None:
                self.flipper.deactivate()
                return self._fail(
                    "Flip blocked: no safe path through restricted space")

            waypoints = path[1:] if len(path) > 1 else [target]
            self._follow_path = [self._hold_joints(p, held)
                                 for p in waypoints]
            self._follow_path_idx = 0
            self._follow_stall = 0
            self._follow_target = list(self._follow_path[0])
            self._follow_limits = limits
            self._follow_switch_mode = self.limit_mode in (
                PICK_AND_PLACE_MODE, PICK_AND_PLACE_FLIPPED_MODE)
            self._follow_max_step = self._synchronized_steps(
                held, self._follow_target)
        return started

    def flip_side_limits(self):
        """Mechanical limits used during a side-flip animation.

        The transition between Pick-and-Place and Pick-and-Place-Flipped
        passes through angles outside both modes' workspaces, so
        mechanical limits are used for the animation.
        """
        return MECHANICAL_LIMITS

    # ------------------------------------------------------------------
    # Update paths (one per kinematic mode)
    # ------------------------------------------------------------------

    def update(self, deltas=None, feedback_fresh=True, resting_enabled=True):
        """Advance one control cycle based on the current mode.

        Dispatches to the appropriate update path:
            1. Flip follow  -> slew to the flip target (overrides all)
            2. Resting      -> collapse / de-torque / restore state machine
            3. Homing       -> step toward home configuration
            4. No input     -> finish the last position-drive target
            5. Joint drive  -> direct joint deltas + clamping
            6. Position drive -> target frame delta + IK solve

        The mechanical arm owns the actual angles in every path; this
        method only commands it, so a bad target is velocity-limited
        rather than applied as a jump.

        Args:
            deltas: dict with keys dx, dy, dz, drx, dry, drz
                    (scaled SpaceMouse deltas), or None when idle.
            feedback_fresh: False when no fresh measured sample arrived
                    (used by resting push abort checks).
            resting_enabled: False when the arm is disabled (used by
                    resting push abort checks).

        Returns:
            bool: True if input was consumed (state was updated).
        """
        if not self.initialized:
            return False
        # Flip motion overrides everything until the target is reached
        if self._follow_target is not None:
            held = list(self._angles)
            target = self._hold_joints(self._follow_target, held)
            limits = self._locked_limits(self._follow_limits)
            before = self._target_error(target)
            arrived = self._drive_mechanical(
                target, limits, max_step=self._follow_max_step,
                tol=WAYPOINT_TOL)
            # A long but healthy segment must not be mistaken for a stall.
            # Reset whenever the arm is still closing on the waypoint and
            # only count cycles that make no progress at all.
            if arrived or self._target_error(target) < before - 1e-9:
                self._follow_stall = 0
            else:
                self._follow_stall += 1

            if arrived or self._follow_stall > WAYPOINT_STALL_LIMIT:
                self._follow_stall = 0
                if (self._follow_path
                        and self._follow_path_idx + 1 < len(self._follow_path)):
                    self._follow_path_idx += 1
                    held = list(self._angles)
                    self._follow_target = self._hold_joints(
                        self._follow_path[self._follow_path_idx], held)
                    self._follow_max_step = self._synchronized_steps(
                        held, self._follow_target)
                else:
                    self.flipper.deactivate()
                    if self._follow_switch_mode:
                        self._switch_mode_after_side_flip()
                    self._follow_target = None
                    self._follow_path = None
                    self._follow_limits = None
                    self._follow_switch_mode = False
                    self._follow_max_step = None

            self.target_angles = list(self._angles)
            self.refresh_fk()
            self._sync_target_to_fk()
            return True

        # Resting owns every joint until the restore sequence completes;
        # teleop input is ignored while it is active.
        if self.resting:
            self._update_resting(
                feedback_fresh=feedback_fresh,
                enabled=resting_enabled)
            self.refresh_fk()
            return True

        # Zenith is a static test pose; teleop input is ignored until exit.
        if self.zenith:
            self._update_zenith()
            self.refresh_fk()
            return True

        # Homing runs even without new input
        if self.homing.active:
            self._update_homing()
            self.refresh_fk()
            return True

        # No new input: keep the mechanical arm moving toward the last
        # position-drive target until it arrives.
        if deltas is None:
            if self.joint_drive:
                return False
            self._finish_position_target()
            self.refresh_fk()
            return True

        # Dispatch to active mode
        if self.joint_drive:
            self._update_joint_drive(deltas)
        else:
            self._update_position_drive(deltas)

        self.refresh_fk()
        return True

    def _switch_mode_after_side_flip(self):
        """Update the flipped flag from the pose reached by a side flip.

        Planar, if active, stays on.  Its valid octants are disjoint per
        side, so use the octant rather than the P&P joint limits, which
        can both reject a planar flip destination.
        """
        if self.planar:
            self.flipped = self._ee_octant() in OCTANT_PLANAR_FLIPPED_ALLOWED
            return
        if PICK_AND_PLACE_FLIPPED_MODE.can_set(self._angles):
            self.flipped = True
        elif PICK_AND_PLACE_MODE.can_set(self._angles):
            self.flipped = False

    def _update_homing(self):
        """Step each allowed joint toward the home configuration.

        Homing runs under HOMING_MODE limits so it can never drive a
        joint outside hardware bounds, regardless of the active mode.
        Locked joints are held in place; when ``home_joint_idx`` is set,
        only that joint is driven and every other joint is held.

        The elbow deadband (arm_constraints) prevents small steps from
        crossing zero.  When the homing target is on the opposite side
        of the deadband, the commanded elbow is held on the target-side
        boundary until the mechanical arm actually crosses it.
        """
        held = list(self._angles)
        was_single = self.home_joint_idx is not None
        mask = self._homing_mask()

        if self._homing_path:
            target = self._hold_joints(
                self._homing_path[self._homing_path_idx], held, mask)
            limits = self._locked_limits(HOMING_MODE.limits)
            before = self._target_error(target, mask)
            arrived = self._drive_mechanical(
                target, limits, mask=mask,
                max_step=self._homing_seg_steps, tol=WAYPOINT_TOL)
            # Count only cycles with no progress.  Legitimate segments can
            # span many control frames (a 170 deg move takes ~4 s at the
            # mechanical slew rate), and treating those as stalls discards
            # the safe path and can drive the arm through a restricted
            # octant.
            if arrived or self._target_error(target, mask) < before - 1e-9:
                self._homing_stall = 0
            else:
                self._homing_stall += 1

            if self._homing_stall > WAYPOINT_STALL_LIMIT:
                # The arm cannot close the remaining fraction of this
                # waypoint (lag, stiction, a hardware limit).  Drop the
                # pre-planned path and let the direct homing step below
                # finish the action instead of hanging forever.
                self._homing_path = None
                self._homing_seg_steps = None
                self._homing_stall = 0
            elif arrived:
                self._homing_stall = 0
                self._homing_path_idx += 1
                if self._homing_path_idx >= len(self._homing_path):
                    self.homing.deactivate()
                    self._homing_path = None
                    self._homing_seg_steps = None
                    self.home_joint_idx = None
                    self.flipped = self._homing_flipped
                    self._homing_cross_dir = 0
                    self._sync_target_to_fk()
                    return
                nxt = self._hold_joints(
                    self._homing_path[self._homing_path_idx],
                    self._angles, mask)
                self._homing_seg_steps = self._synchronized_steps(
                    self._angles, nxt)

            if self._homing_path is not None:
                self._sync_target_to_fk()
                return

        commanded, done = self.homing.step(
            self._angles, limits=HOMING_MODE.limits, mask=mask)
        commanded = HOMING_MODE.geometry_constraint(commanded)
        commanded = self._hold_joints(commanded, held, mask)

        # Deadband crossing: the elbow cannot step through the forbidden
        # band.  Once it is pinned at a boundary with the target on the
        # other side, keep commanding the opposite boundary until the
        # mechanical arm actually crosses it.
        if mask[2]:
            tgt_elbow = self.homing.target[2]
            if self._homing_cross_dir == 0:
                if (abs(commanded[2] - held[2]) < 1e-12
                        and abs(tgt_elbow) > ELBOW_DEADBAND
                        and (held[2] > 0) != (tgt_elbow > 0)):
                    self._homing_cross_dir = 1 if tgt_elbow > 0 else -1

            if self._homing_cross_dir != 0:
                commanded[2] = self._homing_cross_dir * ELBOW_DEADBAND
                crossed = (held[2] <= -ELBOW_DEADBAND
                           if self._homing_cross_dir < 0
                           else held[2] >= ELBOW_DEADBAND)
                if crossed:
                    self._homing_cross_dir = 0

        limits = self._locked_limits(HOMING_MODE.limits)
        self._command_mechanical(commanded, limits)

        if done:
            if self._all_at(self.homing.target, mask=mask):
                if not was_single:
                    self.flipped = self._homing_flipped
                self.home_joint_idx = None
                self._homing_cross_dir = 0
            else:
                # HomingController finished based on its commanded step,
                # but the mechanical arm is still catching up.
                self.homing.activate()

        self._sync_target_to_fk()

    def _update_joint_drive(self, deltas):
        """Direct joint control: apply deltas to joints.

        Mapping:
            Rx -> elbow (t2), Ry -> shoulder (t1), Rz -> base (t0).

        Locked joints ignore their delta.  After applying deltas, joints
        are clamped to the current limiter's effective bounds, cylindrical
        avoidance is enforced if the limiter requires it, and the geometry
        constraint is applied.  Locked angles are then restored.  The
        target frame tracks the arm.
        """
        held = list(self._angles)
        commanded = list(self._angles)
        if not self.locked_joints[2]:
            commanded[2] += deltas.get("drx", 0.0)  # elbow
        if not self.locked_joints[1]:
            commanded[1] += deltas.get("dry", 0.0)  # shoulder
        if not self.locked_joints[0]:
            commanded[0] -= deltas.get("drz", 0.0)  # base

        limits = self._active_limits(commanded)
        commanded = [max(lo, min(hi, a))
                     for a, (lo, hi) in zip(commanded, limits)]

        commanded = self.limit_mode.geometry_constraint(commanded)
        commanded = self._hold_joints(commanded, held)
        if self._workspace_blocked(commanded):
            return

        limits = self._active_limits(commanded)
        self._command_mechanical(commanded, self._locked_limits(limits))
        self._sync_target_to_fk()

    def _finish_position_target(self):
        """Keep slewing toward the last position-drive target while idle.

        The target frame stays on the commanded pose so the marker shows
        where the arm is heading; the mechanical arm catches up at its
        velocity limit.
        """
        held = list(self._angles)
        target = self._hold_joints(self.target_angles, held)

        if any(self.locked_joints):
            self.target_angles = list(target)
            frame = self.model.solve_forward_kinematics(self.target_angles)
            self.target_frame = self._frame_with_orientation_lock(frame)
            self.target_pos = list(self.target_frame.position())

        limits = self._active_limits(self.target_angles)
        self._command_mechanical(self.target_angles,
                                 self._locked_limits(limits))

    def _solve_desired_frame(self, desired, drive=True):
        """Run IK toward a desired frame and command the mechanical arm."""
        # Locked joints are pinned to a zero-width limit so the solver
        # distributes the motion across the remaining motors.
        locked_angles = [wrap2pi(a) for a in self._angles]
        limits = self._active_limits(self.target_angles)
        for i, locked in enumerate(self.locked_joints):
            if locked and i < len(limits):
                limits[i] = (locked_angles[i], locked_angles[i])
        self.solver.joint_limits = limits
        new_target, _, _, _ = self.solver.solve(
            self.target_angles, desired,
            start_pos_tol=TRACK_POS_TOL, start_ori_tol=TRACK_ORI_TOL)
        self.solver_history = list(self.solver.history)

        # Re-impose lockouts after the solve (geometry constraints run
        # inside the solver and could otherwise nudge a locked joint).
        for i, locked in enumerate(self.locked_joints):
            if locked and i < len(new_target):
                new_target[i] = locked_angles[i]

        if self._workspace_blocked(new_target):
            return False

        self.target_angles = new_target
        frame = self.model.solve_forward_kinematics(self.target_angles)
        self.target_frame = self._frame_with_orientation_lock(frame)
        self.target_pos = list(self.target_frame.position())
        if drive:
            self._command_mechanical(self.target_angles,
                                     self._locked_limits(limits))
        return True

    def guide_orientation(self):
        """Snap the target orientation to the nearest exact axis pose.

        This is an explicit command, not automatic.  It only acts when
        every n/o/a column passes the strict alignment thresholds, and
        the target position is preserved.

        Returns:
            bool: True if the orientation was snapped.
        """
        if not self._require_initialized():
            return False
        if self.orientation_locked:
            return self._fail("Guide blocked: orientation is locked")

        snapped = nearest_axis_aligned_rotation(
            self.target_frame.rotation_matrix(),
            ORIENTATION_SNAP_THRESHOLD, ORIENTATION_SNAP_OFF_AXIS)
        if snapped is None:
            return self._fail(
                "Guide unavailable: orientation is not aligned enough")

        desired = self.target_frame.with_rotation(snapped)
        desired = self.limit_mode.clamp_target_position(
            desired, self.target_angles)
        return self._solve_desired_frame(desired)

    def _update_position_drive(self, deltas):
        """Position drive: move target, IK solves, frame is recomputed.

        Position and orientation locks zero their respective deltas before
        the target is updated.  The target orientation is never snapped
        automatically; use guide_orientation() for that.
        """
        if self.position_locked:
            for axis in ("dx", "dy", "dz"):
                deltas[axis] = 0.0
        if self.orientation_locked:
            for axis in ("drx", "dry", "drz"):
                deltas[axis] = 0.0

        for axis in ("dx", "dy", "dz"):
            d = deltas.get(axis, 0.0)
            deltas[axis] = max(-MAX_TRANSLATION_DELTA,
                               min(MAX_TRANSLATION_DELTA, d))

        desired = self.target_frame.apply_delta(deltas)
        locked_rotation = self._locked_orientation()
        if locked_rotation is not None:
            desired = desired.with_rotation(locked_rotation)
        desired = self.limit_mode.clamp_target_position(
            desired, self.target_angles)
        self._solve_desired_frame(desired)

    # ------------------------------------------------------------------
    # Teleop output stage
    # ------------------------------------------------------------------

    def invalidate_teleop_reference(self):
        """Drop the teleop reference on the next cycle.

        Called when another path takes over (homing, flip, pose, joint
        drive), on handover, and on E-stop.  The next teleop cycle restarts
        from the measured angles instead of resuming a stale trajectory.
        """
        self._teleop_needs_reset = True
        self._teleop_stalled = False

    @property
    def teleop_motion_active(self) -> bool:
        """True while the teleop stage still has speed to spend."""
        if self.teleop_reference is None:
            return False
        if any(abs(v) > TELEOP_STOP_EPSILON for v in self.teleop_velocity):
            return True
        return any(abs(v) > TELEOP_STOP_EPSILON
                   for v in self.teleop_joint_velocity)

    def reset_teleop_reference(self, angles):
        """Snap the teleop reference to *angles* and clear its velocities."""
        if not self.initialized or angles is None:
            return False
        angles = [wrap2pi(float(a)) for a in angles]
        if len(angles) != len(self.mechanical.angles):
            return False
        self.teleop_reference = list(angles)
        self.teleop_velocity = [0.0] * 6
        self.teleop_joint_velocity = [0.0] * len(angles)
        self.teleop_lead_cycles = 0
        self._teleop_needs_reset = False
        self._teleop_stalled = False
        self.mechanical.angles = list(angles)
        self.mechanical.joint_speeds = [0.0] * len(angles)
        self._last_commanded = list(angles)
        self.target_angles = list(angles)
        frame = self.model.solve_forward_kinematics(self.target_angles)
        self.target_frame = self._frame_with_orientation_lock(frame)
        self.target_pos = list(self.target_frame.position())
        self.refresh_fk()
        return True

    @staticmethod
    def _ramp_component(value, target, accel, decel, dt):
        """Move one velocity component toward *target* under a ramp."""
        if target == value:
            return value
        slowing = abs(target) < abs(value) or target * value < 0.0
        limit = decel if slowing else accel
        step = limit * dt
        delta = target - value
        if abs(delta) <= step:
            return target
        return value + math.copysign(step, delta)

    def update_teleop(self, velocity, measured, dt=DT):
        """Run one teleop cycle against an internal joint reference.

        The reference is advanced by a coordinated, rate-limited joint
        velocity instead of being rebuilt from the measured angles.  That
        keeps the Cartesian path straight, avoids injecting feedback noise
        into the command, and stops the IK target from running away from
        the arm.

        Args:
            velocity: 6-tuple ``(vx, vy, vz [in/s], wx, wy, wz [rad/s])``.
            measured: Measured joint angles [rad].
            dt: Control period [s].

        Returns:
            bool: True while teleop motion (including deceleration) is
            active; False once the stage is idle or a stall is reported.
        """
        if not self.initialized:
            return False
        if measured is None or len(measured) != len(self.mechanical.angles):
            return False
        if any(math.isnan(float(a)) for a in measured):
            return False

        target = [float(v) for v in velocity]
        if len(target) != 6:
            return False
        moving_target = any(abs(v) > TELEOP_STOP_EPSILON for v in target)

        if self._teleop_stalled:
            # Wait for the stick to return to neutral before trying again.
            if not moving_target:
                self._teleop_stalled = False
                self._teleop_needs_reset = True
            return False
        if self._teleop_needs_reset:
            # Idle: keep the last reference and do not adopt measured noise.
            # A new push re-arms from the measured pose.
            if not moving_target:
                return False
            if not self.reset_teleop_reference(measured):
                return False
        elif self.teleop_reference is None:
            if not self.reset_teleop_reference(measured):
                return False

        if self.position_locked:
            target[0:3] = [0.0, 0.0, 0.0]
        if self.orientation_locked:
            target[3:6] = [0.0, 0.0, 0.0]

        linear = (TELEOP_ACCEL_IN_S2, TELEOP_ACCEL_IN_S2, TELEOP_ACCEL_IN_S2)
        decel = (TELEOP_DECEL_IN_S2, TELEOP_DECEL_IN_S2, TELEOP_DECEL_IN_S2)
        angular = (TELEOP_ANG_ACCEL_RAD_S2,) * 3
        accel_all = linear + angular
        decel_all = decel + angular
        self.teleop_velocity = [
            self._ramp_component(v, t, accel_all[i], decel_all[i], dt)
            for i, (v, t) in enumerate(zip(self.teleop_velocity, target))
        ]
        if (all(abs(v) <= TELEOP_STOP_EPSILON
                for v in self.teleop_velocity)
                and all(abs(v) <= TELEOP_STOP_EPSILON
                        for v in self.teleop_joint_velocity)):
            self.teleop_velocity = [0.0] * 6
            self.teleop_joint_velocity = [0.0] * len(self.teleop_joint_velocity)
            return False

        reference = list(self.teleop_reference)
        frame = self.model.solve_forward_kinematics(reference)
        frame = self._frame_with_orientation_lock(frame)
        desired = frame.apply_delta({
            "dx": self.teleop_velocity[0] * dt,
            "dy": self.teleop_velocity[1] * dt,
            "dz": self.teleop_velocity[2] * dt,
            "drx": self.teleop_velocity[3] * dt,
            "dry": self.teleop_velocity[4] * dt,
            "drz": self.teleop_velocity[5] * dt,
        })
        desired = self.limit_mode.clamp_target_position(desired, reference)

        limits = [tuple(pair) for pair in self._active_limits(reference)]
        for i, locked in enumerate(self.locked_joints):
            if locked and i < len(limits):
                limits[i] = (reference[i], reference[i])

        self.solver.joint_limits = limits
        ik_target, _, _, _ = self.solver.solve(
            reference, desired,
            start_pos_tol=TRACK_POS_TOL, start_ori_tol=TRACK_ORI_TOL)
        for i, locked in enumerate(self.locked_joints):
            if locked and i < len(ik_target):
                ik_target[i] = reference[i]

        if self._workspace_blocked(ik_target):
            self.teleop_velocity = [0.0] * 6
            self.teleop_joint_velocity = [0.0] * len(self.teleop_joint_velocity)
            return False

        diff = [wrap2pi(t - r) for t, r in zip(ik_target, reference)]
        caps = self.mechanical.max_ang_vel
        desired_speed = [0.0] * len(reference)
        scale = 1.0
        for i, delta in enumerate(diff):
            if self.locked_joints[i] or abs(delta) < 1e-12:
                continue
            speed = delta / dt
            desired_speed[i] = speed
            cap = caps[i]
            if abs(speed) > cap:
                scale = min(scale, cap / abs(speed))
        desired_speed = [speed * scale for speed in desired_speed]

        accel_speed = [cap / TELEOP_JOINT_ACCEL_TIME_S for cap in caps]
        change = [want - have for want, have
                  in zip(desired_speed, self.teleop_joint_velocity)]
        ratio = 1.0
        for i, value in enumerate(change):
            if self.locked_joints[i]:
                continue
            limit = accel_speed[i] * dt
            if abs(value) > limit:
                ratio = max(ratio, abs(value) / limit)
        self.teleop_joint_velocity = [
            have + value / ratio
            for have, value in zip(self.teleop_joint_velocity, change)
        ]
        # A joint locked mid-motion must not keep a residual velocity; its
        # limit is already zero-width, so pin the state too.
        for i, locked in enumerate(self.locked_joints):
            if locked:
                self.teleop_joint_velocity[i] = 0.0

        new_reference = [
            ref + speed * dt
            for ref, speed in zip(reference, self.teleop_joint_velocity)
        ]
        for i, (lo, hi) in enumerate(limits):
            new_reference[i] = max(lo, min(hi, new_reference[i]))

        lead = math.radians(TELEOP_LEAD_LIMIT_DEG)
        lead_active = False
        for i, angle in enumerate(new_reference):
            if self.locked_joints[i]:
                continue
            lo = float(measured[i]) - lead
            hi = float(measured[i]) + lead
            if angle < lo or angle > hi:
                new_reference[i] = max(lo, min(hi, angle))
                lead_active = True
        if lead_active:
            self.teleop_lead_cycles += 1
        else:
            self.teleop_lead_cycles = 0
        if self.teleop_lead_cycles > TELEOP_LEAD_STALL_CYCLES:
            self.teleop_velocity = [0.0] * 6
            self.teleop_joint_velocity = [0.0] * len(self.teleop_joint_velocity)
            self._teleop_stalled = True
            self._fail(
                "Teleop tracking stall: arm cannot follow the reference")
            return False

        self.teleop_reference = new_reference
        self.mechanical.angles = list(new_reference)
        self.mechanical.joint_speeds = [
            abs(v) for v in self.teleop_joint_velocity]
        self.target_angles = list(new_reference)
        frame = self.model.solve_forward_kinematics(self.target_angles)
        self.target_frame = self._frame_with_orientation_lock(frame)
        self.target_pos = list(self.target_frame.position())
        self.refresh_fk()
        return True

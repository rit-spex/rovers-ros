from __future__ import annotations

from itertools import permutations, product

import sympy as sp
import numpy as np
import math
import time

from arm_constants import (
    POS_TOL, ORI_TOL, MAX_STEP,
    FD_EPS, DAMPING_LAMBDA_SQ, LIMIT_TOL,
    MAX_BACKTRACK, BACKTRACK_TOL, MAX_ACTIVE_SET_PASSES,
)


class FrameManager:
    """Frame math, residual helpers, and orientation snapping."""

    @staticmethod
    def wrap2pi(angle):
        """Wrap an angle to [-pi, pi]."""
        return (angle + math.pi) % (2 * math.pi) - math.pi

    @staticmethod
    def limit_respecting_diff(current, diff, lo, hi):
        """Choose the signed travel that stays inside ``[lo, hi]``.

        ``diff`` is the shortest angular difference (``wrap2pi(target -
        current)``).  A joint limit can block that shortest path: asking
        the elbow to travel from +135 deg to -130 deg gives +95 deg, which
        would run into the +135 deg stop.  Stepping the other way once
        and then re-picking the shortest path next cycle makes the joint
        oscillate on the boundary forever.  This helper checks the whole
        path up front and returns the opposite direction (the long way
        around) when only that path stays inside the limits.

        Args:
            current: Current joint angle [rad].
            diff: Shortest signed angular distance to the target [rad].
            lo: Lower joint bound [rad]; may be -inf.
            hi: Upper joint bound [rad]; may be +inf.

        Returns:
            float: ``diff`` or the opposite-direction equivalent
            (``diff -/+ 2*pi``) that stays inside the bounds.
        """
        if abs(diff) <= 1e-12:
            return diff
        eps = 1e-9
        end = current + diff
        if not math.isfinite(lo) or not math.isfinite(hi):
            return diff
        if lo - eps <= end <= hi + eps:
            return diff
        alternate = diff - math.copysign(2 * math.pi, diff)
        alternate_end = current + alternate
        if lo - eps <= alternate_end <= hi + eps:
            return alternate
        return diff


    # =============================================================================
    # Axis-aligned orientation helpers
    # =============================================================================

    @staticmethod
    def _build_axis_aligned_rotations():
        """Build the 24 proper signed-permutation rotation matrices."""
        rotations = []
        for perm in permutations(range(3)):
            for signs in product((-1.0, 1.0), repeat=3):
                R = np.zeros((3, 3))
                for col, row in enumerate(perm):
                    R[row, col] = signs[col]
                if np.linalg.det(R) > 0.0:
                    rotations.append(R)
        return rotations

    @staticmethod
    def alignment_score(R) -> float:
        """How closely a rotation matches its nearest axis-aligned rotation.

        The score is max_S trace(S^T R) over the 24 proper signed-permutation
        rotations S.  It equals 3 when R is exactly axis-aligned and decreases
        as R rotates away, so comparing scores tells whether an update moves
        toward or away from alignment.
        """
        R = np.asarray(R, dtype=float)
        return max(float(np.sum(R * S)) for S in _AXIS_ALIGNED_ROTATIONS)


    @staticmethod
    def nearest_axis_aligned_rotation(R, component_threshold=0.95,
                                     off_axis_threshold=0.25):
        """Closest proper axis-aligned rotation, if every axis is close enough.

        A rotation qualifies when each of its three columns (n, o, a) has
        one component with ``abs(value) > component_threshold`` while both
        remaining components have ``abs(value) < off_axis_threshold``.

        Args:
            R: 3x3 rotation matrix.
            component_threshold: Dominant component magnitude, e.g. 0.95.
            off_axis_threshold: Maximum magnitude for the other components.

        Returns:
            np.ndarray | None: The matching 3x3 rotation, or None when any
            column fails the alignment test.
        """
        R = np.asarray(R, dtype=float)
        best = None
        best_score = -np.inf
        for S in _AXIS_ALIGNED_ROTATIONS:
            score = float(np.sum(R * S))
            if score > best_score:
                best_score = score
                best = S

        if best is None:
            return None

        rows = np.argmax(np.abs(best), axis=0)
        for col in range(3):
            magnitudes = np.abs(R[:, col])
            row = rows[col]
            if magnitudes[row] <= component_threshold:
                return None
            off_axis = np.delete(magnitudes, row)
            if not np.all(off_axis < off_axis_threshold):
                return None

        return best.copy()


    @staticmethod
    def _axis_angle_error(R_target, R_current) -> np.ndarray:
        """World-frame rotation that carries R_current onto R_target.

        Returns phi * k, where R_target @ R_current.T = exp([phi * k]x), so
        the vector's norm is the rotation angle in radians — directly
        comparable to ORI_TOL.
        """
        R_err = np.asarray(R_target, dtype=float) @ np.asarray(R_current, dtype=float).T
        cos_phi = float(np.clip((np.trace(R_err) - 1.0) * 0.5, -1.0, 1.0))
        phi = math.acos(cos_phi)
        if phi < 1e-6:
            return np.zeros(3)

        sin_phi = math.sin(phi)
        if sin_phi > 1e-6:
            vee = np.array([R_err[2, 1] - R_err[1, 2],
                            R_err[0, 2] - R_err[2, 0],
                            R_err[1, 0] - R_err[0, 1]])
            return (0.5 * phi / sin_phi) * vee

        # phi ~ pi: R_err is symmetric, the skew part vanishes and the axis
        # must come off the diagonal.  k and -k describe the same rotation
        # at pi, so the (positive) sign is arbitrary.
        k = np.sqrt(np.clip((np.diag(R_err) - cos_phi) / (1.0 - cos_phi), 0.0, 1.0))
        norm = float(np.linalg.norm(k))
        return np.zeros(3) if norm < 1e-12 else phi * (k / norm)


    @staticmethod
    def _frame_residual(fk_flat, target_matrix) -> np.ndarray:
        """6-DOF geometric error [dx, dy, dz, wx, wy, wz] toward a target.

        Position is target - current in inches.  Orientation is the
        world-frame axis-angle vector that rotates the current frame onto
        the target, in radians.
        """
        fk = np.asarray(fk_flat, dtype=float).reshape(4, 4)
        tgt = np.array(target_matrix.tolist(), dtype=float)
        e_pos = tgt[:3, 3] - fk[:3, 3]
        e_rot = _axis_angle_error(tgt[:3, :3], fk[:3, :3])
        return np.concatenate((e_pos, e_rot))


    @staticmethod
    def _residual_to_errors(e) -> tuple[float, float]:
        """(position error [in], orientation error [rad]) from a residual.

        These are the *reported* metrics, checked against POS_TOL/ORI_TOL.
        """
        return float(np.max(np.abs(e[:3]))), float(np.linalg.norm(e[3:]))


    @staticmethod
    def _residual_norm(e) -> float:
        """Scalar objective the Gauss-Newton step actually minimises.

        Deliberately not pos_error + ori_error: that sum is a min-max measure
        while the step is a least-squares one, so a perfectly good step can
        raise the worst position component and get rejected.  Driving the
        2-norm down is what pulls the reported errors down with it, and the
        position block (inches) dominates the orientation block (radians),
        which matches POS_TOL being the tolerance that matters most.
        """
        return float(np.linalg.norm(e))

_AXIS_ALIGNED_ROTATIONS = FrameManager._build_axis_aligned_rotations()

# Compatibility aliases for callers and tests.
wrap2pi = FrameManager.wrap2pi
limit_respecting_diff = FrameManager.limit_respecting_diff
alignment_score = FrameManager.alignment_score
nearest_axis_aligned_rotation = FrameManager.nearest_axis_aligned_rotation
_axis_angle_error = FrameManager._axis_angle_error
_frame_residual = FrameManager._frame_residual
_residual_to_errors = FrameManager._residual_to_errors
_residual_norm = FrameManager._residual_norm


def rotation_from_rpy(roll: float, pitch: float, yaw: float) -> list:
    """Return ``Rz(yaw) @ Ry(pitch) @ Rx(roll)`` as a 3x3 row list."""
    cr, sr = math.cos(roll), math.sin(roll)
    cp, sp = math.cos(pitch), math.sin(pitch)
    cy, sy = math.cos(yaw), math.sin(yaw)
    return [
        [cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
        [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
        [-sp, cp * sr, cp * cr],
    ]


def rotation_to_rpy(rotation) -> tuple[float, float, float]:
    """Return fixed-axis (roll, pitch, yaw) for ``Rz @ Ry @ Rx``.

    At gimbal lock (pitch +/-90 deg) roll is pinned to zero and the
    combined rotation is reported as yaw.
    """
    r20 = max(-1.0, min(1.0, float(rotation[2][0])))
    pitch = math.asin(-r20)
    if abs(r20) < 0.999999:
        roll = math.atan2(float(rotation[2][1]), float(rotation[2][2]))
        yaw = math.atan2(float(rotation[1][0]), float(rotation[0][0]))
    else:
        roll = 0.0
        yaw = math.atan2(-float(rotation[0][1]), float(rotation[1][1]))
    return roll, pitch, yaw


class OrientationMatrix(sp.Matrix):
    def __new__(cls, matrix):
        if isinstance(matrix[0], list):
            return sp.Matrix.__new__(cls, matrix)
        elif len(matrix) == 16:
            return sp.Matrix.__new__(cls, 4, 4, [float(v) for v in matrix])
        else:
            raise ValueError(f"Expected list-of-lists or flat list of 16, got length {len(matrix)}")

    def position(self) -> list[float]:
        """Extract [x, y, z] from the translation column."""
        return [float(self[0, 3]), float(self[1, 3]), float(self[2, 3])]

    def set_position(self, pos) -> None:
        """Set the translation column from a list or array [x, y, z]."""
        self[0, 3] = pos[0]
        self[1, 3] = pos[1]
        self[2, 3] = pos[2]

    def with_rotation(self, rotation) -> OrientationMatrix:
        """Return a copy of this frame with a new 3x3 rotation block."""
        R = np.asarray(rotation, dtype=float)
        pos = self.position()
        return OrientationMatrix([
            [R[0, 0], R[0, 1], R[0, 2], pos[0]],
            [R[1, 0], R[1, 1], R[1, 2], pos[1]],
            [R[2, 0], R[2, 1], R[2, 2], pos[2]],
            [0.0, 0.0, 0.0, 1.0],
        ])

    def rotation_matrix(self) -> np.ndarray:
        """Extract the 3x3 rotation submatrix."""
        return np.array(self.tolist()[:3], dtype=float)[:, :3]

    def print_matrix(self, label: str) -> None:
        """Pretty-print a 4x4 sympy Matrix with aligned columns."""
        arr: np.ndarray = np.array(self.tolist(), dtype=float)
        print(f"  {label}:")
        for row in arr:
            print("    [" + "  ".join(f"{v:12.6f}" for v in row) + "]")

    def apply_delta(self, deltas: dict) -> OrientationMatrix:
        """Return a new frame with translation and rotation applied.

        Translations are in world frame.  Rotations are about the
        frame's own position (conjugated so the pivot is the origin,
        not the world).

        Args:
            deltas: dict with keys dx, dy, dz, drx, dry, drz.
        """
        import sympy as _sp

        dx, dy, dz = deltas["dx"], deltas["dy"], deltas["dz"]
        drx, dry, drz = deltas["drx"], deltas["dry"], deltas["drz"]

        transT = OrientationMatrix([
            [1, 0, 0, dx],
            [0, 1, 0, dy],
            [0, 0, 1, dz],
            [0, 0, 0, 1],
        ])

        transX = OrientationMatrix([
            [1, 0, 0, 0],
            [0, _sp.cos(drx), -_sp.sin(drx), 0],
            [0, _sp.sin(drx),  _sp.cos(drx), 0],
            [0, 0, 0, 1],
        ])
        transY = OrientationMatrix([
            [ _sp.cos(dry), 0, _sp.sin(dry), 0],
            [0, 1, 0, 0],
            [-_sp.sin(dry), 0, _sp.cos(dry), 0],
            [0, 0, 0, 1],
        ])
        transZ = OrientationMatrix([
            [_sp.cos(drz), -_sp.sin(drz), 0, 0],
            [_sp.sin(drz),  _sp.cos(drz), 0, 0],
            [0, 0, 1, 0],
            [0, 0, 0, 1],
        ])

        R_delta = transZ * transY * transX

        pos = self.position()
        T_pos = OrientationMatrix([
            [1, 0, 0, pos[0]],
            [0, 1, 0, pos[1]],
            [0, 0, 1, pos[2]],
            [0, 0, 0, 1],
        ])
        T_pos_inv = OrientationMatrix([
            [1, 0, 0, -pos[0]],
            [0, 1, 0, -pos[1]],
            [0, 0, 1, -pos[2]],
            [0, 0, 0, 1],
        ])

        T_delta = transT * T_pos * R_delta * T_pos_inv
        return T_delta * self



class IterationResults:
    """Stores the result of a single solver iteration.

    Captures the state before and after one Newton step, along with
    timing and error metrics computed against the target frame.
    """

    def __init__(self) -> None:
        # Frames (4x4 sympy Matrices)
        self.start_frame:    OrientationMatrix | None = None      # FK at the start of this iteration
        self.target_frame:   OrientationMatrix | None = None     # Desired pose
        self.computed_frame: OrientationMatrix | None = None   # FK after the solve step

        # Joint angles (list of floats, radians)
        self.start_angles:    list[float] | None = None     # Angles before this iteration
        self.computed_angles: list[float] | None = None  # Angles after this iteration

        # Per-joint position and orientation data (9 joints along the chain)
        self.start_joint_positions: list | None = None   # [[x,y,z], ...] for each joint
        self.start_joint_frames:    list | None = None   # [OrientationMatrix, ...] for each joint
        self.computed_joint_positions: list | None = None
        self.computed_joint_frames:    list | None = None

        # Timing
        self.solve_time: float = 0.0  # Cumulative wall-clock time up to this iteration

        # Pre-step errors (start_frame vs target_frame)
        self.start_pos_error: float | None = None
        self.start_ori_error: float | None = None
        self.start_converged: bool = False

        # Post-step errors (computed_frame vs target_frame)
        self.post_pos_error: float | None = None
        self.post_ori_error: float | None = None
        self.post_converged: bool = False
        self.regressed: bool = False  # True if computed frame is worse than start frame

        # Debug: constraint tracking
        self.clamped_joints: list[int] = []  # indices of joints that hit limits
        self.angle_deltas: list[float] = []  # raw Newton step before constraints
        self.accepted: bool = True           # False if step regressed and was reverted
        self.stagnated: bool = False         # True if error barely changed between iterations

    def hold_state(self) -> None:
        """Default record: the step did not move, so computed == start."""
        self.computed_angles = list(self.start_angles)
        self.computed_frame = self.start_frame
        self.computed_joint_positions = self.start_joint_positions
        self.computed_joint_frames = self.start_joint_frames
        self.post_pos_error = self.start_pos_error
        self.post_ori_error = self.start_ori_error
        self.post_converged = False

    def early_finish(self) -> None:
        """Mark as converged without taking a step — copies start state to computed."""
        self.hold_state()
        self.post_converged = True

    def set_computed(self, model: "RobotOrientationModel",
                     target_matrix: OrientationMatrix,
                     computed_angles: list[float],
                     fk_flat: np.ndarray | None = None) -> None:
        """Fill the post-step fields from a candidate state.

        Args:
            computed_angles: Joint angles after the step.
            fk_flat: Reuse an already-evaluated FK to skip a call.
        """
        self.computed_angles = list(computed_angles)
        if fk_flat is None:
            fk_flat = model._fk_numpy(computed_angles)
        self.computed_frame = OrientationMatrix(list(np.asarray(fk_flat, dtype=float)))
        self.computed_joint_positions = model.joint_positions(computed_angles, frame=False)
        self.computed_joint_frames = model.joint_positions(computed_angles, frame=True)
        self.post_pos_error, self.post_ori_error = _residual_to_errors(
            _frame_residual(fk_flat, target_matrix))
        self.post_converged = (self.post_pos_error < POS_TOL
                               and self.post_ori_error < ORI_TOL)

    @staticmethod
    def _frame_errors(frame_a: OrientationMatrix,
                      frame_b: OrientationMatrix) -> tuple[float, float]:
        """(position error [in], orientation error [rad]) between two frames."""
        flat = np.array(frame_a.tolist(), dtype=float).flatten()
        return _residual_to_errors(_frame_residual(flat, frame_b))

    def calculate_errors(self, model: "RobotOrientationModel",
                         start_angles: list[float], target_matrix: OrientationMatrix,
                         computed_angles: list[float] | None = None) -> None:
        """Compute frames from angles, then calculate start and post errors.

        Also computes and stores per-joint positions and frames for both
        start and computed states, avoiding redundant FK evaluation later.

        Args:
            model: Compiled FK model to evaluate frames.
            start_angles: Joint angles before the solve step.
            target_matrix: Desired 4x4 end-effector frame.
            computed_angles: Joint angles after the solve step (None if not yet computed).
        """
        self.start_angles = list(start_angles)
        self.target_frame = target_matrix
        self.start_frame = model.solve_forward_kinematics(start_angles)

        self.start_joint_positions = model.joint_positions(start_angles, frame=False)
        self.start_joint_frames = model.joint_positions(start_angles, frame=True)

        self.start_pos_error, self.start_ori_error = self._frame_errors(self.start_frame, target_matrix)
        self.start_converged = self.start_pos_error < POS_TOL and self.start_ori_error < ORI_TOL

        if computed_angles is not None:
            self.computed_angles = list(computed_angles)
            self.computed_frame = model.solve_forward_kinematics(computed_angles)
            self.computed_joint_positions = model.joint_positions(computed_angles, frame=False)
            self.computed_joint_frames = model.joint_positions(computed_angles, frame=True)
            self.post_pos_error, self.post_ori_error = self._frame_errors(self.computed_frame, target_matrix)
            self.post_converged = self.post_pos_error < POS_TOL and self.post_ori_error < ORI_TOL

    def print_result(self) -> None:
        """Print a one-line summary of this iteration's error and convergence."""
        if self.post_converged:
            status = "OK"
        elif self.regressed:
            status = "REGRESSED"
        else:
            status = "--"
        print(f"  start=[{self.start_pos_error:.6e}/{self.start_ori_error:.6e}]  "
              f"computed=[{self.post_pos_error:.6e}/{self.post_ori_error:.6e}]  "
              f"[{POS_TOL}/{ORI_TOL}]  {status}")



class RobotOrientationModel:
    """Symbolic forward kinematics model for a serial-link robot arm.

    Build the kinematic chain by calling add_revolute() and add_linkage()
    in order from base to end effector, then call compile_model() to
    generate a fast numpy-backed FK function.

    Coordinate frame: the caller defines the convention.  For this
    project's rover arm, +x is forward, +y is right, +z is up.
    """

    def __init__(self, unknowns: list[sp.Symbol]) -> None:
        """
        Args:
            unknowns: Symbolic variables for each joint angle (e.g. [t0, t1, ...]).
        """
        self.unknowns: list[sp.Symbol] = unknowns
        self.sym_matrix: OrientationMatrix = sp.eye(4)             # Accumulated 4x4 FK transform
        self._fk_func: callable | None = None               # Compiled lambdified FK
        self._joint_frames: list[OrientationMatrix] = []            # Snapshot after each add_* call
        self._joint_frame_funcs: list[callable] = []        # Compiled joint frame FKs

    def add_revolute(self, sym_var: sp.Symbol, axis: str) -> None:
        """Append a revolute (rotation) joint about the given axis.

        Args:
            sym_var: Symbolic angle variable for this joint.
            axis: Rotation axis — 'x', 'y', or 'z' (case-insensitive).

        Raises:
            ValueError: If axis is not 'x', 'y', or 'z'.
        """
        axis_lower = axis.lower()
        if axis_lower not in ["x", "y", "z"]:
            raise ValueError(f"Invalid axis '{axis}'. Must be 'x', 'y', or 'z'.")

        # Standard 4x4 rotation matrices (active rotation)
        if axis_lower == "x":
            transformation = OrientationMatrix([
                [1, 0, 0, 0],
                [0, sp.cos(sym_var), -sp.sin(sym_var), 0],
                [0, sp.sin(sym_var),  sp.cos(sym_var), 0],
                [0, 0, 0, 1],
            ])
        elif axis_lower == "y":
            transformation = OrientationMatrix([
                [ sp.cos(sym_var), 0, sp.sin(sym_var), 0],
                [0, 1, 0, 0],
                [-sp.sin(sym_var), 0, sp.cos(sym_var), 0],
                [0, 0, 0, 1],
            ])
        elif axis_lower == "z":
            transformation = OrientationMatrix([
                [sp.cos(sym_var), -sp.sin(sym_var), 0, 0],
                [sp.sin(sym_var),  sp.cos(sym_var), 0, 0],
                [0, 0, 1, 0],
                [0, 0, 0, 1],
            ])

        self.sym_matrix = self.sym_matrix * transformation
        self._joint_frames.append(self.sym_matrix)

    def add_linkage(self, length: float, axis: str) -> None:
        """Append a rigid translation (linkage) along the given axis.

        Args:
            length: Link length in inches.
            axis: Translation axis — 'x', 'y', or 'z' (case-insensitive).

        Raises:
            ValueError: If axis is not 'x', 'y', or 'z'.
        """
        axis_lower = axis.lower()
        if axis_lower not in ["x", "y", "z"]:
            raise ValueError(f"Invalid axis '{axis}'. Must be 'x', 'y', or 'z'.")

        # 4x4 pure translation matrices
        if axis_lower == "x":
            transformation = OrientationMatrix([
                [1, 0, 0, length],
                [0, 1, 0, 0],
                [0, 0, 1, 0],
                [0, 0, 0, 1],
            ])
        elif axis_lower == "y":
            transformation = OrientationMatrix([
                [1, 0, 0, 0],
                [0, 1, 0, length],
                [0, 0, 1, 0],
                [0, 0, 0, 1],
            ])
        elif axis_lower == "z":
            transformation = OrientationMatrix([
                [1, 0, 0, 0],
                [0, 1, 0, 0],
                [0, 0, 1, length],
                [0, 0, 0, 1],
            ])

        self.sym_matrix = self.sym_matrix * transformation
        self._joint_frames.append(self.sym_matrix)

    def compile_model(self) -> None:
        """Compile the symbolic FK matrix and joint frames into fast numpy functions.

        Must be called after all add_revolute/add_linkage calls are complete.
        """
        # Compile final FK
        flat_entries: list[sp.Expr] = []
        for r in range(4):
            for c in range(4):
                flat_entries.append(self.sym_matrix[r, c])
        self._fk_func = sp.lambdify(self.unknowns, flat_entries, 'numpy')

        # Compile joint frame FKs (one per add_revolute/add_linkage call)
        self._joint_frame_funcs = []
        for frame in self._joint_frames:
            flat: list[sp.Expr] = []
            for r in range(4):
                for c in range(4):
                    flat.append(frame[r, c])
            self._joint_frame_funcs.append(sp.lambdify(self.unknowns, flat, 'numpy'))

    def solve_forward_kinematics(self, substitutions: list[float]) -> OrientationMatrix:
        """Evaluate the forward kinematics at the given joint angles.

        Args:
            substitutions: Numerical joint angles, one per unknown symbol.

        Returns:
            4x4 homogeneous transformation matrix (end-effector frame).
        """
        # Fast path: use compiled numpy function
        if self._fk_func is not None:
            vals = self._fk_func(*substitutions)
            return OrientationMatrix(vals)

        # Slow path: sympy symbolic substitution
        mapping: dict[sp.Symbol, float] = {}
        for n, var in enumerate(self.unknowns):
            mapping[var] = substitutions[n]
        flat = list(self.sym_matrix.subs(mapping).evalf().tolist())
        return OrientationMatrix(flat)

    def _fk_numpy(self, angles: list[float]) -> np.ndarray:
        """Evaluate FK and return a flat numpy array of 16 values.

        Used internally by the Jacobian and solver for fast numerical access.
        """
        if self._fk_func is not None:
            return np.array(self._fk_func(*angles), dtype=float)
        return np.array(self.solve_forward_kinematics(angles), dtype=float).flatten()

    def joint_positions(self, angles: list[float], frame: bool = False) -> list:
        """Compute the 3D position or full frame of each joint along the chain.

        Evaluates each joint frame FK and extracts either the translation
        (position) or the entire 4x4 frame.

        Args:
            angles: Full list of joint angles.
            frame: If True, return full 4x4 numpy arrays. If False, return [x, y, z] arrays.

        Returns:
            List of numpy arrays — one per joint frame in the kinematic chain.
        """
        results: list = []
        for func in self._joint_frame_funcs:
            vals = func(*angles)
            if frame:
                results.append(OrientationMatrix(vals))
            else:
                flat = np.array(vals, dtype=float).flatten()
                results.append(flat[[3, 7, 11]].tolist())  # Tx, Ty, Tz from row-major flat
        return results



class InverseKinematicsSolver:
    """Numerical inverse kinematics solver using damped least squares.

    Each iteration:
        1. Builds the 6-DOF geometric residual (position in inches,
           orientation as an axis-angle vector in radians) and its
           finite-difference Jacobian.
        2. Solves (J^T J + lambda^2 I) dtheta = -J^T e — a damped,
           minimum-norm step that stays well behaved near singularities.
        3. Zeroes the Jacobian columns of joints sitting on a limit whose
           unconstrained step would push them further into it, so the
           correction is distributed across the joints that can move
           instead of being thrown away by the post-step clamp.
        4. Backtracks: if the candidate step makes the error worse it is
           halved and retried, and rejected outright if no halving helps.

    Call solve_unknowns() for the full iterative solve.
    """

    def __init__(self, model: RobotOrientationModel, max_iters: int = 5,
                 joint_limits: list[tuple[float, float]] | None = None,
                 wrap_angles: bool = True,
                 geometry_limiter: callable | None = None) -> None:
        """
        Args:
            model: Compiled RobotOrientationModel to solve against.
            max_iters: Maximum number of Newton iterations.
            joint_limits: Optional per-joint (min, max) angle bounds in radians.
                If provided, angles are clamped to these limits after each step.
                Use None for a joint to leave it unconstrained.
            wrap_angles: If True, wrap all angles to [-pi, pi] after each step.
            geometry_limiter: Optional callable(angles) -> constrained_angles.
                User-defined function to enforce robot-specific geometric
                constraints (e.g. cable routing, physical joint couplings).
                Applied after wrap_angles and joint_limits.
        """
        self.model: RobotOrientationModel = model
        self.max_iters: int = max_iters
        self.joint_limits: list[tuple[float, float]] | None = joint_limits
        self.wrap_angles: bool = wrap_angles
        self.geometry_limiter: callable | None = geometry_limiter
        self.history: list[IterationResults] = []

    def _apply_constraints(self, angles: list[float]) -> tuple[list[float], list[int]]:
        """Clamp to joint limits, then wrap to [-pi, pi].

        Joint limits are applied BEFORE wrapping so that the clamp
        operates in the solver's native angle space.  Wrapping is
        cosmetic — it keeps angles readable but doesn't affect the
        kinematics.

        Returns:
            (constrained_angles, clamped_indices) — list of joint
            indices that were clamped to their limits.
        """
        result = list(angles)
        clamped: list[int] = []
        if self.joint_limits:
            for i, limits in enumerate(self.joint_limits):
                if limits is not None and i < len(result):
                    if result[i] < limits[0]:
                        result[i] = limits[0]
                        clamped.append(i)
                    elif result[i] > limits[1]:
                        result[i] = limits[1]
                        clamped.append(i)
        if self.wrap_angles:
            result = [wrap2pi(a) for a in result]
        if self.geometry_limiter is not None:
            result = self.geometry_limiter(result)
        return result, clamped

    def _numerical_jacobian(self, current_unknowns: list[float],
                            target_matrix: OrientationMatrix,
                            eps: float = FD_EPS) -> tuple[np.ndarray, np.ndarray]:
        """Compute the 6xN residual Jacobian via forward differences.

        Perturbs each joint angle by eps and measures the change in the
        6-DOF geometric residual (not in the raw FK), so the columns are
        directly usable against that residual.

        Args:
            current_unknowns: Current joint angles.
            target_matrix: Desired 4x4 end-effector frame.
            eps: Finite difference step size.

        Returns:
            (J, e0) — 6 x N Jacobian and the unperturbed residual.
        """
        e0: np.ndarray = _frame_residual(
            self.model._fk_numpy(current_unknowns), target_matrix)

        J: np.ndarray = np.zeros((6, len(self.model.unknowns)))
        for i in range(len(self.model.unknowns)):
            perturbed: list[float] = list(current_unknowns)
            perturbed[i] += eps
            e1: np.ndarray = _frame_residual(
                self.model._fk_numpy(perturbed), target_matrix)
            J[:, i] = (e1 - e0) / eps

        return J, e0

    def _active_set(self, angles: list[float],
                    delta: np.ndarray) -> np.ndarray:
        """Mask joints resting on a limit whose step pushes further into it.

        A joint is only locked when it is already against the bound AND
        the trial step drives it further out — a joint at a limit that
        wants to move away stays free.
        """
        locked = np.zeros(len(angles), dtype=bool)
        if not self.joint_limits:
            return locked
        for i, limits in enumerate(self.joint_limits):
            if limits is None or i >= len(angles) or i >= len(delta):
                continue
            lo, hi = limits
            if delta[i] < 0.0 and angles[i] <= lo + LIMIT_TOL:
                locked[i] = True
            elif delta[i] > 0.0 and angles[i] >= hi - LIMIT_TOL:
                locked[i] = True
        return locked

    @staticmethod
    def _limit_crossings(angles: list[float],
                         joint_limits) -> list[int]:
        """Indices whose position has moved past a joint bound."""
        out: list[int] = []
        if not joint_limits:
            return out
        for i, limits in enumerate(joint_limits):
            if limits is None or i >= len(angles):
                continue
            lo, hi = limits
            if angles[i] < lo - LIMIT_TOL or angles[i] > hi + LIMIT_TOL:
                out.append(i)
        return out

    @staticmethod
    def _plain_step(J: np.ndarray, e: np.ndarray) -> np.ndarray:
        """Gauss-Newton step: (J^T J + lambda^2 I) dtheta = -J^T e.

        Descent by construction — d/dalpha of 0.5|e|^2 at alpha=0 is
        -(J^T e)^T (J^T J + lambda^2 I)^-1 (J^T e) <= 0.  The lambda^2 I
        term keeps the normal equations positive definite near
        singularities where a plain pseudo-inverse would run away.
        """
        A = J.T @ J + DAMPING_LAMBDA_SQ * np.eye(J.shape[1])
        return -np.linalg.solve(A, J.T @ e)

    def _solved_step(self, J: np.ndarray, e: np.ndarray,
                     locked: np.ndarray, locked_delta: np.ndarray) -> np.ndarray:
        """Damped step for the free joints, given the locked joints' motion.

        Solves (J^T J + lambda^2 I) dtheta = -J^T e on the free columns.
        The locked joints contribute J[:, locked] @ locked_delta to the
        residual, so the free joints are solved against what is left —
        otherwise they would be asked to correct an error that the locked
        joints have already absorbed.

        NOT guaranteed to be a descent direction: moving a crossing joint
        to its bound is a finite jump, and the caller checks.
        """
        n = J.shape[1]
        delta = np.zeros(n)
        if locked.any():
            delta[locked] = locked_delta[locked]
            e = e - J[:, locked] @ locked_delta[locked]
        free = ~locked
        if not free.any():
            return delta
        Jm = J[:, free]
        A = Jm.T @ Jm + DAMPING_LAMBDA_SQ * np.eye(Jm.shape[1])
        delta[free] = -np.linalg.solve(A, Jm.T @ e)
        return delta

    def solve_iteration(self, current_unknowns: list[float],
                        target_matrix: OrientationMatrix) -> np.ndarray:
        """Run one Newton-Raphson iteration to reduce the frame error.

        Builds the 6-DOF residual and its Jacobian, then solves the damped
        least-squares system under an active set:

          * joints resting on a limit whose step pushes further out are
            frozen in place;
          * joints whose step would carry them past a bound travel only
            as far as the bound allows and are then frozen.

        In both cases the free joints are re-solved against the residual
        that remains, so the clamp no longer discards a correction that
        was computed as if the pinned joint were free.

        No per-component step cap is applied here — the caller scales the
        whole direction uniformly during line search, which keeps the
        direction a valid descent direction.

        Args:
            current_unknowns: Current joint angles.
            target_matrix: Desired 4x4 end-effector frame.

        Returns:
            Updated joint angles as a numpy array (limits not yet applied).
        """
        J, e = self._numerical_jacobian(current_unknowns, target_matrix)
        cur = np.array(current_unknowns, dtype=float)
        locked = np.zeros(len(cur), dtype=bool)
        locked_delta = np.zeros(len(cur))

        # Joints already sitting on a limit and pushing further into it.
        # Probe with the plain step, since nothing is locked yet.
        if self.joint_limits:
            locked = self._active_set(current_unknowns, self._plain_step(J, e))

        # Pull in joints whose step would carry them past a bound
        for _pass in range(MAX_ACTIVE_SET_PASSES + 1):
            delta = self._solved_step(J, e, locked, locked_delta)
            newly = [i for i in self._limit_crossings(cur + delta,
                                                      self.joint_limits)
                     if not locked[i]]
            if not newly:
                break
            for i in newly:
                lo, hi = self.joint_limits[i]
                locked[i] = True
                locked_delta[i] = (lo - cur[i]) if (cur[i] + delta[i]) < lo \
                    else (hi - cur[i])
        else:
            # Pass budget exhausted — settle on the final lock set
            delta = self._solved_step(J, e, locked, locked_delta)

        # Descent guard.  The active-set composite moves a crossing joint
        # to its bound as a finite jump and then re-solves the free joints
        # against the *post-jump* residual, which does not preserve descent
        # of the whole direction — measured on a real case: a gradient of
        # -1352 flips to +247, and a line search cannot recover from an
        # ascent direction.  Back the bound-jump off until the composite
        # descends again.  The pinned columns stay removed throughout, and
        # a joint resting on a bound has locked_delta == 0, so it is never
        # touched by this rescale.
        if float((J @ delta) @ e) > 0.0:
            lam = 1.0
            for _ in range(32):
                lam *= 0.5
                trial = self._solved_step(J, e, locked, locked_delta * lam)
                if float((J @ trial) @ e) <= 0.0:
                    delta = trial
                    break
            else:
                delta = self._plain_step(J, e)

        return cur + delta

    def solve_unknowns(self, current_unknowns: list[float],
                       target_matrix: OrientationMatrix,
                       start_pos_tol: float | None = None,
                       start_ori_tol: float | None = None):
        """Run the full iterative IK solve, recording one result per step.

        Performs up to max_iters Newton iterations, appending an
        IterationResults to self.history after each one.  Stops early if
        the start frame is already within tolerance, if a step meets
        tolerance, if the step improves nothing (stagnation), or if the
        step had to be rejected outright.

        Each candidate step is evaluated once; if it makes the residual
        worse it is scaled down and retried up to MAX_BACKTRACK times.
        The test is on the 6-DOF residual 2-norm — the quantity the step
        itself minimises — rather than on pos_error + ori_error, which is
        a min-max measure and rejects perfectly good least-squares steps.
        Because every accepted step must lower the norm, the run is
        monotonic and can never end behind its starting point; the revert
        below is a safety net.

        The start_* fields of each record are carried forward from the
        previous record's computed_* fields, so the current state is
        never re-evaluated just to read it back.

        Args:
            current_unknowns: Initial joint angle guess.
            target_matrix: Desired 4x4 end-effector frame.
            start_pos_tol: Optional early-exit position tolerance.  Defaults
                to POS_TOL.
            start_ori_tol: Optional early-exit orientation tolerance.
                Defaults to ORI_TOL.  Tracking callers can pass tighter
                values so sub-tolerance target changes still take a step.
        """
        start_pos_tol = POS_TOL if start_pos_tol is None else start_pos_tol
        start_ori_tol = ORI_TOL if start_ori_tol is None else start_ori_tol
        self.history = []
        t0: float = time.time()

        def norm_of(frame) -> float:
            flat = np.array(frame.tolist(), dtype=float).flatten()
            return _residual_norm(_frame_residual(flat, target_matrix))

        # Snapshot initial state for final comparison.  The incoming guess
        # is projected through the same constraints every step will see:
        # otherwise a start that violates the current joint limits scores
        # better than any step that respects them, and the very first step
        # gets rejected outright.  (Limits are mode-dependent, so a mode
        # change can hand the solver a state that is no longer feasible.)
        initial_angles, _ = self._apply_constraints(list(current_unknowns))
        initial_result = IterationResults()
        initial_result.calculate_errors(self.model, initial_angles, target_matrix)
        initial_norm = norm_of(initial_result.start_frame)

        # Current state — the single source of truth each step starts from
        cur_angles = list(initial_angles)
        cur_frame = initial_result.start_frame
        cur_joint_positions = initial_result.start_joint_positions
        cur_joint_frames = initial_result.start_joint_frames
        cur_pos = initial_result.start_pos_error
        cur_ori = initial_result.start_ori_error
        cur_norm = initial_norm

        for _ in range(self.max_iters):
            iter_result = IterationResults()
            iter_result.target_frame = target_matrix
            iter_result.start_angles = list(cur_angles)
            iter_result.start_frame = cur_frame
            iter_result.start_joint_positions = cur_joint_positions
            iter_result.start_joint_frames = cur_joint_frames
            iter_result.start_pos_error = cur_pos
            iter_result.start_ori_error = cur_ori
            iter_result.start_converged = (cur_pos < start_pos_tol
                                           and cur_ori < start_ori_tol)
            iter_result.hold_state()          # default: the step does not move
            iter_result.solve_time = time.time() - t0
            self.history.append(iter_result)

            if iter_result.start_converged:
                iter_result.early_finish()
                return

            # --- one Newton step (active set + damping handled inside) ---
            raw_step = (np.array(self.solve_iteration(cur_angles, target_matrix),
                                 dtype=float)
                        - np.array(cur_angles, dtype=float))
            iter_result.angle_deltas = raw_step.tolist()
            norm_inf = float(np.max(np.abs(raw_step))) if raw_step.size else 0.0

            if norm_inf <= 1e-12:
                # Every joint is pinned or the gradient is already zero.
                iter_result.stagnated = True
                iter_result.solve_time = time.time() - t0
                return

            # --- line search: scale the whole direction uniformly ---
            # Scaling keeps the search direction a descent direction (a
            # per-component clip would bend it), while still bounding the
            # first trial by MAX_STEP.
            alpha = 1.0 if norm_inf <= MAX_STEP else MAX_STEP / norm_inf
            best = None
            first_clamped: list[int] = []
            for attempt in range(MAX_BACKTRACK + 1):
                if norm_inf * alpha <= 1e-12:
                    break
                cand, clamped = self._apply_constraints(
                    list(np.array(cur_angles, dtype=float) + alpha * raw_step))
                if attempt == 0:
                    first_clamped = clamped
                fk = self.model._fk_numpy(cand)
                e = _frame_residual(fk, target_matrix)
                pos_e, ori_e = _residual_to_errors(e)
                norm_e = _residual_norm(e)
                if norm_e <= cur_norm:
                    best = (cand, fk, pos_e, ori_e, clamped, norm_e)
                    break
                alpha *= 0.5

            if best is None:
                # No scaling of the step improves on where we already are.
                iter_result.clamped_joints = first_clamped
                iter_result.accepted = False
                iter_result.solve_time = time.time() - t0
                return

            cand, fk, pos_e, ori_e, clamped, norm_e = best
            iter_result.clamped_joints = clamped
            iter_result.set_computed(self.model, target_matrix, cand, fk_flat=fk)
            # Relative threshold: 1e-5 of a ~50-unit residual is noise,
            # but the same absolute step near 0.01 in is real progress.
            improvement = cur_norm - norm_e
            iter_result.stagnated = improvement < BACKTRACK_TOL * max(1.0, cur_norm)
            iter_result.solve_time = time.time() - t0

            cur_angles = cand
            cur_frame = iter_result.computed_frame
            cur_joint_positions = iter_result.computed_joint_positions
            cur_joint_frames = iter_result.computed_joint_frames
            cur_pos = pos_e
            cur_ori = ori_e
            cur_norm = norm_e

            if iter_result.post_converged or iter_result.stagnated:
                return

        # Safety net: never hand back something worse than we started with
        if self.history:
            last = self.history[-1]
            if norm_of(last.computed_frame) > initial_norm:
                last.regressed = True
                last.accepted = False
                last.computed_angles = list(initial_angles)
                last.computed_frame = initial_result.start_frame
                last.computed_joint_positions = initial_result.start_joint_positions
                last.computed_joint_frames = initial_result.start_joint_frames
                last.post_pos_error = initial_result.start_pos_error
                last.post_ori_error = initial_result.start_ori_error
                last.post_converged = False

    def solve(self, current_angles: list[float],
              target_matrix: OrientationMatrix,
              start_pos_tol: float | None = None,
              start_ori_tol: float | None = None):
        """Run IK and return results directly.

        Convenience wrapper around solve_unknowns that returns the final
        computed angles, joint data, and frame in one call.

        Returns:
            (angles, joint_positions, joint_frames, computed_frame) or
            (current_angles, None, None, None) if no result was produced.
        """
        self.solve_unknowns(current_angles, target_matrix,
                            start_pos_tol=start_pos_tol,
                            start_ori_tol=start_ori_tol)
        if self.history:
            last = self.history[-1]
            return (list(last.computed_angles),
                    last.computed_joint_positions,
                    last.computed_joint_frames,
                    last.computed_frame)
        return list(current_angles), None, None, None

    def print_test_results(self, target:OrientationMatrix):
        """_summary_

        Args:
            target (OrientationMatrix): _description_
        """

        print("=== Iterative Solver ===")

        target.print_matrix("Target Frame")
        print()

        # Print each iteration from history
        for i, res in enumerate(self.history):
            print(f"  Iter {i+1}", end="  ")
            res.print_result()

        # Print final summary
        last = self.history[-1]
        print(f"\n=== Final Result ({last.solve_time:.4f}s) ===")
        print(f"  Start angles: {[f'{v:.6f}' for v in last.start_angles]}")
        print(f"  Computed angles: {[f'{v:.6f}' for v in last.computed_angles]}")
        print()
        last.start_frame.print_matrix("Start Frame")
        last.computed_frame.print_matrix("Computed Frame")
        print(f"\n  Pos error: {last.post_pos_error:.6e}  (tol={POS_TOL})")
        print(f"  Ori error: {last.post_ori_error:.6e}  (tol={ORI_TOL})")
        print(f"  Converged: {last.post_converged}")

    def solve_ik(self, current_unknowns: list[float], target_frame: OrientationMatrix):
        """Run IK from current values toward target frame. Returns new angles."""
        self.solve_unknowns(current_unknowns, target_frame)
        if self.history:
            return list(self.history[-1].computed_angles)
        return list(current_unknowns)


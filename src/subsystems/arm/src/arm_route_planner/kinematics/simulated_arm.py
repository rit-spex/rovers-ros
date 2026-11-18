"""Simulated arm state, kinematic manager, and rover arm controller."""

import heapq
import math
import random

import sympy as sp

from arm_constants import (
    A0, A1, A2, A3,
    HOMING_TARGET, HOMING_SPEED, HOMING_THRESHOLD,
    MAX_TRANSLATION_DELTA, MAX_ANG_VEL, ANGULAR_SPEED_RAMP_TIME,
    ELBOW_DEADBAND, MECHANICAL_LIMITS, PLANAR_BASE,
    KEY_JOINT_IDX,
    ORIENTATION_SNAP_THRESHOLD, ORIENTATION_SNAP_OFF_AXIS,
    END_EFFECTOR_MIN_Z, TRACK_POS_TOL, TRACK_ORI_TOL, DT,
    PATH_PLANNER_STEP, PATH_PLANNER_MAX_EXPANSIONS,
    PATH_PLANAR_BASE, WAYPOINT_TOL, WAYPOINT_STALL_LIMIT,
)
from .generic_frames import (
    InverseKinematicsSolver, RobotOrientationModel, OrientationMatrix,
    wrap2pi, limit_respecting_diff, nearest_axis_aligned_rotation,
)
from .limiters import (
    PLANAR_MODE, HOMING_MODE,
    PICK_AND_PLACE_MODE, PICK_AND_PLACE_FLIPPED_MODE,
    OCTANT_DIRECT_FLIP, OCTANT_PLANAR_EXIT_LOCK,
    OCTANT_NORMAL_ALLOWED, OCTANT_PLANAR_ALLOWED,
    OCTANT_PLANAR_FLIPPED_ALLOWED,
    arm_constraints, _get_octant,
)
from .actions.home import HomingController
from .actions.flip import FlipController


class KinematicsManager:
    """Owns model construction and safe joint-space path planning."""

    @staticmethod
    def build_arm_solver_object():
        """Build the 5-DOF arm FK model and IK solver.

        Chain: t0(z) -> A0(z) -> t1(x) -> A1(y) -> t2(x) -> A2(y)
               -> t3(x) -> t4(y) -> A3(y)

        The solver is constructed with the elbow deadband as its
        geometry_limiter so the singularity constraint is enforced inside
        every Newton iteration rather than patched on afterwards.

        Returns:
            (model, solver) — compiled and ready to use.
        """
        t0, t1, t2, t3, t4 = sp.symbols('t0 t1 t2 t3 t4', real=True)

        model = RobotOrientationModel([t0, t1, t2, t3, t4])

        # Base Joint — rotate about z
        model.add_revolute(t0, "z")

        # Shoulder Joint — translate up, then rotate about x
        model.add_linkage(A0, "z")
        model.add_revolute(t1, "x")

        # Elbow Joint — translate along y, then rotate about x
        model.add_linkage(A1, "y")
        model.add_revolute(t2, "x")

        # Wrist Bend Joint — translate along y, then rotate about x
        model.add_linkage(A2, "y")
        model.add_revolute(t3, "x")

        # Wrist Twist Joint — rotate about y
        model.add_revolute(t4, "y")

        # End effector — translate along y
        model.add_linkage(A3, "y")

        model.compile_model()

        solver = InverseKinematicsSolver(
            model, max_iters=10,
            joint_limits=[list(pair) for pair in PICK_AND_PLACE_MODE.limits],
            wrap_angles=True,
            geometry_limiter=arm_constraints,
        )
        return model, solver


    @staticmethod
    def extract_key_joints(positions, frames):
        """Select the 4 key joints (indices 1, 3, 5, 8) from the full joint data.

        These correspond to the shoulder, elbow, wrist, and end-effector
        frames — after each linkage translation in the kinematic chain.
        """
        pos = [positions[i] for i in KEY_JOINT_IDX]
        frm = [frames[i] for i in KEY_JOINT_IDX]
        return pos, frm

    @staticmethod
    def _position(model, q, wrists):
        flat = model._fk_numpy([q[0], q[1], q[2], wrists[0], wrists[1]])
        return [flat[3], flat[7], flat[11]]


    @staticmethod
    def path_safe(model, angles, planar_base=PLANAR_BASE):
        """True unless the pose sits in a restricted octant outside planar."""
        if angles[0] is None:
            return True
        probe = list(angles)
        octant = _get_octant(_position(model, probe[:3], probe[3:5]))
        if octant not in OCTANT_PLANAR_EXIT_LOCK:
            return True
        return abs(wrap2pi(probe[0])) <= planar_base + 1e-9


    @staticmethod
    def _segment_safe(model, q0, q1, wrists, allow_unsafe_start=False,
                      planar_base=PLANAR_BASE):
        span = max(abs(q1[i] - q0[i]) for i in range(3))
        n = max(1, int(math.ceil(span / PATH_PLANNER_STEP)))
        first = 1 if allow_unsafe_start else 0
        for i in range(first, n + 1):
            t = i / n
            q = [q0[j] + t * (q1[j] - q0[j]) for j in range(3)]
            if not path_safe(model, q + list(wrists),
                             planar_base=planar_base):
                return False
        return True


    @staticmethod
    def _key(q, step):
        return tuple(int(round(v / step)) for v in q)


    @staticmethod
    def _compress(path):
        """Collapse consecutive collinear steps without cutting corners."""
        if len(path) <= 2:
            return path
        out = [list(path[0])]
        for point in path[1:]:
            if len(out) >= 2:
                a, b = out[-2], out[-1]
                d1 = [b[i] - a[i] for i in range(3)]
                d2 = [point[i] - b[i] for i in range(3)]
                same = all(
                    (abs(d1[i]) < 1e-12 and abs(d2[i]) < 1e-12)
                    or (abs(d1[i]) > 1e-12 and abs(d2[i]) > 1e-12
                        and d1[i] * d2[i] > 0)
                    for i in range(3))
                if same:
                    out[-1] = list(point)
                    continue
            out.append(list(point))
        return out


    @staticmethod
    def plan_safe_path(model, start, goal, step=PATH_PLANNER_STEP,
                       max_expansions=PATH_PLANNER_MAX_EXPANSIONS):
        """Plan a joint path that keeps restricted octants in the planar band.

        Only base, shoulder, and elbow are searched; wrist angles are held at
        their start values until the spatial goal is reached.  Returns a list
        of full 5-angle waypoints (start included), or None if no safe path
        was found.
        """
        start = list(start)
        goal = list(goal)
        wrists = list(start[3:5])
        limits = [MECHANICAL_LIMITS[i] for i in range(3)]

        def clip(q):
            return [max(lo, min(hi, v))
                    for v, (lo, hi) in zip(q, limits)]

        start3 = clip(start[:3])
        goal3 = clip(goal[:3])
        if not path_safe(model, goal3 + wrists):
            return None

        if max(abs(a - b) for a, b in zip(start3, goal3)) < 1e-9:
            return [start3 + wrists]

        def safe(q):
            # Intermediate waypoints keep extra distance from the band edge
            # so waypoint-following lag cannot push the arm outside it.
            return path_safe(model, q + wrists,
                             planar_base=PATH_PLANAR_BASE)

        def heuristic(q):
            return sum(abs(q[i] - goal3[i]) for i in range(3))

        start_key = _key(start3, step)
        nodes = {start_key: list(start3)}
        gscore = {start_key: 0.0}
        came = {start_key: None}
        heap = [(heuristic(start3), 0.0, start_key)]
        start_unsafe = not safe(start3)
        expansions = 0

        def reconstruct(last_key, last_q):
            points = [list(last_q)]
            cur = last_key
            while cur is not None:
                points.append(nodes[cur])
                cur = came[cur]
            points.reverse()
            return points

        while heap:
            _, g, key = heapq.heappop(heap)
            if g > gscore.get(key, float("inf")) + 1e-12:
                continue
            q = nodes[key]
            expansions += 1
            if expansions > max_expansions:
                return None

            dist = max(abs(q[i] - goal3[i]) for i in range(3))
            if dist <= step * 1.01:
                # The final approach may use the full planar band.  It only
                # moves inward toward the goal, so it cannot overshoot the
                # boundary the way an intermediate waypoint can.
                if _segment_safe(model, q, goal3, wrists,
                                 allow_unsafe_start=start_unsafe and
                                 key == start_key,
                                 planar_base=PLANAR_BASE):
                    path = reconstruct(key, goal3)
                    path = _compress(path)
                    out = [point + wrists for point in path]
                    if any(abs(a - b) > 1e-9
                           for a, b in zip(goal[3:5], wrists)):
                        if _segment_safe(model, goal3, goal3, wrists,
                                         planar_base=PLANAR_BASE):
                            out.append(list(goal3) + list(goal[3:5]))
                    return out

            for axis in range(3):
                for sign in (-1.0, 1.0):
                    nq = list(q)
                    nq[axis] = max(limits[axis][0],
                                   min(limits[axis][1],
                                       nq[axis] + sign * step))
                    if abs(nq[axis] - q[axis]) < 1e-12:
                        continue
                    if not safe(nq):
                        continue
                    if not _segment_safe(
                            model, q, nq, wrists,
                            allow_unsafe_start=start_unsafe and key == start_key,
                            planar_base=(PLANAR_BASE
                                         if start_unsafe and key == start_key
                                         else PATH_PLANAR_BASE)):
                        continue
                    nkey = _key(nq, step)
                    ng = g + abs(nq[axis] - q[axis])
                    if ng >= gscore.get(nkey, float("inf")) - 1e-12:
                        continue
                    gscore[nkey] = ng
                    came[nkey] = key
                    nodes[nkey] = nq
                    heapq.heappush(heap, (ng + heuristic(nq), ng, nkey))

        return None

# Compatibility aliases for callers and tests.
build_arm_solver_object = KinematicsManager.build_arm_solver_object
extract_key_joints = KinematicsManager.extract_key_joints
path_safe = KinematicsManager.path_safe
plan_safe_path = KinematicsManager.plan_safe_path
_position = KinematicsManager._position
_segment_safe = KinematicsManager._segment_safe
_key = KinematicsManager._key
_compress = KinematicsManager._compress


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

    def __init__(self, angles, max_ang_vel=MAX_ANG_VEL,
                 ramp_time=ANGULAR_SPEED_RAMP_TIME):
        """Create the mechanical arm from an initial angle list.

        Args:
            angles: Initial joint angles [rad].
            max_ang_vel: Maximum joint speed [rad/s].
            ramp_time: Time to reach full speed from standstill [s].
        """
        self.angles = [float(a) for a in angles]
        self.max_ang_vel = max_ang_vel
        self.ramp_time = max(0.0, float(ramp_time))
        self.joint_speeds = [0.0] * len(self.angles)

    def step(self, target_angles, joint_limits, dt=DT, max_step=None):
        """Advance one control cycle toward ``target_angles``.

        Args:
            target_angles: Desired joint angles [rad].
            joint_limits: Per-joint (min, max) pairs, or None.
            dt: Control period [s].
            max_step: Optional per-joint or scalar maximum angular step
                for this cycle [rad].  Defaults to ``max_ang_vel * dt``.

        Returns:
            list[float]: The new (clamped, velocity-limited) angles.

        Raises:
            ValueError: If the target has a different number of joints.
        """
        if len(target_angles) != len(self.angles):
            raise ValueError(
                f"Expected {len(self.angles)} target angles, "
                f"got {len(target_angles)}")

        default_step = self.max_ang_vel * dt
        if max_step is None:
            step_limits = [default_step] * len(self.angles)
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
                          else default_step)
            limit_speed = limit_step / dt if dt > 1e-12 else 0.0
            target_speed = min(self.max_ang_vel, limit_speed)
            if self.ramp_time <= 1e-9:
                speed = target_speed
            else:
                accel = self.max_ang_vel / self.ramp_time
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


def random_initial_angles(jitter=0.2):
    """Return randomized, valid starting angles for *simulation only*.

    The production controller must always be built from measured motor
    angles.  Simulated inputs (the standalone simulator and its tests)
    use this helper to start near the home pose with a small random
    perturbation so runs begin valid and away from a singularity.
    """
    angles = []
    for i, (lo, hi) in enumerate(PICK_AND_PLACE_MODE.limits):
        angle = HOMING_TARGET[i] + random.uniform(-jitter, jitter)
        angles.append(max(lo, min(hi, angle)))
    return angles


class RoverArmController:
    """Central kinematic state container for the 5-DOF rover arm.

    State variables:
        angles: list[float] — latest achieved joint angles [rad]
        target_angles: list[float] — desired joint angles [rad]
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

    def __init__(self, initial_angles):
        """Build the FK model, IK solver, and initialize all controllers.

        Args:
            initial_angles: Starting joint angles [rad], one per joint.
                Production code passes the measured motor angles here
                (or calls ``sync_angles()`` immediately afterwards).
                Simulated inputs should pass ``random_initial_angles()``.
        """
        angles = [float(a) for a in initial_angles]
        if len(angles) != len(PICK_AND_PLACE_MODE.limits):
            raise ValueError(
                f"Expected {len(PICK_AND_PLACE_MODE.limits)} initial "
                f"angles, got {len(angles)}")

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
        self._orientation_lock_rotation = None
        self.home_joint_idx = None     # None=full homing, int=single joint
        self._homing_flipped = False   # side whose home pose is targeted
        self._homing_cross_dir = 0     # elbow deadband crossing direction
        self.limit_mode = PICK_AND_PLACE_MODE
        self.flipped = False           # Pick-and-Place side modifier
        self.planar = False            # base-locked vertical modifier
        self.solver_history = []       # debug: last IK iteration results
        self.last_failure = None       # last user-facing rejection message

        # Joint state — the caller supplies the starting pose.  Production
        # uses measured motor angles; simulation uses random_initial_angles().
        self.mechanical = SimulatedMechanicalArm(angles)
        self.target_angles = list(self.angles)

        # Safe homing path and flip waypoint follow state
        self._homing_path = None
        self._homing_path_idx = 0
        self._homing_seg_steps = None
        self._follow_target = None
        self._follow_path = None
        self._follow_path_idx = 0
        self._follow_limits = None
        self._follow_switch_mode = False
        self._follow_max_step = None
        self._follow_stall = 0
        self._homing_stall = 0

        # Target frame is always FK(target_angles) — a reachable pose
        self.target_frame = self.model.solve_forward_kinematics(self.target_angles)
        self.target_pos = list(self.target_frame.position())

        # Achieved state — computed by refresh_fk()
        self.pos = None
        self.frames = None
        self.l0 = self.l1 = self.l2 = self.l3 = None
        self.refresh_fk()

    # ------------------------------------------------------------------
    # Joint state / mechanical arm
    # ------------------------------------------------------------------

    @property
    def angles(self):
        """Actual joint angles owned by the simulated mechanical arm."""
        return self.mechanical.angles

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
        if len(angles) != len(self.angles):
            raise ValueError(
                f"Expected {len(self.angles)} angles, got {len(angles)}")
        self.mechanical.angles = angles
        if reset_target:
            self.mechanical.joint_speeds = [0.0] * len(angles)
            self.target_angles = list(angles)
            frame = self.model.solve_forward_kinematics(self.target_angles)
            self.target_frame = self._frame_with_orientation_lock(frame)
            self.target_pos = list(self.target_frame.position())
        self.refresh_fk()

    def _command_mechanical(self, target_angles, joint_limits):
        """Slew the mechanical arm toward a target for one cycle."""
        self.mechanical.step(target_angles, joint_limits, DT)

    def _locked_limits(self, joint_limits):
        """Pin locked joints to their actual angles in a limit list."""
        if joint_limits is None:
            result = [None] * len(self.angles)
        else:
            result = [tuple(pair) if pair is not None else None
                      for pair in joint_limits]
        for i, locked in enumerate(self.locked_joints):
            if locked and i < len(result):
                a = self.angles[i]
                result[i] = (a, a)
        return result

    def _all_at(self, target, tol=HOMING_THRESHOLD, mask=None):
        """True when every allowed joint is within ``tol`` of target."""
        for i, tgt in enumerate(target):
            if mask is not None and i < len(mask) and not mask[i]:
                continue
            if abs(wrap2pi(tgt - self.angles[i])) > tol:
                return False
        return True

    def _target_error(self, target, mask=None):
        """Largest remaining error to ``target`` over the allowed joints."""
        error = 0.0
        for i, tgt in enumerate(target):
            if mask is not None and i < len(mask) and not mask[i]:
                continue
            error = max(error, abs(wrap2pi(tgt - self.angles[i])))
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

        The largest move runs at the mechanical arm's normal max speed;
        the other joints are scaled by their travel distance.
        """
        deltas = [abs(wrap2pi(t - s)) for s, t in zip(start, target)]
        max_delta = max(deltas) if deltas else 0.0
        if max_delta < 1e-12:
            return [0.0] * len(deltas)
        max_step = self.mechanical.max_ang_vel * DT
        return [max_step * (d / max_delta) for d in deltas]

    # ------------------------------------------------------------------
    # FK refresh
    # ------------------------------------------------------------------

    def refresh_fk(self):
        """Recompute achieved FK joint data from current angles.

        Evaluates the forward kinematics chain at the current joint
        angles and extracts the 4 key joint positions (after each
        linkage translation: shoulder, elbow, wrist, end-effector)
        and their corresponding 4x4 frames.  Does not touch the
        desired target frame.
        """
        positions = self.model.joint_positions(self.angles, frame=False)
        frames = self.model.joint_positions(self.angles, frame=True)
        self.pos, self.frames = extract_key_joints(positions, frames)
        self.l0, self.l1, self.l2, self.l3 = self.pos

    def _sync_target_to_fk(self):
        """Snap target_angles and target_frame to the achieved FK pose.

        Used when the arm is driven directly (homing, flip, joint
        drive) so the target marker tracks the arm instead of leaving
        a stale desired pose behind.
        """
        self.target_angles = list(self.angles)
        frame = self.model.solve_forward_kinematics(self.target_angles)
        self.target_frame = self._frame_with_orientation_lock(frame)
        self.target_pos = list(self.target_frame.position())

    # ------------------------------------------------------------------
    # Mode queries
    # ------------------------------------------------------------------

    planar_exit_lock_octants = OCTANT_PLANAR_EXIT_LOCK

    def _ee_octant(self):
        """Octant containing the current end-effector position."""
        pos = self.model.solve_forward_kinematics(self.angles).position()
        return _get_octant(pos)

    def _workspace_blocked(self, angles):
        """True when a candidate pose violates the height or octant gates."""
        if self.is_homing or self.is_flipping:
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
        if self.is_homing or self.is_flipping:
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
        """Limits for Pick and Place with its planar/flipped modifiers."""
        if self.planar:
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

    def _settle_side(self):
        """Update the flipped flag from the achieved pose, if possible."""
        if self.planar:
            return
        if PICK_AND_PLACE_FLIPPED_MODE.can_set(self.angles):
            self.flipped = True
        elif PICK_AND_PLACE_MODE.can_set(self.angles):
            self.flipped = False

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
        return PLANAR_MODE.can_set(self.angles)

    @property
    def effective_limits(self):
        """Per-joint limits currently enforced by the arm.

        Flip and homing actions run under full mechanical travel; the
        normal Pick-and-Place limits return when they finish.
        """
        if self.is_flipping or self.is_homing:
            return [tuple(pair) for pair in MECHANICAL_LIMITS]
        return self._active_limits(self.angles)

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

    def set_pnp_normal(self):
        """Select normal Pick-and-Place limits when the arm fits them."""
        if self.planar and not self._planar_exit_allowed():
            return self._fail(
                "Normal unavailable: restricted octant requires planar")
        if not PICK_AND_PLACE_MODE.can_set(self.angles):
            return self._fail(
                "Normal unavailable: pose is outside normal limits")
        self.planar = False
        self.flipped = False
        return True

    def toggle_flipped(self):
        """Select flipped-side limits when the arm fits them.

        This only changes the limit modifier; it never moves the arm.
        Use activate_flip() to physically change sides.
        """
        if self.planar and not self._planar_exit_allowed():
            return self._fail(
                "Flipped unavailable: restricted octant requires planar")
        if self.flipped:
            if not PICK_AND_PLACE_MODE.can_set(self.angles):
                return self._fail(
                    "Normal unavailable: pose is outside normal limits")
            self.flipped = False
            return True

        if not PICK_AND_PLACE_FLIPPED_MODE.can_set(self.angles):
            return self._fail(
                "Flipped unavailable: pose is outside flipped limits")
        self.planar = False
        self.flipped = True
        return True

    def toggle_planar(self):
        """Toggle the base-locked planar modifier.

        Planar cannot be exited while the end effector is in octants
        3, 5, or 7; those regions require the base lock.
        """
        if self.planar:
            if not self._planar_exit_allowed():
                return self._fail(
                    "Planar locked: restricted octant requires planar")
            if PICK_AND_PLACE_FLIPPED_MODE.can_set(self.angles):
                self.planar = False
                self.flipped = True
                return True
            if PICK_AND_PLACE_MODE.can_set(self.angles):
                self.planar = False
                self.flipped = False
                return True
            return self._fail(
                "Planar cannot be exited at this pose")

        if not PLANAR_MODE.can_set(self.angles):
            return self._fail("Planar needs base within +/-5 deg")
        self.planar = True
        return True

    def toggle_homing(self):
        """Toggle full-arm homing on/off."""
        if self.homing.active:
            self.homing.deactivate()
            self.home_joint_idx = None
            self._homing_path = None
            self._homing_stall = 0
            self._settle_side()
        else:
            self.home_joint_idx = None
            self.homing.target = self._home_target()
            self._homing_flipped = self.flipped
            self.homing.activate()
            self._homing_path = None
            self._homing_stall = 0
            if not any(self.locked_joints[:3]):
                effective = self._hold_joints(
                    self.homing.target, self.angles)
                path = plan_safe_path(
                    self.model, self.angles, effective)
                if path and len(path) > 1:
                    self._homing_path = path[1:]
                    self._homing_path_idx = 0
                    self._homing_seg_steps = self._synchronized_steps(
                        self.angles, self._homing_path[0])
        self._homing_cross_dir = 0

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
        self.locked_joints[idx] = not self.locked_joints[idx]
        return self.locked_joints[idx]

    def toggle_position_lock(self):
        """Toggle locking the target's translation."""
        self.position_locked = not self.position_locked
        return self.position_locked

    def toggle_orientation_lock(self):
        """Toggle locking the target's rotation."""
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
        if not (0 <= idx < len(self.locked_joints)):
            return False
        self.locked_joints[idx] = False
        if self.homing.active:
            self.homing.deactivate()
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
        if self.homing.active or self.is_flipping:
            return self._fail("Command blocked: action is running")
        if not 0 <= index < min(len(self.angles), len(self.target_angles)):
            return self._fail("Command blocked: invalid joint")

        held = list(self.angles)
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
        if self.homing.active or self.is_flipping:
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
            desired, self.angles)
        return self._solve_desired_frame(desired)

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
        octant = self._ee_octant()
        if octant not in OCTANT_DIRECT_FLIP and not self.planar:
            return self._fail(
                f"Flip blocked: octant {octant} requires planar")

        limits = self.flip_side_limits()
        started = self.flipper.activate(
            self.angles, self.l0, self.l1, self.l2,
            joint_limits=limits, side=True)
        if not started:
            return self._fail("Flip blocked: another flip is running")
        if started:
            held = list(self.angles)
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

    def update(self, deltas=None):
        """Advance one control cycle based on the current mode.

        Dispatches to the appropriate update path:
            1. Flip follow  -> slew to the flip target (overrides all)
            2. Homing       -> step toward home configuration
            3. No input     -> finish the last position-drive target
            4. Joint drive  -> direct joint deltas + clamping
            5. Position drive -> target frame delta + IK solve

        The mechanical arm owns the actual angles in every path; this
        method only commands it, so a bad target is velocity-limited
        rather than applied as a jump.

        Args:
            deltas: dict with keys dx, dy, dz, drx, dry, drz
                    (scaled SpaceMouse deltas), or None when idle.

        Returns:
            bool: True if input was consumed (state was updated).
        """
        # Flip motion overrides everything until the target is reached
        if self._follow_target is not None:
            held = list(self.angles)
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
                    held = list(self.angles)
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

            self.target_angles = list(self.angles)
            self.refresh_fk()
            self._sync_target_to_fk()
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
        if PICK_AND_PLACE_FLIPPED_MODE.can_set(self.angles):
            self.flipped = True
        elif PICK_AND_PLACE_MODE.can_set(self.angles):
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
        held = list(self.angles)
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
                    self.angles, mask)
                self._homing_seg_steps = self._synchronized_steps(
                    self.angles, nxt)

            if self._homing_path is not None:
                self._sync_target_to_fk()
                return

        commanded, done = self.homing.step(
            self.angles, limits=HOMING_MODE.limits, mask=mask)
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
        held = list(self.angles)
        commanded = list(self.angles)
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
        held = list(self.angles)
        target = self._hold_joints(self.target_angles, held)

        if any(self.locked_joints):
            self.target_angles = list(target)
            frame = self.model.solve_forward_kinematics(self.target_angles)
            self.target_frame = self._frame_with_orientation_lock(frame)
            self.target_pos = list(self.target_frame.position())

        limits = self._active_limits(self.target_angles)
        self._command_mechanical(self.target_angles,
                                 self._locked_limits(limits))

    def _solve_desired_frame(self, desired):
        """Run IK toward a desired frame and command the mechanical arm."""
        # Locked joints are pinned to a zero-width limit so the solver
        # distributes the motion across the remaining motors.
        locked_angles = [wrap2pi(a) for a in self.angles]
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

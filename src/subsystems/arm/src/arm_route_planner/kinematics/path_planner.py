"""Joint-space path planning for the 5-DOF arm.

Builds the FK/IK model and searches a safe joint path that keeps restricted
octants inside the planar band.  Pure library: no ROS imports.
"""

import heapq
import math

import sympy as sp

from arm_constants import (
    A0, A1, A2, A3,
    KEY_JOINT_IDX,
    MECHANICAL_LIMITS, PLANAR_BASE,
    PATH_PLANNER_STEP, PATH_PLANNER_MAX_EXPANSIONS, PATH_PLANAR_BASE,
)
from .linkage_models import (
    InverseKinematicsSolver, RobotOrientationModel, wrap2pi,
)
from .limiters import (
    PICK_AND_PLACE_MODE,
    OCTANT_PLANAR_EXIT_LOCK,
    arm_constraints, _get_octant,
)


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

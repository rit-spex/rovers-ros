import sympy as sp
import numpy as np
import math
import time

from math_helpers import (
    calc_angle_between_points,
    get_flipped_angles,
    wrap_to_pi,
    angle_diff,
    Tz,
)
from arm_parameters import A0, A1, A2, A3

# === Okay this is just geometry ===


def flip_shoulder(th0, th1, th2):
    """Perform a flip operation on the shoulder

    Args:
        th0 (float): Base angle
        th1 (float): Shoulder angle
        th2 (float): Elbow angle

    Returns:
        th0 (float): Base angle
        th1 (float): Shoulder angle
        th2 (float): Elbow angle
    """
    th0 = wrap_to_pi(th0 + math.pi)
    th1 = wrap_to_pi(-th1 + math.pi)
    th2 = -th2
    return th0, th1, th2


def joint_limits(th0, th1, th2, th3, th4, fix_elbow=False):
    """Enforce angular limits on joints, to prevent damage to the joints

    Args:
        th0 (float): Base Angle [rad]
        th1 (float): Shoulder Angle [rad]
        th2 (float): Elbow Angle [rad]
        th3 (float): Wrist Angle 1 [rad]
        th4 (float): Wrist Angle 2 [rad]
        fix_elbow (bool): Correct the elbow if the shoulder is adjusted

    Returns:
        Same angles, but corrected
    """

    # === Base Limits ===
    th0_lims = [-math.pi * 3 / 4, -math.pi / 4]  # Must stay between [-45, 225]
    if th0 < 0:
        if th0 > -math.pi / 2:
            if th0 < th0_lims[1]:
                th0 = th0_lims[1]
        else:
            if th0 > th0_lims[0]:
                th0 = th0_lims[0]

    # === Shoulder Limits ===
    # (You need to then correct the elbow angle, this is def NOT the best way to correct th2)
    range_low = 1 / 16
    th1_lims = [
        -math.pi * (1 - range_low),
        -math.pi * range_low,
    ]  # Must stay between [-45, 225] *** INCORRECT ANGLES
    if th1 < 0:
        if th1 > -math.pi / 2:
            diff = abs(angle_diff(th1, th1_lims[1]))
            if th1 < th1_lims[1]:
                th1 = th1_lims[1]
                if fix_elbow:
                    th2 -= diff
        else:
            diff = abs(angle_diff(th1, th1_lims[0]))
            if th1 > th1_lims[0]:
                th1 = th1_lims[0]
                if fix_elbow:
                    th2 += diff

    # === Elbow Limits ===
    th2 = simple_clamp(th2, [-math.pi * 3 / 4, math.pi * 3 / 4])

    # === Wrist Limits ===
    th3 = simple_clamp(th3, [-math.pi * 3 / 4, math.pi * 3 / 4])

    # === Wrist Limits ===
    th4 = simple_clamp(th4, [-math.pi * 7 / 8, math.pi * 7 / 8])

    return th0, th1, th2, th3, th4


def collision_protection(th0, th1, th2, th3, th4):
    # FIXME
    ## ENFORCE "OCTANT" LIMITS
    # Octant - 1/8 of a 3-D region, specifically the space around the rover.
    # |--------------------------------------------|  no this was not chatgpt, i actually did this -Ethan
    # |             Octant Definitions             |
    # |--------------------------------------------|
    # | O# |  X-Y  |  Z   | Constraints            |
    # |--------------------------------------------|
    # | O1 | Front | High | Free                   |
    # | O2 | Right | High | Free                   |
    # | O3 | Back  | High | 85<th0<95 & -30<th3<30 |
    # | O4 | Left  | High | Free                   |
    # |--------------------------------------------|
    # | O5 | Front | Low  | 85<th0<95 & -30<th3<30 |
    # | O6 | Right | Low  | Elbow Curl             |
    # | O7 | Back  | Low  | 85<th0<95 & -30<th3<30 |
    # | O8 | Left  | Low  | Elbow Curl             |
    # |--------------------------------------------|

    # If (a position is in O3 AND y < -4) AND (th0 out of range OR th3 is out of range): IDK
    pass

    # If a position is in O5 AND (th0 is out of range OR th3 is out of range) : IDK
    pass

    # # If a position is in O7 -> Just limit the angles, no z boundary needed
    # j0, j1, j2, j3 = calc_joint_positions(th0, th1, th2, th3, True)
    # if get_octant(j3) == 7:
    #     th0 = simple_clamp(th0, [math.pi*85/90, math.pi*95/90])
    #     th3 = simple_clamp(th3, [-math.pi/6, math.pi/6])
    # # elif get_octant(j3) == 3:

    ### BUG: RE-APPLY MECHANICAL LIMITS AS THEY ARE DOMINANT (I THINK)

    return th0, th1, th2, th3, th4


def homingStep(homing, th0, th1, th2, th3, th4):

    # NOTE: Not tested, will need collision avoidance, Ethan will do post-SAR

    # FIXME: Should these be input args, or parameters???
    # ang_des = [3.1415 / 2, 3.1415 / 2, 0, 0, 0]
    # d_ang = 1 / 100
    # max_diff = 1 / 40

    # if homing:
    #     # Set shoulder angle
    #     if abs(th1 - ang_des[1]) > max_diff:
    #         if th1 < ang_des[1]:
    #             th1 += d_ang
    #         else:
    #             th1 -= d_ang

    #     # Set elbow angle
    #     elif abs(th2 - ang_des[2]) > max_diff:
    #         if th2 < ang_des[2]:
    #             th2 += d_ang
    #         else:
    #             th2 -= d_ang

    #     # Set wrist angle 1
    #     elif abs(th3 - ang_des[3]) > max_diff:
    #         if th3 < ang_des[3]:
    #             th3 += d_ang
    #         else:
    #             th3 -= d_ang

    #     # Set wrist angle 2
    #     elif abs(th4 - ang_des[4]) > max_diff:
    #         if th4 < ang_des[4]:
    #             th4 += d_ang
    #         else:
    #             th4 -= d_ang

    #     # Set base angle
    #     elif abs(th0 - ang_des[0]) > max_diff:
    #         if abs(th0) < math.pi / 2:
    #             th0 += d_ang
    #         else:
    #             th0 -= d_ang

    #         th0 = wrap_to_pi(th0)

    #     else:
    #         homing = False
    #         _, _, _, point = calc_joint_positions(th0, th1, th2, th3, False)

    return homing, th0, th1, th2, th3, th4


def get_octant(point):
    """Find the 'octant of the rover's grid (see joint_limits for definitions)

    Args:
        point (list or np.array): Position coordinate being checked

    Returns:
        int: Octant number
    """

    # Parse point data
    px = point[0]
    py = point[1]
    pz = point[2]

    # Reduce the octant for the bottom half of the rover
    if pz > 0:
        dz = 0
    else:
        dz = 4

    # Use y=x to check conditions
    if py >= px and py >= -px:
        return 1 + dz
    if py <= px and py >= -px:
        return 2 + dz
    if py <= px and py <= -px:
        return 3 + dz
    if py >= px and py <= -px:
        return 4 + dz

    raise Exception(
        "Math Problemmmm"
    )  # Due to this being used for safety features, this felt appropriate idk im tired


def simple_clamp(angle, limits):
    """Limit angles of a motors to a simple range typically centered about 0 (i.e. [-90, 90])

    Args:
        angle (float): Unclamped motor angle
        limits (list): List of a lower and upper limit

    Returns:
        angle: Limited angle
    """
    if angle < limits[0]:
        angle = limits[0]
    if angle > limits[1]:
        angle = limits[1]
    return angle


# Now we get the gross stuff


def inverse_kinematics(Pvect, current_angs, Rot):
    """Solve Arm Angles using the principles of inverse kinematics

    Args:
        Pvect (list[float]): Target [X,Y,Z] for End Effector [in]
        Rot (list[float]): Target Angle for End Effector [rad]
        current_angs (list[float]): List of angles in [rad]

    Returns:
        th0 (float): Angle of the base joint
        th1 (float): Angle of the shoulder joint
        th2 (float): Angle of the elbow joint
        th3 (float): Angle of the wrist joint
    """

    tolerance = 1e-6

    # --- Fix input ---

    # --- Symbolic Variables ---
    t0, t1, t2, t3 = sp.symbols("t0 t1 t2 t3", real=True)

    # --- Forward Kinematics ---
    f0, f1, f2, f3 = forward_kin_matrix(t0, t1, t2, t3, False)

    # --- Inverse Kinematics equations ---
    eqs = sp.Matrix(
        [
            f3[0, 3] - Pvect[0],
            f3[1, 3] - Pvect[1],
            f3[2, 3] - Pvect[2],
            t1 + t2 + t3 - Rot,
        ]
    )

    # --- Numerical Solve (vector nsolve) ---
    current_angs_sym = tuple(sp.Float(a) for a in current_angs)
    try:
        sol = sp.nsolve(
            eqs, (t0, t1, t2, t3), current_angs_sym, tol=tolerance, maxsteps=250
        )
        th1, th2, th3 = map(float, sol)
        # print("Original works")
    except Exception as e:
        try:
            # Try 2D
            r = np.linalg.norm(Pvect - [0, 0, A0])
            c = (r**2 - A1**2 - A2**2) / (2 * A1 * A2)
            c = max(-1.0, min(1.0, c))
            tht2 = math.acos(c)
            tht1 = math.atan2((A2 * math.sin(tht2)), (A1 + A2 * math.cos(tht2)))
            tht0 = np.arctan2(Pvect[1], Pvect[0]) - math.pi / 2

            # sol = sp.nsolve(eqs, (t0, t1, t2, t3), [tht0, tht1, tht2, current_angs_sym[3]], tol=tolerance, maxsteps=2500)
            sol = sp.nsolve(
                eqs,
                (t0, t1, t2, t3),
                [tht0, tht1, tht2, 0],
                tol=tolerance,
                maxsteps=1000,
            )
            th0, th1, th2, th3 = map(float, sol)
        except Exception as e:
            try:
                # Try 2D
                r = np.linalg.norm(Pvect - [0, 0, A0])
                c = (r**2 - A1**2 - A2**2) / (2 * A1 * A2)
                c = max(-1.0, min(1.0, c))
                tht2 = math.acos(c)
                tht1 = math.atan2((A2 * math.sin(tht2)), (A1 + A2 * math.cos(tht2)))
                tht0 = np.arctan2(Pvect[1], Pvect[0]) - math.pi / 2

                tht0, tht1, tht2 = flip_shoulder(tht0, tht1, tht2)
                # sol = sp.nsolve(eqs, (t0, t1, t2, t3), [tht0, tht1, tht2, current_angs_sym[3]], tol=tolerance, maxsteps=2500)
                sol = sp.nsolve(
                    eqs,
                    (t0, t1, t2, t3),
                    [tht0, tht1, tht2, 0],
                    tol=tolerance,
                    maxsteps=1000,
                )
                th0, th1, th2, th3 = map(float, sol)
            except Exception as e:
                # Try 2D
                r = np.linalg.norm(Pvect - [0, 0, A0])
                c = (r**2 - A1**2 - A2**2) / (2 * A1 * A2)
                c = max(-1.0, min(1.0, c))
                tht2 = math.acos(c)
                tht1 = math.atan2((A2 * math.sin(tht2)), (A1 + A2 * math.cos(tht2)))
                tht0 = np.arctan2(Pvect[1], Pvect[0]) - math.pi / 2

                tht0, tht1, tht2 = flip_shoulder(tht0, tht1, tht2)

                print(f"x{tht0*57.3:10.4f}\t\t{tht1*57.3:10.4f}\t\t{tht2*57.3:10.4f}")

                raise Exception("Failed to calculate positions")

    # Initial values
    th0 = round(wrap_to_pi(th0), 4)
    th1 = round(wrap_to_pi(th1), 4)
    th2 = round(wrap_to_pi(th2), 4)
    th3 = round(wrap_to_pi(th3), 4)

    # Compare which solution is closer
    if abs(angle_diff(-th0, current_angs[0])) < abs(angle_diff(th0, current_angs[0])):

        # Flip the shoulder linkage
        th0, th1, th2 = flip_shoulder(th0, th1, th2)

    # This will reverse the flip action if the logic is near a 180, 0, -180 boundary, detected by a 'correct' flip being 90 deg change
    if abs(angle_diff(th0, current_angs[0])) > 1.2:
        # Flip the shoulder linkage
        th0, th1, th2 = flip_shoulder(th0, th1, th2)

    # # Validate t1 and t2 are correct
    l0, l1, l2, l3 = calc_joint_positions(th0, th1, th2, th3)

    # # Compare which solution is closer
    if abs(angle_diff(-th1, current_angs[1])) < abs(angle_diff(th1, current_angs[1])):

        # Flip the elbow linkage
        th1, th2 = get_flipped_angles(th1, th2, l0, l1, l2)

        # Adjust back the gripper angle
        l0, l1, l2, _ = calc_joint_positions(th0, th1, th2, th3)
        th3 = calc_angle_between_points(l1, l2, l3) - math.pi

        # Update positions
        l0, l1, l2, l3 = calc_joint_positions(th0, th1, th2, th3)

    if abs(angle_diff(-th2, current_angs[2])) < abs(angle_diff(th2, current_angs[2])):
        # Flip the elbow linkage
        th1, th2 = get_flipped_angles(th1, th2, l0, l1, l2)

        # Adjust back the gripper angle
        l0, l1, l2, _ = calc_joint_positions(th0, th1, th2, th3)
        th3 = calc_angle_between_points(l1, l2, l3) - math.pi

        # Update positions
        l0, l1, l2, l3 = calc_joint_positions(th0, th1, th2, th3)

    # if angle_diff(th0, current_angs[0]) > 1.5 or angle_diff(th1, current_angs[1]) > 1.5 or angle_diff(th2, current_angs[2]) > 1.5 or angle_diff(th3, current_angs[3]) > 1.5:
    #     raise Exception("Large flip: ignore this iteration")

    # --- Results
    return th0, th1, th2, th3


def calc_joint_positions(th0, th1, th2, th3, gripper=True):
    """Calculate the positions of each joint/axis

    Args:
        th0 (float): Angle of the base joint
        th1 (float): Angle of the shoulder joint
        th2 (float): Angle of the elbow joint
        th3 (float): Angle of the wrist joint
        gripper (bool): Should the gripper be included as having a non-zero length

    Returns:
        l0 (list[float]): Position of the shoulder joint
        l1 (list[float]): Position of the elbow joint
        l2 (list[float]): Position of the wrist joint
        l3 (list[float]): Position of the gripper joint
    """
    # Solve the forward kinematics
    f0, f1, f2, f3 = forward_kin_matrix(th0, th1, th2, th3, gripper)

    # Solve for the link positions
    l0 = np.array(f0[0:3, 3], dtype=float).flatten()  # if f0 is 4x4 homogeneous
    l1 = np.array(f1[0:3, 3], dtype=float).flatten()
    l2 = np.array(f2[0:3, 3], dtype=float).flatten()
    l3 = np.array(f3[0:3, 3], dtype=float).flatten()

    # Output to code
    return l0, l1, l2, l3


def forward_kin_matrix(t0, t1, t2, t3, gripper=True):
    """Solve the forward kinematics matricies based on the given angles

    Args:
        t0 (float or sym): Base joint angle
        t1 (float or sym): Shoulder joint angle
        t2 (float or sym): Elbow joint angle
        t3 (float or sym): Wrist joint angle

    Returns:
        f0, f1, f2, f3: Coordinate frame matrix for each joint
    """
    # --- Forward Kinematics ---
    # Base Joint
    Tz0 = sp.Matrix(
        [[1, 0, 0, 0], [0, 1, 0, 0], [0, 0, 1, A0], [0, 0, 0, 1]]  # Move up the z
    )
    Rx = sp.Matrix(
        [[1, 0, 0, 0], [0, 0, -1, 0], [0, 1, 0, 0], [0, 0, 0, 1]]  # Rotate y to z
    )
    Ry = sp.Matrix(
        [
            [sp.cos(t0), 0, sp.sin(t0), 0],
            [0, 1, 0, 0],
            [-sp.sin(t0), 0, sp.cos(t0), 0],
            [0, 0, 0, 1],
        ]
    )
    f0 = Tz0 * Rx * Ry

    # Other joints
    f1 = f0 * Tz(A1, t1)  # Shoulder rotation
    f2 = f1 * Tz(A2, t2)  # Elbow rotation
    if gripper:
        f3 = f2 * Tz(A3, t3)  # Wrist rotation
    else:
        f3 = f2 * Tz(0.00001, t3)

    return f0, f1, f2, f3

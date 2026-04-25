import math
from math_helpers import *


def fast_IK_solve(
    current_ang,
    target_point: list[float],
    known_th3: float,
    L1: float,
    L2: float,
    L3: float,
    L4: float,
    logger,
):
    """
    Calculates IK for the end-effector by combining L3 and L4 into a virtual link.
    """
    x, y, z = target_point[0], target_point[1], target_point[2]

    # 1. Base Angle
    th0 = math.atan2(y, x)

    # 2. Planar Geometry
    r = math.sqrt(x**2 + y**2)
    r = max(r, 0.25)
    z_rel = z - L1

    # Distance squared from shoulder to END EFFECTOR
    D_sq = r**2 + z_rel**2
    D = math.sqrt(D_sq)

    # 3. Create the Virtual Link (Combining Elbow and Wrist links)
    # These calculate the length and angle offset of the combined L3/L4 geometry
    L_virt = math.sqrt(L3**2 + L4**2 + (2 * L3 * L4 * math.cos(known_th3)))
    alpha = math.atan2(L4 * math.sin(known_th3), L3 + L4 * math.cos(known_th3))

    # Reachability Check (now using L_virt)
    if D > (L2 + L_virt) or D < abs(L2 - L_virt):
        logger.info("Target point out of physical reach! Halting movement.")
        return *current_ang[0:3], forward_kin(current_ang, L1, L2, L3, L4)

    # 4. Solve the Virtual Triangle
    # Virtual Elbow Angle (angle between L2 and L_virt)
    cos_th2_virt = (D_sq - L2**2 - L_virt**2) / (2 * L2 * L_virt)
    cos_th2_virt = max(min(cos_th2_virt, 1.0), -1.0)

    th2_virt = -math.acos(cos_th2_virt)
    # print(th2_virt)

    # Actual Elbow Angle (remove the virtual offset)
    th2 = th2_virt - alpha
    # print(alpha)

    # Shoulder Angle (solved using the virtual link geometry)
    th1 = math.atan2(z_rel, r) - math.atan2(
        L_virt * math.sin(th2_virt), L2 + L_virt * math.cos(th2_virt)
    )

    # th3 = current_ang[3]

    th0, th1, th2, th3, _, enforced = jointLimits(th0, th1, th2, known_th3, 0.0, False)
    print(enforced)

    if enforced:
        target_point = forward_kin(current_ang, L1, L2, L3, L4)

    return th0, th1, th2, target_point


def forward_kin(
    current_angles: list[float], L1: float, L2: float, L3: float, L4: float
) -> list[float]:
    """
    Calculates the [x, y, z] of the END of the wrist (end-effector).
    """
    th0 = current_angles[0]  # Base angle
    th1 = current_angles[1]  # Shoulder angle
    th2 = current_angles[2]  # Elbow angle
    th3 = current_angles[3]  # Wrist bend angle

    # Planar kinematics summing up all three planar links
    r = (
        (L2 * math.cos(th1))
        + (L3 * math.cos(th1 + th2))
        + (L4 * math.cos(th1 + th2 + th3))
    )

    z = (
        L1
        + (L2 * math.sin(th1))
        + (L3 * math.sin(th1 + th2))
        + (L4 * math.sin(th1 + th2 + th3))
    )

    # Project into 3D using the base rotation
    x = r * math.cos(th0)
    y = r * math.sin(th0)

    return [x, y, z]


def jointLimits(th0, th1, th2, th3, th4, fixElbow=False):
    """Enforce angular limits on joints, to prevent damage to the joints

    Args:
        th0 (float): Base Angle [rad]
        th1 (float): Shoulder Angle [rad]
        th2 (float): Elbow Angle [rad]
        th3 (float): Wrist Angle 1 [rad]
        th4 (float): Wrist Angle 2 [rad]
        fixElbow (bool): Correct the elbow if the shoulder is adjusted

    Returns:
        Same angles, but corrected
    """

    enforced = False
    orig_hash = th0 + th1 + th2 + th3

    # === Base Limits ===
    th0Lims = [-math.pi * 3 / 4, -math.pi / 4]  # Must stay between [-45, 225]
    if th0 < 0:
        if th0 > -math.pi / 2:
            if th0 < th0Lims[1]:
                th0 = th0Lims[1]
        else:
            if th0 > th0Lims[0]:
                th0 = th0Lims[0]

    # === Shoulder Limits ===
    # (You need to then correct the elbow angle, this is def NOT the best way to correct th2)
    rangeLow = 1 / 16
    th1Lims = [
        -math.pi * (1 - rangeLow),
        -math.pi * rangeLow,
    ]  # Must stay between [-45, 225] *** INCORRECT ANGLES
    if th1 < 0:
        if th1 > -math.pi / 2:
            diff = abs(angle_diff(th1, th1Lims[1]))
            if th1 < th1Lims[1]:
                th1 = th1Lims[1]
                if fixElbow:
                    th2 -= diff
        else:
            diff = abs(angle_diff(th1, th1Lims[0]))
            if th1 > th1Lims[0]:
                th1 = th1Lims[0]
                if fixElbow:
                    th2 += diff

    # === Elbow Limits ===
    th2 = simple_clamp(th2, [-math.pi * 3 / 4, math.pi * 3 / 4])

    # === Wrist Limits ===
    th3 = simple_clamp(th3, [-math.pi * 3 / 4, math.pi * 3 / 4])

    # === Wrist Limits ===
    th4 = simple_clamp(th4, [-math.pi * 7 / 8, math.pi * 7 / 8])

    enforced = abs((th0 + th1 + th2 + th3) - orig_hash) >= 0.0001

    return th0, th1, th2, th3, th4, enforced


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


if __name__ == "__main__":
    A0 = 6.5 / 39.0  # Elevation of shoulder joint from base [in]
    A1 = 18.5 / 39.0  # Upper arm length [in]
    A2 = 19.75 / 39.0  # Fore-arm length [in]
    A3 = 11.5 / 39.0
    """
    current_ang = [0.0, 0.0, 0.0]
    point = [0.3, 0.0, 1.2]
    target_th3 = 0.5

    target_th0, target_th1, target_th2 = fast_IK_solve(
        current_ang, None, point, target_th3, A0, A1, A2, A3
    )
    print(target_th0 * 57.0, target_th1 * 57.0, target_th2 * 57.0)

    x, y, z = forward_kin(
        [target_th0, target_th1, target_th2, target_th3], A0, A1, A2, A3
    )
    print(x, y, z)"""

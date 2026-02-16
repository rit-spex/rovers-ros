import sympy as sp
import numpy as np


def wrap_to_pi(angle):
    """Constrain angles to the range [-pi, pi]

    Args:
        angle (float): Angle in radians

    Returns:
        float: Wrapped angle
    """
    return (angle + np.pi) % (2*np.pi) - np.pi


def angle_diff(a, b):
    """
    Compute the signed difference between two angles, wrapped to [-pi, pi].
    Both a and b in radians.
    """
    diff = a - b
    # Wrap to [-pi, pi]
    diff = (diff + np.pi) % (2 * np.pi) - np.pi
    return diff


def get_flipped_angles(th1, th2, l0, l1, l2):
    """
    Computes the flipped configuration of a 2-joint arm while keeping
    the end effector fixed.

    Parameters:
    - th1: float, current shoulder angle (radians)
    - th2: float, current elbow angle (radians)
    - l0: array-like, shoulder position [x, y] or [x, y, z]
    - l1: array-like, elbow position
    - l2: array-like, wrist position

    Returns:
    - new_th1: float, new shoulder angle after flip
    - new_th2: float, new elbow angle after flip
    """

    # Step 1: Create vectors from shoulder to elbow and shoulder to wrist
    v_arm = np.array(l1) - np.array(l0)   # Vector along upper arm
    v_axis = np.array(l2) - np.array(l0)  # Vector along reflection axis (to wrist)

    # Step 2: Normalize vectors to get unit directions
    unit_arm = v_arm / np.linalg.norm(v_arm)
    unit_axis = v_axis / np.linalg.norm(v_axis)

    # Step 3: Compute angle between upper arm and axis of reflection
    # Dot product gives cos(alpha), clip to [-1,1] for numerical safety
    dot_product = np.dot(unit_arm, unit_axis)
    alpha = np.arccos(np.clip(dot_product, -1.0, 1.0))

    # Step 4: Determine how much to shift the shoulder angle
    # Shift is 2*alpha; the sign depends on current elbow bending
    shift = 2 * alpha
    if th2 < 0:
        shift = -shift  # Flip direction if elbow is negative

    # Step 5: Apply shift and wrap to [-pi, pi]
    new_th1 = wrap_to_pi(th1 + shift)

    # Step 6: Flip elbow angle
    new_th2 = -th2

    return new_th1, new_th2


def calc_angle_between_points(A, B, C):
    """
    Calculates the angle ABC in radians at vertex B.

    Args:
        A (list or np.array): Starting point
        B (list or np.array): Vertex point
        C (list or np.array): Ending point

    Returns:
        float: Angle in radians
    """
    a = np.array(A)
    b = np.array(B)
    c = np.array(C)

    # Vectors BA and BC
    ba = a - b
    bc = c - b

    # Dot product and magnitudes
    dot_product = np.dot(ba, bc)
    norm_ba = np.linalg.norm(ba)
    norm_bc = np.linalg.norm(bc)

    # Calculate angle in radians
    # np.clip handles floating point precision issues
    cosine_angle = dot_product / (norm_ba * norm_bc)
    angle_rad = np.arccos(np.clip(cosine_angle, -1.0, 1.0))

    return angle_rad


def Tz(a, theta):
    """Perform a revolution about the existing z axis then move up it

    Args:
        a (float): Joint Length
        theta (float or sym): Angle of rotation

    Returns:
        sp.Matrix: Equivalent transformation matrix
    """
    return sp.Matrix([
        [sp.cos(theta), -sp.sin(theta), 0, a*sp.cos(theta)],
        [sp.sin(theta),  sp.cos(theta), 0, a*sp.sin(theta)],
        [0, 0, 1, 0],
        [0, 0, 0, 1]
    ])



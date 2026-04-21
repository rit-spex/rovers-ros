import math

def fast_IK_solve(current_ang, logger, target_point: list[float], known_th3: float, L1: float, L2: float, L3: float, L4: float):
        """
        Calculates IK for the end-effector by combining L3 and L4 into a virtual link.
        """
        x, y, z = target_point[0], target_point[1], target_point[2]
        
        # 1. Base Angle
        th0 = math.atan2(y, x)
        
        # 2. Planar Geometry
        r = math.sqrt(x**2 + y**2)
        r = max(r, 0.1)
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
            logger.warning("Target point out of physical reach! Halting movement.")
            return current_ang
            
        # 4. Solve the Virtual Triangle
        # Virtual Elbow Angle (angle between L2 and L_virt)
        cos_th2_virt = (D_sq - L2**2 - L_virt**2) / (2 * L2 * L_virt)
        cos_th2_virt = max(min(cos_th2_virt, 1.0), -1.0) 
        
        th2_virt = -math.acos(cos_th2_virt)
        print(th2_virt)
        
        # Actual Elbow Angle (remove the virtual offset)
        th2 = th2_virt - alpha
        print(alpha)
        
        # Shoulder Angle (solved using the virtual link geometry)
        th1 = math.atan2(z_rel, r) - math.atan2(L_virt * math.sin(th2_virt), L2 + L_virt * math.cos(th2_virt))
        
        return th0, th1, -th2


def forward_kin(current_angles: list[float], L1: float, L2: float, L3: float, L4: float) -> list[float]:
    """
    Calculates the [x, y, z] of the END of the wrist (end-effector).
    """
    th0 = current_angles[0]  # Base angle
    th1 = current_angles[1]  # Shoulder angle
    th2 = -current_angles[2]  # Elbow angle
    th3 = current_angles[3]  # Wrist bend angle

    # Planar kinematics summing up all three planar links
    r = (L2 * math.cos(th1)) + \
        (L3 * math.cos(th1 + th2)) + \
        (L4 * math.cos(th1 + th2 + th3))
        
    z = L1 + (L2 * math.sin(th1)) + \
             (L3 * math.sin(th1 + th2)) + \
             (L4 * math.sin(th1 + th2 + th3))

    # Project into 3D using the base rotation
    x = r * math.cos(th0)
    y = r * math.sin(th0)

    return [x, y, z]


if __name__ == '__main__':
    A0 = 6.5 / 39.0        # Elevation of shoulder joint from base [in]
    A1 = 18.5 / 39.0         # Upper arm length [in]
    A2 = 19.75 / 39.0         # Fore-arm length [in]
    A3 = 11.5 / 39.0  

    current_ang = [0.0, 0.0, 0.0]
    point = [0.3, 0.0, 1.2]
    target_th3 = 0.5

    target_th0, target_th1, target_th2 = fast_IK_solve(current_ang, None, point, target_th3, A0, A1, A2, A3)
    print(target_th0*57.0, target_th1*57.0, target_th2*57.0)

    x, y, z = forward_kin([target_th0, target_th1, target_th2, target_th3], A0, A1, A2, A3)
    print(x, y, z)